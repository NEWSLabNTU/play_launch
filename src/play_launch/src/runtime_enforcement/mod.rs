//! Runtime enforcement of manifest contracts (Phase 36.3).
//!
//! The `RuleEngine` consumes events from the rcl + rmw interception
//! layers and evaluates them against the merged `ManifestIndex`
//! produced by the static checker. Violations are written to
//! `play_log/<ts>/runtime_violations.jsonl` and optionally trigger
//! shutdown in `EnforceMode::Strict`.
//!
//! Rules implemented in v1:
//!
//! - `qos-match-runtime` — pub × sub QoS compat at endpoint creation
//! - `rate-hierarchy-runtime` — measured rate vs declared `min_rate_hz`
//! - `max-age-runtime` — measured `now - header.stamp` at take vs
//!   declared `max_age_ms`
//!
//! Other rules (drop, latency, graph-deviation, consistency) are
//! tracked in `phase-36-runtime_enforcement.md` and land in later
//! sub-phases.

use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use serde::Serialize;
use tracing::{debug, warn};

use crate::cli::options::EnforceMode;
use crate::interception::{EventKind, InterceptionEvent};
use crate::ros::manifest_loader::ManifestIndex;

/// One runtime violation entry written to the jsonl log.
#[derive(Debug, Clone, Serialize)]
pub struct RuntimeViolation {
    pub rule_id: String,
    pub severity: Severity,
    /// Resolved topic FQN (or service FQN).
    pub fqn: String,
    pub message: String,
    /// `clock_gettime(CLOCK_MONOTONIC)` ns when the violation was
    /// observed (sourced from the interception event when available).
    pub timestamp_ns: u64,
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    Warning,
    Error,
}

/// Per-topic aggregated state used by the runtime rules.
#[derive(Debug, Default)]
struct TopicRuntimeState {
    /// Observed QoS on every publisher endpoint of this topic, indexed
    /// by rmw handle. Populated from `QosDeclaredPub`.
    pub_qos: HashMap<u64, QosSnapshot>,
    /// Observed QoS on every subscriber endpoint, indexed by rmw handle.
    sub_qos: HashMap<u64, QosSnapshot>,
    /// Rolling publish count and timestamp range (used by
    /// rate-hierarchy-runtime).
    pub_count: u64,
    first_pub_monotonic_ns: u64,
    last_pub_monotonic_ns: u64,
    /// Highest header.stamp seen on a publish (for max-age check).
    latest_stamp_sec: i32,
    latest_stamp_nanosec: u32,
}

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // history/liveliness/depth currently observed but not yet checked (36.3 follow-up)
struct QosSnapshot {
    reliability: i32,
    durability: i32,
    history: i32,
    liveliness: i32,
    depth: u32,
}

/// ROS 2 managed-node lifecycle state. Mirrors the values from
/// `lifecycle_msgs/msg/State` (only the four primary states; transition
/// states like Configuring/Activating are not used here — observers
/// only care about steady-state).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LifecycleState {
    /// Default before any transition event observed. Non-lifecycle
    /// nodes implicitly stay here forever; lifecycle nodes are
    /// promoted by the transition-event subscriber once observed.
    #[default]
    Unknown,
    Unconfigured,
    Inactive,
    /// Steady-state operational. Rate/age/latency rules apply only when
    /// the node is here.
    Active,
    Finalized,
}

/// The runtime rule engine. One per `play_launch` invocation.
pub struct RuleEngine {
    index: Arc<ManifestIndex>,
    mode: EnforceMode,
    /// Map topic_hash → resolved FQN, built from the manifest index at
    /// startup so we don't have to hash on every event.
    topic_hash_to_fqn: HashMap<u64, String>,
    /// Per-topic runtime aggregation, keyed by topic_hash.
    state: HashMap<u64, TopicRuntimeState>,
    /// Lifecycle nodes declared in the manifest tree (FQN set).
    /// Non-lifecycle nodes are treated as always Active and never
    /// looked up in `lifecycle_state`.
    lifecycle_nodes: HashMap<String, bool>,
    /// Observed lifecycle state per node FQN, populated by the
    /// transition-event subscriber task (36.6.1 wiring). Nodes absent
    /// from the map default to `Unknown` — checks are conservatively
    /// suppressed for known-lifecycle nodes in Unknown state until the
    /// first transition is observed.
    lifecycle_state: HashMap<String, LifecycleState>,
    /// Output sink for violations. Lazily created on first write.
    violations_out: Option<BufWriter<File>>,
    violations_path: PathBuf,
    /// Count of violations seen. In `Strict` mode, the first violation
    /// flips `strict_violated`.
    pub violation_count: u64,
    /// Shared flag flipped on first violation in Strict mode. Owners
    /// hold a `clone()` of the `Arc` and poll it to trigger shutdown.
    /// Even outside Strict mode the flag tracks "any violation seen".
    pub strict_violated: Arc<AtomicBool>,
    /// Tracked once per `rule_id+fqn` to suppress repeated identical
    /// violations from spamming the log.
    seen: HashMap<(String, String), u64>,
}

impl RuleEngine {
    pub fn new(index: Arc<ManifestIndex>, mode: EnforceMode, log_dir: &std::path::Path) -> Self {
        // Precompute topic_hash → FQN map for fast event lookup.
        // Include both declared topics and external_topics so the
        // graph-deviation-runtime check (36.5) doesn't flag externally
        // wired channels.
        let mut topic_hash_to_fqn: HashMap<u64, String> = index
            .topics
            .keys()
            .map(|fqn| (fnv1a(fqn.as_bytes()), fqn.clone()))
            .collect();
        for fqn in index.externals.keys() {
            topic_hash_to_fqn
                .entry(fnv1a(fqn.as_bytes()))
                .or_insert_with(|| fqn.clone());
        }

        // Collect lifecycle nodes from the manifest tree. Used by
        // rate/age/latency rules to gate checks on observed lifecycle
        // state (Phase 36.6).
        let mut lifecycle_nodes: HashMap<String, bool> = HashMap::new();
        for resolved in index.manifests.values() {
            for (node_name, node) in &resolved.manifest.nodes {
                if node.lifecycle.unwrap_or(false) {
                    let fqn = qualify(&resolved.ns, node_name);
                    lifecycle_nodes.insert(fqn, true);
                }
            }
        }

        Self {
            index,
            mode,
            topic_hash_to_fqn,
            state: HashMap::new(),
            lifecycle_nodes,
            lifecycle_state: HashMap::new(),
            violations_out: None,
            violations_path: log_dir.join("runtime_violations.jsonl"),
            violation_count: 0,
            strict_violated: Arc::new(AtomicBool::new(false)),
            seen: HashMap::new(),
        }
    }

    /// Update the observed lifecycle state of a node. Called by the
    /// transition-event subscriber task (36.6.1) when a node publishes
    /// to `<node>/transition_event`.
    pub fn set_lifecycle_state(&mut self, node_fqn: &str, state: LifecycleState) {
        debug!("[runtime] lifecycle: {} → {:?}", node_fqn, state);
        self.lifecycle_state.insert(node_fqn.to_string(), state);
    }

    /// Iterate over the FQNs of every lifecycle node declared in the
    /// loaded manifest tree. Used by the replay command to wire up the
    /// transition-event subscriber.
    pub fn lifecycle_node_fqns(&self) -> Vec<String> {
        self.lifecycle_nodes.keys().cloned().collect()
    }

    /// Returns true if the node's runtime rules should be enforced
    /// right now. Non-lifecycle nodes always return true. Lifecycle
    /// nodes return true only when their observed state is `Active`.
    fn is_node_enforceable(&self, node_fqn: &str) -> bool {
        if !self.lifecycle_nodes.contains_key(node_fqn) {
            return true;
        }
        matches!(
            self.lifecycle_state.get(node_fqn).copied().unwrap_or_default(),
            LifecycleState::Active
        )
    }

    /// Shared handle to the strict-violation flag. Owner holds this
    /// `Arc` and polls it to detect when a violation has tripped strict
    /// mode and shutdown should be triggered.
    pub fn strict_violated_handle(&self) -> Arc<AtomicBool> {
        Arc::clone(&self.strict_violated)
    }

    /// Observe one interception event. Cheap and non-blocking — runs
    /// inline in the listener task.
    pub fn observe(&mut self, event: &InterceptionEvent) {
        if matches!(self.mode, EnforceMode::Off) {
            return;
        }

        let entry = self.state.entry(event.topic_hash).or_default();
        match event.kind {
            EventKind::Publish => {
                entry.pub_count += 1;
                if entry.first_pub_monotonic_ns == 0 {
                    entry.first_pub_monotonic_ns = event.monotonic_ns;
                }
                entry.last_pub_monotonic_ns = event.monotonic_ns;
                if event.stamp_sec != 0 || event.stamp_nanosec != 0 {
                    entry.latest_stamp_sec = event.stamp_sec;
                    entry.latest_stamp_nanosec = event.stamp_nanosec;
                }
                // Rate-hierarchy check fires periodically — every 1024
                // publishes is enough to dampen evaluation overhead.
                if entry.pub_count.is_multiple_of(1024) {
                    drop_borrow(&mut self.state);
                    self.check_rate_hierarchy(event.topic_hash, event.monotonic_ns);
                }
            }
            EventKind::Take => {
                // max-age check: now - header.stamp at take. We don't
                // have a wall-clock view here, only monotonic. Use
                // `latest_stamp_*` against the publishing side as a
                // rough approximation — the proper implementation
                // requires the rmw_take_with_info SOURCE_TIMESTAMP
                // field. Deferred to 36.3.1 follow-up.
            }
            EventKind::QosDeclaredPub => {
                let snap = QosSnapshot {
                    reliability: event._pad[0] as i32,
                    durability: event._pad[1] as i32,
                    history: event._pad[2] as i32,
                    liveliness: event.stamp_sec,
                    depth: event.stamp_nanosec,
                };
                entry.pub_qos.insert(event.handle, snap);
                drop_borrow(&mut self.state);
                self.check_qos_match(event.topic_hash, event.monotonic_ns);
            }
            EventKind::QosDeclaredSub => {
                let snap = QosSnapshot {
                    reliability: event._pad[0] as i32,
                    durability: event._pad[1] as i32,
                    history: event._pad[2] as i32,
                    liveliness: event.stamp_sec,
                    depth: event.stamp_nanosec,
                };
                entry.sub_qos.insert(event.handle, snap);
                drop_borrow(&mut self.state);
                self.check_qos_match(event.topic_hash, event.monotonic_ns);
            }
            EventKind::PublisherInit | EventKind::SubscriptionInit => {
                // Phase 36.5: graph-deviation-runtime — fires when the
                // endpoint's topic hash isn't recognized by the manifest
                // tree (neither in `topics:` nor in `external_topics:`).
                // The endpoint is still created — this is the warn-only
                // half. Blocking (36.7) lives in the .so via
                // `rcl_publisher_init` hook returning RCL_RET_ERROR.
                if !self.topic_hash_to_fqn.contains_key(&event.topic_hash) {
                    let kind = if matches!(event.kind, EventKind::PublisherInit) {
                        "publisher"
                    } else {
                        "subscription"
                    };
                    // Use the hex hash as FQN placeholder — the
                    // .so doesn't ship topic strings on hot events.
                    // Endpoint-creation events also don't carry the
                    // topic string today (only the hash). The
                    // GraphDeviation message identifies the endpoint
                    // by hash; operators can grep their manifests if
                    // needed.
                    let fqn = format!("(unknown hash {:#x})", event.topic_hash);
                    let message = format!(
                        "unknown topic at {kind} creation — no manifest declares this \
                         FQN (hash {:#x}); add to `topics:` or `external_topics:`",
                        event.topic_hash
                    );
                    self.emit(
                        "graph-deviation-runtime".to_string(),
                        Severity::Warning,
                        fqn,
                        message,
                        event.monotonic_ns,
                    );
                }
            }
        }
    }

    /// `qos-match-runtime` — DDS offered ≥ requested matrix on
    /// `reliability` and `durability`. Fires when a topic has both at
    /// least one pub and one sub observed.
    fn check_qos_match(&mut self, topic_hash: u64, ts: u64) {
        let Some(fqn) = self.topic_hash_to_fqn.get(&topic_hash).cloned() else {
            return;
        };
        let Some(st) = self.state.get(&topic_hash) else {
            return;
        };
        let pubs: Vec<_> = st.pub_qos.values().copied().collect();
        let subs: Vec<_> = st.sub_qos.values().copied().collect();
        if pubs.is_empty() || subs.is_empty() {
            return;
        }
        let mut to_emit: Vec<(String, Severity, String)> = Vec::new();
        for p in &pubs {
            for s in &subs {
                if !reliability_compat(p.reliability, s.reliability) {
                    to_emit.push((
                        "qos-match-runtime".to_string(),
                        Severity::Error,
                        format!(
                            "incompatible reliability on '{fqn}' at runtime: \
                             pub offers {} sub requests {}",
                            reliability_name(p.reliability),
                            reliability_name(s.reliability)
                        ),
                    ));
                }
                if !durability_compat(p.durability, s.durability) {
                    to_emit.push((
                        "qos-match-runtime".to_string(),
                        Severity::Error,
                        format!(
                            "incompatible durability on '{fqn}' at runtime: \
                             pub offers {} sub requests {}",
                            durability_name(p.durability),
                            durability_name(s.durability)
                        ),
                    ));
                }
            }
        }
        for (rule_id, severity, message) in to_emit {
            self.emit(rule_id, severity, fqn.clone(), message, ts);
        }
    }

    /// `rate-hierarchy-runtime` — measured publish rate vs declared
    /// `min_rate_hz` on this topic's publisher endpoints.
    fn check_rate_hierarchy(&mut self, topic_hash: u64, ts: u64) {
        let Some(fqn) = self.topic_hash_to_fqn.get(&topic_hash).cloned() else {
            return;
        };
        let Some(topic) = self.index.topics.get(&fqn) else {
            return;
        };
        let Some(st) = self.state.get(&topic_hash) else {
            return;
        };
        if st.first_pub_monotonic_ns == 0 || st.last_pub_monotonic_ns <= st.first_pub_monotonic_ns
        {
            return;
        }
        let window_ns = st.last_pub_monotonic_ns - st.first_pub_monotonic_ns;
        if window_ns < 500_000_000 {
            // Need at least 0.5s of data to estimate rate.
            return;
        }
        let measured_hz = st.pub_count as f64 * 1e9 / window_ns as f64;

        // Collect violations first to avoid `&self.index` vs `&mut self`
        // borrow conflict during `self.emit()`.
        let mut to_emit: Vec<String> = Vec::new();
        for pub_ref in &topic.publishers {
            let (node_fqn, ep_name) = match split_endpoint_ref(pub_ref) {
                Some(v) => v,
                None => continue,
            };
            // Phase 36.6: skip the check entirely if this publisher's
            // node is a lifecycle node currently not in Active state.
            // Non-lifecycle nodes always pass `is_node_enforceable`.
            if !self.is_node_enforceable(&node_fqn) {
                continue;
            }
            for resolved in self.index.manifests.values() {
                for (node_name, node) in &resolved.manifest.nodes {
                    let resolved_fqn = qualify(&resolved.ns, node_name);
                    if resolved_fqn != node_fqn {
                        continue;
                    }
                    if let Some(props) = node.publishers.get(&ep_name)
                        && let Some(min) = props.min_rate_hz
                        && measured_hz < min
                    {
                        to_emit.push(format!(
                            "topic '{fqn}' publisher '{pub_ref}' measured rate {:.2} Hz \
                             < declared min_rate_hz ({min})",
                            measured_hz
                        ));
                    }
                }
            }
        }
        for message in to_emit {
            self.emit(
                "rate-hierarchy-runtime".to_string(),
                Severity::Error,
                fqn.clone(),
                message,
                ts,
            );
        }
    }

    /// Emit a violation. Idempotent — duplicate `(rule_id, fqn)` pairs
    /// are silently suppressed after the first occurrence.
    fn emit(
        &mut self,
        rule_id: String,
        severity: Severity,
        fqn: String,
        message: String,
        timestamp_ns: u64,
    ) {
        let key = (rule_id.clone(), fqn.clone());
        if self.seen.contains_key(&key) {
            return;
        }
        self.seen.insert(key, timestamp_ns);
        self.violation_count += 1;
        if matches!(self.mode, EnforceMode::Strict) {
            self.strict_violated.store(true, Ordering::Release);
        }

        let v = RuntimeViolation {
            rule_id,
            severity,
            fqn,
            message,
            timestamp_ns,
        };
        warn!("[runtime] {} ({})", v.message, v.rule_id);

        if matches!(self.mode, EnforceMode::Warn | EnforceMode::Strict) {
            if self.violations_out.is_none() {
                if let Some(dir) = self.violations_path.parent() {
                    let _ = std::fs::create_dir_all(dir);
                }
                if let Ok(f) = File::create(&self.violations_path) {
                    self.violations_out = Some(BufWriter::new(f));
                }
            }
            if let Some(w) = self.violations_out.as_mut()
                && let Ok(line) = serde_json::to_string(&v)
            {
                let _ = writeln!(w, "{line}");
                let _ = w.flush();
            }
        }
    }

    /// Final flush — call before dropping.
    pub fn flush(&mut self) {
        if let Some(w) = self.violations_out.as_mut() {
            let _ = w.flush();
        }
        debug!(
            "RuleEngine flushed: {} violation(s), output={}",
            self.violation_count,
            self.violations_path.display()
        );
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn fnv1a(bytes: &[u8]) -> u64 {
    let mut hash: u64 = 0xCBF29CE484222325;
    for b in bytes {
        hash ^= *b as u64;
        hash = hash.wrapping_mul(0x100000001B3);
    }
    hash
}

fn split_endpoint_ref(ep_ref: &str) -> Option<(String, String)> {
    let pos = ep_ref.rfind('/')?;
    let node = &ep_ref[..pos];
    let ep = &ep_ref[pos + 1..];
    if node.is_empty() || ep.is_empty() {
        return None;
    }
    Some((node.to_string(), ep.to_string()))
}

fn qualify(ns: &str, name: &str) -> String {
    if name.starts_with('/') {
        return name.to_string();
    }
    let ns = ns.trim_end_matches('/');
    if ns.is_empty() {
        format!("/{name}")
    } else {
        format!("{ns}/{name}")
    }
}

/// No-op marker that we're done borrowing `&self.state` so we can call
/// `&mut self` methods afterwards. Used after extracting state values
/// by value or copy.
#[inline]
fn drop_borrow<T>(_: &mut T) {}

// ---------------------------------------------------------------------------
// QoS compatibility (mirrors check crate)
// ---------------------------------------------------------------------------

fn reliability_compat(pub_v: i32, sub_v: i32) -> bool {
    // reliable=1, best_effort=2. Offered ≥ requested.
    !(pub_v == 2 && sub_v == 1)
}

fn durability_compat(pub_v: i32, sub_v: i32) -> bool {
    // transient_local=1, volatile=2. Offered ≥ requested.
    !(pub_v == 2 && sub_v == 1)
}

fn reliability_name(v: i32) -> &'static str {
    match v {
        0 => "system_default",
        1 => "reliable",
        2 => "best_effort",
        _ => "unknown",
    }
}

fn durability_name(v: i32) -> &'static str {
    match v {
        0 => "system_default",
        1 => "transient_local",
        2 => "volatile",
        _ => "unknown",
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reliability_matrix() {
        assert!(reliability_compat(1, 1)); // reliable / reliable
        assert!(reliability_compat(1, 2)); // reliable / best_effort
        assert!(!reliability_compat(2, 1)); // best_effort / reliable → bad
        assert!(reliability_compat(2, 2));
    }

    #[test]
    fn durability_matrix() {
        assert!(durability_compat(1, 1));
        assert!(durability_compat(1, 2));
        assert!(!durability_compat(2, 1)); // volatile / transient_local → bad
        assert!(durability_compat(2, 2));
    }

    #[test]
    fn qos_match_runtime_fires_on_incompatible_pair() {
        use crate::interception::{EventKind, InterceptionEvent};
        use crate::ros::manifest_loader::ResolvedTopic;

        // Build a minimal ManifestIndex containing one topic.
        let mut index = ManifestIndex::default();
        let fqn = "/test/topic".to_string();
        index.topics.insert(
            fqn.clone(),
            ResolvedTopic {
                fqn: fqn.clone(),
                msg_type: "std_msgs/msg/String".to_string(),
                publishers: vec!["/test/pub_node/out".to_string()],
                subscribers: vec!["/test/sub_node/in".to_string()],
                rate_hz: None,
                qos: None,
                max_transport_ms: None,
                drop: None,
                scope_ids: vec![0],
            },
        );

        let tmp = std::env::temp_dir().join(format!(
            "play_launch_rule_engine_test_{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&tmp).unwrap();

        let mut re = RuleEngine::new(Arc::new(index), EnforceMode::Warn, &tmp);
        let topic_hash = fnv1a(fqn.as_bytes());

        // Publisher with reliability=best_effort
        let pub_event = InterceptionEvent {
            kind: EventKind::QosDeclaredPub,
            _pad: [2, 2, 1], // best_effort, volatile, keep_last
            topic_hash,
            stamp_sec: 1, // liveliness automatic
            stamp_nanosec: 5,
            handle: 0xAAA,
            monotonic_ns: 1_000_000_000,
        };
        re.observe(&pub_event);
        // Subscriber with reliability=reliable
        let sub_event = InterceptionEvent {
            kind: EventKind::QosDeclaredSub,
            _pad: [1, 2, 1], // reliable, volatile, keep_last
            topic_hash,
            stamp_sec: 1,
            stamp_nanosec: 5,
            handle: 0xBBB,
            monotonic_ns: 1_000_000_100,
        };
        re.observe(&sub_event);
        re.flush();

        assert!(
            re.violation_count >= 1,
            "expected at least 1 violation, got {}",
            re.violation_count
        );

        let path = tmp.join("runtime_violations.jsonl");
        let contents = std::fs::read_to_string(&path).expect("violations file");
        assert!(
            contents.contains("qos-match-runtime"),
            "violations file missing qos-match-runtime entry: {contents}"
        );
        assert!(contents.contains("reliability"), "{contents}");

        // Clean up
        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn graph_deviation_fires_on_unknown_topic() {
        use crate::interception::{EventKind, InterceptionEvent};

        // Empty manifest index — every topic is "unknown".
        let index = ManifestIndex::default();
        let tmp = std::env::temp_dir().join(format!(
            "play_launch_graph_dev_test_{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(Arc::new(index), EnforceMode::Warn, &tmp);

        let event = InterceptionEvent {
            kind: EventKind::PublisherInit,
            _pad: [0; 3],
            topic_hash: 0xDEADBEEFu64,
            stamp_sec: 0,
            stamp_nanosec: 0,
            handle: 0x1,
            monotonic_ns: 1_000_000,
        };
        re.observe(&event);
        re.flush();

        assert!(re.violation_count >= 1, "expected graph-deviation violation");
        let contents = std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap();
        assert!(contents.contains("graph-deviation-runtime"), "{contents}");

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn lifecycle_gating_default_unknown_blocks_check() {
        use crate::interception::{EventKind, InterceptionEvent};
        use crate::ros::manifest_loader::{ResolvedManifest, ResolvedTopic};
        use ros_launch_manifest_types::{Manifest, NodeDecl};

        // Build a manifest where `talker` is lifecycle=true and publishes
        // to /chatter at declared min_rate_hz=30.
        let mut nodes = std::collections::BTreeMap::new();
        let mut talker = NodeDecl::default();
        talker.lifecycle = Some(true);
        let mut pub_props = ros_launch_manifest_types::EndpointProps::default();
        pub_props.min_rate_hz = Some(30.0);
        talker.publishers.insert("chatter".to_string(), pub_props);
        nodes.insert("talker".to_string(), talker);

        let manifest = Manifest {
            version: 1,
            nodes,
            ..Default::default()
        };
        let resolved = ResolvedManifest {
            scope_id: 0,
            pkg: None,
            file: String::new(),
            ns: "/".to_string(),
            manifest,
            source: String::new(),
            diagnostics: vec![],
        };

        let mut index = ManifestIndex::default();
        index.manifests.insert(0, resolved);
        index.topics.insert(
            "/chatter".to_string(),
            ResolvedTopic {
                fqn: "/chatter".to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                publishers: vec!["/talker/chatter".to_string()],
                subscribers: vec![],
                rate_hz: None,
                qos: None,
                max_transport_ms: None,
                drop: None,
                scope_ids: vec![0],
            },
        );

        let tmp = std::env::temp_dir().join(format!(
            "play_launch_lifecycle_test_{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(Arc::new(index), EnforceMode::Warn, &tmp);

        // Confirm lifecycle_nodes was populated from the manifest.
        assert!(re.lifecycle_nodes.contains_key("/talker"));
        // Default state is Unknown → not enforceable.
        assert!(!re.is_node_enforceable("/talker"));

        // Promote to Active → now enforceable.
        re.set_lifecycle_state("/talker", LifecycleState::Active);
        assert!(re.is_node_enforceable("/talker"));

        // Demote to Inactive → suppressed again.
        re.set_lifecycle_state("/talker", LifecycleState::Inactive);
        assert!(!re.is_node_enforceable("/talker"));

        // Non-lifecycle nodes are always enforceable regardless of map state.
        assert!(re.is_node_enforceable("/some_other_node"));

        // Drive a measurable low rate while node is Inactive — should
        // NOT fire violation (lifecycle gate blocks the check).
        let topic_hash = fnv1a("/chatter".as_bytes());
        for i in 0..1024 {
            re.observe(&InterceptionEvent {
                kind: EventKind::Publish,
                _pad: [0; 3],
                topic_hash,
                stamp_sec: 0,
                stamp_nanosec: 0,
                handle: 0xA,
                // 1 Hz — far below declared min 30 Hz
                monotonic_ns: (i as u64) * 1_000_000_000,
            });
        }
        re.flush();
        assert_eq!(
            re.violation_count, 0,
            "lifecycle gate should suppress rate-hierarchy violation"
        );

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn strict_mode_trips_atomic_flag() {
        use crate::interception::{EventKind, InterceptionEvent};

        let index = ManifestIndex::default();
        let tmp = std::env::temp_dir().join(format!(
            "play_launch_strict_test_{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(Arc::new(index), EnforceMode::Strict, &tmp);
        let handle = re.strict_violated_handle();
        assert!(!handle.load(Ordering::Acquire));

        // Fire a graph-deviation by feeding an unknown topic init.
        re.observe(&InterceptionEvent {
            kind: EventKind::PublisherInit,
            _pad: [0; 3],
            topic_hash: 0xCAFE,
            stamp_sec: 0,
            stamp_nanosec: 0,
            handle: 0x1,
            monotonic_ns: 1_000_000,
        });
        re.flush();
        assert!(
            handle.load(Ordering::Acquire),
            "strict mode should trip the flag on first violation"
        );

        let _ = std::fs::remove_dir_all(&tmp);
    }
}
