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

use std::{
    collections::HashMap,
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
};

use serde::Serialize;
use tracing::{debug, warn};

use crate::{
    cli::options::EnforceMode,
    interception::{EventKind, InterceptionEvent, TopicNameAssembly, decode_topic_name_chunk},
};
pub mod view;
pub use view::ContractView;

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
    /// Per-subscriber take count, indexed by rcl subscription handle.
    /// Used by `drop-rate-runtime` to compute per-sub delivery ratio.
    sub_take_count: HashMap<u64, u64>,
    /// Observed runtime msg-type identifier per endpoint
    /// (FNV1A of "pkg/msg/Name" — derived from rosidl introspection
    /// when init events ship `type_hash` in their overloaded slots).
    /// Used by `consistency-runtime`.
    endpoint_type_hash: HashMap<u64, u64>,
    /// Latest publish monotonic_ns for the most recent header.stamp
    /// observed. Used by `max-latency-runtime` to time the stamp from
    /// pipeline input to output.
    last_pub_for_stamp: HashMap<u64, u64>,
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
    view: Arc<ContractView>,
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
    /// Buffer for in-flight `TopicNameDeclared` chunks keyed by
    /// topic_hash. Each entry holds the expected `total_chunks` and the
    /// chunks received so far. On completion the assembled FQN is
    /// inserted into `discovered_names` for use by violation messages.
    topic_name_chunks: HashMap<u64, TopicNameAssembly>,
    /// Runtime-discovered topic FQNs (one entry per unique topic_hash
    /// announced by the .so). Used only to render nicer messages —
    /// kept SEPARATE from `topic_hash_to_fqn` so the membership check
    /// for graph-deviation-runtime continues to reflect manifest
    /// declarations only.
    discovered_names: HashMap<u64, String>,
    /// Per-topic_hash reassembly buffer for `TypeNameDeclared` chunks.
    /// Distinct from `topic_name_chunks` because both event kinds key
    /// on `topic_hash` and a single topic emits both a name and a type.
    type_name_chunks: HashMap<u64, TopicNameAssembly>,
    /// Runtime-discovered msg-type identity per topic_hash. Dumped to
    /// `play_log/<ts>/discovered_topic_types.tsv` on flush.
    discovered_types: HashMap<u64, String>,
}

// `TopicNameAssembly` (multi-chunk string reassembly) and
// `decode_topic_name_chunk` live in `crate::interception` — shared with
// the frontier/stats `NameCatalog` (Phase 42.0), since both decode the
// same `TopicNameDeclared`/`TypeNameDeclared` wire chunk format.

impl RuleEngine {
    pub fn new(view: Arc<ContractView>, mode: EnforceMode, log_dir: &std::path::Path) -> Self {
        // Precompute topic_hash → FQN map for fast event lookup.
        // Include both declared topics and external_topics so the
        // graph-deviation-runtime check (36.5) doesn't flag externally
        // wired channels.
        let mut topic_hash_to_fqn: HashMap<u64, String> = view
            .topics
            .keys()
            .map(|fqn| (fnv1a(fqn.as_bytes()), fqn.clone()))
            .collect();
        for fqn in &view.externals {
            topic_hash_to_fqn
                .entry(fnv1a(fqn.as_bytes()))
                .or_insert_with(|| fqn.clone());
        }

        // Lifecycle nodes (Phase 36.6): rate/age/latency rules gate on
        // observed lifecycle state.
        let lifecycle_nodes: HashMap<String, bool> = view
            .lifecycle_nodes
            .iter()
            .map(|f| (f.clone(), true))
            .collect();

        Self {
            view,
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
            topic_name_chunks: HashMap::new(),
            discovered_names: HashMap::new(),
            type_name_chunks: HashMap::new(),
            discovered_types: HashMap::new(),
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
            self.lifecycle_state
                .get(node_fqn)
                .copied()
                .unwrap_or_default(),
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
                    // Record (stamp → publish monotonic_ns) for
                    // max-latency tracking. Bounded to last ~512
                    // unique stamps to cap memory.
                    let stamp_key = pack_stamp(event.stamp_sec, event.stamp_nanosec);
                    if entry.last_pub_for_stamp.len() >= 512 {
                        // Drop oldest by clearing — simple bound; rule
                        // tolerates loss because it only matters when
                        // the downstream stamp is still recent.
                        entry.last_pub_for_stamp.clear();
                    }
                    entry
                        .last_pub_for_stamp
                        .insert(stamp_key, event.monotonic_ns);
                }
                // Rate-hierarchy check fires periodically — every 1024
                // publishes is enough to dampen evaluation overhead.
                if entry.pub_count.is_multiple_of(1024) {
                    drop_borrow(&mut self.state);
                    self.check_rate_hierarchy(event.topic_hash, event.monotonic_ns);
                }
                // max-latency-runtime: on every publish, check any
                // scope path whose output is this topic.
                drop_borrow(&mut self.state);
                self.check_max_latency(
                    event.topic_hash,
                    event.stamp_sec,
                    event.stamp_nanosec,
                    event.monotonic_ns,
                );
            }
            EventKind::Take => {
                // Per-sub take count for drop-rate-runtime.
                *entry.sub_take_count.entry(event.handle).or_insert(0) += 1;
                let take_count = *entry.sub_take_count.get(&event.handle).unwrap_or(&0);

                // max-age-runtime: compute now - header.stamp on the
                // system clock and check against the subscriber's
                // declared `max_age_ms`. Approximation: SystemTime::now()
                // is read here rather than at the in-kernel take
                // moment, so we may overstate age by the listener's
                // poll lag (default 10ms). Acceptable for warn-only
                // monitoring on budgets typically ≥100ms.
                if (event.stamp_sec != 0 || event.stamp_nanosec != 0)
                    && let Ok(now) =
                        std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)
                {
                    let stamp_ns =
                        (event.stamp_sec as i128) * 1_000_000_000 + (event.stamp_nanosec as i128);
                    let now_ns = now.as_nanos() as i128;
                    let age_ns = now_ns - stamp_ns;
                    if age_ns > 0 {
                        drop_borrow(&mut self.state);
                        self.check_max_age(
                            event.topic_hash,
                            event.handle,
                            (age_ns / 1_000_000) as f64,
                            event.monotonic_ns,
                        );
                    }
                }

                // drop-rate-runtime: per-sub check fires every 256
                // takes — bounded overhead, enough samples for a
                // stable ratio.
                if take_count.is_multiple_of(256) {
                    drop_borrow(&mut self.state);
                    self.check_drop_rate(event.topic_hash, event.handle, event.monotonic_ns);
                }
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
            EventKind::TopicNameDeclared => {
                // Reassemble multi-chunk topic-name announcement.
                let (idx, total, n, buf) = decode_topic_name_chunk(event);
                let entry = self
                    .topic_name_chunks
                    .entry(event.topic_hash)
                    .or_insert_with(|| TopicNameAssembly::new(total.max(1)));
                if entry.total != total.max(1) {
                    // Mismatched total — discard and restart. Should never
                    // happen since the .so sends a coherent sequence per
                    // topic_hash; treat defensively.
                    *entry = TopicNameAssembly::new(total.max(1));
                }
                entry.add(idx, &buf[..n]);
                if let Some(fqn) = entry.assembled() {
                    self.discovered_names.entry(event.topic_hash).or_insert(fqn);
                    self.topic_name_chunks.remove(&event.topic_hash);
                }
            }
            EventKind::TypeNameDeclared => {
                // Same chunk encoding as TopicNameDeclared but stores
                // into discovered_types for audit + contract backfill.
                let (idx, total, n, buf) = decode_topic_name_chunk(event);
                let entry = self
                    .type_name_chunks
                    .entry(event.topic_hash)
                    .or_insert_with(|| TopicNameAssembly::new(total.max(1)));
                if entry.total != total.max(1) {
                    *entry = TopicNameAssembly::new(total.max(1));
                }
                entry.add(idx, &buf[..n]);
                if let Some(type_id) = entry.assembled() {
                    self.discovered_types
                        .entry(event.topic_hash)
                        .or_insert(type_id);
                    self.type_name_chunks.remove(&event.topic_hash);
                }
            }
            EventKind::OfferedQosIncompatible | EventKind::RequestedQosIncompatible => {
                // Phase 36 DDS events: the DDS layer reported an
                // incompatible QoS match on this topic. Emit a
                // `qos-match-runtime` violation tagged with the source
                // side so operators can distinguish offered vs
                // requested incompatibility. The static check covers
                // declared QoS; this catches mismatches that only
                // surface once DDS discovery completes.
                drop_borrow(&mut self.state);
                self.emit_dds_qos_incompatible(event);
            }
            EventKind::OfferedDeadlineMissed | EventKind::RequestedDeadlineMissed => {
                drop_borrow(&mut self.state);
                self.emit_dds_deadline_missed(event);
            }
            EventKind::LivelinessLost | EventKind::LivelinessChanged => {
                drop_borrow(&mut self.state);
                self.emit_dds_liveliness(event);
            }
            EventKind::MessageLost => {
                drop_borrow(&mut self.state);
                self.emit_dds_message_lost(event);
            }
            EventKind::PublisherInit | EventKind::SubscriptionInit => {
                // Decode the runtime msg-type hash carried via the
                // overloaded stamp slots (Phase 36 consistency-runtime).
                // 0 = type unknown (introspection unavailable).
                let runtime_type_hash =
                    ((event.stamp_sec as u32 as u64) << 32) | (event.stamp_nanosec as u64);
                if runtime_type_hash != 0 {
                    entry
                        .endpoint_type_hash
                        .insert(event.handle, runtime_type_hash);
                    drop_borrow(&mut self.state);
                    self.check_consistency(event.topic_hash, runtime_type_hash, event.monotonic_ns);
                }

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
                    // Prefer the reassembled FQN from the
                    // `TopicNameDeclared` chunks if one has arrived;
                    // otherwise fall back to the bare hash so the
                    // violation still records the endpoint.
                    let fqn = self
                        .discovered_names
                        .get(&event.topic_hash)
                        .cloned()
                        .unwrap_or_else(|| format!("(unknown hash {:#x})", event.topic_hash));
                    let message = format!(
                        "unknown topic at {kind} creation — no manifest declares '{fqn}' \
                         (hash {:#x}); add to `topics:` or `external_topics:`",
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
            EventKind::RingOverflowReport => {
                // Phase 42.0: ring-overflow drop-count telemetry, not a
                // manifest-relevant event. Aggregated by the
                // frontier/stats consumer (`interception::mod.rs`),
                // not by the RuleEngine.
            }
            EventKind::FrontierPublish => {
                // Phase 42.x: FrontierPlugin's advance-only publish
                // notification. Not used here — rate-hierarchy and
                // max-latency tracking above are already driven by the
                // unconditional `Publish` event (one per message, from
                // StatsPlugin). Before this event kind existed, both
                // plugins emitted plain `Publish` for every stamped
                // message, so this match arm's absence would have meant
                // the RuleEngine silently double-counted here too.
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
        let Some(topic) = self.view.topics.get(&fqn) else {
            return;
        };
        let Some(st) = self.state.get(&topic_hash) else {
            return;
        };
        if st.first_pub_monotonic_ns == 0 || st.last_pub_monotonic_ns <= st.first_pub_monotonic_ns {
            return;
        }
        let window_ns = st.last_pub_monotonic_ns - st.first_pub_monotonic_ns;
        if window_ns < 500_000_000 {
            // Need at least 0.5s of data to estimate rate.
            return;
        }
        let measured_hz = st.pub_count as f64 * 1e9 / window_ns as f64;

        // Collect violations first to avoid a `&self.view` vs `&mut self`
        // borrow conflict during `self.emit()`.
        let mut to_emit: Vec<String> = Vec::new();
        for pub_ref in &topic.publishers {
            let (node_fqn, _ep_name) = match split_endpoint_ref(pub_ref) {
                Some(v) => v,
                None => continue,
            };
            // Phase 36.6: skip the check entirely if this publisher's
            // node is a lifecycle node currently not in Active state.
            // Non-lifecycle nodes always pass `is_node_enforceable`.
            if !self.is_node_enforceable(&node_fqn) {
                continue;
            }
            if let Some(&min) = self.view.pub_min_rate.get(pub_ref)
                && measured_hz < min
            {
                to_emit.push(format!(
                    "topic '{fqn}' publisher '{pub_ref}' measured rate {:.2} Hz \
                     < declared min_rate_hz ({min})",
                    measured_hz
                ));
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

    /// `max-age-runtime` — observed `now - header.stamp` vs declared
    /// `EndpointProps.max_age_ms` on the consuming subscriber endpoint.
    /// Lifecycle-gated: skips if the consumer node isn't currently
    /// enforceable.
    fn check_max_age(&mut self, topic_hash: u64, sub_handle: u64, age_ms: f64, ts: u64) {
        let _ = sub_handle; // handle map not yet wired to manifest sub refs
        let Some(fqn) = self.topic_hash_to_fqn.get(&topic_hash).cloned() else {
            return;
        };
        let Some(topic) = self.view.topics.get(&fqn) else {
            return;
        };
        // For each subscriber endpoint, look up its declared `max_age_ms`
        // and report if exceeded. Without per-endpoint runtime
        // identity, fire one violation per (topic, declared sub) pair.
        let mut to_emit: Vec<String> = Vec::new();
        for sub_ref in &topic.subscribers {
            let (node_fqn, _ep_name) = match split_endpoint_ref(sub_ref) {
                Some(v) => v,
                None => continue,
            };
            if !self.is_node_enforceable(&node_fqn) {
                continue;
            }
            if let Some(&max) = self.view.sub_max_age.get(sub_ref)
                && age_ms > max
            {
                to_emit.push(format!(
                    "topic '{fqn}' subscriber '{sub_ref}' observed age {:.1} ms \
                     > declared max_age_ms ({max})",
                    age_ms
                ));
            }
        }
        for message in to_emit {
            self.emit(
                "max-age-runtime".to_string(),
                Severity::Error,
                fqn.clone(),
                message,
                ts,
            );
        }
    }

    /// `drop-rate-runtime` — observed (pub - take) / pub ratio for one
    /// subscriber endpoint, vs declared `topic.drop.max_count` rate.
    fn check_drop_rate(&mut self, topic_hash: u64, sub_handle: u64, ts: u64) {
        let _ = sub_handle;
        let Some(fqn) = self.topic_hash_to_fqn.get(&topic_hash).cloned() else {
            return;
        };
        let Some(topic) = self.view.topics.get(&fqn) else {
            return;
        };
        let Some(max_rate) = topic.max_drop_rate else {
            return;
        };

        let Some(st) = self.state.get(&topic_hash) else {
            return;
        };
        if st.pub_count == 0 {
            return;
        }
        // Sum per-sub take counts and take the max ratio across subs.
        // Worst-case sub = most lossy receiver.
        let mut worst_ratio: f64 = 0.0;
        for take_count in st.sub_take_count.values() {
            if *take_count > st.pub_count {
                continue;
            }
            let lost = st.pub_count - *take_count;
            let ratio = lost as f64 / st.pub_count as f64;
            if ratio > worst_ratio {
                worst_ratio = ratio;
            }
        }
        if worst_ratio > max_rate {
            let message = format!(
                "topic '{fqn}' observed drop rate {:.3} > declared max ({:.3})",
                worst_ratio, max_rate
            );
            self.emit(
                "drop-rate-runtime".to_string(),
                Severity::Error,
                fqn,
                message,
                ts,
            );
        }
    }

    /// `max-latency-runtime` — for every scope path whose output topic
    /// is `topic_hash` and whose declared `max_latency_ms` is set,
    /// look up the most recent publish on the input topic that carried
    /// the same stamp. Latency = current_monotonic_ns − input_pub_monotonic_ns.
    fn check_max_latency(
        &mut self,
        output_topic_hash: u64,
        stamp_sec: i32,
        stamp_nanosec: u32,
        ts: u64,
    ) {
        if stamp_sec == 0 && stamp_nanosec == 0 {
            return;
        }
        let Some(output_fqn) = self.topic_hash_to_fqn.get(&output_topic_hash).cloned() else {
            return;
        };
        let stamp_key = pack_stamp(stamp_sec, stamp_nanosec);

        // Collect candidate scope paths whose output matches this topic
        // before recursing into &mut self for emit.
        let scope_paths = self.view.scope_paths.clone();
        let mut to_emit: Vec<(String, String)> = Vec::new();
        for sp in &scope_paths {
            if !sp.output_topics.iter().any(|t| t == &output_fqn) {
                continue;
            }
            let Some(max) = sp.max_latency_ms else {
                continue;
            };
            for input_fqn in &sp.input_topics {
                let input_hash = fnv1a(input_fqn.as_bytes());
                let Some(st) = self.state.get(&input_hash) else {
                    continue;
                };
                let Some(input_pub_ns) = st.last_pub_for_stamp.get(&stamp_key).copied() else {
                    continue;
                };
                if ts <= input_pub_ns {
                    continue;
                }
                let latency_ms = (ts - input_pub_ns) as f64 / 1_000_000.0;
                if latency_ms > max {
                    to_emit.push((
                        format!(
                            "scope path '{}' (input '{}' → output '{}'): observed latency \
                             {:.2} ms > declared max ({})",
                            sp.name, input_fqn, output_fqn, latency_ms, max
                        ),
                        output_fqn.clone(),
                    ));
                }
            }
        }
        for (message, fqn) in to_emit {
            self.emit(
                "max-latency-runtime".to_string(),
                Severity::Error,
                fqn,
                message,
                ts,
            );
        }
    }

    /// `consistency-runtime` — runtime msg-type hash from introspection
    /// vs declared `type:` in the manifest.
    fn check_consistency(&mut self, topic_hash: u64, runtime_type_hash: u64, ts: u64) {
        let Some(fqn) = self.topic_hash_to_fqn.get(&topic_hash).cloned() else {
            return;
        };
        let declared_type = if let Some(topic) = self.view.topics.get(&fqn) {
            Some(topic.msg_type.clone())
        } else {
            // External topics don't carry a declared type — skip to
            // avoid false positives.
            None
        };
        let Some(declared) = declared_type else {
            return;
        };
        if declared.starts_with("TODO/") {
            return;
        }
        let declared_hash = fnv1a(declared.as_bytes());
        if declared_hash == runtime_type_hash {
            return;
        }
        let message = format!(
            "topic '{fqn}' runtime msg type (hash {:#x}) disagrees with declared '{declared}' \
             (hash {:#x})",
            runtime_type_hash, declared_hash
        );
        self.emit(
            "consistency-runtime".to_string(),
            Severity::Error,
            fqn,
            message,
            ts,
        );
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

    /// Emit a DDS-event violation. Unlike [`emit`], these are NOT
    /// deduplicated by `(rule_id, fqn)` — each fire of `total_count_change`
    /// represents new occurrences inside the DDS implementation, so we
    /// want one violation per emission.
    fn emit_repeatable(
        &mut self,
        rule_id: String,
        severity: Severity,
        fqn: String,
        message: String,
        timestamp_ns: u64,
    ) {
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

    /// Resolve `topic_hash` → FQN string. Falls back to `(unknown hash 0x…)`
    /// when the manifest tree didn't declare the topic.
    fn resolve_fqn(&self, topic_hash: u64) -> String {
        self.topic_hash_to_fqn
            .get(&topic_hash)
            .cloned()
            .unwrap_or_else(|| format!("(unknown hash {:#x})", topic_hash))
    }

    /// Map `rmw_qos_policy_kind_t` bitmask to a short policy name for
    /// the `qos-match-runtime` message body.
    fn policy_kind_name(kind: i32) -> &'static str {
        match kind {
            x if x == (1 << 0) => "INVALID",
            x if x == (1 << 1) => "DURABILITY",
            x if x == (1 << 2) => "DEADLINE",
            x if x == (1 << 3) => "LIVELINESS",
            x if x == (1 << 4) => "RELIABILITY",
            x if x == (1 << 5) => "HISTORY",
            x if x == (1 << 6) => "LIFESPAN",
            x if x == (1 << 7) => "DEPTH",
            x if x == (1 << 8) => "LIVELINESS_LEASE_DURATION",
            x if x == (1 << 9) => "AVOID_ROS_NAMESPACE_CONVENTIONS",
            _ => "UNKNOWN",
        }
    }

    fn emit_dds_qos_incompatible(&mut self, event: &InterceptionEvent) {
        // Only emit when there's actually a delta — DDS may surface
        // status reads with `total_count_change == 0` (rare but
        // possible after reading the same status twice).
        if event.stamp_nanosec == 0 {
            return;
        }
        let fqn = self.resolve_fqn(event.topic_hash);
        let kind_bytes = [event._pad[0], event._pad[1]];
        let policy_kind = i16::from_le_bytes(kind_bytes) as i32;
        let side = if matches!(event.kind, EventKind::OfferedQosIncompatible) {
            "offered"
        } else {
            "requested"
        };
        let msg = format!(
            "DDS reported incompatible QoS on {fqn} ({side} side): {} new mismatch(es) on policy {} (total {})",
            event.stamp_nanosec,
            Self::policy_kind_name(policy_kind),
            event.stamp_sec,
        );
        self.emit_repeatable(
            "qos-match-runtime".to_string(),
            Severity::Error,
            fqn,
            msg,
            event.monotonic_ns,
        );
    }

    fn emit_dds_deadline_missed(&mut self, event: &InterceptionEvent) {
        if event.stamp_nanosec == 0 {
            return;
        }
        let fqn = self.resolve_fqn(event.topic_hash);
        let side = if matches!(event.kind, EventKind::OfferedDeadlineMissed) {
            "publisher"
        } else {
            "subscription"
        };
        let msg = format!(
            "DDS deadline missed on {fqn} ({side} side): {} new miss(es), total {}",
            event.stamp_nanosec, event.stamp_sec,
        );
        self.emit_repeatable(
            "deadline-runtime".to_string(),
            Severity::Warning,
            fqn,
            msg,
            event.monotonic_ns,
        );
    }

    fn emit_dds_liveliness(&mut self, event: &InterceptionEvent) {
        let fqn = self.resolve_fqn(event.topic_hash);
        match event.kind {
            EventKind::LivelinessLost => {
                if event.stamp_nanosec == 0 {
                    return;
                }
                let msg = format!(
                    "DDS liveliness lost on {fqn} (publisher): {} new loss(es), total {}",
                    event.stamp_nanosec, event.stamp_sec,
                );
                self.emit_repeatable(
                    "liveliness-runtime".to_string(),
                    Severity::Error,
                    fqn,
                    msg,
                    event.monotonic_ns,
                );
            }
            EventKind::LivelinessChanged => {
                // Only emit on transitions to or from not-alive.
                let alive_change = event._pad[0] as i8 as i32;
                let not_alive_change = event._pad[1] as i8 as i32;
                if not_alive_change == 0 && alive_change == 0 {
                    return;
                }
                let alive = event.stamp_sec;
                let not_alive = event.stamp_nanosec as i32;
                let msg = format!(
                    "DDS liveliness changed on {fqn} (subscription): alive={} (Δ{}) not_alive={} (Δ{})",
                    alive, alive_change, not_alive, not_alive_change,
                );
                // Only flag as a violation when publishers transitioned
                // into the not-alive state. Pure recoveries (alive++,
                // not_alive--) are logged as warnings still since the
                // topic graph changed shape — operators usually want to
                // know.
                let _ = alive_change;
                let _ = not_alive_change;
                self.emit_repeatable(
                    "liveliness-runtime".to_string(),
                    Severity::Warning,
                    fqn,
                    msg,
                    event.monotonic_ns,
                );
            }
            _ => {}
        }
    }

    fn emit_dds_message_lost(&mut self, event: &InterceptionEvent) {
        if event.stamp_nanosec == 0 {
            return;
        }
        let fqn = self.resolve_fqn(event.topic_hash);
        let msg = format!(
            "DDS dropped messages on {fqn}: {} new loss(es), total {}",
            event.stamp_nanosec, event.stamp_sec,
        );
        self.emit_repeatable(
            "drop-rate-runtime".to_string(),
            Severity::Warning,
            fqn,
            msg,
            event.monotonic_ns,
        );
    }

    /// Final flush — call before dropping.
    pub fn flush(&mut self) {
        if let Some(w) = self.violations_out.as_mut() {
            let _ = w.flush();
        }
        // Dump runtime-discovered (topic, type) pairs alongside the
        // violations jsonl. Useful for auditing manifest coverage and
        // backfilling missing `type:` fields on `external_topics:`
        // declarations.
        if let Some(dir) = self.violations_path.parent() {
            let tsv_path = dir.join("discovered_topic_types.tsv");
            let mut lines: Vec<String> = Vec::with_capacity(self.discovered_types.len());
            for (hash, ty) in &self.discovered_types {
                let topic = self
                    .discovered_names
                    .get(hash)
                    .cloned()
                    .or_else(|| self.topic_hash_to_fqn.get(hash).cloned())
                    .unwrap_or_else(|| format!("hash:{hash:#x}"));
                lines.push(format!("{topic}\t{ty}"));
            }
            lines.sort();
            if !lines.is_empty() {
                let _ = std::fs::create_dir_all(dir);
                let _ = std::fs::write(&tsv_path, lines.join("\n") + "\n");
            }
        }
        debug!(
            "RuleEngine flushed: {} violation(s), {} discovered topic-type pair(s)",
            self.violation_count,
            self.discovered_types.len(),
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

/// Pack a `header.stamp` into a u64 key for the per-stamp publish
/// monotonic_ns map. Combines sec×1e9 + nsec into a single nanosecond
/// timestamp.
#[inline]
fn pack_stamp(sec: i32, nsec: u32) -> u64 {
    (sec as u32 as u64) * 1_000_000_000 + (nsec as u64)
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

pub(crate) fn qualify(ns: &str, name: &str) -> String {
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
    use crate::ros::manifest_loader::ManifestIndex;

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
        use crate::{
            interception::{EventKind, InterceptionEvent},
            ros::manifest_loader::ResolvedTopic,
        };

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

        let mut re = RuleEngine::new(
            Arc::new(ContractView::from_manifest_index(&index)),
            EnforceMode::Warn,
            &tmp,
        );
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
        let tmp =
            std::env::temp_dir().join(format!("play_launch_graph_dev_test_{}", std::process::id()));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(
            Arc::new(ContractView::from_manifest_index(&index)),
            EnforceMode::Warn,
            &tmp,
        );

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

        assert!(
            re.violation_count >= 1,
            "expected graph-deviation violation"
        );
        let contents = std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap();
        assert!(contents.contains("graph-deviation-runtime"), "{contents}");

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn lifecycle_gating_default_unknown_blocks_check() {
        use crate::{
            interception::{EventKind, InterceptionEvent},
            ros::manifest_loader::{ResolvedManifest, ResolvedTopic},
        };
        use ros_launch_manifest_types::{Manifest, NodeDecl};

        // Build a manifest where `talker` is lifecycle=true and publishes
        // to /chatter at declared min_rate_hz=30.
        let mut nodes = std::collections::BTreeMap::new();
        let mut talker = NodeDecl {
            lifecycle: Some(true),
            ..Default::default()
        };
        let pub_props = ros_launch_manifest_types::EndpointProps {
            min_rate_hz: Some(30.0),
            ..Default::default()
        };
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
            channel: crate::ros::manifest_loader::ContractChannel::Provider,
            contract_path: std::path::PathBuf::new(),
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

        let tmp =
            std::env::temp_dir().join(format!("play_launch_lifecycle_test_{}", std::process::id()));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(
            Arc::new(ContractView::from_manifest_index(&index)),
            EnforceMode::Warn,
            &tmp,
        );

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
    fn consistency_runtime_fires_on_type_mismatch() {
        use crate::{
            interception::{EventKind, InterceptionEvent},
            ros::manifest_loader::ResolvedTopic,
        };

        let mut index = ManifestIndex::default();
        let fqn = "/chatter".to_string();
        index.topics.insert(
            fqn.clone(),
            ResolvedTopic {
                fqn: fqn.clone(),
                msg_type: "std_msgs/msg/String".to_string(),
                publishers: vec!["/talker/out".to_string()],
                subscribers: vec![],
                rate_hz: None,
                qos: None,
                max_transport_ms: None,
                drop: None,
                scope_ids: vec![0],
            },
        );

        let tmp = std::env::temp_dir().join(format!(
            "play_launch_consistency_test_{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(
            Arc::new(ContractView::from_manifest_index(&index)),
            EnforceMode::Warn,
            &tmp,
        );

        // Send a PublisherInit with the wrong runtime type hash —
        // declared is std_msgs/msg/String. Use a deliberately wrong
        // hash that won't collide.
        let wrong_hash: u64 = 0xDEAD_BEEF_DEAD_BEEF;
        let event = InterceptionEvent {
            kind: EventKind::PublisherInit,
            _pad: [0; 3],
            topic_hash: fnv1a(fqn.as_bytes()),
            stamp_sec: (wrong_hash >> 32) as i32,
            stamp_nanosec: (wrong_hash & 0xFFFF_FFFF) as u32,
            handle: 0xAA,
            monotonic_ns: 1_000_000,
        };
        re.observe(&event);
        re.flush();

        assert!(re.violation_count >= 1);
        let contents = std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap();
        assert!(contents.contains("consistency-runtime"), "{contents}");

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn max_age_runtime_fires_when_stamp_too_old() {
        use crate::{
            interception::{EventKind, InterceptionEvent},
            ros::manifest_loader::{ResolvedManifest, ResolvedTopic},
        };
        use ros_launch_manifest_types::{Manifest, NodeDecl};

        // Build manifest: subscriber declares max_age_ms=10.
        let mut nodes = std::collections::BTreeMap::new();
        let mut sub_node = NodeDecl::default();
        let sub_props = ros_launch_manifest_types::EndpointProps {
            max_age_ms: Some(10.0),
            ..Default::default()
        };
        sub_node.subscribers.insert("in".to_string(), sub_props);
        nodes.insert("listener".to_string(), sub_node);

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
            channel: crate::ros::manifest_loader::ContractChannel::Provider,
            contract_path: std::path::PathBuf::new(),
            manifest,
            source: String::new(),
            diagnostics: vec![],
        };
        let mut index = ManifestIndex::default();
        index.manifests.insert(0, resolved);
        let fqn = "/chatter".to_string();
        index.topics.insert(
            fqn.clone(),
            ResolvedTopic {
                fqn: fqn.clone(),
                msg_type: "std_msgs/msg/String".to_string(),
                publishers: vec![],
                subscribers: vec!["/listener/in".to_string()],
                rate_hz: None,
                qos: None,
                max_transport_ms: None,
                drop: None,
                scope_ids: vec![0],
            },
        );

        let tmp =
            std::env::temp_dir().join(format!("play_launch_max_age_test_{}", std::process::id()));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(
            Arc::new(ContractView::from_manifest_index(&index)),
            EnforceMode::Warn,
            &tmp,
        );

        // Take event carrying a stamp 1 second in the past — older
        // than max_age_ms=10ms.
        let one_second_ago = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .saturating_sub(std::time::Duration::from_secs(1));
        let event = InterceptionEvent {
            kind: EventKind::Take,
            _pad: [0; 3],
            topic_hash: fnv1a(fqn.as_bytes()),
            stamp_sec: one_second_ago.as_secs() as i32,
            stamp_nanosec: one_second_ago.subsec_nanos(),
            handle: 0x1,
            monotonic_ns: 1_000_000,
        };
        re.observe(&event);
        re.flush();

        assert!(re.violation_count >= 1);
        let contents = std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap();
        assert!(contents.contains("max-age-runtime"), "{contents}");

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn strict_mode_trips_atomic_flag() {
        use crate::interception::{EventKind, InterceptionEvent};

        let index = ManifestIndex::default();
        let tmp =
            std::env::temp_dir().join(format!("play_launch_strict_test_{}", std::process::id()));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(
            Arc::new(ContractView::from_manifest_index(&index)),
            EnforceMode::Strict,
            &tmp,
        );
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
