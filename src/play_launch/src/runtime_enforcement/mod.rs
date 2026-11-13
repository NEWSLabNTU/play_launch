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
    collections::{HashMap, HashSet},
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
};

use serde::Serialize;
use tracing::{debug, warn};

use crate::{
    cli::options::EnforceMode,
    interception::{EventKind, InterceptionEvent, TopicNameAssembly, decode_topic_name_chunk},
};
mod levels;
pub mod view;

/// A predicate's operator as a reader writes it.
fn op_symbol(op: ros_launch_manifest_model::CompareOp) -> &'static str {
    use ros_launch_manifest_model::CompareOp as Op;
    match op {
        Op::Equals => "==",
        Op::NotEquals => "!=",
        Op::Lt => "<",
        Op::Le => "<=",
        Op::Gt => ">",
        Op::Ge => ">=",
    }
}

/// A predicate's constant as the contract wrote it.
fn predicate_constant(v: &ros_launch_manifest_model::PredicateValueContract) -> String {
    use ros_launch_manifest_model::PredicateValueContract as V;
    match v {
        V::Bool(b) => b.to_string(),
        V::Int(i) => i.to_string(),
        V::Float(f) => f.to_string(),
        V::Constant { constant, .. } => constant.clone(),
        V::Str(s) => format!("\"{s}\""),
    }
}
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
    /// Where a `hazard-reaction` was seen, when it was not the sink's
    /// publish: `take` for a sink published off-host (phase 83 F2), whose
    /// first host take is the earliest the host can see the reaction. Absent
    /// otherwise, so every other line is unchanged.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub observed_at: Option<&'static str>,
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    Warning,
    Error,
}

impl Severity {
    /// `Warning < Error`, the static checker's order.
    pub fn at_least(self, threshold: Severity) -> bool {
        match (self, threshold) {
            (_, Severity::Warning) | (Severity::Error, Severity::Error) => true,
            (Severity::Warning, Severity::Error) => false,
        }
    }
}

/// Topics rcl creates for every node before user code runs. Seeded into
/// the runtime view as implicitly external, so `graph-deviation-runtime`
/// reports only topics an author could plausibly have declared (issue
/// #0032). `ContractView::externals` stays exactly what the contracts say.
pub const RCL_INTERNAL_TOPICS: &[&str] = &["/rosout", "/rosout_agg", "/parameter_events"];

/// Live state of one hazard (phase 73).
#[derive(Debug, Default)]
struct HazardRuntime {
    /// Guard FQN → last publish (monotonic ns).
    last_pub: HashMap<String, u64>,
    /// Guard FQN → recent inter-publish gaps, for the cadence-based
    /// silence threshold when no detector is declared.
    gaps: HashMap<String, std::collections::VecDeque<u64>>,
    /// Guard FQN → recent `header.stamp`s it published, so a sink publish
    /// that merely answers the guard's LAST message is not mistaken for the
    /// reaction (measured: 11ms for a 100ms lease before this).
    stamps: HashMap<String, std::collections::VecDeque<u64>>,
    /// The fault in progress: (guard, fault ns = its last publish).
    fault: Option<(String, u64)>,
    detected_ns: Option<u64>,
    reacted: bool,
    /// Recent sink publishes, (monotonic ns, stamp key). The reaction often
    /// lands BEFORE the observer's own silence threshold trips — the node's
    /// watchdog is faster than ten periods — so detection has to look back
    /// for it rather than only forward.
    sink_pubs: std::collections::VecDeque<(u64, Option<u64>, String)>,
    /// Recent host TAKES of a sink (phase 83 F2), same shape as `sink_pubs`.
    /// Read only for a sink no instrumented process publishes: the island's
    /// braking command leaves an MCU and the host first sees it where
    /// `vehicle_cmd_gate` takes it.
    sink_takes: std::collections::VecDeque<(u64, Option<u64>, String)>,
    /// Sinks a host process has been seen publishing: their takes are never
    /// the reaction.
    sink_host_pub: std::collections::HashSet<String>,
    /// Phase 102 M2: each guard publisher's last stamp, `(guard, handle)`,
    /// for `repetition` and `resequencing`.
    last_stamp: HashMap<(String, u64), u64>,
    /// Class findings already reported, `(guard, publisher handle, class)`.
    class_reported: std::collections::HashSet<(String, u64, &'static str)>,
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
    /// When the rate-hierarchy check last ran for this topic (monotonic
    /// ns). Slow topics never reach the publish-count trigger, so a
    /// time-based trigger runs the check at least every ~5 s.
    last_rate_check_monotonic_ns: u64,
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
    /// Phase 73 — live fault detection and reaction, one per hazard.
    hazards: Vec<HazardRuntime>,
    /// Phase 75 — which guarded topics are currently silent, and which modes
    /// have already been reported lost (so a transition reports once, not
    /// once per event).
    silent_topics: std::collections::HashSet<String>,
    modes_lost: std::collections::HashSet<String>,
    /// Phase 102 M2 — value functions, active levels and their commands.
    levels: levels::Levels,
    /// The value-lost functions `recheck_modes` last judged with.
    value_lost_seen: std::collections::HashSet<String>,
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
    /// Count of violations seen, of every severity.
    pub violation_count: u64,
    /// Hashes of the node FQNs a contract describes
    /// (`ContractView::contracted_nodes`), as `EndpointOwner` carries them.
    contracted_node_hashes: HashSet<u64>,
    /// Endpoint handle -> owning node's hash, from the `EndpointOwner`
    /// event that precedes the endpoint's init; taken at the init.
    endpoint_owner: HashMap<u64, u64>,
    /// Node FQNs by hash, from `NodeNameDeclared` chunks (phase 102 M2), so
    /// a finding about an endpoint can name its node.
    node_names: HashMap<u64, String>,
    node_name_chunks: HashMap<u64, TopicNameAssembly>,
    /// `(topic, owner hash)` publisher findings already reported.
    publisher_reported: HashSet<(String, u64)>,
    /// Shared flag flipped on the first violation at or above
    /// `strict_threshold` in Strict mode. Owners hold a `clone()` of the
    /// `Arc` and poll it to trigger shutdown. Below the threshold a
    /// violation is logged and recorded but does not end a strict run
    /// (issue #0032).
    pub strict_violated: Arc<AtomicBool>,
    /// The severity at which a violation trips `strict_violated`. Default
    /// `Error`: the warning class exists for claims the launch tree cannot
    /// settle (`graph-deviation-runtime`). `--strict-on warning` lowers it.
    strict_threshold: Severity,
    /// `"<rule_id> on <fqn>"` of the first violation that tripped strict
    /// mode, for the exit message.
    first_strict_violation: Arc<Mutex<Option<String>>>,
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
        // Issue #0032: every node's `/rosout` and `/parameter_events`
        // publishers exist before user code runs; they are not deviations
        // from any launch tree.
        for fqn in RCL_INTERNAL_TOPICS {
            topic_hash_to_fqn
                .entry(fnv1a(fqn.as_bytes()))
                .or_insert_with(|| fqn.to_string());
        }

        // Lifecycle nodes (Phase 36.6): rate/age/latency rules gate on
        // observed lifecycle state.
        let lifecycle_nodes: HashMap<String, bool> = view
            .lifecycle_nodes
            .iter()
            .map(|f| (f.clone(), true))
            .collect();

        let contracted_node_hashes = view
            .contracted_nodes
            .iter()
            .map(|n| fnv1a(n.as_bytes()))
            .collect();

        Self {
            hazards: view
                .hazards
                .iter()
                .map(|_| HazardRuntime::default())
                .collect(),
            silent_topics: std::collections::HashSet::new(),
            modes_lost: std::collections::HashSet::new(),
            levels: levels::Levels::default(),
            value_lost_seen: std::collections::HashSet::new(),
            view,
            mode,
            topic_hash_to_fqn,
            state: HashMap::new(),
            lifecycle_nodes,
            lifecycle_state: HashMap::new(),
            violations_out: None,
            violations_path: log_dir.join("runtime_violations.jsonl"),
            violation_count: 0,
            contracted_node_hashes,
            endpoint_owner: HashMap::new(),
            node_names: HashMap::new(),
            node_name_chunks: HashMap::new(),
            publisher_reported: HashSet::new(),
            strict_violated: Arc::new(AtomicBool::new(false)),
            strict_threshold: Severity::Error,
            first_strict_violation: Arc::new(Mutex::new(None)),
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

    /// Shared handle to the name of the first violation that tripped
    /// strict mode (`"<rule_id> on <fqn>"`), `None` until one has.
    pub fn first_strict_violation_handle(&self) -> Arc<Mutex<Option<String>>> {
        Arc::clone(&self.first_strict_violation)
    }

    /// Lower (or restate) the severity at which a violation ends a strict
    /// run: `--strict-on warning|error`.
    pub fn with_strict_threshold(mut self, threshold: Severity) -> Self {
        self.strict_threshold = threshold;
        self
    }

    /// Observe one interception event. Cheap and non-blocking — runs
    /// inline in the listener task.
    pub fn observe(&mut self, event: &InterceptionEvent) {
        if matches!(self.mode, EnforceMode::Off) {
            return;
        }
        // Per-endpoint attribution for `endpoint_stats.json`. These carry a
        // NODE hash in `topic_hash`, which must not create topic state below.
        if matches!(event.kind, EventKind::EndpointOwner) {
            self.endpoint_owner.insert(event.handle, event.topic_hash);
        }
        if matches!(event.kind, EventKind::NodeNameDeclared) {
            let (idx, total, n, buf) = decode_topic_name_chunk(event);
            let entry = self
                .node_name_chunks
                .entry(event.topic_hash)
                .or_insert_with(|| TopicNameAssembly::new(total.max(1)));
            if entry.total != total.max(1) {
                *entry = TopicNameAssembly::new(total.max(1));
            }
            entry.add(idx, &buf[..n]);
            if let Some(name) = entry.assembled() {
                self.node_names.entry(event.topic_hash).or_insert(name);
                self.node_name_chunks.remove(&event.topic_hash);
            }
            return;
        }
        if matches!(event.kind, EventKind::FieldValue) {
            self.observe_field(event);
            return;
        }
        if matches!(event.kind, EventKind::Publish)
            && let Some(fqn) = self.topic_hash_to_fqn.get(&event.topic_hash)
            && self.view.modes.iter().any(|m| m.commands.contains_key(fqn))
        {
            let fqn = fqn.clone();
            self.levels.note_publish(&fqn, event.monotonic_ns);
        }
        if matches!(
            event.kind,
            EventKind::EndpointOwner
                | EventKind::NodeNameDeclared
                // Phase 95: calls are keyed by service, not topic; the
                // interception task pairs them (`interception::calls`).
                | EventKind::ServiceClientInit
                | EventKind::ServiceServerInit
                | EventKind::ServiceRequestSent
                | EventKind::ServiceRequestTaken
                | EventKind::ServiceResponseSent
                | EventKind::ServiceResponseTaken
        ) {
            return;
        }
        self.observe_hazards(event);

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
                // publishes on busy topics, and at least every ~5 s on slow
                // ones (a 1 Hz topic would otherwise NEVER reach the count
                // trigger and escape its rate contract entirely).
                let time_due = entry.last_rate_check_monotonic_ns == 0
                    || event
                        .monotonic_ns
                        .saturating_sub(entry.last_rate_check_monotonic_ns)
                        >= 5_000_000_000;
                if entry.pub_count.is_multiple_of(1024) || time_due {
                    entry.last_rate_check_monotonic_ns = event.monotonic_ns;
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
                // #0080: only a node some contract describes is under a
                // claim. An endpoint whose owner is known and is not such a
                // node is outside every contract, so its topic deviates
                // from nothing. An unknown owner (an interceptor that sends
                // no `EndpointOwner`) is still judged, as before.
                let owner = self.endpoint_owner.remove(&event.handle);
                if matches!(event.kind, EventKind::PublisherInit)
                    && let Some(o) = owner
                    && let Some(fqn) = self.topic_hash_to_fqn.get(&event.topic_hash).cloned()
                {
                    drop_borrow(&mut self.state);
                    self.judge_publisher(&fqn, o, event.monotonic_ns);
                }
                let uncontracted = owner.is_some_and(|o| !self.contracted_node_hashes.contains(&o));
                if !uncontracted && !self.topic_hash_to_fqn.contains_key(&event.topic_hash) {
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
            EventKind::EndpointOwner | EventKind::NodeNameDeclared => {
                // Returned early above: no topic is involved.
            }
            EventKind::EndpointQos => {
                // The live Entities view's QoS; `qos-match-runtime` reads the
                // rmw-layer QosDeclared events and the DDS verdicts instead.
            }
            EventKind::TfEdge
            | EventKind::TfFramesDeclared
            | EventKind::LifecycleTransition
            | EventKind::FieldValue
            | EventKind::ServiceClientInit
            | EventKind::ServiceServerInit
            | EventKind::ServiceRequestSent
            | EventKind::ServiceRequestTaken
            | EventKind::ServiceResponseSent
            | EventKind::ServiceResponseTaken => {
                // TF edges for the Health page; no contract rule reads them.
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
                            "event chain '{}' (input '{}' → output '{}'): observed latency \
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
        // External topics don't carry a declared type — skip to avoid false
        // positives.
        let declared_type = self
            .view
            .topics
            .get(&fqn)
            .map(|topic| topic.msg_type.clone());
        let Some(declared) = declared_type else {
            return;
        };
        // A topic the model derived from a launch file's remaps (phase 76)
        // carries no type: nothing was declared, so there is nothing to
        // disagree with. Hashing the empty string reported every such topic
        // as a mismatch (#0079: 100 errors on Autoware).
        if declared.is_empty() || declared.starts_with("TODO/") {
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

    /// Issue #0032: strict mode acts on the engine's own severity
    /// vocabulary. Only a violation at or above the threshold (default
    /// `Error`) ends the run; the rest are logged and written to
    /// `runtime_violations.jsonl` like any other violation. The first one
    /// that trips is remembered by name for the exit message.
    fn trip_strict(&self, rule_id: &str, fqn: &str, severity: Severity) {
        if matches!(self.mode, EnforceMode::Strict) && severity.at_least(self.strict_threshold) {
            if let Ok(mut first) = self.first_strict_violation.lock()
                && first.is_none()
            {
                *first = Some(format!("{rule_id} on {fqn}"));
            }
            self.strict_violated.store(true, Ordering::Release);
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
        self.trip_strict(&rule_id, &fqn, severity);

        let v = RuntimeViolation {
            rule_id,
            severity,
            fqn,
            message,
            timestamp_ns,
            observed_at: None,
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
        self.emit_repeatable_at(rule_id, severity, fqn, message, timestamp_ns, None);
    }

    /// [`Self::emit_repeatable`] with the place the event was observed, for a
    /// reaction seen at a take rather than a publish (phase 83 F2).
    fn emit_repeatable_at(
        &mut self,
        rule_id: String,
        severity: Severity,
        fqn: String,
        message: String,
        timestamp_ns: u64,
        observed_at: Option<&'static str>,
    ) {
        self.violation_count += 1;
        self.trip_strict(&rule_id, &fqn, severity);
        let v = RuntimeViolation {
            rule_id,
            severity,
            fqn,
            message,
            timestamp_ns,
            observed_at,
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

    // -----------------------------------------------------------------------
    // Phase 73 — live fault detection and reaction
    // -----------------------------------------------------------------------

    /// Feed a hazard's guards and sinks from the event stream.
    ///
    /// A guard publish records cadence and provenance; a DDS liveliness or
    /// deadline event on a guard is a detection by the middleware
    /// (`mechanism: qos`); a sink publish during a fault that carries no
    /// guard provenance is the reaction. Silence itself is judged in
    /// [`RuleEngine::tick`], because a dead topic never produces an event.
    fn observe_hazards(&mut self, event: &InterceptionEvent) {
        if self.view.hazards.is_empty() {
            return;
        }
        let Some(fqn) = self.topic_hash_to_fqn.get(&event.topic_hash).cloned() else {
            return;
        };
        let stamp = (event.stamp_sec != 0 || event.stamp_nanosec != 0)
            .then(|| pack_stamp(event.stamp_sec, event.stamp_nanosec));
        let mut pending: Vec<Pending> = Vec::new();
        for (i, watch) in self.view.hazards.iter().enumerate() {
            let rt = &mut self.hazards[i];
            let is_guard = watch.guards.contains(&fqn);
            let is_sink = watch.sinks.contains(&fqn);
            match event.kind {
                EventKind::Publish if is_guard => {
                    if let Some(prev) = rt.last_pub.get(&fqn) {
                        let gaps = rt.gaps.entry(fqn.clone()).or_default();
                        gaps.push_back(event.monotonic_ns.saturating_sub(*prev));
                        if gaps.len() > 32 {
                            gaps.pop_front();
                        }
                    }
                    rt.last_pub.insert(fqn.clone(), event.monotonic_ns);
                    // Phase 102 M2: a stamp equal to, or earlier than, this
                    // publisher's previous one, where the hazard asks.
                    if let Some(k) = stamp
                        && let Some(classes) = self.view.hazard_classes.get(&watch.key)
                    {
                        use ros_launch_manifest_model::FaultKind as F;
                        let prev = rt.last_stamp.insert((fqn.clone(), event.handle), k);
                        let class = match prev {
                            Some(p) if p == k && classes.contains(&F::Repetition) => {
                                Some("repetition")
                            }
                            Some(p) if p > k && classes.contains(&F::Resequencing) => {
                                Some("resequencing")
                            }
                            _ => None,
                        };
                        if let Some(class) = class
                            && rt.class_reported.insert((fqn.clone(), event.handle, class))
                        {
                            pending.push((
                                "hazard-detected".to_string(),
                                Severity::Error,
                                watch.key.clone(),
                                format!(
                                    "hazard '{}': '{fqn}' carried a {class} -- a stamp {} the \
                                     same publisher's previous one",
                                    watch.key,
                                    if class == "repetition" {
                                        "equal to"
                                    } else {
                                        "earlier than"
                                    }
                                ),
                                None,
                            ));
                        }
                    }
                    if let Some(k) = stamp {
                        let st = rt.stamps.entry(fqn.clone()).or_default();
                        st.push_back(k);
                        if st.len() > 512 {
                            st.pop_front();
                        }
                    }
                    if let Some((g, fault_ns)) = &rt.fault
                        && *g == fqn
                    {
                        pending.push((
                            "hazard-recovered".to_string(),
                            Severity::Warning,
                            watch.key.clone(),
                            {
                                // Phase 102 M2: the recovery the run took, against
                                // the FTTI the static credit judged it by.
                                let after =
                                    event.monotonic_ns.saturating_sub(*fault_ns) as f64 / 1e6;
                                let verdict = match watch.ftti_ms {
                                    Some(f) if after <= f => format!(", inside the FTTI {f:.2}ms"),
                                    Some(f) => format!(", past the FTTI {f:.2}ms"),
                                    None => String::new(),
                                };
                                format!(
                                    "hazard '{}': '{fqn}' resumed publishing {after:.2}ms after it \
                                     went silent{verdict}",
                                    watch.key,
                                )
                            },
                            None,
                        ));
                        rt.fault = None;
                        rt.detected_ns = None;
                        rt.reacted = false;
                    }
                }
                EventKind::Publish if is_sink => {
                    rt.sink_host_pub.insert(fqn.clone());
                    rt.sink_pubs
                        .push_back((event.monotonic_ns, stamp, fqn.clone()));
                    if rt.sink_pubs.len() > 256 {
                        rt.sink_pubs.pop_front();
                    }
                    if rt.fault.is_some()
                        && !rt.reacted
                        && let Some(p) = reaction_of(rt, watch)
                    {
                        pending.push(p);
                    }
                }
                // Phase 83 F2: a sink published off-host is seen first where a
                // host process takes it.
                EventKind::Take if is_sink => {
                    rt.sink_takes
                        .push_back((event.monotonic_ns, stamp, fqn.clone()));
                    if rt.sink_takes.len() > 256 {
                        rt.sink_takes.pop_front();
                    }
                    if rt.fault.is_some()
                        && !rt.reacted
                        && let Some(p) = reaction_of(rt, watch)
                    {
                        pending.push(p);
                    }
                }
                EventKind::LivelinessChanged
                | EventKind::LivelinessLost
                | EventKind::RequestedDeadlineMissed
                    if is_guard && rt.fault.is_none() =>
                {
                    // The middleware detected it before the observer's own
                    // silence threshold did: `mechanism: qos`.
                    let not_alive =
                        event.kind != EventKind::LivelinessChanged || (event._pad[1] as i8) > 0;
                    if !not_alive {
                        continue;
                    }
                    let fault_ns = rt.last_pub.get(&fqn).copied().unwrap_or(event.monotonic_ns);
                    rt.fault = Some((fqn.clone(), fault_ns));
                    rt.detected_ns = Some(event.monotonic_ns);
                    pending.push((
                        "hazard-detected".to_string(),
                        Severity::Error,
                        watch.key.clone(),
                        format!(
                            "hazard '{}': '{fqn}' reported {:?} by DDS {:.2}ms after its last \
                             publish (mechanism: qos)",
                            watch.key,
                            event.kind,
                            event.monotonic_ns.saturating_sub(fault_ns) as f64 / 1e6
                        ),
                        None,
                    ));
                    if let Some(p) = reaction_of(rt, watch) {
                        pending.push(p);
                    }
                }
                EventKind::Publish if watch.via_diagnostics && fqn == "/diagnostics" => {
                    if let Some((_, fault_ns)) = rt.fault
                        && rt.detected_ns.is_none()
                        && event.monotonic_ns > fault_ns
                    {
                        rt.detected_ns = Some(event.monotonic_ns);
                    }
                }
                _ => {}
            }
        }
        for (rule, sev, key, msg, at) in pending {
            self.emit_repeatable_at(rule, sev, key, msg, event.monotonic_ns, at);
        }
        self.recheck_modes(event.monotonic_ns);
    }

    /// Phase 75 — derive mode availability from the guards that are silent.
    ///
    /// A function is lost when its group is (every member for `all_of`, any
    /// member otherwise); a mode is lost when a function it requires is. The
    /// ladder is walked in order and the first still-available rung is what
    /// the system falls to — the selection `mrm_handler` makes at runtime,
    /// reported here against the contract that predicted it.
    ///
    /// Reported once per transition: a 50 Hz guard would otherwise emit the
    /// same line fifty times a second.
    fn recheck_modes(&mut self, ts: u64) {
        if self.view.modes.is_empty() {
            return;
        }
        // Which guarded topics are silent, from the hazards' own state — the
        // observer already tracks exactly this, per guard.
        let silent: std::collections::HashSet<String> = self
            .hazards
            .iter()
            .filter_map(|rt| rt.fault.as_ref().map(|(g, _)| g.clone()))
            .collect();
        if silent == self.silent_topics && self.levels.value_lost == self.value_lost_seen {
            return;
        }
        self.silent_topics = silent;
        self.value_lost_seen = self.levels.value_lost.clone();
        let value_lost = self.value_lost_seen.clone();

        // A function is lost by silence (any member; for a redundant group,
        // when fewer than `m` members still publish) or, for a value
        // function, by value (phase 102 M2).
        let lost = |w: &crate::runtime_enforcement::view::ModeWatch,
                    silent: &std::collections::HashSet<String>| {
            w.requires.iter().any(|r| {
                let by_value = r.value_fn.as_ref().is_some_and(|k| value_lost.contains(k));
                let by_silence = if r.all_of {
                    let held = r.members.iter().filter(|t| !silent.contains(*t)).count();
                    held < r.m.unwrap_or(1) as usize
                } else {
                    r.members.iter().any(|t| silent.contains(t))
                };
                by_value || by_silence
            })
        };
        let mut pending: Vec<(Severity, String, String, bool)> = Vec::new();
        for w in &self.view.modes {
            let is_lost = lost(w, &self.silent_topics);
            if is_lost == self.modes_lost.contains(&w.key) {
                continue;
            }
            if is_lost {
                let rung = w.fallback.iter().find(|name| {
                    self.view
                        .modes
                        .iter()
                        .find(|m| &m.key == *name)
                        .is_some_and(|m| !lost(m, &self.silent_topics))
                });
                if let Some(r) = rung {
                    self.levels.enter(r, ts);
                }
                let (sev, to) = match rung {
                    Some(r) => (Severity::Warning, format!("falling to '{r}'")),
                    None if w.fallback.is_empty() => (
                        Severity::Warning,
                        "no fallback sequence declared".to_string(),
                    ),
                    None => (
                        Severity::Error,
                        format!(
                            "NO degradation level of [{}] is available either — the fallback sequence has no \
                                 minimal risk condition left",
                            w.fallback.join(", ")
                        ),
                    ),
                };
                pending.push((
                    sev,
                    w.key.clone(),
                    format!("mode '{}': LOST — {to}", w.key),
                    true,
                ));
            } else {
                // The mode is back: every level it had fallen to is left.
                for l in &w.fallback {
                    self.levels.leave(l);
                }
                pending.push((
                    Severity::Warning,
                    w.key.clone(),
                    format!("mode '{}': available again", w.key),
                    false,
                ));
            }
        }
        for (sev, key, msg, now_lost) in pending {
            self.emit_repeatable("mode-availability".to_string(), sev, key.clone(), msg, ts);
            if now_lost {
                self.modes_lost.insert(key);
            } else {
                self.modes_lost.remove(&key);
            }
        }
    }

    /// Phase 102 M2: a publisher created on a topic the contract constrains.
    /// On a commanded topic any node but its arbiter (the topic's one
    /// declared publisher) is `arbiter-bypass`; on a hazard's monitored
    /// topic whose `on:` names `insertion`, a node the contract does not
    /// name is that fault. A commanded topic reports the bypass only, once.
    fn judge_publisher(&mut self, topic: &str, owner: u64, now: u64) {
        let declared: HashSet<u64> = self
            .view
            .topics
            .get(topic)
            .map(|t| {
                t.publishers
                    .iter()
                    .filter_map(|ep| ep.rsplit_once('/').map(|(n, _)| fnv1a(n.as_bytes())))
                    .collect()
            })
            .unwrap_or_default();
        if declared.contains(&owner) {
            return;
        }
        let commanded = self
            .view
            .modes
            .iter()
            .any(|m| m.commands.contains_key(topic));
        let inserted_into: Vec<String> =
            self.view
                .hazards
                .iter()
                .filter(|h| h.guards.iter().any(|g| g == topic))
                .filter(|h| {
                    self.view.hazard_classes.get(&h.key).is_some_and(|c| {
                        c.contains(&ros_launch_manifest_model::FaultKind::Insertion)
                    })
                })
                .map(|h| h.key.clone())
                .collect();
        if (!commanded && inserted_into.is_empty())
            || !self.publisher_reported.insert((topic.to_string(), owner))
        {
            return;
        }
        let node = self
            .node_names
            .get(&owner)
            .cloned()
            .unwrap_or_else(|| format!("a node (hash {owner:016x})"));
        let arbiter = self
            .view
            .topics
            .get(topic)
            .map(|t| t.publishers.join(", "))
            .unwrap_or_default();
        if commanded {
            self.emit_repeatable(
                "arbiter-bypass".into(),
                Severity::Error,
                topic.to_string(),
                format!(
                    "{node} created a publisher on '{topic}', a commanded topic whose arbiter is \
                     {arbiter}: a command can now reach it around the level's reaction"
                ),
                now,
            );
            return;
        }
        for key in inserted_into {
            self.emit_repeatable(
                "hazard-detected".into(),
                Severity::Error,
                key.clone(),
                format!(
                    "hazard '{key}': {node} created a publisher on the monitored '{topic}', which \
                     the contract does not name (insertion; declared: {arbiter})"
                ),
                now,
            );
        }
    }

    /// Phase 102 M2: one declared field of a published message. It updates
    /// the value functions that read it, and is judged against the
    /// `commands` of every active level past its derived bound.
    fn observe_field(&mut self, event: &InterceptionEvent) {
        let Some((topic, path)) = self.view.fields.get(event._pad[0] as usize).cloned() else {
            return;
        };
        let (kind, bits, now) = (event._pad[1], event.cpu_ns, event.monotonic_ns);

        let mut changed = false;
        for (key, vf) in &self.view.value_fns {
            if vf.when.field != path || !vf.topics.contains(&topic) {
                continue;
            }
            match levels::eval(&vf.when, kind, bits) {
                Some(true) => changed |= self.levels.value_lost.insert(key.clone()),
                Some(false) => changed |= self.levels.value_lost.remove(key),
                None => {}
            }
        }
        if changed {
            self.recheck_modes(now);
        }

        let mut pending: Vec<(Severity, String, String, String)> = Vec::new();
        for m in &self.view.modes {
            let Some(&entered) = self.levels.active.get(&m.key) else {
                continue;
            };
            let Some(preds) = m.commands.get(&topic) else {
                continue;
            };
            let Some(bound_ms) = m.bound_ms else {
                if self
                    .levels
                    .reported
                    .insert((m.key.clone(), "unjudged".into()))
                {
                    pending.push((
                        Severity::Warning,
                        "safe-state-unjudged".into(),
                        m.key.clone(),
                        format!(
                            "level '{}' commands '{topic}', but no hazard's reaction route reaches \
                             it, so when its commands must hold is unknown -- NOT judged",
                            m.key
                        ),
                    ));
                }
                continue;
            };
            let start = entered + (bound_ms * 1e6) as u64;
            if now < start {
                continue;
            }
            for pred in preds.iter().filter(|p| p.field == path) {
                if levels::eval(pred, kind, bits) != Some(false) {
                    continue;
                }
                if !self
                    .levels
                    .reported
                    .insert((m.key.clone(), format!("{topic}.{path}")))
                {
                    continue;
                }
                pending.push((
                    Severity::Error,
                    "safe-state-violated".into(),
                    m.key.clone(),
                    format!(
                        "level '{}' commands {topic} {path} {} {}, and a message carried {} \
                         {:.2}ms after the level's derived bound ({bound_ms:.2}ms after it was \
                         entered)",
                        m.key,
                        op_symbol(pred.op),
                        predicate_constant(&pred.value),
                        levels::show(kind, bits),
                        now.saturating_sub(start) as f64 / 1e6,
                    ),
                ));
            }
        }
        for (sev, rule, key, msg) in pending {
            self.emit_repeatable(rule, sev, key, msg, now);
        }
    }

    /// Phase 102 M2: a commanded topic gone silent while its level holds,
    /// and its consumer states nothing it does on silence.
    fn tick_levels(&mut self, now: u64) {
        let mut pending: Vec<(String, String)> = Vec::new();
        for m in &self.view.modes {
            let Some(&entered) = self.levels.active.get(&m.key) else {
                continue;
            };
            let Some(bound_ms) = m.bound_ms else {
                continue;
            };
            let start = entered + (bound_ms * 1e6) as u64;
            if now < start {
                continue;
            }
            for topic in m.commands.keys() {
                if self.view.silence_handled.contains(topic) {
                    continue;
                }
                let last = self
                    .levels
                    .last_pub
                    .get(topic)
                    .copied()
                    .unwrap_or(0)
                    .max(start);
                let quiet = now.saturating_sub(last);
                if quiet <= self.levels.silence_ns(topic) {
                    continue;
                }
                if !self
                    .levels
                    .reported
                    .insert((m.key.clone(), format!("silent:{topic}")))
                {
                    continue;
                }
                pending.push((
                    m.key.clone(),
                    format!(
                        "level '{}' commands {topic}, which has been silent {:.2}ms while the \
                         level holds, and its consumer states nothing it does on silence (a \
                         subscriber's `max_age` + `on_violation`, or `external: {{ side: sub, \
                         on_silence }}`)",
                        m.key,
                        quiet as f64 / 1e6
                    ),
                ));
            }
        }
        for (key, msg) in pending {
            self.emit_repeatable(
                "final-element-silent".into(),
                Severity::Error,
                key,
                msg,
                now,
            );
        }
    }

    /// Judge silence. A dead guard produces no event, so this runs on the
    /// consumer's poll tick with the current `CLOCK_MONOTONIC` time. The
    /// threshold is the fastest declared detector among the guard's reacting
    /// subscribers (`max_age` today), else ten of the topic's own median
    /// inter-publish gaps and at least 50ms — the same rule `measure` uses
    /// after the fact.
    pub fn tick(&mut self, now_ns: u64) {
        if matches!(self.mode, EnforceMode::Off) {
            return;
        }
        self.tick_levels(now_ns);
        if self.view.hazards.is_empty() {
            return;
        }
        let mut pending: Vec<Pending> = Vec::new();
        for (i, watch) in self.view.hazards.iter().enumerate() {
            let rt = &mut self.hazards[i];
            if rt.fault.is_some() {
                continue;
            }
            for guard in &watch.guards {
                let Some(&last) = rt.last_pub.get(guard) else {
                    continue;
                };
                let gaps = rt.gaps.get(guard);
                if gaps.is_none_or(|g| g.len() < 4) {
                    continue; // not enough cadence to judge silence against
                }
                let threshold_ns = match watch.silence_ms {
                    Some(ms) => (ms * 1e6) as u64,
                    None => (median_gap(gaps) * 10).max(50_000_000),
                };
                if now_ns.saturating_sub(last) > threshold_ns {
                    rt.fault = Some((guard.clone(), last));
                    rt.detected_ns = Some(now_ns);
                    pending.push((
                        "hazard-detected".to_string(),
                        Severity::Error,
                        watch.key.clone(),
                        format!(
                            "hazard '{}': '{guard}' silent for {:.2}ms (threshold {:.2}ms, {}) — the \
                             fault the hazard guards against is in progress",
                            watch.key,
                            now_ns.saturating_sub(last) as f64 / 1e6,
                            threshold_ns as f64 / 1e6,
                            if watch.silence_ms.is_some() {
                                "the declared lease / max_age"
                            } else {
                                "10x the topic's median period"
                            }
                        ),
                        None,
                    ));
                    // The node's own watchdog is usually faster than this
                    // threshold, so the reaction may already have happened.
                    if let Some(p) = reaction_of(rt, watch) {
                        pending.push(p);
                    }
                    break;
                }
            }
        }
        for (rule, sev, key, msg, at) in pending {
            self.emit_repeatable_at(rule, sev, key, msg, now_ns, at);
        }
        self.recheck_modes(now_ns);
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

/// A diagnostic waiting to be emitted: rule, severity, hazard key, message,
/// and where the event was observed when it was not a publish.
type Pending = (String, Severity, String, String, Option<&'static str>);

/// The reaction to the fault in progress, if a provenance-free sink publish
/// after the fault has been seen — looking BACK through the sink buffer,
/// since the node's watchdog usually reacts before the observer's threshold
/// trips. Marks the hazard reacted and returns the diagnostic to emit.
///
/// A sink no instrumented process publishes (phase 83 F2: declared
/// `external: pub`, or simply never seen published by a host process) is
/// judged by its host TAKES instead, and the diagnostic says so: the number
/// then includes the hop from the off-host publisher, which is the island's
/// link, and hiding that would credit the link's latency to the island.
fn reaction_of(
    rt: &mut HazardRuntime,
    watch: &crate::runtime_enforcement::view::HazardWatch,
) -> Option<Pending> {
    let (guard, fault_ns) = rt.fault.clone()?;
    let gap = median_gap(rt.gaps.get(&guard));
    let provenance_free = |(t, stamp, _): &&(u64, Option<u64>, String)| match stamp {
        Some(k) => !rt.stamps.get(&guard).is_some_and(|s| s.contains(k)),
        None => *t > fault_ns + gap,
    };
    let published = rt
        .sink_pubs
        .iter()
        .filter(|(t, _, _)| *t > fault_ns)
        .find(provenance_free)
        .cloned();
    let (hit, observed_at) = match published {
        Some(h) => (h, None),
        None => {
            let off_host = |sink: &String| {
                watch.off_host_sinks.contains(sink) || !rt.sink_host_pub.contains(sink)
            };
            let taken = rt
                .sink_takes
                .iter()
                .filter(|(t, _, sink)| *t > fault_ns && off_host(sink))
                .find(provenance_free)
                .cloned()?;
            (taken, Some("take"))
        }
    };
    rt.reacted = true;
    let (t, _, sink) = hit;
    let react_ms = (t - fault_ns) as f64 / 1e6;
    let settle = watch.settle_ms.unwrap_or(0.0);
    let total = react_ms + settle;
    let (sev, verdict) = match watch.ftti_ms {
        Some(ftti) if total <= ftti => (
            Severity::Warning,
            format!("fits ftti {ftti:.2}ms with {:.2}ms of slack", ftti - total),
        ),
        Some(ftti) => (
            Severity::Error,
            format!("EXCEEDS ftti {ftti:.2}ms by {:.2}ms", total - ftti),
        ),
        None => (Severity::Warning, "no ftti declared".to_string()),
    };
    let detected = rt
        .detected_ns
        .map(|d| {
            format!(
                ", observer noticed at {:.2}ms",
                d.saturating_sub(fault_ns) as f64 / 1e6
            )
        })
        .unwrap_or_default();
    let at = if observed_at.is_some() {
        " (observed_at: take -- the sink is published off-host, so this is its first host \
         take, and the link hop is inside the number)"
    } else {
        ""
    };
    Some((
        "hazard-reaction".to_string(),
        sev,
        watch.key.clone(),
        format!(
            "hazard '{}': reaction on '{sink}' {react_ms:.2}ms after '{guard}' went \
             silent{detected}{at}; + settle {settle:.2}ms = {total:.2}ms — {verdict}",
            watch.key
        ),
        observed_at,
    ))
}

/// Median of recorded inter-publish gaps, ns; a generous default when there
/// is no cadence yet so a sink publish is never mistaken for a reaction on
/// the strength of no evidence.
fn median_gap(gaps: Option<&std::collections::VecDeque<u64>>) -> u64 {
    let Some(g) = gaps.filter(|g| !g.is_empty()) else {
        return 50_000_000;
    };
    let mut v: Vec<u64> = g.iter().copied().collect();
    v.sort_unstable();
    v[v.len() / 2]
}

/// `CLOCK_MONOTONIC` in ns, the clock every interception event carries.
pub fn monotonic_now_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a valid out-pointer; CLOCK_MONOTONIC always exists.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

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

/// Phase 47.B3: only `ContractView::from_manifest_index` calls this now,
/// and that's `#[allow(dead_code)]`-kept for tests only — see its doc
/// comment.
#[allow(dead_code)]
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
    use ros_launch_resolve::ros::manifest_loader::ManifestIndex;

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
        use ros_launch_resolve::ros::manifest_loader::ResolvedTopic;

        // Build a minimal ManifestIndex containing one topic.
        let mut index = ManifestIndex::default();
        let fqn = "/test/topic".to_string();
        index.topics.insert(
            fqn.clone(),
            ResolvedTopic {
                derived_from_remaps: false,
                fqn: fqn.clone(),
                msg_type: "std_msgs/msg/String".to_string(),
                publishers: vec!["/test/pub_node/out".to_string()],
                subscribers: vec!["/test/sub_node/in".to_string()],
                rate_hz: None,
                derived_rate_hz: None,
                qos: None,
                drop: None,
                scope_ids: vec![0],
                on_silence_after_ms: None,
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
            cpu_ns: 0,
            tid: 0,
            payload_bytes: 0,
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
            cpu_ns: 0,
            tid: 0,
            payload_bytes: 0,
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

    /// Sends an `EndpointOwner` for node `owner`, then a publisher init on
    /// an undeclared topic, to an engine whose contracts describe
    /// `/described`. Returns the violations file.
    fn graph_deviation_with_owner(owner: &str, tag: &str) -> String {
        use crate::interception::{EventKind, InterceptionEvent};

        let view = ContractView {
            contracted_nodes: ["/described".to_string()].into_iter().collect(),
            ..ContractView::default()
        };
        let tmp = std::env::temp_dir().join(format!(
            "play_launch_graph_dev_owner_{tag}_{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(Arc::new(view), EnforceMode::Warn, &tmp);
        let event = |kind, topic_hash| InterceptionEvent {
            kind,
            _pad: [0; 3],
            topic_hash,
            stamp_sec: 0,
            stamp_nanosec: 0,
            handle: 0x7,
            monotonic_ns: 1_000_000,
            cpu_ns: 0,
            tid: 0,
            payload_bytes: 0,
        };
        re.observe(&event(EventKind::EndpointOwner, fnv1a(owner.as_bytes())));
        re.observe(&event(EventKind::PublisherInit, 0xDEADBEEFu64));
        re.flush();
        let contents =
            std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap_or_default();
        let _ = std::fs::remove_dir_all(&tmp);
        contents
    }

    #[test]
    fn graph_deviation_judges_only_a_node_a_contract_describes() {
        // #0080: a described node's undeclared topic is a deviation; an
        // undescribed node's is not, since no contract makes a claim on it.
        let described = graph_deviation_with_owner("/described", "described");
        assert!(described.contains("graph-deviation-runtime"), "{described}");
        let other = graph_deviation_with_owner("/elsewhere/node", "other");
        assert!(!other.contains("graph-deviation-runtime"), "{other}");
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
            cpu_ns: 0,
            tid: 0,
            payload_bytes: 0,
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
        use crate::interception::{EventKind, InterceptionEvent};
        use ros_launch_manifest_types::{Manifest, NodeDecl};
        use ros_launch_resolve::ros::manifest_loader::{ResolvedManifest, ResolvedTopic};

        // Build a manifest where `talker` is lifecycle=true and publishes
        // to /chatter at declared min_rate_hz=30.
        let mut nodes = std::collections::BTreeMap::new();
        let mut talker = NodeDecl {
            lifecycle: Some(true),
            ..Default::default()
        };
        let pub_props = ros_launch_manifest_types::EndpointProps {
            min_rate_hz: Some(ros_launch_manifest_types::spelled(
                ros_launch_manifest_types::field_table::Context::Endpoint,
                "min_rate_hz",
                30.0,
            )),
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
            channel: ros_launch_resolve::ros::manifest_loader::ContractChannel::Provider,
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
                derived_from_remaps: false,
                fqn: "/chatter".to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                publishers: vec!["/talker/chatter".to_string()],
                subscribers: vec![],
                rate_hz: None,
                derived_rate_hz: None,
                qos: None,
                drop: None,
                scope_ids: vec![0],
                on_silence_after_ms: None,
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
                cpu_ns: 0,
                tid: 0,
                payload_bytes: 0,
            });
        }
        re.flush();
        assert_eq!(
            re.violation_count, 0,
            "lifecycle gate should suppress rate-hierarchy violation"
        );

        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// Declares `/chatter` with `msg_type` (derived from remaps when empty),
    /// sends a publisher init with a type hash that matches nothing, and
    /// returns how many violations the engine counted and the violations file.
    fn consistency_run(msg_type: &str, tag: &str) -> (u64, String) {
        use crate::interception::{EventKind, InterceptionEvent};
        use ros_launch_resolve::ros::manifest_loader::ResolvedTopic;

        let mut index = ManifestIndex::default();
        let fqn = "/chatter".to_string();
        index.topics.insert(
            fqn.clone(),
            ResolvedTopic {
                derived_from_remaps: msg_type.is_empty(),
                fqn: fqn.clone(),
                msg_type: msg_type.to_string(),
                publishers: vec!["/talker/out".to_string()],
                subscribers: vec![],
                rate_hz: None,
                derived_rate_hz: None,
                qos: None,
                drop: None,
                scope_ids: vec![0],
                on_silence_after_ms: None,
            },
        );

        let tmp = std::env::temp_dir().join(format!(
            "play_launch_consistency_test_{tag}_{}",
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
            cpu_ns: 0,
            tid: 0,
            payload_bytes: 0,
        };
        re.observe(&event);
        re.flush();

        let count = re.violation_count;
        let contents =
            std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap_or_default();
        let _ = std::fs::remove_dir_all(&tmp);
        (count, contents)
    }

    #[test]
    fn consistency_runtime_fires_on_type_mismatch() {
        let (count, contents) = consistency_run("std_msgs/msg/String", "mismatch");
        assert!(count >= 1);
        assert!(contents.contains("consistency-runtime"), "{contents}");
    }

    #[test]
    fn consistency_runtime_ignores_a_topic_with_no_declared_type() {
        // #0079: a topic derived from remaps has an empty type; the engine
        // must not compare the runtime type against the empty string.
        let (_, contents) = consistency_run("", "untyped");
        assert!(!contents.contains("consistency-runtime"), "{contents}");
    }

    #[test]
    fn max_age_runtime_fires_when_stamp_too_old() {
        use crate::interception::{EventKind, InterceptionEvent};
        use ros_launch_manifest_types::{Manifest, NodeDecl};
        use ros_launch_resolve::ros::manifest_loader::{ResolvedManifest, ResolvedTopic};

        // Build manifest: subscriber declares max_age_ms=10.
        let mut nodes = std::collections::BTreeMap::new();
        let mut sub_node = NodeDecl::default();
        let sub_props = ros_launch_manifest_types::EndpointProps {
            max_age: Some(ros_launch_manifest_types::spelled(
                ros_launch_manifest_types::field_table::Context::Endpoint,
                "max_age",
                ros_launch_manifest_types::duration::Duration::from_millis_f64(10.0),
            )),
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
            channel: ros_launch_resolve::ros::manifest_loader::ContractChannel::Provider,
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
                derived_from_remaps: false,
                fqn: fqn.clone(),
                msg_type: "std_msgs/msg/String".to_string(),
                publishers: vec![],
                subscribers: vec!["/listener/in".to_string()],
                rate_hz: None,
                derived_rate_hz: None,
                qos: None,
                drop: None,
                scope_ids: vec![0],
                on_silence_after_ms: None,
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
            cpu_ns: 0,
            tid: 0,
            payload_bytes: 0,
        };
        re.observe(&event);
        re.flush();

        assert!(re.violation_count >= 1);
        let contents = std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap();
        assert!(contents.contains("max-age-runtime"), "{contents}");

        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// Issue #0032: strict mode trips on `Severity::Error` only. The
    /// `graph-deviation-runtime` WARNING that every node's `/rosout`
    /// publisher raises is logged and recorded but leaves the flag alone;
    /// the `rate-hierarchy-runtime` ERROR the contract was written for
    /// sets it.
    #[test]
    fn strict_mode_trips_on_error_severity_only() {
        use crate::interception::{EventKind, InterceptionEvent};
        use ros_launch_manifest_types::{Manifest, NodeDecl};
        use ros_launch_resolve::ros::manifest_loader::{ResolvedManifest, ResolvedTopic};

        // A plain (non-lifecycle) talker publishing /chatter with a declared
        // min_rate_hz of 30.
        let mut nodes = std::collections::BTreeMap::new();
        let mut talker = NodeDecl::default();
        talker.publishers.insert(
            "chatter".to_string(),
            ros_launch_manifest_types::EndpointProps {
                min_rate_hz: Some(ros_launch_manifest_types::spelled(
                    ros_launch_manifest_types::field_table::Context::Endpoint,
                    "min_rate_hz",
                    30.0,
                )),
                ..Default::default()
            },
        );
        nodes.insert("talker".to_string(), talker);
        let resolved = ResolvedManifest {
            scope_id: 0,
            pkg: None,
            file: String::new(),
            ns: "/".to_string(),
            channel: ros_launch_resolve::ros::manifest_loader::ContractChannel::Provider,
            contract_path: std::path::PathBuf::new(),
            manifest: Manifest {
                version: 1,
                nodes,
                ..Default::default()
            },
            source: String::new(),
            diagnostics: vec![],
        };
        let mut index = ManifestIndex::default();
        index.manifests.insert(0, resolved);
        index.topics.insert(
            "/chatter".to_string(),
            ResolvedTopic {
                derived_from_remaps: false,
                fqn: "/chatter".to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                publishers: vec!["/talker/chatter".to_string()],
                subscribers: vec![],
                rate_hz: None,
                derived_rate_hz: None,
                qos: None,
                drop: None,
                scope_ids: vec![0],
                on_silence_after_ms: None,
            },
        );

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

        // A graph deviation: an unknown topic's publisher is created.
        // Warning severity -- recorded, not fatal.
        re.observe(&InterceptionEvent {
            kind: EventKind::PublisherInit,
            _pad: [0; 3],
            topic_hash: 0xCAFE,
            stamp_sec: 0,
            stamp_nanosec: 0,
            handle: 0x1,
            monotonic_ns: 1_000_000,
            cpu_ns: 0,
            tid: 0,
            payload_bytes: 0,
        });
        assert_eq!(re.violation_count, 1, "the warning is still a violation");
        assert!(
            !handle.load(Ordering::Acquire),
            "a warning-severity violation must not end a strict run"
        );

        // The talker at 1 Hz for eight seconds: the rate check (every ~5 s
        // on a slow topic, over at least a 0.5 s window) finds 1 Hz < 30
        // and emits rate-hierarchy-runtime at Error severity.
        let topic_hash = fnv1a("/chatter".as_bytes());
        for i in 0..8u64 {
            re.observe(&InterceptionEvent {
                kind: EventKind::Publish,
                _pad: [0; 3],
                topic_hash,
                stamp_sec: 0,
                stamp_nanosec: 0,
                handle: 0xA,
                monotonic_ns: 2_000_000 + i * 1_000_000_000,
                cpu_ns: 0,
                tid: 0,
                payload_bytes: 0,
            });
        }
        re.flush();
        assert!(
            handle.load(Ordering::Acquire),
            "an error-severity violation must trip the strict flag"
        );
        assert_eq!(
            re.first_strict_violation_handle()
                .lock()
                .unwrap()
                .as_deref(),
            Some("rate-hierarchy-runtime on /chatter"),
            "the exit names the violation that tripped strict mode"
        );
        let contents = std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap();
        assert!(contents.contains("graph-deviation-runtime"), "{contents}");
        assert!(contents.contains("rate-hierarchy-runtime"), "{contents}");

        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// `--strict-on warning`: the other reading, as a threshold. The same
    /// graph deviation that the default leaves alone ends the run.
    #[test]
    fn strict_on_warning_makes_a_warning_fatal() {
        use crate::interception::{EventKind, InterceptionEvent};

        let index = ManifestIndex::default();
        let tmp = std::env::temp_dir().join(format!(
            "play_launch_strict_on_warning_test_{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(
            Arc::new(ContractView::from_manifest_index(&index)),
            EnforceMode::Strict,
            &tmp,
        )
        .with_strict_threshold(Severity::Warning);
        let handle = re.strict_violated_handle();
        re.observe(&InterceptionEvent {
            kind: EventKind::PublisherInit,
            _pad: [0; 3],
            topic_hash: 0xCAFE,
            stamp_sec: 0,
            stamp_nanosec: 0,
            handle: 0x1,
            monotonic_ns: 1_000_000,
            cpu_ns: 0,
            tid: 0,
            payload_bytes: 0,
        });
        assert!(handle.load(Ordering::Acquire));
        assert_eq!(
            re.first_strict_violation_handle()
                .lock()
                .unwrap()
                .as_deref(),
            Some("graph-deviation-runtime on (unknown hash 0xcafe)")
        );

        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// Issue #0032: `/rosout`, `/rosout_agg` and `/parameter_events` exist
    /// for every node before user code runs. They are not graph deviations,
    /// whatever the contract says.
    #[test]
    fn rcl_internal_topics_are_not_graph_deviations() {
        use crate::interception::{EventKind, InterceptionEvent};

        let index = ManifestIndex::default();
        let tmp = std::env::temp_dir().join(format!(
            "play_launch_rcl_internal_test_{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        let view = ContractView::from_manifest_index(&index);
        assert!(
            view.externals.is_empty(),
            "the contract view is not touched"
        );
        let mut re = RuleEngine::new(Arc::new(view), EnforceMode::Strict, &tmp);
        for fqn in RCL_INTERNAL_TOPICS {
            re.observe(&InterceptionEvent {
                kind: EventKind::PublisherInit,
                _pad: [0; 3],
                topic_hash: fnv1a(fqn.as_bytes()),
                stamp_sec: 0,
                stamp_nanosec: 0,
                handle: 0x1,
                monotonic_ns: 1_000_000,
                cpu_ns: 0,
                tid: 0,
                payload_bytes: 0,
            });
        }
        assert_eq!(
            re.violation_count, 0,
            "rcl-internal topics are implicitly external"
        );
        assert!(!re.strict_violated_handle().load(Ordering::Acquire));

        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// Phase 73: a guard that goes silent is detected on the tick, and the
    /// first sink publish afterwards WITHOUT the guard's provenance is the
    /// reaction. The sink's answer to the guard's last message — same stamp —
    /// is not.
    #[test]
    fn hazard_silence_is_detected_and_the_provenance_free_sink_publish_is_the_reaction() {
        use crate::{
            interception::{EventKind, InterceptionEvent},
            runtime_enforcement::view::{HazardWatch, TopicView},
        };
        let mut view = ContractView::default();
        for t in ["/safety/scan", "/safety/brake_cmd"] {
            view.topics.insert(
                t.to_string(),
                TopicView {
                    msg_type: "x".into(),
                    ..Default::default()
                },
            );
        }
        view.hazards.push(HazardWatch {
            key: "drive_blind".into(),
            guards: vec!["/safety/scan".into()],
            sinks: vec!["/safety/brake_cmd".into()],
            ftti_ms: Some(500.0),
            settle_ms: Some(200.0),
            silence_ms: Some(100.0),
            via_diagnostics: false,
            off_host_sinks: vec![],
        });
        let tmp =
            std::env::temp_dir().join(format!("play_launch_hazard_rt_{}", std::process::id()));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(Arc::new(view), EnforceMode::Warn, &tmp);
        let scan = fnv1a(b"/safety/scan");
        let brake = fnv1a(b"/safety/brake_cmd");
        let ev = |kind: EventKind, h: u64, t: u64, stamp: u32| InterceptionEvent {
            kind,
            _pad: [0; 3],
            topic_hash: h,
            stamp_sec: 1,
            stamp_nanosec: stamp,
            handle: 1,
            monotonic_ns: t,
            cpu_ns: 0,
            tid: 1,
            payload_bytes: 0,
        };
        // 20ms cadence; the brake answers each scan 7ms later with its stamp.
        for i in 0..20u64 {
            let t = 1_000_000_000 + i * 20_000_000;
            re.observe(&ev(EventKind::Publish, scan, t, 100 + i as u32));
            re.observe(&ev(
                EventKind::Publish,
                brake,
                t + 7_000_000,
                100 + i as u32,
            ));
        }
        let last = 1_000_000_000 + 19 * 20_000_000;
        re.tick(last + 50_000_000);
        assert_eq!(
            re.violation_count, 0,
            "50ms of silence is under the 100ms threshold"
        );
        // The node's own watchdog reacts at 80ms — BEFORE the observer's
        // threshold. The provenance-free publish is buffered.
        re.observe(&ev(EventKind::Publish, brake, last + 80_000_000, 999_999));
        assert_eq!(re.violation_count, 0, "no fault is known yet");
        re.tick(last + 120_000_000);
        assert_eq!(
            re.violation_count, 2,
            "hazard-detected on the tick, and the buffered reaction with it"
        );
        // The brake's answer to the LAST scan (same stamp) is never a reaction.
        re.observe(&ev(EventKind::Publish, brake, last + 125_000_000, 119));
        assert_eq!(re.violation_count, 2);
        re.flush();
        let log = std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap();
        assert!(log.contains("hazard-detected"), "{log}");
        assert!(log.contains("hazard-reaction"), "{log}");
        assert!(log.contains("80.00ms after"), "{log}");
        assert!(log.contains("fits ftti 500.00ms"), "{log}");
        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// Phase 83 F2: the island publishes its braking command on an MCU, so
    /// the host never sees the sink PUBLISHED -- only vehicle_cmd_gate's
    /// TAKE of it. That take is the reaction, labelled `observed_at: take`,
    /// where it used to be no reaction at all.
    #[test]
    fn an_off_host_sink_is_observed_at_its_first_host_take() {
        use crate::{
            interception::{EventKind, InterceptionEvent},
            runtime_enforcement::view::{HazardWatch, TopicView},
        };
        let mut view = ContractView::default();
        for t in [
            "/system/operation_mode/availability",
            "/system/emergency/control_cmd",
        ] {
            view.topics.insert(
                t.to_string(),
                TopicView {
                    msg_type: "x".into(),
                    ..Default::default()
                },
            );
        }
        view.hazards.push(HazardWatch {
            key: "hpc_loss".into(),
            guards: vec!["/system/operation_mode/availability".into()],
            sinks: vec!["/system/emergency/control_cmd".into()],
            ftti_ms: Some(10_000.0),
            settle_ms: Some(4165.33),
            silence_ms: Some(500.0),
            via_diagnostics: false,
            off_host_sinks: vec![],
        });
        let tmp =
            std::env::temp_dir().join(format!("play_launch_hazard_take_{}", std::process::id()));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(Arc::new(view), EnforceMode::Warn, &tmp);
        let avail = fnv1a(b"/system/operation_mode/availability");
        let cmd = fnv1a(b"/system/emergency/control_cmd");
        let ev = |kind: EventKind, h: u64, t: u64, stamp: u32| InterceptionEvent {
            kind,
            _pad: [0; 3],
            topic_hash: h,
            stamp_sec: 1,
            stamp_nanosec: stamp,
            handle: 1,
            monotonic_ns: t,
            cpu_ns: 0,
            tid: 1,
            payload_bytes: 0,
        };
        for i in 0..20u64 {
            re.observe(&ev(
                EventKind::Publish,
                avail,
                1_000_000_000 + i * 100_000_000,
                100 + i as u32,
            ));
        }
        let last = 1_000_000_000 + 19 * 100_000_000;
        re.tick(last + 520_000_000);
        assert_eq!(re.violation_count, 1, "hazard-detected on the tick");
        // The gate takes the island's first braking command 589ms after the
        // last availability sample -- phase 8's board measurement.
        re.observe(&ev(EventKind::Take, cmd, last + 589_000_000, 7_777));
        assert_eq!(re.violation_count, 2, "the take is the reaction");
        re.flush();
        let log = std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap();
        let reaction = log
            .lines()
            .find(|l| l.contains("hazard-reaction"))
            .expect("a reaction line");
        assert!(reaction.contains("589.00ms after"), "{reaction}");
        assert!(reaction.contains(r#""observed_at":"take""#), "{reaction}");
        assert!(reaction.contains("link hop"), "{reaction}");
        // Every other line is unchanged: no observed_at key.
        let detected = log.lines().find(|l| l.contains("hazard-detected")).unwrap();
        assert!(!detected.contains("observed_at"), "{detected}");
        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// And a sink a host process publishes is never judged by its takes.
    #[test]
    fn a_host_published_sink_is_not_judged_by_its_takes() {
        use crate::{
            interception::{EventKind, InterceptionEvent},
            runtime_enforcement::view::{HazardWatch, TopicView},
        };
        let mut view = ContractView::default();
        for t in ["/scan", "/brake"] {
            view.topics.insert(
                t.to_string(),
                TopicView {
                    msg_type: "x".into(),
                    ..Default::default()
                },
            );
        }
        view.hazards.push(HazardWatch {
            key: "blind".into(),
            guards: vec!["/scan".into()],
            sinks: vec!["/brake".into()],
            ftti_ms: Some(500.0),
            settle_ms: None,
            silence_ms: Some(100.0),
            via_diagnostics: false,
            off_host_sinks: vec![],
        });
        let tmp =
            std::env::temp_dir().join(format!("play_launch_hazard_host_{}", std::process::id()));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(Arc::new(view), EnforceMode::Warn, &tmp);
        let (scan, brake) = (fnv1a(b"/scan"), fnv1a(b"/brake"));
        let ev = |kind: EventKind, h: u64, t: u64, stamp: u32| InterceptionEvent {
            kind,
            _pad: [0; 3],
            topic_hash: h,
            stamp_sec: 1,
            stamp_nanosec: stamp,
            handle: 1,
            monotonic_ns: t,
            cpu_ns: 0,
            tid: 1,
            payload_bytes: 0,
        };
        for i in 0..20u64 {
            let t = 1_000_000_000 + i * 20_000_000;
            re.observe(&ev(EventKind::Publish, scan, t, 100 + i as u32));
            re.observe(&ev(
                EventKind::Publish,
                brake,
                t + 5_000_000,
                100 + i as u32,
            ));
        }
        let last = 1_000_000_000 + 19 * 20_000_000;
        re.tick(last + 120_000_000);
        assert_eq!(re.violation_count, 1);
        re.observe(&ev(EventKind::Take, brake, last + 130_000_000, 5_555));
        assert_eq!(
            re.violation_count, 1,
            "a take of a host-published sink is not the reaction"
        );
        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// Phase 74: a DDS liveliness event on a guard is the detection —
    /// exact, from the middleware, before the observer's own threshold.
    #[test]
    fn a_dds_liveliness_event_on_a_guard_is_the_detection() {
        use crate::{
            interception::{EventKind, InterceptionEvent},
            runtime_enforcement::view::{HazardWatch, TopicView},
        };
        let mut view = ContractView::default();
        view.topics.insert(
            "/safety/scan".to_string(),
            TopicView {
                msg_type: "x".into(),
                ..Default::default()
            },
        );
        view.hazards.push(HazardWatch {
            key: "drive_blind".into(),
            guards: vec!["/safety/scan".into()],
            sinks: vec![],
            ftti_ms: Some(500.0),
            settle_ms: None,
            silence_ms: Some(100.0),
            via_diagnostics: false,
            off_host_sinks: vec![],
        });
        let tmp =
            std::env::temp_dir().join(format!("play_launch_hazard_dds_{}", std::process::id()));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(Arc::new(view), EnforceMode::Warn, &tmp);
        let scan = fnv1a(b"/safety/scan");
        let publish = |t: u64| InterceptionEvent {
            kind: EventKind::Publish,
            _pad: [0; 3],
            topic_hash: scan,
            stamp_sec: 1,
            stamp_nanosec: 7,
            handle: 1,
            monotonic_ns: t,
            cpu_ns: 0,
            tid: 1,
            payload_bytes: 0,
        };
        re.observe(&publish(1_000_000_000));
        re.observe(&publish(1_020_000_000));
        // not_alive_change = +1 rides `_pad[1]`.
        re.observe(&InterceptionEvent {
            kind: EventKind::LivelinessChanged,
            _pad: [0xff, 1, 0], // alive -1, not_alive +1
            topic_hash: scan,
            stamp_sec: 0,
            stamp_nanosec: 1,
            handle: 1,
            monotonic_ns: 1_119_000_000,
            cpu_ns: 0,
            tid: 1,
            payload_bytes: 0,
        });
        // Two lines for one event: phase 36's `liveliness-runtime` (the DDS
        // fact) and phase 74's `hazard-detected` (what it means).
        assert_eq!(re.violation_count, 2);
        re.flush();
        let log = std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap();
        assert!(
            log.contains("reported LivelinessChanged by DDS 99.00ms"),
            "{log}"
        );
        // The tick does not report it a second time.
        re.tick(1_300_000_000);
        assert_eq!(re.violation_count, 2);
        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// Phase 75: a silent guard makes its function lost, which makes every
    /// mode requiring it lost, and the ladder names the rung that is still
    /// available. Reported once per transition, and cleared on recovery.
    #[test]
    fn a_lost_function_loses_its_modes_and_names_the_rung() {
        use crate::{
            interception::{EventKind, InterceptionEvent},
            runtime_enforcement::view::{HazardWatch, ModeWatch, TopicView},
        };
        let mut view = ContractView::default();
        view.topics.insert(
            "/safety/scan".to_string(),
            TopicView {
                msg_type: "x".into(),
                ..Default::default()
            },
        );
        view.hazards.push(HazardWatch {
            key: "drive_blind".into(),
            guards: vec!["/safety/scan".into()],
            sinks: vec![],
            ftti_ms: Some(500.0),
            settle_ms: None,
            silence_ms: Some(100.0),
            via_diagnostics: false,
            off_host_sinks: vec![],
        });
        view.modes.push(ModeWatch {
            key: "driving".into(),
            requires: vec![crate::runtime_enforcement::view::RequiredFn::topics(vec![
                "/safety/scan".into(),
            ])],
            fallback: vec!["stopped".into()],
            ..Default::default()
        });
        view.modes.push(ModeWatch {
            key: "stopped".into(),
            ..Default::default()
        });
        let tmp = std::env::temp_dir().join(format!("play_launch_modes_{}", std::process::id()));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(Arc::new(view), EnforceMode::Warn, &tmp);
        let scan = fnv1a(b"/safety/scan");
        let pubev = |t: u64, stamp: u32| InterceptionEvent {
            kind: EventKind::Publish,
            _pad: [0; 3],
            topic_hash: scan,
            stamp_sec: 1,
            stamp_nanosec: stamp,
            handle: 1,
            monotonic_ns: t,
            cpu_ns: 0,
            tid: 1,
            payload_bytes: 0,
        };
        for i in 0..10u64 {
            re.observe(&pubev(1_000_000_000 + i * 20_000_000, 100 + i as u32));
        }
        let last = 1_000_000_000 + 9 * 20_000_000;
        assert_eq!(re.violation_count, 0);
        re.tick(last + 150_000_000);
        // hazard-detected + mode-availability for `driving`. `stopped`
        // requires nothing, so it is never lost and never reported.
        assert_eq!(re.violation_count, 2);
        re.flush();
        let log = std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap();
        assert!(
            log.contains("mode 'driving': LOST — falling to 'stopped'"),
            "{log}"
        );
        assert!(
            !log.contains("mode 'stopped'"),
            "a mode requiring nothing is never lost: {log}"
        );
        // Idempotent: a second tick reports nothing new.
        re.tick(last + 300_000_000);
        assert_eq!(re.violation_count, 2);
        // The guard resumes: hazard-recovered + mode available again.
        re.observe(&pubev(last + 400_000_000, 999));
        assert_eq!(re.violation_count, 4);
        re.flush();
        let log = std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap();
        assert!(log.contains("mode 'driving': available again"), "{log}");
        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// Phase 102 M2: a value function lost by value takes its mode down;
    /// the level it falls to commands `/cmd linear.x == 0` from its derived
    /// bound (10 ms) on. A message before the bound is not judged, one after
    /// it that breaks the predicate is `safe-state-violated` (once per
    /// entry), and silence with no declared consumer behaviour is
    /// `final-element-silent`. The mode coming back leaves the level.
    #[test]
    fn a_level_holds_its_commands_from_its_bound() {
        use crate::{
            interception::{EventKind, InterceptionEvent},
            runtime_enforcement::view::{ModeWatch, RequiredFn, TopicView, ValueFn},
        };
        use ros_launch_manifest_model::{CompareOp, PredicateContract, PredicateValueContract};
        let mut view = ContractView::default();
        for t in ["/avail", "/cmd"] {
            view.topics.insert(
                t.to_string(),
                TopicView {
                    msg_type: "x".into(),
                    ..Default::default()
                },
            );
        }
        view.fields = vec![
            ("/avail".into(), "ok".into()),
            ("/cmd".into(), "linear.x".into()),
        ];
        view.value_fns.insert(
            "s/avail_ok".into(),
            ValueFn {
                topics: vec!["/avail".into()],
                when: PredicateContract {
                    field: "ok".into(),
                    op: CompareOp::Equals,
                    value: PredicateValueContract::Bool(false),
                },
            },
        );
        view.modes.push(ModeWatch {
            key: "s/driving".into(),
            requires: vec![RequiredFn {
                members: vec!["/avail".into()],
                value_fn: Some("s/avail_ok".into()),
                ..Default::default()
            }],
            fallback: vec!["s/stopped".into()],
            ..Default::default()
        });
        let mut commands = std::collections::BTreeMap::new();
        commands.insert(
            "/cmd".to_string(),
            vec![PredicateContract {
                field: "linear.x".into(),
                op: CompareOp::Equals,
                value: PredicateValueContract::Int(0),
            }],
        );
        view.modes.push(ModeWatch {
            key: "s/stopped".into(),
            commands,
            bound_ms: Some(10.0),
            ..Default::default()
        });
        let tmp = std::env::temp_dir().join(format!("play_launch_levels_{}", std::process::id()));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(Arc::new(view), EnforceMode::Warn, &tmp);
        let field = |t: u64, index: u8, kind: u8, bits: u64| InterceptionEvent {
            kind: EventKind::FieldValue,
            _pad: [index, kind, 0],
            topic_hash: 0,
            stamp_sec: 0,
            stamp_nanosec: 0,
            handle: 1,
            monotonic_ns: t,
            cpu_ns: bits,
            tid: 0,
            payload_bytes: 0,
        };
        let ms = 1_000_000u64;
        let t0 = 1_000 * ms;
        re.observe(&field(t0, 0, 4, 1));
        assert_eq!(re.violation_count, 0, "available while ok is true");
        re.observe(&field(t0 + ms, 0, 4, 0));
        assert_eq!(re.violation_count, 1, "mode-availability: driving lost");
        // Before the bound a moving command is not judged.
        re.observe(&field(t0 + 5 * ms, 1, 1, 0.5f64.to_bits()));
        assert_eq!(re.violation_count, 1);
        // After it, it is, once per entry; a stopped command is fine.
        re.observe(&field(t0 + 20 * ms, 1, 1, 0.5f64.to_bits()));
        re.observe(&field(t0 + 25 * ms, 1, 1, 0.5f64.to_bits()));
        re.observe(&field(t0 + 30 * ms, 1, 1, 0.0f64.to_bits()));
        assert_eq!(re.violation_count, 2);
        // Silent for longer than 1 s (no cadence known), nothing declared.
        re.tick(t0 + 2_000 * ms);
        assert_eq!(re.violation_count, 3);
        re.flush();
        let log = std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap();
        assert!(log.contains("safe-state-violated"), "{log}");
        assert!(
            log.contains("/cmd linear.x == 0, and a message carried 0.5"),
            "{log}"
        );
        assert!(log.contains("final-element-silent"), "{log}");
        // Back to ok: the mode is available and the level left.
        re.observe(&field(t0 + 2_100 * ms, 0, 4, 1));
        assert_eq!(re.violation_count, 4, "mode-availability: available again");
        re.observe(&field(t0 + 2_200 * ms, 1, 1, 0.5f64.to_bits()));
        assert_eq!(re.violation_count, 4, "no level active, nothing judged");
        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// Phase 102 M2: a publisher created on a commanded topic by any node
    /// but its arbiter is `arbiter-bypass`; on a monitored topic whose
    /// hazard names `insertion`, a node the contract does not name is that
    /// fault; and a stamp equal to the same publisher's previous one is
    /// `repetition`. Each reported once.
    #[test]
    fn publishers_and_stamps_are_judged_where_the_contract_asks() {
        use crate::{
            interception::{EventKind, InterceptionEvent},
            runtime_enforcement::view::{HazardWatch, ModeWatch, TopicView},
        };
        use ros_launch_manifest_model::{
            CompareOp, FaultKind, PredicateContract, PredicateValueContract,
        };
        let mut view = ContractView::default();
        view.topics.insert(
            "/cmd".to_string(),
            TopicView {
                msg_type: "x".into(),
                publishers: vec!["/gate/cmd".into()],
                ..Default::default()
            },
        );
        view.topics.insert(
            "/scan".to_string(),
            TopicView {
                msg_type: "x".into(),
                publishers: vec!["/lidar/scan".into()],
                ..Default::default()
            },
        );
        let mut commands = std::collections::BTreeMap::new();
        commands.insert(
            "/cmd".to_string(),
            vec![PredicateContract {
                field: "x".into(),
                op: CompareOp::Equals,
                value: PredicateValueContract::Int(0),
            }],
        );
        view.modes.push(ModeWatch {
            key: "s/stopped".into(),
            commands,
            ..Default::default()
        });
        view.hazards.push(HazardWatch {
            key: "s/blind".into(),
            guards: vec!["/scan".into()],
            sinks: vec![],
            ftti_ms: None,
            settle_ms: None,
            silence_ms: None,
            via_diagnostics: false,
            off_host_sinks: vec![],
        });
        view.hazard_classes.insert(
            "s/blind".into(),
            vec![FaultKind::Insertion, FaultKind::Repetition],
        );
        // Phase 102 E2: the same system, with resequencing asked for.
        let mut reseq_view = view.clone();
        reseq_view
            .hazard_classes
            .insert("s/blind".into(), vec![FaultKind::Resequencing]);
        let tmp = std::env::temp_dir().join(format!("play_launch_pubs_{}", std::process::id()));
        std::fs::create_dir_all(&tmp).unwrap();
        let mut re = RuleEngine::new(Arc::new(view), EnforceMode::Warn, &tmp);
        let ev = |kind: EventKind, topic: u64, handle: u64, stamp: u32| InterceptionEvent {
            kind,
            _pad: [0; 3],
            topic_hash: topic,
            stamp_sec: if matches!(kind, EventKind::Publish) {
                1
            } else {
                0
            },
            stamp_nanosec: stamp,
            handle,
            monotonic_ns: 1_000_000,
            cpu_ns: 0,
            tid: 0,
            payload_bytes: 0,
        };
        let (cmd, scan) = (fnv1a(b"/cmd"), fnv1a(b"/scan"));
        let owned = |re: &mut RuleEngine, node: &str, topic: u64, handle: u64| {
            re.observe(&ev(
                EventKind::EndpointOwner,
                fnv1a(node.as_bytes()),
                handle,
                0,
            ));
            re.observe(&ev(EventKind::PublisherInit, topic, handle, 0));
        };
        owned(&mut re, "/gate", cmd, 1);
        owned(&mut re, "/lidar", scan, 2);
        assert_eq!(re.violation_count, 0, "the declared publishers");
        owned(&mut re, "/teleop", cmd, 3);
        owned(&mut re, "/teleop", cmd, 4);
        assert_eq!(re.violation_count, 1, "arbiter-bypass, once");
        owned(&mut re, "/spoof", scan, 5);
        assert_eq!(re.violation_count, 2, "insertion");
        re.observe(&ev(EventKind::Publish, scan, 2, 10));
        re.observe(&ev(EventKind::Publish, scan, 2, 20));
        assert_eq!(re.violation_count, 2, "advancing stamps");
        re.observe(&ev(EventKind::Publish, scan, 2, 20));
        re.observe(&ev(EventKind::Publish, scan, 2, 20));
        assert_eq!(re.violation_count, 3, "repetition, once");
        re.observe(&ev(EventKind::Publish, scan, 2, 5));
        assert_eq!(re.violation_count, 3, "resequencing is not asked for");
        re.flush();
        let log = std::fs::read_to_string(tmp.join("runtime_violations.jsonl")).unwrap();
        assert!(log.contains("arbiter-bypass"), "{log}");
        assert!(log.contains("(insertion; declared: /lidar/scan)"), "{log}");
        assert!(log.contains("carried a repetition"), "{log}");
        let _ = std::fs::remove_dir_all(&tmp);

        // A stamp earlier than the publisher's previous one is resequencing
        // where the hazard names it; a repeated stamp is not judged there.
        let tmp2 = std::env::temp_dir().join(format!("play_launch_reseq_{}", std::process::id()));
        std::fs::create_dir_all(&tmp2).unwrap();
        let mut re = RuleEngine::new(Arc::new(reseq_view), EnforceMode::Warn, &tmp2);
        owned(&mut re, "/lidar", scan, 2);
        re.observe(&ev(EventKind::Publish, scan, 2, 10));
        re.observe(&ev(EventKind::Publish, scan, 2, 20));
        re.observe(&ev(EventKind::Publish, scan, 2, 20));
        assert_eq!(re.violation_count, 0, "repetition is not asked for");
        re.observe(&ev(EventKind::Publish, scan, 2, 5));
        assert_eq!(re.violation_count, 1, "resequencing");
        re.flush();
        let log = std::fs::read_to_string(tmp2.join("runtime_violations.jsonl")).unwrap();
        assert!(log.contains("resequencing"), "{log}");
        let _ = std::fs::remove_dir_all(&tmp2);
    }
}
