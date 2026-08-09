//! Core manifest types.

use serde::Serialize;
use std::collections::BTreeMap;

/// Argument declaration with optional type constraint.
#[derive(Debug, Clone, Default, Serialize)]
pub enum ArgDecl {
    /// Free string — no constraint (default).
    #[default]
    String,
    /// Boolean — only "true" or "false".
    Bool,
    /// Enum — one of the listed values.
    Choices(Vec<String>),
}

impl ArgDecl {
    /// Valid values for this arg type. None = unconstrained (free string).
    pub fn valid_values(&self) -> Option<Vec<&str>> {
        match self {
            ArgDecl::String => None,
            ArgDecl::Bool => Some(vec!["true", "false"]),
            ArgDecl::Choices(choices) => Some(choices.iter().map(|s| s.as_str()).collect()),
        }
    }
}

/// Top-level manifest.
#[derive(Debug, Clone, Default, Serialize)]
pub struct Manifest {
    pub version: u32,
    /// Manifest arguments — all mandatory. Values provided by scope args from record.json.
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub args: BTreeMap<String, ArgDecl>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub exclude_patterns: Vec<String>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub nodes: BTreeMap<String, NodeDecl>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub topics: BTreeMap<String, TopicDecl>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub services: BTreeMap<String, ServiceDecl>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub actions: BTreeMap<String, ActionDecl>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub includes: BTreeMap<String, IncludeDecl>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub paths: BTreeMap<String, PathDecl>,
    /// Topics this manifest's subtree consumes from / produces to systems
    /// outside the loaded manifest tree (Issue #51). Each entry marks the
    /// listed FQN as expected-external on the named side; the
    /// `dangling-entity` rule skips that side when it would otherwise
    /// fire. An internal `pub:`/`sub:` declaration anywhere in the merged
    /// tree overrides the external mark.
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub external_topics: BTreeMap<String, ExternalTopicDecl>,
    /// Named cross-scope compositions of `paths:` into end-to-end budgets
    /// (Vocabulary v2, Phase 44.1 §4). Integrator-owned — root contract or
    /// overlay, same owner as the platform file. Cross-file link
    /// resolution (`chain-link` rule) is a checker concern (W2); parsing
    /// only validates local segment shape.
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub chains: BTreeMap<String, ChainDecl>,
}

/// Node declaration.
#[derive(Debug, Clone, Default, Serialize)]
pub struct NodeDecl {
    #[serde(rename = "if", skip_serializing_if = "Option::is_none")]
    pub if_condition: Option<String>,
    #[serde(rename = "unless", skip_serializing_if = "Option::is_none")]
    pub unless_condition: Option<String>,
    /// True if this is a ROS 2 lifecycle (managed) node. Rate, latency,
    /// and age contracts on this node's endpoints are runtime-gated on
    /// the node being in the Active state — publishers don't publish
    /// and subscribers don't run callbacks in Unconfigured/Inactive.
    /// Static checking is unaffected; this is a signal for runtime
    /// monitors to skip checks until the node is Active.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lifecycle: Option<bool>,
    #[serde(rename = "pub", skip_serializing_if = "BTreeMap::is_empty")]
    pub publishers: BTreeMap<String, EndpointProps>,
    #[serde(rename = "sub", skip_serializing_if = "BTreeMap::is_empty")]
    pub subscribers: BTreeMap<String, EndpointProps>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub srv: BTreeMap<String, SrvEndpointProps>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub cli: BTreeMap<String, EndpointProps>,
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub paths: BTreeMap<String, PathDecl>,
    /// Mapper hint (Phase 41, RT config v2 design §2.1): platform-agnostic
    /// scheduling criticality — `high` | `medium` | `low`. No priority
    /// numbers; a `SchedMapper` (`play_launch`'s derive pipeline) may use
    /// this as a tie-break or additional signal. Absent/unrecognized values
    /// are ignored (mapper defaults the node non-RT), never a parse error —
    /// this field is advisory, not schema-enforced.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub criticality: Option<String>,
}

/// Publisher/subscriber endpoint properties (all optional).
#[derive(Debug, Clone, Default, Serialize)]
pub struct EndpointProps {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_rate_hz: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_rate_hz: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub jitter_ms: Option<f64>,
    /// Sub endpoint: max data age at receive (now - header.stamp), runtime-checked.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_age_ms: Option<f64>,
    /// Sub endpoint: read-latest, not causal.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub state: Option<bool>,
    /// Sub endpoint: must receive at least once before operational.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub required: Option<bool>,
    /// Per-endpoint QoS override. Field-level merge with topic-level
    /// `qos:` — fields declared here override the topic default; fields
    /// not declared here inherit from the topic.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub qos: Option<QosDecl>,
    /// Sub endpoint: per-subscriber transport latency override (ms).
    /// Used as the edge weight from publisher to this subscriber in
    /// scope-path critical-path computation. Inherits topic-level
    /// `max_transport_ms` when not declared.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_transport_ms: Option<f64>,
    /// Sub endpoint: buffering discipline for a `state: true` subscription
    /// (Vocabulary v2, Phase 44.1 §3). `latest` (default) — staleness is
    /// the failure mode; `queue` — backlog is the failure mode, drained
    /// batch-wise by the consuming timer. Only meaningful alongside
    /// `state: true`; parse-time error otherwise.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub buffer: Option<Buffer>,
}

/// Buffering discipline for a `state: true` subscriber endpoint
/// (Vocabulary v2, Phase 44.1 §3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Buffer {
    /// Read-latest; staleness is the failure mode (today's implicit
    /// semantics — the default when `buffer:` is unset).
    Latest,
    /// Bounded queue, drained batch-wise by the consuming timer callback;
    /// backlog is the failure mode.
    Queue,
}

/// Service endpoint properties.
#[derive(Debug, Clone, Default, Serialize)]
pub struct SrvEndpointProps {
    /// Max time from request to response (runtime monitoring only).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_response_ms: Option<f64>,
}

/// Topic declaration.
#[derive(Debug, Clone, Serialize)]
pub struct TopicDecl {
    #[serde(rename = "if", skip_serializing_if = "Option::is_none")]
    pub if_condition: Option<String>,
    #[serde(rename = "unless", skip_serializing_if = "Option::is_none")]
    pub unless_condition: Option<String>,
    #[serde(rename = "type")]
    pub msg_type: String,
    #[serde(rename = "pub", skip_serializing_if = "Vec::is_empty")]
    pub publishers: Vec<String>,
    #[serde(rename = "sub", skip_serializing_if = "Vec::is_empty")]
    pub subscribers: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub qos: Option<QosDecl>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rate_hz: Option<f64>,
    /// Worst-case transport latency (ms) for this topic hop.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_transport_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub drop: Option<DropSpec>,
    /// Per-topic external override. When set, the matching side
    /// (`pub`/`sub`/`both`) is treated as expected-external — the
    /// `dangling-entity` rule skips it. Equivalent to a
    /// per-topic-scoped entry in the manifest's `external_topics:` block.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub external: Option<ExternalSide>,
}

/// Side of a topic that is provided/consumed by an external system.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum ExternalSide {
    /// Producer is external. The local manifest tree may declare
    /// subscribers but no internal publisher is required.
    Pub,
    /// Consumer is external. The local manifest tree may declare
    /// publishers but no internal subscriber is required.
    Sub,
    /// Both producer and consumer are external — passthrough we don't
    /// model on either side.
    Both,
}

/// External-topic declaration (Issue #51). Marks an FQN as
/// expected-external from the declaring scope's subtree.
#[derive(Debug, Clone, Serialize)]
pub struct ExternalTopicDecl {
    /// Side that is externally provided. Required.
    #[serde(rename = "external")]
    pub side: ExternalSide,
    /// Optional ROS message type. When set, cross-checked against any
    /// internal `topics:` declaration of the same FQN by the
    /// `consistency` rule.
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub msg_type: Option<String>,
    /// Optional QoS profile (offered by the external producer or
    /// requested by the external consumer). Cross-checked under the
    /// `qos-match` rule.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub qos: Option<QosDecl>,
}

/// Service declaration.
#[derive(Debug, Clone, Serialize)]
pub struct ServiceDecl {
    #[serde(rename = "if", skip_serializing_if = "Option::is_none")]
    pub if_condition: Option<String>,
    #[serde(rename = "unless", skip_serializing_if = "Option::is_none")]
    pub unless_condition: Option<String>,
    #[serde(rename = "type")]
    pub srv_type: String,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub server: Vec<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub client: Vec<String>,
}

/// Action declaration.
#[derive(Debug, Clone, Serialize)]
pub struct ActionDecl {
    #[serde(rename = "if", skip_serializing_if = "Option::is_none")]
    pub if_condition: Option<String>,
    #[serde(rename = "unless", skip_serializing_if = "Option::is_none")]
    pub unless_condition: Option<String>,
    #[serde(rename = "type")]
    pub action_type: String,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub server: Vec<String>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub client: Vec<String>,
}

/// Include declaration (external manifest or inline scope).
#[derive(Debug, Clone, Serialize)]
pub enum IncludeDecl {
    /// External: loaded from separate manifest file.
    External { manifest: String },
    /// Inline: embedded manifest (from <group> block).
    Inline(Box<Manifest>),
}

/// QoS declaration.
#[derive(Debug, Clone, Default, Serialize)]
pub struct QosDecl {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reliability: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub durability: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub depth: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub history: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lifespan_ms: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub liveliness: Option<String>,
}

impl QosDecl {
    /// Compute the effective QoS for an endpoint by overlaying the
    /// endpoint's per-field overrides on top of the topic-level default.
    /// Each field follows: `endpoint.<f> ?? topic.<f>`. Fields unspecified
    /// on both sides remain `None` and are skipped by `qos-match`.
    pub fn effective(topic: Option<&QosDecl>, endpoint: Option<&QosDecl>) -> QosDecl {
        let pick_str = |f: fn(&QosDecl) -> Option<&String>| -> Option<String> {
            endpoint.and_then(f).or_else(|| topic.and_then(f)).cloned()
        };
        QosDecl {
            reliability: pick_str(|q| q.reliability.as_ref()),
            durability: pick_str(|q| q.durability.as_ref()),
            depth: endpoint
                .and_then(|q| q.depth)
                .or_else(|| topic.and_then(|q| q.depth)),
            history: pick_str(|q| q.history.as_ref()),
            lifespan_ms: endpoint
                .and_then(|q| q.lifespan_ms)
                .or_else(|| topic.and_then(|q| q.lifespan_ms)),
            liveliness: pick_str(|q| q.liveliness.as_ref()),
        }
    }
}

/// Named causal path.
#[derive(Debug, Clone, Default, Serialize)]
pub struct PathDecl {
    #[serde(rename = "if", skip_serializing_if = "Option::is_none")]
    pub if_condition: Option<String>,
    #[serde(rename = "unless", skip_serializing_if = "Option::is_none")]
    pub unless_condition: Option<String>,
    /// Node paths: endpoint names. Scope paths: topic names (relative/absolute).
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub input: Vec<String>,
    /// Node paths: endpoint names. Scope paths: topic names (relative/absolute).
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub output: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_latency_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tolerance_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub drop: Option<DropSpec>,
    /// Explicit closed-taxonomy cause of this path's output (Vocabulary
    /// v2, Phase 44.1 §1). When absent, [`PathDecl::effective_trigger`]
    /// applies the legacy derivation from `input:`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trigger: Option<Trigger>,
    /// Fan-in synchronization policy for an input-triggered path with
    /// ≥2 endpoints (Vocabulary v2, Phase 44.1 §2).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sync: Option<Sync>,
}

impl PathDecl {
    /// Derive the trigger actually in effect for this path (Vocabulary
    /// v2, Phase 44.1 §1): an explicit `trigger:` always wins; otherwise
    /// a non-empty legacy `input:` list derives an input trigger (today's
    /// 75+ contracts parse identically under this rule); otherwise the
    /// path is [`EffectiveTrigger::Unclassified`] — no trigger fact is
    /// derived, never silently assumed to be a timer.
    pub fn effective_trigger(&self) -> EffectiveTrigger {
        if let Some(trigger) = &self.trigger {
            return match trigger {
                Trigger::Timer { rate_hz } => EffectiveTrigger::Timer { rate_hz: *rate_hz },
                Trigger::Input(endpoints) => EffectiveTrigger::Input(endpoints.clone()),
                Trigger::Once => EffectiveTrigger::Once,
                Trigger::Spontaneous => EffectiveTrigger::Spontaneous,
            };
        }
        if !self.input.is_empty() {
            return EffectiveTrigger::Input(self.input.clone());
        }
        EffectiveTrigger::Unclassified
    }
}

/// Explicit closed taxonomy of what causes a path's output (Vocabulary
/// v2, Phase 44.1 §1). YAML forms:
///
/// ```yaml
/// trigger: { timer: { rate_hz: 10 } }
/// trigger: { input: [a, b] }
/// trigger: once
/// trigger: spontaneous
/// ```
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Trigger {
    /// Periodic self-clocked callback. `rate_hz` is a REAL scheduling
    /// input (mapper fact); must be > 0.
    Timer { rate_hz: f64 },
    /// Output caused by these input endpoint/topic names. Rate is
    /// inherited from upstream, not authored here.
    Input(Vec<String>),
    /// Published once (startup latch); scheduling-irrelevant.
    Once,
    /// Caused outside the graph (operator, network, hardware); event-like,
    /// no upstream inheritance.
    Spontaneous,
}

/// The trigger actually in effect for a path — either the explicit
/// [`Trigger`] or the legacy-derived / unclassified fallback (Vocabulary
/// v2, Phase 44.1 §1). See [`PathDecl::effective_trigger`].
#[derive(Debug, Clone, PartialEq)]
pub enum EffectiveTrigger {
    Timer {
        rate_hz: f64,
    },
    Input(Vec<String>),
    Once,
    Spontaneous,
    /// No explicit `trigger:` and no (non-empty) legacy `input:` list —
    /// trigger facts are not derivable. Never silently assumed to be a
    /// timer.
    Unclassified,
}

/// Fan-in synchronization policy on an input-triggered path with ≥2
/// endpoints (Vocabulary v2, Phase 44.1 §2).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Sync {
    pub policy: SyncPolicy,
    /// Match window for `exact`/`approximate` (required for those
    /// policies).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_interval_ms: Option<f64>,
    /// Collect-until-timeout duration for `timeout_any` (required for
    /// that policy).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout_ms: Option<f64>,
}

/// Fan-in sync matching policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SyncPolicy {
    /// message_filters ExactTime.
    Exact,
    /// message_filters ApproximateTime.
    Approximate,
    /// Collect-until-timeout, publish partial set.
    TimeoutAny,
}

/// Named cross-scope composition of `paths:` into an end-to-end budget
/// (Vocabulary v2, Phase 44.1 §4).
#[derive(Debug, Clone, Serialize)]
pub struct ChainDecl {
    /// Composition math: `reaction` (first-reaction latency) vs `age`
    /// (data staleness at the sink) — the two diverge at junctions.
    pub semantics: ChainSemantics,
    /// End-to-end budget (ms) the chain's segments must fit within.
    pub max_latency_ms: f64,
    /// Alternating path/via segments. First and last must be path
    /// segments; no two `via` segments may be adjacent. Cross-file link
    /// resolution is a checker concern (W2's `chain-link` rule).
    pub segments: Vec<ChainSegment>,
}

/// Chain composition semantics (Vocabulary v2, Phase 44.1 §4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum ChainSemantics {
    /// First-reaction latency (TIMEX "reaction").
    Reaction,
    /// Data staleness at the sink (TIMEX "age").
    Age,
}

/// One segment of a [`ChainDecl`]: either a scoped path reference or an
/// explicit connecting topic (`via:`).
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(untagged)]
pub enum ChainSegment {
    /// `{ scope: <scope>, path: <path-name> }` — a named path in the
    /// given scope's manifest.
    Path { scope: String, path: String },
    /// `{ via: <topic-fqn> }` — the explicit connecting topic between two
    /// path segments. No silent FQN matching.
    Via { via: String },
}

/// Drop tolerance specification.
#[derive(Debug, Clone, Serialize)]
pub struct DropSpec {
    /// "N / W" format: max N drops per W messages.
    pub max_count: Option<DropCount>,
    /// Max consecutive drops.
    pub max_consecutive: Option<u32>,
}

/// Drop count: N drops per W messages.
#[derive(Debug, Clone, Serialize)]
pub struct DropCount {
    pub n: u32,
    pub w: u32,
}

impl DropCount {
    /// Per-message drop probability.
    pub fn drop_rate(&self) -> f64 {
        if self.w == 0 {
            0.0
        } else {
            self.n as f64 / self.w as f64
        }
    }

    /// Delivery rate (1 - drop_rate).
    pub fn delivery_rate(&self) -> f64 {
        1.0 - self.drop_rate()
    }
}

impl std::fmt::Display for DropCount {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} / {}", self.n, self.w)
    }
}

/// Parse "N / W" string into DropCount.
impl std::str::FromStr for DropCount {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let parts: Vec<&str> = s.split('/').map(|p| p.trim()).collect();
        if parts.len() != 2 {
            return Err(format!("expected 'N / W' format, got '{s}'"));
        }
        let n: u32 = parts[0]
            .parse()
            .map_err(|_| format!("invalid drop count: '{}'", parts[0]))?;
        let w: u32 = parts[1]
            .parse()
            .map_err(|_| format!("invalid window size: '{}'", parts[1]))?;
        if w == 0 {
            return Err("window size must be > 0".into());
        }
        Ok(DropCount { n, w })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_drop_count_parse() {
        let dc: DropCount = "5 / 100".parse().unwrap();
        assert_eq!(dc.n, 5);
        assert_eq!(dc.w, 100);
        assert!((dc.drop_rate() - 0.05).abs() < 1e-9);
        assert!((dc.delivery_rate() - 0.95).abs() < 1e-9);
    }

    #[test]
    fn test_drop_count_display() {
        let dc = DropCount { n: 3, w: 50 };
        assert_eq!(dc.to_string(), "3 / 50");
    }

    #[test]
    fn test_drop_count_invalid() {
        assert!("abc".parse::<DropCount>().is_err());
        assert!("5".parse::<DropCount>().is_err());
        assert!("5 / 0".parse::<DropCount>().is_err());
    }

    #[test]
    fn test_endpoint_props_default() {
        let ep = EndpointProps::default();
        assert!(ep.min_rate_hz.is_none());
        assert!(ep.state.is_none());
        assert!(ep.required.is_none());
    }
}
