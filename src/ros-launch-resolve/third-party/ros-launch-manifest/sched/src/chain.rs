//! Chain-aware types (Phase 44.3, design
//! `docs/superpowers/specs/2026-07-17-chain-aware-mapper-design.md`):
//! per-path trigger facts and resolved chains, consumed by
//! [`crate::chain_aware_mapper::ChainAwareMapper`].
//!
//! These mirror `ros-launch-manifest-types::EffectiveTrigger`/`ChainDecl`
//! (W1 report) minimally rather than depending on the `types` crate — the
//! sched crate has no dependency on `ros-launch-manifest-types` today (see
//! `sched/Cargo.toml`) and is kept FQN-string-based/dependency-light,
//! consistent with [`crate::resolve::SchedNode`] and
//! [`crate::mapper::MapperNode`]. Callers (play_launch, wave 4) translate
//! from the real `types` crate's `PathDecl::effective_trigger()` /
//! `ChainDecl` into these when building [`crate::mapper::MapperInput`].
//!
//! **Phase 45.3 scoping note** (`docs/design/system-model-sched-ssot.md`
//! "Type sharing"): retiring this mirror in favor of a `sched` → `types`
//! dependency is a separate cleanup, deliberately NOT done here. What this
//! phase adds is `Serialize`/`Deserialize` on the resolved types below
//! (`ResolvedChain`, `ChainElement`, `MapperPath`, `ChainAwareDetail`,
//! `ChainSemantics`, `EffectiveTrigger`) so the `model` crate can embed them
//! verbatim as the shared scheduling-structure schema (nano-ros RFC-0050/52)
//! — one resolved-side schema, no third hand-mirror. `MapWarning` /
//! `MapDiagnostics` are a `--explain`-time diagnostic, not embedded in the
//! model, and are intentionally left non-serializable.

use serde::{Deserialize, Serialize};

use crate::mapper::Criticality;

/// Mirrors `ros_launch_manifest_types::EffectiveTrigger` (W1 report): the
/// resolved trigger fact for one declared path, after the legacy-`input:`
/// derivation rule has already been applied by the caller.
///
/// Serde shape: **adjacently tagged**, lowercase (`kind`/`value`) — e.g.
/// `kind: timer\nvalue: { rate_hz: 10.0 }`, `kind: input\nvalue: [a, b]`,
/// `kind: once` (no `value:`). Deliberately NOT externally tagged (the
/// default, and what `types::Trigger` uses for its JSON-consumed form):
/// `serde_yaml_ng` renders externally-tagged non-unit variants as a YAML
/// *tag* (`!timer`) rather than a nested map, which round-trips but reads
/// surprisingly in a hand-inspected `system_model.yaml`. Adjacent tagging
/// gives a plain, uniform mapping shape across unit/newtype/struct variants
/// and is what this type actually gets embedded through (YAML, not JSON).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "lowercase")]
pub enum EffectiveTrigger {
    /// Clock-sampled: fires on a wall-clock timer at `rate_hz`.
    Timer { rate_hz: f64 },
    /// Event-driven: fires when (all/any of, per `sync:`) these endpoints
    /// receive data. The endpoint names themselves aren't interpreted by
    /// this crate.
    Input(Vec<String>),
    /// Fires exactly once (e.g. a map loader).
    Once,
    /// Fires at caller-unpredictable times (e.g. a service/action server).
    Spontaneous,
    /// No trigger fact could be derived (no explicit `trigger:`, no legacy
    /// `input:`).
    Unclassified,
}

impl EffectiveTrigger {
    /// The timer period in milliseconds, or `None` for anything but
    /// `Timer` with a positive `rate_hz`.
    pub fn period_ms(&self) -> Option<f64> {
        match self {
            EffectiveTrigger::Timer { rate_hz } if *rate_hz > 0.0 => Some(1000.0 / rate_hz),
            _ => None,
        }
    }
}

/// One of a node's declared causal paths, as seen by the mapper (design
/// steps 1 and 6). Also the per-(node, path) requirement-facts unit — the
/// "mapper input" both play_launch's Linux runtime and nano-ros's own RTOS
/// mapper read. (The 45.2 model-embedding of this was reverted 2026-07-20;
/// consumers call this algorithm crate directly. See scheduling.md
/// §"Cross-repo design agreement".)
///
/// Carries two of the four per-path requirement facts the cross-repo
/// agreement names (this crate's `docs/scheduling.md` §"Cross-repo design
/// agreement"): the effective **trigger** and the **deadline**
/// (`max_latency_ms`). The third, **criticality**, lives one level up on
/// [`crate::mapper::MapperNode`] (it is a
/// per-node, not per-path, fact). The fourth, **budget** (the six-dim RTOS
/// mapper's WCET/execution-time dimension — distinct from the deadline), is
/// [`Self::exec_ms`]: usually `None` today (contracts declare deadlines, not
/// WCETs), but the shared schema must carry a slot for the fact the
/// consumer's mapper wants. Unit matches [`ChainElement::Boundary::exec_ms`]
/// (milliseconds, no invented WCET).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MapperPath {
    pub name: String,
    pub effective_trigger: EffectiveTrigger,
    /// Declared end-to-end latency budget for this path (`max_latency_ms`
    /// in the contract vocabulary) — the **deadline** fact, when present.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_latency_ms: Option<f64>,
    /// Declared execution-time **budget** (WCET) for this path, in
    /// milliseconds — the six-dim RTOS mapper's `budget` dimension, distinct
    /// from the deadline. `None` when not declared (no WCET is ever
    /// invented). Same unit/convention as
    /// [`ChainElement::Boundary::exec_ms`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exec_ms: Option<f64>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub inputs: Vec<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub outputs: Vec<String>,
}

/// A chain's declared latency semantics (mirrors `types::ChainSemantics`).
/// Not otherwise interpreted by the mapper (the reaction-vs-age distinction
/// affects the companion checker's arithmetic, not rank assignment).
///
/// Serde shape: lowercase string (`reaction` | `age`), matching
/// `types::ChainSemantics`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ChainSemantics {
    Reaction,
    Age,
}

/// One `(node, path)` link inside a chain [`ChainElement::Segment`], in
/// source-to-sink topological order. A named struct (rather than a bare
/// `(String, String)` tuple) so the cross-repo `system_model.yaml` is
/// self-documenting — the fields serialize as `node:`/`path:` mappings, not
/// an unlabeled two-element sequence.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct SegmentNode {
    pub node: String,
    pub path: String,
}

/// One element of a resolved chain's clock-segmented decomposition (design
/// "Model: clock-segmented chains"). Already resolved by the caller
/// (wave 4): `via` scopes flattened, and each segment's
/// `nodes_in_topo_order` already computed in source-to-sink order (design
/// issue #3 — fan-in longest-path-to-sink/deadline/name tie-breaks require
/// the launch DAG, which only the caller has; this crate consumes the
/// already-linearized result).
///
/// Serde shape: **adjacently tagged**, snake_case (`kind`/`value`) — same
/// rationale as [`EffectiveTrigger`] (avoids `serde_yaml_ng`'s YAML-tag
/// rendering of externally-tagged struct variants): `kind: segment\nvalue:
/// { nodes_in_topo_order: [...] }` / `kind: boundary\nvalue: { node: ...,
/// path: ..., period_ms: ... }`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum ChainElement {
    /// A maximal run of `trigger: input` path links, in source-to-sink
    /// topological order. Each entry is a [`SegmentNode`].
    Segment {
        nodes_in_topo_order: Vec<SegmentNode>,
    },
    /// A `trigger: timer` hop crossed by the chain. `period_ms` is
    /// `1000 / rate_hz`; `exec_ms` is an optional declared execution-time
    /// fact (no WCETs are invented — `None` when not declared).
    Boundary {
        node: String,
        path: String,
        period_ms: f64,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        exec_ms: Option<f64>,
    },
}

/// A declared chain (`chains:`), fully resolved against the launch DAG
/// (design steps 1–2 input). The resolved chain shape both play_launch's
/// Linux runtime and nano-ros's RTOS mapper consume — via this algorithm
/// crate directly, not embedded in the model (the 45.2 embedding was
/// reverted 2026-07-20).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ResolvedChain {
    pub name: String,
    /// Chain criticality, used for chain ordering (design step 3:
    /// "criticality desc"). The W1 vocabulary has no chain-level
    /// `criticality` field, so the caller derives this as the **max**
    /// over the chain's member nodes' `criticality` (documented choice —
    /// a pure aggregation of already-available per-node facts, not
    /// mapper logic).
    pub criticality: Criticality,
    /// The chain's declared end-to-end latency budget.
    pub max_latency_ms: f64,
    pub semantics: ChainSemantics,
    pub elements: Vec<ChainElement>,
}

/// One rank decision's per-(node, path) detail, for `--explain` support
/// (design step 8). Returned by
/// [`crate::mapper::SchedMapper::map_with_diagnostics`]. These are the
/// LINUX REALIZATION — PiCAS fixed-priority ranks play_launch's
/// Linux runtime applies; nano-ros ignores them entirely and derives
/// its own kernel-feature bindings from the resolved structure instead
/// (`provenance` may still surface for cross-platform diagnostics).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChainAwareDetail {
    pub node: String,
    /// `None` only if a future caller ever needs a node-level (not
    /// path-level) rank decision; the `chain_aware` mapper always
    /// populates this today.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    /// The path's final priority (POSIX RT priority, same convention as
    /// [`crate::resolve::ResolvedTier::priority`]).
    pub priority: i64,
    /// One-line human-readable provenance, e.g.
    /// `derived(chain_aware: sensing_to_actuation S2 drain 2/2 -> prio 44)`.
    pub provenance: String,
}

/// A diagnostic surfaced while deriving a plan (design step 8: chain
/// feasibility, co-location warnings, etc). Additive — not every mapper
/// produces any.
#[derive(Clone, Debug, PartialEq)]
pub enum MapWarning {
    /// A chain's `sampling_cost` (design "Model: clock-segmented chains")
    /// consumes its entire declared budget or more — scheduling cannot fix
    /// this (period/architecture change required). The chain is excluded
    /// from priority shaping; its members keep their local-fact priorities.
    /// The chain was judged FEASIBLE, but one or more of its timer boundaries
    /// carries no WCET (`exec_ms`), so the sampling cost was computed with
    /// those hops counted as ZERO execution time.
    ///
    /// This is not a scheduling problem — it is an evidence problem. Absent is
    /// not zero: the verdict is optimistic by an unknown amount, and reporting
    /// it as a plain `feasible` would claim headroom nobody measured. nano-ros
    /// issue 0259 (the feasibility check assumes `B_i = 0` and `C_i = 0`
    /// wherever a WCET is missing) is the standing analysis; this warning makes
    /// the assumption visible at the point it is made rather than leaving it in
    /// a doc comment.
    ChainFeasibleWithoutWcet {
        chain: String,
        /// `node/path` of each boundary counted as zero.
        boundaries_without_wcet: Vec<String>,
    },
    ChainInfeasible {
        chain: String,
        sampling_cost_ms: f64,
        budget_ms: f64,
    },
    /// Two or more nodes ended at the same real-time priority — band
    /// compression collapses adjacent ranks, so this is a state the mapper
    /// itself produces — and `SCHED_RR` could not be derived to time-slice
    /// between them.
    ///
    /// Under `SCHED_FIFO` a tie means one node can starve the others; `SCHED_RR`
    /// would rotate them. But Linux's RR slice is a **global** sysctl, and at
    /// its 100 ms default a slice can be as long as a tied node's entire
    /// period, at which point RR degenerates to FIFO while *looking* like it
    /// fixed starvation. Deriving it there would be a cosmetic change presented
    /// as a real one, so it is declined and reported instead.
    ///
    /// `rr_timeslice_us: None` means the platform file did not state the
    /// host's slice — unknown, not "assume the default".
    UnmitigatedPriorityTie {
        priority: i64,
        nodes: Vec<String>,
        rr_timeslice_us: Option<u64>,
        shortest_period_us: Option<u64>,
    },
    /// The platform's `rt_priority_band` is narrower than the number of
    /// distinct priority classes that remain after every *legal* collapse
    /// (within segments, then within the same chain) has been applied —
    /// design step 7 / issue #5 forbids collapsing across the
    /// chain/non-chain divide or across criticality buckets, so the mapper
    /// keeps the least-bad assignment (clamping the lowest classes into
    /// `band.min`, which introduces cross-chain/bucket *ties*, never
    /// inversions) and reports it here instead of silently pretending the
    /// band fit.
    BandTooNarrow {
        /// Distinct priority classes remaining after legal collapse.
        distinct_classes: usize,
        /// The band's inclusive width (`max - min + 1`).
        band_width: usize,
        /// Human-readable labels of the classes that were clamped into
        /// `band.min` (Phase 44.4 review, cosmetic-6): `chain '<name>'`
        /// for chain-owned classes, `non-chain bucket` for the
        /// criticality-bucketed remainder. Deduped, ranked order preserved.
        clamped: Vec<String>,
    },
}

/// The richer output of [`crate::mapper::SchedMapper::map_with_diagnostics`]:
/// per-(node, path) rank provenance plus any warnings, alongside the
/// [`crate::mapper::SchedPlan`] itself.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MapDiagnostics {
    pub details: Vec<ChainAwareDetail>,
    pub warnings: Vec<MapWarning>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip<T>(value: &T) -> T
    where
        T: Serialize + serde::de::DeserializeOwned,
    {
        let yaml = serde_yaml_ng::to_string(value).expect("serialize");
        serde_yaml_ng::from_str(&yaml).expect("deserialize")
    }

    #[test]
    fn effective_trigger_roundtrips_every_variant() {
        for v in [
            EffectiveTrigger::Timer { rate_hz: 10.0 },
            EffectiveTrigger::Input(vec!["a".into(), "b".into()]),
            EffectiveTrigger::Once,
            EffectiveTrigger::Spontaneous,
            EffectiveTrigger::Unclassified,
        ] {
            assert_eq!(roundtrip(&v), v);
        }
    }

    #[test]
    fn effective_trigger_yaml_shape() {
        // adjacently tagged: kind/value, no `!tag` YAML tag syntax.
        let timer = EffectiveTrigger::Timer { rate_hz: 10.0 };
        assert_eq!(
            serde_yaml_ng::to_string(&timer).unwrap().trim(),
            "kind: timer\nvalue:\n  rate_hz: 10.0"
        );
        let input = EffectiveTrigger::Input(vec!["a".into()]);
        assert_eq!(
            serde_yaml_ng::to_string(&input).unwrap().trim(),
            "kind: input\nvalue:\n- a"
        );
        // unit variants: just the tag, no `value:` key at all.
        let once = EffectiveTrigger::Once;
        assert_eq!(
            serde_yaml_ng::to_string(&once).unwrap().trim(),
            "kind: once"
        );
        let unclassified = EffectiveTrigger::Unclassified;
        assert_eq!(
            serde_yaml_ng::to_string(&unclassified).unwrap().trim(),
            "kind: unclassified"
        );
    }

    #[test]
    fn chain_semantics_roundtrips_lowercase() {
        assert_eq!(
            serde_yaml_ng::to_string(&ChainSemantics::Reaction)
                .unwrap()
                .trim(),
            "reaction"
        );
        assert_eq!(
            serde_yaml_ng::to_string(&ChainSemantics::Age)
                .unwrap()
                .trim(),
            "age"
        );
        assert_eq!(
            roundtrip(&ChainSemantics::Reaction),
            ChainSemantics::Reaction
        );
        assert_eq!(roundtrip(&ChainSemantics::Age), ChainSemantics::Age);
    }

    #[test]
    fn mapper_path_roundtrips() {
        // with both deadline (max_latency_ms) and budget (exec_ms) present
        let p = MapperPath {
            name: "main".into(),
            effective_trigger: EffectiveTrigger::Timer { rate_hz: 20.0 },
            max_latency_ms: Some(30.0),
            exec_ms: Some(4.5),
            inputs: vec!["/sensing/pointcloud".into()],
            outputs: vec!["/perception/objects".into()],
        };
        let yaml = serde_yaml_ng::to_string(&p).unwrap();
        assert!(yaml.contains("max_latency_ms: 30.0"), "{yaml}");
        assert!(yaml.contains("exec_ms: 4.5"), "{yaml}");
        assert_eq!(roundtrip(&p), p);

        // exec_ms omitted when None (the common case: contracts declare a
        // deadline, rarely a WCET) — the slot exists but isn't emitted.
        let no_budget = MapperPath {
            exec_ms: None,
            ..p.clone()
        };
        assert!(
            !serde_yaml_ng::to_string(&no_budget)
                .unwrap()
                .contains("exec_ms"),
        );
        assert_eq!(roundtrip(&no_budget), no_budget);
    }

    #[test]
    fn chain_element_roundtrips_segment_and_boundary() {
        let segment = ChainElement::Segment {
            nodes_in_topo_order: vec![
                SegmentNode {
                    node: "detector".into(),
                    path: "main".into(),
                },
                SegmentNode {
                    node: "tracker".into(),
                    path: "main".into(),
                },
            ],
        };
        assert_eq!(roundtrip(&segment), segment);

        let boundary = ChainElement::Boundary {
            node: "planner".into(),
            path: "main".into(),
            period_ms: 50.0,
            exec_ms: Some(5.0),
        };
        assert_eq!(roundtrip(&boundary), boundary);

        // adjacently tagged, snake_case; segment entries are named
        // node:/path: mappings, not unlabeled tuples.
        let seg_yaml = serde_yaml_ng::to_string(&segment).unwrap();
        assert!(seg_yaml.starts_with("kind: segment"), "{seg_yaml}");
        assert!(seg_yaml.contains("node: detector"), "{seg_yaml}");
        assert!(seg_yaml.contains("path: main"), "{seg_yaml}");
        assert!(
            serde_yaml_ng::to_string(&boundary)
                .unwrap()
                .starts_with("kind: boundary")
        );
    }

    #[test]
    fn resolved_chain_roundtrips() {
        let chain = ResolvedChain {
            name: "sensing_to_actuation".into(),
            criticality: Criticality::High,
            max_latency_ms: 100.0,
            semantics: ChainSemantics::Reaction,
            elements: vec![
                ChainElement::Segment {
                    nodes_in_topo_order: vec![SegmentNode {
                        node: "detector".into(),
                        path: "main".into(),
                    }],
                },
                ChainElement::Boundary {
                    node: "planner".into(),
                    path: "main".into(),
                    period_ms: 20.0,
                    exec_ms: None,
                },
            ],
        };
        assert_eq!(roundtrip(&chain), chain);
    }

    #[test]
    fn chain_aware_detail_roundtrips() {
        let d = ChainAwareDetail {
            node: "/perception/tracker".into(),
            path: Some("main".into()),
            priority: 44,
            provenance: "derived(chain_aware: sensing_to_actuation S2 drain 2/2 -> prio 44)".into(),
        };
        assert_eq!(roundtrip(&d), d);

        // path: None is omitted, not written as `path: null`
        let node_level = ChainAwareDetail {
            node: "/perception/tracker".into(),
            path: None,
            priority: 10,
            provenance: "derived(default)".into(),
        };
        assert!(
            !serde_yaml_ng::to_string(&node_level)
                .unwrap()
                .contains("path")
        );
        assert_eq!(roundtrip(&node_level), node_level);
    }
}
