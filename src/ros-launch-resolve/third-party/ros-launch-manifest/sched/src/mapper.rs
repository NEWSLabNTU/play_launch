//! `SchedMapper` trait + registry + built-ins (design §4): the pluggable
//! algorithm that derives a scheduling plan from per-node timing facts
//! (`MapperInput`) and platform facts (`PlatformFacts`).
//!
//! `SchedPlan` is a type alias for the crate's existing
//! [`crate::resolve::ResolvedTierTable`] — deliberately reused rather than
//! inventing a parallel type, since it is already exactly "ordered
//! priority/core/sched_class placement grouped by tier, with member node
//! names", which is what every mapper (manual, rate-monotonic,
//! deadline-monotonic) produces. Built-in mappers that assign each node its
//! own unique priority represent that as a one-member-per-tier
//! `ResolvedTier` (tier name = node name); nodes with no facts collapse into
//! the existing synthesized [`crate::resolve::DEFAULT_TIER`] (priority 0,
//! no `sched_class` — i.e. `SCHED_OTHER`/non-RT), exactly like an unmatched
//! node in the legacy resolver.

use std::collections::BTreeMap;

use crate::{
    chain::{MapDiagnostics, ResolvedChain},
    platform::{PlatformResources, PriorityBand},
    resolve::{DEFAULT_TIER, ResolvedTier, ResolvedTierTable, SchedError, SchedNode, resolve},
    types::SystemSched,
};

/// Mapper hint: platform-agnostic criticality (design §2.1, contract
/// `nodes.<name>.criticality`). No priority numbers.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
#[serde(rename_all = "lowercase")]
pub enum Criticality {
    Low,
    Medium,
    High,
}

/// One node as seen by a `SchedMapper` — dependency-free, extracted from
/// launch + contract by the caller (play_launch, in wave 2). Intentionally
/// minimal: no graph edges yet (YAGNI; a future field, e.g. `depends_on`,
/// can be added additively once a mapper needs them).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MapperNode {
    pub name: String,
    pub scope: String,
    pub rate_hz: Option<f64>,
    pub deadline_us: Option<u64>,
    pub criticality: Option<Criticality>,
    pub path_budget_ms: Option<f64>,
    /// Per-path trigger facts (Phase 44.3, additive): the node's declared
    /// causal paths with their resolved `trigger:`/`max_latency_ms` facts,
    /// as translated by the caller from `ros-launch-manifest-types`'
    /// `PathDecl`/`EffectiveTrigger` (W1). Empty for callers/mappers that
    /// don't populate it (`rate_monotonic`/`deadline_monotonic`/`manual`
    /// ignore this field entirely — only [`crate::chain_aware_mapper`]
    /// consumes it).
    pub paths: Vec<crate::chain::MapperPath>,
}

/// The mapper's input: every node's timing facts, plus (bridge path only) a
/// legacy tiers+assign spec for the `manual` mapper.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MapperInput {
    pub nodes: Vec<MapperNode>,
    /// Present only when driving the `manual` mapper via the legacy `.toml`
    /// bridge ([`crate::bridge::parse_legacy_toml`]) — the `manual` mapper
    /// requires this and errors ([`MapError::MissingLegacySpec`]) without
    /// it. Other built-in mappers ignore this field.
    pub legacy: Option<SystemSched>,
    /// Resolved `chains:` (Phase 44.3, additive), already flattened by the
    /// caller (W4) against the launch DAG: `via` scopes resolved, and each
    /// segment's `nodes_in_topo_order` already computed (fan-in
    /// longest-path-to-sink/deadline/name tie-breaks are the caller's job —
    /// it has the DAG; this crate stays FQN-string-based and
    /// dependency-free). Empty for callers/mappers that don't populate it
    /// (only [`crate::chain_aware_mapper`] consumes it).
    pub chains: Vec<ResolvedChain>,
}

/// The parsed `resources:` facts for the mapper's target, per
/// [`crate::platform::PlatformFile::resources`].
pub type PlatformFacts = PlatformResources;

/// The mapper's output: reuses [`ResolvedTierTable`] (see module docs).
pub type SchedPlan = ResolvedTierTable;

/// Errors produced while deriving a plan.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum MapError {
    #[error("mapper `{mapper}` requires `resources.rt_priority_band` (target must be `posix`)")]
    MissingPriorityBand { mapper: String },
    #[error(
        "mapper `{mapper}`'s `rt_priority_band` {{ min: {min}, max: {max} }} is invalid: {reason}"
    )]
    InvalidPriorityBand {
        mapper: String,
        min: i64,
        max: i64,
        reason: String,
    },
    #[error(
        "mapper `manual` requires a legacy tiers+assign spec (only reachable via the `.toml` bridge)"
    )]
    MissingLegacySpec,
    #[error("manual mapper resolve error: {0}")]
    Resolve(#[from] SchedError),
}

/// A pluggable scheduling-context derivation strategy (design §4).
///
/// `map` derives a [`SchedPlan`] from per-node facts and platform facts.
/// Applying explicit `overrides` on top of the derived plan (design §5, §6:
/// "override beats derived, always") is the caller's job (play_launch, wave
/// 2's pipeline) — not part of the trait, since overrides are
/// mapper-independent and platform-file-scoped, not mapper logic.
pub trait SchedMapper {
    /// The name this mapper is registered/selected under (`platform.mapper`
    /// in the YAML schema).
    fn name(&self) -> &str;

    /// Derive a scheduling plan from per-node facts and platform facts.
    fn map(&self, input: &MapperInput, facts: &PlatformFacts) -> Result<SchedPlan, MapError>;

    /// Richer variant returning per-(node, path) rank provenance and
    /// diagnostics (warnings) alongside the plan (Phase 44.3, additive
    /// defaulted method — design step 6/8: `--explain` support and
    /// chain-feasibility warnings). The default implementation calls
    /// [`SchedMapper::map`] and returns no diagnostics; existing built-ins
    /// (`manual`/`rate_monotonic`/`deadline_monotonic`) need no changes.
    /// [`crate::chain_aware_mapper::ChainAwareMapper`] overrides this.
    fn map_with_diagnostics(
        &self,
        input: &MapperInput,
        facts: &PlatformFacts,
    ) -> Result<(SchedPlan, MapDiagnostics), MapError> {
        let plan = self.map(input, facts)?;
        Ok((plan, MapDiagnostics::default()))
    }
}

/// `manual` — the legacy semantics: consumes `input.legacy` (tiers +
/// `[[assign]]`) and delegates to [`crate::resolve::resolve`] for `posix`,
/// reproducing today's output exactly. This is the bridge path's mapper;
/// `facts` is ignored (the legacy schema carries no separate platform
/// facts — priority numbers live in the tier's own `[tiers.<name>.posix]`
/// sub-table).
#[derive(Debug, Default)]
pub struct ManualMapper;

impl SchedMapper for ManualMapper {
    fn name(&self) -> &str {
        "manual"
    }

    fn map(&self, input: &MapperInput, _facts: &PlatformFacts) -> Result<SchedPlan, MapError> {
        let legacy = input.legacy.as_ref().ok_or(MapError::MissingLegacySpec)?;
        let nodes: Vec<SchedNode> = input
            .nodes
            .iter()
            .map(|n| SchedNode {
                name: n.name.clone(),
                scope: n.scope.clone(),
            })
            .collect();
        let table = resolve(&legacy.tiers, &legacy.assign, &nodes, "posix")?;
        Ok(table)
    }
}

/// Extract the `posix` `rt_priority_band` from `facts`, or error naming
/// `mapper_name`.
pub(crate) fn require_posix_band(
    facts: &PlatformFacts,
    mapper_name: &str,
) -> Result<PriorityBand, MapError> {
    let PlatformResources::Posix(posix) = facts else {
        return Err(MapError::MissingPriorityBand {
            mapper: mapper_name.to_string(),
        });
    };
    let band = posix
        .rt_priority_band
        .ok_or_else(|| MapError::MissingPriorityBand {
            mapper: mapper_name.to_string(),
        })?;
    if let Err(reason) = band.validate_posix() {
        return Err(MapError::InvalidPriorityBand {
            mapper: mapper_name.to_string(),
            min: band.min,
            max: band.max,
            reason,
        });
    }
    Ok(band)
}

/// Priority for rank `i` (0 = highest) out of `n` ranked nodes, spread
/// linearly across `band` (rank 0 → `band.max`, rank `n-1` → `band.min`).
/// `n <= 1` → `band.max`.
fn spread_priority(i: usize, n: usize, band: &PriorityBand) -> i64 {
    if n <= 1 {
        return band.max;
    }
    let span = (band.max - band.min) as f64;
    let frac = i as f64 / (n - 1) as f64;
    band.max - (span * frac).round() as i64
}

/// Build a [`SchedPlan`] from nodes already ranked highest-priority-first
/// (`ranked`), spreading priorities across `band`, plus the remaining
/// (fact-less) nodes collapsed into the synthesized default tier.
fn build_ranked_plan(
    ranked: &[&MapperNode],
    rest: &[&MapperNode],
    band: &PriorityBand,
) -> SchedPlan {
    let n = ranked.len();
    let mut tiers: Vec<ResolvedTier> = Vec::with_capacity(n + 1);

    for (i, node) in ranked.iter().enumerate() {
        tiers.push(ResolvedTier {
            name: node.name.clone(),
            priority: spread_priority(i, n, band),
            sched_class: Some("SCHED_FIFO".to_string()),
            class: Some("real_time".to_string()),
            members: vec![node.name.clone()],
            ..Default::default()
        });
    }

    if !rest.is_empty() {
        let mut members: Vec<String> = rest.iter().map(|n| n.name.clone()).collect();
        members.sort();
        tiers.push(ResolvedTier {
            name: DEFAULT_TIER.to_string(),
            members,
            ..Default::default()
        });
    }

    // Highest priority first; ties by name — same convention as `resolve()`.
    tiers.sort_by(|a, b| b.priority.cmp(&a.priority).then(a.name.cmp(&b.name)));
    ResolvedTierTable { tiers }
}

/// `rate_monotonic` — higher rate → higher priority, spread within
/// `resources.rt_priority_band`. Deterministic: ranked by rate descending,
/// ties broken by node name ascending. Nodes with no `rate_hz` fall into
/// the non-RT default tier.
#[derive(Debug, Default)]
pub struct RateMonotonicMapper;

impl SchedMapper for RateMonotonicMapper {
    fn name(&self) -> &str {
        "rate_monotonic"
    }

    fn map(&self, input: &MapperInput, facts: &PlatformFacts) -> Result<SchedPlan, MapError> {
        let band = require_posix_band(facts, self.name())?;

        let mut ranked: Vec<&MapperNode> =
            input.nodes.iter().filter(|n| n.rate_hz.is_some()).collect();
        ranked.sort_by(|a, b| {
            b.rate_hz
                .partial_cmp(&a.rate_hz)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| a.name.cmp(&b.name))
        });

        let rest: Vec<&MapperNode> = input.nodes.iter().filter(|n| n.rate_hz.is_none()).collect();

        Ok(build_ranked_plan(&ranked, &rest, &band))
    }
}

/// `deadline_monotonic` — shorter deadline → higher priority, spread
/// within `resources.rt_priority_band`. Deterministic: ranked by
/// `deadline_us` ascending, ties broken by node name ascending. Nodes with
/// no `deadline_us` fall into the non-RT default tier.
#[derive(Debug, Default)]
pub struct DeadlineMonotonicMapper;

impl SchedMapper for DeadlineMonotonicMapper {
    fn name(&self) -> &str {
        "deadline_monotonic"
    }

    fn map(&self, input: &MapperInput, facts: &PlatformFacts) -> Result<SchedPlan, MapError> {
        let band = require_posix_band(facts, self.name())?;

        let mut ranked: Vec<&MapperNode> = input
            .nodes
            .iter()
            .filter(|n| n.deadline_us.is_some())
            .collect();
        ranked.sort_by(|a, b| {
            a.deadline_us
                .cmp(&b.deadline_us)
                .then_with(|| a.name.cmp(&b.name))
        });

        let rest: Vec<&MapperNode> = input
            .nodes
            .iter()
            .filter(|n| n.deadline_us.is_none())
            .collect();

        Ok(build_ranked_plan(&ranked, &rest, &band))
    }
}

/// Registry of named mappers. Built-ins are registered by
/// [`MapperRegistry::with_builtins`]; consumers (play_launch, nano-ros)
/// register additional mappers at link time via [`MapperRegistry::register`].
#[derive(Default)]
pub struct MapperRegistry {
    mappers: BTreeMap<String, Box<dyn SchedMapper>>,
}

impl MapperRegistry {
    /// An empty registry (no built-ins).
    pub fn new() -> Self {
        MapperRegistry {
            mappers: BTreeMap::new(),
        }
    }

    /// Register (or replace) a mapper under its own `name()`.
    pub fn register(&mut self, mapper: Box<dyn SchedMapper>) {
        self.mappers.insert(mapper.name().to_string(), mapper);
    }

    /// Look up a mapper by name.
    pub fn get(&self, name: &str) -> Option<&dyn SchedMapper> {
        self.mappers.get(name).map(|b| b.as_ref())
    }

    /// A registry pre-populated with `manual`, `rate_monotonic`, and
    /// `deadline_monotonic`.
    pub fn with_builtins() -> Self {
        let mut registry = Self::new();
        registry.register(Box::new(ManualMapper));
        registry.register(Box::new(RateMonotonicMapper));
        registry.register(Box::new(DeadlineMonotonicMapper));
        registry.register(Box::new(crate::chain_aware_mapper::ChainAwareMapper));
        registry
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PosixResources;

    fn node(name: &str, rate_hz: Option<f64>, deadline_us: Option<u64>) -> MapperNode {
        MapperNode {
            name: name.to_string(),
            scope: "/".to_string(),
            rate_hz,
            deadline_us,
            criticality: None,
            path_budget_ms: None,
            paths: Vec::new(),
        }
    }

    fn posix_facts(band: Option<(i64, i64)>) -> PlatformFacts {
        PlatformFacts::Posix(PosixResources {
            rt_priority_band: band.map(|(min, max)| PriorityBand { min, max }),
            isolated_cpus: vec![],
        })
    }

    #[test]
    fn with_builtins_registers_all_four() {
        let registry = MapperRegistry::with_builtins();
        assert!(registry.get("manual").is_some());
        assert!(registry.get("rate_monotonic").is_some());
        assert!(registry.get("deadline_monotonic").is_some());
        assert!(registry.get("chain_aware").is_some());
    }

    #[test]
    fn unknown_mapper_name_is_none() {
        let registry = MapperRegistry::with_builtins();
        assert!(registry.get("nonexistent").is_none());
    }

    #[test]
    fn rate_monotonic_orders_by_rate_desc() {
        let mapper = RateMonotonicMapper;
        let input = MapperInput {
            nodes: vec![
                node("/slow", Some(10.0), None),
                node("/fast", Some(100.0), None),
                node("/mid", Some(50.0), None),
            ],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = posix_facts(Some((10, 40)));
        let plan = mapper.map(&input, &facts).expect("maps");

        // Highest priority first: fast(100) > mid(50) > slow(10).
        assert_eq!(plan.tiers[0].name, "/fast");
        assert_eq!(plan.tiers[0].priority, 40);
        assert_eq!(plan.tiers[1].name, "/mid");
        assert_eq!(plan.tiers[2].name, "/slow");
        assert_eq!(plan.tiers[2].priority, 10);
    }

    #[test]
    fn rate_monotonic_ties_broken_by_name_asc() {
        let mapper = RateMonotonicMapper;
        let input = MapperInput {
            nodes: vec![node("/b", Some(50.0), None), node("/a", Some(50.0), None)],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = posix_facts(Some((10, 40)));
        let plan = mapper.map(&input, &facts).expect("maps");
        assert_eq!(plan.tiers[0].name, "/a");
        assert_eq!(plan.tiers[1].name, "/b");
    }

    #[test]
    fn rate_monotonic_missing_rate_goes_to_default_non_rt() {
        let mapper = RateMonotonicMapper;
        let input = MapperInput {
            nodes: vec![node("/rt", Some(100.0), None), node("/bg", None, None)],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = posix_facts(Some((10, 40)));
        let plan = mapper.map(&input, &facts).expect("maps");

        let default_tier = plan
            .tiers
            .iter()
            .find(|t| t.name == DEFAULT_TIER)
            .expect("default tier present");
        assert_eq!(default_tier.members, vec!["/bg".to_string()]);
        assert_eq!(default_tier.priority, 0);
        assert!(default_tier.sched_class.is_none());
    }

    #[test]
    fn rate_monotonic_single_node_gets_band_max() {
        let mapper = RateMonotonicMapper;
        let input = MapperInput {
            nodes: vec![node("/only", Some(10.0), None)],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = posix_facts(Some((10, 40)));
        let plan = mapper.map(&input, &facts).expect("maps");
        assert_eq!(plan.tiers[0].priority, 40);
    }

    #[test]
    fn rate_monotonic_missing_band_errors() {
        let mapper = RateMonotonicMapper;
        let input = MapperInput {
            nodes: vec![node("/a", Some(10.0), None)],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = posix_facts(None);
        let err = mapper.map(&input, &facts).unwrap_err();
        assert!(matches!(err, MapError::MissingPriorityBand { .. }));
    }

    #[test]
    fn rate_monotonic_non_posix_facts_errors() {
        let mapper = RateMonotonicMapper;
        let input = MapperInput {
            nodes: vec![node("/a", Some(10.0), None)],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = PlatformFacts::Raw(serde_yaml_ng::Value::Null);
        let err = mapper.map(&input, &facts).unwrap_err();
        assert!(matches!(err, MapError::MissingPriorityBand { .. }));
    }

    #[test]
    fn rate_monotonic_more_nodes_than_band_width_stays_in_band_and_ordered() {
        // 5 distinct rates into a 3-priority band {10..=12}: collisions are
        // unavoidable, but the result must be deterministic, entirely inside
        // the band, and monotonic (higher rate never gets a LOWER priority
        // than a slower node — adjacent ranks may tie, never invert).
        let mapper = RateMonotonicMapper;
        let input = MapperInput {
            nodes: vec![
                node("/n100", Some(100.0), None),
                node("/n80", Some(80.0), None),
                node("/n60", Some(60.0), None),
                node("/n40", Some(40.0), None),
                node("/n20", Some(20.0), None),
            ],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = posix_facts(Some((10, 12)));
        let plan = mapper.map(&input, &facts).expect("maps");

        // Deterministic exact spread: rank i of 5 across span 2 →
        // 12 - round(2*i/4) = [12, 11, 11, 10, 10].
        let got: Vec<(String, i64)> = plan
            .tiers
            .iter()
            .map(|t| (t.name.clone(), t.priority))
            .collect();
        assert_eq!(
            got,
            vec![
                ("/n100".to_string(), 12),
                ("/n60".to_string(), 11),
                ("/n80".to_string(), 11),
                ("/n20".to_string(), 10),
                ("/n40".to_string(), 10),
            ],
            "note: equal-priority tiers order by name in the OUTPUT table (/n60 before /n80, \
             /n20 before /n40); rank (and thus priority) is still assigned by rate"
        );

        // Every priority inside the band.
        for t in &plan.tiers {
            assert!(
                (10..=12).contains(&t.priority),
                "tier {} priority {} escaped band 10..=12",
                t.name,
                t.priority
            );
        }

        // Monotonic: for every pair, the faster node's priority is >= the
        // slower node's (never inverted). This is exactly what the
        // contradiction detector checks — reuse it as the oracle.
        assert!(
            crate::validate::rate_priority_contradictions(&input, &plan).is_empty(),
            "band-compressed plan must never invert rate order"
        );
    }

    #[test]
    fn deadline_monotonic_more_nodes_than_band_width_stays_in_band_and_ordered() {
        let mapper = DeadlineMonotonicMapper;
        let input = MapperInput {
            nodes: vec![
                node("/d1", None, Some(1_000)),
                node("/d2", None, Some(2_000)),
                node("/d3", None, Some(3_000)),
                node("/d4", None, Some(4_000)),
                node("/d5", None, Some(5_000)),
            ],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = posix_facts(Some((10, 12)));
        let plan = mapper.map(&input, &facts).expect("maps");

        for t in &plan.tiers {
            assert!((10..=12).contains(&t.priority));
        }
        assert!(crate::validate::deadline_priority_contradictions(&input, &plan).is_empty());

        // Tightest deadline holds band max; loosest holds band min.
        let prio = |name: &str| {
            plan.tiers
                .iter()
                .find(|t| t.name == name)
                .map(|t| t.priority)
                .unwrap()
        };
        assert_eq!(prio("/d1"), 12);
        assert_eq!(prio("/d5"), 10);
    }

    #[test]
    fn rate_monotonic_band_outside_posix_rt_range_errors() {
        let mapper = RateMonotonicMapper;
        let input = MapperInput {
            nodes: vec![node("/a", Some(10.0), None)],
            legacy: None,
            chains: Vec::new(),
        };
        // 0 is not a legal SCHED_FIFO/SCHED_RR priority.
        let err = mapper.map(&input, &posix_facts(Some((0, 40)))).unwrap_err();
        let MapError::InvalidPriorityBand { reason, .. } = &err else {
            panic!("expected InvalidPriorityBand, got: {err:?}");
        };
        assert!(reason.contains("1..=99"), "got: {reason}");

        // 200 is beyond 99.
        let err = mapper
            .map(&input, &posix_facts(Some((10, 200))))
            .unwrap_err();
        assert!(matches!(err, MapError::InvalidPriorityBand { .. }));
    }

    #[test]
    fn rate_monotonic_invalid_band_errors() {
        let mapper = RateMonotonicMapper;
        let input = MapperInput {
            nodes: vec![node("/a", Some(10.0), None)],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = posix_facts(Some((40, 10)));
        let err = mapper.map(&input, &facts).unwrap_err();
        assert!(matches!(err, MapError::InvalidPriorityBand { .. }));
    }

    #[test]
    fn deadline_monotonic_orders_by_deadline_asc() {
        let mapper = DeadlineMonotonicMapper;
        let input = MapperInput {
            nodes: vec![
                node("/loose", None, Some(100_000)),
                node("/tight", None, Some(5_000)),
                node("/mid", None, Some(50_000)),
            ],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = posix_facts(Some((10, 40)));
        let plan = mapper.map(&input, &facts).expect("maps");
        assert_eq!(plan.tiers[0].name, "/tight");
        assert_eq!(plan.tiers[0].priority, 40);
        assert_eq!(plan.tiers[1].name, "/mid");
        assert_eq!(plan.tiers[2].name, "/loose");
        assert_eq!(plan.tiers[2].priority, 10);
    }

    #[test]
    fn deadline_monotonic_missing_deadline_goes_to_default_non_rt() {
        let mapper = DeadlineMonotonicMapper;
        let input = MapperInput {
            nodes: vec![node("/rt", None, Some(1_000)), node("/bg", None, None)],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = posix_facts(Some((10, 40)));
        let plan = mapper.map(&input, &facts).expect("maps");
        let default_tier = plan.tiers.iter().find(|t| t.name == DEFAULT_TIER).unwrap();
        assert_eq!(default_tier.members, vec!["/bg".to_string()]);
    }

    #[test]
    fn deadline_monotonic_ties_broken_by_name_asc() {
        let mapper = DeadlineMonotonicMapper;
        let input = MapperInput {
            nodes: vec![node("/b", None, Some(5_000)), node("/a", None, Some(5_000))],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = posix_facts(Some((10, 40)));
        let plan = mapper.map(&input, &facts).expect("maps");
        assert_eq!(plan.tiers[0].name, "/a");
        assert_eq!(plan.tiers[1].name, "/b");
    }

    #[test]
    fn manual_mapper_requires_legacy_spec() {
        let mapper = ManualMapper;
        let input = MapperInput {
            nodes: vec![node("/a", None, None)],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = posix_facts(None);
        let err = mapper.map(&input, &facts).unwrap_err();
        assert!(matches!(err, MapError::MissingLegacySpec));
    }
}
