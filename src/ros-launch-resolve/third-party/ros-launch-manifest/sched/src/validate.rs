//! Conflict-semantics validation helpers (design §6), consumed by
//! play_launch's `--sched-apply warn|strict` pipeline (wave 2). Pure
//! functions over an already-derived [`SchedPlan`] — no I/O, no mode
//! decision (warn vs strict is the caller's call).

use std::collections::BTreeMap;

use crate::{
    mapper::{MapperInput, SchedPlan},
    platform::PriorityBand,
    resolve::DEFAULT_TIER,
};

/// One tier whose priority falls outside the platform's `rt_priority_band`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BandViolation {
    /// The tier name (for built-in derived mappers, this is the node name —
    /// see [`crate::mapper`] module docs).
    pub tier: String,
    pub priority: i64,
    pub band: PriorityBand,
}

/// List every non-default tier in `plan` whose priority falls outside
/// `band`. The caller (play_launch) decides warn-and-clamp vs strict-error
/// per `--sched-apply`; this function only reports.
pub fn band_violations(plan: &SchedPlan, band: &PriorityBand) -> Vec<BandViolation> {
    plan.tiers
        .iter()
        .filter(|t| t.name != DEFAULT_TIER)
        // A `SCHED_DEADLINE` tier has NO priority — deadline threads preempt
        // every fixed-priority thread regardless of RT priority, which is why
        // the kernel gives them a reservation instead of a number. Its
        // `priority` field is therefore absent, not zero, and comparing it
        // against the RT band would flag a violation that does not exist.
        //
        // This is the same defect class as the `SCHED_OTHER` tier that was
        // once "clamped" into the RT band and printed as `SCHED_OTHER 10` — a
        // state Linux cannot represent, and fatal under strict mode. Skipping
        // is the fix in both cases: check only tiers that have a priority to
        // check.
        .filter(|t| {
            t.posix
                .as_ref()
                .is_none_or(|p| p.sched.priority().is_some())
        })
        .filter(|t| !band.contains(t.priority))
        .map(|t| BandViolation {
            tier: t.name.clone(),
            priority: t.priority,
            band: *band,
        })
        .collect()
}

/// A pair of nodes whose final priority order contradicts their timing-fact
/// order (design §6: "a final priority order that contradicts the
/// contract's rate/deadline order yields a warning citing both sources").
#[derive(Clone, Debug, PartialEq)]
pub struct Contradiction {
    /// The node whose timing fact implies it should be prioritized at least
    /// as high as `node_b`, but whose final priority is lower.
    pub node_a: String,
    pub node_b: String,
    /// Which fact produced the contradiction: `"rate_hz"` or `"deadline_us"`.
    pub kind: &'static str,
}

/// Build a node-name → priority lookup by scanning every tier's members
/// (tier-grouped or one-node-per-tier plans both work).
fn priority_by_node(plan: &SchedPlan) -> BTreeMap<String, i64> {
    let mut out = BTreeMap::new();
    for tier in &plan.tiers {
        for member in &tier.members {
            out.insert(member.clone(), tier.priority);
        }
    }
    out
}

/// Generic O(n²) pairwise contradiction scan: for every pair of nodes with a
/// known `key`, if `key` orders `a` at least as important as `b` (per
/// `a_at_least_as_important`) but the final priority says otherwise, report
/// it. Ties in `key` are not contradictions (no implied order to violate).
fn scan_contradictions<'a>(
    nodes: impl Iterator<Item = (&'a str, f64)>,
    prio: &BTreeMap<String, i64>,
    kind: &'static str,
) -> Vec<Contradiction> {
    let present: Vec<(&str, f64)> = nodes.filter(|(name, _)| prio.contains_key(*name)).collect();

    let mut out = Vec::new();
    for i in 0..present.len() {
        for j in (i + 1)..present.len() {
            let (name_a, key_a) = present[i];
            let (name_b, key_b) = present[j];
            if key_a == key_b {
                continue;
            }
            // "at least as important" per this fact's semantics: rate — higher
            // is better; deadline — lower is better. Callers pass `key`
            // pre-oriented so "a_more_important iff key_a > key_b" always
            // holds here.
            let a_should_lead = key_a > key_b;
            let (higher_name, lower_name) = if a_should_lead {
                (name_a, name_b)
            } else {
                (name_b, name_a)
            };
            let higher_prio = prio[higher_name];
            let lower_prio = prio[lower_name];
            if higher_prio < lower_prio {
                out.push(Contradiction {
                    node_a: higher_name.to_string(),
                    node_b: lower_name.to_string(),
                    kind,
                });
            }
        }
    }
    out
}

/// Rate-vs-priority contradictions: a node with a strictly higher `rate_hz`
/// than another must not end up at a strictly lower final priority.
pub fn rate_priority_contradictions(input: &MapperInput, plan: &SchedPlan) -> Vec<Contradiction> {
    let prio = priority_by_node(plan);
    let iter = input
        .nodes
        .iter()
        .filter_map(|n| n.rate_hz.map(|r| (n.name.as_str(), r)));
    scan_contradictions(iter, &prio, "rate_hz")
}

/// Deadline-vs-priority contradictions: a node with a strictly shorter
/// `deadline_us` than another must not end up at a strictly lower final
/// priority. (Deadline is "smaller is more urgent", so the comparison key
/// fed to the shared scan is negated deadline.)
pub fn deadline_priority_contradictions(
    input: &MapperInput,
    plan: &SchedPlan,
) -> Vec<Contradiction> {
    let prio = priority_by_node(plan);
    let iter = input
        .nodes
        .iter()
        .filter_map(|n| n.deadline_us.map(|d| (n.name.as_str(), -(d as f64))));
    scan_contradictions(iter, &prio, "deadline_us")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        mapper::{MapperNode, RateMonotonicMapper, SchedMapper},
        platform::{PlatformResources, PosixResources},
        resolve::{ResolvedTier, ResolvedTierTable},
    };

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

    #[test]
    fn band_violations_flags_out_of_band_tiers() {
        let plan = ResolvedTierTable {
            tiers: vec![
                ResolvedTier {
                    name: "/a".to_string(),
                    priority: 50, // outside [10, 40]
                    members: vec!["/a".to_string()],
                    ..Default::default()
                },
                ResolvedTier {
                    name: "/b".to_string(),
                    priority: 20, // inside
                    members: vec!["/b".to_string()],
                    ..Default::default()
                },
            ],
        };
        let band = PriorityBand { min: 10, max: 40 };
        let violations = band_violations(&plan, &band);
        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].tier, "/a");
        assert_eq!(violations[0].priority, 50);
    }

    #[test]
    fn band_violations_ignores_default_tier() {
        let plan = ResolvedTierTable {
            tiers: vec![ResolvedTier {
                name: DEFAULT_TIER.to_string(),
                priority: 0, // outside [10, 40] but must not be flagged
                members: vec!["/bg".to_string()],
                ..Default::default()
            }],
        };
        let band = PriorityBand { min: 10, max: 40 };
        assert!(band_violations(&plan, &band).is_empty());
    }

    #[test]
    fn rate_contradiction_detected_when_priority_order_reversed() {
        // Build a plan by hand where the FASTER node has LOWER priority —
        // a hand-authored override could produce this; the mapper itself
        // never would.
        let plan = ResolvedTierTable {
            tiers: vec![
                ResolvedTier {
                    name: "/fast".to_string(),
                    priority: 10,
                    members: vec!["/fast".to_string()],
                    ..Default::default()
                },
                ResolvedTier {
                    name: "/slow".to_string(),
                    priority: 40,
                    members: vec!["/slow".to_string()],
                    ..Default::default()
                },
            ],
        };
        let input = MapperInput {
            nodes: vec![
                node("/fast", Some(100.0), None),
                node("/slow", Some(1.0), None),
            ],
            legacy: None,
            chains: Vec::new(),
        };
        let violations = rate_priority_contradictions(&input, &plan);
        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].node_a, "/fast");
        assert_eq!(violations[0].node_b, "/slow");
        assert_eq!(violations[0].kind, "rate_hz");
    }

    #[test]
    fn rate_monotonic_mapper_output_never_contradicts_itself() {
        // Sanity: the mapper's own derived plan must never contradict its
        // own input (it's monotonic by construction).
        let mapper = RateMonotonicMapper;
        let input = MapperInput {
            nodes: vec![
                node("/a", Some(10.0), None),
                node("/b", Some(50.0), None),
                node("/c", Some(100.0), None),
            ],
            legacy: None,
            chains: Vec::new(),
        };
        let facts = PlatformResources::Posix(PosixResources {
            rt_priority_band: Some(PriorityBand { min: 10, max: 40 }),
            isolated_cpus: vec![],
            rr_timeslice_us: None,
        });
        let plan = mapper.map(&input, &facts).expect("maps");
        assert!(rate_priority_contradictions(&input, &plan).is_empty());
    }

    #[test]
    fn deadline_contradiction_detected_when_priority_order_reversed() {
        let plan = ResolvedTierTable {
            tiers: vec![
                ResolvedTier {
                    name: "/tight".to_string(),
                    priority: 10, // should be high (tight deadline), but isn't
                    members: vec!["/tight".to_string()],
                    ..Default::default()
                },
                ResolvedTier {
                    name: "/loose".to_string(),
                    priority: 40,
                    members: vec!["/loose".to_string()],
                    ..Default::default()
                },
            ],
        };
        let input = MapperInput {
            nodes: vec![
                node("/tight", None, Some(1_000)),
                node("/loose", None, Some(100_000)),
            ],
            legacy: None,
            chains: Vec::new(),
        };
        let violations = deadline_priority_contradictions(&input, &plan);
        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].node_a, "/tight");
        assert_eq!(violations[0].node_b, "/loose");
        assert_eq!(violations[0].kind, "deadline_us");
    }

    #[test]
    fn no_contradiction_when_priorities_tie() {
        let plan = ResolvedTierTable {
            tiers: vec![ResolvedTier {
                name: "/both".to_string(),
                priority: 20,
                members: vec!["/a".to_string(), "/b".to_string()],
                ..Default::default()
            }],
        };
        let input = MapperInput {
            nodes: vec![node("/a", Some(100.0), None), node("/b", Some(1.0), None)],
            legacy: None,
            chains: Vec::new(),
        };
        // Equal priority is not treated as a contradiction (only a strict
        // reversal is).
        assert!(rate_priority_contradictions(&input, &plan).is_empty());
    }

    #[test]
    fn no_contradiction_for_nodes_missing_from_plan() {
        let plan = ResolvedTierTable { tiers: vec![] };
        let input = MapperInput {
            nodes: vec![node("/a", Some(100.0), None), node("/b", Some(1.0), None)],
            legacy: None,
            chains: Vec::new(),
        };
        assert!(rate_priority_contradictions(&input, &plan).is_empty());
    }
}
