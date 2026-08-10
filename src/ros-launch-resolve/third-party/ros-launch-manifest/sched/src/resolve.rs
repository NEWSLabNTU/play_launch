//! Selector-based tier resolution.

use std::collections::{BTreeMap, BTreeSet};

use crate::types::{AssignRule, TierDef};

/// The synthesized tier for nodes matched by no assign rule.
pub const DEFAULT_TIER: &str = "default";

/// One node as seen by the resolver — dependency-free (no parser types).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SchedNode {
    /// Fully-qualified node name (e.g. `/perception/lidar/ndt_localizer`).
    pub name: String,
    /// The node's namespace / scope path (e.g. `/perception/lidar`).
    pub scope: String,
}

/// Errors from parsing or resolving the scheduling spec.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SchedError {
    #[error("failed to parse system scheduling TOML: {0}")]
    Parse(String),
    #[error("assign rule node selector `{selector}` matches no node in the system")]
    UnknownNodeSelector { selector: String },
    #[error("assign rule scope selector `{selector}` matches no node's scope in the system")]
    UnknownScopeSelector { selector: String },
    #[error(
        "node `{node}` is matched by two tiers (`{tier_a}` and `{tier_b}`); \
         a node must belong to exactly one tier"
    )]
    NodeMatchedByMultipleTiers {
        node: String,
        tier_a: String,
        tier_b: String,
    },
    #[error("assign rule references tier `{tier}`, which has no [tiers.{tier}] definition")]
    UnknownTier { tier: String },
    #[error("tier `{tier}` has no [tiers.{tier}.{target}] sub-table for the resolve target")]
    MissingPlatformSpec { tier: String, target: String },
    #[error("tier `{tier}`: {source}")]
    Posix {
        tier: String,
        #[source]
        source: crate::posix::PosixError,
    },
}

/// Normalize a namespace/scope path: ensure a single leading slash, no trailing.
fn norm_scope(s: &str) -> String {
    let trimmed = s.trim_matches('/');
    if trimmed.is_empty() {
        "/".to_string()
    } else {
        format!("/{trimmed}")
    }
}

/// A node selector matches a node by full name or by its bare (last-segment) name.
fn node_selector_matches(selector: &str, node: &SchedNode) -> bool {
    node.name == selector || node.name.rsplit('/').next() == Some(selector)
}

/// A scope selector matches a node whose scope equals it or is a descendant.
fn scope_selector_matches(selector: &str, node: &SchedNode) -> bool {
    let sel = norm_scope(selector);
    let ns = norm_scope(&node.scope);
    ns == sel || ns.starts_with(&format!("{}/", sel.trim_end_matches('/')))
}

/// Insert a node→tier assignment; error if it conflicts with a prior one.
fn assign_one<'a>(
    map: &mut BTreeMap<String, &'a str>,
    node: &str,
    tier: &'a str,
) -> Result<(), SchedError> {
    if let Some(prev) = map.insert(node.to_string(), tier)
        && prev != tier
    {
        return Err(SchedError::NodeMatchedByMultipleTiers {
            node: node.to_string(),
            tier_a: prev.to_string(),
            tier_b: tier.to_string(),
        });
    }
    Ok(())
}

/// Build tier-name → sorted member node names. Unmatched nodes → `DEFAULT_TIER`.
pub(crate) fn bind_nodes(
    assigns: &[AssignRule],
    nodes: &[SchedNode],
) -> Result<BTreeMap<String, Vec<String>>, SchedError> {
    let mut node_tier: BTreeMap<String, &str> = BTreeMap::new();
    // Tracks nodes assigned by an explicit `nodes` rule (pass 1 only).
    // Scope rules (pass 2) skip these without error; scope-vs-scope conflicts
    // are detected by `assign_one` since the node is in `node_tier` but NOT here.
    let mut explicit_nodes: BTreeSet<String> = BTreeSet::new();

    // Pass 1: explicit node selectors (highest precedence).
    for rule in assigns {
        for sel in &rule.nodes {
            let hits: Vec<&SchedNode> = nodes
                .iter()
                .filter(|n| node_selector_matches(sel, n))
                .collect();
            if hits.is_empty() {
                return Err(SchedError::UnknownNodeSelector {
                    selector: sel.clone(),
                });
            }
            for n in hits {
                assign_one(&mut node_tier, &n.name, &rule.tier)?;
                explicit_nodes.insert(n.name.clone());
            }
        }
    }

    // Pass 2: scope selectors.
    // - Nodes claimed by an explicit node rule (pass 1) are skipped silently: explicit wins.
    // - Nodes claimed by a prior scope rule raise NodeMatchedByMultipleTiers via assign_one.
    for rule in assigns {
        if let Some(scope_sel) = &rule.scope {
            let hits: Vec<&SchedNode> = nodes
                .iter()
                .filter(|n| scope_selector_matches(scope_sel, n))
                .collect();
            if hits.is_empty() {
                return Err(SchedError::UnknownScopeSelector {
                    selector: scope_sel.clone(),
                });
            }
            for n in hits {
                if explicit_nodes.contains(&n.name) {
                    continue; // explicit node rule wins
                }
                assign_one(&mut node_tier, &n.name, &rule.tier)?;
            }
        }
    }

    // Collect members, defaulting the unmatched.
    let mut members: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for n in nodes {
        let tier = node_tier
            .get(&n.name)
            .copied()
            .unwrap_or(DEFAULT_TIER)
            .to_string();
        members.entry(tier).or_default().push(n.name.clone());
    }
    for v in members.values_mut() {
        v.sort();
    }
    Ok(members)
}

/// One resolved tier: generic policy + concrete placement for one target.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ResolvedTier {
    pub name: String,
    pub priority: i64,
    pub stack_bytes: Option<u32>,
    pub core: Option<u32>,
    pub sched_class: Option<String>,
    pub preempt_threshold: Option<i64>,
    pub class: Option<String>,
    pub period_us: Option<u64>,
    pub budget_us: Option<u64>,
    /// Effective deadline: platform override if present, else the generic head.
    pub deadline_us: Option<u64>,
    pub deadline_policy: Option<String>,
    pub spin_period_us: Option<u64>,
    /// Typed `posix` placement (Phase 60).
    ///
    /// Additive beside the `sched_class`/`priority`/`core` trio above, which
    /// stays readable-but-deprecated for one release. When present this is
    /// authoritative: it is the only form that can express `SCHED_DEADLINE`,
    /// a multi-CPU mask, a cpuset partition, or a uclamp range, and it cannot
    /// represent the illegal combinations the trio could (a priority on
    /// `SCHED_OTHER`, an affinity mask on `SCHED_DEADLINE`).
    pub posix: Option<crate::posix::PosixPlacement>,
    /// Member node names, sorted.
    pub members: Vec<String>,
}

impl ResolvedTier {
    fn default_tier(members: Vec<String>) -> Self {
        ResolvedTier {
            name: DEFAULT_TIER.to_string(),
            members,
            ..Default::default()
        }
    }
}

/// The ordered tier table for one target.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ResolvedTierTable {
    /// Highest RTOS/OS priority first.
    pub tiers: Vec<ResolvedTier>,
}

impl ResolvedTierTable {
    /// True when the whole system collapsed to the single synthesized default.
    pub fn is_single_tier(&self) -> bool {
        self.tiers.len() == 1 && self.tiers[0].name == DEFAULT_TIER
    }
}

/// Resolve tiers + assign rules against the system's nodes for one target.
pub fn resolve(
    tiers: &std::collections::BTreeMap<String, TierDef>,
    assigns: &[AssignRule],
    nodes: &[SchedNode],
    target: &str,
) -> Result<ResolvedTierTable, SchedError> {
    // Every assign rule must name a real tier (the synthesized default excepted).
    for rule in assigns {
        if rule.tier != DEFAULT_TIER && !tiers.contains_key(&rule.tier) {
            return Err(SchedError::UnknownTier {
                tier: rule.tier.clone(),
            });
        }
    }

    let members_by_tier = bind_nodes(assigns, nodes)?;

    let mut out: Vec<ResolvedTier> = Vec::with_capacity(members_by_tier.len());
    for (name, members) in members_by_tier {
        // The default tier needs no [tiers.default] table.
        if name == DEFAULT_TIER && !tiers.contains_key(DEFAULT_TIER) {
            out.push(ResolvedTier::default_tier(members));
            continue;
        }
        let def = tiers
            .get(&name)
            .ok_or_else(|| SchedError::UnknownTier { tier: name.clone() })?;
        let spec = def
            .platform(target)
            .ok_or_else(|| SchedError::MissingPlatformSpec {
                tier: name.clone(),
                target: target.to_string(),
            })?;
        // Lower the stringly-typed trio into the typed placement. Only the
        // `posix` target has one — every other target keeps its own
        // vocabulary, which this crate does not interpret.
        let posix = if matches!(target, "posix" | "native") {
            Some(
                posix_placement_from_spec(spec).map_err(|source| SchedError::Posix {
                    tier: name.clone(),
                    source,
                })?,
            )
        } else {
            None
        };

        out.push(ResolvedTier {
            name,
            posix,
            priority: spec.priority,
            stack_bytes: spec.stack_bytes,
            core: spec.core,
            sched_class: spec.sched_class.clone(),
            preempt_threshold: spec.preempt_threshold,
            class: def.class.clone(),
            period_us: def.period_us,
            budget_us: def.budget_us,
            deadline_us: spec.deadline_us.or(def.deadline_us),
            deadline_policy: def.deadline_policy.clone(),
            spin_period_us: def.spin_period_us,
            members,
        });
    }

    // Highest priority first; ties by name for determinism.
    out.sort_by(|a, b| b.priority.cmp(&a.priority).then(a.name.cmp(&b.name)));
    Ok(ResolvedTierTable { tiers: out })
}

/// Lower a `[tiers.<name>.posix]` sub-table into the typed placement.
///
/// This is where the v1/legacy path stops failing open: an unrecognized
/// `sched_class` used to become `SCHED_OTHER` silently, so `SCHED_FIF0`
/// dropped a node out of real-time with no diagnostic. It is now an error
/// naming the six legal values.
///
/// An absent `sched_class` still means `SCHED_OTHER` — that is a real default,
/// not a parse failure, and it is what an unmatched node has always gotten.
fn posix_placement_from_spec(
    spec: &crate::types::TierPlatformSpec,
) -> Result<crate::posix::PosixPlacement, crate::posix::PosixError> {
    use crate::posix::{PosixAffinity, PosixPolicyKind, PosixSched};

    let kind = match spec.sched_class.as_deref() {
        Some(s) => PosixPolicyKind::parse(s)?,
        None => PosixPolicyKind::Other,
    };

    // `priority` is `i64` to admit RTOS negative cooperative priorities, so it
    // is narrowed here rather than at the schema. Out-of-range values are
    // caught by `PosixPlacement::validate`, which reports the real bound.
    let priority = spec.priority.clamp(i32::MIN as i64, i32::MAX as i64) as i32;

    let sched = match kind {
        PosixPolicyKind::Fifo => PosixSched::Fifo { priority },
        PosixPolicyKind::Rr => PosixSched::Rr { priority },
        PosixPolicyKind::Idle => PosixSched::Idle,
        PosixPolicyKind::Batch => PosixSched::Batch { nice: 0 },
        PosixPolicyKind::Other => PosixSched::Other { nice: 0 },
        PosixPolicyKind::Deadline => {
            // v1 cannot author a reservation: it has no per-tier runtime, and
            // substituting one would invent a number. Deriving SCHED_DEADLINE
            // is the v2 mapper's job.
            return Err(crate::posix::PosixError::UnknownSchedClass {
                value: "SCHED_DEADLINE (not authorable in the v1 tier schema \
                        — use a v2 platform file with a declared budget)"
                    .to_string(),
            });
        }
    };

    let affinity = match spec.core {
        Some(cpu) => PosixAffinity::Cpus { cpus: vec![cpu] },
        None => PosixAffinity::Inherit,
    };

    crate::posix::PosixPlacement::new(sched, affinity)
}

#[cfg(test)]
mod bind_tests {
    use super::*;

    fn node(name: &str, scope: &str) -> SchedNode {
        SchedNode {
            name: name.to_string(),
            scope: scope.to_string(),
        }
    }

    fn rule_nodes(tier: &str, names: &[&str]) -> AssignRule {
        AssignRule {
            tier: tier.to_string(),
            nodes: names.iter().map(|s| s.to_string()).collect(),
            scope: None,
        }
    }

    fn rule_scope(tier: &str, scope: &str) -> AssignRule {
        AssignRule {
            tier: tier.to_string(),
            nodes: vec![],
            scope: Some(scope.to_string()),
        }
    }

    #[test]
    fn unmatched_nodes_go_to_default() {
        let nodes = vec![node("/a", "/"), node("/b", "/")];
        let m = bind_nodes(&[], &nodes).unwrap();
        assert_eq!(m.len(), 1);
        assert_eq!(m[DEFAULT_TIER], vec!["/a".to_string(), "/b".to_string()]);
    }

    #[test]
    fn explicit_node_selector_by_bare_name() {
        let nodes = vec![node("/ns/ndt_localizer", "/ns"), node("/ns/other", "/ns")];
        let m = bind_nodes(&[rule_nodes("control", &["ndt_localizer"])], &nodes).unwrap();
        assert_eq!(m["control"], vec!["/ns/ndt_localizer".to_string()]);
        assert_eq!(m[DEFAULT_TIER], vec!["/ns/other".to_string()]);
    }

    #[test]
    fn scope_selector_matches_subtree() {
        let nodes = vec![
            node("/perception/lidar/a", "/perception/lidar"),
            node("/perception/lidar/deep/b", "/perception/lidar/deep"),
            node("/control/c", "/control"),
        ];
        let m = bind_nodes(&[rule_scope("perception", "/perception/lidar")], &nodes).unwrap();
        assert_eq!(
            m["perception"],
            vec![
                "/perception/lidar/a".to_string(),
                "/perception/lidar/deep/b".to_string()
            ]
        );
        assert_eq!(m[DEFAULT_TIER], vec!["/control/c".to_string()]);
    }

    #[test]
    fn explicit_node_wins_over_scope() {
        let nodes = vec![node("/p/a", "/p"), node("/p/b", "/p")];
        let assigns = vec![rule_scope("bg", "/p"), rule_nodes("control", &["/p/a"])];
        let m = bind_nodes(&assigns, &nodes).unwrap();
        assert_eq!(m["control"], vec!["/p/a".to_string()]);
        assert_eq!(m["bg"], vec!["/p/b".to_string()]);
    }

    #[test]
    fn conflicting_node_rules_error() {
        let nodes = vec![node("/a", "/")];
        let assigns = vec![rule_nodes("hi", &["/a"]), rule_nodes("lo", &["/a"])];
        let err = bind_nodes(&assigns, &nodes).unwrap_err();
        assert!(matches!(err, SchedError::NodeMatchedByMultipleTiers { .. }));
    }

    #[test]
    fn scope_scope_conflict_errors() {
        // Two scope rules for different tiers, both matching the same node — must error.
        let nodes = vec![node("/p/a", "/p"), node("/p/b", "/p")];
        let assigns = vec![rule_scope("hi", "/p"), rule_scope("lo", "/p")];
        let err = bind_nodes(&assigns, &nodes).unwrap_err();
        assert!(matches!(err, SchedError::NodeMatchedByMultipleTiers { .. }));
    }

    #[test]
    fn scope_selector_false_prefix_excluded() {
        // Selector "/per" must NOT match namespace "/perception" — boundary proof.
        let nodes = vec![node("/perception/a", "/perception")];
        let err = bind_nodes(&[rule_scope("hi", "/per")], &nodes).unwrap_err();
        assert!(matches!(err, SchedError::UnknownScopeSelector { .. }));
    }

    #[test]
    fn unknown_node_selector_errors() {
        let nodes = vec![node("/a", "/")];
        let err = bind_nodes(&[rule_nodes("hi", &["ghost"])], &nodes).unwrap_err();
        assert!(matches!(err, SchedError::UnknownNodeSelector { .. }));
    }

    #[test]
    fn unknown_scope_selector_errors() {
        let nodes = vec![node("/a", "/")];
        let err = bind_nodes(&[rule_scope("hi", "/nowhere")], &nodes).unwrap_err();
        assert!(matches!(err, SchedError::UnknownScopeSelector { .. }));
    }
}

#[cfg(test)]
mod resolve_tests {
    use super::*;
    use crate::parse::parse_system_sched;

    fn node(name: &str, scope: &str) -> SchedNode {
        SchedNode {
            name: name.to_string(),
            scope: scope.to_string(),
        }
    }

    #[test]
    fn no_assigns_degenerates_to_single_default_tier() {
        let sched = parse_system_sched("").unwrap();
        let table = resolve(&sched.tiers, &sched.assign, &[node("/a", "/")], "posix").unwrap();
        assert!(table.is_single_tier());
        assert_eq!(table.tiers[0].members, vec!["/a".to_string()]);
        assert_eq!(table.tiers[0].priority, 0);
    }

    #[test]
    fn resolves_platform_numbers_and_effective_deadline() {
        let src = r#"
[tiers.control]
class = "real_time"
deadline_us = 50000

[tiers.control.posix]
priority = 80
sched_class = "SCHED_FIFO"
core = 1
deadline_us = 40000

[[assign]]
tier = "control"
nodes = ["/a"]
"#;
        let sched = parse_system_sched(src).unwrap();
        let table = resolve(&sched.tiers, &sched.assign, &[node("/a", "/")], "posix").unwrap();
        let t = table.tiers.iter().find(|t| t.name == "control").unwrap();
        assert_eq!(t.priority, 80);
        assert_eq!(t.core, Some(1));
        assert_eq!(t.class.as_deref(), Some("real_time"));
        assert_eq!(t.deadline_us, Some(40000)); // platform override wins
        assert_eq!(t.members, vec!["/a".to_string()]);
    }

    #[test]
    fn orders_tiers_highest_priority_first() {
        let src = r#"
[tiers.hi.posix]
priority = 80
[tiers.lo.posix]
priority = 10

[[assign]]
tier = "hi"
nodes = ["/a"]
[[assign]]
tier = "lo"
nodes = ["/b"]
"#;
        let sched = parse_system_sched(src).unwrap();
        let nodes = vec![node("/a", "/"), node("/b", "/")];
        let table = resolve(&sched.tiers, &sched.assign, &nodes, "posix").unwrap();
        assert_eq!(table.tiers[0].name, "hi");
        assert_eq!(table.tiers[0].priority, 80);
        assert_eq!(table.tiers[1].name, "lo");
    }

    #[test]
    fn unknown_tier_in_assign_errors() {
        let src = r#"
[[assign]]
tier = "ghost"
nodes = ["/a"]
"#;
        let sched = parse_system_sched(src).unwrap();
        let err = resolve(&sched.tiers, &sched.assign, &[node("/a", "/")], "posix").unwrap_err();
        assert!(matches!(err, SchedError::UnknownTier { .. }));
    }

    #[test]
    fn missing_platform_spec_errors() {
        let src = r#"
[tiers.hi.freertos]
priority = 5

[[assign]]
tier = "hi"
nodes = ["/a"]
"#;
        let sched = parse_system_sched(src).unwrap();
        // resolving for posix must fail: only freertos declared.
        let err = resolve(&sched.tiers, &sched.assign, &[node("/a", "/")], "posix").unwrap_err();
        assert!(matches!(err, SchedError::MissingPlatformSpec { .. }));
    }

    #[test]
    fn deadline_falls_back_to_generic_head_when_platform_omits_it() {
        let src = r#"
[tiers.rt]
deadline_us = 50000

[tiers.rt.posix]
priority = 40

[[assign]]
tier = "rt"
nodes = ["/a"]
"#;
        let sched = parse_system_sched(src).unwrap();
        let table = resolve(&sched.tiers, &sched.assign, &[node("/a", "/")], "posix").unwrap();
        let t = table.tiers.iter().find(|t| t.name == "rt").unwrap();
        // Platform sub-table has no deadline_us → falls back to the generic head's 50000.
        assert_eq!(t.deadline_us, Some(50000));
    }

    #[test]
    fn ties_broken_by_name() {
        let src = r#"
[tiers.b.posix]
priority = 50
[tiers.a.posix]
priority = 50

[[assign]]
tier = "b"
nodes = ["/b_node"]
[[assign]]
tier = "a"
nodes = ["/a_node"]
"#;
        let sched = parse_system_sched(src).unwrap();
        let nodes = vec![node("/a_node", "/"), node("/b_node", "/")];
        let table = resolve(&sched.tiers, &sched.assign, &nodes, "posix").unwrap();
        // Equal priority → sorted by name ascending.
        assert_eq!(table.tiers[0].name, "a");
        assert_eq!(table.tiers[1].name, "b");
    }
}
