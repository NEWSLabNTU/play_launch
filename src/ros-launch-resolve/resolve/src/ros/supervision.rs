//! Supervised budgets (phase 93).
//!
//! A budget states `supervise: { reaction | enters, mechanism }` beside its
//! `source:` (rlm v0.1.61): the runtime checks it while the system runs, and
//! a miss starts a fallback. This module is the checker's half:
//!
//! - [`supervised`] -- every supervised budget of the merged contracts, its
//!   reaction resolved to a path, and whether it lowers to a runtime monitor
//!   (a node path's input-triggered `max_latency`: the route monitor stamps
//!   the take of its first input and arms `due = take + budget`).
//! - [`detection`] -- the detection delay, by mechanism: `executor` is the
//!   tick + the longest callback on that executor (its measured `work`
//!   term) + the check (taken as 0); `timer` is the tick + the platform's
//!   switch. Both read `supervision: { tick, switch }`.
//! - [`overrun_rows`] -- per supervised budget a rung or floor row charges,
//!   a `RungRole::Overrun` row: the time before it + the budget + the
//!   detection + the reaction's route and settle, against the FTTI
//!   (`supervised-fit`).
//! - [`check`] -- `supervised-unlowered` (a supervised budget no monitor
//!   reads), `supervised-executor-long-callback` (an `executor` supervision
//!   whose detection a callback's overrun can delay past the tick) and
//!   `measured-unsupervised` (a measured budget nothing supervises); the
//!   last two at the severity of `policy.unsupervised_measured`.

use super::{
    manifest_loader::{
        ManifestIndex, ReactionWalk, ResolvedHazard, RungBudget, RungRole, TermKey, WalkStart,
        resolve_node_fqn, walk_reaction_at, walk_terms,
    },
    value_rules::{Settle, settle_for, site},
};
use ros_launch_manifest_check::{Diagnostic, Severity};
use ros_launch_manifest_types::{
    Bound, BudgetLevel, PolicySeverity, SourceKind, SuperviseMechanism, SuperviseReaction,
    Supervision, SupervisionPlatform,
};
use std::collections::HashSet;

/// The path a supervision's miss starts.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum ReactionTarget {
    /// A node path, `(node FQN, path)`.
    NodePath { node: String, path: String },
    /// A scope path of a manifest (an `enters:` mode's reaction too).
    ScopePath { scope_id: usize, name: String },
}

impl std::fmt::Display for ReactionTarget {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ReactionTarget::NodePath { node, path } => write!(f, "{node}/{path}"),
            ReactionTarget::ScopePath { name, .. } => write!(f, "scope path '{name}'"),
        }
    }
}

/// One supervised budget of the merged contracts.
#[derive(Debug, Clone)]
pub(crate) struct Supervised {
    pub term: TermKey,
    /// The node whose budget it is, by FQN, when it is a node's.
    pub node: Option<String>,
    /// The node path the key sits under (`nodes.n.paths.<p>.…`).
    pub path: Option<String>,
    pub budget_ms: f64,
    pub sv: Supervision,
    /// The reaction, or why it does not resolve.
    pub reaction: Result<ReactionTarget, String>,
    /// `Ok` when the budget lowers to a runtime monitor with its reaction;
    /// otherwise why not.
    pub lowered: Result<(), String>,
}

/// `(node name, path name, the key's rest)` of `nodes.<n>.paths.<p>.<rest>`,
/// with `<p>` matched against the node's declared paths (a path name may
/// hold a dot).
fn split_node_path<'a>(
    m: &'a ros_launch_manifest_types::Manifest,
    key: &'a str,
) -> Option<(&'a str, &'a str, &'a str)> {
    let rest = key.strip_prefix("nodes.")?;
    let (n, rest) = rest.split_once('.')?;
    let rest = rest.strip_prefix("paths.")?;
    let nd = m.nodes.get(n)?;
    nd.paths.keys().find_map(|p| {
        rest.strip_prefix(p.as_str())
            .and_then(|r| r.strip_prefix('.'))
            .map(|r| (n, p.as_str(), r))
    })
}

/// Every supervised budget, in a stable order (scope, then key).
pub(crate) fn supervised(index: &ManifestIndex) -> Vec<Supervised> {
    let mut ms: Vec<_> = index.manifests.values().collect();
    ms.sort_by_key(|m| m.scope_id);
    let mut out = Vec::new();
    for m in ms {
        for (key, b, sv) in m.manifest.supervised() {
            let Some(budget_ms) = b.ms() else {
                continue;
            };
            let at = split_node_path(&m.manifest, &key);
            let node_name = key
                .strip_prefix("nodes.")
                .and_then(|r| r.split_once('.'))
                .map(|(n, _)| n);
            let node = node_name.map(|n| resolve_node_fqn(index, m.scope_id, &m.ns, n));
            let path = at.map(|(_, p, _)| p.to_string());
            let scope_path = |name: &str| {
                index
                    .scope_paths
                    .iter()
                    .any(|p| p.scope_id == m.scope_id && p.path_name == name)
                    .then(|| ReactionTarget::ScopePath {
                        scope_id: m.scope_id,
                        name: name.to_string(),
                    })
            };
            let reaction = match &sv.reaction {
                SuperviseReaction::Reaction(name) => node
                    .as_ref()
                    .filter(|n| {
                        index
                            .node_paths
                            .iter()
                            .any(|p| &p.node_fqn == *n && &p.path_name == name)
                    })
                    .map(|n| ReactionTarget::NodePath {
                        node: n.clone(),
                        path: name.clone(),
                    })
                    .or_else(|| scope_path(name))
                    .ok_or_else(|| match &node {
                        Some(n) => format!(
                            "its reaction '{name}' is neither a path of {n} nor a scope path"
                        ),
                        None => format!("its reaction '{name}' is not a scope path"),
                    }),
                SuperviseReaction::Enters(mode) => {
                    match index
                        .modes
                        .iter()
                        .find(|md| md.scope_id == m.scope_id && &md.name == mode)
                    {
                        None => Err(format!("it enters '{mode}', which is not a mode")),
                        Some(md) => match &md.decl.reaction {
                            Some(r) => scope_path(r).ok_or_else(|| {
                                format!(
                                    "it enters '{mode}', whose reaction '{r}' is not a scope path"
                                )
                            }),
                            None => Err(format!(
                                "it enters '{mode}', which names no `reaction:` scope path to run"
                            )),
                        },
                    }
                }
            };
            let lowered = match (&node, at) {
                (Some(n), Some((_, p, "max_latency"))) => {
                    let input = index.node_paths.iter().any(|np| {
                        &np.node_fqn == n
                            && np.path_name == p
                            && matches!(
                                np.path.effective_trigger(),
                                ros_launch_manifest_types::EffectiveTrigger::Input(_)
                            )
                    });
                    match (&reaction, input) {
                        (Err(why), _) => Err(why.clone()),
                        (Ok(_), false) => Err(format!(
                            "{n}/{p} is not input-triggered, and the route monitor stamps the \
                             take of a path's first input"
                        )),
                        (Ok(_), true) => Ok(()),
                    }
                }
                _ => Err(
                    "only a node path's `max_latency` lowers to a runtime monitor (the route \
                     deadline); nothing checks this budget while the system runs"
                        .to_string(),
                ),
            };
            out.push(Supervised {
                term: (m.scope_id, key),
                node,
                path,
                budget_ms,
                sv: sv.clone(),
                reaction,
                lowered,
            });
        }
    }
    out
}

/// The lowering of node path `(node, path)`'s supervised `max_latency`, when
/// it lowers: the deadline, the reaction and the mechanism.
pub(crate) fn lowered_route(
    index: &ManifestIndex,
    node: &str,
    path: &str,
) -> Option<(f64, ReactionTarget, SuperviseMechanism)> {
    supervised(index).into_iter().find_map(|s| {
        let here = s.node.as_deref() == Some(node)
            && s.path.as_deref() == Some(path)
            && s.term.1.ends_with(".max_latency");
        match (here, s.lowered, s.reaction) {
            (true, Ok(()), Ok(r)) => Some((s.budget_ms, r, s.sv.mechanism)),
            _ => None,
        }
    })
}

/// The platform terms that apply to scope `scope_id`: its own
/// `supervision:`, else the root contract's.
fn platform(index: &ManifestIndex, scope_id: usize) -> Option<&SupervisionPlatform> {
    let own = index
        .manifests
        .get(&scope_id)
        .and_then(|m| m.manifest.supervision.as_ref());
    own.or_else(|| {
        let mut ms: Vec<_> = index.manifests.values().collect();
        ms.sort_by_key(|m| m.scope_id);
        ms.into_iter().find_map(|m| m.manifest.supervision.as_ref())
    })
}

/// The detection delay of a supervised budget, ms, and how it divides; or
/// why it is unknown.
pub(crate) fn detection(index: &ManifestIndex, s: &Supervised) -> Result<(f64, String), String> {
    let p = platform(index, s.term.0);
    let tick = p
        .and_then(|p| p.tick)
        .map(|d| d.as_millis_f64())
        .ok_or_else(|| {
            "the detection delay is unknown: state the monitor's tick (`supervision: { tick }` \
             at the contract's top)"
                .to_string()
        })?;
    match s.sv.mechanism {
        SuperviseMechanism::Timer => {
            let switch = p
                .and_then(|p| p.switch)
                .map(|d| d.as_millis_f64())
                .ok_or_else(|| {
                    "the detection delay is unknown: a `timer` supervision adds the interrupt \
                     and thread switch, `supervision: { switch }`"
                        .to_string()
                })?;
            Ok((
                tick + switch,
                format!("tick {tick:.2}ms + switch {switch:.2}ms (timer)"),
            ))
        }
        SuperviseMechanism::Executor => {
            let node = s.node.as_ref().ok_or_else(|| {
                "an `executor` supervision is checked in a node's spin loop, and this budget \
                 is not a node's"
                    .to_string()
            })?;
            let (work, on) = longest_callback(index, node)?;
            Ok((
                tick + work,
                format!(
                    "tick {tick:.2}ms + the longest callback on {node}'s executor {work:.2}ms \
                     ({on}'s `work`) + the check (taken as 0) (executor)"
                ),
            ))
        }
    }
}

/// The longest callback on `node`'s executor: the largest measured `work`
/// term over its paths, sized as `budget-rule` sizes it, or an analysed
/// path's `max_latency`. A path with neither makes it unknown.
fn longest_callback(index: &ManifestIndex, node: &str) -> Result<(f64, String), String> {
    let mut best: Option<(f64, String)> = None;
    for p in index.node_paths.iter().filter(|p| p.node_fqn == node) {
        let work = p
            .path
            .max_latency
            .as_ref()
            .and_then(|b| b.source.as_ref())
            .and_then(|s| s.measured.as_ref())
            .and_then(|m| {
                m.terms
                    .iter()
                    .find(|t| t.name.as_deref() == Some("work"))
                    .map(|t| t.budget_ms(m.margin))
            });
        // An analysed `max_latency` bounds the callback with it.
        let work = work.or_else(|| {
            p.path
                .max_latency
                .as_ref()
                .filter(|b| b.level() == Some(BudgetLevel::Bound))
                .and_then(Bound::ms)
        });
        let Some(w) = work else {
            return Err(format!(
                "the detection delay is unknown: an `executor` supervision waits for the longest \
                 callback on {node}'s executor, and {node}/{} states neither a measured `work` \
                 term (`max_latency: {{ source: {{ measured: {{ terms: {{ work: ... }} }} }} }}`) \
                 nor an analysed `max_latency`",
                p.path_name
            ));
        };
        if best.as_ref().is_none_or(|(b, _)| w > *b) {
            best = Some((w, p.path_name.clone()));
        }
    }
    best.ok_or_else(|| format!("the detection delay is unknown: {node} declares no path"))
}

/// What an overrun row needs from the row it is built on.
pub(crate) struct RowCtx<'a> {
    pub h: &'a ResolvedHazard,
    pub rung: &'a str,
    pub ftti_ms: Option<f64>,
    pub fdti_ms: Option<f64>,
    pub waited_ms: f64,
    pub walk: Option<&'a ReactionWalk>,
    pub after_deadline: bool,
    /// The stated budgets before the route: detection, windows waited.
    pub prior_terms: Vec<TermKey>,
    /// Every output topic of the hazard's rungs: where a node path's
    /// reaction may land.
    pub hazard_sinks: &'a HashSet<String>,
    /// The topics, services and graph, for the reaction walk.
    pub topics: &'a std::collections::BTreeMap<String, super::manifest_loader::ResolvedTopic>,
    pub services: &'a std::collections::BTreeMap<String, super::manifest_loader::ResolvedService>,
    pub graph: &'a super::manifest_graph::GlobalDataflowGraph,
}

/// A supervised budget charged on a row, and when it starts.
struct Charged<'a> {
    s: &'a Supervised,
    /// Time charged before the budget starts, from the fault.
    before_ms: f64,
    /// The stated budgets up to and including this one.
    terms: Vec<TermKey>,
}

/// The `RungRole::Overrun` rows of one rung or floor row (phase 93), and
/// their `supervised-fit` verdicts.
pub(crate) fn overrun_rows(
    index: &ManifestIndex,
    ctx: &RowCtx<'_>,
    budgets: &mut Vec<RungBudget>,
    diags: &mut Vec<Diagnostic>,
) {
    let (Some(walk), Some(fdti)) = (ctx.walk, ctx.fdti_ms) else {
        return;
    };
    let all = supervised(index);
    if all.is_empty() {
        return;
    }
    let charged_terms = walk_terms(index, walk, ctx.after_deadline);
    let link = walk
        .link
        .as_ref()
        .filter(|_| !ctx.after_deadline)
        .map_or(0.0, |l| l.ms);
    let start = fdti + ctx.waited_ms;
    // The terms of `walk_terms` up to (and including) `key`.
    let upto = |key: &TermKey| -> Vec<TermKey> {
        let mut v: Vec<TermKey> = Vec::new();
        for (k, _) in &charged_terms {
            v.push(k.clone());
            if k == key {
                break;
            }
        }
        v
    };
    let mut charged: Vec<Charged<'_>> = Vec::new();
    let mut elapsed = start + link;
    for hop in &walk.hops {
        let at_hop = elapsed + hop.sampling_ms.unwrap_or(0.0);
        if let Some(s) = all.iter().find(|s| {
            s.node.as_deref() == Some(hop.node.as_str())
                && s.path.as_deref() == Some(hop.path.as_str())
                && s.term.1.ends_with(".max_latency")
        }) {
            charged.push(Charged {
                s,
                before_ms: at_hop,
                terms: upto(&s.term),
            });
        }
        elapsed += hop.ms;
    }
    if let Some(at) = &walk.settle
        && let Some(s) = all.iter().find(|s| {
            s.node.as_deref() == Some(at.node.as_str())
                && s.path.as_deref() == Some(at.path.as_str())
                && s.term.1.ends_with(".safe_state.settle.response")
        })
    {
        let route = walk.charged(ctx.after_deadline).0;
        charged.push(Charged {
            s,
            before_ms: start + route,
            terms: upto(&s.term),
        });
    }
    for c in charged {
        let s = c.s;
        let key = &s.term.1;
        let path = format!("{key}.supervise");
        let where_ = site(index, s.term.0, key);
        let row_name = format!("{} ({key})", ctx.rung);
        let fail = |why: String, budgets: &mut Vec<RungBudget>, diags: &mut Vec<Diagnostic>| {
            budgets.push(RungBudget {
                hazard: ctx.h.name.clone(),
                rung: row_name.clone(),
                role: RungRole::Overrun,
                fdti_ms: Some(fdti),
                windows_ms: ctx.waited_ms,
                ftti_ms: ctx.ftti_ms,
                note: format!("overrun of {key}: {why}"),
                terms: [ctx.prior_terms.clone(), c.terms.clone()].concat(),
                ..Default::default()
            });
            diags.push(Diagnostic {
                rule_id: "supervised-fit".to_string(),
                severity: Severity::Error,
                message: format!(
                    "{where_}: hazard '{}': `{key}` is supervised, but its overrun case cannot \
                     be checked: {why}",
                    ctx.h.name
                ),
                path: path.clone(),
                span: None,
            });
        };
        let (det, det_why) = match detection(index, s) {
            Ok(d) => d,
            Err(why) => {
                fail(why, budgets, diags);
                continue;
            }
        };
        let target = match &s.reaction {
            Ok(t) => t.clone(),
            Err(why) => {
                fail(why.clone(), budgets, diags);
                continue;
            }
        };
        let (rwalk, scope_route) = match &target {
            ReactionTarget::NodePath { node, path } => {
                let sinks: HashSet<&str> = ctx.hazard_sinks.iter().map(String::as_str).collect();
                (
                    walk_reaction_at(
                        WalkStart::Path(node, path),
                        &sinks,
                        ctx.topics,
                        ctx.services,
                        &index.node_paths,
                        ctx.graph,
                    ),
                    None,
                )
            }
            ReactionTarget::ScopePath { scope_id, name } => {
                let sp = index
                    .scope_paths
                    .iter()
                    .find(|p| p.scope_id == *scope_id && &p.path_name == name);
                match sp {
                    None => (None, None),
                    Some(sp) => {
                        let sinks: HashSet<&str> =
                            sp.output_topics.iter().map(String::as_str).collect();
                        (
                            walk_reaction_at(
                                WalkStart::Topics(&sp.input_topics),
                                &sinks,
                                ctx.topics,
                                ctx.services,
                                &index.node_paths,
                                ctx.graph,
                            ),
                            sp.path.max_latency.as_ref().and_then(Bound::ms),
                        )
                    }
                }
            }
        };
        let (route_ms, settle, rterms, first_node) = match (&rwalk, scope_route) {
            (Some(w), _) => {
                let settle = match &w.settle {
                    Some(at) => {
                        let mut sink: Vec<Diagnostic> = Vec::new();
                        settle_for(
                            index,
                            &index.launch_params,
                            ctx.h,
                            ctx.rung,
                            &at.node,
                            &at.path,
                            &at.ss,
                            &mut sink,
                        )
                    }
                    None => Settle::default(),
                };
                (
                    w.route_ms,
                    settle,
                    walk_terms(index, w, true)
                        .into_iter()
                        .map(|(k, _)| k)
                        .collect::<Vec<_>>(),
                    w.hops.first().map(|h| h.node.clone()),
                )
            }
            (None, Some(d)) => (d, Settle::default(), Vec::new(), None),
            (None, None) => {
                fail(
                    format!("its reaction {target} reaches none of the hazard's safe states"),
                    budgets,
                    diags,
                );
                continue;
            }
        };
        let settle_ms = settle.ms.unwrap_or(0.0);
        let total = c.before_ms + s.budget_ms + det + route_ms + settle_ms;
        let hang = s.sv.mechanism == SuperviseMechanism::Executor
            && first_node.is_some()
            && first_node == s.node;
        let hang_note = if hang {
            format!(
                "; the reaction runs on {}'s executor, the one the check runs on, so this \
                 covers slowness, not a hang",
                s.node.as_deref().unwrap_or_default()
            )
        } else {
            String::new()
        };
        let settle_note = if settle.ms.is_none() {
            " (the reaction's settle is unknown, charged 0)".to_string()
        } else {
            String::new()
        };
        let arith = format!(
            "before it {:.2}ms + budget {:.2}ms + detection {det:.2}ms ({det_why}) + reaction \
             {target} {route_ms:.2}ms + settle {settle_ms:.2}ms{settle_note} = {total:.2}ms",
            c.before_ms, s.budget_ms
        );
        budgets.push(RungBudget {
            hazard: ctx.h.name.clone(),
            rung: row_name.clone(),
            role: RungRole::Overrun,
            fdti_ms: Some(fdti),
            windows_ms: ctx.waited_ms,
            route_ms: Some(c.before_ms - start + s.budget_ms + det + route_ms),
            settle_ms: settle.ms,
            settle_how: settle.how,
            settle_response_ms: settle.response_ms,
            total_ms: Some(total),
            ftti_ms: ctx.ftti_ms,
            note: format!("overrun of {key}: {arith}{hang_note}"),
            terms: [ctx.prior_terms.clone(), c.terms.clone(), rterms].concat(),
            ..Default::default()
        });
        let Some(ftti) = ctx.ftti_ms else {
            continue;
        };
        let (severity, verdict) = if total > ftti {
            (
                Severity::Error,
                format!("exceeds the fault-tolerant time interval {ftti:.2}ms"),
            )
        } else {
            (
                Severity::Info,
                format!(
                    "fits the fault-tolerant time interval {ftti:.2}ms with {:.2}ms of slack",
                    ftti - total
                ),
            )
        };
        diags.push(Diagnostic {
            rule_id: "supervised-fit".to_string(),
            severity,
            message: format!(
                "{where_}: hazard '{}', rung '{}': if `{key}` overruns, {arith} {verdict}{hang_note}",
                ctx.h.name, ctx.rung
            ),
            path,
            span: None,
        });
    }
}

/// The level of the stated budget `term` (phase 93).
pub(crate) fn level_of(index: &ManifestIndex, term: &TermKey) -> Option<BudgetLevel> {
    let m = index.manifests.get(&term.0)?;
    if let Some((_, b)) = m.manifest.bounds().into_iter().find(|(k, _)| *k == term.1) {
        return b.level();
    }
    // A timer's rate (`...trigger.timer.rate_hz`) states a source and no
    // supervision.
    m.manifest
        .budget_sources()
        .get(&term.1)
        .and_then(|s| BudgetLevel::of(Some(s), None))
}

/// The severity of `policy.unsupervised_measured` for scope `scope_id`: its
/// own `policy:`, else the root contract's; warning by default.
fn policy_severity(index: &ManifestIndex, scope_id: usize) -> Severity {
    let policy = index
        .manifests
        .get(&scope_id)
        .and_then(|m| m.manifest.policy.as_ref())
        .or_else(|| {
            let mut ms: Vec<_> = index.manifests.values().collect();
            ms.sort_by_key(|m| m.scope_id);
            ms.into_iter().find_map(|m| m.manifest.policy.as_ref())
        })
        .and_then(|p| p.unsupervised_measured)
        .unwrap_or_default();
    match policy {
        PolicySeverity::Warn => Severity::Warning,
        PolicySeverity::Error => Severity::Error,
    }
}

/// `supervised-executor-long-callback`: an `executor` supervision is checked
/// in the spin loop, between callbacks. Its detection is the tick + the
/// longest callback on that executor, and that is a bound only when every
/// callback on it is: a callback with no analysed bound can run past it, and
/// the check waits for it. When the supervised path's own callback is the
/// one that overruns, the late output is published before the check runs,
/// so the miss is seen at completion, late by the overrun. `mechanism:
/// timer` is checked off the executor and catches both.
fn executor_long_callback(index: &ManifestIndex, s: &Supervised) -> Option<Diagnostic> {
    if s.sv.mechanism != SuperviseMechanism::Executor {
        return None;
    }
    let node = s.node.as_deref()?;
    let own = s
        .path
        .as_deref()
        .filter(|_| s.term.1.ends_with(".max_latency"));
    let mut why: Vec<String> = Vec::new();
    let mut others: Vec<String> = Vec::new();
    for p in index.node_paths.iter().filter(|p| p.node_fqn == node) {
        let level = p.path.max_latency.as_ref().and_then(|b| b.level());
        if level == Some(BudgetLevel::Bound) {
            continue;
        }
        let what = match level {
            Some(BudgetLevel::Supervised) => "supervised only",
            Some(BudgetLevel::Unsupervised) => "measured only",
            _ if p.path.max_latency.is_none() => "unstated",
            _ => "not analysed",
        };
        if own == Some(p.path_name.as_str()) {
            why.push(format!(
                "its own callback, {node}/{}, has no analysed bound ({what}) and can itself \
                 overrun: the late output is published before the overdue check runs, so the \
                 miss is seen only when the callback completes, late by the overrun",
                p.path_name
            ));
        } else {
            others.push(format!("{} ({what})", p.path_name));
        }
    }
    if !others.is_empty() {
        why.push(format!(
            "its detection, the tick + the longest callback on {node}'s executor, is not \
             bounded: a callback with no analysed bound can run past it ({})",
            others.join(", ")
        ));
    }
    if why.is_empty() {
        return None;
    }
    let key = &s.term.1;
    Some(Diagnostic {
        rule_id: "supervised-executor-long-callback".to_string(),
        severity: policy_severity(index, s.term.0),
        message: format!(
            "{}: `{key}` is supervised with `mechanism: executor`, which is checked in the spin \
             loop between callbacks; {}. Use `mechanism: timer` (an independent timer and \
             thread, `supervision: {{ tick, switch }}`), or state an analysed bound for every \
             callback on the executor (`source: analysed`)",
            site(index, s.term.0, key),
            why.join("; and ")
        ),
        path: format!("{key}.supervise.mechanism"),
        span: None,
    })
}

/// `supervised-unlowered`, `supervised-executor-long-callback` and
/// `measured-unsupervised` (phase 93).
pub(crate) fn check(index: &mut ManifestIndex) {
    let mut diags: Vec<Diagnostic> = Vec::new();
    for s in supervised(index) {
        if let Err(why) = &s.lowered {
            diags.push(Diagnostic {
                rule_id: "supervised-unlowered".to_string(),
                severity: Severity::Error,
                message: format!(
                    "{}: `{}` is supervised, but no runtime monitor checks it: {why}. A \
                     supervised budget is a claim that a miss is caught and handled; drop \
                     `supervise:` or supervise the route that carries this budget",
                    site(index, s.term.0, &s.term.1),
                    s.term.1
                ),
                path: format!("{}.supervise", s.term.1),
                span: None,
            });
        }
    }
    for s in supervised(index) {
        if let Some(d) = executor_long_callback(index, &s) {
            diags.push(d);
        }
    }
    for b in &index.stated {
        let term: TermKey = (b.scope_id, b.key.clone());
        if level_of(index, &term) != Some(BudgetLevel::Unsupervised) {
            continue;
        }
        debug_assert!(
            b.source
                .as_ref()
                .is_some_and(|s| s.kind == SourceKind::Measured)
        );
        let severity = policy_severity(index, b.scope_id);
        diags.push(Diagnostic {
            rule_id: "measured-unsupervised".to_string(),
            severity,
            message: format!(
                "{}: `{}: {:.2}ms` is measured and nothing supervises it: an observed maximum, \
                 not a bound, and a run that exceeds it goes unnoticed. Supervise it \
                 (`supervise: {{ reaction, mechanism }}`), analyse it (`source: analysed`), or \
                 accept it (`policy: {{ unsupervised_measured: warn }}`, the default)",
                site(index, b.scope_id, &b.key),
                b.key,
                b.ms
            ),
            path: b.key.clone(),
            span: None,
        });
    }
    index.merge_diagnostics.extend(diags);
}
