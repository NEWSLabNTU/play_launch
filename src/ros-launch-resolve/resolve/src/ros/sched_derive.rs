//! Build a `SchedMapper` [`MapperInput`] from a launch dump plus a resolved
//! contract index (Phase 41.2 — the v2 derive→override→validate pipeline's
//! extraction stage).
//!
//! Extraction rules (design `2026-07-16-rt-config-v2-design.md` §4, brief
//! `.superpowers/sdd/p41-w2-brief.md` deliverable 1):
//!
//! - `rate_hz`: the **max** of every declared rate fact across the node's
//!   own publications — both the topic-level `rate_hz` (on every topic the
//!   node publishes) and the endpoint-level `pub.<ep>.min_rate_hz` (on the
//!   node's own manifest entry, same scope). Using the max (rather than min
//!   or first-match) means the node's derived priority reflects its
//!   *fastest* obligation — the rate that would starve first if it were
//!   under-prioritized.
//! - `deadline_us` / `path_budget_ms`: both come from the **same** source —
//!   every node-path (`nodes.<name>.paths.*`) owned by the node — taking the
//!   **min** `max_latency_ms` across those paths (the tightest budget is the
//!   most urgent), converted to microseconds (`× 1000`, rounded) for
//!   `deadline_us` and kept as milliseconds for `path_budget_ms`.
//! - `criticality`: read from the new `nodes.<name>.criticality` contract
//!   field (`high`/`medium`/`low`, case-insensitive). Absent or unrecognized
//!   values map to `None` (ignore-if-absent, per the brief) — this is an
//!   advisory hint, not a schema-enforced value.
//!
//! Nodes with no scope id (e.g. the synthetic single-node dump built by
//! `play_launch run`) or no matching contract entry get a bare [`MapperNode`]
//! (`name`/`scope` only) — built-in mappers already default fact-less nodes
//! to the non-RT default tier, so this is not a special case here.

use std::collections::{BTreeMap, BTreeSet};

use ros_launch_manifest_derive::{DeriveFacts, DeriveReport, mapper_input_from_model};
use ros_launch_manifest_sched::{
    ChainElement, ChainSemantics, Criticality, MapperInput, MapperNode, MapperPath, ResolvedChain,
    SystemSched,
};

use crate::ros::{
    launch_dump::LaunchDump,
    manifest_loader::ManifestIndex,
    sched_loader::{ScheduledRecord, scheduled_records_from_dump},
};
// The tests below still spell the mirror type; the production code reads it
// through `model_builder::convert_trigger` since phase 78 W1.
#[cfg(test)]
use ros_launch_manifest_sched::EffectiveTrigger;

/// Build the mapper's input the way both consumers do since phase 78 W2
/// (design issue #52): resolve the model of `dump` plus `index` and hand
/// `ros_launch_manifest_derive::mapper_input_from_model` that model and the
/// platform file's per-node budgets as [`DeriveFacts`]. This is what
/// `derive_sched_plan` calls; [`mapper_input_from_dump`] is the derivation
/// it replaces, kept until W3 deletes it so the `tests` module can hold the
/// two against each other on every contract fixture.
///
/// `legacy` is set afterwards, as the crate documents: only the `.toml`
/// bridge's `manual` mapper reads it.
pub fn mapper_input_via_model(
    dump: &LaunchDump,
    index: Option<&ManifestIndex>,
    legacy: Option<SystemSched>,
    budgets: &BTreeMap<String, u64>,
) -> (MapperInput, DeriveReport) {
    let model = pre_sched_model(dump, index);
    let (mut input, report) = mapper_input_from_model(&model, &derive_facts_from_budgets(budgets));
    input.legacy = legacy;
    // `MapperNode::scope` is the NAMESPACE the `manual` mapper's `[[assign]]
    // scope = "/perception"` selector matches against
    // (`ros_launch_manifest_sched::resolve::scope_selector_matches`), and the
    // one the dump-side derivation filled from the record's effective
    // namespace. The crate copies `NodeInstance::scope`, which is the model's
    // FILE-SCOPE key (`bringup.launch.xml`), so a `.toml` bridge user's scope
    // rule would match nothing. Until the crate carries the namespace, it is
    // read off the node's own key: `structure.nodes` is keyed by FQN, and an
    // FQN's parent is exactly the effective namespace the record had.
    for node in &mut input.nodes {
        node.scope = namespace_of(&node.name);
    }
    (input, report)
}

/// The model the derivation reads: structure and contracts of `dump` under
/// `index`, with no execution layer. `derive_sched_plan` builds it before the
/// plan exists, and the plan is what the execution layer is made of; the
/// caller's full model (provenance, args, execution) is built afterwards by
/// `build_checked_model`, from the same inputs. A run with no contract index
/// (`play_launch run`'s synthetic single-node dump) derives from an empty
/// one, which is the model `resolve` emits for a tree with no contracts.
pub fn pre_sched_model(
    dump: &LaunchDump,
    index: Option<&ManifestIndex>,
) -> ros_launch_manifest_model::SystemModel {
    let empty;
    let index = match index {
        Some(index) => index,
        None => {
            empty = ManifestIndex::default();
            &empty
        }
    };
    super::model_builder::build_system_model(
        dump,
        index,
        None,
        BTreeMap::new(),
        &BTreeSet::new(),
        None,
    )
}

/// The platform file's `budget` overrides as the crate's per-node fact, in
/// milliseconds. `budgets` already carries every spelling a selector may
/// take (`posix_budgets`: the selector, its slashed form and its bare name),
/// and the crate looks a node up by FQN and then by bare name, the same
/// fallback [`budget_us_for`] applies. No per-path fact exists here: a
/// platform file speaks of nodes.
pub fn derive_facts_from_budgets(budgets: &BTreeMap<String, u64>) -> DeriveFacts {
    DeriveFacts {
        path_exec_ms: BTreeMap::new(),
        node_exec_ms: budgets
            .iter()
            .map(|(selector, us)| (selector.clone(), *us as f64 / 1000.0))
            .collect(),
    }
}

/// The namespace an FQN lives in: `/perception/sensor_node` -> `/perception`,
/// `/talker` -> `/`. The inverse of `sched_loader::join_fqn`.
fn namespace_of(fqn: &str) -> String {
    match fqn.rsplit_once('/') {
        Some((ns, _)) if !ns.is_empty() => ns.to_string(),
        _ => "/".to_string(),
    }
}

/// Build the mapper's input from a launch dump and (optionally) a resolved
/// contract index. `legacy` is threaded straight through to
/// [`MapperInput::legacy`] — non-`None` only when driving the `manual`
/// mapper via the `.toml` bridge.
///
/// The derivation phase 78 replaces: `derive_sched_plan` calls
/// [`mapper_input_via_model`] instead, and the `tests` module asserts the two
/// agree on every contract fixture until W3 deletes this one.
pub fn mapper_input_from_dump(
    dump: &LaunchDump,
    index: Option<&ManifestIndex>,
    legacy: Option<SystemSched>,
    budgets: &BTreeMap<String, u64>,
) -> MapperInput {
    let nodes = scheduled_records_from_dump(dump)
        .iter()
        .map(|r| build_mapper_node(r, index, budgets))
        .collect();
    // Every chain the mapper sees is DERIVED. `chains:`/`segments:` were
    // removed in phase 68 W4 — a written route was a second copy of the graph
    // — so there is no authored spelling left to prefer one over.
    let chains = index
        .map(|i| {
            let graph = super::manifest_graph::build_global_graph(i);
            resolve_chains_derived(i, &graph, budgets)
        })
        .unwrap_or_default();
    MapperInput {
        nodes,
        legacy,
        chains,
    }
}

fn build_mapper_node(
    record: &ScheduledRecord,
    index: Option<&ManifestIndex>,
    budgets: &BTreeMap<String, u64>,
) -> MapperNode {
    let Some(index) = index else {
        return MapperNode {
            name: record.fqn.clone(),
            scope: record.scope_ns.clone(),
            ..Default::default()
        };
    };

    let rate_hz = extract_rate_hz(record, index);
    let (path_budget_ms, deadline_us) = extract_path_facts(record, index);
    let criticality = extract_criticality(record, index);
    let paths = extract_paths(record, index, budgets);

    MapperNode {
        name: record.fqn.clone(),
        scope: record.scope_ns.clone(),
        rate_hz,
        deadline_us,
        criticality,
        path_budget_ms,
        paths,
        claims_concurrency: node_decl(record, index)
            .map(claims_concurrency)
            .unwrap_or(false),
    }
}

/// Every declared causal path this node owns (Phase 44.4 §2), translated
/// from `ros_launch_manifest_types::PathDecl`/`EffectiveTrigger` (W1) into
/// the sched crate's dependency-free mirror types
/// ([`ros_launch_manifest_sched::MapperPath`]/[`ros_launch_manifest_sched::EffectiveTrigger`]).
/// `inputs` uses the *effective* trigger's endpoint list (matching the same
/// source-of-truth fix `chain_checks::resolve_segment` applies — the raw
/// `path.input` field is empty whenever the author used the explicit
/// `trigger: { input: [...] }` form); `outputs` is the raw declared list
/// (always populated, either form).
fn extract_paths(
    record: &ScheduledRecord,
    index: &ManifestIndex,
    budgets: &BTreeMap<String, u64>,
) -> Vec<MapperPath> {
    // A declared budget is per NODE, and `MapperPath::exec_ms` is per PATH, so
    // the two only line up when the node has exactly one path. Where it has
    // several the split is genuinely unknown — `play_launch measure` documents
    // its own emitted budget as the SUM of the per-path maxima — and attributing
    // that sum to any one path would overstate it. Absent is the honest answer,
    // and the `chain-sampling-feasibility` diagnostic already reports absent
    // cost as "feasible ON INCOMPLETE EVIDENCE" rather than as feasible.
    let path_count = index
        .node_paths
        .iter()
        .filter(|p| p.node_fqn == record.fqn)
        .count();
    let node_exec_ms = if path_count == 1 {
        budget_us_for(budgets, &record.fqn).map(|us| us as f64 / 1000.0)
    } else {
        None
    };

    // The spellings the model lowers (phase 78 W1) and the shared derivation
    // carries, so the parity gate can compare the whole input: endpoints as
    // qualified refs (`<node FQN>/<endpoint>`), inside the trigger and in
    // `inputs`/`outputs` alike, and a node's paths in name order (the
    // model's `contracts.node_paths` is a sorted map).
    let qualify = |e: &String| format!("{}/{e}", record.fqn);
    let mut paths: Vec<MapperPath> = index
        .node_paths
        .iter()
        .filter(|p| p.node_fqn == record.fqn)
        .map(|p| {
            let effective = match p.path.effective_trigger() {
                ros_launch_manifest_types::EffectiveTrigger::Input(eps) => {
                    ros_launch_manifest_types::EffectiveTrigger::Input(
                        eps.iter().map(qualify).collect(),
                    )
                }
                other => other,
            };
            let inputs = match &effective {
                ros_launch_manifest_types::EffectiveTrigger::Input(eps) => eps.clone(),
                _ => Vec::new(),
            };
            MapperPath {
                name: p.path_name.clone(),
                effective_trigger: super::model_builder::convert_trigger(effective),
                max_latency_ms: p.path.max_latency.map(|d| d.as_millis_f64()),
                // Cost, from the platform file's declared `budget` for this
                // node. The comment that used to sit here said the vocabulary
                // declared no execution-time fact — true when it was written,
                // and stale since phase 60 made cost authorable. The slot was
                // hard-coded `None` for long enough that phase 58's design pass
                // named it as the blocker for proportional deadline
                // decomposition, which needs a per-hop cost to distribute slack
                // against.
                exec_ms: node_exec_ms,
                inputs,
                outputs: p.path.output.iter().map(qualify).collect(),
                max_jitter_ms: p.path.max_jitter.map(|d| d.as_millis_f64()),
                miss: p.path.miss.as_ref().map(convert_miss),
            }
        })
        .collect();
    paths.sort_by(|a, b| a.name.cmp(&b.name));
    paths
}

/// Translate the contract's `miss:` into the mapper's mirror of it.
pub(crate) fn convert_miss(
    m: &ros_launch_manifest_types::MissSpec,
) -> ros_launch_manifest_sched::MapperMiss {
    use ros_launch_manifest_sched::{MapperMiss, MapperMissAction};
    use ros_launch_manifest_types::MissAction as A;
    MapperMiss {
        tolerate_n: m.tolerate.as_ref().map(|c| c.n),
        tolerate_w: m.tolerate.as_ref().map(|c| c.w),
        consecutive: m.consecutive,
        action: m.action.map(|a| match a {
            A::Continue => MapperMissAction::Continue,
            A::SkipNext => MapperMissAction::SkipNext,
            A::Abort => MapperMissAction::Abort,
        }),
    }
}

/// Does this node claim that some of its callbacks may run concurrently?
///
/// One bit, derived the same way `GlobalNode::exclusion_groups` derives them:
/// an absent `concurrency:` means every path serialises (`false`), and a
/// declaration claims concurrency whenever it does NOT put every declared path
/// into a single group. `exclusive: []` therefore claims full concurrency,
/// which is the opposite of omitting the section — a distinction the parser
/// preserves and this must not flatten.
fn claims_concurrency(node: &ros_launch_manifest_types::NodeDecl) -> bool {
    let Some(decl) = &node.concurrency else {
        return false;
    };
    if node.paths.len() <= 1 {
        // One path cannot contend with itself, whatever is declared.
        return false;
    }
    // Merge declared sets that share a member, exactly as the graph does.
    let mut groups: Vec<std::collections::BTreeSet<&str>> = Vec::new();
    for declared in &decl.exclusive {
        let mut merged: std::collections::BTreeSet<&str> =
            declared.iter().map(|s| s.as_str()).collect();
        groups.retain(|g| {
            if g.is_disjoint(&merged) {
                true
            } else {
                merged.extend(g.iter().copied());
                false
            }
        });
        if !merged.is_empty() {
            groups.push(merged);
        }
    }
    // Concurrency is claimed unless one group covers every declared path.
    !groups
        .iter()
        .any(|g| node.paths.keys().all(|p| g.contains(p.as_str())))
}

/// Build `ResolvedChain`s from **derived** routes rather than from authored
/// `chains:`/`segments:`. A derived route is the one
/// `check_scope_path_critical_path` already computes for a scope path.
///
/// This is the seam phase 68 W1 left open. W1 taught the CHECKER to derive a
/// route from `trigger`/`output`; the mapper went on reading authored
/// segments, so the two knew different things about the same system and
/// removing `segments:` did not move the derivation to a new source — it
/// removed the mapper's only source. Measured on `rt_workspace`: with
/// `chains:` deleted the mapper reported `non-chain` provenance and fell back
/// to ranking by budget, which on that three-node fixture coincidentally
/// produced the same priorities. One derivation, one place, two consumers is
/// the design; this is the second consumer.
///
/// `semantics` is `Reaction` for every derived route, because `PathDecl` has
/// no `semantics` field yet — moving it there is `contract-axes.md` §3.6, the
/// one thing a scope path still cannot say that a chain can.
pub(crate) fn resolve_chains_derived(
    index: &ManifestIndex,
    graph: &super::manifest_graph::GlobalDataflowGraph,
    budgets: &BTreeMap<String, u64>,
) -> Vec<ResolvedChain> {
    use super::manifest_graph::{critical_path, subgraph_for_scope_path, subtree_scope_ids};
    use ros_launch_manifest_types::EffectiveTrigger as T;

    let mut out = Vec::new();
    for sp in &index.scope_paths {
        let Some(max_latency_ms) = sp.path.max_latency.map(|d| d.as_millis_f64()) else {
            continue;
        };
        let subtree = subtree_scope_ids(index, sp.scope_id);
        let subgraph = subgraph_for_scope_path(graph, subtree, &sp.input_topics, &sp.output_topics);
        let Some(cp) = critical_path(&subgraph) else {
            continue;
        };

        let mut elements: Vec<ChainElement> = Vec::new();
        let mut criticality = Criticality::Low;
        for (node_fqn, path_name) in &cp.vertices {
            let Some(path_name) = path_name else {
                // A hop no declared path accounts for carries no trigger fact,
                // so it can be neither a boundary nor a segment link. Skipping
                // it keeps the route honest rather than inventing a category.
                continue;
            };
            if let Some(c) = node_criticality(index, sp.scope_id, node_fqn)
                && c > criticality
            {
                criticality = c;
            }
            let decl = graph
                .nodes
                .get(node_fqn)
                .and_then(|n| n.paths.get(path_name));
            let trigger = decl.map(|d| d.effective_trigger());
            match trigger {
                Some(T::Timer { rate_hz }) if rate_hz > 0.0 => {
                    // The one-path rule `extract_paths` applies to
                    // `MapperPath::exec_ms`, applied to the boundary too
                    // (phase 78 W2, seam 1). This used to hand a node's
                    // budget to the boundary whatever the path count, so a
                    // node with two timer paths and a 2 ms budget was
                    // counted at period + 2 ms here and at period + 0 on
                    // the path; the shared crate counts both at zero and
                    // says so (`ChainFeasibleWithoutWcet`).
                    let path_count = index
                        .node_paths
                        .iter()
                        .filter(|p| &p.node_fqn == node_fqn)
                        .count();
                    let exec_ms = (path_count == 1)
                        .then(|| budget_us_for(budgets, node_fqn))
                        .flatten()
                        .map(|us| us as f64 / 1000.0);
                    elements.push(ChainElement::Boundary {
                        node: node_fqn.clone(),
                        path: path_name.clone(),
                        period_ms: 1000.0 / rate_hz,
                        exec_ms,
                    });
                }
                _ => push_segment_node(&mut elements, node_fqn.clone(), path_name.clone()),
            }
        }
        if elements.is_empty() {
            continue;
        }
        out.push(ResolvedChain {
            name: sp.path_name.clone(),
            criticality,
            max_latency_ms,
            semantics: ChainSemantics::Reaction,
            elements,
        });
    }
    out
}

/// Resolve one chain's segments into the alternating `Segment`/`Boundary`
/// decomposition (design "Model: clock-segmented chains"): `via` segments
/// are connectivity-only (already validated by `chain-link`) and don't
/// appear in the result; consecutive non-boundary path segments merge into
/// one `Segment` (declaration order IS source-to-sink topo order — a chain's
/// `segments:` list is already linearized by the author); a `Timer`
/// (`rate_hz > 0`) path segment becomes its own `Boundary` — the same
/// boundary-vs-non-boundary split `chain_checks::check_one_chain` already
/// uses for `sampling_cost`.
/// Look up a declared budget by full FQN, falling back to the bare node name.
///
/// Override selectors may be either form — `[[assign]].nodes` has always
/// accepted both — while a chain element carries the full FQN. Without the
/// fallback a perfectly good `overrides: { sensor_node: { budget_us: 2000 } }`
/// silently fails to reach the chain, and the feasibility verdict reports
/// "no measured WCET" for a cost that was in fact declared.
fn budget_us_for(budgets: &BTreeMap<String, u64>, node_fqn: &str) -> Option<u64> {
    budgets.get(node_fqn).copied().or_else(|| {
        node_fqn
            .rsplit('/')
            .next()
            .filter(|bare| !bare.is_empty())
            .and_then(|bare| budgets.get(bare).copied())
    })
}

/// Append `(node, path)` to the last `ChainElement::Segment` run if the
/// previous element was also a `Segment`, else start a new one — merges
/// consecutive non-boundary path segments into one `Segment` element, in
/// declaration order.
fn push_segment_node(elements: &mut Vec<ChainElement>, node: String, path: String) {
    let entry = ros_launch_manifest_sched::SegmentNode { node, path };
    if let Some(ChainElement::Segment {
        nodes_in_topo_order,
    }) = elements.last_mut()
    {
        nodes_in_topo_order.push(entry);
    } else {
        elements.push(ChainElement::Segment {
            nodes_in_topo_order: vec![entry],
        });
    }
}

/// `nodes.<bare_name>.criticality` for a specific (scope, node_fqn) pair —
/// the same lookup [`extract_criticality`] does for a [`ScheduledRecord`],
/// generalized to any node identity (chain segments don't carry a
/// `ScheduledRecord`, only a resolved `(scope_id, node_fqn)`).
fn node_criticality(index: &ManifestIndex, scope_id: usize, node_fqn: &str) -> Option<Criticality> {
    // Phase 72: the hazards decide first; the label where none reaches, and
    // (phase 78 W2, seam 2) where the one that reaches buckets to the
    // no-requirement level. The model carries no `node_criticality` entry
    // for either, so the shared derivation reads the label for both; this
    // used to return `None` for the second and emit nothing.
    if let Some(c) = index
        .derived_criticality
        .get(node_fqn)
        .and_then(|d| d.bucket())
    {
        return Some(c);
    }
    let bare = node_fqn.rsplit('/').next()?;
    let resolved = index.manifests.get(&scope_id)?;
    let raw = resolved.manifest.nodes.get(bare)?.criticality.as_deref()?;
    parse_criticality(raw)
}

/// The label's bucket, for comparing a declaration against the derivation.
pub(crate) fn parse_criticality_label(raw: &str) -> Option<Criticality> {
    parse_criticality(raw)
}

/// Case-insensitive `high`/`medium`/`low` -> [`Criticality`] parse, shared by
/// [`extract_criticality`] and [`node_criticality`].
fn parse_criticality(raw: &str) -> Option<Criticality> {
    match raw.to_ascii_lowercase().as_str() {
        "high" => Some(Criticality::High),
        "medium" => Some(Criticality::Medium),
        "low" => Some(Criticality::Low),
        other => {
            tracing::debug!("sched: unrecognized criticality `{other}` — ignored");
            None
        }
    }
}

/// The node's own contract declaration (`nodes.<bare_name>`), if this
/// record's scope has a loaded manifest and that manifest declares it.
fn node_decl<'a>(
    record: &ScheduledRecord,
    index: &'a ManifestIndex,
) -> Option<&'a ros_launch_manifest_types::NodeDecl> {
    let scope_id = record.scope_id?;
    let resolved = index.manifests.get(&scope_id)?;
    resolved.manifest.nodes.get(&record.bare_name)
}

/// The fastest timer trigger among the node's declared paths, and nothing
/// else (phase 78, "Rates"): the rate that would starve first if the node
/// were under-prioritised. Authored `topics.<t>.rate_hz`, the graph's
/// derived topic rate and `pub.<ep>.min_rate_hz` are PROMISES the monitors
/// read; no mapper does. This used to take the max over all three, so a
/// node two hops downstream of the only timer ranked as if it had a period
/// of its own, and a node whose only rate fact was a promise ranked at all.
/// The shared derivation applies this rule, and the parity gate holds this
/// copy to it until W3 deletes it.
fn extract_rate_hz(record: &ScheduledRecord, index: &ManifestIndex) -> Option<f64> {
    use ros_launch_manifest_types::EffectiveTrigger as T;
    index
        .node_paths
        .iter()
        .filter(|p| p.node_fqn == record.fqn)
        .filter_map(|p| match p.path.effective_trigger() {
            T::Timer { rate_hz } if rate_hz > 0.0 => Some(rate_hz),
            _ => None,
        })
        .fold(None, |acc: Option<f64>, v| {
            Some(acc.map_or(v, |a| a.max(v)))
        })
}

/// `(path_budget_ms, deadline_us)`: the tightest (min) `max_latency_ms`
/// across every node-path this node owns, as milliseconds and (rounded)
/// microseconds respectively. `None` when the node owns no path with a
/// declared budget.
fn extract_path_facts(
    record: &ScheduledRecord,
    index: &ManifestIndex,
) -> (Option<f64>, Option<u64>) {
    let mut min_ms = index
        .node_paths
        .iter()
        .filter(|p| p.node_fqn == record.fqn)
        .filter_map(|p| p.path.max_latency.map(|d| d.as_millis_f64()))
        .fold(None, |acc: Option<f64>, v| {
            Some(acc.map_or(v, |a: f64| a.min(v)))
        });

    // A service server's `max_response` is a deadline too, and until now it
    // was the only declared timing fact the mapper could not see: a node whose
    // contract said `srv: { lookup: { max_response: 5ms } }` and nothing else
    // reported `default (no timing facts)` and landed on SCHED_OTHER at
    // priority 0 — a 5 ms requirement scheduled as though none had been
    // written. It joins the same tightest-wins fold as a path budget, because
    // it is the same kind of claim: a bound on how long this node may take to
    // produce an answer.
    if let Some(decl) = node_decl(record, index) {
        for props in decl.srv.values() {
            if let Some(ms) = props.max_response.map(|d| d.as_millis_f64()) {
                min_ms = Some(min_ms.map_or(ms, |a: f64| a.min(ms)));
            }
        }
    }

    let deadline_us = min_ms.map(|ms| (ms * 1000.0).round() as u64);
    (min_ms, deadline_us)
}

/// `nodes.<name>.criticality`, case-insensitive, ignore-if-absent-or-unrecognized.
fn extract_criticality(record: &ScheduledRecord, index: &ManifestIndex) -> Option<Criticality> {
    // Phase 72: derived from hazards where any reaches this node; the label
    // where none does or where the bucket is the no-requirement level (phase
    // 78 W2, seam 2 -- see `node_criticality`).
    if let Some(c) = index
        .derived_criticality
        .get(&record.fqn)
        .and_then(|d| d.bucket())
    {
        return Some(c);
    }
    let raw = node_decl(record, index)?.criticality.as_deref()?;
    parse_criticality(raw)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::manifest_loader::{
        ContractChannel, ResolvedManifest, ResolvedNodePath, ResolvedScopePath, ResolvedTopic,
    };
    use ros_launch_manifest_types::{Manifest, NodeDecl, PathDecl};
    use std::collections::BTreeMap;

    fn dump_with_two_nodes() -> LaunchDump {
        let json = serde_json::json!({
            "node": [
                {
                    "executable": "talker",
                    "name": "talker",
                    "exec_name": "talker",
                    "params_files": [],
                    "cmd": [],
                    "scope": 0
                },
                {
                    "executable": "listener",
                    "name": "listener",
                    "exec_name": "listener",
                    "params_files": [],
                    "cmd": [],
                    "scope": 0
                }
            ],
            "load_node": [],
            "container": [],
            "lifecycle_node": [],
            "file_data": {},
            "scopes": [
                {
                    "id": 0,
                    "ns": "/",
                    "parent": null,
                    "origin": {"file": "manifest.launch.xml", "path": "/nowhere/manifest.launch.xml"}
                }
            ]
        });
        serde_json::from_value(json).expect("valid LaunchDump")
    }

    fn empty_resolved_manifest(scope_id: usize, manifest: Manifest) -> ResolvedManifest {
        ResolvedManifest {
            scope_id,
            pkg: None,
            file: "manifest.launch.xml".to_string(),
            ns: "/".to_string(),
            channel: ContractChannel::Provider,
            contract_path: std::path::PathBuf::new(),
            manifest,
            source: String::new(),
            diagnostics: vec![],
        }
    }

    #[test]
    fn no_index_gives_bare_records() {
        let dump = dump_with_two_nodes();
        let input = mapper_input_from_dump(&dump, None, None, &BTreeMap::new());
        assert_eq!(input.nodes.len(), 2);
        for n in &input.nodes {
            assert_eq!(n.rate_hz, None);
            assert_eq!(n.deadline_us, None);
            assert_eq!(n.criticality, None);
            assert_eq!(n.path_budget_ms, None);
        }
        assert!(input.legacy.is_none());
    }

    /// Phase 78: a node's rate is its fastest timer trigger. The topic's
    /// authored `rate_hz` (100), its derived rate (80) and the publisher's
    /// `min_rate_hz` (30) are promises no mapper reads; before this the max
    /// of the three, 100, was the rate, and a node with no timer at all
    /// ranked on a promise.
    #[test]
    fn rate_hz_is_the_fastest_timer_trigger_and_never_a_promise() {
        let dump = dump_with_two_nodes();

        let tick = PathDecl {
            trigger: Some(ros_launch_manifest_types::Trigger::Timer { rate_hz: 50.0 }),
            output: vec!["chatter".to_string()],
            ..Default::default()
        };
        let slow = PathDecl {
            trigger: Some(ros_launch_manifest_types::Trigger::Timer { rate_hz: 5.0 }),
            output: vec!["chatter".to_string()],
            ..Default::default()
        };
        let mut nodes = BTreeMap::new();
        nodes.insert(
            "talker".to_string(),
            NodeDecl {
                publishers: {
                    let mut m = BTreeMap::new();
                    m.insert(
                        "chatter".to_string(),
                        ros_launch_manifest_types::EndpointProps {
                            min_rate_hz: Some(30.0),
                            ..Default::default()
                        },
                    );
                    m
                },
                paths: BTreeMap::from([
                    ("slow".to_string(), slow.clone()),
                    ("tick".to_string(), tick.clone()),
                ]),
                ..Default::default()
            },
        );
        // The listener publishes on a topic that promises a rate, and has no
        // timer: no rate.
        nodes.insert(
            "listener".to_string(),
            NodeDecl {
                publishers: {
                    let mut m = BTreeMap::new();
                    m.insert(
                        "echo".to_string(),
                        ros_launch_manifest_types::EndpointProps {
                            min_rate_hz: Some(30.0),
                            ..Default::default()
                        },
                    );
                    m
                },
                ..Default::default()
            },
        );
        let manifest = Manifest {
            version: 1,
            nodes,
            ..Default::default()
        };

        let mut index = ManifestIndex::default();
        index
            .manifests
            .insert(0, empty_resolved_manifest(0, manifest));
        for (name, path) in [("slow", slow), ("tick", tick)] {
            index.node_paths.push(ResolvedNodePath {
                node_fqn: "/talker".to_string(),
                path_name: name.to_string(),
                path,
                scope_id: 0,
            });
        }
        for (fqn, publisher) in [("/chatter", "/talker/chatter"), ("/echo", "/listener/echo")] {
            index.topics.insert(
                fqn.to_string(),
                ResolvedTopic {
                    derived_from_remaps: false,
                    fqn: fqn.to_string(),
                    msg_type: "std_msgs/msg/String".to_string(),
                    qos: None,
                    publishers: vec![publisher.to_string()],
                    subscribers: vec![],
                    rate_hz: Some(100.0),
                    derived_rate_hz: Some(80.0),
                    max_transport_ms: None,
                    drop: None,
                    scope_ids: vec![0],
                },
            );
        }

        let (input, _) =
            assert_both_derivations_agree("promised rates", &dump, &index, &BTreeMap::new());
        let talker = input.nodes.iter().find(|n| n.name == "/talker").unwrap();
        assert_eq!(
            talker.rate_hz,
            Some(50.0),
            "the fastest timer, not the promise"
        );
        let listener = input.nodes.iter().find(|n| n.name == "/listener").unwrap();
        assert_eq!(listener.rate_hz, None, "a promise alone is no rate");
    }

    #[test]
    fn deadline_and_path_budget_use_tightest_path() {
        let dump = dump_with_two_nodes();

        let mut index = ManifestIndex::default();
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "loose".to_string(),
            path: PathDecl {
                max_latency: Some(
                    ros_launch_manifest_types::duration::Duration::from_millis_f64(50.0),
                ),
                ..Default::default()
            },
            scope_id: 0,
        });
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "tight".to_string(),
            path: PathDecl {
                max_latency: Some(
                    ros_launch_manifest_types::duration::Duration::from_millis_f64(10.0),
                ),
                ..Default::default()
            },
            scope_id: 0,
        });

        let input = mapper_input_from_dump(&dump, Some(&index), None, &BTreeMap::new());
        let talker = input.nodes.iter().find(|n| n.name == "/talker").unwrap();
        assert_eq!(talker.path_budget_ms, Some(10.0));
        assert_eq!(talker.deadline_us, Some(10_000));

        let listener = input.nodes.iter().find(|n| n.name == "/listener").unwrap();
        assert_eq!(listener.path_budget_ms, None);
        assert_eq!(listener.deadline_us, None);
    }

    #[test]
    fn criticality_is_case_insensitive_and_ignores_unrecognized() {
        let dump = dump_with_two_nodes();

        let mut nodes = BTreeMap::new();
        nodes.insert(
            "talker".to_string(),
            NodeDecl {
                criticality: Some("HIGH".to_string()),
                ..Default::default()
            },
        );
        nodes.insert(
            "listener".to_string(),
            NodeDecl {
                criticality: Some("urgent".to_string()), // not a recognized value
                ..Default::default()
            },
        );
        let manifest = Manifest {
            version: 1,
            nodes,
            ..Default::default()
        };

        let mut index = ManifestIndex::default();
        index
            .manifests
            .insert(0, empty_resolved_manifest(0, manifest));

        let input = mapper_input_from_dump(&dump, Some(&index), None, &BTreeMap::new());
        let talker = input.nodes.iter().find(|n| n.name == "/talker").unwrap();
        assert_eq!(talker.criticality, Some(Criticality::High));

        let listener = input.nodes.iter().find(|n| n.name == "/listener").unwrap();
        assert_eq!(listener.criticality, None);
    }

    // ── Phase 44.4: per-path extraction + chain resolution ──

    /// A declared budget reaches `MapperPath::exec_ms` — the slot phase 58's
    /// design pass named as the blocker for proportional deadline
    /// decomposition, and which was hard-coded `None` behind a comment that
    /// went stale when phase 60 made cost authorable.
    ///
    /// And it stays absent where it cannot be attributed. A budget is per NODE
    /// while this field is per PATH, so the two line up only for a single-path
    /// node; `play_launch measure` documents its own emitted budget as the SUM
    /// of a node's per-path maxima, so giving that sum to one of several paths
    /// would overstate it. Absent is a value here, not a gap — the feasibility
    /// diagnostic reports it as "incomplete evidence" rather than as feasible.
    #[test]
    fn a_declared_budget_reaches_a_single_path_and_no_further() {
        let dump = dump_with_two_nodes();
        let budgets = BTreeMap::from([
            ("/talker".to_string(), 3_500u64),
            ("/listener".to_string(), 900u64),
        ]);

        // `/talker` gets ONE path — the budget is unambiguously its cost.
        let mut index = ManifestIndex::default();
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "publish".to_string(),
            path: PathDecl {
                trigger: Some(ros_launch_manifest_types::Trigger::Timer { rate_hz: 10.0 }),
                output: vec!["out_ep".to_string()],
                ..Default::default()
            },
            scope_id: 0,
        });
        // `/listener` gets TWO — the split is unknown, so neither may claim it.
        for name in ["a", "b"] {
            index.node_paths.push(ResolvedNodePath {
                node_fqn: "/listener".to_string(),
                path_name: name.to_string(),
                path: PathDecl {
                    trigger: Some(ros_launch_manifest_types::Trigger::Input(vec![
                        "out_ep".to_string(),
                    ])),
                    ..Default::default()
                },
                scope_id: 0,
            });
        }

        let input = mapper_input_from_dump(&dump, Some(&index), None, &budgets);

        let talker = input.nodes.iter().find(|n| n.name == "/talker").unwrap();
        assert_eq!(
            talker.paths[0].exec_ms,
            Some(3.5),
            "a single-path node's declared budget IS that path's cost"
        );

        let listener = input.nodes.iter().find(|n| n.name == "/listener").unwrap();
        assert_eq!(listener.paths.len(), 2);
        for path in &listener.paths {
            assert_eq!(
                path.exec_ms, None,
                "a multi-path node's budget must not be attributed to any one path"
            );
        }
    }

    #[test]
    fn extract_paths_uses_effective_trigger_endpoints_not_raw_input_field() {
        let dump = dump_with_two_nodes();

        let mut index = ManifestIndex::default();
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "publish".to_string(),
            path: PathDecl {
                // Explicit `trigger: { input: [...] }` form — W2's bugfix
                // rule: raw `path.input` is empty here, endpoint names only
                // live inside the effective trigger.
                trigger: Some(ros_launch_manifest_types::Trigger::Input(vec![
                    "in_ep".to_string(),
                ])),
                output: vec!["out_ep".to_string()],
                max_latency: Some(
                    ros_launch_manifest_types::duration::Duration::from_millis_f64(12.5),
                ),
                ..Default::default()
            },
            scope_id: 0,
        });
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "tick".to_string(),
            path: PathDecl {
                trigger: Some(ros_launch_manifest_types::Trigger::Timer { rate_hz: 20.0 }),
                ..Default::default()
            },
            scope_id: 0,
        });

        let input = mapper_input_from_dump(&dump, Some(&index), None, &BTreeMap::new());
        let talker = input.nodes.iter().find(|n| n.name == "/talker").unwrap();
        assert_eq!(talker.paths.len(), 2);

        // Endpoints are qualified refs, the model's spelling (phase 78 W2).
        let publish = talker.paths.iter().find(|p| p.name == "publish").unwrap();
        assert_eq!(publish.inputs, vec!["/talker/in_ep".to_string()]);
        assert_eq!(publish.outputs, vec!["/talker/out_ep".to_string()]);
        assert_eq!(publish.max_latency_ms, Some(12.5));
        assert_eq!(
            publish.effective_trigger,
            EffectiveTrigger::Input(vec!["/talker/in_ep".to_string()])
        );

        let tick = talker.paths.iter().find(|p| p.name == "tick").unwrap();
        assert_eq!(
            tick.effective_trigger,
            EffectiveTrigger::Timer { rate_hz: 20.0 }
        );
        assert!(tick.inputs.is_empty());

        let listener = input.nodes.iter().find(|n| n.name == "/listener").unwrap();
        assert!(listener.paths.is_empty());
    }

    /// Hand-built index: a two-node chain where one path is a Timer
    /// (Boundary) and the other Input (Segment), with distinct node
    /// criticalities. `/talker/tick` declares `max_latency_ms: 2.0` and NO
    /// budget, which is what lets the cost tests below tell a declared
    /// deadline apart from a declared cost.
    fn chain_index_for_cost_tests() -> ManifestIndex {
        // (Boundary) and the other is Input (Segment), with distinct node
        // criticalities — asserts both the Boundary/Segment split and the
        // "criticality = max over member nodes" rule.
        let ms = ros_launch_manifest_types::duration::Duration::from_millis_f64;
        let tick = PathDecl {
            trigger: Some(ros_launch_manifest_types::Trigger::Timer { rate_hz: 50.0 }),
            output: vec!["chatter".to_string()],
            max_latency: Some(ms(2.0)),
            ..Default::default()
        };
        let react = PathDecl {
            trigger: Some(ros_launch_manifest_types::Trigger::Input(vec![
                "chatter".to_string(),
            ])),
            output: vec!["reaction".to_string()],
            max_latency: Some(ms(8.0)),
            ..Default::default()
        };

        let mut nodes = BTreeMap::new();
        nodes.insert(
            "talker".to_string(),
            NodeDecl {
                criticality: Some("low".to_string()),
                // The route is derived from the graph, and `build_global_graph`
                // reads paths off the manifest's own `NodeDecl` — declaring
                // them only in `index.node_paths` gives a node with no trigger
                // facts, hence no route and an empty chain list.
                paths: BTreeMap::from([("tick".to_string(), tick.clone())]),
                ..Default::default()
            },
        );
        nodes.insert(
            "listener".to_string(),
            NodeDecl {
                criticality: Some("high".to_string()),
                paths: BTreeMap::from([("react".to_string(), react.clone())]),
                ..Default::default()
            },
        );
        let manifest = Manifest {
            version: 1,
            nodes,
            ..Default::default()
        };

        let mut index = ManifestIndex::default();
        index
            .manifests
            .insert(0, empty_resolved_manifest(0, manifest));
        // The requirement, stated as a scope path: two ends and a budget. The
        // route between them is derived from the trigger/output facts of the
        // two node paths below — which is the whole point of the spelling that
        // replaced `chains:`/`segments:`.
        index.scope_paths.push(ResolvedScopePath {
            scope_id: 0,
            path_name: "mixed_chain".to_string(),
            input_topics: vec!["/chatter".to_string()],
            output_topics: vec!["/reaction".to_string()],
            path: PathDecl {
                max_latency: Some(ms(100.0)),
                ..Default::default()
            },
        });
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "tick".to_string(),
            path: tick,
            scope_id: 0,
        });
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/listener".to_string(),
            path_name: "react".to_string(),
            path: react,
            scope_id: 0,
        });
        index.topics.insert(
            "/chatter".to_string(),
            ResolvedTopic {
                derived_from_remaps: false,
                fqn: "/chatter".to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                qos: None,
                publishers: vec!["/talker/chatter".to_string()],
                subscribers: vec!["/listener/chatter".to_string()],
                rate_hz: None,
                derived_rate_hz: None,
                max_transport_ms: None,
                drop: None,
                scope_ids: vec![0],
            },
        );
        index.topics.insert(
            "/reaction".to_string(),
            ResolvedTopic {
                derived_from_remaps: false,
                fqn: "/reaction".to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                qos: None,
                publishers: vec!["/listener/reaction".to_string()],
                subscribers: vec![],
                rate_hz: None,
                derived_rate_hz: None,
                max_transport_ms: None,
                drop: None,
                scope_ids: vec![0],
            },
        );

        index
    }

    /// A declared budget — and only a declared budget — becomes a cost.
    #[test]
    fn a_declared_budget_becomes_the_boundary_cost() {
        let index = chain_index_for_cost_tests();
        let budgets = BTreeMap::from([("/talker".to_string(), 3_500u64)]);
        let graph = super::super::manifest_graph::build_global_graph(&index);
        let chains = resolve_chains_derived(&index, &graph, &budgets);
        let chain = chains
            .iter()
            .find(|c| c.name == "mixed_chain")
            .expect("mixed_chain should resolve");
        match &chain.elements[0] {
            ChainElement::Boundary { exec_ms, .. } => {
                // 3500us declared -> 3.5ms cost. Note the unit change: the
                // platform file speaks microseconds, the chain math
                // milliseconds.
                assert_eq!(*exec_ms, Some(3.5));
            }
            other => panic!("expected a Boundary, got {other:?}"),
        }
    }

    /// Regression guard for the conflation this wave removed: a path's
    /// declared latency must NEVER reappear as its execution cost.
    ///
    /// The two differ by construction here — `max_latency_ms` is 2.0 and no
    /// budget is declared — so any future change that reaches for the deadline
    /// again fails loudly instead of silently making every feasibility verdict
    /// optimistic.
    #[test]
    fn a_declared_deadline_is_never_used_as_a_cost() {
        let index = chain_index_for_cost_tests();
        let graph = super::super::manifest_graph::build_global_graph(&index);
        let chains = resolve_chains_derived(&index, &graph, &BTreeMap::new());
        let chain = chains
            .iter()
            .find(|c| c.name == "mixed_chain")
            .expect("mixed_chain should resolve");
        for element in &chain.elements {
            if let ChainElement::Boundary { exec_ms, node, .. } = element {
                assert_eq!(
                    *exec_ms, None,
                    "{node}: cost must be absent without a declared budget, never the deadline"
                );
            }
        }
    }

    #[test]
    fn a_derived_route_classifies_timer_as_boundary_and_criticality_is_max_of_members() {
        let index = chain_index_for_cost_tests();

        let graph = super::super::manifest_graph::build_global_graph(&index);
        let chains = resolve_chains_derived(&index, &graph, &BTreeMap::new());
        let chain = chains
            .iter()
            .find(|c| c.name == "mixed_chain")
            .expect("mixed_chain should resolve");
        assert_eq!(chain.elements.len(), 2);
        match &chain.elements[0] {
            ChainElement::Boundary {
                node,
                path,
                period_ms,
                exec_ms,
            } => {
                assert_eq!(node, "/talker");
                assert_eq!(path, "tick");
                assert_eq!(*period_ms, 20.0); // 1000 / 50 Hz
                // No declared budget, so the cost is ABSENT. This assertion
                // used to read `Some(2.0)` — the path's `max_latency_ms`,
                // i.e. its DEADLINE, standing in for its cost. Absent and
                // zero and "the deadline" are three different answers, and
                // only the first is true here.
                assert_eq!(*exec_ms, None);
            }
            other => panic!("expected a Boundary, got {other:?}"),
        }
        match &chain.elements[1] {
            ChainElement::Segment {
                nodes_in_topo_order,
            } => {
                assert_eq!(
                    nodes_in_topo_order,
                    &vec![ros_launch_manifest_sched::SegmentNode {
                        node: "/listener".to_string(),
                        path: "react".to_string(),
                    }]
                );
            }
            other => panic!("expected a Segment, got {other:?}"),
        }
        // max(Low, High) = High.
        assert_eq!(chain.criticality, Criticality::High);
    }

    // -----------------------------------------------------------------------
    // Phase 78 W2: the transition gate. `mapper_input_from_dump` (this file's
    // derivation, which W3 deletes) and `mapper_input_via_model` (the shared
    // crate over the resolved model, which `derive_sched_plan` calls) must
    // agree on every contract fixture: as a `MapperInput`, as the chains
    // alone, and as the `RankedPlan` the chain-aware ranker makes of it, byte
    // for byte on its Debug text. A red row is a fact the model failed to
    // carry or a rule the port got wrong, never a tolerance.
    // -----------------------------------------------------------------------

    use ros_launch_manifest_derive::resolve_chains;
    use ros_launch_manifest_sched::{RankedPlan, chain_aware_rank};
    use std::path::{Path, PathBuf};

    /// `<repo>/src/ros-launch-resolve/resolve` is this crate.
    fn repo_root() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("the repository root")
            .to_path_buf()
    }

    fn launch_stem(launch: &Path) -> String {
        launch
            .file_name()
            .expect("a launch file name")
            .to_string_lossy()
            .trim_end_matches(".launch.xml")
            .to_string()
    }

    /// Every `*.launch.xml` with a contract sidecar next to it under
    /// `tests/fixtures/contract_*/launch/` and `tests/fixtures/rt_workspace/launch/`,
    /// in path order. `contract_merge` contributes two (its include is a
    /// fixture launch of its own).
    fn contract_fixture_launches() -> Vec<PathBuf> {
        let fixtures = repo_root().join("tests/fixtures");
        let mut dirs: Vec<PathBuf> = std::fs::read_dir(&fixtures)
            .unwrap_or_else(|e| panic!("read {}: {e}", fixtures.display()))
            .flatten()
            .map(|e| e.path())
            .filter(|p| {
                let name = p.file_name().map(|n| n.to_string_lossy().to_string());
                name.is_some_and(|n| n.starts_with("contract_") || n == "rt_workspace")
            })
            .collect();
        dirs.sort();
        let mut out = Vec::new();
        for dir in dirs {
            let launch_dir = dir.join("launch");
            let mut files: Vec<PathBuf> = std::fs::read_dir(&launch_dir)
                .into_iter()
                .flatten()
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.to_string_lossy().ends_with(".launch.xml"))
                .collect();
            files.sort();
            for launch in files {
                let sidecar = launch_dir.join(format!("{}.contract.yaml", launch_stem(&launch)));
                if sidecar.is_file() {
                    out.push(launch);
                }
            }
        }
        out
    }

    fn fixture_name(launch: &Path) -> String {
        launch
            .strip_prefix(repo_root().join("tests/fixtures"))
            .unwrap_or(launch)
            .display()
            .to_string()
    }

    /// The launch tree, through the Rust parser, as `resolve` reads it.
    fn dump_of(launch: &Path) -> LaunchDump {
        let record = crate::verbs::parse_launch_file(launch, Default::default())
            .unwrap_or_else(|e| panic!("parse {}: {e}", launch.display()));
        let json = serde_json::to_string(&record).expect("record as json");
        serde_json::from_str(&json).expect("a LaunchDump")
    }

    /// The contract index the provider-sidecar channel resolves for the
    /// launch, exactly as `check` and `resolve` do without `--contracts`.
    fn index_of(dump: &LaunchDump, name: &str) -> ManifestIndex {
        let sources = crate::ros::manifest_loader::ContractSources {
            overlay: None,
            provider: true,
        };
        crate::ros::manifest_loader::load_manifests(dump, &sources)
            .unwrap_or_else(|e| panic!("{name}: load manifests: {e}"))
    }

    /// The per-node budgets of the launch's own posix platform file, when it
    /// ships one, the way `derive_sched_plan` reads them.
    fn budgets_of(launch: &Path) -> BTreeMap<String, u64> {
        let platform = launch.with_file_name(format!("{}.system.posix.yaml", launch_stem(launch)));
        if !platform.is_file() {
            return BTreeMap::new();
        }
        let file = ros_launch_manifest_sched::parse_platform_file(&platform)
            .unwrap_or_else(|e| panic!("parse {}: {e}", platform.display()));
        crate::ros::sched_loader::posix_budgets(&file)
    }

    /// The one rendering every consumer compares: `{:#?}` plus a trailing
    /// newline (rlm `derive/tests/parity.rs`).
    fn render(plan: &RankedPlan) -> String {
        format!("{plan:#?}\n")
    }

    /// Both derivations of one dump under one index and one set of budgets,
    /// asserted equal in every form a consumer reads, and returned so a test
    /// can look at what they agree on.
    fn assert_both_derivations_agree(
        name: &str,
        dump: &LaunchDump,
        index: &ManifestIndex,
        budgets: &BTreeMap<String, u64>,
    ) -> (MapperInput, String) {
        let from_dump = mapper_input_from_dump(dump, Some(index), None, budgets);
        let (from_model, report) = mapper_input_via_model(dump, Some(index), None, budgets);
        assert!(
            report.paths_without_trigger.is_empty(),
            "{name}: the model built in-process carries a trigger on every path: {:?}",
            report.paths_without_trigger
        );
        assert_eq!(from_dump, from_model, "{name}: the mapper input");

        // The chains alone, as the phase doc names the second assertion.
        let model = pre_sched_model(dump, Some(index));
        let graph = super::super::manifest_graph::build_global_graph(index);
        assert_eq!(
            resolve_chains_derived(index, &graph, budgets),
            resolve_chains(&model, &derive_facts_from_budgets(budgets)),
            "{name}: the resolved chains"
        );

        let plan_from_dump = render(&chain_aware_rank(&from_dump));
        let plan_from_model = render(&chain_aware_rank(&from_model));
        assert_eq!(plan_from_dump, plan_from_model, "{name}: the ranked plan");
        (from_model, plan_from_model)
    }

    /// Gates 1 of the phase: every contract fixture, `contract_derived_chain`
    /// included, derives the same input, the same chains and the same ranked
    /// plan from the dump and from the model.
    #[test]
    fn every_contract_fixture_derives_the_same_plan_from_dump_and_from_model() {
        let launches = contract_fixture_launches();
        assert!(
            launches
                .iter()
                .any(|l| l.to_string_lossy().contains("contract_derived_chain")),
            "the fixture walk must reach contract_derived_chain: {launches:?}"
        );
        let mut summary = Vec::new();
        for launch in &launches {
            let name = fixture_name(launch);
            let dump = dump_of(launch);
            let index = index_of(&dump, &name);
            let budgets = budgets_of(launch);
            let (input, plan) = assert_both_derivations_agree(&name, &dump, &index, &budgets);
            summary.push(format!(
                "{name}: {} node(s), {} chain(s), {} budget selector(s), {} ranked item(s)",
                input.nodes.len(),
                input.chains.len(),
                budgets.len(),
                plan.matches("RankItem {").count()
            ));
        }
        eprintln!(
            "phase 78 W2 parity over {} fixture launch(es):\n  {}",
            launches.len(),
            summary.join("\n  ")
        );
    }

    /// rlm v0.1.37 `derive/tests/snapshots/contract_derived_chain.ranked_plan.txt`,
    /// verbatim: the Debug text of
    /// `chain_aware_rank(&mapper_input_from_model(fixture, no facts))` on the
    /// resolved `contract_derived_chain` model checked in there. Both
    /// consumers assert their own rank of the same system is this text;
    /// re-taking it is an rlm change (`UPDATE_RANKED_PLAN_SNAPSHOT=1` there)
    /// that both gates then move with.
    const RLM_RANKED_PLAN_SNAPSHOT: &str = r#"RankedPlan {
    items: [
        RankItem {
            node: "/control/control_node",
            path: "control",
            fine_group: 0,
            coarse_group: Some(
                "points_to_cmd",
            ),
            tie_group: None,
            provenance: "derived(chain_aware: points_to_cmd segment drain 1/2)",
        },
        RankItem {
            node: "/perception/filter_component",
            path: "filter",
            fine_group: 0,
            coarse_group: Some(
                "points_to_cmd",
            ),
            tie_group: None,
            provenance: "derived(chain_aware: points_to_cmd segment drain 2/2)",
        },
        RankItem {
            node: "/perception/sensor_node",
            path: "tick",
            fine_group: 1,
            coarse_group: Some(
                "points_to_cmd",
            ),
            tie_group: None,
            provenance: "derived(chain_aware: points_to_cmd boundary RM period=10ms)",
        },
    ],
    warnings: [
        ChainFeasibleWithoutWcet {
            chain: "points_to_cmd",
            boundaries_without_wcet: [
                "/perception/sensor_node/tick",
            ],
        },
    ],
}
"#;

    /// Parity assertion 1 of design issue #52, play_launch's side: the rank
    /// of `contract_derived_chain` from this repository's own launch, contract
    /// and platform file is rlm's golden snapshot, from either derivation.
    #[test]
    fn contract_derived_chain_ranks_to_rlm_snapshot() {
        assert!(RLM_RANKED_PLAN_SNAPSHOT.is_ascii());
        let launch =
            repo_root().join("tests/fixtures/contract_derived_chain/launch/bringup.launch.xml");
        let name = fixture_name(&launch);
        let dump = dump_of(&launch);
        let index = index_of(&dump, &name);
        let budgets = budgets_of(&launch);
        assert!(
            budgets.is_empty(),
            "the fixture's platform file pins a priority, never a budget, so the \
             snapshot's no-facts rank is the rank here: {budgets:?}"
        );
        let (input, plan) = assert_both_derivations_agree(&name, &dump, &index, &budgets);
        if plan != RLM_RANKED_PLAN_SNAPSHOT {
            let first_diff = plan
                .lines()
                .zip(RLM_RANKED_PLAN_SNAPSHOT.lines())
                .position(|(a, e)| a != e)
                .map_or(
                    plan.lines()
                        .count()
                        .min(RLM_RANKED_PLAN_SNAPSHOT.lines().count()),
                    |i| i,
                )
                + 1;
            panic!(
                "{name}: the rank differs from rlm's snapshot at line {first_diff}\n\
                 --- expected\n{RLM_RANKED_PLAN_SNAPSHOT}\n--- actual\n{plan}"
            );
        }
        // The facts rlm's parity test pins in words, so a diff reads as a fact.
        let sensor = input
            .nodes
            .iter()
            .find(|n| n.name == "/perception/sensor_node")
            .expect("the sensor");
        assert_eq!(
            sensor.rate_hz,
            Some(100.0),
            "the timer's rate, from the trigger"
        );
        assert_eq!(sensor.criticality, None);
        let control = input
            .nodes
            .iter()
            .find(|n| n.name == "/control/control_node")
            .expect("the controller");
        assert_eq!(control.rate_hz, None);
        assert_eq!(control.deadline_us, Some(10_000));
        assert_eq!(control.criticality, Some(Criticality::High));
        assert!(input.nodes.iter().all(|n| !n.claims_concurrency));
    }

    /// Seam 1, resolved in the crate's favour: the one-path budget rule holds
    /// on a chain boundary too. `resolve_chains_derived` used to give a node's
    /// budget to the boundary whatever the path count, while `extract_paths`
    /// gave it to `MapperPath::exec_ms` only on a one-path node; rlm's
    /// `a_node_budget_is_not_attributed_to_a_boundary_of_a_multi_path_node`
    /// pins the number (a 100 Hz boundary with a 2 ms node budget on a
    /// two-timer node: sampling cost 12 ms here, 10 ms and a
    /// `ChainFeasibleWithoutWcet` warning there). Here, the same shape in this
    /// file's own fixture: 50 Hz, 3.5 ms budget, a second timer path added.
    #[test]
    fn a_node_budget_reaches_a_boundary_only_on_a_one_path_node_on_both_derivations() {
        let ms = ros_launch_manifest_types::duration::Duration::from_millis_f64;
        let mut index = chain_index_for_cost_tests();
        let diag = PathDecl {
            trigger: Some(ros_launch_manifest_types::Trigger::Timer { rate_hz: 1.0 }),
            output: vec!["diag".to_string()],
            max_latency: Some(ms(1.0)),
            ..Default::default()
        };
        let talker = index
            .manifests
            .get_mut(&0)
            .expect("scope 0")
            .manifest
            .nodes
            .get_mut("talker")
            .expect("talker");
        talker
            .publishers
            .insert("diag".to_string(), Default::default());
        talker.paths.insert("diag".to_string(), diag.clone());
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "diag".to_string(),
            path: diag,
            scope_id: 0,
        });
        index.topics.insert(
            "/diag".to_string(),
            ResolvedTopic {
                derived_from_remaps: false,
                fqn: "/diag".to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                qos: None,
                publishers: vec!["/talker/diag".to_string()],
                subscribers: vec![],
                rate_hz: None,
                derived_rate_hz: None,
                max_transport_ms: None,
                drop: None,
                scope_ids: vec![0],
            },
        );
        let budgets = BTreeMap::from([("/talker".to_string(), 3_500u64)]);

        let dump = dump_with_two_nodes();
        let (input, _) = assert_both_derivations_agree("two-timer talker", &dump, &index, &budgets);
        let chain = input
            .chains
            .iter()
            .find(|c| c.name == "mixed_chain")
            .expect("mixed_chain resolves");
        let sampling_ms: f64 = chain
            .elements
            .iter()
            .filter_map(|e| match e {
                ChainElement::Boundary {
                    node,
                    path,
                    period_ms,
                    exec_ms,
                } => {
                    assert_eq!((node.as_str(), path.as_str()), ("/talker", "tick"));
                    assert_eq!(*period_ms, 20.0);
                    assert_eq!(
                        *exec_ms, None,
                        "a node budget is not split over two paths, on the boundary either"
                    );
                    Some(period_ms + exec_ms.unwrap_or(0.0))
                }
                ChainElement::Segment { .. } => None,
            })
            .sum();
        assert_eq!(sampling_ms, 20.0, "counted at zero, not at 23.5");
        let talker = input
            .nodes
            .iter()
            .find(|n| n.name == "/talker")
            .expect("talker");
        assert_eq!(talker.paths.len(), 2);
        assert!(talker.paths.iter().all(|p| p.exec_ms.is_none()));
        assert_eq!(talker.rate_hz, Some(50.0), "the fastest timer");
        let plan = chain_aware_rank(&input);
        assert_eq!(
            plan.warnings,
            vec![
                ros_launch_manifest_sched::MapWarning::ChainFeasibleWithoutWcet {
                    chain: "mixed_chain".to_string(),
                    boundaries_without_wcet: vec!["/talker/tick".to_string()],
                }
            ],
            "and the verdict says the boundary was counted at zero"
        );
    }

    /// Seam 2, resolved in the crate's favour: a node whose derived
    /// criticality buckets to the no-requirement level keeps its advisory
    /// label. `extract_criticality` used to emit nothing for it, while the
    /// model carries no `node_criticality` entry and the crate falls back to
    /// `NodeInstance::criticality`. A hazard that buckets to a level still
    /// overrides the label on both.
    #[test]
    fn a_hazard_bucketing_to_no_requirement_leaves_the_label_standing_on_both_derivations() {
        use crate::ros::manifest_loader::DerivedCriticality;
        let mut index = chain_index_for_cost_tests();
        // listener is labelled `high`; the hazard that reaches it says QM.
        index.derived_criticality.insert(
            "/listener".to_string(),
            DerivedCriticality {
                level: "QM".to_string(),
                rank: 0,
                scale_len: 5,
                hazard: "h1".to_string(),
                role: "reaction",
            },
        );
        // talker is labelled `low`; the hazard that reaches it says ASIL D.
        index.derived_criticality.insert(
            "/talker".to_string(),
            DerivedCriticality {
                level: "D".to_string(),
                rank: 4,
                scale_len: 5,
                hazard: "h1".to_string(),
                role: "detector",
            },
        );
        let dump = dump_with_two_nodes();
        let (input, _) =
            assert_both_derivations_agree("hazard buckets", &dump, &index, &BTreeMap::new());
        let of = |name: &str| {
            input
                .nodes
                .iter()
                .find(|n| n.name == name)
                .unwrap_or_else(|| panic!("{name}"))
                .criticality
        };
        assert_eq!(of("/listener"), Some(Criticality::High), "the label stands");
        assert_eq!(
            of("/talker"),
            Some(Criticality::High),
            "the hazard overrides the label"
        );
        let chain = input
            .chains
            .iter()
            .find(|c| c.name == "mixed_chain")
            .expect("mixed_chain resolves");
        assert_eq!(chain.criticality, Criticality::High);
    }
}
