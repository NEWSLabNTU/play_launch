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

use std::collections::BTreeMap;

use ros_launch_manifest_sched::{
    ChainElement, ChainSemantics, Criticality, EffectiveTrigger, MapperInput, MapperNode,
    MapperPath, ResolvedChain, SystemSched,
};

use crate::ros::{
    launch_dump::LaunchDump,
    manifest_loader::ManifestIndex,
    sched_loader::{ScheduledRecord, scheduled_records_from_dump},
};

/// Build the mapper's input from a launch dump and (optionally) a resolved
/// contract index. `legacy` is threaded straight through to
/// [`MapperInput::legacy`] — non-`None` only when driving the `manual`
/// mapper via the `.toml` bridge.
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
/// ([`ros_launch_manifest_sched::MapperPath`]/[`EffectiveTrigger`]).
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

    index
        .node_paths
        .iter()
        .filter(|p| p.node_fqn == record.fqn)
        .map(|p| {
            let effective = p.path.effective_trigger();
            let inputs = match &effective {
                ros_launch_manifest_types::EffectiveTrigger::Input(eps) => eps.clone(),
                _ => Vec::new(),
            };
            MapperPath {
                name: p.path_name.clone(),
                effective_trigger: convert_trigger(effective),
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
                outputs: p.path.output.clone(),
                max_jitter_ms: p.path.max_jitter.map(|d| d.as_millis_f64()),
                miss: p.path.miss.as_ref().map(convert_miss),
            }
        })
        .collect()
}

/// Translate the contract's `miss:` into the mapper's mirror of it.
fn convert_miss(m: &ros_launch_manifest_types::MissSpec) -> ros_launch_manifest_sched::MapperMiss {
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

/// Translate a `types::EffectiveTrigger` (W1) into the sched crate's
/// dependency-free mirror ([`chain.rs` module doc][crate root] — the sched
/// crate has no dependency on `ros_launch_manifest_types`).
fn convert_trigger(t: ros_launch_manifest_types::EffectiveTrigger) -> EffectiveTrigger {
    use ros_launch_manifest_types::EffectiveTrigger as T;
    match t {
        T::Timer { rate_hz } => EffectiveTrigger::Timer { rate_hz },
        T::Input(eps) => EffectiveTrigger::Input(eps),
        T::Once => EffectiveTrigger::Once,
        T::Spontaneous => EffectiveTrigger::Spontaneous,
        T::Unclassified => EffectiveTrigger::Unclassified,
    }
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
                    elements.push(ChainElement::Boundary {
                        node: node_fqn.clone(),
                        path: path_name.clone(),
                        period_ms: 1000.0 / rate_hz,
                        exec_ms: budget_us_for(budgets, node_fqn).map(|us| us as f64 / 1000.0),
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
    let bare = node_fqn.rsplit('/').next()?;
    let resolved = index.manifests.get(&scope_id)?;
    let raw = resolved.manifest.nodes.get(bare)?.criticality.as_deref()?;
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

/// `<node_fqn>/<ep_name>` -> `Some(node_fqn)`, mirroring
/// `manifest_loader::split_endpoint_ref_for_check` (private to that module,
/// reimplemented here rather than exposed — it's a one-line string split).
fn ep_ref_node_fqn(ep_ref: &str) -> Option<&str> {
    let pos = ep_ref.rfind('/')?;
    if pos == 0 {
        return None;
    }
    Some(&ep_ref[..pos])
}

/// Max declared rate across the node's publications: every topic-level
/// `rate_hz` where this node is a publisher, plus every one of the node's
/// own `pub.<ep>.min_rate_hz` declarations.
fn extract_rate_hz(record: &ScheduledRecord, index: &ManifestIndex) -> Option<f64> {
    let mut best: Option<f64> = None;
    let mut consider = |v: Option<f64>| {
        if let Some(v) = v {
            best = Some(best.map_or(v, |b| b.max(v)));
        }
    };

    for topic in index.topics.values() {
        if topic
            .publishers
            .iter()
            .any(|p| ep_ref_node_fqn(p) == Some(record.fqn.as_str()))
        {
            consider(topic.rate_hz);
        }
    }

    if let Some(decl) = node_decl(record, index) {
        for props in decl.publishers.values() {
            consider(props.min_rate_hz);
        }
    }

    best
}

/// `(path_budget_ms, deadline_us)`: the tightest (min) `max_latency_ms`
/// across every node-path this node owns, as milliseconds and (rounded)
/// microseconds respectively. `None` when the node owns no path with a
/// declared budget.
fn extract_path_facts(
    record: &ScheduledRecord,
    index: &ManifestIndex,
) -> (Option<f64>, Option<u64>) {
    let min_ms = index
        .node_paths
        .iter()
        .filter(|p| p.node_fqn == record.fqn)
        .filter_map(|p| p.path.max_latency.map(|d| d.as_millis_f64()))
        .fold(None, |acc: Option<f64>, v| {
            Some(acc.map_or(v, |a: f64| a.min(v)))
        });

    let deadline_us = min_ms.map(|ms| (ms * 1000.0).round() as u64);
    (min_ms, deadline_us)
}

/// `nodes.<name>.criticality`, case-insensitive, ignore-if-absent-or-unrecognized.
fn extract_criticality(record: &ScheduledRecord, index: &ManifestIndex) -> Option<Criticality> {
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
                {"id": 0, "ns": "/", "parent": null}
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

    #[test]
    fn rate_hz_is_max_of_topic_and_endpoint_facts() {
        let dump = dump_with_two_nodes();

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
        index.topics.insert(
            "/chatter".to_string(),
            ResolvedTopic {
                fqn: "/chatter".to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                qos: None,
                publishers: vec!["/talker/chatter".to_string()],
                subscribers: vec![],
                rate_hz: Some(100.0),
                max_transport_ms: None,
                drop: None,
                scope_ids: vec![0],
            },
        );

        let input = mapper_input_from_dump(&dump, Some(&index), None, &BTreeMap::new());
        let talker = input.nodes.iter().find(|n| n.name == "/talker").unwrap();
        // topic-level rate_hz (100) beats the endpoint-level min_rate_hz (30).
        assert_eq!(talker.rate_hz, Some(100.0));

        let listener = input.nodes.iter().find(|n| n.name == "/listener").unwrap();
        assert_eq!(listener.rate_hz, None);
    }

    #[test]
    fn rate_hz_falls_back_to_endpoint_min_rate_hz_when_topic_has_none() {
        let dump = dump_with_two_nodes();

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
        // No topics declared at all — only the endpoint-level fact exists.
        let input = mapper_input_from_dump(&dump, Some(&index), None, &BTreeMap::new());
        let talker = input.nodes.iter().find(|n| n.name == "/talker").unwrap();
        assert_eq!(talker.rate_hz, Some(30.0));
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

        let publish = talker.paths.iter().find(|p| p.name == "publish").unwrap();
        assert_eq!(publish.inputs, vec!["in_ep".to_string()]);
        assert_eq!(publish.outputs, vec!["out_ep".to_string()]);
        assert_eq!(publish.max_latency_ms, Some(12.5));
        assert_eq!(
            publish.effective_trigger,
            EffectiveTrigger::Input(vec!["in_ep".to_string()])
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
                fqn: "/chatter".to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                qos: None,
                publishers: vec!["/talker/chatter".to_string()],
                subscribers: vec!["/listener/chatter".to_string()],
                rate_hz: None,
                max_transport_ms: None,
                drop: None,
                scope_ids: vec![0],
            },
        );
        index.topics.insert(
            "/reaction".to_string(),
            ResolvedTopic {
                fqn: "/reaction".to_string(),
                msg_type: "std_msgs/msg/String".to_string(),
                qos: None,
                publishers: vec!["/listener/reaction".to_string()],
                subscribers: vec![],
                rate_hz: None,
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
}
