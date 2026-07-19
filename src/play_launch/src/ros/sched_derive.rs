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

use ros_launch_manifest_check::Severity;
use ros_launch_manifest_sched::{
    ChainElement, ChainSemantics, Criticality, EffectiveTrigger, MapperInput, MapperNode,
    MapperPath, ResolvedChain, SystemSched,
};
use ros_launch_manifest_types::ChainSegment;

use crate::ros::{
    chain_checks::{build_pub_sub_maps, resolve_segment},
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
) -> MapperInput {
    let nodes = scheduled_records_from_dump(dump)
        .iter()
        .map(|r| build_mapper_node(r, index))
        .collect();
    let chains = index.map(resolve_chains).unwrap_or_default();
    MapperInput {
        nodes,
        legacy,
        chains,
    }
}

/// Every node FQN that participates in at least one resolved `chains:`
/// declaration (Phase 44.4 review, Critical-1): resolves chains against the
/// index and collects the member node identities. Used by the replay path
/// to compute chain membership when the scheduling plan came from a
/// `SystemModel` (`SchedPlan::from_model`, whose execution layer is
/// bindings-only and carries no chain structure) — the manifest index is
/// re-loaded at replay time anyway, so membership is derivable there
/// without extending the SystemModel YAML schema.
pub fn chain_member_fqns(index: &ManifestIndex) -> std::collections::BTreeSet<String> {
    resolve_chains(index)
        .iter()
        .flat_map(|c| c.elements.iter())
        .flat_map(|e| -> Vec<String> {
            match e {
                ChainElement::Segment {
                    nodes_in_topo_order,
                } => nodes_in_topo_order
                    .iter()
                    .map(|sn| sn.node.clone())
                    .collect(),
                ChainElement::Boundary { node, .. } => vec![node.clone()],
            }
        })
        .collect()
}

fn build_mapper_node(record: &ScheduledRecord, index: Option<&ManifestIndex>) -> MapperNode {
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
    let paths = extract_paths(record, index);

    MapperNode {
        name: record.fqn.clone(),
        scope: record.scope_ns.clone(),
        rate_hz,
        deadline_us,
        criticality,
        path_budget_ms,
        paths,
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
fn extract_paths(record: &ScheduledRecord, index: &ManifestIndex) -> Vec<MapperPath> {
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
                max_latency_ms: p.path.max_latency_ms,
                // budget (WCET) — the current contract vocabulary declares
                // no per-path execution-time fact, so this stays None until a
                // future vocab addition (the shared schema carries the slot).
                exec_ms: None,
                inputs,
                outputs: p.path.output.clone(),
            }
        })
        .collect()
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

fn convert_semantics(s: ros_launch_manifest_types::ChainSemantics) -> ChainSemantics {
    match s {
        ros_launch_manifest_types::ChainSemantics::Reaction => ChainSemantics::Reaction,
        ros_launch_manifest_types::ChainSemantics::Age => ChainSemantics::Age,
    }
}

/// Resolve every declared `chains:` block in the merged index into the
/// sched crate's [`ResolvedChain`] (Phase 44.4 §2), reusing
/// `chain_checks::resolve_segment` (44.2) for `{scope, path}` resolution —
/// the same function `chain-link`/`chain-budget` use, so extraction and
/// checking never disagree about what a segment resolves to.
///
/// A chain is **excluded** (not pushed to the result, logged at `debug`)
/// when:
/// - it already has a `Severity::Error` diagnostic under `chains.<name>`
///   (either a cross-scope `chain-link` error, in `index.merge_diagnostics`,
///   or a per-manifest `chain-shape` cyclic-chain error, in that scope's own
///   `ResolvedManifest::diagnostics`) — a broken/cyclic chain is invisible
///   to the mapper, exactly like the design's "chains failing chain-link are
///   excluded from MapperInput" rule (44.2 already emits the diagnostic;
///   this wave acts on it);
/// - any of its path segments resolves to a **scope-level aggregate** path
///   (`node_fqn: None`) rather than a node-owned one — `ChainElement`
///   requires a node identity per segment (the POSIX apply layer schedules
///   processes, not scope aggregates), which a whole-scope aggregate path
///   cannot provide. Not a `chain-link` failure (the checker accepts scope
///   aggregates as valid hops); a documented sched-extraction limitation.
pub(crate) fn resolve_chains(index: &ManifestIndex) -> Vec<ResolvedChain> {
    let (pub_map, sub_map) = build_pub_sub_maps(index);

    index
        .manifests
        .values()
        .flat_map(|resolved| resolved.manifest.chains.iter())
        .filter_map(|(chain_name, chain)| {
            if chain_has_error_diagnostic(index, chain_name) {
                tracing::debug!(
                    "sched: chain '{chain_name}' excluded from mapper input — has a chain-link \
                     or chain-shape error"
                );
                return None;
            }
            let resolved_chain = build_resolved_chain(index, &pub_map, &sub_map, chain_name, chain);
            if resolved_chain.is_none() {
                tracing::debug!(
                    "sched: chain '{chain_name}' excluded from mapper input — a path segment \
                     resolved to a scope-level aggregate path (no owning node) or could not be \
                     resolved"
                );
            }
            resolved_chain
        })
        .collect()
}

/// `true` if `chains.<chain_name>` has any `Severity::Error` diagnostic,
/// either cross-scope (`chain-link`, in `merge_diagnostics`) or per-manifest
/// (`chain-shape`, in some scope's own `ResolvedManifest::diagnostics`).
fn chain_has_error_diagnostic(index: &ManifestIndex, chain_name: &str) -> bool {
    let path = format!("chains.{chain_name}");
    let cross_scope = index
        .merge_diagnostics
        .iter()
        .any(|d| d.severity == Severity::Error && d.path == path);
    let per_manifest = index.manifests.values().any(|m| {
        m.diagnostics
            .iter()
            .any(|d| d.severity == Severity::Error && d.path == path)
    });
    cross_scope || per_manifest
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
fn build_resolved_chain(
    index: &ManifestIndex,
    pub_map: &std::collections::HashMap<String, String>,
    sub_map: &std::collections::HashMap<String, String>,
    chain_name: &str,
    chain: &ros_launch_manifest_types::ChainDecl,
) -> Option<ResolvedChain> {
    let mut elements: Vec<ChainElement> = Vec::new();
    let mut criticality: Option<Criticality> = None;

    for seg in &chain.segments {
        let ChainSegment::Path { scope, path } = seg else {
            continue;
        };
        let resolved = resolve_segment(index, pub_map, sub_map, scope, path)?;
        // `ChainElement` requires a node identity — scope-level aggregate
        // paths (`node_fqn: None`) can't provide one; exclude the whole
        // chain (documented limitation, see `resolve_chains` doc comment).
        let node_fqn = resolved.node_fqn.clone()?;

        if let Some(scope_id) = resolved.scope_id
            && let Some(c) = node_criticality(index, scope_id, &node_fqn)
            && criticality.is_none_or(|cur| c > cur)
        {
            criticality = Some(c);
        }

        match resolved.trigger {
            ros_launch_manifest_types::EffectiveTrigger::Timer { rate_hz } if rate_hz > 0.0 => {
                elements.push(ChainElement::Boundary {
                    node: node_fqn,
                    path: path.clone(),
                    period_ms: 1000.0 / rate_hz,
                    exec_ms: resolved.max_latency_ms,
                });
            }
            _ => push_segment_node(&mut elements, node_fqn, path.clone()),
        }
    }

    if elements.is_empty() {
        return None;
    }

    Some(ResolvedChain {
        name: chain_name.to_string(),
        // No W1 chain-level `criticality` field — the caller (this wave)
        // derives it as the max over the chain's member nodes'
        // `criticality`, per the W3 report's documented resolution of this
        // question. `Low` when no member declares a criticality at all
        // (matches `Criticality`'s own "advisory hint, absent = no signal"
        // convention elsewhere in this module).
        criticality: criticality.unwrap_or(Criticality::Low),
        max_latency_ms: chain.max_latency_ms,
        semantics: convert_semantics(chain.semantics),
        elements,
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
        .filter_map(|p| p.path.max_latency_ms)
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
        ContractChannel, ResolvedManifest, ResolvedNodePath, ResolvedTopic,
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
        let input = mapper_input_from_dump(&dump, None, None);
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

        let input = mapper_input_from_dump(&dump, Some(&index), None);
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
        let input = mapper_input_from_dump(&dump, Some(&index), None);
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
                max_latency_ms: Some(50.0),
                ..Default::default()
            },
            scope_id: 0,
        });
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "tight".to_string(),
            path: PathDecl {
                max_latency_ms: Some(10.0),
                ..Default::default()
            },
            scope_id: 0,
        });

        let input = mapper_input_from_dump(&dump, Some(&index), None);
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

        let input = mapper_input_from_dump(&dump, Some(&index), None);
        let talker = input.nodes.iter().find(|n| n.name == "/talker").unwrap();
        assert_eq!(talker.criticality, Some(Criticality::High));

        let listener = input.nodes.iter().find(|n| n.name == "/listener").unwrap();
        assert_eq!(listener.criticality, None);
    }

    // ── Phase 44.4: per-path extraction + chain resolution ──

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
                max_latency_ms: Some(12.5),
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

        let input = mapper_input_from_dump(&dump, Some(&index), None);
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

    // ── Real cross-scope chain fixtures (mirrors
    // `chain_checks::tests::chain_dump`/`overlay_index` — duplicated
    // locally since those helpers are private to that module's own test
    // submodule). ──

    fn fixture_dir() -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../ros-launch-manifest/tests/fixtures")
    }

    fn overlay_index(dump: &LaunchDump) -> ManifestIndex {
        use crate::ros::manifest_loader::{ContractSources, load_manifests};

        let tmp = tempfile::TempDir::new().unwrap();
        for scope in &dump.scopes {
            let Some(origin) = &scope.origin else {
                continue;
            };
            let pkg_dir = origin.pkg.as_deref().unwrap_or("_");
            let stem = origin
                .file
                .strip_suffix(".launch.xml")
                .or_else(|| origin.file.strip_suffix(".launch.py"))
                .or_else(|| origin.file.strip_suffix(".launch.yaml"))
                .or_else(|| origin.file.strip_suffix(".launch"))
                .unwrap_or(&origin.file);
            let src = fixture_dir().join(pkg_dir).join("manifest.yaml");
            if !src.exists() {
                continue;
            }
            let dest_dir = tmp.path().join(pkg_dir).join("launch");
            std::fs::create_dir_all(&dest_dir).unwrap();
            std::fs::copy(&src, dest_dir.join(format!("{stem}.contract.yaml"))).unwrap();
        }
        let sources = ContractSources {
            overlay: Some(tmp.path().to_path_buf()),
            provider: false,
        };
        load_manifests(dump, &sources).unwrap()
    }

    fn make_chain_dump() -> LaunchDump {
        use crate::ros::launch_dump::{ScopeEntry, ScopeOrigin};
        let scope =
            |id: usize, pkg: &str, file: &str, ns: &str, parent: Option<usize>| ScopeEntry {
                id,
                origin: Some(ScopeOrigin {
                    pkg: Some(pkg.to_string()),
                    file: file.to_string(),
                    path: None,
                }),
                ns: ns.to_string(),
                args: std::collections::HashMap::new(),
                parent,
            };
        LaunchDump {
            node: vec![],
            load_node: vec![],
            container: vec![],
            lifecycle_node: vec![],
            file_data: std::collections::HashMap::new(),
            variables: std::collections::HashMap::new(),
            scopes: vec![
                scope(0, "manifest_chain_root", "root.launch.xml", "", None),
                scope(1, "manifest_chain_a", "a.launch.xml", "/a", Some(0)),
                scope(2, "manifest_chain_b", "b.launch.xml", "/b", Some(0)),
            ],
        }
    }

    #[test]
    fn resolve_chains_includes_the_clean_chain_with_segment_elements() {
        let index = overlay_index(&make_chain_dump());
        let chains = resolve_chains(&index);
        let ok = chains
            .iter()
            .find(|c| c.name == "ok_chain")
            .expect("ok_chain should resolve — no chain-link errors");
        // producer.make (spontaneous) -> consumer.consume (input): neither
        // is a Timer trigger, so both merge into one Segment run, in
        // declaration order.
        assert_eq!(ok.elements.len(), 1);
        match &ok.elements[0] {
            ChainElement::Segment {
                nodes_in_topo_order,
            } => {
                assert_eq!(
                    nodes_in_topo_order,
                    &vec![
                        ros_launch_manifest_sched::SegmentNode {
                            node: "/a/producer".to_string(),
                            path: "make".to_string(),
                        },
                        ros_launch_manifest_sched::SegmentNode {
                            node: "/b/consumer".to_string(),
                            path: "consume".to_string(),
                        },
                    ]
                );
            }
            other => panic!("expected a Segment, got {other:?}"),
        }
    }

    #[test]
    fn resolve_chains_excludes_chains_with_chain_link_errors() {
        let index = overlay_index(&make_chain_dump());
        let chains = resolve_chains(&index);
        for broken in [
            "broken_via_chain",
            "via_not_output_chain",
            "via_not_consumed_chain",
        ] {
            assert!(
                !chains.iter().any(|c| c.name == broken),
                "chain '{broken}' has a chain-link error and must be excluded from MapperInput, \
                 got: {:?}",
                chains.iter().map(|c| &c.name).collect::<Vec<_>>()
            );
        }
    }

    #[test]
    fn resolve_chains_includes_budget_and_sampling_warned_chains() {
        // `chain-budget`/`chain-sampling-feasibility` are WARNINGS, not
        // `chain-link` ERRORs — these chains resolve fine structurally and
        // must still reach the mapper (which has its own feasibility step
        // and emits `MapWarning::ChainInfeasible` for the infeasible one).
        let index = overlay_index(&make_chain_dump());
        let chains = resolve_chains(&index);
        assert!(chains.iter().any(|c| c.name == "budget_blown_chain"));
        assert!(chains.iter().any(|c| c.name == "sampling_infeasible_chain"));
    }

    #[test]
    fn build_resolved_chain_classifies_timer_as_boundary_and_criticality_is_max_of_members() {
        // Hand-built index: a two-node chain where one path is a Timer
        // (Boundary) and the other is Input (Segment), with distinct node
        // criticalities — asserts both the Boundary/Segment split and the
        // "criticality = max over member nodes" rule.
        let mut nodes = BTreeMap::new();
        nodes.insert(
            "talker".to_string(),
            NodeDecl {
                criticality: Some("low".to_string()),
                ..Default::default()
            },
        );
        nodes.insert(
            "listener".to_string(),
            NodeDecl {
                criticality: Some("high".to_string()),
                ..Default::default()
            },
        );
        let mut chains = BTreeMap::new();
        chains.insert(
            "mixed_chain".to_string(),
            ros_launch_manifest_types::ChainDecl {
                semantics: ros_launch_manifest_types::ChainSemantics::Reaction,
                max_latency_ms: 100.0,
                segments: vec![
                    ChainSegment::Path {
                        scope: "/".to_string(),
                        path: "tick".to_string(),
                    },
                    ChainSegment::Via {
                        via: "/chatter".to_string(),
                    },
                    ChainSegment::Path {
                        scope: "/".to_string(),
                        path: "react".to_string(),
                    },
                ],
            },
        );
        let manifest = Manifest {
            version: 1,
            nodes,
            chains,
            ..Default::default()
        };

        let mut index = ManifestIndex::default();
        index
            .manifests
            .insert(0, empty_resolved_manifest(0, manifest));
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/talker".to_string(),
            path_name: "tick".to_string(),
            path: PathDecl {
                trigger: Some(ros_launch_manifest_types::Trigger::Timer { rate_hz: 50.0 }),
                output: vec!["chatter".to_string()],
                max_latency_ms: Some(2.0),
                ..Default::default()
            },
            scope_id: 0,
        });
        index.node_paths.push(ResolvedNodePath {
            node_fqn: "/listener".to_string(),
            path_name: "react".to_string(),
            path: PathDecl {
                trigger: Some(ros_launch_manifest_types::Trigger::Input(vec![
                    "chatter".to_string(),
                ])),
                max_latency_ms: Some(8.0),
                ..Default::default()
            },
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

        let chains = resolve_chains(&index);
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
                assert_eq!(*exec_ms, Some(2.0));
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
