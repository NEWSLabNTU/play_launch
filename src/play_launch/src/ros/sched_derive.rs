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

use ros_launch_manifest_sched::{Criticality, MapperInput, MapperNode, SystemSched};

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
) -> MapperInput {
    let nodes = scheduled_records_from_dump(dump)
        .iter()
        .map(|r| build_mapper_node(r, index))
        .collect();
    MapperInput { nodes, legacy }
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

    MapperNode {
        name: record.fqn.clone(),
        scope: record.scope_ns.clone(),
        rate_hz,
        deadline_us,
        criticality,
        path_budget_ms,
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
    match raw.to_ascii_lowercase().as_str() {
        "high" => Some(Criticality::High),
        "medium" => Some(Criticality::Medium),
        "low" => Some(Criticality::Low),
        other => {
            tracing::debug!(
                "sched: unrecognized criticality `{other}` for node `{}` — ignored",
                record.fqn
            );
            None
        }
    }
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
}
