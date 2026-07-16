//! Declared causal graph export (`play_launch check --export-graph`).
//!
//! Phase 42.1 (`docs/design/autoware-system-model-study.md`, Q1). This is an
//! **export**, not a validation step: it walks the already-loaded
//! `ManifestIndex` (same data the `check` rules consult) and serializes the
//! declared graph — ROS nodes, topics, pub/sub wiring (tagged causal / state
//! / required), node-level and scope-level `paths:`, and a cycle catalogue
//! computed BEFORE `state:` cuts are applied — so downstream tooling (Phase
//! 42 W2's measured-model join, W4's report) can consume a stable JSON
//! shape without re-deriving it from the manifests.
//!
//! Reuses `manifest_graph::build_global_graph()` (Phase 35) for the
//! node-to-node edge graph (already carries `is_state` per edge, exactly the
//! "before cuts" graph we need for cycle detection) — no changes to the
//! `ros-launch-manifest` check crate are required.
//!
//! JSON schema is documented in `docs/design/causal-graph-export.md`.

use super::manifest_graph::{self, GlobalDataflowGraph};
use super::manifest_loader::ManifestIndex;
use eyre::Result;
use serde::Serialize;
use std::collections::{HashMap, HashSet};
use std::path::Path;

/// Schema version. Bump on breaking JSON shape changes; additive fields
/// (new optional keys) don't require a bump.
const SCHEMA_VERSION: u32 = 1;

/// Top-level export payload.
#[derive(Debug, Serialize)]
pub struct GraphExport {
    pub version: u32,
    pub nodes: Vec<NodeOut>,
    pub topics: Vec<TopicOut>,
    pub pub_edges: Vec<PubEdgeOut>,
    pub sub_edges: Vec<SubEdgeOut>,
    pub node_paths: Vec<NodePathOut>,
    pub scope_paths: Vec<ScopePathOut>,
    pub cycles: Vec<CycleOut>,
}

/// A ROS node vertex.
#[derive(Debug, Serialize)]
pub struct NodeOut {
    pub fqn: String,
    pub scope_id: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pkg: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub criticality: Option<String>,
}

/// A topic vertex.
#[derive(Debug, Serialize)]
pub struct TopicOut {
    pub fqn: String,
    #[serde(rename = "type")]
    pub msg_type: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rate_hz: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_transport_ms: Option<f64>,
}

/// A publisher edge: node → topic.
#[derive(Debug, Serialize)]
pub struct PubEdgeOut {
    pub node: String,
    pub topic: String,
    pub endpoint: String,
    /// Topic-level declared rate (the primary "declared rate fact").
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rate_hz: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_rate_hz: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_rate_hz: Option<f64>,
}

/// A subscriber edge: topic → node, tagged causal / state / required.
#[derive(Debug, Serialize)]
pub struct SubEdgeOut {
    pub topic: String,
    pub node: String,
    pub endpoint: String,
    /// `true` unless the endpoint declares `state: true`. Mutually
    /// exclusive with `state`.
    pub causal: bool,
    /// `state: true` — polled/read-latest, not a causal dependency. Our
    /// existing cycle-cut primitive (see `causal-dag` rule).
    pub state: bool,
    /// `required: true` — must receive at least once before operational.
    /// Orthogonal to `causal`/`state`.
    pub required: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_rate_hz: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_age_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_transport_ms: Option<f64>,
}

/// A node-level path: input/output are the node's own endpoint names.
#[derive(Debug, Serialize)]
pub struct NodePathOut {
    pub node: String,
    pub path_name: String,
    pub input: Vec<String>,
    pub output: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_latency_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tolerance_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation: Option<String>,
    pub scope_id: usize,
    /// Always `false` — node paths are intra-node by construction.
    pub cross_node: bool,
}

/// A scope-level path: input/output are resolved topic FQNs, spanning
/// potentially many nodes across the scope's subtree.
#[derive(Debug, Serialize)]
pub struct ScopePathOut {
    pub scope_id: usize,
    pub path_name: String,
    pub input_topics: Vec<String>,
    pub output_topics: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_latency_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tolerance_ms: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub correlation: Option<String>,
    /// Always `true` — scope paths cross node boundaries by construction.
    pub cross_node: bool,
}

/// One directed edge participating in a cycle (node-to-node, topic as label).
#[derive(Debug, Clone, Serialize)]
pub struct CycleMemberOut {
    pub from: String,
    pub to: String,
    pub topic: String,
    pub state: bool,
}

/// A cycle found in the causal graph BEFORE `state:` cuts are applied
/// (i.e. `state: true` sub edges are included as if causal). Q1 needs both
/// "which cycles exist" and "which cut breaks them".
#[derive(Debug, Serialize)]
pub struct CycleOut {
    /// Full cycle, in traversal order.
    pub members: Vec<CycleMemberOut>,
    /// Subset of `members` that are `state: true` — the edges whose
    /// removal breaks this cycle in the actual (post-cut) causal-dag rule.
    pub cut_by_state: Vec<CycleMemberOut>,
    /// `true` iff at least one member is a `state:` edge — i.e. this cycle
    /// is already broken by the current declarations and the
    /// `causal-dag` rule does not flag it.
    pub broken: bool,
}

/// Build the full graph export from a resolved `ManifestIndex`.
pub fn build_export(index: &ManifestIndex) -> GraphExport {
    let global = manifest_graph::build_global_graph(index);

    // Node metadata (pkg, criticality) isn't tracked on `GlobalNode` — pull
    // it directly from the per-scope manifests, keyed by the same FQN
    // computation `build_global_graph` uses.
    let mut node_meta: HashMap<String, (Option<String>, Option<String>)> = HashMap::new();
    let mut pub_props: HashMap<String, ros_launch_manifest_types::EndpointProps> = HashMap::new();
    let mut sub_props: HashMap<String, ros_launch_manifest_types::EndpointProps> = HashMap::new();
    for resolved in index.manifests.values() {
        for (node_name, node_decl) in &resolved.manifest.nodes {
            let node_fqn = qualify_name(&resolved.ns, node_name);
            node_meta.insert(
                node_fqn.clone(),
                (resolved.pkg.clone(), node_decl.criticality.clone()),
            );
            for (ep, props) in &node_decl.publishers {
                pub_props.insert(format!("{node_fqn}/{ep}"), props.clone());
            }
            for (ep, props) in &node_decl.subscribers {
                sub_props.insert(format!("{node_fqn}/{ep}"), props.clone());
            }
        }
    }

    let mut nodes: Vec<NodeOut> = global
        .nodes
        .keys()
        .map(|fqn| {
            let (pkg, criticality) = node_meta.get(fqn).cloned().unwrap_or_default();
            let scope_id = global.nodes[fqn].scope_id;
            NodeOut {
                fqn: fqn.clone(),
                scope_id,
                pkg,
                criticality,
            }
        })
        .collect();
    nodes.sort_by(|a, b| a.fqn.cmp(&b.fqn));

    let topics: Vec<TopicOut> = index
        .topics
        .values()
        .map(|t| TopicOut {
            fqn: t.fqn.clone(),
            msg_type: t.msg_type.clone(),
            rate_hz: t.rate_hz,
            max_transport_ms: t.max_transport_ms,
        })
        .collect();

    let mut pub_edges = Vec::new();
    let mut sub_edges = Vec::new();
    for topic in index.topics.values() {
        for pref in &topic.publishers {
            let Some((node, ep)) = split_endpoint_ref(pref) else {
                continue;
            };
            let props = pub_props.get(pref);
            pub_edges.push(PubEdgeOut {
                node,
                topic: topic.fqn.clone(),
                endpoint: ep,
                rate_hz: topic.rate_hz,
                min_rate_hz: props.and_then(|p| p.min_rate_hz),
                max_rate_hz: props.and_then(|p| p.max_rate_hz),
            });
        }
        for sref in &topic.subscribers {
            let Some((node, ep)) = split_endpoint_ref(sref) else {
                continue;
            };
            let props = sub_props.get(sref);
            let state = props.and_then(|p| p.state).unwrap_or(false);
            let required = props.and_then(|p| p.required).unwrap_or(false);
            sub_edges.push(SubEdgeOut {
                topic: topic.fqn.clone(),
                node,
                endpoint: ep,
                causal: !state,
                state,
                required,
                min_rate_hz: props.and_then(|p| p.min_rate_hz),
                max_age_ms: props.and_then(|p| p.max_age_ms),
                max_transport_ms: props
                    .and_then(|p| p.max_transport_ms)
                    .or(topic.max_transport_ms),
            });
        }
    }
    pub_edges
        .sort_by(|a, b| (&a.topic, &a.node, &a.endpoint).cmp(&(&b.topic, &b.node, &b.endpoint)));
    sub_edges
        .sort_by(|a, b| (&a.topic, &a.node, &a.endpoint).cmp(&(&b.topic, &b.node, &b.endpoint)));

    let mut node_paths: Vec<NodePathOut> = index
        .node_paths
        .iter()
        .map(|p| NodePathOut {
            node: p.node_fqn.clone(),
            path_name: p.path_name.clone(),
            input: p.path.input.clone(),
            output: p.path.output.clone(),
            max_latency_ms: p.path.max_latency_ms,
            tolerance_ms: p.path.tolerance_ms,
            correlation: p.path.correlation.clone(),
            scope_id: p.scope_id,
            cross_node: false,
        })
        .collect();
    node_paths.sort_by(|a, b| {
        (a.scope_id, &a.node, &a.path_name).cmp(&(b.scope_id, &b.node, &b.path_name))
    });

    let mut scope_paths: Vec<ScopePathOut> = index
        .scope_paths
        .iter()
        .map(|p| ScopePathOut {
            scope_id: p.scope_id,
            path_name: p.path_name.clone(),
            input_topics: p.input_topics.clone(),
            output_topics: p.output_topics.clone(),
            max_latency_ms: p.path.max_latency_ms,
            tolerance_ms: p.path.tolerance_ms,
            correlation: p.path.correlation.clone(),
            cross_node: true,
        })
        .collect();
    scope_paths.sort_by(|a, b| (a.scope_id, &a.path_name).cmp(&(b.scope_id, &b.path_name)));

    let cycles = find_cycles(&global);

    GraphExport {
        version: SCHEMA_VERSION,
        nodes,
        topics,
        pub_edges,
        sub_edges,
        node_paths,
        scope_paths,
        cycles,
    }
}

/// Split an endpoint FQN like `/ns/node/endpoint` into `(node_fqn, endpoint_name)`.
fn split_endpoint_ref(ep_ref: &str) -> Option<(String, String)> {
    let pos = ep_ref.rfind('/')?;
    let node = &ep_ref[..pos];
    let ep = &ep_ref[pos + 1..];
    if node.is_empty() || ep.is_empty() {
        return None;
    }
    Some((node.to_string(), ep.to_string()))
}

/// Prefix a relative name with a namespace, matching `qualify_name` in
/// `manifest_loader` (duplicated here to avoid making it `pub`).
fn qualify_name(ns: &str, name: &str) -> String {
    if name.starts_with('/') {
        return name.to_string();
    }
    let ns = ns.trim_end_matches('/');
    if ns.is_empty() {
        format!("/{name}")
    } else {
        format!("{ns}/{name}")
    }
}

/// Find cycles in the node-to-node causal graph, counting `state:` edges as
/// causal (i.e. the graph BEFORE cuts are applied — Q1 needs to see cycles
/// that a cut currently breaks, not just the ones that survive).
///
/// Uses a single DFS pass with the classic white/gray/black coloring: each
/// back-edge encountered (an edge into a vertex currently on the DFS stack)
/// yields one simple cycle, reconstructed from the DFS path. This finds
/// every cycle reachable via a back-edge in this traversal order — the
/// standard practical approach for "list the cycles" at study scale. It is
/// not a from-scratch enumeration of all elementary cycles (e.g. Johnson's
/// algorithm) — dense graphs with many alternate simple cycles through the
/// same SCC may see only a subset. Good enough for Autoware-scale manifests
/// (this is an export for human/tooling inspection, not the validation
/// rule).
fn find_cycles(graph: &GlobalDataflowGraph) -> Vec<CycleOut> {
    #[derive(Clone, Copy, PartialEq, Eq)]
    enum Color {
        White,
        Gray,
        Black,
    }

    struct State<'g> {
        graph: &'g GlobalDataflowGraph,
        color: HashMap<&'g str, Color>,
        path_edges: Vec<usize>,
        path_nodes: Vec<&'g str>,
        cycles: Vec<Vec<usize>>,
        seen: HashSet<Vec<usize>>,
    }

    fn dfs<'g>(v: &'g str, st: &mut State<'g>) {
        st.color.insert(v, Color::Gray);
        st.path_nodes.push(v);

        if let Some(out_idx) = st.graph.out_edges.get(v) {
            let mut idxs = out_idx.clone();
            idxs.sort_unstable();
            for eidx in idxs {
                let to = st.graph.edges[eidx].to.as_str();
                match st.color.get(to).copied().unwrap_or(Color::White) {
                    Color::White => {
                        st.path_edges.push(eidx);
                        dfs(to, st);
                        st.path_edges.pop();
                    }
                    Color::Gray => {
                        if let Some(start) = st.path_nodes.iter().position(|&n| n == to) {
                            let mut cyc: Vec<usize> = st.path_edges[start..].to_vec();
                            cyc.push(eidx);
                            let mut key = cyc.clone();
                            key.sort_unstable();
                            if st.seen.insert(key) {
                                st.cycles.push(cyc);
                            }
                        }
                    }
                    Color::Black => {}
                }
            }
        }

        st.path_nodes.pop();
        st.color.insert(v, Color::Black);
    }

    let mut order: Vec<&str> = graph.nodes.keys().map(|s| s.as_str()).collect();
    order.sort_unstable();

    let mut st = State {
        graph,
        color: order.iter().map(|&n| (n, Color::White)).collect(),
        path_edges: Vec::new(),
        path_nodes: Vec::new(),
        cycles: Vec::new(),
        seen: HashSet::new(),
    };

    for v in order {
        if st.color.get(v).copied().unwrap_or(Color::White) == Color::White {
            dfs(v, &mut st);
        }
    }

    st.cycles
        .into_iter()
        .map(|edge_idxs| {
            let members: Vec<CycleMemberOut> = edge_idxs
                .iter()
                .map(|&i| {
                    let e = &graph.edges[i];
                    CycleMemberOut {
                        from: e.from.clone(),
                        to: e.to.clone(),
                        topic: e.topic.clone(),
                        state: e.is_state,
                    }
                })
                .collect();
            let cut_by_state: Vec<CycleMemberOut> =
                members.iter().filter(|m| m.state).cloned().collect();
            let broken = !cut_by_state.is_empty();
            CycleOut {
                members,
                cut_by_state,
                broken,
            }
        })
        .collect()
}

/// Escape a string for inclusion in a DOT quoted identifier.
fn dot_escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

/// Render the export as a Graphviz DOT graph: topics as boxes, nodes as
/// ellipses, `state:` sub edges dashed. Kept deliberately plain — this is
/// for human inspection (`dot -Tsvg`), not a styling exercise.
pub fn render_dot(export: &GraphExport) -> String {
    let mut s = String::new();
    s.push_str("digraph causal {\n  rankdir=LR;\n");

    for n in &export.nodes {
        s.push_str(&format!(
            "  \"{}\" [shape=ellipse, label=\"{}\"];\n",
            dot_escape(&n.fqn),
            dot_escape(&n.fqn),
        ));
    }
    for t in &export.topics {
        s.push_str(&format!(
            "  \"{}\" [shape=box, label=\"{}\"];\n",
            dot_escape(&t.fqn),
            dot_escape(&t.fqn),
        ));
    }
    for e in &export.pub_edges {
        s.push_str(&format!(
            "  \"{}\" -> \"{}\";\n",
            dot_escape(&e.node),
            dot_escape(&e.topic),
        ));
    }
    for e in &export.sub_edges {
        let style = if e.state {
            " [style=dashed]"
        } else if e.required {
            " [color=blue]"
        } else {
            ""
        };
        s.push_str(&format!(
            "  \"{}\" -> \"{}\"{style};\n",
            dot_escape(&e.topic),
            dot_escape(&e.node),
        ));
    }

    s.push_str("}\n");
    s
}

/// Write the graph export to `path`, picking JSON or DOT by extension
/// (`.dot` → Graphviz; anything else, including no extension → JSON).
pub fn export_to_file(index: &ManifestIndex, path: &Path) -> Result<()> {
    let export = build_export(index);
    let is_dot = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| e.eq_ignore_ascii_case("dot"))
        .unwrap_or(false);

    if is_dot {
        std::fs::write(path, render_dot(&export))?;
    } else {
        let json = serde_json::to_string_pretty(&export)?;
        std::fs::write(path, json)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::launch_dump::{LaunchDump, ScopeEntry, ScopeOrigin};
    use crate::ros::manifest_loader::ContractSources;
    use std::collections::HashMap;

    /// Load a synthetic system through the real `load_manifests` pipeline
    /// (single file scope, contract delivered via the overlay channel into
    /// a tempdir) so the export sees exactly the same `ManifestIndex` shape
    /// `check` builds:
    ///
    /// - `sensor` publishes `scan` (10 Hz)
    /// - `filter` subscribes `scan` (causal), publishes `filtered`; has a
    ///   node-level path `passthrough` (scan endpoint -> output endpoint)
    /// - `localizer` subscribes `odom` (causal), publishes `pose`
    /// - `planner` subscribes `pose` (`state: true, required: true` — the
    ///   cycle-breaking cut), publishes `cmd`
    /// - `controller` subscribes `cmd` (causal), publishes `odom`
    ///
    /// Causal chain `localizer -> planner -> controller -> localizer`
    /// (via `pose`, `cmd`, `odom`) would cycle if `pose` weren't
    /// `state: true`. A scope-level path `end_to_end` (scan -> filtered)
    /// covers the scope-path export.
    fn synthetic_index() -> ManifestIndex {
        let yaml = r#"
version: 1
nodes:
  sensor:
    pub: [scan]
  filter:
    sub: [scan]
    pub: [output]
    paths:
      passthrough:
        input: [scan]
        output: [output]
        max_latency_ms: 20.0
  localizer:
    sub: [odom]
    pub: [pose]
  planner:
    sub:
      pose: { state: true, required: true }
    pub: [cmd]
  controller:
    sub: [cmd]
    pub: [odom]
topics:
  scan:
    type: sensor_msgs/msg/LaserScan
    rate_hz: 10.0
    pub: [sensor/scan]
    sub: [filter/scan]
  filtered:
    type: sensor_msgs/msg/LaserScan
    pub: [filter/output]
    sub: []
  pose:
    type: geometry_msgs/msg/PoseStamped
    pub: [localizer/pose]
    sub: [planner/pose]
  cmd:
    type: geometry_msgs/msg/Twist
    pub: [planner/cmd]
    sub: [controller/cmd]
  odom:
    type: nav_msgs/msg/Odometry
    pub: [controller/odom]
    sub: [localizer/odom]
paths:
  end_to_end:
    input: [scan]
    output: [filtered]
    max_latency_ms: 50.0
"#;

        let tmp = tempfile::TempDir::new().unwrap();
        let dest_dir = tmp.path().join("synthetic_pkg").join("launch");
        std::fs::create_dir_all(&dest_dir).unwrap();
        std::fs::write(dest_dir.join("synthetic.contract.yaml"), yaml).unwrap();

        let dump = LaunchDump {
            node: vec![],
            load_node: vec![],
            container: vec![],
            lifecycle_node: vec![],
            file_data: HashMap::new(),
            variables: HashMap::new(),
            scopes: vec![ScopeEntry {
                id: 0,
                origin: Some(ScopeOrigin {
                    pkg: Some("synthetic_pkg".to_string()),
                    file: "synthetic.launch.xml".to_string(),
                    path: None,
                }),
                ns: "".to_string(),
                args: HashMap::new(),
                parent: None,
            }],
        };

        let sources = ContractSources {
            overlay: Some(tmp.path().to_path_buf()),
            provider: false,
        };
        crate::ros::manifest_loader::load_manifests(&dump, &sources)
            .expect("synthetic manifest loads")
    }

    #[test]
    fn nodes_topics_and_pkg_criticality() {
        let index = synthetic_index();
        let export = build_export(&index);

        assert_eq!(export.nodes.len(), 5, "expected 5 nodes");
        assert_eq!(export.topics.len(), 5, "expected 5 topics");

        let planner = export
            .nodes
            .iter()
            .find(|n| n.fqn == "/planner")
            .expect("planner node present");
        assert_eq!(planner.pkg.as_deref(), Some("synthetic_pkg"));

        let scan = export
            .topics
            .iter()
            .find(|t| t.fqn == "/scan")
            .expect("scan topic present");
        assert_eq!(scan.rate_hz, Some(10.0));
    }

    #[test]
    fn sub_edge_tags_state_and_required() {
        let index = synthetic_index();
        let export = build_export(&index);

        let pose_edge = export
            .sub_edges
            .iter()
            .find(|e| e.topic == "/pose" && e.node == "/planner")
            .expect("pose sub edge present");
        assert!(pose_edge.state, "pose->planner should be state: true");
        assert!(!pose_edge.causal, "state edges are not causal");
        assert!(pose_edge.required, "pose->planner should be required: true");

        let scan_edge = export
            .sub_edges
            .iter()
            .find(|e| e.topic == "/scan" && e.node == "/filter")
            .expect("scan sub edge present");
        assert!(!scan_edge.state);
        assert!(scan_edge.causal);
        assert!(!scan_edge.required);
    }

    #[test]
    fn cycle_found_and_marked_broken_by_state_cut() {
        let index = synthetic_index();
        let export = build_export(&index);

        assert_eq!(
            export.cycles.len(),
            1,
            "expected exactly one cycle: localizer -> planner -> controller -> localizer, got {:?}",
            export.cycles
        );
        let cycle = &export.cycles[0];
        assert!(
            cycle.broken,
            "cycle should be marked broken (pose is state: true)"
        );
        assert_eq!(cycle.cut_by_state.len(), 1);
        assert_eq!(cycle.cut_by_state[0].topic, "/pose");
        assert_eq!(cycle.members.len(), 3, "3-edge cycle");
    }

    #[test]
    fn node_and_scope_paths_exported() {
        let index = synthetic_index();
        let export = build_export(&index);

        assert_eq!(export.node_paths.len(), 1);
        let np = &export.node_paths[0];
        assert_eq!(np.node, "/filter");
        assert_eq!(np.path_name, "passthrough");
        assert_eq!(np.max_latency_ms, Some(20.0));
        assert!(!np.cross_node);

        assert_eq!(export.scope_paths.len(), 1);
        let sp = &export.scope_paths[0];
        assert_eq!(sp.path_name, "end_to_end");
        assert_eq!(sp.input_topics, vec!["/scan".to_string()]);
        assert_eq!(sp.output_topics, vec!["/filtered".to_string()]);
        assert!(sp.cross_node);
    }

    #[test]
    fn dot_render_is_well_formed_and_marks_state_dashed() {
        let index = synthetic_index();
        let export = build_export(&index);
        let dot = render_dot(&export);

        assert!(dot.starts_with("digraph causal {"));
        assert!(dot.trim_end().ends_with('}'));
        assert!(dot.contains("style=dashed"), "state edge should be dashed");
        // Every node/topic fqn should appear as a quoted identifier.
        for n in &export.nodes {
            assert!(dot.contains(&format!("\"{}\"", n.fqn)));
        }
        for t in &export.topics {
            assert!(dot.contains(&format!("\"{}\"", t.fqn)));
        }
    }
}
