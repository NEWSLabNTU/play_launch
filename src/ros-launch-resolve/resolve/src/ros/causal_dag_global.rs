//! Cross-scope causal-DAG check on the **merged** dataflow graph.
//!
//! Motivated by a Phase 42 study finding (`.superpowers/sdd/p42-w1-report.md`
//! Q1): the `ros-launch-manifest` check crate's `causal-dag` rule
//! (`src/ros-launch-manifest/check/src/rules/causal_dag.rs`) builds its
//! `DataflowGraph` **per manifest file**, so a cycle spanning multiple
//! contract files/scopes is invisible to it — a real 3-hop cycle across
//! `vehicle_cmd_gate` / `simple_planning_simulator` / `tier4_control_launch`
//! was found in Autoware's contracts that no existing rule catches.
//!
//! This module runs the same "state edges don't count as causal" cycle
//! check the per-manifest rule performs, but over `play_launch`'s already
//! merged, cross-scope [`GlobalDataflowGraph`]
//! (`super::manifest_graph::build_global_graph`). Emitted as a **warning**
//! (not an error): existing contracts in the wild may legitimately trip
//! this (e.g. a simulator closing a physical control loop that a real
//! deployment wouldn't declare), so this is advisory, not a hard failure,
//! unlike the per-manifest `causal-dag` rule which is severity Error.
//!
//! Cycle detection: a single DFS pass over the merged graph (causal edges
//! only — `is_state` edges are skipped, matching the per-manifest rule's
//! semantics), white/gray/black coloring, one simple cycle reconstructed
//! per back-edge encountered, deduplicated by the sorted set of member
//! node FQNs. Not an exhaustive elementary-cycle enumeration — practical
//! and sufficient at Autoware scale (74 nodes, 171 sub edges as of the
//! Phase 42 study).

use super::manifest_graph::{GlobalDataflowGraph, GlobalEdge};
use super::manifest_loader::ManifestIndex;
use ros_launch_manifest_check::{Diagnostic, Severity};
use std::collections::{BTreeSet, HashMap, HashSet};

/// One member edge of a detected cycle, for reporting.
struct CycleEdge {
    from: String,
    to: String,
    topic: String,
}

/// DFS node color for cycle detection.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Color {
    White,
    Gray,
    Black,
}

/// Run the `causal-dag-global` check: find cycles in the merged causal
/// graph (state edges excluded) and record one warning per unique cycle
/// in `index.merge_diagnostics`.
pub fn check_causal_dag_global(index: &mut ManifestIndex, graph: &GlobalDataflowGraph) {
    for cycle in find_causal_cycles(graph) {
        let members_desc = cycle
            .iter()
            .map(|e| format!("{} -> {} (via {})", e.from, e.to, e.topic))
            .collect::<Vec<_>>()
            .join(", ");

        // Cite every distinct contract file whose scope owns a node
        // participating in the cycle.
        let mut contract_files: BTreeSet<String> = BTreeSet::new();
        for edge in &cycle {
            for fqn in [&edge.from, &edge.to] {
                if let Some(node) = graph.nodes.get(fqn)
                    && let Some(resolved) = index.manifests.get(&node.scope_id)
                {
                    contract_files.insert(resolved.contract_path.display().to_string());
                }
            }
        }
        let files_desc = contract_files.into_iter().collect::<Vec<_>>().join(", ");

        index.merge_diagnostics.push(Diagnostic {
            rule_id: "causal-dag-global".to_string(),
            severity: Severity::Warning,
            message: format!(
                "cross-scope causal cycle in the merged dataflow graph (state edges already \
                 excluded): {members_desc}. Contract files: {files_desc}. This may be a \
                 legitimate design (e.g. a simulator closing a physical control loop) or a \
                 missed 'state: true' tag on one of these subscribers — see the per-manifest \
                 'causal-dag' rule, which cannot see cross-scope cycles like this one."
            ),
            path: "graph".to_string(),
            span: None,
        });
    }
}

/// Find cycles in the causal (non-state) subgraph via a single DFS pass.
fn find_causal_cycles(graph: &GlobalDataflowGraph) -> Vec<Vec<CycleEdge>> {
    let mut color: HashMap<&str, Color> = graph
        .nodes
        .keys()
        .map(|k| (k.as_str(), Color::White))
        .collect();
    let mut path: Vec<&str> = Vec::new();
    let mut seen_cycles: HashSet<Vec<String>> = HashSet::new();
    let mut result: Vec<Vec<CycleEdge>> = Vec::new();

    // Deterministic iteration order for reproducible diagnostics.
    let mut starts: Vec<&str> = graph.nodes.keys().map(|k| k.as_str()).collect();
    starts.sort_unstable();

    for start in starts {
        if color[start] == Color::White {
            dfs(
                start,
                graph,
                &mut color,
                &mut path,
                &mut seen_cycles,
                &mut result,
            );
        }
    }

    result
}

fn dfs<'a>(
    node: &'a str,
    graph: &'a GlobalDataflowGraph,
    color: &mut HashMap<&'a str, Color>,
    path: &mut Vec<&'a str>,
    seen_cycles: &mut HashSet<Vec<String>>,
    result: &mut Vec<Vec<CycleEdge>>,
) {
    color.insert(node, Color::Gray);
    path.push(node);

    let mut out_indices: Vec<usize> = graph.out_edges.get(node).cloned().unwrap_or_default();
    // Deterministic order: sort by (to, topic).
    out_indices.sort_by(|&a, &b| {
        let ea = &graph.edges[a];
        let eb = &graph.edges[b];
        (ea.to.as_str(), ea.topic.as_str()).cmp(&(eb.to.as_str(), eb.topic.as_str()))
    });

    for idx in out_indices {
        let edge = &graph.edges[idx];
        if edge.is_state {
            continue;
        }
        let to = edge.to.as_str();
        match color.get(to).copied().unwrap_or(Color::Black) {
            Color::White => {
                dfs(to, graph, color, path, seen_cycles, result);
            }
            Color::Gray => {
                // Back edge: `to` is an ancestor on the current path — cycle found.
                if let Some(pos) = path.iter().position(|n| *n == to) {
                    let mut key: Vec<String> = path[pos..].iter().map(|s| s.to_string()).collect();
                    key.sort();
                    if seen_cycles.insert(key) {
                        // Reconstruct member edges along path[pos..], closing the
                        // loop back to `to` via the current back edge.
                        let mut members = Vec::new();
                        for w in path[pos..].windows(2) {
                            let (f, t) = (w[0], w[1]);
                            if let Some(e) = find_causal_edge(graph, f, t) {
                                members.push(CycleEdge {
                                    from: f.to_string(),
                                    to: t.to_string(),
                                    topic: e.topic.clone(),
                                });
                            }
                        }
                        members.push(CycleEdge {
                            from: node.to_string(),
                            to: to.to_string(),
                            topic: edge.topic.clone(),
                        });
                        result.push(members);
                    }
                }
            }
            Color::Black => {}
        }
    }

    path.pop();
    color.insert(node, Color::Black);
}

/// Find a causal (non-state) edge from `from` to `to`, if one exists.
fn find_causal_edge<'a>(
    graph: &'a GlobalDataflowGraph,
    from: &str,
    to: &str,
) -> Option<&'a GlobalEdge> {
    graph.out_edges.get(from)?.iter().find_map(|&idx| {
        let e = &graph.edges[idx];
        (!e.is_state && e.to == to).then_some(e)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros::manifest_graph::GlobalNode;
    use crate::ros::manifest_loader::{ContractChannel, ResolvedManifest};
    use std::path::PathBuf;

    fn node(fqn: &str, scope_id: usize) -> GlobalNode {
        GlobalNode {
            fqn: fqn.to_string(),
            scope_id,
            paths: Default::default(),
            subscribers: Default::default(),
            publishers: Default::default(),
        }
    }

    fn edge(from: &str, to: &str, topic: &str, is_state: bool) -> GlobalEdge {
        GlobalEdge {
            from: from.to_string(),
            to: to.to_string(),
            topic: topic.to_string(),
            sub_endpoint: "in".to_string(),
            max_transport_ms: None,
            is_state,
        }
    }

    fn resolved_manifest(scope_id: usize, contract_path: &str) -> ResolvedManifest {
        ResolvedManifest {
            scope_id,
            pkg: Some("pkg".to_string()),
            file: "manifest.launch.xml".to_string(),
            ns: String::new(),
            channel: ContractChannel::Overlay,
            contract_path: PathBuf::from(contract_path),
            manifest: ros_launch_manifest_types::Manifest::default(),
            source: String::new(),
            diagnostics: Vec::new(),
        }
    }

    /// Build a `GlobalDataflowGraph` by hand, wiring `out_edges`/`in_edges`
    /// the same way `build_global_graph` does.
    fn build_graph(nodes: Vec<GlobalNode>, edges: Vec<GlobalEdge>) -> GlobalDataflowGraph {
        let mut graph = GlobalDataflowGraph::default();
        for n in nodes {
            graph.nodes.insert(n.fqn.clone(), n);
        }
        for e in edges {
            let idx = graph.edges.len();
            graph.out_edges.entry(e.from.clone()).or_default().push(idx);
            graph.in_edges.entry(e.to.clone()).or_default().push(idx);
            graph.edges.push(e);
        }
        graph
    }

    /// Two synthetic "manifests" (scopes) whose merged graph has a 2-node
    /// cycle across scope boundaries — exactly the shape a per-manifest
    /// `causal-dag` rule cannot see (each scope's own subgraph is acyclic).
    #[test]
    fn cross_scope_cycle_across_two_manifests_warns() {
        let graph = build_graph(
            vec![node("/a", 0), node("/b", 1)],
            vec![edge("/a", "/b", "/x", false), edge("/b", "/a", "/y", false)],
        );
        let mut index = ManifestIndex::default();
        index
            .manifests
            .insert(0, resolved_manifest(0, "/tmp/a.contract.yaml"));
        index
            .manifests
            .insert(1, resolved_manifest(1, "/tmp/b.contract.yaml"));

        check_causal_dag_global(&mut index, &graph);

        let warnings: Vec<_> = index
            .merge_diagnostics
            .iter()
            .filter(|d| d.rule_id == "causal-dag-global")
            .collect();
        assert_eq!(warnings.len(), 1, "got: {warnings:?}");
        assert_eq!(warnings[0].severity, Severity::Warning);
        assert!(warnings[0].message.contains("/a -> /b (via /x)"));
        assert!(warnings[0].message.contains("/b -> /a (via /y)"));
        assert!(warnings[0].message.contains("a.contract.yaml"));
        assert!(warnings[0].message.contains("b.contract.yaml"));
    }

    /// A cycle broken by a `state: true` edge must not be flagged — matches
    /// the per-manifest `causal-dag` rule's semantics exactly.
    #[test]
    fn state_cut_cycle_does_not_warn() {
        let graph = build_graph(
            vec![node("/a", 0), node("/b", 0)],
            vec![
                edge("/a", "/b", "/x", false),
                edge("/b", "/a", "/y", true), // state: true breaks the cycle
            ],
        );
        let mut index = ManifestIndex::default();
        index
            .manifests
            .insert(0, resolved_manifest(0, "/tmp/a.contract.yaml"));

        check_causal_dag_global(&mut index, &graph);

        assert!(
            index
                .merge_diagnostics
                .iter()
                .all(|d| d.rule_id != "causal-dag-global"),
            "got: {:?}",
            index.merge_diagnostics
        );
    }

    /// An acyclic graph produces no warnings.
    #[test]
    fn acyclic_graph_does_not_warn() {
        let graph = build_graph(
            vec![node("/a", 0), node("/b", 0), node("/c", 0)],
            vec![edge("/a", "/b", "/x", false), edge("/b", "/c", "/y", false)],
        );
        let mut index = ManifestIndex::default();
        index
            .manifests
            .insert(0, resolved_manifest(0, "/tmp/a.contract.yaml"));

        check_causal_dag_global(&mut index, &graph);

        assert!(index.merge_diagnostics.is_empty());
    }
}
