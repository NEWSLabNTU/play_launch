//! Global dataflow graph built from a merged `ManifestIndex`.
//!
//! Phase 35.1–35.3 deliverable. The graph spans all scopes in the
//! manifest tree and is used for:
//! - Critical-path latency computation (replaces the old per-manifest
//!   sum check, which was incorrect for parallel topologies)
//! - Scope path verification — finding the actual dataflow between
//!   resolved input and output topics within a scope's subtree
//!
//! Built from:
//! - `index.topics` (merged topic FQN → publishers/subscribers)
//! - `index.manifests` (per-scope NodeDecl with paths and endpoint props)

use super::manifest_loader::ManifestIndex;
use ros_launch_manifest_types::PathDecl;
use std::collections::{BTreeMap, HashMap, HashSet};

/// A node in the global dataflow graph (one ROS 2 node).
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct GlobalNode {
    /// Fully-qualified node name (with namespace prefix).
    pub fqn: String,
    /// Scope this node belongs to.
    pub scope_id: usize,
    /// Node-level paths declared in the manifest.
    pub paths: BTreeMap<String, PathDecl>,
    /// Subscriber endpoint properties (`state`, `required`, etc.)
    /// keyed by endpoint name (not FQN).
    pub subscribers: BTreeMap<String, ros_launch_manifest_types::EndpointProps>,
    /// Publisher endpoint properties.
    pub publishers: BTreeMap<String, ros_launch_manifest_types::EndpointProps>,
}

impl GlobalNode {
    /// Worst-case `max_latency_ms` across this node's paths.
    /// Returns 0.0 if no path declares a budget.
    pub fn max_latency_ms(&self) -> f64 {
        self.paths
            .values()
            .filter_map(|p| p.max_latency_ms)
            .fold(0.0_f64, f64::max)
    }
}

/// A directed edge in the global dataflow graph.
///
/// Edges go from a publisher node to a subscriber node, carrying the
/// resolved topic FQN. Multiple edges can exist between the same pair
/// of nodes (different topics).
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct GlobalEdge {
    /// Source node FQN (publisher).
    pub from: String,
    /// Destination node FQN (subscriber).
    pub to: String,
    /// Topic FQN this edge represents.
    pub topic: String,
    /// Subscriber endpoint name on the destination node.
    pub sub_endpoint: String,
    /// Worst-case transport latency for this hop (from topic).
    pub max_transport_ms: Option<f64>,
    /// True if the subscriber is `state: true` (polled, non-causal).
    /// State edges don't carry latency through the graph.
    pub is_state: bool,
}

/// The global dataflow graph spanning all scopes in the merged index.
#[derive(Debug, Default)]
#[allow(dead_code)]
pub struct GlobalDataflowGraph {
    /// All nodes by FQN.
    pub nodes: HashMap<String, GlobalNode>,
    /// All edges (may have multiple between the same pair).
    pub edges: Vec<GlobalEdge>,
    /// Lookup: node FQN → indices into `edges` for outgoing edges.
    pub out_edges: HashMap<String, Vec<usize>>,
    /// Lookup: node FQN → indices into `edges` for incoming edges.
    pub in_edges: HashMap<String, Vec<usize>>,
    /// Lookup: topic FQN → set of publisher node FQNs.
    pub topic_publishers: HashMap<String, Vec<String>>,
    /// Lookup: topic FQN → set of subscriber node FQNs.
    pub topic_subscribers: HashMap<String, Vec<String>>,
}

/// Build a global dataflow graph from a merged `ManifestIndex`.
pub fn build_global_graph(index: &ManifestIndex) -> GlobalDataflowGraph {
    let mut graph = GlobalDataflowGraph::default();

    // Step 1: collect all nodes from all manifests, computing FQNs.
    for resolved in index.manifests.values() {
        for (node_name, node_decl) in &resolved.manifest.nodes {
            let fqn = qualify(&resolved.ns, node_name);
            graph.nodes.insert(
                fqn.clone(),
                GlobalNode {
                    fqn,
                    scope_id: resolved.scope_id,
                    paths: node_decl.paths.clone(),
                    subscribers: node_decl.subscribers.clone(),
                    publishers: node_decl.publishers.clone(),
                },
            );
        }
    }

    // Step 2: build edges from merged topics.
    // Each topic has publishers and subscribers as endpoint FQNs
    // (e.g. "/perception/cropbox/output"). Extract the node FQN by
    // dropping the last segment (the endpoint name).
    for (topic_fqn, topic) in &index.topics {
        let pub_node_eps: Vec<(String, String)> = topic
            .publishers
            .iter()
            .filter_map(|ep_ref| split_endpoint_ref(ep_ref))
            .collect();
        let sub_node_eps: Vec<(String, String)> = topic
            .subscribers
            .iter()
            .filter_map(|ep_ref| split_endpoint_ref(ep_ref))
            .collect();

        // Index publishers/subscribers by topic for quick lookups.
        graph
            .topic_publishers
            .entry(topic_fqn.clone())
            .or_default()
            .extend(pub_node_eps.iter().map(|(n, _)| n.clone()));
        graph
            .topic_subscribers
            .entry(topic_fqn.clone())
            .or_default()
            .extend(sub_node_eps.iter().map(|(n, _)| n.clone()));

        // Create one edge per (pub, sub) pair.
        for (pub_node, _pub_ep) in &pub_node_eps {
            for (sub_node, sub_ep) in &sub_node_eps {
                let is_state = graph
                    .nodes
                    .get(sub_node)
                    .and_then(|n| n.subscribers.get(sub_ep))
                    .and_then(|props| props.state)
                    .unwrap_or(false);

                let edge = GlobalEdge {
                    from: pub_node.clone(),
                    to: sub_node.clone(),
                    topic: topic_fqn.clone(),
                    sub_endpoint: sub_ep.clone(),
                    max_transport_ms: topic.max_transport_ms,
                    is_state,
                };
                let idx = graph.edges.len();
                graph
                    .out_edges
                    .entry(pub_node.clone())
                    .or_default()
                    .push(idx);
                graph
                    .in_edges
                    .entry(sub_node.clone())
                    .or_default()
                    .push(idx);
                graph.edges.push(edge);
            }
        }
    }

    graph
}

/// Split an endpoint FQN like `/ns/node/endpoint` into `(node_fqn, endpoint_name)`.
/// Returns `None` if the ref has no `/` separator.
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
/// `manifest_loader`.
fn qualify(ns: &str, name: &str) -> String {
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

// ── Phase 35.2: subgraph extraction ──

/// A view of the global graph restricted to a scope subtree.
#[derive(Debug)]
#[allow(dead_code)]
pub struct ScopeSubgraph<'a> {
    pub graph: &'a GlobalDataflowGraph,
    /// Set of scope IDs in the subtree (the scope itself + all descendants).
    pub scope_subtree: HashSet<usize>,
    /// Source node FQNs (publishers of the path's input topics
    /// that are in the subtree).
    pub sources: Vec<String>,
    /// Sink node FQNs (subscribers of the path's output topics
    /// that are in the subtree).
    pub sinks: Vec<String>,
}

#[allow(dead_code)]
impl<'a> ScopeSubgraph<'a> {
    /// Returns true if the node is in the scope subtree.
    pub fn contains_node(&self, node_fqn: &str) -> bool {
        self.graph
            .nodes
            .get(node_fqn)
            .map(|n| self.scope_subtree.contains(&n.scope_id))
            .unwrap_or(false)
    }

    /// Returns the outgoing edges from a node that stay within the
    /// scope subtree (and are not state-only edges).
    pub fn out_causal_edges(&self, node_fqn: &str) -> Vec<&GlobalEdge> {
        self.graph
            .out_edges
            .get(node_fqn)
            .map(|indices| {
                indices
                    .iter()
                    .map(|i| &self.graph.edges[*i])
                    .filter(|e| !e.is_state && self.contains_node(&e.to))
                    .collect()
            })
            .unwrap_or_default()
    }
}

/// Compute the set of all scopes in `root_scope_id`'s subtree (root + descendants).
/// Uses `index.scope_parents` (which maps each scope to its parent).
pub fn subtree_scope_ids(index: &ManifestIndex, root_scope_id: usize) -> HashSet<usize> {
    // Build a child→parent index, then walk descendants.
    let mut subtree = HashSet::new();
    subtree.insert(root_scope_id);

    // Repeatedly add scopes whose parent is already in the set, until fixed point.
    // O(n^2) but n is small (hundreds at most).
    loop {
        let before = subtree.len();
        for (scope_id, parent) in &index.scope_parents {
            if let Some(parent_id) = parent
                && subtree.contains(parent_id)
            {
                subtree.insert(*scope_id);
            }
        }
        if subtree.len() == before {
            break;
        }
    }
    subtree
}

/// Extract the subgraph relevant to a scope path, identifying source
/// and sink nodes from the path's resolved input and output topics.
pub fn subgraph_for_scope_path<'a>(
    graph: &'a GlobalDataflowGraph,
    scope_subtree: HashSet<usize>,
    input_topics: &[String],
    output_topics: &[String],
) -> ScopeSubgraph<'a> {
    let in_subtree = |node_fqn: &str| -> bool {
        graph
            .nodes
            .get(node_fqn)
            .map(|n| scope_subtree.contains(&n.scope_id))
            .unwrap_or(false)
    };

    let mut sources = Vec::new();
    for topic in input_topics {
        if let Some(pubs) = graph.topic_publishers.get(topic) {
            for pub_node in pubs {
                if in_subtree(pub_node) && !sources.contains(pub_node) {
                    sources.push(pub_node.clone());
                }
            }
        }
        // Also include subscribers of the input topic — they're the
        // first nodes inside the scope to receive the data.
        if let Some(subs) = graph.topic_subscribers.get(topic) {
            for sub_node in subs {
                if in_subtree(sub_node) && !sources.contains(sub_node) {
                    sources.push(sub_node.clone());
                }
            }
        }
    }

    let mut sinks = Vec::new();
    for topic in output_topics {
        if let Some(pubs) = graph.topic_publishers.get(topic) {
            for pub_node in pubs {
                if in_subtree(pub_node) && !sinks.contains(pub_node) {
                    sinks.push(pub_node.clone());
                }
            }
        }
    }

    ScopeSubgraph {
        graph,
        scope_subtree,
        sources,
        sinks,
    }
}

// ── Phase 35.3: critical-path latency computation ──

/// A critical path through the dataflow subgraph from a source to a sink.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct CriticalPath {
    /// Total worst-case latency (ms) along the path.
    pub total_ms: f64,
    /// Node FQNs visited in order, source first, sink last.
    pub nodes: Vec<String>,
}

/// Compute the worst-case (longest) latency from any source to any sink
/// in the subgraph using forward DP. Returns `None` if there is no path.
///
/// **Topology semantics:**
/// - Series chains: latencies sum.
/// - Fork-join (multiple incoming edges at a node): the node waits for
///   the slowest incoming branch, so we take the **max** over predecessors.
/// - Parallel branches don't sum at the join; only the slowest counts.
/// - State edges (`state: true`) don't propagate latency.
///
/// This is a topological-order DP where each node's latency is
/// `max(predecessor_latencies + edge_transport) + node_processing`.
#[allow(dead_code)]
pub fn critical_path(subgraph: &ScopeSubgraph) -> Option<CriticalPath> {
    if subgraph.sources.is_empty() || subgraph.sinks.is_empty() {
        return None;
    }

    // Topological sort the subgraph (skip state edges).
    let order = topo_sort(subgraph)?;

    // Forward DP: latency[node] = max over predecessors of
    // (latency[pred] + edge.transport) + node.processing.
    // Track the predecessor that gave the max for path reconstruction.
    let mut latency: HashMap<String, f64> = HashMap::new();
    let mut prev: HashMap<String, Option<String>> = HashMap::new();
    let source_set: HashSet<&str> = subgraph.sources.iter().map(|s| s.as_str()).collect();

    for node in &order {
        let processing = subgraph
            .graph
            .nodes
            .get(node)
            .map(|n| n.max_latency_ms())
            .unwrap_or(0.0);

        // If this is a source, initial latency is just its processing time.
        if source_set.contains(node.as_str()) {
            let cur = latency.entry(node.clone()).or_insert(0.0);
            if processing > *cur {
                *cur = processing;
            }
            prev.entry(node.clone()).or_insert(None);
            continue;
        }

        // Otherwise, take max over incoming causal edges from nodes already
        // visited in topological order.
        let in_indices = subgraph.graph.in_edges.get(node);
        let mut best: Option<(f64, String)> = None;
        if let Some(indices) = in_indices {
            for &idx in indices {
                let edge = &subgraph.graph.edges[idx];
                if edge.is_state || !subgraph.contains_node(&edge.from) {
                    continue;
                }
                let Some(&pred_lat) = latency.get(&edge.from) else {
                    continue;
                };
                let candidate = pred_lat + edge.max_transport_ms.unwrap_or(0.0);
                if best.as_ref().map(|(b, _)| candidate > *b).unwrap_or(true) {
                    best = Some((candidate, edge.from.clone()));
                }
            }
        }

        if let Some((arrival, predecessor)) = best {
            latency.insert(node.clone(), arrival + processing);
            prev.insert(node.clone(), Some(predecessor));
        }
    }

    // Find the sink with the maximum latency.
    let mut best_sink: Option<(String, f64)> = None;
    for sink in &subgraph.sinks {
        if let Some(&lat) = latency.get(sink) {
            if best_sink.as_ref().map(|(_, b)| lat > *b).unwrap_or(true) {
                best_sink = Some((sink.clone(), lat));
            }
        }
    }
    let (sink, total_ms) = best_sink?;

    // Reconstruct the path by walking `prev` backwards.
    let mut nodes = Vec::new();
    let mut cur = Some(sink);
    while let Some(node) = cur {
        let next = prev.get(&node).cloned().flatten();
        nodes.push(node);
        cur = next;
    }
    nodes.reverse();

    Some(CriticalPath { total_ms, nodes })
}

/// Topological sort restricted to the scope subtree, skipping state edges.
/// Returns `None` if a cycle is detected (other than via state edges).
fn topo_sort(subgraph: &ScopeSubgraph) -> Option<Vec<String>> {
    // Kahn's algorithm. In-degree = causal incoming edges within subtree.
    let mut indeg: HashMap<String, usize> = HashMap::new();
    let mut all_nodes: Vec<String> = subgraph
        .graph
        .nodes
        .keys()
        .filter(|fqn| subgraph.contains_node(fqn))
        .cloned()
        .collect();
    all_nodes.sort(); // Deterministic order

    for n in &all_nodes {
        indeg.insert(n.clone(), 0);
    }

    for n in &all_nodes {
        if let Some(indices) = subgraph.graph.in_edges.get(n) {
            for &idx in indices {
                let edge = &subgraph.graph.edges[idx];
                if edge.is_state || !subgraph.contains_node(&edge.from) {
                    continue;
                }
                *indeg.get_mut(n).unwrap() += 1;
            }
        }
    }

    let mut queue: std::collections::VecDeque<String> = all_nodes
        .iter()
        .filter(|n| indeg.get(*n).copied().unwrap_or(0) == 0)
        .cloned()
        .collect();
    let mut order = Vec::new();
    while let Some(n) = queue.pop_front() {
        order.push(n.clone());
        if let Some(indices) = subgraph.graph.out_edges.get(&n) {
            for &idx in indices {
                let edge = &subgraph.graph.edges[idx];
                if edge.is_state || !subgraph.contains_node(&edge.to) {
                    continue;
                }
                if let Some(d) = indeg.get_mut(&edge.to) {
                    *d -= 1;
                    if *d == 0 {
                        queue.push_back(edge.to.clone());
                    }
                }
            }
        }
    }
    if order.len() != all_nodes.len() {
        // Cycle detected (not via state edges) — abort.
        return None;
    }
    Some(order)
}
