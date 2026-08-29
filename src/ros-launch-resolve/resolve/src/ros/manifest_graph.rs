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
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

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
    /// Declared exclusion between this node's paths (phase 67). `None` means
    /// the conservative default — see [`GlobalNode::exclusion_groups`].
    pub concurrency: Option<ros_launch_manifest_types::ConcurrencyDecl>,
}

impl GlobalNode {
    /// Worst-case `max_latency_ms` across this node's paths.
    /// Returns 0.0 if no path declares a budget.
    ///
    /// This is the **fallback** cost, used only when a traversal cannot be
    /// attributed to a specific path (see [`GlobalNode::traversal_latency_ms`]).
    /// Charging it unconditionally is what made a route through a node's cheap
    /// output pay for its expensive one.
    pub fn max_latency_ms(&self) -> f64 {
        self.paths
            .values()
            .filter_map(|p| p.max_latency.map(|d| d.as_millis_f64()))
            .fold(0.0_f64, f64::max)
    }

    /// Names of paths whose `output` publishes `endpoint`.
    pub fn paths_producing(&self, endpoint: &str) -> Vec<&str> {
        self.paths
            .iter()
            .filter(|(_, p)| p.output.iter().any(|o| o == endpoint))
            .map(|(name, _)| name.as_str())
            .collect()
    }

    /// Names of paths whose *effective* trigger consumes `endpoint`.
    ///
    /// Reads `effective_trigger()` rather than the legacy `input:` field, so a
    /// path written in Vocabulary v2 (`trigger: { input: [...] }`) is matched.
    /// Timer-, once- and spontaneous-triggered paths consume nothing and so
    /// never match — a route cannot arrive *into* them, which is exactly what
    /// makes them chain boundaries.
    pub fn paths_consuming(&self, endpoint: &str) -> Vec<&str> {
        use ros_launch_manifest_types::EffectiveTrigger;
        self.paths
            .iter()
            .filter(|(_, p)| match p.effective_trigger() {
                EffectiveTrigger::Input(eps) => eps.iter().any(|e| e == endpoint),
                _ => false,
            })
            .map(|(name, _)| name.as_str())
            .collect()
    }

    /// Latency charged for traversing this node by way of `path`.
    ///
    /// `None` means the traversal could not be attributed to any declared
    /// path, in which case the conservative node-wide maximum applies — never
    /// less than the old behaviour, and never more than it either.
    ///
    /// A **timer-triggered** path is a clock boundary and is charged
    /// `period + exec` rather than `exec`: a message arriving at an arbitrary
    /// point in the period waits up to a whole period before the callback that
    /// forwards it runs, and that wait is latency no priority assignment can
    /// remove. Same formula and same one-period-per-boundary bound as
    /// `chain_checks`'s `sampling_cost`, so the chain and scope-path forms of
    /// one system agree. The boundary's own `exec` is included here and must
    /// not also be added elsewhere.
    pub fn traversal_latency_ms(&self, path: Option<&str>) -> f64 {
        use ros_launch_manifest_types::EffectiveTrigger;
        let Some(decl) = path.and_then(|name| self.paths.get(name)) else {
            return self.max_latency_ms();
        };
        let exec = decl.max_latency.map(|d| d.as_millis_f64()).unwrap_or(0.0);
        match decl.effective_trigger() {
            EffectiveTrigger::Timer { rate_hz } if rate_hz > 0.0 => 1000.0 / rate_hz + exec,
            _ => exec,
        }
    }

    /// Sets of this node's paths that may not run concurrently.
    ///
    /// **Derived, never authored** — a maximal mutually exclusive set IS a
    /// callback group, so the group is a consequence of the declared exclusion
    /// relation. nano-ros already infers groups this way and treats an explicit
    /// declaration as an override.
    ///
    /// The default is the load-bearing part. An absent `concurrency:` means
    /// every path of the node is in ONE group — which is what both
    /// realizations already do (`rclcpp`'s implicit per-node callback group is
    /// `MutuallyExclusive`, and nano-ros's `default_cbg_type` is the same
    /// string), so an author writes nothing unless claiming MORE concurrency
    /// than the safe answer. An explicit `exclusive: []` is therefore NOT the
    /// same as omitting the section: it says every path may run concurrently.
    ///
    /// Declared sets are merged transitively: `[[a, b], [b, c]]` is one group
    /// `{a, b, c}`, because exclusion is not transitive by intent but a shared
    /// member makes the three mutually serialised in any realization that maps
    /// a group to one thread.
    pub fn exclusion_groups(&self) -> Vec<BTreeSet<String>> {
        let Some(decl) = &self.concurrency else {
            // No declaration: everything serialises.
            return if self.paths.is_empty() {
                Vec::new()
            } else {
                vec![self.paths.keys().cloned().collect()]
            };
        };

        let mut groups: Vec<BTreeSet<String>> = Vec::new();
        for declared in &decl.exclusive {
            let mut merged: BTreeSet<String> = declared.iter().cloned().collect();
            // Absorb any existing group sharing a member.
            groups.retain(|g| {
                if g.is_disjoint(&merged) {
                    true
                } else {
                    merged.extend(g.iter().cloned());
                    false
                }
            });
            if !merged.is_empty() {
                groups.push(merged);
            }
        }
        groups
    }

    /// Paths of this node that must serialise with `path` — its group, minus
    /// itself. Empty when the path may run concurrently with everything.
    pub fn exclusive_siblings_of(&self, path: &str) -> BTreeSet<String> {
        let mut out: BTreeSet<String> = self
            .exclusion_groups()
            .into_iter()
            .find(|g| g.contains(path))
            .unwrap_or_default();
        out.remove(path);
        out
    }

    /// The sampling half of [`GlobalNode::traversal_latency_ms`], reported
    /// separately so a diagnostic can name it.
    ///
    /// Non-zero only for a timer-triggered path. Kept distinct because the
    /// two terms answer different questions — `exec` is work that scheduling
    /// can shorten, a period is a wait that it cannot — which is the whole
    /// reason `chain-sampling-feasibility` exists.
    pub fn sampling_cost_ms(&self, path: Option<&str>) -> f64 {
        use ros_launch_manifest_types::EffectiveTrigger;
        match path.and_then(|name| self.paths.get(name)) {
            Some(decl) => match decl.effective_trigger() {
                EffectiveTrigger::Timer { rate_hz } if rate_hz > 0.0 => 1000.0 / rate_hz,
                _ => 0.0,
            },
            None => 0.0,
        }
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
    /// Publisher endpoint name on the source node.
    ///
    /// The counterpart of `sub_endpoint`, and the half that lets a
    /// traversal be attributed to a *path* rather than to a node: the
    /// path that produced this hop is the one whose `output` names this
    /// endpoint. Without it the graph can only say "some path of `from`
    /// published something", which is why the critical path used to
    /// charge every route the maximum over all of a node's paths.
    pub pub_endpoint: String,
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
    //
    // Uses `manifest_loader::resolve_node_fqn` (not a locally re-derived
    // `scope.ns + bare_name`) so this graph's node keys agree with the
    // launch-dump-verified identity every other consumer
    // (`resolve_node_paths`/`resolve_topics`/`resolve_services`,
    // `causal_graph::build_export`) uses — see `ManifestIndex::node_identity`.
    for resolved in index.manifests.values() {
        for (node_name, node_decl) in &resolved.manifest.nodes {
            let fqn = super::manifest_loader::resolve_node_fqn(
                index,
                resolved.scope_id,
                &resolved.ns,
                node_name,
            );
            graph.nodes.insert(
                fqn.clone(),
                GlobalNode {
                    fqn,
                    scope_id: resolved.scope_id,
                    paths: node_decl.paths.clone(),
                    subscribers: node_decl.subscribers.clone(),
                    publishers: node_decl.publishers.clone(),
                    concurrency: node_decl.concurrency.clone(),
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
        for (pub_node, pub_ep) in &pub_node_eps {
            for (sub_node, sub_ep) in &sub_node_eps {
                let sub_props = graph
                    .nodes
                    .get(sub_node)
                    .and_then(|n| n.subscribers.get(sub_ep));
                let is_state = sub_props.and_then(|p| p.state).unwrap_or(false);
                // Per-subscriber transport latency override (Issue #44):
                // the same ROS topic can have heterogeneous transport across
                // subscribers (intra-process ~0ms vs cross-network ~10ms).
                // Prefer the sub endpoint's value; fall back to the topic
                // default; otherwise the edge contributes 0.
                let max_transport_ms = sub_props
                    .and_then(|p| p.max_transport.map(|d| d.as_millis_f64()))
                    .or(topic.max_transport_ms);

                let edge = GlobalEdge {
                    from: pub_node.clone(),
                    to: sub_node.clone(),
                    topic: topic_fqn.clone(),
                    pub_endpoint: pub_ep.clone(),
                    sub_endpoint: sub_ep.clone(),
                    max_transport_ms,
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
    /// The `(node, path)` pairs the route actually traverses, source first.
    ///
    /// `nodes` collapses these to node names for the diagnostic text; rules
    /// that need to know WHICH path was crossed — exclusion blocking, per-path
    /// cost attribution — read this.
    pub vertices: Vec<(String, Option<String>)>,
    /// The part of `total_ms` contributed by clock boundaries on this route —
    /// one period per timer-triggered path crossed.
    ///
    /// Reported separately because it is the part **no scheduling assignment
    /// can remove**: `exec` is work that a higher priority can shorten, a
    /// sampling period is a wait that it cannot. Same split
    /// `chain-sampling-feasibility` makes for chains.
    pub sampling_cost_ms: f64,
}

/// A vertex of the PATH-level dataflow graph: one declared path of one node.
///
/// `None` for the path name means "this node, reached by a traversal that no
/// declared path accounts for" — the fallback that preserves the old node-wide
/// cost rather than inventing a cheaper one.
type PathVertex = (String, Option<String>);

/// Lower a node-level subgraph to path granularity.
///
/// The facts a contract declares are per PATH — `trigger`, `output` and
/// `max_latency` all live on `PathDecl` — while the dataflow graph is keyed by
/// node. A node with two causal outputs (one image in, boxes and masks out at
/// different costs) cannot be represented at node granularity at all: there is
/// one vertex and two answers. So the DP runs here instead, where a vertex is a
/// path and an edge joins the path that PUBLISHED a topic to the path whose
/// trigger CONSUMES it.
///
/// Returns the vertex list (deterministically ordered) and the edges as
/// `(from_index, to_index, transport_ms)`.
fn build_path_graph(sg: &ScopeSubgraph) -> (Vec<PathVertex>, Vec<(usize, usize, f64)>) {
    use std::collections::BTreeSet;

    let mut vset: BTreeSet<PathVertex> = BTreeSet::new();
    for (fqn, node) in &sg.graph.nodes {
        if !sg.contains_node(fqn) {
            continue;
        }
        if node.paths.is_empty() {
            vset.insert((fqn.clone(), None));
        } else {
            for name in node.paths.keys() {
                vset.insert((fqn.clone(), Some(name.clone())));
            }
        }
    }

    // Resolve each node-level hop into the path pair(s) it actually connects.
    let mut raw: Vec<(PathVertex, PathVertex, f64)> = Vec::new();
    for edge in &sg.graph.edges {
        if edge.is_state || !sg.contains_node(&edge.from) || !sg.contains_node(&edge.to) {
            continue;
        }
        let (Some(from_node), Some(to_node)) =
            (sg.graph.nodes.get(&edge.from), sg.graph.nodes.get(&edge.to))
        else {
            continue;
        };

        // A hop no path claims still has to go somewhere: it lands on the
        // node's fallback vertex, which charges the node-wide maximum.
        let producers: Vec<Option<String>> = {
            let p = from_node.paths_producing(&edge.pub_endpoint);
            if p.is_empty() {
                vec![None]
            } else {
                p.into_iter().map(|s| Some(s.to_string())).collect()
            }
        };
        let consumers: Vec<Option<String>> = {
            let c = to_node.paths_consuming(&edge.sub_endpoint);
            if c.is_empty() {
                vec![None]
            } else {
                c.into_iter().map(|s| Some(s.to_string())).collect()
            }
        };

        let transport = edge.max_transport_ms.unwrap_or(0.0);
        for p in &producers {
            for c in &consumers {
                let a = (edge.from.clone(), p.clone());
                let b = (edge.to.clone(), c.clone());
                vset.insert(a.clone());
                vset.insert(b.clone());
                raw.push((a, b, transport));
            }
        }
    }

    let vertices: Vec<PathVertex> = vset.into_iter().collect();
    let index: HashMap<&PathVertex, usize> =
        vertices.iter().enumerate().map(|(i, v)| (v, i)).collect();
    let edges: Vec<(usize, usize, f64)> = raw
        .iter()
        .filter_map(|(a, b, t)| Some((*index.get(a)?, *index.get(b)?, *t)))
        .collect();

    (vertices, edges)
}

/// Kahn's algorithm over the path graph. `None` on a cycle.
fn topo_sort_paths(n: usize, edges: &[(usize, usize, f64)]) -> Option<Vec<usize>> {
    let mut indeg = vec![0usize; n];
    let mut out: Vec<Vec<usize>> = vec![Vec::new(); n];
    for &(a, b, _) in edges {
        indeg[b] += 1;
        out[a].push(b);
    }
    // Ascending seed order keeps the result deterministic for a given graph.
    let mut queue: std::collections::VecDeque<usize> = (0..n).filter(|i| indeg[*i] == 0).collect();
    let mut order = Vec::with_capacity(n);
    while let Some(v) = queue.pop_front() {
        order.push(v);
        for &w in &out[v] {
            indeg[w] -= 1;
            if indeg[w] == 0 {
                queue.push_back(w);
            }
        }
    }
    (order.len() == n).then_some(order)
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
/// **Granularity:** the DP runs over PATHS, not nodes (see
/// [`build_path_graph`]). A route through a node is charged the latency of the
/// path that actually produced the traversed topic, so an unrelated sibling
/// output — a second extraction from the same input, say — no longer inflates
/// it. Where a hop matches no declared path the node-wide maximum still
/// applies, so this is never less conservative than charging by node.
#[allow(dead_code)]
pub fn critical_path(subgraph: &ScopeSubgraph) -> Option<CriticalPath> {
    if subgraph.sources.is_empty() || subgraph.sinks.is_empty() {
        return None;
    }

    let (vertices, edges) = build_path_graph(subgraph);
    if vertices.is_empty() {
        return None;
    }
    let order = topo_sort_paths(vertices.len(), &edges)?;

    let mut incoming: Vec<Vec<(usize, f64)>> = vec![Vec::new(); vertices.len()];
    for &(a, b, t) in &edges {
        incoming[b].push((a, t));
    }

    let source_nodes: HashSet<&str> = subgraph.sources.iter().map(|s| s.as_str()).collect();
    let sink_nodes: HashSet<&str> = subgraph.sinks.iter().map(|s| s.as_str()).collect();

    let mut latency: Vec<Option<f64>> = vec![None; vertices.len()];
    let mut prev: Vec<Option<usize>> = vec![None; vertices.len()];

    for &v in &order {
        let (node_fqn, path_name) = &vertices[v];
        let cost = subgraph
            .graph
            .nodes
            .get(node_fqn)
            .map(|n| n.traversal_latency_ms(path_name.as_deref()))
            .unwrap_or(0.0);

        // A source may start the route — but it must not *truncate* one.
        //
        // `subgraph_for_scope_path` seeds sources from both the publishers and
        // the subscribers of the input topic, because an input published
        // outside the subtree has no publisher to start from. When the
        // publisher IS in the subtree, that makes the subscriber a source too,
        // and treating a source as having nothing upstream then discards the
        // hop that produced its data — dropping every node before it. So a
        // source takes the LARGER of "start here" and "arrive from upstream"
        // rather than skipping its incoming edges.
        let is_source = source_nodes.contains(node_fqn.as_str());

        let mut best: Option<(f64, usize)> = None;
        for &(pred, transport) in &incoming[v] {
            let Some(pred_lat) = latency[pred] else {
                continue;
            };
            let candidate = pred_lat + transport;
            if best.map(|(b, _)| candidate > b).unwrap_or(true) {
                best = Some((candidate, pred));
            }
        }
        match (best, is_source) {
            // Arriving from upstream beats starting here, or there is no
            // "starting here" to compare against.
            (Some((arrival, pred)), false) => {
                latency[v] = Some(arrival + cost);
                prev[v] = Some(pred);
            }
            (Some((arrival, pred)), true) => {
                if arrival + cost > cost {
                    latency[v] = Some(arrival + cost);
                    prev[v] = Some(pred);
                } else {
                    latency[v] = Some(cost);
                }
            }
            (None, true) => latency[v] = Some(cost),
            (None, false) => {}
        }
    }

    let mut best_sink: Option<(usize, f64)> = None;
    for (v, (node_fqn, _)) in vertices.iter().enumerate() {
        if !sink_nodes.contains(node_fqn.as_str()) {
            continue;
        }
        let Some(lat) = latency[v] else { continue };
        if best_sink.map(|(_, b)| lat > b).unwrap_or(true) {
            best_sink = Some((v, lat));
        }
    }
    let (sink_v, total_ms) = best_sink?;

    // Walk `prev` back to the source, reporting NODE names — several
    // consecutive vertices can belong to one node, and the diagnostic speaks
    // in nodes. The sampling term is summed over the SAME walk, so it is the
    // sampling cost of the winning route rather than of the whole subgraph.
    let mut nodes: Vec<String> = Vec::new();
    let mut traversed: Vec<(String, Option<String>)> = Vec::new();
    let mut sampling_cost_ms = 0.0_f64;
    let mut cur = Some(sink_v);
    while let Some(v) = cur {
        let (name, path_name) = &vertices[v];
        traversed.push((name.clone(), path_name.clone()));
        sampling_cost_ms += subgraph
            .graph
            .nodes
            .get(name)
            .map(|n| n.sampling_cost_ms(path_name.as_deref()))
            .unwrap_or(0.0);
        if nodes.last().map(|n| n != name).unwrap_or(true) {
            nodes.push(name.clone());
        }
        cur = prev[v];
    }
    nodes.reverse();
    traversed.reverse();

    Some(CriticalPath {
        total_ms,
        nodes,
        vertices: traversed,
        sampling_cost_ms,
    })
}

// ── Rate propagation (phase 68 follow-up) ────────────────────────────────────

/// A topic's publication rate, derived from the timers that ultimately drive
/// it — or the reason it could not be.
///
/// `Unknown` is a first-class answer, not an error and not zero. The same
/// absent-versus-zero distinction phase 60 removed from the chain checker and
/// phase 58 W2 kept in `measure`: a topic whose rate cannot be derived must
/// read as "not derivable, here is why", because reporting 0 Hz would be a
/// claim, and omitting it silently reads as "nothing to say".
#[derive(Debug, Clone, PartialEq)]
pub enum DerivedRate {
    Hz(f64),
    Unknown(&'static str),
}

/// Derive every topic's publication rate by propagating from the timer paths
/// that start each chain.
///
/// A rate is a CONSEQUENCE: a timer path publishes at its own rate, and an
/// input-triggered path publishes at the rate its inputs arrive. Both facts
/// are already declared, so the number written on `topics.<t>.rate_hz` is a
/// second copy — which is the whole subject of `contract-primitives.md`.
///
/// # The arithmetic, and why each case is what it is
///
/// - **Timer** → its own `rate_hz`. This is the only source; nothing else
///   creates messages.
/// - **Input, no `sync:`** → the **SUM** of its inputs' rates. A subscription
///   callback fires once per message on *each* topic it is registered for, so
///   a path triggered by two 10 Hz topics runs 20 times a second. Taking the
///   min here would be the natural-looking mistake and would understate a
///   fan-in node's load by exactly the factor that matters for scheduling.
/// - **Input, with `sync:`** → the **MIN**. A synchronizer emits one output
///   per matched set, so it is paced by its slowest input.
/// - **Several paths producing one endpoint, or several nodes publishing one
///   topic** → the sum, for the same reason: they publish independently.
/// - **`once` / `spontaneous` / unclassified** → `Unknown`. Nothing in the
///   contract says how often an event-driven or one-shot path fires; that is
///   a genuine absence of information, not a rate of zero.
/// - **A cycle** → `Unknown`. A feedback loop's steady-state rate is not
///   determined by the declarations alone, and guessing one would be worse
///   than saying so.
///
/// Any unknown contributor makes the whole sum unknown: a partial sum would be
/// a lower bound presented as a rate.
pub fn derive_topic_rates(
    index: &ManifestIndex,
    graph: &GlobalDataflowGraph,
) -> HashMap<String, DerivedRate> {
    // (node FQN, subscriber endpoint) -> topic FQN, so an input-triggered
    // path can find the topic each of its trigger endpoints reads.
    let mut sub_topic: HashMap<(String, String), String> = HashMap::new();
    for (topic_fqn, topic) in &index.topics {
        for ep_ref in &topic.subscribers {
            if let Some((node, ep)) = split_endpoint_ref(ep_ref) {
                sub_topic.insert((node, ep), topic_fqn.clone());
            }
        }
    }

    let mut memo: HashMap<String, DerivedRate> = HashMap::new();
    let mut in_progress: HashSet<String> = HashSet::new();
    let topics: Vec<String> = index.topics.keys().cloned().collect();
    for topic in &topics {
        let r = topic_rate(topic, index, graph, &sub_topic, &mut memo, &mut in_progress);
        memo.insert(topic.clone(), r);
    }
    memo
}

fn topic_rate(
    topic_fqn: &str,
    index: &ManifestIndex,
    graph: &GlobalDataflowGraph,
    sub_topic: &HashMap<(String, String), String>,
    memo: &mut HashMap<String, DerivedRate>,
    in_progress: &mut HashSet<String>,
) -> DerivedRate {
    if let Some(cached) = memo.get(topic_fqn) {
        return cached.clone();
    }
    if !in_progress.insert(topic_fqn.to_string()) {
        return DerivedRate::Unknown("a cycle in the dataflow graph");
    }

    let result = (|| {
        let Some(topic) = index.topics.get(topic_fqn) else {
            return DerivedRate::Unknown("no such topic in the merged index");
        };
        if topic.publishers.is_empty() {
            return DerivedRate::Unknown("no declared publisher");
        }

        let mut total = 0.0;
        for ep_ref in &topic.publishers {
            let Some((node_fqn, endpoint)) = split_endpoint_ref(ep_ref) else {
                return DerivedRate::Unknown("a publisher endpoint that does not parse");
            };
            let Some(node) = graph.nodes.get(&node_fqn) else {
                return DerivedRate::Unknown("a publisher with no node declaration");
            };
            let producing = node.paths_producing(&endpoint);
            if producing.is_empty() {
                return DerivedRate::Unknown("a publisher endpoint no declared path produces");
            }
            for path_name in producing {
                match path_rate(node, path_name, index, graph, sub_topic, memo, in_progress) {
                    DerivedRate::Hz(hz) => total += hz,
                    unknown => return unknown,
                }
            }
        }
        DerivedRate::Hz(total)
    })();

    in_progress.remove(topic_fqn);
    result
}

fn path_rate(
    node: &GlobalNode,
    path_name: &str,
    index: &ManifestIndex,
    graph: &GlobalDataflowGraph,
    sub_topic: &HashMap<(String, String), String>,
    memo: &mut HashMap<String, DerivedRate>,
    in_progress: &mut HashSet<String>,
) -> DerivedRate {
    use ros_launch_manifest_types::EffectiveTrigger as T;
    let Some(decl) = node.paths.get(path_name) else {
        return DerivedRate::Unknown("a path that is not declared");
    };
    match decl.effective_trigger() {
        T::Timer { rate_hz } if rate_hz > 0.0 => DerivedRate::Hz(rate_hz),
        T::Timer { .. } => DerivedRate::Unknown("a timer declaring a non-positive rate"),
        T::Input(endpoints) if !endpoints.is_empty() => {
            // A synchronizer emits one output per matched set, so it is paced
            // by its slowest input; an ordinary multi-input callback fires per
            // message on any of them, so those rates add.
            let synchronized = decl.sync.is_some();
            let mut acc: Option<f64> = None;
            for ep in &endpoints {
                let Some(t) = sub_topic.get(&(node.fqn.clone(), ep.clone())) else {
                    return DerivedRate::Unknown("a trigger endpoint bound to no topic");
                };
                match topic_rate(t, index, graph, sub_topic, memo, in_progress) {
                    DerivedRate::Hz(hz) => {
                        acc = Some(match acc {
                            None => hz,
                            Some(a) if synchronized => a.min(hz),
                            Some(a) => a + hz,
                        })
                    }
                    unknown => return unknown,
                }
            }
            match acc {
                Some(hz) => DerivedRate::Hz(hz),
                None => DerivedRate::Unknown("an input trigger with no endpoints"),
            }
        }
        T::Input(_) => DerivedRate::Unknown("an input trigger with no endpoints"),
        T::Once => DerivedRate::Unknown("a `once` trigger — it fires exactly once"),
        T::Spontaneous => {
            DerivedRate::Unknown("a `spontaneous` trigger — the contract says nothing about when")
        }
        T::Unclassified => DerivedRate::Unknown("a path with no declared trigger"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_launch_manifest_types::{PathDecl, Trigger};

    fn timer_path(rate_hz: f64, out: &[&str], latency_ms: Option<f64>) -> PathDecl {
        PathDecl {
            trigger: Some(Trigger::Timer { rate_hz }),
            output: out.iter().map(|s| s.to_string()).collect(),
            max_latency: latency_ms
                .map(ros_launch_manifest_types::duration::Duration::from_millis_f64),
            ..Default::default()
        }
    }

    fn path(trigger_in: &[&str], out: &[&str], latency_ms: Option<f64>) -> PathDecl {
        PathDecl {
            trigger: (!trigger_in.is_empty())
                .then(|| Trigger::Input(trigger_in.iter().map(|s| s.to_string()).collect())),
            output: out.iter().map(|s| s.to_string()).collect(),
            max_latency: latency_ms
                .map(ros_launch_manifest_types::duration::Duration::from_millis_f64),
            ..Default::default()
        }
    }

    fn node(fqn: &str, paths: &[(&str, PathDecl)]) -> GlobalNode {
        GlobalNode {
            fqn: fqn.to_string(),
            scope_id: 0,
            paths: paths
                .iter()
                .map(|(n, p)| (n.to_string(), p.clone()))
                .collect(),
            subscribers: BTreeMap::new(),
            publishers: BTreeMap::new(),
            concurrency: None,
        }
    }

    fn edge(from: &str, pub_ep: &str, to: &str, sub_ep: &str) -> GlobalEdge {
        GlobalEdge {
            from: from.to_string(),
            to: to.to_string(),
            topic: format!("/{pub_ep}"),
            pub_endpoint: pub_ep.to_string(),
            sub_endpoint: sub_ep.to_string(),
            max_transport_ms: None,
            is_state: false,
        }
    }

    fn assemble(nodes: Vec<GlobalNode>, edges: Vec<GlobalEdge>) -> GlobalDataflowGraph {
        let mut g = GlobalDataflowGraph::default();
        for n in nodes {
            g.nodes.insert(n.fqn.clone(), n);
        }
        for (i, e) in edges.into_iter().enumerate() {
            g.out_edges.entry(e.from.clone()).or_default().push(i);
            g.in_edges.entry(e.to.clone()).or_default().push(i);
            g.edges.push(e);
        }
        g
    }

    /// The motivating topology: ONE node, one input, two distinct causal
    /// outputs with different costs, each feeding a different consumer.
    ///
    ///   /image -> detector -+-(boxes, 20ms)-> tracker   (10ms) -> /tracks
    ///                       +-(masks, Xms) -> segmenter ( 5ms) -> /seg
    fn two_output_graph(masks_ms: f64) -> GlobalDataflowGraph {
        assemble(
            vec![
                node(
                    "/detector",
                    &[
                        ("to_boxes", path(&["image"], &["boxes"], Some(20.0))),
                        ("to_masks", path(&["image"], &["masks"], Some(masks_ms))),
                    ],
                ),
                node(
                    "/tracker",
                    &[("main", path(&["boxes"], &["tracks"], Some(10.0)))],
                ),
                node(
                    "/segmenter",
                    &[("main", path(&["masks"], &["seg"], Some(5.0)))],
                ),
            ],
            vec![
                edge("/detector", "boxes", "/tracker", "boxes"),
                edge("/detector", "masks", "/segmenter", "masks"),
            ],
        )
    }

    fn subgraph<'a>(g: &'a GlobalDataflowGraph, source: &str, sink: &str) -> ScopeSubgraph<'a> {
        ScopeSubgraph {
            graph: g,
            scope_subtree: HashSet::from([0usize]),
            sources: vec![source.to_string()],
            sinks: vec![sink.to_string()],
        }
    }

    /// A route is charged the path that produced the topic it traverses, not
    /// the node's most expensive output. `/tracks` is reached through `boxes`
    /// (20ms) + tracker (10ms) = 30ms, and must stay 30ms however costly the
    /// unrelated `masks` path becomes.
    ///
    /// Before the path-level DP this returned 45 / 110 / 30 for the three
    /// cases below — tracking an output the route never touches, one for one.
    #[test]
    fn route_is_charged_its_own_path_not_the_node_maximum() {
        for masks_ms in [35.0, 100.0, 12.0] {
            let g = two_output_graph(masks_ms);
            let cp = critical_path(&subgraph(&g, "/detector", "/tracker"))
                .expect("a route from detector to tracker exists");
            assert_eq!(
                cp.total_ms, 30.0,
                "to_tracks must cost 20+10=30ms with to_masks at {masks_ms}ms, got {}",
                cp.total_ms
            );
            assert_eq!(cp.nodes, vec!["/detector", "/tracker"]);
        }
    }

    /// The sibling route IS charged the expensive path — the fix must not make
    /// everything cheap, only attribute correctly.
    #[test]
    fn the_other_route_still_pays_its_own_cost() {
        for (masks_ms, expected) in [(35.0, 40.0), (100.0, 105.0), (12.0, 17.0)] {
            let g = two_output_graph(masks_ms);
            let cp = critical_path(&subgraph(&g, "/detector", "/segmenter"))
                .expect("a route from detector to segmenter exists");
            assert_eq!(
                cp.total_ms, expected,
                "to_seg must cost {masks_ms}+5={expected}ms, got {}",
                cp.total_ms
            );
        }
    }

    /// Fork-join still takes the max over branches and adds the join, at path
    /// granularity: max(50, 30) + 20 = 70, never the sum 100.
    #[test]
    fn fork_join_takes_max_not_sum() {
        let g = assemble(
            vec![
                node(
                    "/lidar",
                    &[("main", path(&["input"], &["out"], Some(50.0)))],
                ),
                node(
                    "/camera",
                    &[("main", path(&["input"], &["out"], Some(30.0)))],
                ),
                node(
                    "/fusion",
                    &[("main", path(&["lidar", "camera"], &["out"], Some(20.0)))],
                ),
            ],
            vec![
                edge("/lidar", "out", "/fusion", "lidar"),
                edge("/camera", "out", "/fusion", "camera"),
            ],
        );
        let sg = ScopeSubgraph {
            graph: &g,
            scope_subtree: HashSet::from([0usize]),
            sources: vec!["/lidar".to_string(), "/camera".to_string()],
            sinks: vec!["/fusion".to_string()],
        };
        let cp = critical_path(&sg).expect("fork-join has a critical path");
        assert_eq!(cp.total_ms, 70.0, "max(50,30)+20, not the sum 100");
    }

    /// A hop no declared path accounts for falls back to the node-wide
    /// maximum, so the change is never less conservative than charging by
    /// node. Here the edge arrives on an endpoint `main` does not consume.
    #[test]
    fn unattributable_hop_falls_back_to_the_node_maximum() {
        let g = assemble(
            vec![
                node("/src", &[("main", path(&["in"], &["out"], Some(7.0)))]),
                node(
                    "/sink",
                    &[
                        ("a", path(&["expected"], &["o"], Some(3.0))),
                        ("b", path(&["other"], &["o"], Some(11.0))),
                    ],
                ),
            ],
            // arrives on `surprise`, which neither path declares
            vec![edge("/src", "out", "/sink", "surprise")],
        );
        let cp = critical_path(&subgraph(&g, "/src", "/sink")).expect("route exists");
        assert_eq!(
            cp.total_ms, 18.0,
            "7 + max(3,11) — the conservative fallback, not the cheaper path"
        );
    }

    /// `rt_workspace`'s three-node chain, expressed as a SCOPE PATH rather
    /// than as `chains:` / `segments:`:
    ///
    ///   sensor_node  timer 100Hz -> points_raw       (no declared exec)
    ///   filter       input       -> points_filtered  5ms
    ///   control      input       -> cmd              10ms
    ///
    /// `chain_checks` reports this as `25.00ms = 15.00ms event-segment +
    /// 10.00ms sampling_cost`. The scope-path form must agree, because the
    /// two are descriptions of one system — that agreement is what makes
    /// retiring `segments:` lossless.
    fn rt_workspace_graph() -> GlobalDataflowGraph {
        assemble(
            vec![
                node(
                    "/perception/sensor_node",
                    &[("tick", timer_path(100.0, &["points_raw"], None))],
                ),
                node(
                    "/perception/filter_component",
                    &[(
                        "filter",
                        path(&["points_raw"], &["points_filtered"], Some(5.0)),
                    )],
                ),
                node(
                    "/control/control_node",
                    &[("control", path(&["points_filtered"], &["cmd"], Some(10.0)))],
                ),
            ],
            vec![
                edge(
                    "/perception/sensor_node",
                    "points_raw",
                    "/perception/filter_component",
                    "points_raw",
                ),
                edge(
                    "/perception/filter_component",
                    "points_filtered",
                    "/control/control_node",
                    "points_filtered",
                ),
            ],
        )
    }

    /// The acceptance test for W1.b. Both forms of one system must total
    /// 25ms; against a 20ms budget both must complain.
    #[test]
    fn scope_path_total_matches_the_chain_form() {
        let g = rt_workspace_graph();
        // `subgraph_for_scope_path` seeds sources from BOTH the publishers and
        // the subscribers of the input topic, so the real subgraph for a path
        // whose input is /perception/points_raw has two sources.
        let sg = ScopeSubgraph {
            graph: &g,
            scope_subtree: HashSet::from([0usize]),
            sources: vec![
                "/perception/sensor_node".to_string(),
                "/perception/filter_component".to_string(),
            ],
            sinks: vec!["/control/control_node".to_string()],
        };
        let cp = critical_path(&sg).expect("the chain has a critical path");
        assert_eq!(
            cp.total_ms, 25.0,
            "10ms sampling + 5ms filter + 10ms control, matching chain-budget"
        );
        assert_eq!(
            cp.sampling_cost_ms, 10.0,
            "one period of the 100Hz boundary, reported separately"
        );
        assert_eq!(
            cp.nodes,
            vec![
                "/perception/sensor_node",
                "/perception/filter_component",
                "/control/control_node"
            ],
            "the route must not be truncated at the subscriber-source"
        );
    }

    /// The sampling term tracks the boundary's rate, and is a *period*, not a
    /// deadline: halving the rate doubles it.
    #[test]
    fn sampling_cost_is_one_period_per_boundary() {
        for (rate_hz, expected_sampling) in [(100.0, 10.0), (50.0, 20.0), (10.0, 100.0)] {
            let g = assemble(
                vec![
                    node("/src", &[("tick", timer_path(rate_hz, &["out"], None))]),
                    node("/sink", &[("main", path(&["in"], &["o"], Some(4.0)))]),
                ],
                vec![edge("/src", "out", "/sink", "in")],
            );
            let cp = critical_path(&subgraph(&g, "/src", "/sink")).expect("route exists");
            assert_eq!(cp.sampling_cost_ms, expected_sampling);
            assert_eq!(cp.total_ms, expected_sampling + 4.0);
        }
    }

    /// A boundary that declares its own execution budget is charged
    /// `period + exec`, matching `chain_checks` exactly — and that exec must
    /// not then be counted a second time.
    #[test]
    fn a_boundary_with_a_declared_exec_is_charged_period_plus_exec() {
        let g = assemble(
            vec![
                node("/src", &[("tick", timer_path(100.0, &["out"], Some(3.0)))]),
                node("/sink", &[("main", path(&["in"], &["o"], Some(4.0)))]),
            ],
            vec![edge("/src", "out", "/sink", "in")],
        );
        let cp = critical_path(&subgraph(&g, "/src", "/sink")).expect("route exists");
        assert_eq!(cp.total_ms, 17.0, "10 period + 3 exec + 4 = 17, not 20");
        assert_eq!(cp.sampling_cost_ms, 10.0, "the period alone");
    }

    /// An event-triggered route has no clock crossing, so nothing is added.
    /// Guards against charging a period to every route.
    #[test]
    fn an_event_only_route_has_no_sampling_cost() {
        let g = assemble(
            vec![
                node("/src", &[("main", path(&["in"], &["out"], Some(6.0)))]),
                node("/sink", &[("main", path(&["out"], &["o"], Some(4.0)))]),
            ],
            vec![edge("/src", "out", "/sink", "out")],
        );
        let cp = critical_path(&subgraph(&g, "/src", "/sink")).expect("route exists");
        assert_eq!(cp.sampling_cost_ms, 0.0);
        assert_eq!(cp.total_ms, 10.0);
    }

    fn node_with_concurrency(
        fqn: &str,
        paths: &[(&str, PathDecl)],
        exclusive: Option<Vec<Vec<&str>>>,
    ) -> GlobalNode {
        let mut n = node(fqn, paths);
        n.concurrency = exclusive.map(|groups| ros_launch_manifest_types::ConcurrencyDecl {
            exclusive: groups
                .into_iter()
                .map(|g| g.into_iter().map(|s| s.to_string()).collect())
                .collect(),
        });
        n
    }

    fn p() -> PathDecl {
        path(&["in"], &["out"], Some(1.0))
    }

    /// The default is the load-bearing part: with no `concurrency:` every path
    /// of the node is in ONE group, matching rclcpp's implicit
    /// `MutuallyExclusive` callback group and nano-ros's `default_cbg_type`.
    #[test]
    fn absent_concurrency_puts_every_path_in_one_group() {
        let n = node_with_concurrency("/n", &[("a", p()), ("b", p()), ("c", p())], None);
        let groups = n.exclusion_groups();
        assert_eq!(groups.len(), 1);
        assert_eq!(groups[0].len(), 3);
        assert_eq!(n.exclusive_siblings_of("a").len(), 2);
    }

    /// An explicit empty list is NOT the same as omitting the section: it says
    /// every path may run concurrently.
    #[test]
    fn an_empty_exclusive_list_means_everything_is_concurrent() {
        let n = node_with_concurrency("/n", &[("a", p()), ("b", p())], Some(vec![]));
        assert!(n.exclusion_groups().is_empty());
        assert!(n.exclusive_siblings_of("a").is_empty());
    }

    /// Declared sets sharing a member merge: `[[a,b],[b,c]]` is one group,
    /// because any realization mapping a group to one thread serialises all
    /// three.
    #[test]
    fn groups_sharing_a_member_merge() {
        let n = node_with_concurrency(
            "/n",
            &[("a", p()), ("b", p()), ("c", p()), ("d", p())],
            Some(vec![vec!["a", "b"], vec!["b", "c"], vec!["d"]]),
        );
        let groups = n.exclusion_groups();
        assert_eq!(groups.len(), 2, "{{a,b,c}} and {{d}}");
        let abc = groups.iter().find(|g| g.len() == 3).expect("merged group");
        assert!(abc.contains("a") && abc.contains("b") && abc.contains("c"));
        assert_eq!(n.exclusive_siblings_of("d").len(), 0);
        assert_eq!(n.exclusive_siblings_of("a").len(), 2);
    }

    /// A partial declaration leaves unnamed paths concurrent — the author
    /// opted them out by not naming them.
    #[test]
    fn paths_outside_a_declared_group_are_concurrent() {
        let n = node_with_concurrency(
            "/n",
            &[("a", p()), ("b", p()), ("health", p())],
            Some(vec![vec!["a", "b"]]),
        );
        assert!(n.exclusive_siblings_of("health").is_empty());
        assert_eq!(n.exclusive_siblings_of("a"), ["b".to_string()].into());
    }

    // ── Rate propagation ────────────────────────────────────────────────────

    /// Build an index whose topics wire `(node, endpoint)` pairs together,
    /// which is what the rate walk reads (the graph's own edges only exist
    /// where a topic has a subscriber).
    fn rate_index(
        topics: &[(&str, &[&str], &[&str])],
    ) -> crate::ros::manifest_loader::ManifestIndex {
        use crate::ros::manifest_loader::{ManifestIndex, ResolvedTopic};
        let mut index = ManifestIndex::default();
        for (fqn, pubs, subs) in topics {
            index.topics.insert(
                fqn.to_string(),
                ResolvedTopic {
                    fqn: fqn.to_string(),
                    msg_type: "std_msgs/msg/String".to_string(),
                    qos: None,
                    publishers: pubs.iter().map(|s| s.to_string()).collect(),
                    subscribers: subs.iter().map(|s| s.to_string()).collect(),
                    rate_hz: None,
                    derived_rate_hz: None,
                    max_transport_ms: None,
                    drop: None,
                    scope_ids: vec![0],
                },
            );
        }
        index
    }

    /// The base case: a timer's rate reaches a topic two hops downstream.
    /// This is what lets `rt_workspace` delete eight of the nine copies of
    /// `100` it used to carry.
    #[test]
    fn a_timer_rate_propagates_along_the_chain() {
        let g = assemble(
            vec![
                node("/a", &[("tick", timer_path(100.0, &["out"], None))]),
                node("/b", &[("relay", path(&["inp"], &["out"], None))]),
            ],
            vec![],
        );
        let index = rate_index(&[("/t1", &["/a/out"], &["/b/inp"]), ("/t2", &["/b/out"], &[])]);
        let rates = derive_topic_rates(&index, &g);
        assert_eq!(rates["/t1"], DerivedRate::Hz(100.0));
        assert_eq!(rates["/t2"], DerivedRate::Hz(100.0));
    }

    /// Two inputs, no `sync:` — the rates ADD.
    ///
    /// A subscription callback fires once per message on *each* topic it is
    /// registered for, so a path triggered by a 10 Hz and a 30 Hz topic runs
    /// 40 times a second. Taking the min (or the max) is the natural-looking
    /// mistake and understates a fan-in node's load by exactly the factor
    /// that decides whether it fits.
    #[test]
    fn fan_in_without_sync_sums_its_input_rates() {
        let g = assemble(
            vec![
                node("/fast", &[("tick", timer_path(30.0, &["out"], None))]),
                node("/slow", &[("tick", timer_path(10.0, &["out"], None))]),
                node("/merge", &[("both", path(&["a", "b"], &["out"], None))]),
            ],
            vec![],
        );
        let index = rate_index(&[
            ("/fast_t", &["/fast/out"], &["/merge/a"]),
            ("/slow_t", &["/slow/out"], &["/merge/b"]),
            ("/merged", &["/merge/out"], &[]),
        ]);
        let rates = derive_topic_rates(&index, &g);
        assert_eq!(rates["/merged"], DerivedRate::Hz(40.0));
    }

    /// The same shape with `sync:` declared — a synchronizer emits one output
    /// per matched set, so it is paced by its SLOWEST input.
    #[test]
    fn fan_in_with_sync_takes_the_slowest_input() {
        use ros_launch_manifest_types::{Sync, SyncPolicy};
        let mut both = path(&["a", "b"], &["out"], None);
        both.sync = Some(Sync {
            policy: SyncPolicy::Approximate,
            max_interval: Some(
                ros_launch_manifest_types::duration::Duration::from_millis_f64(20.0),
            ),
            timeout: None,
        });
        let g = assemble(
            vec![
                node("/fast", &[("tick", timer_path(30.0, &["out"], None))]),
                node("/slow", &[("tick", timer_path(10.0, &["out"], None))]),
                node("/merge", &[("both", both)]),
            ],
            vec![],
        );
        let index = rate_index(&[
            ("/fast_t", &["/fast/out"], &["/merge/a"]),
            ("/slow_t", &["/slow/out"], &["/merge/b"]),
            ("/merged", &["/merge/out"], &[]),
        ]);
        let rates = derive_topic_rates(&index, &g);
        assert_eq!(rates["/merged"], DerivedRate::Hz(10.0));
    }

    /// A cycle is UNKNOWN, not zero and not a hang. A feedback loop's
    /// steady-state rate is not determined by the declarations alone.
    #[test]
    fn a_cycle_is_unknown_rather_than_a_number() {
        let g = assemble(
            vec![
                node("/a", &[("loop", path(&["inp"], &["out"], None))]),
                node("/b", &[("loop", path(&["inp"], &["out"], None))]),
            ],
            vec![],
        );
        let index = rate_index(&[
            ("/ab", &["/a/out"], &["/b/inp"]),
            ("/ba", &["/b/out"], &["/a/inp"]),
        ]);
        let rates = derive_topic_rates(&index, &g);
        assert!(
            matches!(rates["/ab"], DerivedRate::Unknown(_)),
            "got {:?}",
            rates["/ab"]
        );
    }

    /// An unknown ANYWHERE upstream makes the result unknown, rather than a
    /// partial sum presented as a rate. `spontaneous` says the contract does
    /// not know when the path fires, and a downstream rate cannot know better.
    #[test]
    fn an_unknown_upstream_is_not_silently_treated_as_zero() {
        use ros_launch_manifest_types::Trigger;
        let mut spont = PathDecl {
            output: vec!["out".to_string()],
            ..Default::default()
        };
        spont.trigger = Some(Trigger::Spontaneous);
        let g = assemble(
            vec![
                node("/ext", &[("event", spont)]),
                node("/down", &[("relay", path(&["inp"], &["out"], None))]),
            ],
            vec![],
        );
        let index = rate_index(&[
            ("/events", &["/ext/out"], &["/down/inp"]),
            ("/downstream", &["/down/out"], &[]),
        ]);
        let rates = derive_topic_rates(&index, &g);
        match &rates["/downstream"] {
            DerivedRate::Unknown(why) => assert!(why.contains("spontaneous"), "{why}"),
            other => panic!("expected Unknown, got {other:?}"),
        }
    }
}
