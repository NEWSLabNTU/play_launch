//! Dataflow graph construction from a manifest.

use petgraph::graph::{DiGraph, NodeIndex};
use ros_launch_manifest_types::Manifest;
use std::collections::HashMap;

/// A node in the dataflow graph.
#[derive(Debug, Clone)]
pub enum GraphNode {
    /// A ROS node from the manifest.
    Node { name: String },
    /// A child scope (include).
    Scope { name: String },
}

impl GraphNode {
    pub fn name(&self) -> &str {
        match self {
            GraphNode::Node { name } | GraphNode::Scope { name } => name,
        }
    }
}

/// An edge in the dataflow graph (a topic connection).
#[derive(Debug, Clone)]
pub struct GraphEdge {
    pub topic: String,
}

/// The dataflow graph built from a manifest.
pub struct DataflowGraph {
    pub graph: DiGraph<GraphNode, GraphEdge>,
    /// Map from node/scope name to graph index.
    pub index_map: HashMap<String, NodeIndex>,
}

impl DataflowGraph {
    /// Build the dataflow graph from a manifest.
    ///
    /// Nodes and includes become vertices. Topic wiring creates edges.
    pub fn build(manifest: &Manifest) -> Self {
        let mut graph = DiGraph::new();
        let mut index_map = HashMap::new();

        // Add nodes
        for name in manifest.nodes.keys() {
            let idx = graph.add_node(GraphNode::Node { name: name.clone() });
            index_map.insert(name.clone(), idx);
        }

        // Add includes as scope vertices
        for name in manifest.includes.keys() {
            let idx = graph.add_node(GraphNode::Scope { name: name.clone() });
            index_map.insert(name.clone(), idx);
        }

        // Build endpoint → vertex map for topic wiring
        // Format: "node_name/endpoint" → vertex index
        let mut endpoint_to_vertex: HashMap<String, NodeIndex> = HashMap::new();

        for (node_name, node_decl) in &manifest.nodes {
            if let Some(idx) = index_map.get(node_name) {
                for ep_name in node_decl.publishers.keys() {
                    endpoint_to_vertex.insert(format!("{node_name}/{ep_name}"), *idx);
                }
                for ep_name in node_decl.subscribers.keys() {
                    endpoint_to_vertex.insert(format!("{node_name}/{ep_name}"), *idx);
                }
            }
        }

        // Include exports/imports map to the scope vertex
        for scope_name in manifest.includes.keys() {
            if let Some(idx) = index_map.get(scope_name) {
                // Exports from this scope are publishers
                // Imports to this scope are subscribers
                // They're referenced as "scope_name/group_name" in topics
                // We map any "scope_name/*" to the scope vertex
                endpoint_to_vertex.insert(format!("{scope_name}/"), *idx);
            }
        }

        // Add edges from topic wiring (skip state endpoints — they don't carry causality)
        for (topic_name, topic_decl) in &manifest.topics {
            for pub_ref in &topic_decl.publishers {
                for sub_ref in &topic_decl.subscribers {
                    // Skip edges where the subscriber endpoint is marked state: true
                    if is_state_endpoint(sub_ref, manifest) {
                        continue;
                    }
                    let pub_vertex = resolve_vertex(pub_ref, &endpoint_to_vertex, &index_map);
                    let sub_vertex = resolve_vertex(sub_ref, &endpoint_to_vertex, &index_map);

                    if let (Some(from), Some(to)) = (pub_vertex, sub_vertex) {
                        graph.add_edge(
                            from,
                            to,
                            GraphEdge {
                                topic: topic_name.clone(),
                            },
                        );
                    }
                }
            }
        }

        Self { graph, index_map }
    }
}

/// Check if an endpoint reference points to a state endpoint (read-latest, not causal).
fn is_state_endpoint(endpoint_ref: &str, manifest: &Manifest) -> bool {
    if let Some((node_name, ep_name)) = endpoint_ref.split_once('/')
        && let Some(node) = manifest.nodes.get(node_name)
        && let Some(props) = node.subscribers.get(ep_name)
    {
        return props.state.unwrap_or(false);
    }
    false
}

/// Resolve an endpoint reference like "node/endpoint" or "scope/group" to a vertex.
fn resolve_vertex(
    endpoint_ref: &str,
    endpoint_to_vertex: &HashMap<String, NodeIndex>,
    index_map: &HashMap<String, NodeIndex>,
) -> Option<NodeIndex> {
    // Direct match
    if let Some(idx) = endpoint_to_vertex.get(endpoint_ref) {
        return Some(*idx);
    }
    // Try scope prefix match: "scope_name/anything" → scope vertex
    if let Some(slash_pos) = endpoint_ref.find('/') {
        let scope_name = &endpoint_ref[..slash_pos];
        if let Some(idx) = index_map.get(scope_name) {
            return Some(*idx);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_launch_manifest_types::parse_manifest_str;

    #[test]
    fn test_build_simple_graph() {
        let yaml = r#"
version: 1
nodes:
  talker:
    pub: [chatter]
  listener:
    sub: [chatter]
topics:
  chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
    sub: [listener/chatter]
"#;
        let m = parse_manifest_str(yaml).unwrap();
        let g = DataflowGraph::build(&m);

        assert_eq!(g.graph.node_count(), 2);
        assert_eq!(g.graph.edge_count(), 1);
        assert!(g.index_map.contains_key("talker"));
        assert!(g.index_map.contains_key("listener"));
    }

    #[test]
    fn test_build_pipeline_graph() {
        let yaml = r#"
version: 1
nodes:
  cropbox:
    pub: [output]
    sub: [input]
  ground:
    pub: [output]
    sub: [input]
  centerpoint:
    pub: [objects]
    sub: [pointcloud]
topics:
  cropped:
    type: PointCloud2
    pub: [cropbox/output]
    sub: [ground/input]
  no_ground:
    type: PointCloud2
    pub: [ground/output]
    sub: [centerpoint/pointcloud]
"#;
        let m = parse_manifest_str(yaml).unwrap();
        let g = DataflowGraph::build(&m);

        assert_eq!(g.graph.node_count(), 3);
        assert_eq!(g.graph.edge_count(), 2); // cropbox→ground, ground→centerpoint
    }

    #[test]
    fn test_build_with_includes() {
        let yaml = r#"
version: 1
includes:
  lidar:
    manifest: tier4/lidar.launch.yaml
nodes:
  fusion:
    sub: [lidar_objects]
    pub: [fused]
topics:
  lidar_objects:
    type: DetectedObjects
    pub: [lidar/output]
    sub: [fusion/lidar_objects]
"#;
        let m = parse_manifest_str(yaml).unwrap();
        let g = DataflowGraph::build(&m);

        assert_eq!(g.graph.node_count(), 2); // lidar scope + fusion node
        assert_eq!(g.graph.edge_count(), 1); // lidar→fusion
    }
}
