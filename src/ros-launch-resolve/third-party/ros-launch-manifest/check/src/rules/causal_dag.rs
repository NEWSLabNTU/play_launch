//! Rule: the causal path graph (excluding state edges) must be acyclic.

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use petgraph::algo::is_cyclic_directed;
use ros_launch_manifest_types::Manifest;

pub struct CausalDagRule;

impl ValidationRule for CausalDagRule {
    fn id(&self) -> &str {
        "causal-dag"
    }

    fn check(&self, _manifest: &Manifest, graph: &DataflowGraph, ctx: &mut CheckContext) {
        // The dataflow graph built from topic wiring should be acyclic
        // when state endpoints are excluded. Since the graph builder only
        // includes topic wiring (not state reads), we can check directly.
        if is_cyclic_directed(&graph.graph) {
            ctx.error(
                self.id(),
                "graph",
                "causal dataflow graph contains a cycle. Use 'state: true' on feedback endpoints to break cycles.".to_string(),
            );
        }
    }
}
