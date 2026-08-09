//! Rule: cross-entity consistency checks (placeholder for phase 34.5).

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::Manifest;

pub struct ConsistencyRule;

impl ValidationRule for ConsistencyRule {
    fn id(&self) -> &str {
        "consistency"
    }

    fn check(&self, _manifest: &Manifest, _graph: &DataflowGraph, _ctx: &mut CheckContext) {
        // Placeholder — cross-entity consistency checks will be added in phase 34.5.
    }
}
