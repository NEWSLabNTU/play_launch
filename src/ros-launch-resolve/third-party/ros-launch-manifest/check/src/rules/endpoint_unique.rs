//! Rule: endpoint names unique per node across pub/sub/srv/cli.

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::Manifest;
use std::collections::HashSet;

pub struct EndpointUniqueRule;

impl ValidationRule for EndpointUniqueRule {
    fn id(&self) -> &str {
        "endpoint-unique"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        for (node_name, node) in &manifest.nodes {
            let mut seen = HashSet::new();
            for name in node
                .publishers
                .keys()
                .chain(node.subscribers.keys())
                .chain(node.srv.keys())
                .chain(node.cli.keys())
            {
                if !seen.insert(name.as_str()) {
                    ctx.error(
                        self.id(),
                        &format!("nodes.{node_name}"),
                        format!("duplicate endpoint name '{name}' across pub/sub/srv/cli"),
                    );
                }
            }
        }
    }
}
