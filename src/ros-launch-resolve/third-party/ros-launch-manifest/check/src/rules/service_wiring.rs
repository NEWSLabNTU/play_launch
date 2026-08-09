//! Rule: every service client (`cli:`) must have a matching server declaration.
//!
//! For each `cli:` endpoint on a node, checks that there is a `services:` entry
//! at scope level whose `server:` list is non-empty. This catches cases where a
//! node calls a service that no one serves.

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::Manifest;

pub struct ServiceWiringRule;

impl ValidationRule for ServiceWiringRule {
    fn id(&self) -> &str {
        "service-wiring"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        // Collect all service names that have servers declared
        let served: std::collections::HashSet<&str> = manifest
            .services
            .iter()
            .filter(|(_, svc)| !svc.server.is_empty())
            .map(|(name, _)| name.as_str())
            .collect();

        // Check each node's cli: endpoints
        for (node_name, node) in &manifest.nodes {
            for cli_name in node.cli.keys() {
                // Look for a services: entry that lists this node/endpoint as a client,
                // or a services: entry with the same name as the cli endpoint
                let full_ref = format!("{node_name}/{cli_name}");
                let has_server = manifest
                    .services
                    .values()
                    .any(|svc| svc.client.contains(&full_ref) && !svc.server.is_empty())
                    || served.contains(cli_name.as_str());

                if !has_server {
                    ctx.warning(
                        self.id(),
                        &format!("nodes.{node_name}.cli.{cli_name}"),
                        format!("service client '{cli_name}' has no matching server in services:"),
                    );
                }
            }
        }
    }
}
