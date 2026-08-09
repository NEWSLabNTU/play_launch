//! Rule: service type consistency between server and client declarations.
//!
//! For each `services:` entry that has both `server:` and `client:` lists,
//! checks that the server and client node endpoints don't declare conflicting
//! service types. (Currently, `SrvEndpointProps` doesn't carry a type field —
//! only the scope-level `services:` entry does. This rule validates the
//! scope-level declaration is present and non-empty.)

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::Manifest;

pub struct ServiceTypeRule;

impl ValidationRule for ServiceTypeRule {
    fn id(&self) -> &str {
        "service-type"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        for (svc_name, svc) in &manifest.services {
            // Service must have a type
            if svc.srv_type.is_empty() {
                ctx.error(
                    self.id(),
                    &format!("services.{svc_name}"),
                    format!("service '{svc_name}' has no type declared"),
                );
                continue;
            }

            // Verify server endpoints exist as srv: on their nodes
            for server_ref in &svc.server {
                if let Some((node_name, ep_name)) = server_ref.split_once('/')
                    && let Some(node) = manifest.nodes.get(node_name)
                    && !node.srv.contains_key(ep_name)
                {
                    ctx.warning(
                        self.id(),
                        &format!("services.{svc_name}"),
                        format!(
                            "server '{server_ref}' not found as srv: endpoint on node '{node_name}'"
                        ),
                    );
                }
            }

            // Verify client endpoints exist as cli: on their nodes
            for client_ref in &svc.client {
                if let Some((node_name, ep_name)) = client_ref.split_once('/')
                    && let Some(node) = manifest.nodes.get(node_name)
                    && !node.cli.contains_key(ep_name)
                {
                    ctx.warning(
                        self.id(),
                        &format!("services.{svc_name}"),
                        format!(
                            "client '{client_ref}' not found as cli: endpoint on node '{node_name}'"
                        ),
                    );
                }
            }
        }
    }
}
