//! Rule: detect structurally invalid entities after condition filtering.
//!
//! - Topic with 0 publishers: warning (no data source — may be wired by parent)
//! - Topic with 0 subscribers: warning (unused — may be an export)
//! - Service with 0 servers: error (calls will fail)
//! - Action with 0 servers: error (goals can't be processed)

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::Manifest;

pub struct DanglingEntityRule;

impl ValidationRule for DanglingEntityRule {
    fn id(&self) -> &str {
        "dangling-entity"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        // Topics
        for (name, topic) in &manifest.topics {
            if topic.publishers.is_empty() && !topic.subscribers.is_empty() {
                ctx.warning(
                    self.id(),
                    &format!("topics.{name}"),
                    format!("topic '{name}' has no publishers (no data source)"),
                );
            }
            if !topic.publishers.is_empty() && topic.subscribers.is_empty() {
                ctx.warning(
                    self.id(),
                    &format!("topics.{name}"),
                    format!("topic '{name}' has no subscribers (output unused)"),
                );
            }
        }

        // Services
        for (name, svc) in &manifest.services {
            if svc.server.is_empty() && !svc.client.is_empty() {
                ctx.error(
                    self.id(),
                    &format!("services.{name}"),
                    format!("service '{name}' has no server (calls will fail)"),
                );
            }
        }

        // Actions
        for (name, act) in &manifest.actions {
            if act.server.is_empty() && !act.client.is_empty() {
                ctx.error(
                    self.id(),
                    &format!("actions.{name}"),
                    format!("action '{name}' has no server (goals can't be processed)"),
                );
            }
        }
    }
}
