//! Web UI type definitions.
//!
//! Contains all data types used by the web UI for displaying node state,
//! statistics, and status information.
//!
//! Types derive `ts_rs::TS` to auto-generate TypeScript declarations that
//! mirror the exact serde JSON shapes. Run `cargo test` to regenerate
//! the `.d.ts` files in `bindings/`.

use crate::member_actor::{MemberState as ActorMemberState, MemberSummary as ActorMemberSummary};
use serde::{Deserialize, Serialize};
#[cfg(test)]
use ts_rs::TS;

/// Node status for regular nodes and containers (process-based)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
#[serde(rename_all = "lowercase")]
pub enum NodeStatus {
    /// Node is currently running
    Running,
    /// Node has stopped (exited normally)
    Stopped,
    /// Node has failed (exited with error)
    Failed,
    /// Node is pending start
    Pending,
}

/// Reason why a composable node is blocked
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
#[serde(rename_all = "snake_case")]
#[allow(clippy::enum_variant_names)]
pub enum ComposableBlockReason {
    /// Container was stopped by user
    ContainerStopped,
    /// Container crashed or failed
    ContainerFailed,
    /// Container hasn't started yet
    ContainerNotStarted,
    /// Shutdown signal received
    Shutdown,
}

/// Status for composable nodes (loaded into containers)
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
#[serde(rename_all = "lowercase", tag = "status", content = "reason")]
pub enum ComposableNodeStatus {
    /// Successfully loaded into container
    Loaded,
    /// Currently being loaded into container
    Loading,
    /// Currently being unloaded from container
    Unloading,
    /// Load failed
    #[allow(dead_code)]
    Failed,
    /// Not yet loaded (initial state)
    #[allow(dead_code)]
    Pending,
    /// Waiting for container to be ready (not yet attempted to load)
    Unloaded,
    /// Cannot be loaded because container is unavailable
    #[serde(rename = "blocked")]
    Blocked(ComposableBlockReason),
}

/// Unified status for all node types
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
#[serde(tag = "type", content = "value")]
pub enum UnifiedStatus {
    /// Process-based status (regular nodes and containers)
    Process(NodeStatus),
    /// Composable node status
    Composable(ComposableNodeStatus),
}

/// Type of node in the registry
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
#[serde(rename_all = "snake_case")]
pub enum NodeType {
    /// Regular ROS node
    Node,
    /// Node container (for composable nodes)
    Container,
    /// Composable node loaded into a container
    ComposableNode,
}

/// Summary information about a node (for web UI listings)
#[derive(Debug, Clone, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
pub struct NodeSummary {
    pub name: String,
    pub node_type: NodeType,
    pub status: UnifiedStatus,
    pub pid: Option<u32>,
    pub package: Option<String>,
    pub executable: String,
    pub namespace: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[cfg_attr(test, ts(optional = nullable))]
    pub target_container: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[cfg_attr(test, ts(optional = nullable))]
    pub container_name: Option<String>,
    pub is_container: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[cfg_attr(test, ts(optional = nullable))]
    pub exec_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[cfg_attr(test, ts(optional = nullable))]
    pub node_name: Option<String>,
    /// Unix timestamp (seconds) when stderr was last modified
    #[serde(skip_serializing_if = "Option::is_none")]
    #[cfg_attr(test, ts(optional = nullable, type = "number"))]
    pub stderr_last_modified: Option<u64>,
    /// Size of stderr file in bytes
    #[cfg_attr(test, ts(type = "number"))]
    pub stderr_size: u64,
    /// Last few lines of stderr for quick preview
    #[serde(skip_serializing_if = "Option::is_none")]
    #[cfg_attr(test, ts(optional = nullable))]
    pub stderr_preview: Option<Vec<String>>,
    /// Whether respawn is enabled for this node
    #[serde(skip_serializing_if = "Option::is_none")]
    #[cfg_attr(test, ts(optional = nullable))]
    pub respawn_enabled: Option<bool>,
    /// Respawn delay in seconds
    #[serde(skip_serializing_if = "Option::is_none")]
    #[cfg_attr(test, ts(optional = nullable))]
    pub respawn_delay: Option<f64>,
    /// Auto-load when container starts (for composable nodes)
    #[serde(skip_serializing_if = "Option::is_none")]
    #[cfg_attr(test, ts(optional = nullable))]
    pub auto_load: Option<bool>,
}

impl NodeSummary {
    /// Convert from MemberSummary to NodeSummary (web UI)
    pub fn from_member_summary(member: &ActorMemberSummary) -> Self {
        use crate::member_actor::MemberType as ActorMemberType;

        // Convert member type
        let node_type = match member.member_type {
            ActorMemberType::Node => NodeType::Node,
            ActorMemberType::Container => NodeType::Container,
            ActorMemberType::ComposableNode => NodeType::ComposableNode,
        };

        // Convert state to UnifiedStatus
        let status = Self::convert_state(&member.state);

        Self {
            name: member.name.clone(),
            node_type,
            status,
            pid: member.pid,
            package: member.package.clone(),
            executable: member.executable.clone(),
            namespace: member.namespace.clone(),
            target_container: member.target_container.clone(),
            // Container name will be set dynamically in the handler for composable nodes
            container_name: None,
            is_container: member.is_container,
            exec_name: member.exec_name.clone(),
            node_name: member.node_name.clone(),
            stderr_last_modified: member.stderr_last_modified,
            stderr_size: member.stderr_size,
            stderr_preview: member.stderr_preview.clone(),
            respawn_enabled: member.respawn_enabled,
            respawn_delay: member.respawn_delay,
            auto_load: member.auto_load,
        }
    }

    fn convert_state(state: &ActorMemberState) -> UnifiedStatus {
        use crate::member_actor::web_query::BlockReason as ActorBlockReason;

        match state {
            ActorMemberState::Pending => UnifiedStatus::Process(NodeStatus::Pending),
            ActorMemberState::Running { .. } => UnifiedStatus::Process(NodeStatus::Running),
            ActorMemberState::Respawning { .. } => UnifiedStatus::Process(NodeStatus::Running),
            ActorMemberState::Stopped => UnifiedStatus::Process(NodeStatus::Stopped),
            ActorMemberState::Failed { .. } => UnifiedStatus::Process(NodeStatus::Failed),
            ActorMemberState::Unloaded => UnifiedStatus::Composable(ComposableNodeStatus::Unloaded),
            ActorMemberState::Loading => UnifiedStatus::Composable(ComposableNodeStatus::Loading),
            ActorMemberState::Unloading => {
                UnifiedStatus::Composable(ComposableNodeStatus::Unloading)
            }
            ActorMemberState::Loaded { .. } => {
                UnifiedStatus::Composable(ComposableNodeStatus::Loaded)
            }
            ActorMemberState::Blocked { reason } => {
                let web_reason = match reason {
                    ActorBlockReason::ContainerStopped => ComposableBlockReason::ContainerStopped,
                    ActorBlockReason::ContainerFailed => ComposableBlockReason::ContainerFailed,
                    ActorBlockReason::ContainerNotStarted => {
                        ComposableBlockReason::ContainerNotStarted
                    }
                    ActorBlockReason::Shutdown => ComposableBlockReason::Shutdown,
                };
                UnifiedStatus::Composable(ComposableNodeStatus::Blocked(web_reason))
            }
        }
    }
}

/// Detailed information about a node (for API responses)
#[derive(Debug, Clone, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
#[allow(dead_code)]
pub struct NodeDetails {
    #[serde(flatten)]
    pub summary: NodeSummary,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[cfg_attr(test, ts(optional = nullable))]
    pub cmdline: Option<String>,
}

/// Boolean path parameter for toggle endpoints (e.g. `/respawn/true`).
///
/// Replaces manual `match enabled_str { "true" => ..., "false" => ... }` parsing.
/// Axum deserializes the path segment automatically; ts-rs generates `"true" | "false"`.
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
#[serde(rename_all = "lowercase")]
pub enum BoolParam {
    True,
    False,
}

impl BoolParam {
    pub fn as_bool(self) -> bool {
        matches!(self, BoolParam::True)
    }
}
