//! Web UI type definitions.
//!
//! Contains all data types used by the web UI for displaying node state,
//! statistics, and status information.

use serde::Serialize;
use std::path::PathBuf;

// Re-export NodeLogPaths from member module
pub use crate::member::NodeLogPaths;

/// Node status for regular nodes and containers (process-based)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
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
#[serde(rename_all = "snake_case")]
#[allow(clippy::enum_variant_names)]
pub enum ComposableBlockReason {
    /// Container was stopped by user
    ContainerStopped,
    /// Container crashed or failed
    ContainerFailed,
    /// Container hasn't started yet
    ContainerNotStarted,
}

/// Status for composable nodes (loaded into containers)
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase", tag = "status", content = "reason")]
pub enum ComposableNodeStatus {
    /// Successfully loaded into container
    Loaded,
    /// Currently being loaded into container
    Loading,
    /// Load failed
    #[allow(dead_code)]
    Failed,
    /// Not yet loaded (initial state)
    Pending,
    /// Cannot be loaded because container is unavailable
    #[serde(rename = "blocked")]
    Blocked(ComposableBlockReason),
}

/// Unified status for all node types
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[serde(tag = "type", content = "value")]
pub enum UnifiedStatus {
    /// Process-based status (regular nodes and containers)
    Process(NodeStatus),
    /// Composable node status
    Composable(ComposableNodeStatus),
}

/// Type of node in the registry
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
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
pub struct NodeSummary {
    pub name: String,
    pub node_type: NodeType,
    pub status: UnifiedStatus,
    pub pid: Option<u32>,
    pub package: Option<String>,
    pub executable: String,
    pub namespace: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_container: Option<String>,
    pub is_container: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exec_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub node_name: Option<String>,
    /// Unix timestamp (seconds) when stderr was last modified
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stderr_last_modified: Option<u64>,
    /// Size of stderr file in bytes
    pub stderr_size: u64,
    /// Last few lines of stderr for quick preview
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stderr_preview: Option<Vec<String>>,
    /// Whether respawn is enabled for this node
    #[serde(skip_serializing_if = "Option::is_none")]
    pub respawn_enabled: Option<bool>,
    /// Respawn delay in seconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub respawn_delay: Option<f64>,
}

/// Detailed information about a node (for API responses)
#[derive(Debug, Clone, Serialize)]
#[allow(dead_code)]
pub struct NodeDetails {
    #[serde(flatten)]
    pub summary: NodeSummary,
    pub output_dir: PathBuf,
    pub log_paths: NodeLogPaths,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cmdline: Option<String>,
}

/// Health summary statistics for all nodes
#[derive(Debug, Clone, Serialize)]
pub struct HealthSummary {
    // Process-level counts
    pub processes_running: usize,
    pub processes_stopped: usize,

    // Regular node counts
    pub nodes_running: usize,
    pub nodes_stopped: usize,
    pub nodes_failed: usize,
    pub nodes_total: usize,

    // Container counts
    pub containers_running: usize,
    pub containers_stopped: usize,
    pub containers_failed: usize,
    pub containers_total: usize,

    // Composable node counts
    pub composable_loaded: usize,
    pub composable_failed: usize,
    pub composable_pending: usize,
    pub composable_total: usize,

    /// Number of nodes with significant stderr output (>10KB)
    pub noisy: usize,
}
