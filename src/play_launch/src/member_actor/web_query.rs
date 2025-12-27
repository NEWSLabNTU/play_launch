//! Web UI query types for member actors
//!
//! These types provide a clean interface for the web UI to query actor state
//! without depending on the event_driven module.

use serde::Serialize;
use std::path::PathBuf;

/// Type of member in the actor system
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum MemberType {
    /// Regular ROS node
    Node,
    /// Node container (for composable nodes)
    Container,
    /// Composable node loaded into a container
    ComposableNode,
}

/// Unified state for all member types (for web UI)
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(tag = "type", content = "value")]
pub enum MemberState {
    /// Process-based states (regular nodes and containers)
    Pending,
    Running {
        pid: u32,
    },
    Respawning {
        attempt: u32,
    },
    Stopped,
    Failed {
        error: String,
    },

    /// Composable node states
    Loading,
    Loaded {
        unique_id: u64,
    },
    Blocked {
        reason: BlockReason,
    },
}

/// Reason why a composable node is blocked
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
#[allow(clippy::enum_variant_names)] // "Container" prefix is meaningful and clear
pub enum BlockReason {
    /// Container was stopped by user
    ContainerStopped,
    /// Container crashed or failed
    ContainerFailed,
    /// Container hasn't started yet
    ContainerNotStarted,
}

/// Summary information about a member (for web UI listings)
#[derive(Debug, Clone, Serialize)]
pub struct MemberSummary {
    pub name: String,
    pub member_type: MemberType,
    pub state: MemberState,
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
    /// Output directory for logs
    pub output_dir: PathBuf,
}

/// Detailed information about a member (for API responses)
#[derive(Debug, Clone, Serialize)]
pub struct MemberDetails {
    #[serde(flatten)]
    pub summary: MemberSummary,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cmdline: Option<String>,
}

/// Health summary statistics for all members
#[derive(Debug, Clone, Default, Serialize)]
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
