//! Member type redesign - distinct types for Regular Nodes, Containers, and Composable Nodes
//!
//! This module implements the new event-driven architecture with explicit state management.
//! Unlike the previous unified NodeHandle approach, each member type has its own struct
//! with type-specific fields.
//!
//! # Design Principles
//! - Separation of concerns: Each type has distinct fields
//! - Explicit state: State stored in memory, not computed from filesystem
//! - No Child handles: ProcessMonitor owns Child, members only store PID in ProcessState

use crate::{
    execution::node_cmdline::NodeCommandLine,
    ros::launch_dump::{ComposableNodeRecord, NodeRecord},
};
use serde::Serialize;
use std::path::{Path, PathBuf};

/// Paths to log files for a node
#[derive(Debug, Clone, Serialize)]
pub struct NodeLogPaths {
    /// Path to stdout log file
    pub stdout: PathBuf,
    /// Path to stderr log file
    pub stderr: PathBuf,
    /// Path to PID file
    pub pid_file: PathBuf,
    /// Path to command line file
    pub cmdline_file: PathBuf,
    /// Path to status file
    pub status_file: PathBuf,
    /// Path to metrics CSV (if monitoring enabled)
    pub metrics_file: PathBuf,
}

impl NodeLogPaths {
    /// Create log paths from an output directory
    pub fn from_output_dir(output_dir: &Path) -> Self {
        Self {
            stdout: output_dir.join("out"),
            stderr: output_dir.join("err"),
            pid_file: output_dir.join("pid"),
            cmdline_file: output_dir.join("cmdline"),
            status_file: output_dir.join("status"),
            metrics_file: output_dir.join("metrics.csv"),
        }
    }
}

/// State for processes (Regular Nodes and Containers)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum ProcessState {
    /// Process is running with the given PID
    Running { pid: u32 },
    /// Process was stopped intentionally (clean shutdown)
    Stopped,
    /// Process failed/crashed while running
    Failed { exit_code: Option<i32> },
    /// Process not yet started (initial state)
    Pending,
}

/// State for composable nodes
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub enum ComposableState {
    /// Successfully loaded via LoadNode service
    Loaded,
    /// LoadNode service call in progress
    Loading,
    /// Not loaded (initial state or load failed)
    Unloaded,
    /// Cannot be loaded because container is unavailable
    Blocked { reason: BlockReason },
}

/// Reason why a composable node is blocked
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum BlockReason {
    /// Container was stopped by user
    Stopped,
    /// Container crashed
    Failed,
    /// Container hasn't started yet (initial state)
    #[allow(dead_code)]
    NotStarted,
}

/// Regular ROS node (non-container, non-composable)
#[derive(Debug, Clone)]
pub struct RegularNode {
    /// Unique name for the node (directory name in logs)
    pub name: String,
    /// Current state (includes PID when running)
    pub state: ProcessState,
    /// Launch file record
    pub record: NodeRecord,
    /// Command line for execution
    pub cmdline: NodeCommandLine,
    /// Output directory for logs
    pub output_dir: PathBuf,
    /// Paths to log files
    pub log_paths: NodeLogPaths,
    /// Whether respawn is enabled
    pub respawn_enabled: bool,
    /// Delay before respawning (seconds)
    pub respawn_delay: f64,
}

/// Container node (can host composable nodes)
#[derive(Debug, Clone)]
pub struct Container {
    /// Unique name for the container (directory name in logs)
    pub name: String,
    /// Current state (includes PID when running)
    pub state: ProcessState,
    /// Launch file record
    pub record: NodeRecord,
    /// Command line for execution
    pub cmdline: NodeCommandLine,
    /// Output directory for logs
    pub output_dir: PathBuf,
    /// Paths to log files
    pub log_paths: NodeLogPaths,
    /// List of composable nodes loaded into this container
    #[allow(dead_code)]
    pub composable_nodes: Vec<String>,
    /// Whether respawn is enabled
    pub respawn_enabled: bool,
    /// Delay before respawning (seconds)
    pub respawn_delay: f64,
}

/// Composable node (loaded into a container via LoadNode service)
#[derive(Debug, Clone)]
pub struct ComposableNode {
    /// Unique name for the node (directory name in logs)
    pub name: String,
    /// Current state
    pub state: ComposableState,
    /// Name of the container this node belongs to
    pub container_name: String,
    /// Launch file record
    pub record: ComposableNodeRecord,
    /// Output directory for logs
    #[allow(dead_code)]
    pub output_dir: PathBuf,
    /// Paths to log files
    pub log_paths: NodeLogPaths,
}

/// Unified enum for storage in Registry
#[derive(Debug, Clone)]
pub enum Member {
    Node(RegularNode),
    Container(Container),
    ComposableNode(ComposableNode),
}

impl Member {
    /// Get the name of this member
    #[allow(dead_code)]
    pub fn name(&self) -> &str {
        match self {
            Member::Node(n) => &n.name,
            Member::Container(c) => &c.name,
            Member::ComposableNode(cn) => &cn.name,
        }
    }

    /// Get the output directory
    #[allow(dead_code)]
    pub fn output_dir(&self) -> &PathBuf {
        match self {
            Member::Node(n) => &n.output_dir,
            Member::Container(c) => &c.output_dir,
            Member::ComposableNode(cn) => &cn.output_dir,
        }
    }

    /// Check if this is a container
    pub fn is_container(&self) -> bool {
        matches!(self, Member::Container(_))
    }

    /// Check if this is a composable node
    #[allow(dead_code)]
    pub fn is_composable(&self) -> bool {
        matches!(self, Member::ComposableNode(_))
    }

    /// Check if this is a regular node
    #[allow(dead_code)]
    pub fn is_regular_node(&self) -> bool {
        matches!(self, Member::Node(_))
    }
}

impl ProcessState {
    /// Check if the process is currently running
    #[allow(dead_code)]
    pub fn is_running(&self) -> bool {
        matches!(self, ProcessState::Running { .. })
    }

    /// Get the PID if running
    pub fn pid(&self) -> Option<u32> {
        match self {
            ProcessState::Running { pid } => Some(*pid),
            _ => None,
        }
    }
}

impl ComposableState {
    /// Check if the node is loaded
    #[allow(dead_code)]
    pub fn is_loaded(&self) -> bool {
        matches!(self, ComposableState::Loaded)
    }

    /// Check if the node is blocked
    pub fn is_blocked(&self) -> bool {
        matches!(self, ComposableState::Blocked { .. })
    }

    /// Check if the node is loading
    #[allow(dead_code)]
    pub fn is_loading(&self) -> bool {
        matches!(self, ComposableState::Loading)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_process_state_is_running() {
        assert!(ProcessState::Running { pid: 1234 }.is_running());
        assert!(!ProcessState::Stopped.is_running());
        assert!(!ProcessState::Failed { exit_code: Some(1) }.is_running());
        assert!(!ProcessState::Pending.is_running());
    }

    #[test]
    fn test_process_state_pid() {
        assert_eq!(ProcessState::Running { pid: 1234 }.pid(), Some(1234));
        assert_eq!(ProcessState::Stopped.pid(), None);
        assert_eq!(ProcessState::Failed { exit_code: Some(1) }.pid(), None);
        assert_eq!(ProcessState::Pending.pid(), None);
    }

    #[test]
    fn test_composable_state_checks() {
        assert!(ComposableState::Loaded.is_loaded());
        assert!(!ComposableState::Loading.is_loaded());
        assert!(!ComposableState::Unloaded.is_loaded());

        assert!(ComposableState::Blocked {
            reason: BlockReason::Failed
        }
        .is_blocked());
        assert!(!ComposableState::Loaded.is_blocked());

        assert!(ComposableState::Loading.is_loading());
        assert!(!ComposableState::Loaded.is_loading());
    }

    #[test]
    fn test_member_type_checks() {
        let reg_node = Member::Node(RegularNode {
            name: "test".to_string(),
            state: ProcessState::Pending,
            record: NodeRecord {
                executable: "test_node".to_string(),
                package: Some("test_pkg".to_string()),
                name: Some("test".to_string()),
                namespace: Some("/".to_string()),
                exec_name: None,
                params: vec![],
                params_files: vec![],
                remaps: vec![],
                ros_args: None,
                args: None,
                cmd: vec![],
                env: None,
                respawn: None,
                respawn_delay: None,
                global_params: None,
            },
            cmdline: NodeCommandLine {
                command: vec!["test".to_string()],
                user_args: vec![],
                remaps: std::collections::HashMap::new(),
                params: std::collections::HashMap::new(),
                params_files: std::collections::HashSet::new(),
                log_level: None,
                log_config_file: None,
                rosout_logs: None,
                stdout_logs: None,
                enclave: None,
                env: std::collections::HashMap::new(),
            },
            output_dir: PathBuf::from("/tmp"),
            log_paths: NodeLogPaths {
                stdout: PathBuf::from("/tmp/out"),
                stderr: PathBuf::from("/tmp/err"),
                pid_file: PathBuf::from("/tmp/pid"),
                cmdline_file: PathBuf::from("/tmp/cmdline"),
                status_file: PathBuf::from("/tmp/status"),
                metrics_file: PathBuf::from("/tmp/metrics.csv"),
            },
            respawn_enabled: false,
            respawn_delay: 0.0,
        });

        assert!(reg_node.is_regular_node());
        assert!(!reg_node.is_container());
        assert!(!reg_node.is_composable());
        assert_eq!(reg_node.name(), "test");
    }
}
