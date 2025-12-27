//! State types for member actors
//!
//! This module defines the state machines and configuration for member actors.

use serde::Serialize;
use std::path::PathBuf;
use tokio::process::Child;

/// Configuration for a member actor
#[derive(Debug, Clone)]
pub struct ActorConfig {
    /// Whether respawn is enabled
    pub respawn_enabled: bool,
    /// Delay before respawn (seconds)
    pub respawn_delay: f64,
    /// Maximum respawn attempts (None = infinite)
    pub max_respawn_attempts: Option<u32>,
    /// Output directory for logs
    pub output_dir: PathBuf,
    /// Process group ID for spawned processes
    pub pgid: Option<i32>,
}

/// State machine for a regular node or container
#[derive(Debug)]
pub enum NodeState {
    /// Waiting to start
    Pending,
    /// Process is running
    Running {
        /// The Child handle (consumed by wait())
        child: Child,
        /// Process ID
        pid: u32,
    },
    /// Waiting to respawn after exit
    Respawning {
        /// Exit code from previous run
        exit_code: Option<i32>,
        /// Current attempt number (0-indexed)
        attempt: u32,
    },
    /// Stopped cleanly (no respawn)
    Stopped {
        /// Final exit code
        exit_code: Option<i32>,
    },
    /// Failed permanently (max attempts reached or fatal error)
    Failed {
        /// Error description
        error: String,
    },
}

/// State machine for composable nodes
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum ComposableState {
    /// Not loaded (initial state)
    Unloaded,
    /// LoadNode service call in progress
    Loading,
    /// Successfully loaded via LoadNode service
    Loaded {
        /// Unique ID from LoadNode response
        unique_id: u64,
    },
    /// Cannot be loaded because container is unavailable
    Blocked {
        /// Reason for blocking
        reason: BlockReason,
    },
}

/// Reason why a composable node is blocked
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum BlockReason {
    /// Container was stopped by user
    Stopped,
    /// Container crashed
    Failed,
    /// Container hasn't started yet
    NotStarted,
}

/// Container state (for supervision)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContainerState {
    /// Container is running
    Running {
        /// Process ID
        pid: u32,
    },
    /// Container stopped cleanly
    Stopped,
    /// Container failed/crashed
    Failed,
    /// Container not yet started
    Pending,
}

impl NodeState {
    /// Check if the state is terminal (no further transitions)
    pub fn is_terminal(&self) -> bool {
        matches!(self, NodeState::Stopped { .. } | NodeState::Failed { .. })
    }

    /// Get the PID if running
    pub fn pid(&self) -> Option<u32> {
        match self {
            NodeState::Running { pid, .. } => Some(*pid),
            _ => None,
        }
    }
}

impl ComposableState {
    /// Check if the state is terminal
    pub fn is_terminal(&self) -> bool {
        matches!(
            self,
            ComposableState::Blocked {
                reason: BlockReason::Stopped | BlockReason::Failed
            }
        )
    }

    /// Check if loaded
    pub fn is_loaded(&self) -> bool {
        matches!(self, ComposableState::Loaded { .. })
    }
}

impl ContainerState {
    /// Check if container is ready for composable nodes
    pub fn is_ready(&self) -> bool {
        matches!(self, ContainerState::Running { .. })
    }

    /// Get the PID if running
    pub fn pid(&self) -> Option<u32> {
        match self {
            ContainerState::Running { pid } => Some(*pid),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_state_terminal() {
        assert!(NodeState::Stopped { exit_code: Some(0) }.is_terminal());
        assert!(NodeState::Failed {
            error: "test".to_string()
        }
        .is_terminal());
        assert!(!NodeState::Pending.is_terminal());
        assert!(!NodeState::Respawning {
            exit_code: Some(1),
            attempt: 0
        }
        .is_terminal());
    }

    #[test]
    fn test_composable_state_loaded() {
        assert!(ComposableState::Loaded { unique_id: 123 }.is_loaded());
        assert!(!ComposableState::Unloaded.is_loaded());
        assert!(!ComposableState::Loading.is_loaded());
    }

    #[test]
    fn test_container_state_ready() {
        assert!(ContainerState::Running { pid: 123 }.is_ready());
        assert!(!ContainerState::Pending.is_ready());
        assert!(!ContainerState::Stopped.is_ready());
        assert!(!ContainerState::Failed.is_ready());
    }
}
