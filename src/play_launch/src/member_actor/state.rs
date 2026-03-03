//! State types for member actors
//!
//! This module defines the state machines and configuration for member actors.

use serde::Serialize;
use std::path::PathBuf;
use tokio::process::Child;
#[cfg(test)]
use ts_rs::TS;

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
        #[allow(dead_code)] // Semantic field — used for logging/debugging
        exit_code: Option<i32>,
    },
    /// Failed permanently (max attempts reached or fatal error)
    Failed {
        /// Error description
        #[allow(dead_code)] // Semantic field — used for logging/debugging
        error: String,
    },
}

/// State machine for composable nodes
///
/// State transitions:
/// - Blocked (container not running) → Unloaded (container starts)
/// - Unloaded (never loaded) → Loading (auto-trigger load)
/// - Loading (in-flight) → Loaded (success) | Failed (error) | Blocked (container crash)
/// - Loaded (success) → Blocked (container crash)
/// - Failed (error) → stays Failed (no auto-retry, wait for manual trigger or container restart)
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub enum ComposableState {
    /// Container NOT running (Pending/Stopped/Failed)
    Blocked {
        /// Reason for blocking
        reason: BlockReason,
    },
    /// Container running, load never attempted
    Unloaded,
    /// Container running, load request in-flight
    Loading {
        /// When the load started (for metrics)
        #[serde(skip)]
        started_at: std::time::Instant,
    },
    /// Container running, unload request in-flight
    Unloading {
        /// When the unload started (for metrics)
        #[serde(skip)]
        started_at: std::time::Instant,
    },
    /// Container running, last load succeeded
    Loaded {
        /// Unique ID from LoadNode response
        unique_id: u64,
    },
    /// Container running, last load failed (no auto-retry)
    Failed {
        /// Error message from load failure
        error: String,
    },
}

/// Reason why a composable node is blocked
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
pub enum BlockReason {
    /// Container hasn't started yet (Pending)
    NotStarted,
    /// Container was stopped by user
    Stopped,
    /// Container crashed or failed
    Failed,
    /// Shutdown signal received
    Shutdown,
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
    #[allow(dead_code)] // Used in tests, semantic API for state machines
    pub fn is_terminal(&self) -> bool {
        matches!(self, NodeState::Stopped { .. } | NodeState::Failed { .. })
    }
}

impl ComposableState {
    /// Check if the state is terminal (actor should stop)
    #[allow(dead_code)] // Used in tests, semantic API for state machines
    pub fn is_terminal(&self) -> bool {
        matches!(
            self,
            ComposableState::Blocked {
                reason: BlockReason::Shutdown
            }
        )
    }

    /// Check if loaded successfully
    #[allow(dead_code)] // Used in tests, semantic API for state machines
    pub fn is_loaded(&self) -> bool {
        matches!(self, ComposableState::Loaded { .. })
    }
}

impl ContainerState {
    /// Check if container is ready for composable nodes
    #[allow(dead_code)] // Used in tests, semantic API for state machines
    pub fn is_ready(&self) -> bool {
        matches!(self, ContainerState::Running { .. })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_state_terminal() {
        assert!(NodeState::Stopped { exit_code: Some(0) }.is_terminal());
        assert!(
            NodeState::Failed {
                error: "test".to_string()
            }
            .is_terminal()
        );
        assert!(!NodeState::Pending.is_terminal());
        assert!(
            !NodeState::Respawning {
                exit_code: Some(1),
                attempt: 0
            }
            .is_terminal()
        );
    }

    #[test]
    fn test_composable_state_loaded() {
        assert!(ComposableState::Loaded { unique_id: 123 }.is_loaded());
        assert!(!ComposableState::Unloaded.is_loaded());
        assert!(
            !ComposableState::Loading {
                started_at: std::time::Instant::now()
            }
            .is_loaded()
        );
    }

    #[test]
    fn test_container_state_ready() {
        assert!(ContainerState::Running { pid: 123 }.is_ready());
        assert!(!ContainerState::Pending.is_ready());
        assert!(!ContainerState::Stopped.is_ready());
        assert!(!ContainerState::Failed.is_ready());
    }
}
