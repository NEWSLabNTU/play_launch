//! Process-side state machines (regular nodes and containers) and actor
//! configuration. Moved from `member_actor/state.rs` in phase-51.

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
    /// Phase 38: resolved posix scheduling for THIS member (None = no tier / not applied).
    pub sched: Option<crate::execution::sched_apply::AppliedTier>,
    /// Phase 38: how to apply (Off short-circuits the actor hook).
    pub sched_mode: crate::execution::sched_apply::SchedApplyMode,
    /// Phase 38.10: handle to the RT helper for delegated (non-root)
    /// scheduling application. `None` means apply directly in-process via
    /// `apply_tier` (only works as root) — see
    /// `execution::rt_helper_client::apply_sched`.
    pub sched_helper: Option<crate::execution::rt_helper_client::SchedHelper>,
    /// Phase 61: shared startup admission control. Every actor asks this
    /// before spawning, so a wide launch does not start every process in the
    /// same millisecond. `StartupGovernor::disabled()` restores the previous
    /// spawn-immediately behaviour.
    pub startup: std::sync::Arc<crate::execution::startup_governor::StartupGovernor>,
    /// Phase 61 W2: which startup stage this member belongs to. Stage 0 starts
    /// immediately; a higher stage waits until every member of every lower one
    /// is up (or gone, or the stage timed out).
    pub startup_stage: crate::execution::startup_order::Stage,
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
    fn test_container_state_ready() {
        assert!(ContainerState::Running { pid: 123 }.is_ready());
        assert!(!ContainerState::Pending.is_ready());
        assert!(!ContainerState::Stopped.is_ready());
        assert!(!ContainerState::Failed.is_ready());
    }
}
