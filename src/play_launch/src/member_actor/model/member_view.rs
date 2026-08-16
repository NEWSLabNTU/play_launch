//! Web-facing derived view of member state.
//!
//! `MemberState` is a MIRROR of the actor-side state machines — it is
//! produced from them via the `From` impls below (used by the phase-51
//! state reducer), never hand-encoded at call sites.

use super::{
    composable_state::{BlockReason, ComposableState},
    node_state::NodeState,
};
use serde::Serialize;
use std::path::PathBuf;
#[cfg(test)]
use ts_rs::TS;

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
    Unloaded,
    Loading,
    Loaded {
        unique_id: u64,
    },
    Unloading,
    Blocked {
        reason: BlockReason,
    },
}

impl From<&NodeState> for MemberState {
    fn from(state: &NodeState) -> Self {
        match state {
            NodeState::Pending => MemberState::Pending,
            NodeState::Running { pid, .. } => MemberState::Running { pid: *pid },
            NodeState::Respawning { attempt, .. } => MemberState::Respawning { attempt: *attempt },
            NodeState::Stopped { .. } => MemberState::Stopped,
            NodeState::Failed { error } => MemberState::Failed {
                error: error.clone(),
            },
        }
    }
}

impl From<&ComposableState> for MemberState {
    fn from(state: &ComposableState) -> Self {
        match state {
            ComposableState::Blocked { reason } => MemberState::Blocked { reason: *reason },
            ComposableState::Unloaded => MemberState::Unloaded,
            ComposableState::Loading { .. } => MemberState::Loading,
            ComposableState::Unloading { .. } => MemberState::Unloading,
            ComposableState::Loaded { unique_id } => MemberState::Loaded {
                unique_id: *unique_id,
            },
            ComposableState::Failed { error } => MemberState::Failed {
                error: error.clone(),
            },
        }
    }
}

/// Summary information about a member (for web UI listings)
#[derive(Debug, Clone, Serialize)]
pub struct MemberSummary {
    /// Canonical collision-proof id (`kind:/ns/name[#N]`) — the key for
    /// every registry and REST path (phase-50, docs/design/member-identity.md)
    pub id: String,
    /// Display name (bare)
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
    /// Auto-load when container starts (for composable nodes)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auto_load: Option<bool>,
    /// Output directory for logs
    pub output_dir: PathBuf,
}

/// Health summary statistics for all members
#[derive(Debug, Clone, Default, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
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

impl HealthSummary {
    /// Whether every member has reached a settled state — running, stopped,
    /// failed, or (for a composable) loaded or failed.
    ///
    /// Phase 61 replaced `composable_pending == 0` as the completion test.
    /// That test was true at t=0 and stayed true until the first container
    /// came up, because a composable whose container has not started yet is
    /// `Blocked`, which counts as neither pending nor loaded nor failed. It
    /// only ever worked by a race: every container used to spawn within the
    /// first 100 ms, so the first completion check already saw pending loads.
    /// Once startup was paced, containers began queueing behind the
    /// concurrency limit, the race started losing every time, and play_launch
    /// announced "Startup complete: all nodes ready (nodes 12/44, containers
    /// 0/16, composable 0/84)" two tenths of a second into a three-minute
    /// startup — then stopped reporting progress, because the progress task
    /// exits on completion.
    pub fn startup_complete(&self) -> bool {
        let nodes_settled =
            self.nodes_running + self.nodes_stopped + self.nodes_failed >= self.nodes_total;
        let containers_settled =
            self.containers_running + self.containers_stopped + self.containers_failed
                >= self.containers_total;
        let composables_settled =
            self.composable_loaded + self.composable_failed >= self.composable_total;

        nodes_settled && containers_settled && composables_settled
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Regression guard for issue #0016. The old test was
    /// `composable_pending == 0`, which every one of these cases satisfies —
    /// including the two that are plainly not complete.
    #[test]
    fn startup_is_not_complete_before_anything_has_started() {
        // The exact shape that printed "Startup complete: all nodes ready
        // (nodes 12/44, containers 0/16, composable 0/84)": no container has
        // spawned, so every composable is Blocked and counts as neither
        // pending nor loaded nor failed.
        let just_started = HealthSummary {
            nodes_running: 12,
            nodes_total: 44,
            containers_total: 16,
            composable_total: 84,
            ..Default::default()
        };
        assert_eq!(just_started.composable_pending, 0);
        assert!(!just_started.startup_complete());

        // Containers up, composables still blocked or queued.
        let midway = HealthSummary {
            nodes_running: 44,
            nodes_total: 44,
            containers_running: 16,
            containers_total: 16,
            composable_loaded: 40,
            composable_total: 84,
            ..Default::default()
        };
        assert!(!midway.startup_complete());

        // Everything settled — a failure is settled too, or a launch with one
        // dead node would never report completion at all.
        let done = HealthSummary {
            nodes_running: 43,
            nodes_failed: 1,
            nodes_total: 44,
            containers_running: 16,
            containers_total: 16,
            composable_loaded: 84,
            composable_total: 84,
            ..Default::default()
        };
        assert!(done.startup_complete());
    }

    #[test]
    fn empty_launch_is_immediately_complete() {
        // Nothing to wait for. `>=` rather than `==` throughout keeps this
        // from depending on the zero case being special-cased.
        assert!(HealthSummary::default().startup_complete());
    }

    #[test]
    fn from_node_state() {
        assert_eq!(MemberState::from(&NodeState::Pending), MemberState::Pending);
        assert_eq!(
            MemberState::from(&NodeState::Stopped { exit_code: Some(0) }),
            MemberState::Stopped
        );
        assert_eq!(
            MemberState::from(&NodeState::Respawning {
                exit_code: None,
                attempt: 3
            }),
            MemberState::Respawning { attempt: 3 }
        );
    }

    #[test]
    fn from_composable_state() {
        assert_eq!(
            MemberState::from(&ComposableState::Loaded { unique_id: 7 }),
            MemberState::Loaded { unique_id: 7 }
        );
        assert_eq!(
            MemberState::from(&ComposableState::Blocked {
                reason: BlockReason::ContainerNotStarted
            }),
            MemberState::Blocked {
                reason: BlockReason::ContainerNotStarted
            }
        );
        assert_eq!(
            MemberState::from(&ComposableState::Loading {
                started_at: std::time::Instant::now()
            }),
            MemberState::Loading
        );
    }
}
