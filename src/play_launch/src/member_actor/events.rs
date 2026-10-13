//! Event types for actor communication
//!
//! This module defines events for:
//! - Control events: Commands sent TO actors
//! - State events: Status updates sent FROM actors

use crate::ros::parameter_types::ParamUpdate;
use serde::Serialize;
#[cfg(test)]
use ts_rs::TS;

/// Control commands sent to an actor
#[derive(Debug, Clone)]
pub enum ControlEvent {
    /// Request to start the process (if stopped)
    Start,
    /// Request to stop the process gracefully
    Stop,
    /// Request to restart the process (kill + respawn)
    Restart,
    /// Toggle respawn on/off
    ToggleRespawn(bool),
    /// Toggle auto-load on/off (composable nodes only)
    ToggleAutoLoad(bool),
    /// Load a composable node (container actors only)
    LoadComposable {
        /// Name of the composable node
        name: String,
    },
    /// Unload a composable node by name (container actors only - Phase 12)
    UnloadComposable {
        /// Name of the composable node
        name: String,
    },
    /// Load all composable nodes with auto_load=true (container actors only - Phase 12)
    LoadAllComposables,
    /// Unload all loaded composable nodes (container actors only - Phase 12)
    UnloadAllComposables,
    /// Toggle auto-load for a specific composable node (container actors only - Phase 12)
    ToggleComposableAutoLoad {
        /// Name of the composable node
        name: String,
        /// New auto-load value
        enabled: bool,
    },
    /// Retry loading a composable node (composable node actors only)
    /// Transitions the node back to Unloaded state to trigger re-loading
    Load,
    /// Unload a composable node (composable node actors only)
    Unload,
}

/// State events emitted by actors
///
/// These events are sent to the coordinator for:
/// - Web UI state updates
/// - Logging/metrics
/// - Coordination between actors
#[derive(Debug, Clone, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum StateEvent {
    /// Process started successfully
    Started {
        /// Member name
        name: String,
        /// Process ID
        pid: u32,
    },
    /// Process exited (may respawn if configured)
    Exited {
        /// Member name
        name: String,
        /// Exit code (None if killed)
        exit_code: Option<i32>,
        /// The signal that ended it, when one did (SIGSEGV = 11, ...).
        #[serde(default)]
        signal: Option<i32>,
        /// When it exited, Unix ms.
        #[serde(default)]
        at_ms: u64,
        /// How long it had been running, when known.
        #[serde(default)]
        uptime_ms: Option<u64>,
        /// Whether the kernel's OOM killer ended it: from the member's cgroup
        /// `memory.events` `oom_kill` counter, read at spawn and at exit.
        /// `None` when the member has no cgroup to ask.
        #[serde(default)]
        oom_killed: Option<bool>,
    },
    /// Process is respawning after exit
    Respawning {
        /// Member name
        name: String,
        /// Current attempt number
        attempt: u32,
        /// Delay before spawn (seconds)
        delay: f64,
    },
    /// Actor terminated (final state reached)
    Terminated {
        /// Member name
        name: String,
    },
    /// Actor failed permanently
    Failed {
        /// Member name
        name: String,
        /// Error description
        error: String,
    },
    /// Composable node loading started
    LoadStarted {
        /// Composable node name
        name: String,
    },
    /// Composable node loaded successfully
    LoadSucceeded {
        /// Composable node name
        name: String,
        /// Full ROS node name
        full_node_name: String,
        /// Unique ID from LoadNode response
        #[cfg_attr(test, ts(type = "number"))]
        unique_id: u64,
    },
    /// Composable node load failed
    LoadFailed {
        /// Composable node name
        name: String,
        /// Error message
        error: String,
    },
    /// Composable node unloaded successfully
    Unloaded {
        /// Composable node name
        name: String,
    },
    /// A crashed composable node will be reloaded into its container after
    /// `delay` (`composable_respawn`). Its state stays `Failed` until the
    /// reload starts; this event only carries the restart count.
    ComposableRespawning {
        /// Composable node name
        name: String,
        /// Reloads scheduled for this composable so far, this one included
        restart_count: u32,
        /// Delay before the reload (seconds)
        delay: f64,
    },
    /// Composable node blocked (container unavailable)
    Blocked {
        /// Composable node name
        name: String,
        /// Reason for blocking
        reason: super::model::BlockReason,
    },
    /// Parameter(s) changed on a managed node (Phase 24)
    ParameterChanged {
        /// Member name (not FQN)
        name: String,
        /// Changed parameters
        parameters: Vec<ParamUpdate>,
    },
}

/// Send a state event, logging instead of silently dropping on failure
/// (phase-51.3: the reducer derives the web mirror from this stream, so a
/// dropped event is a lost web transition — a real defect, not noise).
/// `send().await` on the bounded channel only fails when the receiver is
/// gone (shutdown teardown), so this logs at debug during normal exit
/// paths would be noise — error is right: it should never happen mid-run.
pub async fn emit(tx: &tokio::sync::mpsc::Sender<StateEvent>, event: StateEvent) {
    if let Err(e) = tx.send(event).await {
        tracing::error!(
            "StateEvent for '{}' dropped — receiver gone: {:?}",
            e.0.member_name(),
            e.0
        );
    }
}

impl StateEvent {
    /// Get the member name from this event (the reducer's map key)
    pub fn member_name(&self) -> &str {
        match self {
            StateEvent::Started { name, .. }
            | StateEvent::Exited { name, .. }
            | StateEvent::Respawning { name, .. }
            | StateEvent::Terminated { name }
            | StateEvent::Failed { name, .. }
            | StateEvent::LoadStarted { name }
            | StateEvent::LoadSucceeded { name, .. }
            | StateEvent::LoadFailed { name, .. }
            | StateEvent::Unloaded { name }
            | StateEvent::ComposableRespawning { name, .. }
            | StateEvent::Blocked { name, .. }
            | StateEvent::ParameterChanged { name, .. } => name,
        }
    }

    /// Check if this is a terminal event (actor won't send more events)
    #[allow(dead_code)] // Used in tests, useful for event routing
    pub fn is_terminal(&self) -> bool {
        matches!(self, StateEvent::Terminated { .. })
    }
}

/// The facts recorded with an exit: when, after how long, and whether the
/// kernel's OOM killer did it. `oom_baseline` is the member cgroup's
/// `oom_kill` counter read at spawn; a higher count now means this exit was an
/// OOM kill — in a container's group, possibly of one of its composables, which
/// with `memory.oom.group=1` takes the container with it.
pub(crate) fn exit_facts(
    spawned_at: Option<std::time::Instant>,
    oom_baseline: Option<u64>,
    cgroup: Option<&crate::execution::cgroup::CgroupHandle>,
) -> (u64, Option<u64>, Option<bool>) {
    let at_ms = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_millis() as u64);
    let uptime_ms = spawned_at.map(|t| t.elapsed().as_millis() as u64);
    let oom_killed = match (oom_baseline, cgroup.and_then(|c| c.oom_kills())) {
        (Some(before), Some(now)) => Some(now > before),
        _ => None,
    };
    (at_ms, uptime_ms, oom_killed)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_state_event_member_name() {
        let event = StateEvent::Started {
            name: "test_node".to_string(),
            pid: 123,
        };
        assert_eq!(event.member_name(), "test_node");

        let event = StateEvent::Failed {
            name: "test_node".to_string(),
            error: "test error".to_string(),
        };
        assert_eq!(event.member_name(), "test_node");
    }

    #[test]
    fn test_state_event_terminal() {
        assert!(
            StateEvent::Terminated {
                name: "test".to_string()
            }
            .is_terminal()
        );
        assert!(
            !StateEvent::Started {
                name: "test".to_string(),
                pid: 123
            }
            .is_terminal()
        );
    }

    #[test]
    fn test_state_event_serialization() {
        let event = StateEvent::Started {
            name: "test_node".to_string(),
            pid: 123,
        };
        let json = serde_json::to_string(&event).unwrap();
        assert!(json.contains("\"type\":\"started\""));
        assert!(json.contains("\"name\":\"test_node\""));
        assert!(json.contains("\"pid\":123"));
    }
}
