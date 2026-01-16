//! Event types for actor communication
//!
//! This module defines events for:
//! - Control events: Commands sent TO actors
//! - State events: Status updates sent FROM actors

use serde::Serialize;

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
    /// Send specific signal to process (Unix only)
    #[cfg(unix)]
    Kill(nix::sys::signal::Signal),
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
    /// Notify actor that it was discovered as loaded via ListNodes
    DiscoveredLoaded {
        /// Unique ID from ListNodes response
        unique_id: u64,
    },
}

/// State events emitted by actors
///
/// These events are sent to the coordinator for:
/// - Web UI state updates
/// - Logging/metrics
/// - Coordination between actors
#[derive(Debug, Clone, Serialize)]
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
    /// Composable node blocked (container unavailable)
    Blocked {
        /// Composable node name
        name: String,
        /// Reason for blocking
        reason: super::state::BlockReason,
    },
    /// Node discovered via ListNodes query
    NodeDiscovered {
        /// Container name
        container_name: String,
        /// Full ROS node name (with namespace)
        full_node_name: String,
        /// Unique ID from ListNodes response
        unique_id: u64,
    },
    /// ListNodes query requested for a container
    ListNodesRequested {
        /// Container name to query
        container_name: String,
        /// Member name that requested the query
        requester: String,
    },
}

impl StateEvent {
    /// Get the member name from this event
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
            | StateEvent::Blocked { name, .. } => name,
            StateEvent::NodeDiscovered { container_name, .. } => container_name,
            StateEvent::ListNodesRequested { requester, .. } => requester,
        }
    }

    /// Check if this is a terminal event (actor won't send more events)
    pub fn is_terminal(&self) -> bool {
        matches!(self, StateEvent::Terminated { .. })
    }
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
        assert!(StateEvent::Terminated {
            name: "test".to_string()
        }
        .is_terminal());
        assert!(!StateEvent::Started {
            name: "test".to_string(),
            pid: 123
        }
        .is_terminal());
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
