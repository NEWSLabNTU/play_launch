//! Composable-node state machine and THE single [`BlockReason`].
//!
//! Phase-51 unification: the actor side used a 4-variant `state::BlockReason`
//! (`NotStarted`/`Stopped`/`Failed`/`Shutdown`, PascalCase on the SSE wire)
//! and the web side a 5-variant `web_query::BlockReason` (`Container*` +
//! `ContainerNotFound`, snake_case on REST), hand-mapped at every insert.
//! This is now the one definition, snake_case everywhere. The web UI only
//! branches on the status string, never the reason value, so the SSE wire
//! change (`"NotStarted"` → `"container_not_started"`) is display-only.

use serde::Serialize;
#[cfg(test)]
use ts_rs::TS;

/// Reason why a composable node is blocked
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
#[serde(rename_all = "snake_case")]
#[allow(clippy::enum_variant_names)] // "Container" prefix is meaningful and clear
pub enum BlockReason {
    /// No container matched the composable's target_container_name at
    /// registration (phase-50: registered visibly instead of dropped)
    ContainerNotFound,
    /// Container was stopped by user
    ContainerStopped,
    /// Container crashed or failed
    ContainerFailed,
    /// Container hasn't started yet
    ContainerNotStarted,
    /// Shutdown signal received
    Shutdown,
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

#[cfg(test)]
mod tests {
    use super::*;

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
    fn block_reason_serializes_snake_case() {
        let json = serde_json::to_string(&BlockReason::ContainerNotStarted).unwrap();
        assert_eq!(json, "\"container_not_started\"");
    }
}
