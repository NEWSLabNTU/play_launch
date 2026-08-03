//! Shared utilities for command handlers

use crate::process::kill_all_descendants;
use std::sync::Arc;
use tracing::debug;

// `build_tokio_runtime` and `parse_to_launch_dump` used to live here as
// byte-identical copies of `ros_launch_resolve::verbs`'. The verbs moved into
// the shared library so the two CLIs could not drift; copying these two back
// up re-created exactly the drift that move was meant to end (and CLAUDE.md
// now states the rule: shared logic goes in the library, never in a CLI).
// They are re-exported here so the call sites read unchanged.
pub(crate) use ros_launch_resolve::verbs::build_tokio_runtime;

/// Guard that ensures child processes are cleaned up on drop.
///
/// Used by replay and run commands to guarantee process cleanup even on panic.
pub(crate) struct CleanupGuard {
    enabled: Arc<std::sync::atomic::AtomicBool>,
}

impl CleanupGuard {
    pub(crate) fn new() -> Self {
        Self {
            enabled: Arc::new(std::sync::atomic::AtomicBool::new(true)),
        }
    }

    /// Disable the cleanup guard (call after graceful shutdown completes)
    pub(crate) fn disable(&self) {
        self.enabled
            .store(false, std::sync::atomic::Ordering::Relaxed);
        debug!("CleanupGuard disabled - graceful shutdown completed");
    }
}

impl Drop for CleanupGuard {
    fn drop(&mut self) {
        if self.enabled.load(std::sync::atomic::Ordering::Relaxed) {
            debug!("CleanupGuard: Ensuring all child processes are terminated");
            kill_all_descendants();
        } else {
            debug!("CleanupGuard: Skipped (disabled after graceful shutdown)");
        }
    }
}

/// Forward state events from runner to SSE broadcaster, then wait for all actors to complete.
///
/// Combines event forwarding with completion waiting in a single task.
/// Takes ownership of the runner (no Arc/Mutex needed).
///
/// Phase 24: Also updates the node FQN map when LoadSucceeded events arrive,
/// so the parameter proxy can construct service names for composable nodes.
pub(crate) async fn forward_state_events_and_wait(
    mut runner: crate::member_actor::MemberRunner,
    broadcaster: Arc<crate::web::StateEventBroadcaster>,
    node_fqn_map: Arc<tokio::sync::RwLock<std::collections::HashMap<String, String>>>,
) -> eyre::Result<()> {
    debug!("Starting state event forwarding and completion waiting");

    // Forward events until done
    while let Some(event) = runner.next_state_event().await {
        // Phase 24: Update FQN map for composable nodes when they load
        if let crate::member_actor::events::StateEvent::LoadSucceeded {
            ref name,
            ref full_node_name,
            ..
        } = event
        {
            let mut fqn_map = node_fqn_map.write().await;
            fqn_map.insert(name.clone(), full_node_name.clone());
            debug!("Updated FQN map: {} -> {}", name, full_node_name);
        }
        broadcaster.broadcast(event);
    }

    // Join all actor tasks
    runner.wait_for_completion().await
}
