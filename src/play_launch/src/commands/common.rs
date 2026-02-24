//! Shared utilities for command handlers

use crate::process::kill_all_descendants;
use std::sync::Arc;
use tracing::debug;

/// Build a Tokio multi-thread runtime with adaptive thread pool configuration.
///
/// Uses number of CPUs capped at 8 for efficiency (async workload, mostly idle).
/// Automatically adapts to platform: 2-core Pi, 4-core laptop, 8-core AGX Orin, 32-core server.
pub(crate) fn build_tokio_runtime() -> eyre::Result<tokio::runtime::Runtime> {
    let worker_threads = std::cmp::min(num_cpus::get(), 8);
    let max_blocking = worker_threads * 2;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(worker_threads)
        .max_blocking_threads(max_blocking)
        .thread_name("play_launch-worker")
        .enable_all()
        .build()?;
    debug!(
        "Tokio runtime created ({} worker threads, {} max blocking threads)",
        worker_threads, max_blocking
    );
    Ok(runtime)
}

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
pub(crate) async fn forward_state_events_and_wait(
    mut runner: crate::member_actor::MemberRunner,
    broadcaster: Arc<crate::web::StateEventBroadcaster>,
) -> eyre::Result<()> {
    debug!("Starting state event forwarding and completion waiting");

    // Forward events until done
    while let Some(event) = runner.next_state_event().await {
        broadcaster.broadcast(event).await;
    }

    // Join all actor tasks
    runner.wait_for_completion().await
}
