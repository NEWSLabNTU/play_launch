//! Signal handling and shutdown orchestration for replay/run commands.
//!
//! Handles SIGINT/SIGTERM with 3-stage escalation (graceful -> force -> kill),
//! signal debouncing, background task draining, and cleanup guard management.

use super::common::CleanupGuard;
use futures::stream::FuturesUnordered;
use tracing::{debug, error, info, warn};

/// Signal debounce window -- ignore signals within this time after sending our own kill
const SIGNAL_DEBOUNCE: std::time::Duration = std::time::Duration::from_millis(200);

/// Timeout for draining background tasks during cleanup
const CLEANUP_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(2);

/// Interval for printing startup progress statistics
const PROGRESS_INTERVAL: std::time::Duration = std::time::Duration::from_secs(10);

/// Interval for checking if all nodes have finished loading
const COMPLETION_CHECK_INTERVAL: std::time::Duration = std::time::Duration::from_millis(100);

/// Context for completion waiting -- shared between Unix and Windows paths.
pub(crate) struct CompletionContext {
    pub(crate) shutdown_tx: tokio::sync::watch::Sender<bool>,
    pub(crate) member_handle: std::sync::Arc<crate::member_actor::MemberHandle>,
    pub(crate) runner_task: Option<tokio::task::JoinHandle<eyre::Result<()>>>,
    pub(crate) total_tasks: usize,
}

/// Wait for completion with Unix signal handling (SIGINT/SIGTERM).
///
/// Uses 3-stage escalation:
/// 1. First signal: graceful SIGTERM + shutdown broadcast
/// 2. Second signal: force SIGTERM
/// 3. Third signal: SIGKILL + immediate exit
#[cfg(unix)]
pub(crate) async fn wait_for_completion_unix<F>(
    pgid: i32,
    ctx: CompletionContext,
    mut background_tasks: FuturesUnordered<F>,
    _task_names: Vec<&'static str>,
    cleanup_guard: &CleanupGuard,
) where
    F: std::future::Future<
        Output = (
            &'static str,
            Result<eyre::Result<()>, tokio::task::JoinError>,
        ),
    >,
{
    use crate::process::kill_process_group;
    use futures::stream::StreamExt;

    let mut sigint = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())
        .expect("Failed to register SIGINT handler");
    let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
        .expect("Failed to register SIGTERM handler");

    // Fuse SIGINT and SIGTERM streams into a unified termination signal stream
    let sigint_stream = async_stream::stream! {
        while let Some(()) = sigint.recv().await {
            yield ();
        }
    };
    let sigterm_stream = async_stream::stream! {
        while let Some(()) = sigterm.recv().await {
            yield ();
        }
    };
    let termination_signals = futures::stream::select(sigint_stream, sigterm_stream);
    tokio::pin!(termination_signals);

    let mut kill_level = 0u8;
    let mut last_signal_sent = std::time::Instant::now() - PROGRESS_INTERVAL; // Initialize to past time

    // Track background task completion during main loop
    let mut background_tasks_completed = 0;
    let mut all_background_tasks_done_logged = false;

    // Keep looping to handle multiple signals until runner task completes
    let runner_future = async {
        if let Some(task) = ctx.runner_task {
            task.await
        } else {
            // No runner task, wait forever
            std::future::pending().await
        }
    };
    tokio::pin!(runner_future);

    loop {
        tokio::select! {
            biased;  // Process signals first for responsive shutdown

            // Unified signal handling for both SIGINT and SIGTERM with 3-stage escalation
            _ = termination_signals.next() => {
                // Ignore feedback signals within 200ms of our own kill_process_group
                // This prevents feedback loops while still allowing future user Ctrl-C presses
                let now = std::time::Instant::now();
                let time_since_last = now.duration_since(last_signal_sent);
                if time_since_last < SIGNAL_DEBOUNCE {
                    debug!("Ignoring signal feedback ({}ms since last signal sent)", time_since_last.as_millis());
                    continue;
                }

                kill_level += 1;

                match kill_level {
                    1 => {
                        info!("Shutting down gracefully (SIGTERM)...");
                        info!("Press Ctrl-C again to force terminate");
                        last_signal_sent = std::time::Instant::now();
                        kill_process_group(pgid, nix::sys::signal::Signal::SIGTERM);
                        debug!("Sending shutdown signal to background tasks...");
                        let _ = ctx.shutdown_tx.send(true);
                        debug!("Shutdown signal sent");
                        let _ = ctx.member_handle.shutdown();
                        // Continue looping to handle more signals
                    }
                    2 => {
                        warn!("Force terminating stubborn processes...");
                        warn!("Press Ctrl-C once more for immediate kill");
                        last_signal_sent = std::time::Instant::now();
                        kill_process_group(pgid, nix::sys::signal::Signal::SIGTERM);
                        // Continue looping to handle more signals
                    }
                    _ => {
                        warn!("Immediate kill! Sending SIGKILL");
                        kill_process_group(pgid, nix::sys::signal::Signal::SIGKILL);
                        std::process::exit(1);
                    }
                }
            }

            // Runner task completion (actors finished) - triggers shutdown
            result = &mut runner_future => {
                debug!("Runner task has completed");
                match result {
                    Ok(Ok(())) => {
                        debug!("All actors completed successfully");
                    }
                    Ok(Err(e)) => {
                        error!("Runner task failed: {:#}", e);
                    }
                    Err(e) => {
                        error!("Runner task panicked: {:#}", e);
                    }
                }
                debug!("Breaking out of wait loop - actors finished");
                break;  // Runner task completion triggers shutdown
            }

            // Background service task completion/failure - logged but doesn't trigger shutdown
            Some((task_name, result)) = background_tasks.next() => {
                background_tasks_completed += 1;
                debug!("Background service task '{}' completed during main loop ({}/{})", task_name, background_tasks_completed, ctx.total_tasks);
                match result {
                    Ok(Ok(())) => {
                        debug!("Background service task '{}' completed successfully", task_name);
                    }
                    Ok(Err(e)) => {
                        error!("Background service task '{}' failed: {:#}", task_name, e);
                    }
                    Err(e) => {
                        error!("Background service task '{}' panicked: {:#}", task_name, e);
                    }
                }

                // Log when all background tasks have completed
                if background_tasks_completed == ctx.total_tasks && !all_background_tasks_done_logged {
                    debug!("All background tasks completed, waiting for actors to finish...");
                    all_background_tasks_done_logged = true;
                }
                // Continue loop - background task completion doesn't trigger shutdown
            }
        }
    }

    // Trigger shutdown for any remaining tasks
    debug!("Triggering shutdown for remaining tasks...");
    debug!("Sending shutdown signal (again, in case missed during main loop)...");
    let _ = ctx.shutdown_tx.send(true);
    debug!("Shutdown signal sent");
    let _ = ctx.member_handle.shutdown();

    // Drain remaining background tasks (exits immediately if already empty)
    // Note: Some tasks may have already completed during the main loop (especially stats task)
    // IMPORTANT: Keep signal handling active during cleanup so Ctrl-C works
    debug!(
        "Draining remaining background tasks (originally {} total)...",
        ctx.total_tasks
    );

    // Set a shorter timeout for background task cleanup (2 seconds)
    // After that, we'll just exit - the tasks will be aborted
    let cleanup_timeout = tokio::time::sleep(CLEANUP_TIMEOUT);
    tokio::pin!(cleanup_timeout);

    // Track which tasks complete during cleanup
    let mut tasks_completed_in_cleanup: Vec<&str> = Vec::new();

    loop {
        tokio::select! {
            biased;

            // Timeout: give up on waiting for background tasks and force exit
            _ = &mut cleanup_timeout => {
                let completed_in_cleanup = tasks_completed_in_cleanup.len();
                if completed_in_cleanup > 0 {
                    debug!("Cleanup timeout: {} tasks completed, exiting", completed_in_cleanup);
                } else {
                    debug!("Cleanup timeout: no tasks completed in cleanup phase, exiting");
                    debug!("Note: Tasks may have already completed during main loop");
                }
                break;
            }

            // Continue handling signals during cleanup (Unix version)
            _ = termination_signals.next() => {
                // Ignore feedback signals within 200ms
                let now = std::time::Instant::now();
                let time_since_last = now.duration_since(last_signal_sent);
                if time_since_last < SIGNAL_DEBOUNCE {
                    debug!("Ignoring signal feedback during cleanup ({}ms since last)", time_since_last.as_millis());
                    continue;
                }

                kill_level += 1;

                match kill_level {
                    2 => {
                        warn!("Force terminating during cleanup...");
                        warn!("Press Ctrl-C once more for immediate kill");
                        last_signal_sent = std::time::Instant::now();
                        kill_process_group(pgid, nix::sys::signal::Signal::SIGTERM);
                    }
                    _ => {
                        warn!("Immediate kill during cleanup! Sending SIGKILL");
                        kill_process_group(pgid, nix::sys::signal::Signal::SIGKILL);
                        std::process::exit(1);
                    }
                }
            }

            // Drain background tasks
            Some((task_name, result)) = background_tasks.next() => {
                tasks_completed_in_cleanup.push(task_name);

                match result {
                    Ok(Ok(())) => {
                        debug!("Background task '{}' completed in cleanup phase", task_name);
                    }
                    Ok(Err(e)) => {
                        warn!("Background task '{}' error in cleanup phase: {:#}", task_name, e);
                    }
                    Err(e) => {
                        warn!("Background task '{}' panicked in cleanup phase: {:#}", task_name, e);
                    }
                }
            }

            // All tasks drained
            else => {
                debug!("All background tasks completed");
                break;
            }
        }
    }

    // Disable CleanupGuard after graceful shutdown completes
    cleanup_guard.disable();
}

/// Wait for completion with Windows signal handling (Ctrl-C)
#[cfg(not(unix))]
pub(crate) async fn wait_for_completion_windows<F>(
    ctx: CompletionContext,
    mut background_tasks: FuturesUnordered<F>,
    _task_names: Vec<&'static str>,
    cleanup_guard: &CleanupGuard,
) where
    F: std::future::Future<
        Output = (
            &'static str,
            Result<eyre::Result<()>, tokio::task::JoinError>,
        ),
    >,
{
    use futures::stream::StreamExt;

    // Track background task completion during main loop
    let mut background_tasks_completed = 0;
    let mut all_background_tasks_done_logged = false;

    // Keep looping to handle multiple Ctrl-C until runner task completes
    let runner_future = async {
        if let Some(task) = ctx.runner_task {
            task.await
        } else {
            // No runner task, wait forever
            std::future::pending().await
        }
    };
    tokio::pin!(runner_future);

    loop {
        tokio::select! {
            biased;

            // Ctrl-C handling
            _ = tokio::signal::ctrl_c() => {
                info!("Received Ctrl-C, shutting down...");
                let _ = ctx.shutdown_tx.send(true);
                let _ = ctx.member_handle.shutdown();
                crate::process::kill_all_descendants();
                std::process::exit(130);
            }

            // Runner task completion (actors finished) - triggers shutdown
            result = &mut runner_future => {
                debug!("Runner task has completed");
                match result {
                    Ok(Ok(())) => {
                        debug!("All actors completed successfully");
                    }
                    Ok(Err(e)) => {
                        error!("Runner task failed: {:#}", e);
                    }
                    Err(e) => {
                        error!("Runner task panicked: {:#}", e);
                    }
                }
                debug!("Breaking out of wait loop - actors finished");
                break;  // Runner task completion triggers shutdown
            }

            // Background service task completion/failure - logged but doesn't trigger shutdown
            Some((task_name, result)) = background_tasks.next() => {
                background_tasks_completed += 1;
                debug!("Background service task '{}' completed during main loop ({}/{})", task_name, background_tasks_completed, ctx.total_tasks);
                match result {
                    Ok(Ok(())) => {
                        debug!("Background service task '{}' completed successfully", task_name);
                    }
                    Ok(Err(e)) => {
                        error!("Background service task '{}' failed: {:#}", task_name, e);
                    }
                    Err(e) => {
                        error!("Background service task '{}' panicked: {:#}", task_name, e);
                    }
                }

                // Log when all background tasks have completed
                if background_tasks_completed == ctx.total_tasks && !all_background_tasks_done_logged {
                    debug!("All background tasks completed, waiting for actors to finish...");
                    all_background_tasks_done_logged = true;
                }
                // Continue loop - background task completion doesn't trigger shutdown
            }
        }
    }

    // Trigger shutdown for any remaining tasks
    debug!("Triggering shutdown for remaining tasks...");
    debug!("Sending shutdown signal (again, in case missed during main loop)...");
    let _ = ctx.shutdown_tx.send(true);
    debug!("Shutdown signal sent");
    let _ = ctx.member_handle.shutdown();

    // Drain remaining background tasks (exits immediately if already empty)
    // Note: Some tasks may have already completed during the main loop (especially stats task)
    // IMPORTANT: Keep signal handling active during cleanup so Ctrl-C works
    debug!(
        "Draining remaining background tasks (originally {} total)...",
        ctx.total_tasks
    );

    // Set a shorter timeout for background task cleanup (2 seconds)
    // After that, we'll just exit - the tasks will be aborted
    let cleanup_timeout = tokio::time::sleep(CLEANUP_TIMEOUT);
    tokio::pin!(cleanup_timeout);

    // Track which tasks complete during cleanup
    let mut tasks_completed_in_cleanup: Vec<&str> = Vec::new();

    loop {
        tokio::select! {
            biased;

            // Timeout: give up on waiting for background tasks and force exit
            _ = &mut cleanup_timeout => {
                let completed_in_cleanup = tasks_completed_in_cleanup.len();
                if completed_in_cleanup > 0 {
                    debug!("Cleanup timeout: {} tasks completed, exiting", completed_in_cleanup);
                } else {
                    debug!("Cleanup timeout: no tasks completed in cleanup phase, exiting");
                    debug!("Note: Tasks may have already completed during main loop");
                }
                break;
            }

            // Continue handling Ctrl-C during cleanup (Windows version)
            _ = tokio::signal::ctrl_c() => {
                let remaining = ctx.total_tasks - tasks_completed_in_cleanup.len();
                warn!("Ctrl-C during cleanup - force killing all processes ({}/{} tasks still pending)", remaining, ctx.total_tasks);
                crate::process::kill_all_descendants();
                std::process::exit(130);
            }

            // Drain background tasks
            Some((task_name, result)) = background_tasks.next() => {
                tasks_completed_in_cleanup.push(task_name);

                match result {
                    Ok(Ok(())) => {
                        debug!("Background task '{}' completed in cleanup phase", task_name);
                    }
                    Ok(Err(e)) => {
                        warn!("Background task '{}' error in cleanup phase: {:#}", task_name, e);
                    }
                    Err(e) => {
                        warn!("Background task '{}' panicked in cleanup phase: {:#}", task_name, e);
                    }
                }
            }

            // All tasks drained
            else => {
                debug!("All background tasks completed");
                break;
            }
        }
    }

    // Disable CleanupGuard after graceful shutdown completes
    cleanup_guard.disable();
}

/// Print periodic startup progress (every 10s while loading, immediate completion message)
pub(crate) async fn print_periodic_statistics(
    member_handle: std::sync::Arc<crate::member_actor::MemberHandle>,
    mut shutdown_signal: tokio::sync::watch::Receiver<bool>,
) {
    // Print progress every 10 seconds while loading
    let mut progress_interval = tokio::time::interval(PROGRESS_INTERVAL);
    progress_interval.tick().await; // Consume the immediate first tick

    // Check completion every 100ms for immediate detection
    let mut completion_check = tokio::time::interval(COMPLETION_CHECK_INTERVAL);
    completion_check.tick().await; // Consume the immediate first tick

    loop {
        tokio::select! {
            _ = progress_interval.tick() => {
                // Print progress update every 10 seconds
                let health = member_handle.get_health_summary().await;
                let startup_complete = health.composable_pending == 0;

                if !startup_complete {
                    info!(
                        "Startup: nodes {}/{} running, containers {}/{} running, composable {}/{} loaded ({} pending)",
                        health.nodes_running,
                        health.nodes_total,
                        health.containers_running,
                        health.containers_total,
                        health.composable_loaded,
                        health.composable_total,
                        health.composable_pending
                    );
                }
                // If complete, the completion_check will handle printing the message
            }
            _ = completion_check.tick() => {
                // Check frequently for completion (every 100ms)
                let health = member_handle.get_health_summary().await;
                let startup_complete = health.composable_pending == 0;

                if startup_complete {
                    // Print completion message immediately
                    let total_failures = health.nodes_failed + health.containers_failed + health.composable_failed;

                    if total_failures == 0 {
                        info!(
                            "Startup complete: all nodes ready (nodes {}/{}, containers {}/{}, composable {}/{})",
                            health.nodes_running,
                            health.nodes_total,
                            health.containers_running,
                            health.containers_total,
                            health.composable_loaded,
                            health.composable_total
                        );
                    } else {
                        warn!(
                            "Startup complete with failures: {} nodes failed, {} containers failed, {} composable failed (nodes {}/{} running, containers {}/{} running, composable {}/{} loaded)",
                            health.nodes_failed,
                            health.containers_failed,
                            health.composable_failed,
                            health.nodes_running,
                            health.nodes_total,
                            health.containers_running,
                            health.containers_total,
                            health.composable_loaded,
                            health.composable_total
                        );
                    }

                    debug!("Startup complete, stopping periodic progress updates");
                    break;
                }
            }
            _ = shutdown_signal.changed() => {
                if *shutdown_signal.borrow() {
                    debug!("Periodic statistics task shutting down");
                    break;
                }
            }
        }
    }
}
