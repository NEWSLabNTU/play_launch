//! Shared utilities for command handlers

use crate::{cli::options::ParserBackend, process::kill_all_descendants};
use eyre::Context as _;
use std::sync::Arc;
use tracing::debug;

/// Maximum number of Tokio worker threads (async workload is mostly idle)
const MAX_WORKER_THREADS: usize = 8;

/// Build a Tokio multi-thread runtime with adaptive thread pool configuration.
///
/// Uses number of CPUs capped at `MAX_WORKER_THREADS` for efficiency.
/// Automatically adapts to platform: 2-core Pi, 4-core laptop, 8-core AGX Orin, 32-core server.
pub(crate) fn build_tokio_runtime() -> eyre::Result<tokio::runtime::Runtime> {
    let worker_threads = std::cmp::min(num_cpus::get(), MAX_WORKER_THREADS);
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

/// Parse a launch file into an in-memory [`crate::ros::launch_dump::LaunchDump`]
/// — the parser-agnostic intermediate `resolve`/`launch` build a SystemModel
/// from. Phase 47.B4/B5: no user-facing artifact results from this. The
/// Rust parser path never touches disk (parse → JSON string → deserialize,
/// entirely in memory — the same trick `resolve`'s Rust branch has used
/// since Phase 46.4). The Python parser path necessarily round-trips
/// through a private OS-temp scratch file (the PyO3 `dump_launch` bridge
/// itself only knows how to write JSON to a path) that is deleted before
/// this function returns — an interop detail, not a `record.json` companion
/// left for the user.
///
/// Returns the dump plus the resolved launch file path (used for
/// provenance hashing by callers).
pub(crate) async fn parse_to_launch_dump(
    package_or_path: &str,
    launch_file: Option<&str>,
    launch_arguments: &[String],
    parser: ParserBackend,
) -> eyre::Result<(crate::ros::launch_dump::LaunchDump, std::path::PathBuf)> {
    let launch_path = super::launch::resolve_launch_file(package_or_path, launch_file)?;

    let dump = match parser {
        ParserBackend::Rust => {
            let cli_args = super::parse_launch_arguments(launch_arguments);
            let record =
                play_launch_parser::parse_launch_file(&launch_path, cli_args).map_err(|e| {
                    eyre::eyre!(
                        "Rust parser error: {e}\n\nHint: If you encounter parsing issues, try \
                         the Python parser:\n  play_launch <cmd> {package_or_path} {} --parser \
                         python",
                        launch_file.unwrap_or("")
                    )
                })?;
            let json = serde_json::to_string_pretty(&record)?;
            serde_json::from_str(&json)?
        }
        ParserBackend::Python => {
            let scratch_path = std::env::temp_dir().join(format!(
                "play_launch-parse-{}-{}.record.json",
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_nanos())
                    .unwrap_or(0),
            ));
            let launcher = crate::python::dump_launcher::DumpLauncher::new()
                .wrap_err("Failed to initialize dump_launch. Ensure ROS workspace is sourced.")?;
            launcher
                .dump_launch(
                    package_or_path,
                    launch_file,
                    launch_arguments,
                    &scratch_path,
                )
                .await?;
            let dump =
                crate::ros::launch_dump::load_launch_dump(&scratch_path).wrap_err_with(|| {
                    format!("loading Python-parsed record {}", scratch_path.display())
                })?;
            let _ = std::fs::remove_file(&scratch_path);

            // Phase 46.5 — fail loud on a stale pre-Phase-40.1 Python install
            // (missing `ScopeOrigin.path`). Shared across every caller so
            // stale usage can never silently pass anywhere.
            crate::ros::launch_dump::ensure_python_scope_paths(&dump)?;
            dump
        }
    };

    Ok((dump, launch_path))
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
