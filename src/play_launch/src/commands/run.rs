//! Run command - execute a single ROS node

use crate::{
    cli,
    cli::config::load_runtime_config,
    execution::context::{prepare_node_contexts, NodeContextClasses},
    member_actor::{ActorConfig, MemberCoordinatorBuilder},
    monitoring::resource_monitor::MonitorConfig,
    process::{kill_all_descendants, kill_process_group},
    ros::launch_dump::LaunchDump,
    util::{log_dir::create_log_dir, logging::is_verbose},
    web,
};
use eyre::WrapErr;
use futures::stream::{FuturesUnordered, StreamExt};
use std::{
    collections::{HashMap, HashSet},
    fs,
    path::PathBuf,
    sync::{Arc, Mutex},
};
use tracing::{debug, error, info, warn};

/// Guard that ensures child processes are cleaned up on drop
struct CleanupGuard {
    enabled: Arc<std::sync::atomic::AtomicBool>,
}

impl CleanupGuard {
    fn new() -> Self {
        Self {
            enabled: Arc::new(std::sync::atomic::AtomicBool::new(true)),
        }
    }

    /// Disable the cleanup guard (call after graceful shutdown completes)
    fn disable(&self) {
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

pub fn handle_run(args: &cli::options::RunArgs) -> eyre::Result<()> {
    use crate::ros::launch_dump::{LaunchDump, NodeRecord};
    use tokio::runtime::Runtime;

    info!("Running single node: {} {}", args.package, args.executable);

    // Build command line for the node
    let mut cmd = vec![
        "ros2".to_string(),
        "run".to_string(),
        args.package.clone(),
        args.executable.clone(),
    ];
    cmd.extend(args.args.clone());

    // Create a minimal LaunchDump with a single node
    let node_record = NodeRecord {
        executable: args.executable.clone(),
        package: Some(args.package.clone()),
        name: Some(args.executable.clone()),
        namespace: Some("/".to_string()),
        exec_name: Some(args.executable.clone()),
        params: vec![],
        params_files: vec![],
        global_params: None,
        remaps: vec![],
        env: None,
        ros_args: None,
        args: Some(args.args.clone()),
        cmd,
        respawn: args.common.disable_respawn.then_some(false),
        respawn_delay: Some(0.0),
    };

    let launch_dump = LaunchDump {
        node: vec![node_record],
        load_node: vec![],
        container: vec![],
        lifecycle_node: vec![],
        file_data: HashMap::new(),
    };

    // Build the async runtime and run directly
    let runtime = Runtime::new()?;
    debug!(
        "Tokio runtime created (default config uses {} worker threads = num CPUs)",
        num_cpus::get()
    );

    runtime.block_on(run_direct(&launch_dump, &args.common))
}

async fn run_direct(
    launch_dump: &LaunchDump,
    common: &cli::options::CommonOptions,
) -> eyre::Result<()> {
    debug!("Starting direct node execution");

    // Install cleanup guard
    let cleanup_guard = CleanupGuard::new();
    debug!("CleanupGuard installed");

    // Spawn anchor task and get PGID (Phase 4: async anchor with shutdown support)
    #[cfg(unix)]
    let (pgid_tx, pgid_rx) = tokio::sync::oneshot::channel();
    #[cfg(unix)]
    let (pgid, shutdown_tx, shutdown_rx, anchor_task) = {
        use crate::process::pgid::run_anchor_task;

        // Create shutdown channel for anchor (will be cloned for other tasks)
        let (shutdown_tx, shutdown_rx) = tokio::sync::watch::channel(false);

        // Spawn anchor task
        let anchor_task = tokio::spawn(run_anchor_task(pgid_tx, shutdown_rx.clone()));

        // Wait for PGID
        let pgid = pgid_rx
            .await
            .wrap_err("Failed to receive PGID from anchor task")?;
        debug!("Anchor process task started with PGID: {}", pgid);

        (pgid, shutdown_tx, shutdown_rx, anchor_task)
    };

    #[cfg(not(unix))]
    let (pgid, shutdown_tx, shutdown_rx, anchor_task) = {
        let (shutdown_tx, shutdown_rx) = tokio::sync::watch::channel(false);
        // Dummy anchor task for non-Unix platforms
        let anchor_task = tokio::spawn(async { Ok(()) });
        (0i32, shutdown_tx, shutdown_rx, anchor_task)
    };

    // Load runtime configuration
    info!("Loading runtime configuration...");
    let runtime_config = load_runtime_config(
        common.config.as_deref(),
        common.enable_monitoring,
        common.monitor_interval_ms,
        common.enable_diagnostics,
    )?;
    info!("Runtime configuration loaded successfully");

    // Create temporary log directory
    info!("Creating log directories...");
    let log_dir = create_log_dir(&common.log_dir)?;
    info!("Log directory created: {}", log_dir.display());

    let node_log_dir = log_dir.join("node");
    fs::create_dir(&node_log_dir)?;
    info!("Created node log directory");

    // Create diagnostic registry (empty - diagnostic monitoring not supported in 'run' mode)
    let diagnostic_registry = Arc::new(crate::diagnostics::DiagnosticRegistry::new());

    // Initialize NVML for GPU monitoring
    let nvml = match nvml_wrapper::Nvml::init() {
        Ok(nvml) => {
            let device_count = nvml.device_count().unwrap_or(0);
            info!(
                "NVML initialized successfully with {} GPU device(s)",
                device_count
            );
            Some(nvml)
        }
        Err(e) => {
            error!("Failed to initialize NVML: {}", e);
            None
        }
    };

    // Initialize monitoring (now using async tokio task!)
    let process_registry = Arc::new(Mutex::new(HashMap::<u32, PathBuf>::new()));
    debug!("Process registry initialized (empty)");
    let monitor_task = if runtime_config.monitoring.enabled {
        let monitor_config = MonitorConfig {
            sample_interval_ms: runtime_config.monitoring.sample_interval_ms,
        };

        if is_verbose() {
            info!(
                "Resource monitoring enabled (interval: {}ms)",
                runtime_config.monitoring.sample_interval_ms
            );
        } else {
            debug!(
                "Resource monitoring enabled (interval: {}ms)",
                runtime_config.monitoring.sample_interval_ms
            );
        }

        // Spawn async monitoring task (tokio task, not thread!)
        let task = tokio::spawn(crate::monitoring::resource_monitor::run_monitoring_task(
            monitor_config,
            log_dir.clone(),
            process_registry.clone(),
            nvml,
            shutdown_rx.clone(),
        ));

        debug!("Monitoring task spawned successfully");
        Some(task)
    } else {
        debug!("Resource monitoring disabled");
        None
    };

    // Prepare node execution contexts
    let container_names = HashSet::new();
    let NodeContextClasses {
        container_contexts: _,
        non_container_node_contexts: pure_node_contexts,
    } = prepare_node_contexts(launch_dump, &node_log_dir, &container_names)?;

    // Create MemberCoordinatorBuilder
    let mut builder = MemberCoordinatorBuilder::new();

    debug!("Adding {} node(s) to builder", pure_node_contexts.len());

    // Add actors to builder
    for context in pure_node_contexts {
        // Use the node name from record.json directly
        let member_name = context
            .record
            .name
            .as_ref()
            .cloned()
            .unwrap_or_else(|| "unknown".to_string());

        // Create actor config
        let actor_config = ActorConfig {
            respawn_enabled: !common.disable_respawn && context.record.respawn.unwrap_or(false),
            respawn_delay: context.record.respawn_delay.unwrap_or(0.0),
            max_respawn_attempts: None,
            output_dir: context.output_dir.clone(),
            pgid: Some(pgid),
            list_nodes_loading_timeout_secs: 30, // Not applicable for regular nodes
        };

        // Add to builder
        builder.add_regular_node(
            member_name,
            context,
            actor_config,
            Some(process_registry.clone()),
        );
    }

    // Spawn all actors and get handle + runner
    // No shared ROS node needed for run command (single node, no containers)
    debug!("Spawning all {} actors...", builder.member_count());
    let (member_handle, member_runner) = builder.spawn(None, None).await;
    let member_handle = std::sync::Arc::new(member_handle); // Wrap in Arc for sharing
    debug!("All actors spawned successfully");

    // Setup web UI if requested (direct StateEvent streaming)
    let (runner_task, web_ui_task) = if common.web_ui {
        debug!("Setting up web UI with direct StateEvent streaming...");

        // Create state event broadcaster for SSE clients
        let state_broadcaster = std::sync::Arc::new(web::StateEventBroadcaster::new());

        // Start web server
        let web_state = Arc::new(web::WebState::new(
            member_handle.clone(), // Clone the Arc, not MemberHandle
            log_dir.clone(),
            state_broadcaster.clone(),
            diagnostic_registry.clone(),
        ));
        let addr = common.web_ui_addr.clone();
        let port = common.web_ui_port;

        // Log web UI URL before spawning (addr will be moved)
        info!("Web UI available at http://{}:{}", addr, port);

        let (_shutdown_tx, web_shutdown) = tokio::sync::watch::channel(false);

        // Spawn web server task
        let web_server_task = tokio::spawn(async move {
            if let Err(e) = web::run_server(web_state, &addr, port, web_shutdown).await {
                error!("Web server error: {}", e);
            }
            Ok(())
        });

        // Runner task forwards state events and waits for completion
        let runner_task = tokio::spawn(async move {
            forward_state_events_and_wait(member_runner, state_broadcaster).await
        });

        (Some(runner_task), Some(web_server_task))
    } else {
        // Runner task just waits for completion (no forwarding)
        let runner_task = tokio::spawn(async move { member_runner.wait_for_completion().await });

        (Some(runner_task), None)
    };

    // Phase 6: Collect all background tasks into FuturesUnordered for unified lifecycle management
    debug!("Setting up FuturesUnordered for background tasks...");
    let mut background_tasks = FuturesUnordered::new();

    // Add anchor task (always present on Unix)
    background_tasks.push(anchor_task);

    // Add optional monitoring task
    if let Some(task) = monitor_task {
        background_tasks.push(task);
    }

    // Add runner task (always present)
    if let Some(task) = runner_task {
        background_tasks.push(task);
    }

    // Add optional web UI task
    if let Some(task) = web_ui_task {
        background_tasks.push(task);
    }

    debug!(
        "Background tasks collection created ({} tasks)",
        background_tasks.len()
    );

    // Install signal handlers for graceful shutdown with escalation
    let mut kill_level = 0u8;
    debug!("Signal handlers installed, entering main event loop");

    // Wait for background tasks (including runner) to complete OR signals
    debug!("Waiting for completion (background tasks or signals)...");

    #[cfg(unix)]
    {
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

        let mut self_initiated_shutdown = false;

        // Keep looping to handle multiple signals until actors/tasks complete
        loop {
            tokio::select! {
                biased;  // Process signals first for responsive shutdown

                // Unified signal handling for both SIGINT and SIGTERM with 3-stage escalation
                _ = termination_signals.next() => {
                    // Ignore feedback from our own kill_process_group
                    if !self_initiated_shutdown {
                        kill_level += 1;

                        match kill_level {
                            1 => {
                                info!("Shutting down gracefully (SIGTERM)...");
                                info!("Press Ctrl-C again to force terminate");
                                self_initiated_shutdown = true;
                                if pgid != 0 {
                                    kill_process_group(pgid, nix::sys::signal::Signal::SIGTERM);
                                }
                                let _ = shutdown_tx.send(true);
                                // Continue looping to handle more signals
                            }
                            2 => {
                                warn!("Force terminating stubborn processes...");
                                warn!("Press Ctrl-C once more for immediate kill");
                                if pgid != 0 {
                                    kill_process_group(pgid, nix::sys::signal::Signal::SIGTERM);
                                }
                                // Continue looping to handle more signals
                            }
                            _ => {
                                warn!("Immediate kill! Sending SIGKILL");
                                if pgid != 0 {
                                    kill_process_group(pgid, nix::sys::signal::Signal::SIGKILL);
                                }
                                std::process::exit(1);
                            }
                        }
                    }
                }

                // Background task completion/failure (including runner task)
                Some(result) = background_tasks.next() => {
                    // A background task finished (usually means error or shutdown)
                    match result {
                        Ok(Ok(())) => {
                            warn!("Background task finished early (clean exit)");
                        }
                        Ok(Err(e)) => {
                            error!("Background task failed: {:#}", e);
                        }
                        Err(e) => {
                            error!("Background task panicked: {:#}", e);
                        }
                    }
                    break;  // Task failure, exit loop
                }
            }
        }
    }

    #[cfg(not(unix))]
    {
        // Keep looping to handle multiple Ctrl-C until actors/tasks complete
        loop {
            tokio::select! {
                biased;

                // Ctrl-C handling
                _ = tokio::signal::ctrl_c() => {
                    info!("Received Ctrl-C, shutting down...");
                    let _ = shutdown_tx.send(true);
                    kill_all_descendants();
                    std::process::exit(130);
                }

                // Background task completion/failure (including runner task)
                Some(result) = background_tasks.next() => {
                    // A background task finished (usually means error or shutdown)
                    match result {
                        Ok(Ok(())) => {
                            warn!("Background task finished early (clean exit)");
                        }
                        Ok(Err(e)) => {
                            error!("Background task failed: {:#}", e);
                        }
                        Err(e) => {
                            error!("Background task panicked: {:#}", e);
                        }
                    }
                    break;  // Task failure, exit loop
                }
            }
        }
    }

    // Trigger shutdown for any remaining tasks
    debug!("Triggering shutdown for remaining tasks...");
    let _ = shutdown_tx.send(true);

    // Drain remaining background tasks (exits immediately if already empty)
    // With async component loader, all tasks should complete quickly on shutdown
    debug!("Draining remaining background tasks...");
    while let Some(result) = background_tasks.next().await {
        match result {
            Ok(Ok(())) => {
                debug!("Background task completed successfully");
            }
            Ok(Err(e)) => {
                warn!("Background task error during shutdown: {:#}", e);
            }
            Err(e) => {
                warn!("Background task panic during shutdown: {:#}", e);
            }
        }
    }
    debug!("All background tasks completed");

    // Handle graceful shutdown with process cleanup (same logic as before)
    #[cfg(unix)]
    let result = handle_shutdown_simple(Some(pgid), member_handle, &cleanup_guard).await;
    #[cfg(not(unix))]
    let result = handle_shutdown_simple(None, member_handle, &cleanup_guard).await;

    result
}

/// Forward state events from runner to SSE broadcaster, then wait for all actors to complete
///
/// This combines event forwarding with completion waiting in a single task.
async fn forward_state_events_and_wait(
    mut runner: crate::member_actor::MemberRunner,
    broadcaster: Arc<crate::web::StateEventBroadcaster>,
) -> eyre::Result<()> {
    tracing::debug!("Starting state event forwarding and completion waiting");

    // Forward events until done
    while let Some(event) = runner.next_state_event().await {
        broadcaster.broadcast(event).await;
    }

    // Join all actor tasks
    runner.wait_for_completion().await
}

/// Handle graceful shutdown (runner task already waited for completion)
async fn handle_shutdown_simple(
    pgid: Option<i32>,
    _member_handle: std::sync::Arc<crate::member_actor::MemberHandle>,
    cleanup_guard: &CleanupGuard,
) -> eyre::Result<()> {
    // Runner task already waited for actors to complete
    let result = Ok(());

    #[cfg(unix)]
    {
        use nix::sys::signal::Signal;

        info!("All actors completed");

        // Graceful shutdown check
        if let Some(pgid) = pgid {
            // Wait up to 5 seconds for graceful shutdown
            let start = std::time::Instant::now();
            let mut all_exited_gracefully = false;

            while start.elapsed() < std::time::Duration::from_secs(5) {
                // Check if any processes are still running
                let has_running = nix::sys::wait::waitpid(
                    nix::unistd::Pid::from_raw(-pgid),
                    Some(nix::sys::wait::WaitPidFlag::WNOHANG),
                );
                match has_running {
                    Ok(nix::sys::wait::WaitStatus::Exited(_, _))
                    | Ok(nix::sys::wait::WaitStatus::Signaled(_, _, _)) => {
                        // A child exited, keep waiting for others
                        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                    }
                    Err(nix::errno::Errno::ECHILD) => {
                        // No more children
                        info!("All processes exited gracefully");
                        all_exited_gracefully = true;
                        cleanup_guard.disable();
                        break;
                    }
                    _ => {
                        // Still have children, wait a bit
                        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                    }
                }
            }

            // If still running after grace period, force kill
            if !all_exited_gracefully && start.elapsed() >= std::time::Duration::from_secs(5) {
                info!("Grace period expired, sending SIGKILL to remaining processes");
                kill_process_group(pgid, Signal::SIGKILL);
            }
        }

        info!("All child processes terminated");
    }

    #[cfg(not(unix))]
    {
        info!("All actors completed");
        kill_all_descendants();
        info!("All child processes terminated");
    }

    result
}
