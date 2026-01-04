//! Replay command - replay recorded launch execution

use crate::{
    cli,
    cli::config::load_runtime_config,
    execution::context::{
        prepare_composable_node_contexts, prepare_node_contexts, ComposableNodeContextSet,
        NodeContextClasses,
    },
    monitoring::resource_monitor::MonitorConfig,
    process::{kill_all_descendants, kill_process_group},
    ros::{
        container_readiness::SERVICE_DISCOVERY_HANDLE,
        launch_dump::{load_launch_dump, NodeContainerRecord},
    },
    util::{log_dir::create_log_dir, logging::is_verbose},
    web,
};
use eyre::Context;
use futures::stream::FuturesUnordered;
use rayon::prelude::*;
use std::{
    collections::{HashMap, HashSet},
    fs,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
};
use tokio::runtime::Runtime;
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

pub fn handle_replay(args: &cli::options::ReplayArgs) -> eyre::Result<()> {
    let input_file = &args.input_file;

    // Load runtime configuration to check service readiness settings
    let runtime_config = cli::config::load_runtime_config(
        args.common.config.as_deref(),
        args.common.enable_monitoring,
        args.common.monitor_interval_ms,
    )?;

    // Print configuration summary
    info!("Configuration:");
    info!(
        "  Monitoring: {}",
        if runtime_config.monitoring.enabled {
            "enabled"
        } else {
            "disabled"
        }
    );
    if runtime_config.monitoring.enabled {
        info!(
            "  Monitoring interval: {}ms",
            runtime_config.monitoring.sample_interval_ms
        );
    }
    info!(
        "  Container readiness: {}",
        if runtime_config.container_readiness.wait_for_service_ready {
            "enabled"
        } else {
            "disabled"
        }
    );
    if runtime_config.container_readiness.wait_for_service_ready {
        let timeout = runtime_config
            .container_readiness
            .service_ready_timeout_secs;
        if timeout == 0 {
            info!("  Container timeout: unlimited");
        } else {
            info!("  Container timeout: {}s", timeout);
        }
    }
    if args.common.web_ui {
        info!(
            "  Web UI: http://{}:{}",
            args.common.web_ui_addr, args.common.web_ui_port
        );
    }

    // Note: ROS service discovery and anchor process now started in play() async function (Phase 2, 4)

    // Build the async runtime
    let runtime = Runtime::new()?;

    // Run the whole playing task in the runtime
    runtime.block_on(play(input_file, &args.common))
}

/// Play the launch according to the launch record.
async fn play(input_file: &Path, common: &cli::options::CommonOptions) -> eyre::Result<()> {
    debug!("=== Starting play() function ===");

    // Install cleanup guard to ensure children are killed even if we're interrupted
    let cleanup_guard = CleanupGuard::new();
    debug!("CleanupGuard installed");

    // Spawn anchor process task to allocate PGID (Phase 4: async task with shutdown)
    #[cfg(unix)]
    let (pgid_tx, pgid_rx) = tokio::sync::oneshot::channel();
    #[cfg(unix)]
    let pgid = {
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

        // Store shutdown sender for later use
        let shutdown_signal = shutdown_rx;

        (pgid, shutdown_tx, shutdown_signal, anchor_task)
    };

    #[cfg(not(unix))]
    let (pgid, shutdown_tx, shutdown_signal, anchor_task) = {
        let (shutdown_tx, shutdown_rx) = tokio::sync::watch::channel(false);
        // Dummy anchor task for non-Unix platforms
        let anchor_task = tokio::spawn(async { Ok(()) });
        (0i32, shutdown_tx, shutdown_rx, anchor_task)
    };

    #[cfg(unix)]
    let (pgid, shutdown_tx, shutdown_signal, anchor_task) = pgid;
    #[cfg(not(unix))]
    let (pgid, shutdown_tx, shutdown_signal, anchor_task) =
        (pgid, shutdown_tx, shutdown_signal, anchor_task);

    // Load runtime configuration
    debug!("Loading runtime configuration...");
    let runtime_config = load_runtime_config(
        common.config.as_deref(),
        common.enable_monitoring,
        common.monitor_interval_ms,
    )?;
    debug!("Runtime configuration loaded successfully");

    debug!("Loading launch dump from: {}", input_file.display());
    let launch_dump = load_launch_dump(input_file)?;
    debug!("Launch dump loaded successfully");

    // Prepare directories
    debug!("Creating log directories...");
    let log_dir = create_log_dir(&common.log_dir)?;
    debug!("Log directory created: {}", log_dir.display());

    let params_files_dir = log_dir.join("params_files");
    fs::create_dir(&params_files_dir)?;
    debug!("Created params_files directory");

    let node_log_dir = log_dir.join("node");
    fs::create_dir(&node_log_dir)
        .wrap_err_with(|| format!("unable to create directory {}", node_log_dir.display()))?;
    debug!("Created node log directory");

    let load_node_log_dir = log_dir.join("load_node");
    fs::create_dir(&load_node_log_dir)
        .wrap_err_with(|| format!("unable to create directory {}", load_node_log_dir.display()))?;
    debug!("Created load_node log directory");

    // Initialize NVML for GPU monitoring
    debug!("Initializing NVML...");
    let nvml = match nvml_wrapper::Nvml::init() {
        Ok(nvml) => {
            let device_count = nvml.device_count().unwrap_or(0);
            debug!(
                "NVML initialized successfully with {} GPU device(s)",
                device_count
            );
            Some(nvml)
        }
        Err(e) => {
            error!("Failed to initialize NVML: {}", e);
            error!("GPU monitoring will be unavailable. Ensure NVIDIA drivers are installed.");
            None
        }
    };

    // Shutdown signal already created with anchor process (Phase 4)

    // Create ONE shared ROS node for all ROS operations (service discovery, LoadNode calls, etc.)
    debug!("Creating shared ROS node and executor...");
    let shared_ros_node = {
        use std::sync::mpsc as std_mpsc;
        let (node_tx, node_rx) = std_mpsc::channel();
        let shutdown_rx_executor = shutdown_signal.clone();

        // Spawn a dedicated thread for the ROS executor
        // This thread will spin the executor continuously
        let executor_thread = std::thread::spawn(move || {
            use rclrs::CreateBasicExecutor;
            use tracing::{debug, error};

            debug!("Starting ROS executor thread for all ROS operations");

            // Create ROS context, executor, and node
            let context = match rclrs::Context::new(
                vec!["play_launch".to_string()],
                rclrs::InitOptions::default(),
            ) {
                Ok(ctx) => ctx,
                Err(e) => {
                    error!("Failed to create ROS context: {:#}", e);
                    return;
                }
            };

            let mut executor = context.create_basic_executor();
            let node = match executor.create_node("play_launch") {
                Ok(n) => Arc::new(n),
                Err(e) => {
                    error!("Failed to create ROS node: {:#}", e);
                    return;
                }
            };

            // Send node back to main thread
            if node_tx.send(node).is_err() {
                error!("Failed to send node to main thread");
                return;
            }

            debug!("ROS executor thread: node created, starting spin loop");

            // Spin the executor until shutdown
            loop {
                // Check for shutdown signal (non-blocking)
                if shutdown_rx_executor.has_changed().is_ok() && *shutdown_rx_executor.borrow() {
                    debug!("ROS executor thread received shutdown signal");
                    break;
                }

                // Spin once
                executor.spin(rclrs::SpinOptions::spin_once());

                // Small sleep to prevent busy-waiting
                std::thread::sleep(std::time::Duration::from_millis(10));
            }

            debug!("ROS executor thread shutting down");
        });

        // The executor thread will run until shutdown signal is received
        std::mem::forget(executor_thread); // Don't block on join

        // Wait for the node to be created
        match node_rx.recv() {
            Ok(node) => {
                debug!("Shared ROS node created: /play_launch");
                Some(node)
            }
            Err(e) => {
                error!("Failed to receive node from executor thread: {:#}", e);
                error!("ROS operations (service discovery, LoadNode) will not be available");
                None
            }
        }
    };

    // Initialize monitoring if enabled (now using async tokio task!)
    debug!("Initializing monitoring and process registry...");
    let process_registry = Arc::new(Mutex::new(HashMap::<u32, PathBuf>::new()));
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
            shutdown_signal.clone(),
        ));

        Some(task)
    } else {
        debug!("Resource monitoring disabled");
        None
    };
    debug!("Monitoring initialization complete");

    // Start ROS service discovery task if enabled (default: true) (Phase 2: async task)
    let service_discovery_task = if runtime_config.container_readiness.wait_for_service_ready {
        debug!("Starting ROS service discovery task...");

        // Spawn async service discovery task (tokio task, not thread!)
        // Pass the shared ROS node for service discovery
        let (task, handle) = crate::ros::container_readiness::spawn_service_discovery_task(
            shared_ros_node.clone(),
            shutdown_signal.clone(),
        );

        // Store handle in global
        SERVICE_DISCOVERY_HANDLE
            .set(handle)
            .expect("SERVICE_DISCOVERY_HANDLE already set");

        debug!("ROS service discovery task started successfully");
        Some(task)
    } else {
        info!("Service readiness checking disabled (set container_readiness.wait_for_service_ready: true in config to enable)");
        None
    };

    // Build a table of composable node containers
    debug!("Building container names table...");
    let container_names: HashSet<String> = launch_dump
        .container
        .par_iter()
        .map(|record| {
            let NodeContainerRecord { namespace, name } = record;
            // Handle namespace ending with '/' to avoid double slashes
            if namespace.ends_with('/') {
                format!("{namespace}{name}")
            } else {
                format!("{namespace}/{name}")
            }
        })
        .collect();
    debug!(
        "Container names table built with {} containers",
        container_names.len()
    );

    // Prepare node execution contexts
    debug!("Preparing node execution contexts...");
    let NodeContextClasses {
        container_contexts,
        non_container_node_contexts: pure_node_contexts,
    } = prepare_node_contexts(&launch_dump, &node_log_dir, &container_names)?;

    // Prepare LoadNode request execution contexts
    let ComposableNodeContextSet { load_node_contexts } =
        prepare_composable_node_contexts(&launch_dump, &load_node_log_dir)?;

    // Create MemberCoordinatorBuilder for collecting member definitions
    debug!("Creating MemberCoordinatorBuilder...");
    let mut builder = crate::member_actor::MemberCoordinatorBuilder::new();

    // Print summary of nodes to spawn
    let num_pure_nodes = pure_node_contexts.len();
    let num_containers = container_contexts.len();
    let num_composable_nodes = load_node_contexts.len();
    info!(
        "Spawning {} nodes ({} pure nodes, {} containers, {} composable nodes)",
        num_pure_nodes + num_containers + num_composable_nodes,
        num_pure_nodes,
        num_containers,
        num_composable_nodes
    );

    // Add regular nodes to builder
    debug!("Adding {} regular nodes", num_pure_nodes);
    for context in pure_node_contexts {
        let dir_name = context
            .output_dir
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("unknown");
        let member_name = format!("NODE '{}'", dir_name);

        let actor_config = crate::member_actor::ActorConfig {
            respawn_enabled: !common.disable_respawn && context.record.respawn.unwrap_or(false),
            respawn_delay: context.record.respawn_delay.unwrap_or(0.0),
            max_respawn_attempts: None,
            output_dir: context.output_dir.clone(),
            pgid: Some(pgid),
        };

        builder.add_regular_node(
            member_name,
            context,
            actor_config,
            Some(process_registry.clone()),
        );
    }

    // Add containers
    debug!("Adding {} containers", num_containers);
    for context in container_contexts {
        let dir_name = context
            .node_context
            .output_dir
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("unknown");
        let member_name = format!("NODE '{}'", dir_name);

        let actor_config = crate::member_actor::ActorConfig {
            respawn_enabled: !common.disable_respawn
                && context.node_context.record.respawn.unwrap_or(false),
            respawn_delay: context.node_context.record.respawn_delay.unwrap_or(0.0),
            max_respawn_attempts: None,
            output_dir: context.node_context.output_dir.clone(),
            pgid: Some(pgid),
        };

        // Add container (oneshot receiver is ignored since composable nodes will be matched internally)
        std::mem::drop(builder.add_container(
            member_name.clone(),
            context.node_context,
            actor_config,
            Some(process_registry.clone()),
        ));
    }

    // Add composable nodes (builder will match them with containers during spawn())
    debug!("Adding {} composable nodes", num_composable_nodes);

    let composable_config = crate::member_actor::ComposableActorConfig::default();

    for context in load_node_contexts {
        let member_name = format!("LOAD_NODE '{}'", context.record.node_name);

        builder.add_composable_node(member_name, context, composable_config.clone());
    }

    // Now spawn all actors at once and get handle + runner
    // Pass the shared ROS node for container actors to use
    debug!("Spawning all {} actors...", builder.member_count());
    let (member_handle, member_runner) = builder.spawn(shared_ros_node).await;
    let member_handle = std::sync::Arc::new(member_handle); // Wrap in Arc for sharing
    debug!("All actors spawned successfully");

    // Setup web UI if requested (direct StateEvent streaming)
    let (runner_task, web_ui_task) = if common.web_ui {
        debug!("Setting up web UI with direct StateEvent streaming...");

        // Create state event broadcaster for SSE clients
        let state_broadcaster = std::sync::Arc::new(web::StateEventBroadcaster::new());

        // Start web server
        let web_state = std::sync::Arc::new(web::WebState::new(
            member_handle.clone(), // Clone the Arc, not MemberHandle
            log_dir.clone(),
            state_broadcaster.clone(),
        ));
        let addr = common.web_ui_addr.clone();
        let port = common.web_ui_port;
        let web_shutdown = shutdown_signal.clone();

        // Log web UI URL before spawning (addr will be moved)
        info!("Web UI available at http://{}:{}", addr, port);

        // Spawn web server
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
        let runner_task = tokio::spawn(async move {
            tracing::debug!("Runner task: starting, about to call wait_for_completion");
            let result = member_runner.wait_for_completion().await;
            tracing::debug!("Runner task: wait_for_completion returned: {:?}", result);
            result
        });

        (Some(runner_task), None)
    };

    // Phase 6: Collect background tasks (NOT including runner task) into FuturesUnordered
    // The runner task is special - only its completion should trigger shutdown
    debug!("Setting up FuturesUnordered for background service tasks...");
    let background_tasks = FuturesUnordered::new();

    // Add anchor task (always present)
    background_tasks.push(anchor_task);

    // Add optional service tasks (these can complete without triggering shutdown)
    if let Some(task) = monitor_task {
        background_tasks.push(task);
    }
    if let Some(task) = service_discovery_task {
        background_tasks.push(task);
    }
    if let Some(task) = web_ui_task {
        background_tasks.push(task);
    }

    debug!(
        "Background service tasks collection created ({} tasks)",
        background_tasks.len()
    );

    // Wait for either actors to complete (runner task) OR signals
    debug!("Waiting for completion (runner task or signals)...");

    #[cfg(unix)]
    wait_for_completion_unix(
        pgid,
        shutdown_tx.clone(),
        member_handle,
        runner_task,
        background_tasks,
        &cleanup_guard,
    )
    .await;

    #[cfg(not(unix))]
    wait_for_completion_windows(
        shutdown_tx.clone(),
        member_handle,
        runner_task,
        background_tasks,
        &cleanup_guard,
    )
    .await;

    debug!("play() function completed, returning");
    Ok(())
}

/// Wait for completion with Unix signal handling (SIGINT/SIGTERM)
#[cfg(unix)]
async fn wait_for_completion_unix(
    pgid: i32,
    shutdown_tx: tokio::sync::watch::Sender<bool>,
    member_handle: std::sync::Arc<crate::member_actor::MemberHandle>,
    runner_task: Option<tokio::task::JoinHandle<eyre::Result<()>>>,
    mut background_tasks: FuturesUnordered<tokio::task::JoinHandle<eyre::Result<()>>>,
    cleanup_guard: &CleanupGuard,
) {
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
    let mut self_initiated_shutdown = false; // Track if we sent SIGTERM to avoid feedback loop

    // Keep looping to handle multiple signals until runner task completes
    let runner_future = async {
        if let Some(task) = runner_task {
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
                // Ignore feedback from our own kill_process_group
                if !self_initiated_shutdown {
                    kill_level += 1;

                    match kill_level {
                        1 => {
                            info!("Shutting down gracefully (SIGTERM)...");
                            info!("Press Ctrl-C again to force terminate");
                            self_initiated_shutdown = true;  // Mark that we're sending SIGTERM ourselves
                            kill_process_group(pgid, nix::sys::signal::Signal::SIGTERM);
                            let _ = shutdown_tx.send(true);
                            let _ = member_handle.shutdown();
                            // Continue looping to handle more signals
                        }
                        2 => {
                            warn!("Force terminating stubborn processes...");
                            warn!("Press Ctrl-C once more for immediate kill");
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
            Some(result) = background_tasks.next() => {
                debug!("A background service task has completed");
                match result {
                    Ok(Ok(())) => {
                        debug!("Background service task completed successfully");
                    }
                    Ok(Err(e)) => {
                        error!("Background service task failed: {:#}", e);
                    }
                    Err(e) => {
                        error!("Background service task panicked: {:#}", e);
                    }
                }
                // Continue loop - background task completion doesn't trigger shutdown
            }
        }
    }

    // Trigger shutdown for any remaining tasks
    debug!("Triggering shutdown for remaining tasks...");
    let _ = shutdown_tx.send(true);
    let _ = member_handle.shutdown();

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

    // Disable CleanupGuard after graceful shutdown completes
    cleanup_guard.disable();
}

/// Wait for completion with Windows signal handling (Ctrl-C)
#[cfg(not(unix))]
async fn wait_for_completion_windows(
    shutdown_tx: tokio::sync::watch::Sender<bool>,
    member_handle: std::sync::Arc<crate::member_actor::MemberHandle>,
    runner_task: Option<tokio::task::JoinHandle<eyre::Result<()>>>,
    mut background_tasks: FuturesUnordered<tokio::task::JoinHandle<eyre::Result<()>>>,
    cleanup_guard: &CleanupGuard,
) {
    use futures::stream::StreamExt;

    // Keep looping to handle multiple Ctrl-C until runner task completes
    let runner_future = async {
        if let Some(task) = runner_task {
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
                let _ = shutdown_tx.send(true);
                let _ = member_handle.shutdown();
                kill_all_descendants();
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
            Some(result) = background_tasks.next() => {
                debug!("A background service task has completed");
                match result {
                    Ok(Ok(())) => {
                        debug!("Background service task completed successfully");
                    }
                    Ok(Err(e)) => {
                        error!("Background service task failed: {:#}", e);
                    }
                    Err(e) => {
                        error!("Background service task panicked: {:#}", e);
                    }
                }
                // Continue loop - background task completion doesn't trigger shutdown
            }
        }
    }

    // Trigger shutdown for any remaining tasks
    debug!("Triggering shutdown for remaining tasks...");
    let _ = shutdown_tx.send(true);
    let _ = member_handle.shutdown();

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

    // Disable CleanupGuard after graceful shutdown completes
    cleanup_guard.disable();
}

/// Forward state events to SSE broadcaster and wait for all actors to complete
///
/// This combines the event forwarding and completion waiting into a single task.
/// Takes ownership of the runner (no Arc/Mutex needed!).
async fn forward_state_events_and_wait(
    mut runner: crate::member_actor::MemberRunner,
    broadcaster: std::sync::Arc<crate::web::StateEventBroadcaster>,
) -> eyre::Result<()> {
    tracing::debug!("Starting state event forwarding and completion waiting");

    // Forward events and wait for completion
    // The runner's wait loop already updates the state cache internally
    loop {
        match runner.next_state_event().await {
            Some(event) => {
                // Broadcast to all SSE subscribers
                broadcaster.broadcast(event).await;
            }
            None => {
                // No more events, actors are done
                tracing::debug!("State event forwarding ending (no more events)");
                break;
            }
        }
    }

    // Now wait for all actor tasks to complete (join them)
    // This is quick because next_state_event() already waited for them to finish
    runner.wait_for_completion().await
}
