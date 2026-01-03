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
struct CleanupGuard;

impl Drop for CleanupGuard {
    fn drop(&mut self) {
        debug!("CleanupGuard: Ensuring all child processes are terminated");
        kill_all_descendants();
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
    let _cleanup_guard = CleanupGuard;
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
        let (task, handle) =
            crate::ros::container_readiness::spawn_service_discovery_task(shutdown_signal.clone());

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

    // Initialize component loader for service-based loading (Phase 3: async task with shutdown)
    debug!("Starting component loader task...");
    let (component_loader_task, component_loader) =
        crate::ros::component_loader::spawn_component_loader_task(shutdown_signal.clone());
    let component_loader = Some(component_loader);
    let component_loader_task = Some(component_loader_task);
    debug!("Component loader task started successfully");

    // Create MemberCoordinator for actor-based execution
    debug!("Creating MemberCoordinator...");
    let coordinator = std::sync::Arc::new(tokio::sync::Mutex::new(
        crate::member_actor::MemberCoordinator::new(),
    ));

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

    // Spawn regular nodes
    debug!("Spawning {} regular nodes", num_pure_nodes);
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

        coordinator.lock().await.spawn_regular_node(
            member_name,
            context,
            actor_config,
            Some(process_registry.clone()),
        )?;
    }

    // Spawn containers and collect their state receivers
    debug!("Spawning {} containers", num_containers);
    let mut container_state_receivers = HashMap::new();
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

        // Spawn container and get its state receiver
        let container_state_rx = coordinator.lock().await.spawn_container(
            member_name.clone(),
            context.node_context,
            actor_config,
            Some(process_registry.clone()),
        )?;

        // Store the state receiver for composable nodes to use
        container_state_receivers.insert(context.node_container_name.clone(), container_state_rx);
    }

    // Group composable nodes by container and spawn them
    if let Some(ref loader) = component_loader {
        debug!("Spawning {} composable nodes", num_composable_nodes);

        let composable_config = crate::member_actor::ComposableActorConfig::default();

        for context in load_node_contexts {
            let member_name = format!("LOAD_NODE '{}'", context.record.node_name);

            // Find the container state receiver for this composable node
            if let Some(container_state_rx) =
                container_state_receivers.get(&context.record.target_container_name)
            {
                coordinator.lock().await.spawn_composable_node(
                    member_name,
                    context,
                    composable_config.clone(),
                    container_state_rx.clone(),
                    loader.clone(),
                )?;
            } else {
                warn!(
                    "Container '{}' not found for composable node '{}', skipping",
                    context.record.target_container_name, context.record.node_name
                );
            }
        }
    } else {
        warn!("Component loader not available, composable nodes will not be spawned");
    }

    // Setup web UI if requested (direct StateEvent streaming)
    let web_ui_task = if common.web_ui {
        info!("Setting up web UI with direct StateEvent streaming...");

        // Create state event broadcaster for SSE clients
        let state_broadcaster = std::sync::Arc::new(web::StateEventBroadcaster::new());

        // Start web server
        let web_state = std::sync::Arc::new(web::WebState::new(
            coordinator.clone(),
            log_dir.clone(),
            state_broadcaster.clone(),
        ));
        let addr = common.web_ui_addr.clone();
        let port = common.web_ui_port;
        let web_shutdown = shutdown_signal.clone();
        let forwarder_shutdown = shutdown_signal.clone();

        // Log web UI URL before spawning (addr will be moved)
        info!("Web UI available at http://{}:{}", addr, port);

        // Spawn web server and state forwarder as separate tasks
        // They don't need to complete together - each will exit when shutdown is triggered
        let web_server_task = tokio::spawn(async move {
            if let Err(e) = web::run_server(web_state, &addr, port, web_shutdown).await {
                error!("Web server error: {}", e);
            }
            Ok(())
        });

        let coordinator_clone = coordinator.clone();
        let state_forwarder_task = tokio::spawn(async move {
            forward_state_events(coordinator_clone, state_broadcaster, forwarder_shutdown).await;
            Ok(())
        });

        // Add both tasks to background_tasks instead of wrapping in a single task
        Some((web_server_task, state_forwarder_task))
    } else {
        None
    };

    // Phase 6: Collect all background tasks into FuturesUnordered for unified lifecycle management
    debug!("Setting up FuturesUnordered for background tasks...");
    let background_tasks = FuturesUnordered::new();

    // Add anchor task (always present)
    background_tasks.push(anchor_task);

    // Add optional tasks
    if let Some(task) = monitor_task {
        background_tasks.push(task);
    }
    if let Some(task) = service_discovery_task {
        background_tasks.push(task);
    }
    if let Some(task) = component_loader_task {
        background_tasks.push(task);
    }
    if let Some((web_server_task, state_forwarder_task)) = web_ui_task {
        background_tasks.push(web_server_task);
        background_tasks.push(state_forwarder_task);
    }

    debug!(
        "Background tasks collection created ({} tasks)",
        background_tasks.len()
    );

    // Wait for either actors to complete OR any background task to finish/fail OR signals
    debug!("Waiting for completion (actors, background tasks, or signals)...");

    #[cfg(unix)]
    wait_for_completion_unix(
        pgid,
        shutdown_tx.clone(),
        coordinator.clone(),
        background_tasks,
    )
    .await;

    #[cfg(not(unix))]
    wait_for_completion_windows(shutdown_tx.clone(), coordinator.clone(), background_tasks).await;

    info!("play() function completed, returning");
    Ok(())
}

/// Wait for completion with Unix signal handling (SIGINT/SIGTERM)
#[cfg(unix)]
async fn wait_for_completion_unix(
    pgid: i32,
    shutdown_tx: tokio::sync::watch::Sender<bool>,
    coordinator: Arc<tokio::sync::Mutex<crate::member_actor::MemberCoordinator>>,
    mut background_tasks: FuturesUnordered<tokio::task::JoinHandle<eyre::Result<()>>>,
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
    let mut coordinator_wait =
        std::pin::pin!(async { coordinator.lock().await.wait_for_completion().await });

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
                            self_initiated_shutdown = true;  // Mark that we're sending SIGTERM ourselves
                            kill_process_group(pgid, nix::sys::signal::Signal::SIGTERM);
                            let _ = shutdown_tx.send(true);
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

            // Actor completion
            result = &mut coordinator_wait => {
                info!("All actors completed");
                if let Err(e) = result {
                    error!("Actor completion error: {:#}", e);
                }
                break;  // Normal completion, exit loop
            }

            // Background task completion/failure
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

    // Trigger shutdown for any remaining tasks
    debug!("Triggering shutdown for remaining tasks...");
    let _ = shutdown_tx.send(true);

    // Wait for coordinator's actor tasks to complete (if not already completed)
    debug!("Waiting for coordinator actor tasks to complete...");
    if let Err(e) = coordinator_wait.await {
        warn!("Coordinator completion error during shutdown: {:#}", e);
    }
    debug!("Coordinator actor tasks completed");

    // Drain remaining background tasks (exits immediately if already empty)
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
}

/// Wait for completion with Windows signal handling (Ctrl-C)
#[cfg(not(unix))]
async fn wait_for_completion_windows(
    shutdown_tx: tokio::sync::watch::Sender<bool>,
    coordinator: Arc<tokio::sync::Mutex<crate::member_actor::MemberCoordinator>>,
    mut background_tasks: FuturesUnordered<tokio::task::JoinHandle<eyre::Result<()>>>,
) {
    let mut coordinator_wait =
        std::pin::pin!(async { coordinator.lock().await.wait_for_completion().await });

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

            // Actor completion
            result = &mut coordinator_wait => {
                info!("All actors completed");
                if let Err(e) = result {
                    error!("Actor completion error: {:#}", e);
                }
                break;  // Normal completion, exit loop
            }

            // Background task completion/failure
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

    // Trigger shutdown for any remaining tasks
    debug!("Triggering shutdown for remaining tasks...");
    let _ = shutdown_tx.send(true);

    // Wait for coordinator's actor tasks to complete (if not already completed)
    debug!("Waiting for coordinator actor tasks to complete...");
    if let Err(e) = coordinator_wait.await {
        warn!("Coordinator completion error during shutdown: {:#}", e);
    }
    debug!("Coordinator actor tasks completed");

    // Drain remaining background tasks (exits immediately if already empty)
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
}

/// Forward state events from coordinator to SSE broadcaster
///
/// This task runs in the background and continuously forwards StateEvents
/// from the coordinator to the web UI broadcaster for SSE clients.
async fn forward_state_events(
    coordinator: std::sync::Arc<tokio::sync::Mutex<crate::member_actor::MemberCoordinator>>,
    broadcaster: std::sync::Arc<crate::web::StateEventBroadcaster>,
    mut shutdown_rx: tokio::sync::watch::Receiver<bool>,
) {
    tracing::debug!("Starting state event forwarding task");

    loop {
        tokio::select! {
            biased;

            // Check shutdown signal first
            _ = shutdown_rx.changed() => {
                if *shutdown_rx.borrow() {
                    tracing::debug!("State event forwarding task received shutdown signal");
                    break;
                }
            }

            // Get next state event from coordinator
            event_result = async {
                let mut coord = coordinator.lock().await;
                coord.next_state_event().await
            } => {
                match event_result {
                    Some(event) => {
                        // Update coordinator's internal state cache
                        {
                            let mut coord = coordinator.lock().await;
                            coord.update_state(&event);
                        }

                        // Broadcast to all SSE subscribers
                        broadcaster.broadcast(event).await;
                    }
                    None => {
                        // Coordinator finished, no more events
                        tracing::debug!("State event forwarding task ending (coordinator finished)");
                        break;
                    }
                }
            }
        }
    }
}
