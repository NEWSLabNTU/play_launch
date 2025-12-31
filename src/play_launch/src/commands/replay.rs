//! Replay command - replay recorded launch execution

use crate::{
    cli,
    cli::config::load_runtime_config,
    event_driven::{
        event_processor::EventProcessor,
        events::EventBus,
        member::{
            ComposableNode, ComposableState, Container, NodeLogPaths, ProcessState, RegularNode,
        },
        process_monitor::ProcessMonitor,
        registry::Registry,
    },
    execution::{
        context::{
            prepare_composable_node_contexts, prepare_node_contexts, ComposableNodeContextSet,
            NodeContextClasses,
        },
        spawn::spawn_nodes_event_driven,
    },
    monitoring::resource_monitor::{spawn_monitor_thread, MonitorConfig},
    process::{
        kill_all_descendants, kill_process_group, spawn_anchor_process, GRACEFUL_SHUTDOWN_DONE,
    },
    ros::{
        container_readiness::SERVICE_DISCOVERY_HANDLE,
        launch_dump::{load_launch_dump, NodeContainerRecord},
    },
    util::{log_dir::create_log_dir, logging::is_verbose},
    web,
};
use eyre::Context;
use rayon::prelude::*;
use std::{
    collections::{HashMap, HashSet},
    fs,
    path::{Path, PathBuf},
    sync::{atomic::Ordering, Arc, Mutex},
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

    // Start ROS service discovery thread if service checking is enabled (default: true)
    if runtime_config.container_readiness.wait_for_service_ready {
        match crate::ros::container_readiness::start_service_discovery_thread() {
            Ok(handle) => {
                SERVICE_DISCOVERY_HANDLE
                    .set(handle)
                    .expect("SERVICE_DISCOVERY_HANDLE already set");
                debug!("ROS service discovery thread started successfully");
            }
            Err(e) => {
                error!("Failed to start ROS service discovery thread: {}", e);
                error!("Falling back to process-based container checking");
            }
        }
    } else {
        info!("Service readiness checking disabled (set container_readiness.wait_for_service_ready: true in config to enable)");
    }

    // Spawn anchor zombie process to allocate PGID
    #[cfg(unix)]
    let (mut anchor, pgid) = spawn_anchor_process()?;
    #[cfg(unix)]
    debug!("Anchor process allocated PGID: {}", pgid);

    // Build the async runtime.
    let runtime = Runtime::new()?;

    // Run the whole playing task in the runtime.
    #[cfg(unix)]
    let result = runtime.block_on(play(input_file, &args.common, pgid));

    #[cfg(not(unix))]
    let result = runtime.block_on(play(input_file, &args.common, 0));

    // Kill anchor process
    #[cfg(unix)]
    {
        let _ = anchor.kill();
        debug!("Anchor process killed");
    }

    result
}

/// Play the launch according to the launch record.
async fn play(
    input_file: &Path,
    common: &cli::options::CommonOptions,
    pgid: i32,
) -> eyre::Result<()> {
    debug!("=== Starting play() function ===");

    // Install cleanup guard to ensure children are killed even if we're interrupted
    let _cleanup_guard = CleanupGuard;
    debug!("CleanupGuard installed");

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

    // Initialize monitoring if enabled
    debug!("Initializing monitoring and process registry...");
    let process_registry = Arc::new(Mutex::new(HashMap::<u32, PathBuf>::new()));
    let _monitor_thread = if runtime_config.monitoring.enabled {
        let monitor_config = MonitorConfig {
            enabled: true,
            sample_interval_ms: runtime_config.monitoring.sample_interval_ms,
        };
        match spawn_monitor_thread(
            monitor_config,
            log_dir.clone(),
            process_registry.clone(),
            nvml,
        ) {
            Ok(handle) => {
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
                Some(handle)
            }
            Err(e) => {
                error!("Failed to start monitoring thread: {}", e);
                None
            }
        }
    } else {
        debug!("Resource monitoring disabled");
        None
    };
    debug!("Monitoring initialization complete");

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

    // Create shutdown signal for graceful termination (shared with web server and respawn)
    let (shutdown_tx, shutdown_rx) = tokio::sync::watch::channel(false);
    let shutdown_signal = shutdown_rx;

    // Initialize component loader for service-based loading (before web server for component loader access)
    debug!("Initializing component loader for service-based node loading");
    let component_loader = match crate::ros::component_loader::start_component_loader_thread() {
        Ok(loader) => {
            debug!("Component loader initialized successfully");
            Some(loader)
        }
        Err(e) => {
            warn!("Failed to start component loader: {}. Composable node reloading will not be available.", e);
            None
        }
    };

    // Create event-driven infrastructure (always enabled)
    debug!("Setting up event-driven architecture...");

    // Create EventBus (returns bus + receiver)
    let (event_bus, event_rx) = EventBus::new();

    // Create ProcessMonitor
    let process_monitor = Arc::new(ProcessMonitor::new(
        event_bus.clone(),
        shutdown_signal.clone(),
    ));

    // Create MemberRegistry
    let member_registry = Arc::new(tokio::sync::Mutex::new(Registry::new(log_dir.clone())));

    // Populate registry with members from contexts
    {
        let mut reg = member_registry.lock().await;

        // Register containers
        for context in &container_contexts {
            // Use the same log_name format as ExecutionContext
            let dir_name = context
                .node_context
                .output_dir
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("unknown");
            let log_name = format!("NODE '{}'", dir_name);

            let container = Container {
                name: log_name,
                record: context.node_context.record.clone(),
                cmdline: context.node_context.cmdline.clone(),
                output_dir: context.node_context.output_dir.clone(),
                log_paths: NodeLogPaths::from_output_dir(&context.node_context.output_dir),
                composable_nodes: Vec::new(), // Will be populated when composable nodes are loaded
                state: ProcessState::Pending,
                respawn_enabled: context.node_context.record.respawn.unwrap_or(false),
                respawn_delay: context.node_context.record.respawn_delay.unwrap_or(0.0),
            };
            reg.register_container(container);
        }

        // Register regular nodes
        for context in &pure_node_contexts {
            // Use the same log_name format as ExecutionContext
            let dir_name = context
                .output_dir
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("unknown");
            let log_name = format!("NODE '{}'", dir_name);

            let node = RegularNode {
                name: log_name,
                record: context.record.clone(),
                cmdline: context.cmdline.clone(),
                output_dir: context.output_dir.clone(),
                log_paths: NodeLogPaths::from_output_dir(&context.output_dir),
                state: ProcessState::Pending,
                respawn_enabled: context.record.respawn.unwrap_or(false),
                respawn_delay: context.record.respawn_delay.unwrap_or(0.0),
            };
            reg.register_node(node);
        }

        // Register composable nodes
        for context in &load_node_contexts {
            // Find the container log name for this composable node
            // The target_container_name from the record needs to be converted to the log name format
            let container_log_name = container_contexts
                .iter()
                .find(|c| {
                    let container_name = c
                        .node_context
                        .record
                        .name
                        .as_ref()
                        .unwrap_or(&c.node_container_name);
                    container_name == &context.record.target_container_name
                        || c.node_container_name == context.record.target_container_name
                })
                .and_then(|c| {
                    c.node_context
                        .output_dir
                        .file_name()
                        .and_then(|n| n.to_str())
                        .map(|dir_name| format!("NODE '{}'", dir_name))
                })
                .unwrap_or_else(|| {
                    // Fallback: if not found, use the original target_container_name
                    warn!(
                        "Container not found for composable node {}, using original name",
                        context.record.node_name
                    );
                    context.record.target_container_name.clone()
                });

            let composable = ComposableNode {
                name: context.record.node_name.clone(),
                record: context.record.clone(),
                output_dir: context.output_dir.clone(),
                log_paths: NodeLogPaths::from_output_dir(&context.output_dir),
                container_name: container_log_name,
                state: ComposableState::Unloaded,
            };
            reg.register_composable_node(composable);
        }

        debug!("Member registry created with {} members", reg.len());
    }

    // Get service discovery handle if available
    let service_discovery_handle = SERVICE_DISCOVERY_HANDLE.get().cloned();

    // Start EventProcessor in background
    let event_processor = EventProcessor::new(
        member_registry.clone(),
        event_rx,
        event_bus.clone(),
        process_monitor.clone(),
        shutdown_tx.clone(),
        shutdown_signal.clone(),
        Some(process_registry.clone()),
        Some(pgid),
        component_loader.clone(),
        service_discovery_handle,
    );

    tokio::spawn(async move {
        event_processor.run().await;
        debug!("EventProcessor shut down");
    });

    // Start web server if --web-ui flag is enabled
    if common.web_ui {
        let web_state = std::sync::Arc::new(web::WebState::new(
            member_registry.clone(),
            event_bus.clone(),
            log_dir.clone(),
        ));
        let addr = common.web_ui_addr.clone();
        let port = common.web_ui_port;
        let web_shutdown = shutdown_signal.clone();

        // Spawn web server as a background task
        tokio::spawn(async move {
            if let Err(e) = web::run_server(web_state, &addr, port, web_shutdown).await {
                error!("Web server error: {}", e);
            }
        });
    }

    // === EVENT-DRIVEN EXECUTION PATH ===

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

    // Spawn non-container nodes using event-driven spawn
    match spawn_nodes_event_driven(
        pure_node_contexts,
        &process_monitor,
        &event_bus,
        Some(process_registry.clone()),
        Some(pgid),
    )
    .await
    {
        Ok(_pids) => {
            debug!("Spawned {} pure nodes", _pids.len());
        }
        Err(e) => {
            error!("Failed to spawn pure nodes: {}", e);
            return Err(e);
        }
    }

    // Spawn containers using event-driven spawn
    debug!("Spawning {} containers...", container_contexts.len());
    match crate::execution::spawn::spawn_containers_event_driven(
        container_contexts,
        &process_monitor,
        &event_bus,
        Some(process_registry.clone()),
        Some(pgid),
    )
    .await
    {
        Ok(pids) => {
            debug!("Spawned {} containers with PIDs: {:?}", pids.len(), pids);
        }
        Err(e) => {
            error!("Failed to spawn containers: {}", e);
            return Err(e);
        }
    }

    // Composable nodes will be loaded automatically when containers start
    // via ProcessStarted event → LoadRequested events
    info!("Composable nodes will be loaded automatically");

    // In event-driven mode, ProcessMonitor handles waiting for processes
    // We just need to wait for shutdown signal
    info!("Waiting for shutdown signal...");

    #[cfg(unix)]
    let result = {
        use nix::sys::signal::Signal;

        let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
            .expect("Failed to register SIGTERM handler");
        let mut sigint = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())
            .expect("Failed to register SIGINT handler");

        debug!("Signal handlers registered");

        tokio::select! {
            _ = sigint.recv() => {
                info!("Received SIGINT (Ctrl-C), shutting down gracefully...");
                let _ = shutdown_tx.send(true);
                kill_process_group(pgid, Signal::SIGTERM);

                // Wait for graceful shutdown (5-second grace period)
                let start = std::time::Instant::now();
                let mut all_exited_gracefully = false;

                while start.elapsed() < std::time::Duration::from_secs(5) {
                    // Check if all processes have exited
                    let has_running = nix::sys::wait::waitpid(
                        Some(nix::unistd::Pid::from_raw(-pgid)),
                        Some(nix::sys::wait::WaitPidFlag::WNOHANG),
                    );

                    match has_running {
                        Err(nix::errno::Errno::ECHILD) => {
                            // No more children
                            info!("All processes exited gracefully");
                            all_exited_gracefully = true;
                            GRACEFUL_SHUTDOWN_DONE.store(true, Ordering::Relaxed);
                            break;
                        }
                        Ok(nix::sys::wait::WaitStatus::Exited(_, _)) |
                        Ok(nix::sys::wait::WaitStatus::Signaled(_, _, _)) => {
                            // At least one child exited, continue checking for others
                            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                        }
                        Ok(nix::sys::wait::WaitStatus::StillAlive) | Ok(_) => {
                            // Children still running
                            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                        }
                        Err(e) => {
                            warn!("Error checking process status: {}", e);
                            break;
                        }
                    }
                }

                // Force kill any remaining processes only if grace period expired and processes didn't exit
                if !all_exited_gracefully && start.elapsed() >= std::time::Duration::from_secs(5) {
                    info!("Grace period expired, force killing remaining processes...");
                    kill_process_group(pgid, Signal::SIGKILL);
                }

                // Exit with 130 (128+2) to indicate termination by SIGINT
                std::process::exit(130);
            }
            _ = sigterm.recv() => {
                info!("Received SIGTERM, shutting down gracefully...");
                let _ = shutdown_tx.send(true);
                kill_process_group(pgid, Signal::SIGTERM);
                tokio::time::sleep(std::time::Duration::from_secs(5)).await;
                kill_process_group(pgid, Signal::SIGKILL);
                // Exit with 143 (128+15) to indicate termination by SIGTERM
                std::process::exit(143);
            }
            _ = async {
                // Wait for shutdown signal to be set to true
                let mut shutdown_watch = shutdown_signal.clone();
                loop {
                    if *shutdown_watch.borrow() {
                        info!("Shutdown signal received");
                        break;
                    }
                    if shutdown_watch.changed().await.is_err() {
                        info!("Shutdown channel closed");
                        break;
                    }
                }
            } => {
                info!("Execution complete");
                Ok(())
            }
        }
    };

    #[cfg(not(unix))]
    let result = {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                info!("Received SIGINT (Ctrl-C), shutting down gracefully...");
                let _ = shutdown_tx.send(true);
                kill_all_descendants();
                info!("All child processes terminated");
                // Exit with 130 (128+2) to indicate termination by SIGINT
                std::process::exit(130);
            }
            _ = async {
                // Wait for shutdown signal to be set to true
                let mut shutdown_watch = shutdown_signal.clone();
                loop {
                    if *shutdown_watch.borrow() {
                        info!("Shutdown signal received");
                        break;
                    }
                    if shutdown_watch.changed().await.is_err() {
                        info!("Shutdown channel closed");
                        break;
                    }
                }
            } => {
                info!("Execution complete");
                Ok(())
            }
        }
    };

    result
}
