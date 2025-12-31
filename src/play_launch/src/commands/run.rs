//! Run command - execute a single ROS node

use crate::{
    cli,
    cli::config::load_runtime_config,
    event_driven::{
        event_processor::EventProcessor,
        events::EventBus,
        member::{NodeLogPaths, ProcessState, RegularNode},
        process_monitor::ProcessMonitor,
        registry::Registry,
    },
    execution::{
        context::{prepare_node_contexts, NodeContextClasses},
        spawn::spawn_nodes_event_driven,
    },
    monitoring::resource_monitor::{spawn_monitor_thread, MonitorConfig},
    process::{
        kill_all_descendants, kill_process_group, spawn_anchor_process, GRACEFUL_SHUTDOWN_DONE,
    },
    ros::{container_readiness::SERVICE_DISCOVERY_HANDLE, launch_dump::LaunchDump},
    util::{log_dir::create_log_dir, logging::is_verbose},
    web,
};
use std::{
    collections::{HashMap, HashSet},
    fs,
    path::PathBuf,
    sync::{
        atomic::{AtomicUsize, Ordering},
        Arc, Mutex,
    },
};
use tracing::{debug, error, info};

/// CleanupGuard ensures all child processes are killed when dropped
struct CleanupGuard;

impl Drop for CleanupGuard {
    fn drop(&mut self) {
        debug!("CleanupGuard: Ensuring all child processes are terminated");
        // Only kill if we didn't already gracefully shut down
        if !GRACEFUL_SHUTDOWN_DONE.load(Ordering::Relaxed) {
            kill_all_descendants();
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
        remaps: vec![],
        ros_args: None,
        args: Some(args.args.clone()),
        cmd,
        env: None,
        respawn: None,
        respawn_delay: None,
        global_params: None,
    };

    let launch_dump = LaunchDump {
        node: vec![node_record],
        load_node: vec![],
        container: vec![],
        lifecycle_node: vec![],
        file_data: HashMap::new(),
    };

    // Spawn anchor zombie process to allocate PGID
    #[cfg(unix)]
    let (mut anchor, pgid) = spawn_anchor_process()?;
    #[cfg(unix)]
    debug!("Anchor process allocated PGID: {}", pgid);

    // Build the async runtime and run directly
    let runtime = Runtime::new()?;
    debug!(
        "Tokio runtime created (default config uses {} worker threads = num CPUs)",
        num_cpus::get()
    );

    #[cfg(unix)]
    let result = runtime.block_on(run_direct(&launch_dump, &args.common, pgid));

    #[cfg(not(unix))]
    let result = runtime.block_on(run_direct(&launch_dump, &args.common, 0));

    // Kill anchor process
    #[cfg(unix)]
    {
        let _ = anchor.kill();
        debug!("Anchor process killed");
    }

    result
}

async fn run_direct(
    launch_dump: &LaunchDump,
    common: &cli::options::CommonOptions,
    pgid: i32,
) -> eyre::Result<()> {
    info!("=== Starting direct node execution ===");

    // Install cleanup guard
    let _cleanup_guard = CleanupGuard;
    info!("CleanupGuard installed");

    // Load runtime configuration
    info!("Loading runtime configuration...");
    let runtime_config = load_runtime_config(
        common.config.as_deref(),
        common.enable_monitoring,
        common.monitor_interval_ms,
    )?;
    info!("Runtime configuration loaded successfully");

    // Create temporary log directory
    info!("Creating log directories...");
    let log_dir = create_log_dir(&common.log_dir)?;
    info!("Log directory created: {}", log_dir.display());

    let node_log_dir = log_dir.join("node");
    fs::create_dir(&node_log_dir)?;
    info!("Created node log directory");

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

    // Initialize monitoring
    let process_registry = Arc::new(Mutex::new(HashMap::<u32, PathBuf>::new()));
    debug!("Process registry initialized (empty)");
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
                debug!("Monitoring thread handle created successfully");
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

    // Prepare node execution contexts
    let container_names = HashSet::new();
    let NodeContextClasses {
        container_contexts: _,
        non_container_node_contexts: pure_node_contexts,
    } = prepare_node_contexts(launch_dump, &node_log_dir, &container_names)?;

    // Create shutdown signal for graceful respawn termination
    let (shutdown_tx, shutdown_rx) = tokio::sync::watch::channel(false);
    let shutdown_signal = shutdown_rx;

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

    // No component loader for run command (no containers/composable nodes)
    let component_loader: Option<crate::ros::component_loader::ComponentLoaderHandle> = None;

    // Populate registry with members from contexts
    {
        let mut reg = member_registry.lock().await;

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

    info!("Spawning 1 node");

    // Use event-driven execution (always)
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
            debug!("Node spawned successfully");
        }
        Err(e) => {
            error!("Failed to spawn nodes: {}", e);
            return Err(e);
        }
    }

    // Wait for shutdown signal
    // Progressive shutdown: 1st Ctrl-C sends SIGTERM, 2nd sends SIGTERM again, 3rd sends SIGKILL
    #[cfg(unix)]
    let result = {
        use nix::sys::signal::Signal;

        let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
            .expect("Failed to register SIGTERM handler");
        let mut sigint = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())
            .expect("Failed to register SIGINT handler");

        debug!("Signal handlers registered");

        let shutdown_level = Arc::new(AtomicUsize::new(0));

        loop {
            tokio::select! {
                _ = sigint.recv() => {
                    let level = shutdown_level.fetch_add(1, Ordering::SeqCst) + 1;
                    info!("Received SIGINT (shutdown level {})", level);

                    match level {
                        1 => {
                            info!("Shutting down gracefully (SIGTERM)...");
                            let _ = shutdown_tx.send(true);
                            kill_process_group(pgid, Signal::SIGTERM);
                            info!("Press Ctrl-C again to force terminate");
                        }
                        2 => {
                            info!("Force terminating stubborn processes (SIGTERM)...");
                            kill_process_group(pgid, Signal::SIGTERM);
                            info!("Press Ctrl-C once more for immediate kill");
                        }
                        3 => {
                            info!("Immediate kill! Sending SIGKILL to process group");
                            kill_process_group(pgid, Signal::SIGKILL);
                            info!("All child processes terminated");
                            break Ok(());
                        }
                        _ => {
                            // Level 4+: Ignore additional Ctrl-C presses, cleanup is in progress
                            info!("Cleanup in progress, please wait...");
                        }
                    }
                }
                _ = sigterm.recv() => {
                    info!("Received SIGTERM, performing immediate kill...");
                    let _ = shutdown_tx.send(true);
                    kill_process_group(pgid, Signal::SIGTERM);
                    tokio::time::sleep(std::time::Duration::from_millis(200)).await;
                    kill_process_group(pgid, Signal::SIGKILL);
                    info!("All child processes terminated");
                    break Ok(());
                }
                _ = async {
                    // Wait for shutdown signal to be set to true
                    let mut shutdown_watch = shutdown_signal.clone();
                    loop {
                        if *shutdown_watch.borrow() {
                            break;
                        }
                        if shutdown_watch.changed().await.is_err() {
                            break;
                        }
                    }
                } => {
                    info!("Shutdown signal received, waiting for processes to exit...");
                    // SIGTERM was already sent by the SIGINT handler
                    // Wait up to 5 seconds for graceful shutdown
                    let start = std::time::Instant::now();
                    let mut all_exited_gracefully = false;

                    while start.elapsed() < std::time::Duration::from_secs(5) {
                        // Check if any processes are still running
                        let has_running = nix::sys::wait::waitpid(
                            nix::unistd::Pid::from_raw(-pgid),
                            Some(nix::sys::wait::WaitPidFlag::WNOHANG)
                        );
                        match has_running {
                            Ok(nix::sys::wait::WaitStatus::Exited(_, _)) |
                            Ok(nix::sys::wait::WaitStatus::Signaled(_, _, _)) => {
                                // A child exited, keep waiting for others
                                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                            }
                            Err(nix::errno::Errno::ECHILD) => {
                                // No more children
                                info!("All processes exited gracefully");
                                all_exited_gracefully = true;
                                GRACEFUL_SHUTDOWN_DONE.store(true, Ordering::Relaxed);
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
                    info!("All child processes terminated");
                    break Ok(());
                }
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
                        break;
                    }
                    if shutdown_watch.changed().await.is_err() {
                        break;
                    }
                }
            } => {
                info!("Shutdown signal received, waiting for processes to exit...");
                kill_all_descendants();
                info!("All child processes terminated");
                Ok(())
            }
        }
    };

    result
}
