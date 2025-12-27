//! Run command - execute a single ROS node

use crate::{
    cli,
    cli::config::load_runtime_config,
    execution::context::{prepare_node_contexts, NodeContextClasses},
    member_actor::{ActorConfig, MemberCoordinator},
    monitoring::resource_monitor::MonitorConfig,
    process::{
        kill_all_descendants, kill_process_group, spawn_anchor_process, GRACEFUL_SHUTDOWN_DONE,
    },
    ros::launch_dump::LaunchDump,
    util::{log_dir::create_log_dir, logging::is_verbose},
    web,
};
use futures::stream::{FuturesUnordered, StreamExt};
use std::{
    collections::{HashMap, HashSet},
    fs,
    path::PathBuf,
    sync::{atomic::Ordering, Arc, Mutex},
};
use tracing::{debug, error, info, warn};

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
    let result = runtime.block_on(run_direct(&launch_dump, &args.common, Some(pgid)));

    #[cfg(not(unix))]
    let result = runtime.block_on(run_direct(&launch_dump, &args.common, None));

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
    pgid: Option<i32>,
) -> eyre::Result<()> {
    debug!("Starting direct node execution");

    // Install cleanup guard
    let _cleanup_guard = CleanupGuard;
    debug!("CleanupGuard installed");

    // Create shutdown signal for graceful termination
    let (shutdown_tx, shutdown_rx) = tokio::sync::watch::channel(false);

    // Spawn killer task IMMEDIATELY (before any spawning) for responsive signal handling
    let shutdown_tx_clone = shutdown_tx.clone();
    let killer_task = tokio::spawn(async move {
        shutdown_killer_task(pgid, shutdown_tx_clone).await;
        Ok::<(), eyre::Report>(())
    });
    debug!("Shutdown killer task spawned, signal handlers active");

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

    // Create MemberCoordinator
    let coordinator = std::sync::Arc::new(tokio::sync::Mutex::new(MemberCoordinator::new()));

    debug!("Spawning {} node(s)", pure_node_contexts.len());

    // Spawn actors for each node
    for context in pure_node_contexts {
        // Extract directory name for log/member name
        let dir_name = context
            .output_dir
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("unknown");
        let member_name = format!("NODE '{}'", dir_name);

        // Create actor config
        let actor_config = ActorConfig {
            respawn_enabled: !common.disable_respawn && context.record.respawn.unwrap_or(false),
            respawn_delay: context.record.respawn_delay.unwrap_or(0.0),
            max_respawn_attempts: None,
            output_dir: context.output_dir.clone(),
            pgid,
        };

        // Spawn the actor
        coordinator.lock().await.spawn_regular_node(
            member_name,
            context,
            actor_config,
            Some(process_registry.clone()),
        )?;
    }

    debug!("All actors spawned successfully");

    // Setup web UI if requested (direct StateEvent streaming)
    let _web_ui_enabled = if common.web_ui {
        info!("Setting up web UI with direct StateEvent streaming...");

        // Create state event broadcaster for SSE clients
        let state_broadcaster = std::sync::Arc::new(web::StateEventBroadcaster::new());

        // Start web server
        let web_state = Arc::new(web::WebState::new(
            coordinator.clone(),
            log_dir.clone(),
            state_broadcaster.clone(),
        ));
        let addr = common.web_ui_addr.clone();
        let port = common.web_ui_port;
        let shutdown_rx = {
            let coord = coordinator.lock().await;
            coord.shutdown_rx()
        };

        // Log web UI URL before spawning (addr will be moved)
        info!("Web UI available at http://{}:{}", addr, port);

        // Spawn web server
        tokio::spawn(async move {
            if let Err(e) = web::run_server(web_state, &addr, port, shutdown_rx).await {
                error!("Web server error: {}", e);
            }
        });

        // Spawn task to forward state events from coordinator to broadcaster
        let coordinator_clone = coordinator.clone();
        tokio::spawn(async move {
            forward_state_events(coordinator_clone, state_broadcaster).await;
        });
        true
    } else {
        false
    };

    // Phase 6: Collect all background tasks into FuturesUnordered for unified lifecycle management
    debug!("Setting up FuturesUnordered for background tasks...");
    let mut background_tasks = FuturesUnordered::new();

    // Add killer task (always present)
    background_tasks.push(killer_task);

    // Add optional monitoring task
    if let Some(task) = monitor_task {
        background_tasks.push(task);
    }

    debug!(
        "Background tasks collection created ({} tasks)",
        background_tasks.len()
    );

    // Wait for either actors to complete OR any background task to finish/fail
    debug!("Waiting for completion (actors or background tasks)...");
    let coordinator_wait = async { coordinator.lock().await.wait_for_completion().await };

    tokio::select! {
        result = coordinator_wait => {
            info!("All actors completed");
            if let Err(e) = result {
                error!("Actor completion error: {:#}", e);
            }
        }
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
        }
    }

    // Trigger shutdown for any remaining tasks
    debug!("Triggering shutdown for remaining tasks...");
    let _ = shutdown_tx.send(true);

    // Wait for all remaining background tasks to finish
    debug!("Waiting for remaining background tasks to finish...");
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
    handle_shutdown_simple(pgid, coordinator).await
}

/// Forward state events from coordinator to SSE broadcaster
///
/// This task runs in the background and continuously forwards StateEvents
/// from the coordinator to the web UI broadcaster for SSE clients.
async fn forward_state_events(
    coordinator: std::sync::Arc<tokio::sync::Mutex<crate::member_actor::MemberCoordinator>>,
    broadcaster: std::sync::Arc<crate::web::StateEventBroadcaster>,
) {
    tracing::debug!("Starting state event forwarding task");

    loop {
        // Get next state event from coordinator
        let event = {
            let mut coord = coordinator.lock().await;
            coord.next_state_event().await
        };

        match event {
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

/// Background task that handles shutdown signals (3-stage shutdown)
///
/// This task responds immediately to Ctrl-C and SIGTERM signals without blocking.
/// It maintains shutdown state and escalates kill signals on repeated Ctrl-C presses.
async fn shutdown_killer_task(pgid: Option<i32>, shutdown_tx: tokio::sync::watch::Sender<bool>) {
    #[cfg(unix)]
    {
        use nix::sys::signal::Signal;

        let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
            .expect("Failed to register SIGTERM handler");
        let mut sigint = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())
            .expect("Failed to register SIGINT handler");

        let mut shutdown_level = 0u8;

        loop {
            tokio::select! {
                biased;

                _ = sigint.recv() => {
                    shutdown_level += 1;

                    // Broadcast shutdown on first Ctrl-C
                    if shutdown_level == 1 {
                        let _ = shutdown_tx.send(true);
                    }

                    match shutdown_level {
                        1 => {
                            eprintln!("Shutting down gracefully (SIGTERM)...");
                            eprintln!("Press Ctrl-C again to force terminate");
                            if let Some(pgid) = pgid {
                                kill_process_group(pgid, Signal::SIGTERM);
                            }
                        }
                        2 => {
                            eprintln!("Force terminating stubborn processes...");
                            eprintln!("Press Ctrl-C once more for immediate kill");
                            if let Some(pgid) = pgid {
                                kill_process_group(pgid, Signal::SIGTERM);
                            }
                        }
                        3 => {
                            eprintln!("Immediate kill! Sending SIGKILL");
                            if let Some(pgid) = pgid {
                                kill_process_group(pgid, Signal::SIGKILL);
                            }
                            break;
                        }
                        _ => {
                            eprintln!("Cleanup in progress, please wait...");
                        }
                    }
                }
                _ = sigterm.recv() => {
                    eprintln!("Received SIGTERM, performing immediate kill...");
                    let _ = shutdown_tx.send(true);
                    if let Some(pgid) = pgid {
                        kill_process_group(pgid, Signal::SIGTERM);
                        tokio::time::sleep(std::time::Duration::from_millis(200)).await;
                        kill_process_group(pgid, Signal::SIGKILL);
                    }
                    break;
                }
            }
        }
    }

    #[cfg(not(unix))]
    {
        // Windows: simple Ctrl-C handling
        if tokio::signal::ctrl_c().await.is_ok() {
            eprintln!("Received Ctrl-C, shutting down...");
            kill_all_descendants();
        }
    }
}

/// Wait for actors to complete with graceful shutdown
async fn handle_shutdown_simple(
    pgid: Option<i32>,
    coordinator: Arc<tokio::sync::Mutex<MemberCoordinator>>,
) -> eyre::Result<()> {
    // Wait for actors to complete (killer task handles signals in background)
    let result = {
        let mut coord = coordinator.lock().await;
        coord.wait_for_completion().await
    };

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
