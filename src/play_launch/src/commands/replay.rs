//! Replay command - replay recorded launch execution

use super::common::{build_tokio_runtime, forward_state_events_and_wait, CleanupGuard};
use crate::{
    cli,
    cli::config::load_runtime_config,
    execution::context::{
        prepare_composable_node_contexts, prepare_container_contexts, prepare_node_contexts,
        ComposableNodeContextSet,
    },
    monitoring::resource_monitor::MonitorConfig,
    process::kill_process_group,
    ros::{container_readiness::SERVICE_DISCOVERY_HANDLE, launch_dump::load_launch_dump},
    util::{log_dir::create_log_dir, logging::is_verbose},
    web,
};
use eyre::Context;
use futures::stream::FuturesUnordered;
use std::{
    collections::HashMap,
    fs,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
};
use tracing::{debug, error, info, warn};

/// ROS executor spin timeout — how long to wait for work before checking shutdown signal
const EXECUTOR_SPIN_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(50);

/// Signal debounce window — ignore signals within this time after sending our own kill
const SIGNAL_DEBOUNCE: std::time::Duration = std::time::Duration::from_millis(200);

/// Timeout for draining background tasks during cleanup
const CLEANUP_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(2);

/// Interval for printing startup progress statistics
const PROGRESS_INTERVAL: std::time::Duration = std::time::Duration::from_secs(10);

/// Interval for checking if all nodes have finished loading
const COMPLETION_CHECK_INTERVAL: std::time::Duration = std::time::Duration::from_millis(100);

pub fn handle_replay(args: &cli::options::ReplayArgs) -> eyre::Result<()> {
    let input_file = &args.input_file;

    // Become a child subreaper so orphaned grandchildren get reparented to us
    // instead of PID 1. This ensures waitpid() catches exits of any descendant
    // even if the intermediate parent dies first.
    #[cfg(target_os = "linux")]
    {
        use libc::{prctl, PR_SET_CHILD_SUBREAPER};
        if unsafe { prctl(PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0) } != 0 {
            warn!(
                "Failed to set PR_SET_CHILD_SUBREAPER: {}",
                std::io::Error::last_os_error()
            );
        }
    }

    // Load runtime configuration to check service readiness settings
    let runtime_config = cli::config::load_runtime_config(
        args.common.config.as_deref(),
        args.common.is_monitoring_enabled(),
        args.common.monitor_interval_ms,
        args.common.is_diagnostics_enabled(),
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
    if args.common.is_web_ui_enabled() {
        let (addr, port) = args.common.parse_web_addr()?;
        info!("  Web UI: http://{}:{}", addr, port);
    }

    // Note: ROS service discovery and anchor process now started in play() async function (Phase 2, 4)

    let runtime = build_tokio_runtime()?;
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
        common.is_monitoring_enabled(),
        common.monitor_interval_ms,
        common.is_diagnostics_enabled(),
    )?;
    debug!("Runtime configuration loaded successfully");

    debug!("Loading launch dump from: {}", input_file.display());
    let launch_dump = load_launch_dump(input_file)?;
    debug!(
        "Launch dump loaded: {} nodes, {} containers, {} composable nodes",
        launch_dump.node.len(),
        launch_dump.container.len(),
        launch_dump.load_node.len()
    );

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

    // Phase 12: Composable nodes are now virtual members, no separate log directories needed
    let load_node_log_dir = log_dir.join("load_node");
    // Note: Directory not created since composable nodes don't have separate process logs
    debug!("Skipped load_node log directory (Phase 12: virtual members)");

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

                // Spin with timeout (waits up to 50ms for work, processes it, then returns)
                // This allows ROS to process messages during the wait, unlike sleep()
                executor.spin(rclrs::SpinOptions::new().timeout(EXECUTOR_SPIN_TIMEOUT));
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

    // Initialize diagnostic monitoring if enabled
    debug!("Initializing diagnostic monitoring...");
    let diagnostic_registry = Arc::new(crate::diagnostics::DiagnosticRegistry::new());
    let _diagnostic_task = if runtime_config.diagnostics.enabled && shared_ros_node.is_some() {
        if is_verbose() {
            info!(
                "Diagnostic monitoring enabled (topics: {:?})",
                runtime_config.diagnostics.topics
            );
        } else {
            debug!(
                "Diagnostic monitoring enabled (topics: {:?})",
                runtime_config.diagnostics.topics
            );
        }

        // Spawn async diagnostic monitoring task
        let task = tokio::spawn(crate::diagnostics::run_diagnostic_task(
            runtime_config.diagnostics.clone(),
            shared_ros_node.clone().unwrap(),
            log_dir.clone(),
            diagnostic_registry.clone(),
            shutdown_signal.clone(),
        ));

        Some(task)
    } else {
        if runtime_config.diagnostics.enabled && shared_ros_node.is_none() {
            warn!("Diagnostic monitoring enabled but ROS node unavailable - diagnostics disabled");
        } else {
            debug!("Diagnostic monitoring disabled");
        }
        None
    };
    debug!("Diagnostic monitoring initialization complete");

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

    // Prepare node and container execution contexts
    debug!("Preparing node execution contexts...");
    let pure_node_contexts = prepare_node_contexts(&launch_dump, &node_log_dir)?;

    debug!("Preparing container execution contexts...");
    let container_contexts =
        prepare_container_contexts(&launch_dump, &node_log_dir, common.container_mode)?;

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
        // Use the node name from record.json directly
        let member_name = context
            .record
            .name
            .as_ref()
            .cloned()
            .unwrap_or_else(|| "unknown".to_string());

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
        // Use the container name from record.json directly
        let member_name = context
            .node_context
            .record
            .name
            .as_ref()
            .cloned()
            .unwrap_or_else(|| "unknown".to_string());

        let actor_config = crate::member_actor::ActorConfig {
            respawn_enabled: !common.disable_respawn
                && context.node_context.record.respawn.unwrap_or(false),
            respawn_delay: context.node_context.record.respawn_delay.unwrap_or(0.0),
            max_respawn_attempts: None,
            output_dir: context.node_context.output_dir.clone(),
            pgid: Some(pgid),
        };

        // Add container (oneshot receiver is ignored since composable nodes will be matched internally)
        let use_component_events =
            common.container_mode != crate::cli::options::ContainerMode::Stock;
        std::mem::drop(builder.add_container(
            member_name.clone(),
            context.node_context,
            actor_config,
            Some(process_registry.clone()),
            use_component_events,
        ));
    }

    // Add composable nodes (builder will match them with containers during spawn())
    // Phase 12: Composable nodes are managed by containers as virtual members
    debug!("Adding {} composable nodes", num_composable_nodes);

    for context in load_node_contexts {
        // Use the composable node name from record.json directly
        let member_name = context.record.node_name.clone();
        // Auto-load enabled by default for all composable nodes
        builder.add_composable_node(member_name, context, true);
    }

    // Now spawn all actors at once and get handle + runner
    // Pass the shared ROS node for container actors to use
    debug!("Spawning all {} actors...", builder.member_count());
    let (member_handle, member_runner) = builder.spawn(shared_ros_node).await;
    let member_handle = std::sync::Arc::new(member_handle); // Wrap in Arc for sharing
    debug!("All actors spawned successfully");

    // Setup periodic statistics output task (runs every 10 seconds)
    let stats_task = {
        let stats_handle = member_handle.clone();
        let stats_shutdown = shutdown_signal.clone();
        tokio::spawn(async move {
            print_periodic_statistics(stats_handle, stats_shutdown).await;
            Ok(())
        })
    };

    // Setup web UI if requested (direct StateEvent streaming)
    let (runner_task, web_ui_task) = if common.is_web_ui_enabled() {
        debug!("Setting up web UI with direct StateEvent streaming...");

        // Parse web address
        let (addr, port) = common.parse_web_addr()?;

        // Create state event broadcaster for SSE clients
        let state_broadcaster = std::sync::Arc::new(web::StateEventBroadcaster::new());

        // Start web server
        let web_state = std::sync::Arc::new(web::WebState::new(
            member_handle.clone(), // Clone the Arc, not MemberHandle
            log_dir.clone(),
            state_broadcaster.clone(),
            diagnostic_registry.clone(),
        ));
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

    // Use a custom struct to track task names
    struct NamedTask {
        name: &'static str,
        task: tokio::task::JoinHandle<eyre::Result<()>>,
    }

    let mut named_tasks = Vec::new();

    // Add anchor task (always present)
    named_tasks.push(NamedTask {
        name: "anchor",
        task: anchor_task,
    });

    // Add stats task (always present - exits on its own when startup completes)
    named_tasks.push(NamedTask {
        name: "stats",
        task: stats_task,
    });

    // Add optional service tasks (these can complete without triggering shutdown)
    if let Some(task) = monitor_task {
        named_tasks.push(NamedTask {
            name: "monitor",
            task,
        });
    }
    if let Some(task) = service_discovery_task {
        named_tasks.push(NamedTask {
            name: "service_discovery",
            task,
        });
    }
    if let Some(task) = web_ui_task {
        named_tasks.push(NamedTask {
            name: "web_ui",
            task,
        });
    }

    let total_tasks = named_tasks.len();
    let task_names: Vec<&'static str> = named_tasks.iter().map(|t| t.name).collect();

    debug!(
        "Background service tasks collection created ({} tasks)",
        total_tasks
    );

    // Convert to FuturesUnordered with task names embedded in results
    let background_tasks = FuturesUnordered::new();
    for named in named_tasks {
        let name = named.name;
        let task = named.task;
        background_tasks.push(async move {
            let result = task.await;
            (name, result)
        });
    }

    // Wait for either actors to complete (runner task) OR signals
    debug!("Waiting for completion (runner task or signals)...");

    let ctx = CompletionContext {
        shutdown_tx: shutdown_tx.clone(),
        member_handle,
        runner_task,
        total_tasks,
    };

    #[cfg(unix)]
    wait_for_completion_unix(
        pgid,
        ctx,
        background_tasks,
        task_names.clone(),
        &cleanup_guard,
    )
    .await;

    #[cfg(not(unix))]
    wait_for_completion_windows(ctx, background_tasks, task_names, &cleanup_guard).await;

    debug!("play() function completed, returning");
    Ok(())
}

/// Context for completion waiting
struct CompletionContext {
    shutdown_tx: tokio::sync::watch::Sender<bool>,
    member_handle: std::sync::Arc<crate::member_actor::MemberHandle>,
    runner_task: Option<tokio::task::JoinHandle<eyre::Result<()>>>,
    total_tasks: usize,
}

/// Wait for completion with Unix signal handling (SIGINT/SIGTERM)
#[cfg(unix)]
async fn wait_for_completion_unix<F>(
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
async fn wait_for_completion_windows<F>(
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
                let remaining = ctx.total_tasks - tasks_completed.len();
                warn!("Ctrl-C during cleanup - force killing all processes ({}/{} tasks still pending)", remaining, ctx.total_tasks);
                warn!("Stubborn tasks: {:?}", tasks_pending);
                crate::process::kill_all_descendants();
                std::process::exit(130);
            }

            // Drain background tasks
            Some((task_name, result)) = background_tasks.next() => {
                tasks_completed.push(task_name);
                tasks_pending.retain(|&name| name != task_name);

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
async fn print_periodic_statistics(
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
