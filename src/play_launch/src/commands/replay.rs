//! Replay command - replay recorded launch execution

use super::{
    common::{build_tokio_runtime, forward_state_events_and_wait, CleanupGuard},
    signal_handler::{self, CompletionContext},
};
use crate::{
    cli,
    cli::config::load_runtime_config,
    execution::context::{
        prepare_composable_node_contexts, prepare_container_contexts, prepare_node_contexts,
        ComposableNodeContextSet,
    },
    monitoring::resource_monitor::MonitorConfig,
    ros::{container_readiness::SERVICE_DISCOVERY_HANDLE, launch_dump::load_launch_dump},
    util::{log_dir::create_log_dir, logging::is_verbose},
    web,
};
use eyre::Context;
use futures::stream::FuturesUnordered;
use rclrs::IntoPrimitiveOptions;
use std::{
    collections::HashMap,
    fs,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
};
use tracing::{debug, error, info, warn};

/// ROS executor spin timeout — how long to wait for work before checking shutdown signal
const EXECUTOR_SPIN_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(50);

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

        // Detach the executor thread — dropping a JoinHandle does not join.
        // The thread will run until the shutdown signal is received.
        drop(executor_thread);

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
    // Clone before spawning — we need it for parameter_events subscription (Phase 24)
    let shared_ros_node_for_params = shared_ros_node.clone();
    debug!("Spawning all {} actors...", builder.member_count());
    let (member_handle, member_runner) = builder.spawn(shared_ros_node).await;
    let member_handle = std::sync::Arc::new(member_handle); // Wrap in Arc for sharing
    debug!("All actors spawned successfully");

    // Setup periodic statistics output task (runs every 10 seconds)
    let stats_task = {
        let stats_handle = member_handle.clone();
        let stats_shutdown = shutdown_signal.clone();
        tokio::spawn(async move {
            signal_handler::print_periodic_statistics(stats_handle, stats_shutdown).await;
            Ok(())
        })
    };

    // Setup web UI if requested (direct StateEvent streaming)
    let (runner_task, web_ui_task, param_events_task) = if common.is_web_ui_enabled() {
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

        // Phase 24: Spawn /parameter_events subscriber task
        let param_task = if let Some(ref ros_node) = shared_ros_node_for_params {
            let (param_event_tx, mut param_event_rx) =
                tokio::sync::mpsc::unbounded_channel::<rcl_interfaces::msg::ParameterEvent>();

            // Create subscription on the shared ROS node (callback runs on executor thread)
            match ros_node.create_subscription(
                "/parameter_events".reliable().keep_last(1000),
                move |msg: rcl_interfaces::msg::ParameterEvent| {
                    let _ = param_event_tx.send(msg);
                },
            ) {
                Ok(param_sub) => {
                    let param_shutdown = shutdown_signal.clone();
                    let param_fqn_map = member_handle.node_fqn_map().clone();
                    let param_broadcaster = state_broadcaster.clone();

                    Some(tokio::spawn(async move {
                        let _sub = param_sub;
                        loop {
                            tokio::select! {
                                Some(event) = param_event_rx.recv() => {
                                    let fqn_map = param_fqn_map.read().await;
                                    let member_name = fqn_map
                                        .iter()
                                        .find(|(_, fqn)| *fqn == &event.node)
                                        .map(|(name, _)| name.clone());
                                    drop(fqn_map);

                                    if let Some(name) = member_name {
                                        let mut updates = Vec::new();
                                        for p in event.new_parameters.iter().chain(event.changed_parameters.iter()) {
                                            updates.push(crate::ros::parameter_types::ParamUpdate {
                                                param_name: p.name.clone(),
                                                value: crate::ros::parameter_types::ParamValue::from_ros(&p.value),
                                            });
                                        }
                                        if !updates.is_empty() {
                                            param_broadcaster.broadcast(
                                                crate::member_actor::events::StateEvent::ParameterChanged {
                                                    name,
                                                    parameters: updates,
                                                },
                                            ).await;
                                        }
                                    }
                                }
                                _ = async {
                                    let mut rx = param_shutdown.clone();
                                    loop {
                                        if *rx.borrow() { break; }
                                        if rx.changed().await.is_err() { break; }
                                    }
                                } => break,
                                else => break,
                            }
                        }
                        Ok(())
                    }))
                }
                Err(e) => {
                    warn!("Failed to create /parameter_events subscription: {:#}", e);
                    None
                }
            }
        } else {
            None
        };

        // Runner task forwards state events and waits for completion
        let node_fqn_map = member_handle.node_fqn_map().clone();
        let runner_task = tokio::spawn(async move {
            forward_state_events_and_wait(member_runner, state_broadcaster, node_fqn_map).await
        });

        (Some(runner_task), Some(web_server_task), param_task)
    } else {
        // Runner task just waits for completion (no forwarding)
        let runner_task = tokio::spawn(async move {
            tracing::debug!("Runner task: starting, about to call wait_for_completion");
            let result = member_runner.wait_for_completion().await;
            tracing::debug!("Runner task: wait_for_completion returned: {:?}", result);
            result
        });

        (Some(runner_task), None, None)
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
    if let Some(task) = param_events_task {
        named_tasks.push(NamedTask {
            name: "parameter_events",
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
    signal_handler::wait_for_completion_unix(
        pgid,
        ctx,
        background_tasks,
        task_names.clone(),
        &cleanup_guard,
    )
    .await;

    #[cfg(not(unix))]
    signal_handler::wait_for_completion_windows(ctx, background_tasks, task_names, &cleanup_guard)
        .await;

    debug!("play() function completed, returning");
    Ok(())
}
