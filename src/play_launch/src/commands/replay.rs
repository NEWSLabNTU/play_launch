//! Replay command - replay recorded launch execution

use super::{
    common::{CleanupGuard, build_tokio_runtime, forward_state_events_and_wait},
    signal_handler::{self, CompletionContext},
};
use crate::{
    cli,
    cli::config::load_runtime_config,
    execution::context::{
        ComposableNodeContextSet, prepare_composable_node_contexts, prepare_container_contexts,
        prepare_node_contexts,
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

    // Phase 43.1 — model↔record binding gate: when a SystemModel is given,
    // the record we are about to run MUST be one of the model's hashed
    // inputs. The checked artifact and the running artifact stay the same
    // thing; a stale pair refuses instead of silently diverging.
    let system_model = match &args.model {
        Some(model_path) => Some(std::sync::Arc::new(verify_model_record_binding(
            model_path, input_file,
        )?)),
        None => None,
    };

    // Verify all required ROS 2 system packages are installed
    crate::ros::ament_index::check_system_deps()?;

    // Become a child subreaper so orphaned grandchildren get reparented to us
    // instead of PID 1. This ensures waitpid() catches exits of any descendant
    // even if the intermediate parent dies first.
    #[cfg(target_os = "linux")]
    {
        use libc::{PR_SET_CHILD_SUBREAPER, prctl};
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
    info!(
        "  Interception: {}",
        if runtime_config.interception.enabled {
            "enabled"
        } else {
            "disabled"
        }
    );

    // Note: ROS service discovery and anchor process now started in play() async function (Phase 2, 4)

    let runtime = build_tokio_runtime()?;
    runtime.block_on(play(input_file, &args.common, system_model))
}

/// Play the launch according to the launch record.
async fn play(
    input_file: &Path,
    common: &cli::options::CommonOptions,
    system_model: Option<std::sync::Arc<ros_launch_manifest_model::SystemModel>>,
) -> eyre::Result<()> {
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

    // Resolve contract sources (overlay > provider sidecar) and load
    // manifests. The provider channel is on by default, so this runs even
    // when no manifest flags are given — scopes simply have nothing to
    // load when no contract file exists in any channel.
    let contract_sources = common.contract_sources();
    let _manifest_index = {
        debug!(
            "Resolving contracts: overlay={:?}, provider={}",
            contract_sources.overlay, contract_sources.provider
        );
        let index = crate::ros::manifest_loader::load_manifests(&launch_dump, &contract_sources)?;
        if index.total_errors > 0 {
            warn!(
                "Manifest static checks found {} error(s) — contracts may not hold",
                index.total_errors
            );
        }
        Some(index)
    };

    // Phase 43.2 — the rule engine, blocking allowlist, and lifecycle
    // wiring read a source-agnostic ContractView: from the checked
    // SystemModel when `--model` was given, else from the manifest tree
    // loaded above. (The sched plan still reads the manifest index until
    // 43.3 lands.)
    let contract_view: Option<std::sync::Arc<crate::runtime_enforcement::ContractView>> =
        match &system_model {
            Some(m) => {
                info!("Contract source: SystemModel (checked at resolve time)");
                Some(std::sync::Arc::new(
                    crate::runtime_enforcement::ContractView::from_model(m),
                ))
            }
            None => _manifest_index.as_ref().map(|i| {
                std::sync::Arc::new(
                    crate::runtime_enforcement::ContractView::from_manifest_index(i),
                )
            }),
        };

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

    // Create metrics broadcaster (shared between monitoring task and web UI)
    let metrics_broadcaster: Option<Arc<web::SystemMetricsBroadcaster>> =
        if runtime_config.monitoring.enabled {
            Some(Arc::new(web::SystemMetricsBroadcaster::new()))
        } else {
            None
        };

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
            metrics_broadcaster.clone(),
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
        info!(
            "Service readiness checking disabled (set container_readiness.wait_for_service_ready: true in config to enable)"
        );
        None
    };

    // Prepare node and container execution contexts
    debug!("Preparing node execution contexts...");
    let mut pure_node_contexts = prepare_node_contexts(&launch_dump, &node_log_dir)?;

    debug!("Preparing container execution contexts...");
    let mut container_contexts =
        prepare_container_contexts(&launch_dump, &node_log_dir, common.container_mode)?;

    // Prepare LoadNode request execution contexts
    let ComposableNodeContextSet { load_node_contexts } =
        prepare_composable_node_contexts(&launch_dump, &load_node_log_dir)?;

    // Phase 36.7: write the topic-FQN allowlist file when
    // `--block-unauthorized-endpoints` is set. Children's
    // `rcl_publisher_init` and `rcl_subscription_init` hooks refuse
    // topics not in this file.
    // Safety gate: since 40.2 the manifest index is ALWAYS built (provider
    // channel is on by default), so `is_some()` no longer means "a contract
    // source was configured". An EMPTY index must not produce an empty
    // allowlist — that would block every rcl endpoint in every child. Blocking
    // enforcement only engages when at least one authorized topic exists.
    let has_authorized_topics = contract_view.as_ref().is_some_and(|v| !v.is_empty());
    let allowlist_file: Option<std::path::PathBuf> = if common.block_unauthorized_endpoints
        && has_authorized_topics
    {
        let view = contract_view.as_ref().unwrap();
        let path = log_dir.join("expected_graph.txt");
        let mut contents = String::with_capacity(view.topics.len() * 32);
        contents.push_str("# Phase 36.7 allowlist — every topic FQN authorized in this launch\n");
        for fqn in view.topics.keys() {
            contents.push_str(fqn);
            contents.push('\n');
        }
        for fqn in &view.externals {
            contents.push_str(fqn);
            contents.push('\n');
        }
        std::fs::write(&path, contents).wrap_err("write allowlist file")?;
        info!(
            "Blocking enforcement: allowlist written to {} ({} topics + {} externals)",
            path.display(),
            view.topics.len(),
            view.externals.len()
        );
        Some(path)
    } else {
        if common.block_unauthorized_endpoints {
            warn!(
                "--block-unauthorized-endpoints requested but no contract declares any \
                     topic (no overlay or provider contract resolved) — blocking \
                     enforcement DISABLED; an empty allowlist would block every endpoint"
            );
        }
        None
    };

    // Setup interception if enabled (Phase 29)
    let mut interception_consumers: Vec<crate::interception::ChildConsumer> = Vec::new();
    let _interception_so_path = if runtime_config.interception.enabled {
        match crate::interception::find_interception_so() {
            Some(so_path) => {
                info!("Interception enabled (so: {})", so_path.display());
                // Inject LD_PRELOAD + fd env vars into each pure node
                for ctx in &mut pure_node_contexts {
                    match crate::interception::setup_child_interception(
                        &mut ctx.cmdline.env,
                        &so_path,
                        &runtime_config.interception,
                        allowlist_file.as_deref(),
                    ) {
                        Ok(consumer) => interception_consumers.push(consumer),
                        Err(e) => warn!("Interception setup failed for node: {:#}", e),
                    }
                }
                // Inject into each container
                for ctx in &mut container_contexts {
                    match crate::interception::setup_child_interception(
                        &mut ctx.node_context.cmdline.env,
                        &so_path,
                        &runtime_config.interception,
                        allowlist_file.as_deref(),
                    ) {
                        Ok(consumer) => interception_consumers.push(consumer),
                        Err(e) => warn!("Interception setup failed for container: {:#}", e),
                    }
                }
                debug!(
                    "Interception: {} consumers created",
                    interception_consumers.len()
                );
                Some(so_path)
            }
            None => {
                warn!(
                    "Interception enabled but libplay_launch_interception.so not found — disabling"
                );
                None
            }
        }
    } else {
        None
    };

    // Phase 38: resolve the scheduling spec (if any) once, before spawning any
    // member. Phase 38.10: prefer delegating to the RT helper (works
    // unprivileged given `play_launch setcap`); only fall back to the
    // root-or-capped-helper privilege check. Not being root is no longer
    // grounds to hard-fail — that's the entire point of this wave.
    // Resolve the platform file through the shipping channels (Phase 41.3):
    // explicit `--sched <path>` > overlay > provider sidecar. Reuses the
    // overlay root already discovered for contracts above, so both channels
    // agree on which root is in play for this invocation.
    // Phase 43.3 — when a SystemModel with an execution layer was given,
    // the plan comes from it directly (derive/override/band-validate ran at
    // resolve time); the platform-file channels are only consulted on the
    // legacy path.
    let model_plan: Option<crate::execution::sched_plan::SchedPlan> = match &system_model {
        Some(m) if !m.execution.is_empty() => {
            info!("Scheduling source: SystemModel execution layer");
            Some(crate::execution::sched_plan::SchedPlan::from_model(
                m,
                &common.target,
                common.sched_apply,
            )?)
        }
        Some(_) => {
            info!("SystemModel carries no execution layer — scheduling disabled");
            None
        }
        None => None,
    };
    let resolved_sched = if system_model.is_some() {
        None
    } else {
        crate::ros::sched_loader::resolve_platform_file(
            &launch_dump,
            common.sched.as_deref(),
            contract_sources.overlay.as_deref(),
            contract_sources.provider,
            &common.target,
        )
    };
    let legacy_plan = if let Some(resolved) = &resolved_sched {
        Some(crate::execution::sched_plan::SchedPlan::build(
            &launch_dump,
            _manifest_index.as_ref(),
            &resolved.path,
            &common.target,
            common.sched_apply,
        )?)
    } else {
        None
    };
    let (sched_plan, sched_helper, sched_helper_join) = if let Some(plan) =
        model_plan.or(legacy_plan)
    {
        let (sched_helper, sched_helper_join) = if common.sched_apply
            != crate::execution::sched_apply::SchedApplyMode::Off
        {
            // Privilege is about whether the apply can SUCCEED, which is
            // independent of whether the helper process starts: a helper
            // that spawns fine but lacks CAP_SYS_NICE is useless (every
            // apply would EPERM). So check first, then spawn.
            let privileged = crate::execution::sched_apply::has_sched_privilege()
                || crate::commands::capabilities::rt_helper_has_cap_sys_nice();
            if !privileged {
                let msg = "scheduling: no privilege to apply; run `play_launch setcap` (grants cap_sys_nice to play_launch_rt_helper) or run as root";
                if common.sched_apply == crate::execution::sched_apply::SchedApplyMode::Strict {
                    eyre::bail!("{msg}");
                }
                tracing::warn!("{msg}; scheduling will not be applied");
            }

            match crate::execution::rt_helper_client::SchedHelper::spawn().await {
                Ok((helper, join)) => (Some(helper), Some(join)),
                Err(e) => {
                    // Without the helper, only root can apply (direct
                    // fallback). If privilege depended on the capped
                    // helper, a spawn failure means scheduling CANNOT be
                    // applied — Strict must abort at this boundary, not
                    // degrade into per-node EPERMs mid-run.
                    if crate::execution::sched_apply::has_sched_privilege() {
                        tracing::debug!("RT helper unavailable ({e:#}); applying directly as root");
                    } else {
                        let msg = format!(
                            "scheduling: RT helper failed to start ({e:#}); cannot apply without root"
                        );
                        if common.sched_apply
                            == crate::execution::sched_apply::SchedApplyMode::Strict
                        {
                            eyre::bail!("{msg}");
                        }
                        tracing::warn!("{msg}; scheduling will not be applied");
                    }
                    (None, None)
                }
            }
        } else {
            (None, None)
        };

        (
            Some(std::sync::Arc::new(plan)),
            sched_helper,
            sched_helper_join,
        )
    } else {
        (None, None, None)
    };

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
        // Use node name, falling back to exec_name (many Autoware nodes have name=null)
        let member_name = context
            .record
            .name
            .as_ref()
            .or(context.record.exec_name.as_ref())
            .cloned()
            .unwrap_or_else(|| "unknown".to_string());

        let actor_config = crate::member_actor::ActorConfig {
            respawn_enabled: !common.disable_respawn && context.record.respawn.unwrap_or(false),
            respawn_delay: context.record.respawn_delay.unwrap_or(0.0),
            max_respawn_attempts: None,
            output_dir: context.output_dir.clone(),
            pgid: Some(pgid),
            sched: sched_plan.as_ref().and_then(|p| {
                let fqn = crate::ros::sched_loader::fqn_for(
                    &launch_dump,
                    context.record.namespace.as_deref(),
                    &member_name,
                    context.record.scope,
                );
                p.for_fqn(&fqn).cloned()
            }),
            sched_mode: common.sched_apply,
            sched_helper: sched_helper.clone(),
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
            sched: sched_plan.as_ref().and_then(|p| {
                let fqn = crate::ros::sched_loader::fqn_for(
                    &launch_dump,
                    context.node_context.record.namespace.as_deref(),
                    &member_name,
                    context.node_context.record.scope,
                );
                p.for_fqn(&fqn).cloned()
            }),
            sched_mode: common.sched_apply,
            sched_helper: sched_helper.clone(),
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

        // Phase 38.9: resolve this composable's tier at the builder (Option B) —
        // no FQN recompute at apply time, no C++/Rust FQN-format dependency.
        let composable_tier = sched_plan.as_ref().and_then(|p| {
            let fqn = crate::ros::sched_loader::fqn_for(
                &launch_dump,
                Some(&context.record.namespace),
                &context.record.node_name,
                context.record.scope,
            );
            p.for_fqn(&fqn).cloned()
        });

        // Auto-load enabled by default for all composable nodes
        builder.add_composable_node(member_name, context, true, composable_tier);
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
        // Build node→scope mapping from launch dump.
        // Keys must match the member names used by the actor system:
        //   nodes: name.or(exec_name)  (see context.rs line 153)
        //   containers: name.or(exec_name)
        //   composable nodes: node_name
        let mut node_scope_map = std::collections::HashMap::new();
        for n in &launch_dump.node {
            if let Some(scope) = n.scope {
                let member_name = n.name.as_ref().or(n.exec_name.as_ref());
                if let Some(name) = member_name {
                    node_scope_map.insert(name.clone(), scope);
                }
            }
        }
        for c in &launch_dump.container {
            if let Some(scope) = c.scope {
                // Container member name is c.name (matching actor system in context.rs)
                let member_name: &str = &c.name;
                node_scope_map.insert(member_name.to_string(), scope);
            }
        }
        for ln in &launch_dump.load_node {
            if let Some(scope) = ln.scope {
                node_scope_map.insert(ln.node_name.clone(), scope);
            }
        }

        let web_state = std::sync::Arc::new(web::WebState::new(
            member_handle.clone(),
            log_dir.clone(),
            state_broadcaster.clone(),
            diagnostic_registry.clone(),
            metrics_broadcaster.clone(),
            launch_dump.scopes.clone(),
            node_scope_map,
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
                                            );
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

    // Spawn interception consumer task if we have consumers (Phase 29)
    if !interception_consumers.is_empty() {
        // Phase 36.3: construct RuleEngine if a contract source resolved
        // and --enforce-rules is not Off. The engine observes every
        // event the listener dispatches.
        let rule_engine = match (contract_view.as_ref(), common.enforce_rules) {
            (Some(view), mode) if !matches!(mode, crate::cli::options::EnforceMode::Off) => Some(
                crate::runtime_enforcement::RuleEngine::new(view.clone(), mode, &log_dir),
            ),
            _ => None,
        };

        // Phase 36.6.1: wire lifecycle transition-event subscriptions
        // for every lifecycle node declared in the manifest tree.
        // Updates flow through an unbounded mpsc channel into the
        // interception listener, which applies them to the RuleEngine
        // before each event-drain pass.
        let (lifecycle_tx, lifecycle_rx) = tokio::sync::mpsc::unbounded_channel::<(
            String,
            crate::runtime_enforcement::LifecycleState,
        )>();
        let lifecycle_node_fqns: Vec<String> = rule_engine
            .as_ref()
            .map(|re| re.lifecycle_node_fqns())
            .unwrap_or_default();
        // Subscription handles are stored as opaque `Box<dyn Any>` so
        // they live for the duration of replay without leaking the
        // generic-parameter list into this scope.
        let mut _lifecycle_subs: Vec<Box<dyn std::any::Any + Send + Sync>> = Vec::new();
        if !lifecycle_node_fqns.is_empty()
            && let Some(ros_node) = shared_ros_node_for_params.as_ref()
        {
            for fqn in &lifecycle_node_fqns {
                let topic = format!("{}/transition_event", fqn.trim_end_matches('/'));
                let fqn_owned = fqn.clone();
                let tx = lifecycle_tx.clone();
                match ros_node.create_subscription(
                    topic.as_str(),
                    move |msg: lifecycle_msgs::msg::TransitionEvent| {
                        // goal_state.id matches lifecycle_msgs/msg/State::PRIMARY_STATE_*.
                        let st = match msg.goal_state.id {
                            1 => crate::runtime_enforcement::LifecycleState::Unconfigured,
                            2 => crate::runtime_enforcement::LifecycleState::Inactive,
                            3 => crate::runtime_enforcement::LifecycleState::Active,
                            4 => crate::runtime_enforcement::LifecycleState::Finalized,
                            _ => crate::runtime_enforcement::LifecycleState::Unknown,
                        };
                        let _ = tx.send((fqn_owned.clone(), st));
                    },
                ) {
                    Ok(sub) => {
                        debug!("Subscribed to {} for lifecycle gating", topic);
                        _lifecycle_subs.push(Box::new(sub));
                    }
                    Err(e) => warn!(
                        "Failed to subscribe to {} for lifecycle gating: {:#}",
                        topic, e
                    ),
                }
            }
        }
        drop(lifecycle_tx); // keep only senders inside callbacks alive

        // Phase 36.4: in Strict mode, spawn a watcher task that polls
        // the engine's `strict_violated` flag and triggers shutdown on
        // first hit. The flag is also flipped under Warn mode but only
        // Strict listens for it.
        let strict_handle = rule_engine.as_ref().and_then(|re| {
            matches!(
                common.enforce_rules,
                crate::cli::options::EnforceMode::Strict
            )
            .then(|| re.strict_violated_handle())
        });
        if let Some(handle) = strict_handle {
            let shutdown_tx_strict = shutdown_tx.clone();
            let strict_watch_task = tokio::spawn(async move {
                let poll_interval = tokio::time::Duration::from_millis(100);
                loop {
                    tokio::time::sleep(poll_interval).await;
                    if handle.load(std::sync::atomic::Ordering::Acquire) {
                        warn!("[runtime] Strict enforcement violated — initiating shutdown");
                        let _ = shutdown_tx_strict.send(true);
                        break;
                    }
                }
                Ok::<(), eyre::Error>(())
            });
            named_tasks.push(NamedTask {
                name: "runtime_strict_watch",
                task: strict_watch_task,
            });
        }

        let interception_task = tokio::spawn(crate::interception::run_interception_task(
            interception_consumers,
            log_dir.clone(),
            runtime_config.interception.clone(),
            shutdown_signal.clone(),
            rule_engine,
            Some(lifecycle_rx),
        ));
        named_tasks.push(NamedTask {
            name: "interception",
            task: interception_task,
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

    // Phase 38.10: all actors have completed (and with them, every ActorConfig
    // holding a SchedHelper clone). Drop our own clone so the owner task sees
    // the mpsc channel close and shuts the RT helper down gracefully; bound
    // the wait so a wedged owner task can't hang shutdown.
    if let Some(join) = sched_helper_join {
        drop(sched_helper);
        match tokio::time::timeout(std::time::Duration::from_secs(5), join).await {
            Ok(Ok(())) => debug!("RT helper owner task shut down cleanly"),
            Ok(Err(e)) => warn!("RT helper owner task panicked: {:#}", e),
            Err(_) => warn!("RT helper owner task did not shut down within timeout"),
        }
    }

    debug!("play() function completed, returning");
    Ok(())
}

/// Phase 43.1 — refuse a (model, record) pair that doesn't match: the
/// record's sha256 must appear among the model's `meta.inputs` hashes
/// (`resolve` hashes the record companion / --record file it consumed).
fn verify_model_record_binding(
    model_path: &std::path::Path,
    record_path: &std::path::Path,
) -> eyre::Result<ros_launch_manifest_model::SystemModel> {
    use sha2::Digest as _;

    let yaml = std::fs::read_to_string(model_path)
        .wrap_err_with(|| format!("reading SystemModel {}", model_path.display()))?;
    let model = ros_launch_manifest_model::SystemModel::from_yaml_str(&yaml)
        .map_err(|e| eyre::eyre!("loading SystemModel {}: {e}", model_path.display()))?;
    let bytes = std::fs::read(record_path)
        .wrap_err_with(|| format!("reading record {}", record_path.display()))?;
    let digest = format!("{:x}", sha2::Sha256::digest(&bytes));

    let Some(bound) = &model.meta.record else {
        eyre::bail!(
            "SystemModel {} carries no bound record (resolved to stdout?) — \
             re-run `play_launch resolve -o <file>` so a record companion is \
             emitted and bound.",
            model_path.display(),
        );
    };
    if bound.sha256 == digest {
        info!(
            "SystemModel {} binds record {} (sha256 {})",
            model_path.display(),
            record_path.display(),
            &digest[..12],
        );
        return Ok(model);
    }
    eyre::bail!(
        "record {} (sha256 {digest}) does not match the record bound to \
         SystemModel {} ({} sha256 {}) — the pair is stale. Re-run \
         `play_launch resolve`, or pass the record companion emitted next \
         to the model.",
        record_path.display(),
        model_path.display(),
        bound.path,
        bound.sha256,
    );
}
