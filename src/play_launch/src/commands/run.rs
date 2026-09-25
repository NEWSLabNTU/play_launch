//! Run command - execute a single ROS node

use super::common::{CleanupGuard, build_tokio_runtime, forward_state_events_and_wait};
use crate::{
    cli,
    cli::config::load_runtime_config,
    execution::context::prepare_node_contexts,
    member_actor::{ActorConfig, MemberCoordinatorBuilder},
    monitoring::resource_monitor::MonitorConfig,
    process::kill_process_group,
    util::{log_dir::create_log_dir, logging::is_verbose},
    web,
};
use eyre::WrapErr;
use futures::stream::{FuturesUnordered, StreamExt};
use ros_launch_resolve::ros::launch_dump::LaunchDump;
use std::{
    collections::HashMap,
    fs,
    path::PathBuf,
    sync::{Arc, Mutex},
};
use tracing::{debug, error, info, warn};

/// Time to wait for processes to exit gracefully before SIGKILL
const GRACEFUL_SHUTDOWN_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(5);

/// Interval for polling child process status during shutdown
const SHUTDOWN_POLL_INTERVAL: std::time::Duration = std::time::Duration::from_millis(100);

/// Issue #0045 — `--enforce-rules` used to be accepted here and do nothing.
///
/// The flag lives on `CommonOptions`, so every verb parses it, but only the
/// launch-tree verbs can act on it: a contract is LOCATED by launch file
/// (`<overlay>/<pkg>/launch/<stem>.contract.yaml`, or the sidecar beside the
/// launch file — `manifest_loader::ContractChannel`), and `run` names a
/// package and an executable, so no channel has a key to look one up with.
/// Same structural fact `run --check` already prints out loud.
///
/// So this is a refusal rather than a wiring job: with no contract there is
/// no `ContractView`, hence no rule with anything to say, and a strict run
/// that measures nothing would exit 0 for the wrong reason — the shape phase
/// 79 spent three waves removing from `launch`/`up`.
///
/// Only an EXPLICIT mode is refused. `enforce_rules` defaults to `Warn`, so
/// refusing the value alone would reject every plain `play_launch run`.
fn refuse_unsupported_enforcement(args: &cli::options::RunArgs) -> eyre::Result<()> {
    let mode = args.common.contract_opts.enforce_rules;
    if !enforcement_is_refused(mode, enforce_rules_given_on_argv(mode)) {
        if !matches!(mode, cli::options::EnforceMode::Off) {
            debug!(
                "--enforce-rules {mode:?} (default): `run` resolves no contract, so no runtime \
                 rule is evaluated"
            );
        }
        return Ok(());
    }
    Err(eyre::eyre!("{}", enforcement_refusal(mode)))
}

/// The predicate, apart from where its inputs come from.
fn enforcement_is_refused(mode: cli::options::EnforceMode, explicit: bool) -> bool {
    explicit && !matches!(mode, cli::options::EnforceMode::Off)
}

/// Did the user type `--enforce-rules`, or is this clap's default?
///
/// Asked of clap rather than of `std::env::args()` directly, because `run`'s
/// node arguments are `trailing_var_arg`: in `run pkg exe -- --enforce-rules
/// strict` those two tokens belong to the NODE, and a textual scan would
/// refuse a command that never addressed play_launch at all. (Without the
/// `--`, clap takes the flag for itself — measured, not assumed.) Re-parsing
/// the same argv the process already parsed successfully is cheap and gives
/// the exact answer.
fn enforce_rules_given_on_argv(mode: cli::options::EnforceMode) -> bool {
    use clap::{CommandFactory, parser::ValueSource};

    match cli::options::Options::command().try_get_matches_from(std::env::args_os()) {
        Ok(matches) => matches
            .subcommand_matches("run")
            .and_then(|m| m.value_source("enforce_rules"))
            .is_some_and(|source| source != ValueSource::DefaultValue),
        // Unreachable in practice (this argv already parsed once). Fall back
        // to the one thing that needs no parser: a value that is not the
        // default cannot have come from anywhere but the command line. Erring
        // this way keeps a plain `run` working, which erring the other way
        // would not.
        Err(_) => mode != cli::options::EnforceMode::Warn,
    }
}

fn enforcement_refusal(mode: cli::options::EnforceMode) -> String {
    use clap::ValueEnum;
    // The mode as the user must SPELL it (`record-only`, not `RecordOnly`),
    // taken from clap so a rename cannot leave this message behind.
    let spelling = mode
        .to_possible_value()
        .map(|v| v.get_name().to_string())
        .unwrap_or_else(|| format!("{mode:?}").to_lowercase());
    format!(
        "--enforce-rules {} is not available on `run`: a contract is located by launch file \
         (<pkg>/launch/<stem>.contract.yaml, in the --contracts overlay or shipped beside the \
         launch file) and `run` names a package and an executable, so no contract can resolve, \
         no rule engine is built and no runtime rule can fire. A run that measures nothing must \
         not report a pass.\n\
         \n\
         To enforce contracts on this node, put it in a one-node launch file and use \
         `play_launch launch` (or `resolve` then `up`), which resolve contracts and enforce \
         them. To run it unenforced, pass `--enforce-rules off`.",
        spelling,
    )
}

pub fn handle_run(args: &cli::options::RunArgs) -> eyre::Result<()> {
    use ros_launch_resolve::ros::launch_dump::{LaunchDump, NodeRecord};

    // Before anything is created: a refusal must leave no half-written
    // bundle and no moved `latest` (issue #0023), the same reason phase 79
    // judges its own precondition ahead of `create_log_dir`.
    refuse_unsupported_enforcement(args)?;

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
        // `play_launch run` builds one node by hand; there is no launch file
        // and so no on_exit handler to honour.
        on_exit_shutdown: None,
        executable: args.executable.clone(),
        package: Some(args.package.clone()),
        name: Some(args.executable.clone()),
        namespace: Some("/".to_string()),
        exec_name: Some(args.executable.clone()),
        params: vec![],
        params_files: vec![],
        param_sources: Vec::new(),
        global_params: None,
        remaps: vec![],
        env: None,
        ros_args: None,
        args: Some(args.args.clone()),
        cmd,
        respawn: args.common.containers.disable_respawn.then_some(false),
        respawn_delay: Some(0.0),
        // No launch file, so no `<timer>` to have enclosed this node.
        start_delay_secs: None,
        scope: None,
    };

    let launch_dump = LaunchDump {
        node: vec![node_record],
        load_node: vec![],
        container: vec![],
        lifecycle_node: vec![],
        file_data: HashMap::new(),
        variables: HashMap::new(),
        scopes: Vec::new(),
        // `run` builds its dump by hand from the CLI; there is no launch file
        // for the parser to have dropped an action from.
        dropped_actions: Vec::new(),
    };

    let runtime = build_tokio_runtime()?;
    runtime.block_on(run_direct(&launch_dump, &args.common, args.check))
}

async fn run_direct(
    launch_dump: &LaunchDump,
    common: &cli::options::CommonOptions,
    check: bool,
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
    let mut runtime_config = load_runtime_config(
        common.config.as_deref(),
        common.is_monitoring_enabled(),
        common.features.monitor_interval_ms,
        common.is_diagnostics_enabled(),
    )?;
    info!("Runtime configuration loaded successfully");

    // Issue #0053 — `--interception on` used to be accepted here and do
    // nothing: `run` read only the scheduling half of `runtime_config`, so no
    // ring was created, no `LD_PRELOAD` was injected and the bundle carried no
    // `interception/` directory at all. Unlike `--enforce-rules` (issue #0045,
    // refused above) there is no obstacle: the hooks live in whatever process
    // is spawned, and everything they write is keyed by node and topic rather
    // than by launch scope, so a single-node run can be measured.
    if let Some(switch) = common.contract_opts.interception {
        runtime_config.interception.enabled = Some(switch.is_on());
    }
    // `decide` takes the enforcement mode because on `launch`/`up` a non-`off`
    // mode IMPLIES interception — the hooks are the rule engine's only event
    // source (issue #0031). `run` builds no rule engine and refuses every
    // explicit non-`off` mode, so nothing here can imply anything: the mode it
    // decides against is structurally `Off`, leaving `--interception on|off`
    // and `interception.enabled` as the only inputs. Passing the real mode
    // instead would turn interception on for every plain `play_launch run`, in
    // the name of an engine that is not built.
    let interception_decision = runtime_config
        .interception
        .decide(cli::options::EnforceMode::Off);
    match interception_decision {
        crate::cli::config::InterceptionDecision::Configured => info!(
            "Interception: enabled ({})",
            if common.contract_opts.interception.is_some() {
                "--interception on"
            } else {
                "interception.enabled: true in --config"
            }
        ),
        _ => debug!(
            "Interception: off (`run` enforces no contract, so only --interception on or \
             `interception.enabled: true` turns it on)"
        ),
    }
    runtime_config.interception.enabled = Some(interception_decision.enabled());

    // Create temporary log directory
    info!("Creating log directories...");
    let log_dir = create_log_dir(&common.log_dir)?;
    info!("Log directory created: {}", log_dir.display());
    // Issue #0023: the launcher's own log joins the bundle (see `up::play`).
    crate::util::run_log::attach_or_warn(
        &log_dir,
        crate::util::run_log::RunInfo::capture(common.config.as_deref()),
    );

    let node_log_dir = log_dir.join("node");
    fs::create_dir(&node_log_dir)?;
    info!("Created node log directory");

    // Create diagnostic registry (empty - diagnostic monitoring not supported
    // in 'run' mode, so nothing is ever stored and the staleness limit is moot)
    let diagnostic_registry = Arc::new(crate::diagnostics::DiagnosticRegistry::new(None));

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
            None, // No metrics broadcaster in run mode
            None, // `run` spawns a single node and builds no cgroup tree
        ));

        debug!("Monitoring task spawned successfully");
        Some(task)
    } else {
        debug!("Resource monitoring disabled");
        None
    };

    // Prepare node execution contexts
    let mut pure_node_contexts = prepare_node_contexts(launch_dump, &node_log_dir)?;

    // Phase 38: resolve the scheduling spec (if any) once, before spawning any
    // member. Phase 38.10: prefer delegating to the RT helper (works
    // unprivileged given `play_launch setcap`); only fall back to the
    // root-or-capped-helper privilege check. Not being root is no longer
    // grounds to hard-fail — that's the entire point of this wave.
    // Resolve the platform file through the shipping channels (Phase 41.3):
    // explicit `--sched <path>` > overlay > provider sidecar. `run` has no
    // contract-loading infrastructure of its own, but the overlay-root
    // discovery and the `--no-provider-contracts` gate
    // (`CommonOptions::contract_sources`) are shared with contract
    // resolution regardless.
    let sched_sources = common.contract_sources()?;
    let resolved_sched = ros_launch_resolve::ros::sched_loader::resolve_platform_file(
        launch_dump,
        common.sched_opts.sched.as_deref(),
        sched_sources.overlay.as_deref(),
        sched_sources.provider,
        &common.sched_opts.target,
    );

    if check {
        // Structural, not a shortcut: contracts are keyed by launch file and
        // `run` has none, so no sidecar can be located. Saying that out loud
        // is the whole point -- a silent skip here would exit 0 having
        // checked nothing checkable.
        println!(
            "no contracts checked: `run` has no launch file, so no contract \
             sidecar can apply."
        );
        match &resolved_sched {
            Some(resolved) => {
                // `resolve_platform_file` only LOCATES a path — on the
                // explicit `--sched <path>` branch it does not even stat it.
                // Printing "OK" off that alone is the vacuous pass this
                // check exists to prevent (design D6), so actually build the
                // plan, exactly as the non-check path below does. A parse or
                // validation failure propagates as an error and exits
                // non-zero, same as `run` without `--check`.
                let plan = crate::execution::sched_plan::SchedPlan::build(
                    launch_dump,
                    None,
                    &resolved.path,
                    &common.sched_opts.target,
                    common.sched_opts.sched_apply,
                )
                .wrap_err_with(|| {
                    format!("failed to parse platform file {}", resolved.path.display())
                })?;
                println!(
                    "Platform file: {} (target: {}) — OK, {} node assignment(s)",
                    resolved.path.display(),
                    common.sched_opts.target,
                    plan.assignment_count(),
                );
                return Ok(());
            }
            None => {
                println!(
                    "Platform file: none resolved (target: {}) — nothing to validate",
                    common.sched_opts.target
                );
                return Ok(());
            }
        }
    }

    // Issue #0053 — the observation half of `runtime_config`, wired here
    // rather than anywhere earlier so that `--check` (which returns above)
    // creates no ring and injects nothing.
    //
    // The per-child plumbing itself is not copied from `up`: the shared memory
    // ring, the fds and every env var belong to one function
    // (`interception::setup_child_interception`), and the two names a bundle is
    // keyed by come from `up::interception_identity_path` and
    // `up::interception_node_name`, lifted out rather than duplicated. What is
    // `run`-shaped is only the loop (one collection, never a container) and the
    // NAME: `run` builds its record by hand, so `model_fqn` is `None` and the
    // helper would fall back to the bare executable — while every consumer of
    // the bundle (`measure`, `contract capture`) joins on a node FQN. The
    // FQN comes from the same `fqn_for` the scheduling lookup below uses.
    let mut interception_consumers: Vec<crate::interception::ChildConsumer> = Vec::new();
    if interception_decision.enabled() {
        match crate::interception::find_interception_so() {
            Some(so_path) => {
                info!("Interception enabled (so: {})", so_path.display());
                let identity_path = super::up::interception_identity_path(&log_dir);
                for ctx in &mut pure_node_contexts {
                    let fqn = ros_launch_resolve::ros::sched_loader::fqn_for(
                        launch_dump,
                        ctx.record.namespace.as_deref(),
                        ctx.record
                            .name
                            .as_deref()
                            .or(ctx.record.exec_name.as_deref())
                            .unwrap_or("unknown"),
                        ctx.record.scope,
                    );
                    let node_name = super::up::interception_node_name(
                        Some(fqn.as_str()),
                        ctx.record.name.as_deref(),
                        ctx.record.exec_name.as_deref(),
                    );
                    match crate::interception::setup_child_interception(
                        &node_name,
                        &mut ctx.cmdline.env,
                        &so_path,
                        &runtime_config.interception,
                        // No contract resolves on `run` (issue #0045), so there
                        // is no allowlist to block endpoints against.
                        None,
                        identity_path.as_deref(),
                    ) {
                        Ok(consumer) => interception_consumers.push(consumer),
                        Err(e) => warn!("Interception setup failed for node: {:#}", e),
                    }
                }
                debug!(
                    "Interception: {} consumer(s) created",
                    interception_consumers.len()
                );
            }
            None => warn!(
                "Interception was asked for but libplay_launch_interception.so was not found \
                 (searched $PLAY_LAUNCH_INTERCEPTION_SO, the directory of the running binary, \
                 and ../lib beside it) -- this run records no per-message data; build it with \
                 `just build-interception`"
            ),
        }
    }

    let (sched_plan, sched_helper, sched_helper_join) = if let Some(resolved) = &resolved_sched {
        let path = &resolved.path;
        // `play_launch run` has no contract-loading infrastructure (single
        // node, no launch tree) — the derive pipeline runs with no
        // `ManifestIndex`, so every node gets a bare `MapperNode` (matches
        // v1 behavior for the legacy `manual`/`.toml` path exactly).
        let plan = crate::execution::sched_plan::SchedPlan::build(
            launch_dump,
            None,
            path,
            &common.sched_opts.target,
            common.sched_opts.sched_apply,
        )?;

        let (sched_helper, sched_helper_join) = if common.sched_opts.sched_apply
            != crate::execution::sched_apply::SchedApplyMode::Off
        {
            // Privilege is about whether the apply can SUCCEED, which is
            // independent of whether the helper process starts: a helper
            // that spawns fine but lacks CAP_SYS_NICE is useless (every
            // apply would EPERM). So check first, then spawn.
            // Issue #0015 — one report, naming WHY the capability is absent.
            // "never granted" and "granted, then the helper was rebuilt" are
            // different situations and used to print the same sentence.
            if let Some(msg) = crate::commands::capabilities::rt_privilege_report() {
                if common.sched_opts.sched_apply
                    == crate::execution::sched_apply::SchedApplyMode::Strict
                {
                    eyre::bail!("{msg}");
                }
                tracing::warn!("{msg}\n  Scheduling will NOT be applied.");
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
                        if common.sched_opts.sched_apply
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

    // Create MemberCoordinatorBuilder
    let mut builder = MemberCoordinatorBuilder::new();

    debug!("Adding {} node(s) to builder", pure_node_contexts.len());

    // Add actors to builder
    for context in pure_node_contexts {
        // Use node name, falling back to exec_name (many Autoware nodes have name=null)
        let member_name = context
            .record
            .name
            .as_ref()
            .or(context.record.exec_name.as_ref())
            .cloned()
            .unwrap_or_else(|| "unknown".to_string());

        // Create actor config
        let actor_config = ActorConfig {
            respawn_enabled: !common.containers.disable_respawn
                && context.record.respawn.unwrap_or(false),
            respawn_delay: context.record.respawn_delay.unwrap_or(0.0),
            max_respawn_attempts: None,
            output_dir: context.output_dir.clone(),
            pgid: Some(pgid),
            sched: sched_plan.as_ref().and_then(|p| {
                let fqn = ros_launch_resolve::ros::sched_loader::fqn_for(
                    launch_dump,
                    context.record.namespace.as_deref(),
                    &member_name,
                    context.record.scope,
                );
                p.for_fqn(&fqn).cloned()
            }),
            sched_mode: common.sched_opts.sched_apply,
            sched_helper: sched_helper.clone(),
            // `run` starts a single node, so there is nothing to pace and no
            // point paying for a gate that can never contend.
            startup: std::sync::Arc::new(
                crate::execution::startup_governor::StartupGovernor::disabled(),
            ),
            // One node, so there is no order to impose.
            startup_stage: 0,
            // `run` builds its one node by hand — no launch file, no <timer>.
            start_after: None,
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
    let (member_handle, member_runner) = builder.spawn(None).await;
    let member_handle = std::sync::Arc::new(member_handle); // Wrap in Arc for sharing
    debug!("All actors spawned successfully");

    // Setup web UI if requested (direct StateEvent streaming)
    let (runner_task, web_ui_task) = if common.is_web_ui_enabled() {
        debug!("Setting up web UI with direct StateEvent streaming...");

        // Create state event broadcaster for SSE clients
        let state_broadcaster = std::sync::Arc::new(web::StateEventBroadcaster::new());

        // Start web server
        let web_state = Arc::new(web::WebState::new(
            member_handle.clone(),
            log_dir.clone(),
            state_broadcaster.clone(),
            diagnostic_registry.clone(),
            None,
            Vec::new(),
            std::collections::HashMap::new(),
        ));

        // Parse web address
        let (addr, port) = common.parse_web_addr()?;

        // Log web UI URL before spawning (addr will be moved)
        info!("Web UI available at http://{}:{}", addr, port);

        // The run's own shutdown channel, as `up` wires it. This used to be
        // a throwaway `watch::channel` whose sender died at the end of this
        // block, so the server observed "sender gone", logged `Web server
        // shutting down...` a millisecond after `Web UI available at`, and
        // returned — which the loop below reads as the run being over: it
        // broke, signalled shutdown, the anchor was reaped, and a node still
        // being spawned failed `setpgid(0, pgid)` with a bare EPERM (issue
        // #0024). It also meant `run` never had a web UI or monitoring past
        // its first millisecond.
        let web_shutdown = shutdown_rx.clone();

        // Spawn web server task
        let web_server_task = tokio::spawn(async move {
            if let Err(e) = web::run_server(web_state, &addr, port, web_shutdown).await {
                error!("Web server error: {}", e);
            }
            Ok(())
        });

        // Runner task forwards state events and waits for completion
        let node_fqn_map = member_handle.node_fqn_map().clone();
        let runner_task = tokio::spawn(async move {
            forward_state_events_and_wait(member_runner, state_broadcaster, node_fqn_map).await
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

    // Issue #0053 — drain the children's rings and write the summaries.
    //
    // NO RULE ENGINE, and the two `None`s are how that is guaranteed rather
    // than merely intended: #0045 settled that `run` resolves no contract, so
    // there is nothing to enforce and no lifecycle subscription to feed it.
    // The task ends on the shutdown signal, so it is never the background task
    // whose completion breaks the loop below; the drain after that loop is what
    // gives it time to write `frontier_summary.json`, `stats_summary.json` and
    // `events.jsonl`.
    if !interception_consumers.is_empty() {
        background_tasks.push(tokio::spawn(crate::interception::run_interception_task(
            interception_consumers,
            log_dir.clone(),
            runtime_config.interception.clone(),
            shutdown_rx.clone(),
            None,
            None,
        )));
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
                    crate::process::kill_all_descendants();
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

    // Phase 38.10: all actors have completed (and with them, every
    // ActorConfig holding a SchedHelper clone). Drop our own clone so the
    // owner task sees the mpsc channel close and shuts the RT helper down
    // gracefully; bound the wait so a wedged owner task can't hang shutdown.
    if let Some(join) = sched_helper_join {
        drop(sched_helper);
        match tokio::time::timeout(std::time::Duration::from_secs(5), join).await {
            Ok(Ok(())) => debug!("RT helper owner task shut down cleanly"),
            Ok(Err(e)) => warn!("RT helper owner task panicked: {:#}", e),
            Err(_) => warn!("RT helper owner task did not shut down within timeout"),
        }
    }

    result
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

            while start.elapsed() < GRACEFUL_SHUTDOWN_TIMEOUT {
                // Check if any processes are still running
                let has_running = nix::sys::wait::waitpid(
                    nix::unistd::Pid::from_raw(-pgid),
                    Some(nix::sys::wait::WaitPidFlag::WNOHANG),
                );
                match has_running {
                    Ok(nix::sys::wait::WaitStatus::Exited(_, _))
                    | Ok(nix::sys::wait::WaitStatus::Signaled(_, _, _)) => {
                        // A child exited, keep waiting for others
                        tokio::time::sleep(SHUTDOWN_POLL_INTERVAL).await;
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
                        tokio::time::sleep(SHUTDOWN_POLL_INTERVAL).await;
                    }
                }
            }

            // If still running after grace period, force kill
            if !all_exited_gracefully && start.elapsed() >= GRACEFUL_SHUTDOWN_TIMEOUT {
                info!("Grace period expired, sending SIGKILL to remaining processes");
                kill_process_group(pgid, Signal::SIGKILL);
            }
        }

        info!("All child processes terminated");
    }

    #[cfg(not(unix))]
    {
        info!("All actors completed");
        crate::process::kill_all_descendants();
        info!("All child processes terminated");
    }

    result
}

#[cfg(test)]
mod enforcement_refusal_tests {
    //! Issue #0045. The flag is on `CommonOptions`, so `run` parses it
    //! whatever it can do with it; these pin WHICH invocations it refuses.

    use super::*;
    use cli::options::EnforceMode;

    /// The default (`warn`, not typed) must never refuse — every plain
    /// `play_launch run` carries it.
    #[test]
    fn default_mode_is_not_refused() {
        assert!(!enforcement_is_refused(EnforceMode::Warn, false));
        assert!(!enforcement_is_refused(EnforceMode::Strict, false));
    }

    /// `--enforce-rules off` is the documented way to say "run it
    /// unenforced", so typing it must not be an error.
    #[test]
    fn explicit_off_is_not_refused() {
        assert!(!enforcement_is_refused(EnforceMode::Off, true));
    }

    #[test]
    fn every_other_explicit_mode_is_refused() {
        for mode in [
            EnforceMode::Warn,
            EnforceMode::Strict,
            EnforceMode::RecordOnly,
        ] {
            assert!(
                enforcement_is_refused(mode, true),
                "{mode:?} was accepted and would then be ignored"
            );
        }
    }

    /// The message has to name the mode as the user spelled it, say why no
    /// contract can resolve, and name the way out — a refusal that only says
    /// "no" sends the reader to the source.
    #[test]
    fn the_refusal_names_the_mode_the_reason_and_the_alternative() {
        let msg = enforcement_refusal(EnforceMode::Strict);
        assert!(msg.contains("--enforce-rules strict"), "{msg}");
        assert!(msg.contains("launch file"), "{msg}");
        assert!(msg.contains("play_launch launch"), "{msg}");
        assert!(msg.contains("--enforce-rules off"), "{msg}");
        assert!(
            enforcement_refusal(EnforceMode::RecordOnly).contains("--enforce-rules record-only"),
            "the mode must be spelled the way the CLI takes it, not the way \
             Rust names the variant"
        );
    }
}
