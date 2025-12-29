mod cli;
mod event_driven;
mod execution;
mod monitoring;
mod python;
mod ros;
mod web;

use crate::{
    cli::{config::load_runtime_config, options::Options},
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
    ros::{
        container_readiness::ServiceDiscoveryHandle,
        launch_dump::{load_launch_dump, NodeContainerRecord},
    },
};
use clap::Parser;
use eyre::Context;
use rayon::prelude::*;
use std::{
    collections::{HashMap, HashSet},
    fs,
    path::{Path, PathBuf},
    process,
    sync::{
        atomic::{AtomicBool, AtomicUsize, Ordering},
        Arc, Mutex, OnceLock,
    },
};
use tokio::runtime::Runtime;
use tracing::{debug, error, info, warn};

/// Global handle for ROS service discovery (optional, only initialized if service checking enabled)
static SERVICE_DISCOVERY_HANDLE: OnceLock<ServiceDiscoveryHandle> = OnceLock::new();

/// Global flag for verbose logging
static VERBOSE_LOGGING: OnceLock<bool> = OnceLock::new();

/// Global flag to indicate graceful shutdown was completed (avoids duplicate SIGKILL in CleanupGuard)
static GRACEFUL_SHUTDOWN_DONE: AtomicBool = AtomicBool::new(false);

/// Check if verbose logging is enabled
pub fn is_verbose() -> bool {
    VERBOSE_LOGGING.get().copied().unwrap_or(false)
}

/// Kill all descendant processes of the current process recursively
#[cfg(unix)]
fn kill_all_descendants() {
    // Skip if graceful shutdown was already completed
    if GRACEFUL_SHUTDOWN_DONE.load(Ordering::Relaxed) {
        debug!("Graceful shutdown already completed, skipping CleanupGuard force kill");
        return;
    }

    let my_pid = process::id();
    debug!("Killing all descendant processes of PID {}", my_pid);

    // Create a dummy cancellation token that's never cancelled (cleanup should always complete)
    let cancel_token = Arc::new(AtomicBool::new(false));

    // Find all descendant PIDs recursively (including grandchildren)
    let descendants = find_all_descendants(my_pid, &cancel_token);

    if !descendants.is_empty() {
        debug!(
            "Found {} descendant processes to terminate: {:?}",
            descendants.len(),
            descendants
        );

        // Kill them in reverse order (children before parents) with SIGTERM
        for &pid in descendants.iter().rev() {
            debug!("Sending SIGTERM to PID {}", pid);
            use nix::{
                sys::signal::{kill, Signal},
                unistd::Pid,
            };
            let _ = kill(Pid::from_raw(pid as i32), Signal::SIGTERM);
        }

        // Give processes time to terminate gracefully
        std::thread::sleep(std::time::Duration::from_millis(200));

        // Force kill any remaining processes with SIGKILL
        for &pid in descendants.iter().rev() {
            debug!("Sending SIGKILL to PID {}", pid);
            use nix::{
                sys::signal::{kill, Signal},
                unistd::Pid,
            };
            let _ = kill(Pid::from_raw(pid as i32), Signal::SIGKILL);
        }
    } else {
        debug!("No descendant processes found to terminate");
    }
}

/// Spawn an anchor zombie process to allocate and hold a process group ID
#[cfg(unix)]
fn spawn_anchor_process() -> eyre::Result<(std::process::Child, i32)> {
    use std::os::unix::process::CommandExt;

    // Spawn a minimal process that exits immediately to become a zombie
    let anchor = std::process::Command::new("true")
        .process_group(0) // Creates new PGID = anchor's PID
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .wrap_err("Failed to spawn anchor zombie process")?;

    let pgid = anchor.id() as i32;
    debug!("Anchor zombie process created with PGID: {}", pgid);

    // Don't call wait() - let it become a zombie to hold the PGID
    Ok((anchor, pgid))
}

/// Kill an entire process group with a single signal
#[cfg(unix)]
fn kill_process_group(pgid: i32, signal: nix::sys::signal::Signal) {
    use nix::{sys::signal::killpg, unistd::Pid};

    match killpg(Pid::from_raw(pgid), signal) {
        Ok(_) => debug!("Sent {:?} to process group {}", signal, pgid),
        Err(e) => warn!("Failed to kill process group {}: {}", pgid, e),
    }
}

/// Recursively find all descendant PIDs of a given parent PID
#[cfg(unix)]
fn find_all_descendants(parent_pid: u32, cancel: &AtomicBool) -> Vec<u32> {
    use sysinfo::System;

    // Check cancellation before doing expensive work
    if cancel.load(Ordering::Relaxed) {
        return Vec::new();
    }

    let mut result = Vec::new();
    let mut sys = System::new();
    sys.refresh_processes(sysinfo::ProcessesToUpdate::All, true);

    // Find all processes that have parent_pid as their parent
    for (i, (pid, process)) in sys.processes().iter().enumerate() {
        // Check cancellation every 100 processes
        if i % 100 == 0 && cancel.load(Ordering::Relaxed) {
            return result;
        }

        if let Some(parent) = process.parent() {
            if parent.as_u32() == parent_pid {
                let child_pid = pid.as_u32();
                // Add this child
                result.push(child_pid);
                // Recursively find this child's descendants (grandchildren, etc.)
                result.extend(find_all_descendants(child_pid, cancel));

                // Check cancellation after recursive call
                if cancel.load(Ordering::Relaxed) {
                    return result;
                }
            }
        }
    }

    result
}

#[cfg(not(unix))]
fn kill_all_descendants() {
    // No-op on non-Unix systems
}

/// Extract verbose flag from command options
fn get_verbose_flag(opts: &Options) -> bool {
    match &opts.command {
        cli::options::Command::Launch(args) => args.common.verbose,
        cli::options::Command::Run(args) => args.common.verbose,
        cli::options::Command::Dump(_) => false, // Dump doesn't use CommonOptions
        cli::options::Command::Replay(args) => args.common.verbose,
        cli::options::Command::Plot(_) => false, // Plot doesn't use CommonOptions
        cli::options::Command::SetcapIoHelper => false,
        cli::options::Command::VerifyIoHelper => false,
    }
}

fn main() -> eyre::Result<()> {
    // Parse command-line options first (before initializing tracing)
    let opts = Options::parse();

    // Store verbose flag globally for conditional logging
    let verbose = get_verbose_flag(&opts);
    VERBOSE_LOGGING
        .set(verbose)
        .expect("VERBOSE_LOGGING already set");

    // Initialize tracing subscriber with INFO as default level
    // Priority: RUST_LOG > default (INFO)
    if std::env::var("RUST_LOG").is_ok() {
        // RUST_LOG env var takes precedence (for development/debugging)
        tracing_subscriber::fmt::init();
    } else {
        // Default to INFO level - verbose flag controls detail, not level
        tracing_subscriber::fmt()
            .with_max_level(tracing::Level::INFO)
            .init();
    }

    // Debug: Check AMENT_PREFIX_PATH at startup
    if let Ok(ament_path) = std::env::var("AMENT_PREFIX_PATH") {
        debug!(
            "AMENT_PREFIX_PATH first 200 chars: {}",
            ament_path.chars().take(200).collect::<String>()
        );
    } else {
        warn!("AMENT_PREFIX_PATH NOT SET!");
    }

    // Route to appropriate handler based on subcommand
    match &opts.command {
        cli::options::Command::Launch(args) => {
            handle_launch(args)?;
        }
        cli::options::Command::Run(args) => {
            handle_run(args)?;
        }
        cli::options::Command::Dump(args) => {
            handle_dump(args)?;
        }
        cli::options::Command::Replay(args) => {
            handle_replay(args)?;
        }
        cli::options::Command::Plot(args) => {
            handle_plot(args)?;
        }
        cli::options::Command::SetcapIoHelper => {
            handle_setcap_io_helper()?;
        }
        cli::options::Command::VerifyIoHelper => {
            handle_verify_io_helper()?;
        }
    }

    Ok(())
}

/// Handle the 'launch' subcommand (dump + replay)
fn handle_launch(args: &cli::options::LaunchArgs) -> eyre::Result<()> {
    use crate::python::dump_launcher::DumpLauncher;
    use tokio::runtime::Runtime;

    info!("Step 1/2: Recording launch execution...");

    // Create tokio runtime for async operations
    let runtime = Runtime::new()?;

    // Run dump phase
    runtime.block_on(async {
        let launcher = DumpLauncher::new()
            .wrap_err("Failed to initialize dump_launch. Ensure ROS workspace is sourced.")?;

        // Use default record.json in current directory
        let record_path = PathBuf::from("record.json");

        launcher
            .dump_launch(
                &args.package_or_path,
                args.launch_file.as_deref(),
                &args.launch_arguments,
                &record_path,
            )
            .await?;

        Ok::<(), eyre::Report>(())
    })?;

    info!("Step 2/2: Replaying launch execution...");

    // Create replay args and call handle_replay
    let replay_args = cli::options::ReplayArgs {
        input_file: PathBuf::from("record.json"),
        common: args.common.clone(),
    };

    handle_replay(&replay_args)?;

    Ok(())
}

/// Handle the 'run' subcommand (direct node execution)
fn handle_run(args: &cli::options::RunArgs) -> eyre::Result<()> {
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

/// Run a launch dump directly without file I/O
async fn run_direct(
    launch_dump: &ros::launch_dump::LaunchDump,
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

        info!("Member registry created with {} members", reg.len());
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

    info!("Spawning node...");

    // Use event-driven execution (always)
    info!("Using event-driven architecture");
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
            info!("All nodes spawned successfully");
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

/// Handle the 'dump' subcommand (dump only, no replay)
fn handle_dump(args: &cli::options::DumpArgs) -> eyre::Result<()> {
    use crate::python::dump_launcher::DumpLauncher;
    use tokio::runtime::Runtime;

    info!("Recording launch execution (dump only, no replay)...");

    // Create tokio runtime for async operations
    let runtime = Runtime::new()?;

    // Run dump phase
    runtime.block_on(async {
        let launcher = DumpLauncher::new()
            .wrap_err("Failed to initialize dump_launch. Ensure ROS workspace is sourced.")?;

        match &args.subcommand {
            cli::options::DumpSubcommand::Launch(launch_args) => {
                launcher
                    .dump_launch(
                        &launch_args.package_or_path,
                        launch_args.launch_file.as_deref(),
                        &launch_args.launch_arguments,
                        &args.output,
                    )
                    .await?;
            }
            cli::options::DumpSubcommand::Run(run_args) => {
                launcher
                    .dump_run(
                        &run_args.package,
                        &run_args.executable,
                        &run_args.args,
                        &args.output,
                    )
                    .await?;
            }
        }

        Ok::<(), eyre::Report>(())
    })?;

    info!("Dump completed successfully: {}", args.output.display());
    info!(
        "To replay: play_launch replay --input-file {}",
        args.output.display()
    );

    Ok(())
}

/// Handle the 'plot' subcommand
fn handle_plot(args: &cli::options::PlotArgs) -> eyre::Result<()> {
    use crate::python::plot_launcher::PlotLauncher;
    use tokio::runtime::Runtime;

    info!("Generating resource usage plots...");

    // Create tokio runtime for async operations
    let runtime = Runtime::new()?;

    // Run plot phase
    runtime.block_on(async {
        let launcher = PlotLauncher::new()
            .wrap_err("Failed to initialize plotting module. Ensure Python 3 is installed.")?;

        launcher
            .plot(
                args.log_dir.as_deref(),
                &args.base_log_dir,
                args.output_dir.as_deref(),
                &args.metrics,
                args.list_metrics,
            )
            .await?;

        Ok::<(), eyre::Report>(())
    })?;

    if !args.list_metrics {
        info!("Plotting completed successfully");
    }

    Ok(())
}

/// Find the I/O helper binary path via the wrapper's --binary-path flag
fn find_io_helper_path() -> eyre::Result<PathBuf> {
    // Call the wrapper script with --binary-path to get the actual binary location
    let output = process::Command::new("play_launch_io_helper")
        .arg("--binary-path")
        .output()
        .wrap_err("Failed to run 'play_launch_io_helper --binary-path'")?;

    if output.status.success() {
        let path = String::from_utf8_lossy(&output.stdout).trim().to_string();
        Ok(PathBuf::from(path))
    } else {
        let stderr = String::from_utf8_lossy(&output.stderr);
        eyre::bail!(
            "play_launch_io_helper not found.\n\
            Ensure play_launch is properly installed (pip install play_launch).\n\
            {stderr}"
        )
    }
}

/// Handle the 'setcap-io-helper' subcommand
fn handle_setcap_io_helper() -> eyre::Result<()> {
    let io_helper = find_io_helper_path()?;

    println!("Setting CAP_SYS_PTRACE on {}", io_helper.display());
    println!("This requires sudo privileges.\n");

    // Run setcap with sudo
    let status = process::Command::new("sudo")
        .args(["setcap", "cap_sys_ptrace+ep"])
        .arg(&io_helper)
        .status()
        .wrap_err("Failed to run setcap")?;

    if !status.success() {
        eyre::bail!("Failed to set capability");
    }

    // Verify the capability was set
    let output = process::Command::new("getcap")
        .arg(&io_helper)
        .output()
        .wrap_err("Failed to run getcap")?;

    let stdout = String::from_utf8_lossy(&output.stdout);

    if stdout.contains("cap_sys_ptrace") {
        println!("\n✓ I/O helper ready: {}", stdout.trim());
        println!("\nNote: Rerun this command after upgrading play_launch.");
    } else {
        eyre::bail!(
            "Capability may not have been set correctly. Output: {}",
            stdout
        );
    }

    Ok(())
}

/// Handle the 'verify-io-helper' subcommand
fn handle_verify_io_helper() -> eyre::Result<()> {
    let io_helper = find_io_helper_path()?;

    let output = process::Command::new("getcap")
        .arg(&io_helper)
        .output()
        .wrap_err("Failed to run getcap")?;

    let stdout = String::from_utf8_lossy(&output.stdout);

    // getcap outputs "=ep" but setcap uses "+ep" - check for both formats
    if stdout.contains("cap_sys_ptrace=ep") || stdout.contains("cap_sys_ptrace+ep") {
        println!("✓ I/O helper ready: {}", stdout.trim());
        Ok(())
    } else if !stdout.trim().is_empty() {
        println!(
            "⚠ I/O helper has unexpected capabilities: {}",
            stdout.trim()
        );
        println!("  Expected: cap_sys_ptrace=ep");
        println!("  Run: play_launch setcap-io-helper");
        std::process::exit(1);
    } else {
        println!("✗ I/O helper has no capabilities set");
        println!("  Run: play_launch setcap-io-helper");
        std::process::exit(1);
    }
}

/// Handle the 'replay' subcommand
fn handle_replay(args: &cli::options::ReplayArgs) -> eyre::Result<()> {
    let input_file = &args.input_file;

    // Load runtime configuration to check service readiness settings
    let runtime_config = cli::config::load_runtime_config(
        args.common.config.as_deref(),
        args.common.enable_monitoring,
        args.common.monitor_interval_ms,
    )?;

    // Start ROS service discovery thread if service checking is enabled (default: true)
    if runtime_config.container_readiness.wait_for_service_ready {
        info!("Starting ROS service discovery thread for container readiness checking...");
        if runtime_config
            .container_readiness
            .service_ready_timeout_secs
            == 0
        {
            info!("Container service readiness will wait indefinitely");
        } else {
            info!(
                "Container service readiness timeout: {}s",
                runtime_config
                    .container_readiness
                    .service_ready_timeout_secs
            );
        }

        match ros::container_readiness::start_service_discovery_thread() {
            Ok(handle) => {
                SERVICE_DISCOVERY_HANDLE
                    .set(handle)
                    .expect("SERVICE_DISCOVERY_HANDLE already set");
                info!("ROS service discovery thread started successfully");
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

/// Guard that ensures child processes are cleaned up on drop
struct CleanupGuard;

impl Drop for CleanupGuard {
    fn drop(&mut self) {
        debug!("CleanupGuard: Ensuring all child processes are terminated");
        kill_all_descendants();
    }
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
    info!(
        "Node contexts prepared: {} containers, {} pure nodes",
        container_contexts.len(),
        pure_node_contexts.len()
    );

    // Prepare LoadNode request execution contexts
    debug!("Preparing composable node contexts...");
    let ComposableNodeContextSet { load_node_contexts } =
        prepare_composable_node_contexts(&launch_dump, &load_node_log_dir)?;
    info!(
        "Composable node contexts prepared: {} load_node contexts",
        load_node_contexts.len()
    );

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

        info!("Member registry created with {} members", reg.len());
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
    info!("Using event-driven architecture");

    // Spawn non-container nodes using event-driven spawn
    info!(
        "Spawning {} non-container nodes...",
        pure_node_contexts.len()
    );
    match spawn_nodes_event_driven(
        pure_node_contexts,
        &process_monitor,
        &event_bus,
        Some(process_registry.clone()),
        Some(pgid),
    )
    .await
    {
        Ok(pids) => {
            info!(
                "Spawned {} non-container nodes with PIDs: {:?}",
                pids.len(),
                pids
            );
        }
        Err(e) => {
            error!("Failed to spawn non-container nodes: {}", e);
            return Err(e);
        }
    }

    // Spawn containers using event-driven spawn
    info!("Spawning {} containers...", container_contexts.len());
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
            info!("Spawned {} containers with PIDs: {:?}", pids.len(), pids);
        }
        Err(e) => {
            error!("Failed to spawn containers: {}", e);
            return Err(e);
        }
    }

    // Composable nodes will be loaded automatically when containers start
    // via ProcessStarted event → LoadRequested events
    info!("Composable nodes will be loaded automatically (via event-driven architecture)");

    // In event-driven mode, ProcessMonitor handles waiting for processes
    // We just need to wait for shutdown signal
    info!("Event-driven execution active. Waiting for shutdown signal...");

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
                info!("Event-driven execution complete");
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
                info!("Event-driven execution complete");
                Ok(())
            }
        }
    };

    result
}

/// Format current timestamp as YYYY-MM-DD_HH-MM-SS
fn format_timestamp() -> String {
    chrono::Local::now().format("%Y-%m-%d_%H-%M-%S").to_string()
}

/// Create a directory to store logging data.
///
/// Creates a timestamped subdirectory under the base log directory.
/// Format: base_dir/YYYY-MM-DD_HH-MM-SS/ or base_dir/YYYY-MM-DD_HH-MM-SS-N/ if conflicts occur
pub fn create_log_dir(log_dir: &Path) -> eyre::Result<PathBuf> {
    // Create base directory if it doesn't exist
    if !log_dir.exists() {
        fs::create_dir_all(log_dir)
            .wrap_err_with(|| format!("unable to create base directory {}", log_dir.display()))?;
    }

    // Create timestamped subdirectory
    let timestamp = format_timestamp();
    let mut timestamped_dir = log_dir.join(&timestamp);

    // If directory already exists, add -1, -2, ... suffix
    if timestamped_dir.exists() {
        for n in 1..=1000 {
            timestamped_dir = log_dir.join(format!("{}-{}", timestamp, n));
            if !timestamped_dir.exists() {
                break;
            }
        }

        // Check if we exhausted all attempts
        if timestamped_dir.exists() {
            eyre::bail!(
                "unable to find available timestamped directory after 1000 attempts for timestamp {}",
                timestamp
            );
        }
    }

    fs::create_dir(&timestamped_dir)
        .wrap_err_with(|| format!("unable to create directory {}", timestamped_dir.display()))?;

    // Create or update "latest" symlink
    let latest_symlink = log_dir.join("latest");

    // Remove old symlink if it exists
    if latest_symlink.exists() || latest_symlink.symlink_metadata().is_ok() {
        let _ = fs::remove_file(&latest_symlink); // Ignore errors if symlink doesn't exist
    }

    // Create new symlink pointing to the timestamped directory
    #[cfg(unix)]
    {
        use std::os::unix::fs::symlink;
        if let Some(dir_name) = timestamped_dir.file_name() {
            symlink(dir_name, &latest_symlink).wrap_err_with(|| {
                format!(
                    "Failed to create 'latest' symlink: {} -> {}",
                    latest_symlink.display(),
                    timestamped_dir.display()
                )
            })?;
        }
    }

    Ok(timestamped_dir)
}
