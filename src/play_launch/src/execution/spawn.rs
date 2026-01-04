// Actor-based execution for all commands.
// Traditional spawn_nodes() is used for `run` command only.
#![allow(dead_code)]

use super::context::{ComposableNodeContext, ExecutionContext, NodeContainerContext, NodeContext};
use crate::{ros::container_readiness::SERVICE_DISCOVERY_HANDLE, util::logging::is_verbose};
use futures::{
    future::{BoxFuture, FutureExt},
    stream::StreamExt,
};
use rayon::prelude::*;
use std::{
    collections::{HashMap, HashSet},
    fs::File,
    future::Future,
    io::prelude::*,
    path::{Path, PathBuf},
    process::ExitStatus,
    sync::{Arc, Mutex},
    time::Duration,
};
use tracing::{debug, error, info, warn};

/// Resolve a container name to absolute form.
///
/// ROS 2 performs automatic namespace resolution at runtime:
/// - If the name starts with '/', it's already absolute - return as-is
/// - Otherwise, it's relative - prepend '/' (assuming root namespace context)
///
/// This matches the behavior of rclpy's create_client() when resolving
/// service names in the root namespace.
fn resolve_container_name(name: &str) -> String {
    if name.starts_with('/') {
        name.to_string()
    } else {
        format!("/{}", name)
    }
}

/// The options to spawn a composable node loading process.
#[derive(Debug, Clone, Copy)]
pub struct SpawnComposableNodeConfig {
    pub max_concurrent_spawn: usize,
    pub max_attempts: usize,
    pub wait_timeout: Duration,
}

/// Configuration for executing composable nodes.
#[derive(Clone)]
pub struct ComposableNodeExecutionConfig {
    pub standalone_composable_nodes: bool,
    pub load_orphan_composable_nodes: bool,
    pub spawn_config: SpawnComposableNodeConfig,
    pub load_node_delay: Duration,
    pub service_wait_config: Option<crate::ros::container_readiness::ContainerWaitConfig>,
    pub process_registry: Option<Arc<Mutex<HashMap<u32, PathBuf>>>>,
    pub process_configs: Vec<crate::cli::config::ProcessConfig>,
}

/// The set of node containers and composable nodes belong to the same
/// container name.
struct NodeContainerGroup {
    pub container_contexts: Vec<NodeContext>,
    pub composable_node_contexts: Vec<ComposableNodeContext>,
}

/// The set of composable nodes belong to the same container name.
pub struct ComposableNodeGroup {
    pub composable_node_contexts: Vec<ComposableNodeContext>,
}

/// The set of tasks waiting for the completion of node container and
/// composable node loading/execution processes.
pub enum ComposableNodeTasks {
    Standalone {
        wait_composable_node_tasks: Vec<BoxFuture<'static, eyre::Result<()>>>,
    },
    Container {
        wait_container_tasks: Vec<BoxFuture<'static, eyre::Result<()>>>,
        load_nice_composable_nodes_task: BoxFuture<'static, ()>,
        load_orphan_composable_nodes_task: Option<BoxFuture<'static, ()>>,
    },
}

/// Spawn ROS node processes.
pub fn spawn_nodes(
    node_contexts: Vec<NodeContext>,
    process_registry: Option<Arc<Mutex<HashMap<u32, PathBuf>>>>,
    shutdown_signal: tokio::sync::watch::Receiver<bool>,
    disable_respawn: bool,
    pgid: Option<i32>,
) -> Vec<impl Future<Output = eyre::Result<()>>> {
    node_contexts
        .into_iter()
        .map(|context| {
            // Extract respawn configuration
            let respawn_enabled = !disable_respawn && context.record.respawn.unwrap_or(false);
            let respawn_delay_secs = context.record.respawn_delay.unwrap_or(0.0);

            let process_registry_clone = process_registry.clone();
            let mut shutdown_signal_clone = shutdown_signal.clone();

            async move {
                loop {
                    // Check if shutdown was already signaled (handles race condition)
                    if *shutdown_signal_clone.borrow() {
                        debug!("Shutdown already signaled, stopping respawn loop");
                        return Ok(());
                    }

                    // Prepare execution context
                    let exec = match context.to_exec_context(pgid) {
                        Ok(exec) => exec,
                        Err(err) => {
                            error!("Unable to prepare execution for node: {err}");
                            error!("which is caused by the node: {:#?}", context.record);
                            if !respawn_enabled {
                                return Err(err);
                            }
                            // Wait before retry on error
                            tokio::select! {
                                _ = tokio::time::sleep(Duration::from_secs_f64(respawn_delay_secs)) => {}
                                _ = shutdown_signal_clone.changed() => {
                                    debug!("Shutdown signal received during error delay");
                                    return Ok(());
                                }
                            }
                            continue;
                        }
                    };

                    let ExecutionContext {
                        log_name,
                        output_dir,
                        mut command,
                    } = exec;

                    // Spawn the process
                    let child = match command.spawn() {
                        Ok(child) => child,
                        Err(err) => {
                            error!("{log_name} is unable to start: {err}");
                            error!("Check {}", output_dir.display());
                            if !respawn_enabled {
                                return Err(err.into());
                            }
                            // Wait before retry on spawn error
                            warn!("{log_name} will respawn in {:.1}s", respawn_delay_secs);
                            tokio::select! {
                                _ = tokio::time::sleep(Duration::from_secs_f64(respawn_delay_secs)) => {}
                                _ = shutdown_signal_clone.changed() => {
                                    debug!("Shutdown signal received during spawn error delay");
                                    return Ok(());
                                }
                            }
                            continue;
                        }
                    };

                    // Register PID for monitoring
                    if let Some(ref registry) = process_registry_clone {
                        if let Some(pid) = child.id() {
                            if let Ok(mut reg) = registry.lock() {
                                reg.insert(pid, output_dir.clone());
                                debug!(
                                    "=== REGISTERED PID {} for node {} (total in registry: {}) ===",
                                    pid,
                                    log_name,
                                    reg.len()
                                );
                            }
                            // Initialize CSV file with headers immediately
                            if let Err(e) = crate::monitoring::resource_monitor::initialize_metrics_csv(&output_dir) {
                                warn!("Failed to initialize metrics CSV for {}: {}", log_name, e);
                            }
                        }
                    }

                    let pid_to_remove = child.id();
                    let result = wait_for_node(&log_name, &output_dir, child).await;

                    // Unregister PID when process exits
                    if let Some(ref registry) = process_registry_clone {
                        if let Some(pid) = pid_to_remove {
                            if let Ok(mut reg) = registry.lock() {
                                reg.remove(&pid);
                                debug!("Unregistered PID {} for node {}", pid, log_name);
                            }
                        }
                    }

                    // Check if we should respawn
                    if !respawn_enabled {
                        return result;
                    }

                    // Log respawn intent
                    if result.is_err() {
                        warn!("{log_name} exited with error, respawning in {:.1}s", respawn_delay_secs);
                    } else if is_verbose() {
                        info!("{log_name} exited, respawning in {:.1}s", respawn_delay_secs);
                    }

                    // Apply respawn delay with shutdown handling
                    tokio::select! {
                        _ = tokio::time::sleep(Duration::from_secs_f64(respawn_delay_secs)) => {}
                        _ = shutdown_signal_clone.changed() => {
                            debug!("Shutdown signal received during respawn delay");
                            return Ok(());
                        }
                    }

                    // Loop continues to restart process
                }
            }
        })
        .collect()
}

/// Load composable node into containers or execute them in standalone
/// containers.
pub fn spawn_or_load_composable_nodes(
    container_contexts: Vec<NodeContainerContext>,
    load_node_contexts: Vec<ComposableNodeContext>,
    container_names: &HashSet<String>,
    config: ComposableNodeExecutionConfig,
    pgid: Option<i32>,
) -> ComposableNodeTasks {
    if config.standalone_composable_nodes {
        info!("standalone composable node: {}", load_node_contexts.len());

        let standalone_composable_node_tasks = spawn_standalone_composable_nodes(
            load_node_contexts,
            config.process_registry.clone(),
            config.process_configs,
            pgid,
        );

        ComposableNodeTasks::Standalone {
            wait_composable_node_tasks: standalone_composable_node_tasks
                .into_iter()
                .map(|future| future.boxed())
                .collect(),
        }
    } else {
        // Separate orphan and non-orphan composable nodes
        // Try both exact match and resolved name (with namespace resolution)
        let (nice_load_node_contexts, orphan_load_node_contexts): (Vec<_>, Vec<_>) =
            load_node_contexts
                .into_par_iter()
                .partition_map(|mut context| {
                    use rayon::iter::Either;

                    let target_name = &context.record.target_container_name;

                    // Try exact match first
                    if container_names.contains(target_name) {
                        return Either::Left(context);
                    }

                    // Try with namespace resolution (e.g., "pointcloud_container" -> "/pointcloud_container")
                    let resolved_name = resolve_container_name(target_name);
                    if container_names.contains(&resolved_name) {
                        debug!(
                            "Resolved container name '{}' to '{}' for composable node '{}'",
                            target_name, resolved_name, context.record.node_name
                        );
                        // Update the target_container_name to the resolved name
                        // so subsequent code can use it consistently
                        context.record.target_container_name = resolved_name;
                        return Either::Left(context);
                    }

                    // No match found - it's an orphan
                    Either::Right(context)
                });

        info!("node containers: {}", container_names.len());
        info!(
            "loaded composable node: {}",
            nice_load_node_contexts.len() + orphan_load_node_contexts.len()
        );

        // Spawn node containers and prepare composable node
        // loading tasks
        let (container_tasks, load_nice_composable_nodes_task) =
            spawn_node_containers_and_load_composable_nodes(
                container_names,
                container_contexts,
                nice_load_node_contexts,
                &config,
                pgid,
            );

        let container_tasks: Vec<_> = container_tasks
            .into_iter()
            .map(|future| future.boxed())
            .collect();

        // Optionally load orphan composable nodes
        let load_orphan_composable_nodes_task = if orphan_load_node_contexts.is_empty() {
            None
        } else if config.load_orphan_composable_nodes {
            info!(
                "orphan composable node: {}",
                orphan_load_node_contexts.len()
            );
            let spawn_config = config.spawn_config;
            let task = async move {
                run_load_composable_nodes(orphan_load_node_contexts, spawn_config).await
            };
            Some(task.boxed())
        } else {
            warn!(
                "{} orphan composable nodes are not loaded",
                orphan_load_node_contexts.len()
            );
            None
        };

        ComposableNodeTasks::Container {
            wait_container_tasks: container_tasks,
            load_nice_composable_nodes_task: load_nice_composable_nodes_task.boxed(),
            load_orphan_composable_nodes_task,
        }
    }
}

/// Container process info for tracking
struct ContainerProcessInfo {
    pub pid: u32,
    pub output_dir: std::path::PathBuf,
}

/// Classify the container and composable node contexts into groups by
/// their container name.
fn build_container_groups(
    container_names: &HashSet<String>,
    container_contexts: Vec<NodeContainerContext>,
    nice_load_node_contexts: Vec<ComposableNodeContext>,
) -> HashMap<String, NodeContainerGroup> {
    let mut container_groups: HashMap<String, _> = container_names
        .iter()
        .map(|container_name| {
            (
                container_name.to_string(),
                NodeContainerGroup {
                    container_contexts: vec![],
                    composable_node_contexts: vec![],
                },
            )
        })
        .collect();

    for context in container_contexts {
        container_groups
            .get_mut(context.node_container_name.as_str())
            .expect("unable to find the container for the LoadNode request")
            .container_contexts
            .push(context.node_context);
    }

    for context in nice_load_node_contexts {
        container_groups
            .get_mut(&context.record.target_container_name)
            .expect("unable to find the container for the composable node")
            .composable_node_contexts
            .push(context);
    }

    container_groups
}

/// Spawn all node containers in each container group.
fn spawn_node_containers(
    container_groups: HashMap<String, NodeContainerGroup>,
    process_registry: Option<Arc<Mutex<HashMap<u32, PathBuf>>>>,
    process_configs: Vec<crate::cli::config::ProcessConfig>,
    pgid: Option<i32>,
) -> (
    Vec<impl Future<Output = eyre::Result<()>>>,
    HashMap<String, ComposableNodeGroup>,
    Vec<ContainerProcessInfo>,
) {
    let mut all_container_pids = Vec::new();

    let (container_task_groups, load_node_groups): (Vec<_>, HashMap<_, _>) = container_groups
        .into_iter()
        .filter_map(|(container_name, group)| {
            let NodeContainerGroup {
                container_contexts,
                composable_node_contexts: load_node_contexts,
            } = group;

            // Spawn all container nodes in the group and collect PIDs
            let mut container_tasks = Vec::new();
            let mut container_pids = Vec::new();

            for context in container_contexts {
                let exec = match context.to_exec_context(pgid) {
                    Ok(exec) => exec,
                    Err(err) => {
                        error!("Unable to prepare execution for node: {err}",);
                        error!("which is caused by the node: {:#?}", context.record);
                        continue;
                    }
                };
                let ExecutionContext {
                    log_name,
                    output_dir,
                    mut command,
                } = exec;

                let child = match command.spawn() {
                    Ok(child) => child,
                    Err(err) => {
                        error!("{log_name} is unable to start: {err}",);
                        error!("Check {}", output_dir.display());
                        continue;
                    }
                };

                // Capture PID for readiness checking and register for monitoring
                if let Some(pid) = child.id() {
                    container_pids.push(ContainerProcessInfo {
                        pid,
                        output_dir: output_dir.clone(),
                    });

                    // Register PID for monitoring
                    if let Some(ref registry) = process_registry {
                        if let Ok(mut reg) = registry.lock() {
                            reg.insert(pid, output_dir.clone());
                            debug!(
                                "=== REGISTERED container PID {} for node {} (total in registry: {}) ===",
                                pid,
                                log_name,
                                reg.len()
                            );
                        }
                        // Initialize CSV file with headers immediately
                        if let Err(e) = crate::monitoring::resource_monitor::initialize_metrics_csv(&output_dir) {
                            warn!("Failed to initialize metrics CSV for container {}: {}", log_name, e);
                        }
                    }

                    // Apply process control (CPU affinity and nice values)
                    for config in &process_configs {
                        if config.matches(&log_name) {
                            if let Err(e) = config.apply(pid) {
                                warn!(
                                    "Failed to apply process control to container {}: {}",
                                    log_name, e
                                );
                            } else {
                                debug!(
                                    "Applied process control to container {} (PID {})",
                                    log_name, pid
                                );
                            }
                            break; // Only apply first matching config
                        }
                    }
                }

                let registry_clone = process_registry.clone();
                let pid_to_remove = child.id();
                let task = async move {
                    let result = wait_for_node(&log_name, &output_dir, child).await;
                    // Unregister PID when container exits
                    if let Some(registry) = registry_clone {
                        if let Some(pid) = pid_to_remove {
                            if let Ok(mut reg) = registry.lock() {
                                reg.remove(&pid);
                                debug!("Unregistered container PID {} for node {}", pid, log_name);
                            }
                        }
                    }
                    result
                };
                container_tasks.push(task);
            }

            // If there is no container node spawned successfully,
            // skip later load node tasks.
            if container_tasks.is_empty() {
                return None;
            }

            all_container_pids.extend(container_pids);

            Some((container_name, container_tasks, load_node_contexts))
        })
        .map(|(container_name, container_tasks, load_node_contexts)| {
            (
                container_tasks,
                (
                    container_name,
                    ComposableNodeGroup {
                        composable_node_contexts: load_node_contexts,
                    },
                ),
            )
        })
        .unzip();

    let container_tasks = container_task_groups.into_iter().flatten().collect();

    (container_tasks, load_node_groups, all_container_pids)
}

/// Run all composable nodes in each container group.
async fn run_load_composable_node_groups(
    load_node_groups: HashMap<String, ComposableNodeGroup>,
    config: SpawnComposableNodeConfig,
) {
    let load_node_contexts: Vec<_> = load_node_groups
        .into_iter()
        .flat_map(|(_container_name, context)| context.composable_node_contexts)
        .collect();

    run_load_composable_nodes(load_node_contexts, config).await;
}

/// Check if a process with given PID is still running
fn is_process_running(pid: u32) -> bool {
    // On Unix, we can check if a process exists by sending signal 0
    #[cfg(unix)]
    {
        use nix::{sys::signal::kill, unistd::Pid};
        // Signal 0 (None) checks if process exists without sending an actual signal
        kill(Pid::from_raw(pid as i32), None).is_ok()
    }

    #[cfg(not(unix))]
    {
        // On non-Unix, assume it's running
        true
    }
}

/// Wait for container processes to be running and stable
async fn wait_for_containers_running(
    container_pids: Vec<ContainerProcessInfo>,
    initial_delay: Duration,
) {
    if container_pids.is_empty() {
        return;
    }

    info!(
        "Waiting for {} container(s) to start...",
        container_pids.len()
    );

    // Initial delay to let processes start
    tokio::time::sleep(initial_delay).await;

    // Check that all container processes are running
    let mut all_running = true;
    for container_info in &container_pids {
        if !is_process_running(container_info.pid) {
            error!(
                "Container process PID {} exited prematurely (check {})",
                container_info.pid,
                container_info.output_dir.display()
            );
            all_running = false;
        } else {
            debug!("Container PID {} is running", container_info.pid);
        }
    }

    if all_running {
        info!("All {} container(s) are running", container_pids.len());
    } else {
        warn!("Some containers failed to start, but continuing anyway");
    }
}

/// Spawn node container processes and load all composable nodes to
/// them.
fn spawn_node_containers_and_load_composable_nodes(
    container_names: &HashSet<String>,
    container_contexts: Vec<NodeContainerContext>,
    load_node_contexts: Vec<ComposableNodeContext>,
    config: &ComposableNodeExecutionConfig,
    pgid: Option<i32>,
) -> (
    Vec<impl Future<Output = eyre::Result<()>>>,
    impl Future<Output = ()>,
) {
    // Build container groups
    let container_groups =
        build_container_groups(container_names, container_contexts, load_node_contexts);

    // Spawn node containers in each node container group and get their PIDs
    let (container_tasks, nice_load_node_groups, container_pids) = spawn_node_containers(
        container_groups,
        config.process_registry.clone(),
        config.process_configs.clone(),
        pgid,
    );

    let container_names_vec: Vec<String> = container_names.iter().cloned().collect();
    let load_node_delay = config.load_node_delay;
    let service_wait_config = config.service_wait_config.clone();
    let spawn_config = config.spawn_config;

    let load_composable_nodes_task = async move {
        if nice_load_node_groups.is_empty() {
            return;
        }

        // First, wait for container processes to be running
        wait_for_containers_running(container_pids, load_node_delay).await;

        // Optionally, wait for container services to be ready
        if let Some(service_config) = service_wait_config {
            if let Some(discovery_handle) = SERVICE_DISCOVERY_HANDLE.get() {
                info!("Waiting for container services to be ready...");
                if let Err(e) = crate::ros::container_readiness::wait_for_containers_ready(
                    &container_names_vec,
                    &service_config,
                    discovery_handle,
                )
                .await
                {
                    error!("Error waiting for container services: {}", e);
                }
            } else {
                warn!("Service discovery not initialized, skipping service readiness check");
            }
        }

        info!("Proceeding to load composable nodes");

        // Spawn composable nodes having belonging containers.
        run_load_composable_node_groups(nice_load_node_groups, spawn_config).await;
    };

    (container_tasks, load_composable_nodes_task)
}

/// Spawn standalone composable nodes.
fn spawn_standalone_composable_nodes(
    load_node_contexts: Vec<ComposableNodeContext>,
    process_registry: Option<Arc<Mutex<HashMap<u32, PathBuf>>>>,
    process_configs: Vec<crate::cli::config::ProcessConfig>,
    pgid: Option<i32>,
) -> Vec<impl Future<Output = eyre::Result<()>>> {
    load_node_contexts
        .into_iter()
        .filter_map(|context| {
            let ComposableNodeContext {
                log_name,
                output_dir,
                ..
            } = &context;
            let mut command = match context.to_standalone_node_command(pgid) {
                Ok(command) => command,
                Err(err) => {
                    error!("{log_name} fails to generate command: {err}",);
                    return None;
                }
            };

            let child = match command.spawn() {
                Ok(child) => child,
                Err(err) => {
                    error!("{log_name} is unable to start: {err}",);
                    error!("Check {}", output_dir.display());
                    return None;
                }
            };

            // Register PID for monitoring and apply process control
            if let Some(pid) = child.id() {
                // Register PID for monitoring
                if let Some(ref registry) = process_registry {
                    if let Ok(mut reg) = registry.lock() {
                        reg.insert(pid, output_dir.clone());
                        debug!("=== REGISTERED standalone composable node PID {} for {} (total in registry: {}) ===", pid, log_name, reg.len());
                    }
                    // Initialize CSV file with headers immediately
                    if let Err(e) = crate::monitoring::resource_monitor::initialize_metrics_csv(output_dir) {
                        warn!("Failed to initialize metrics CSV for standalone composable node {}: {}", log_name, e);
                    }
                }

                // Apply process control (CPU affinity and nice values)
                for config in &process_configs {
                    if config.matches(log_name) {
                        if let Err(e) = config.apply(pid) {
                            warn!(
                                "Failed to apply process control to standalone composable node {}: {}",
                                log_name, e
                            );
                        } else {
                            debug!("Applied process control to standalone composable node {} (PID {})", log_name, pid);
                        }
                        break; // Only apply first matching config
                    }
                }
            }

            let registry_clone = process_registry.clone();
            let pid_to_remove = child.id();
            let task = async move {
                let ComposableNodeContext {
                    log_name,
                    output_dir,
                    ..
                } = &context;
                let result = wait_for_node(log_name, output_dir, child).await;
                // Unregister PID when process exits
                if let Some(registry) = registry_clone {
                    if let Some(pid) = pid_to_remove {
                        if let Ok(mut reg) = registry.lock() {
                            reg.remove(&pid);
                            debug!("Unregistered standalone composable node PID {} for {}", pid, log_name);
                        }
                    }
                }
                result
            };
            Some(task)
        })
        .collect()
}

/// Load a set of composable nodes.
async fn run_load_composable_nodes(
    load_node_contexts: Vec<ComposableNodeContext>,
    config: SpawnComposableNodeConfig,
) {
    let SpawnComposableNodeConfig {
        max_concurrent_spawn,
        max_attempts,
        wait_timeout,
    } = config;

    let total_count = load_node_contexts.len();

    let mut futures = futures::stream::iter(load_node_contexts.into_iter())
        .map(|context| async move {
            run_load_composable_node(&context, wait_timeout, max_attempts).await
        })
        .buffer_unordered(max_concurrent_spawn)
        .zip(futures::stream::iter(0..));

    while let Some(((), nth)) = futures.next().await {
        if (nth + 1) % 10 == 0 {
            info!(
                "Done loading {} out of {total_count} composable nodes.",
                nth + 1
            );
        }
    }
}

/// Load one composable node up to a max number of failing attempts.
async fn run_load_composable_node(
    context: &ComposableNodeContext,
    wait_timeout: Duration,
    max_attempts: usize,
) {
    let ComposableNodeContext {
        log_name,
        output_dir,
        ..
    } = context;
    debug!("{log_name} is loading");

    for round in 1..=max_attempts {
        let result = run_load_composable_node_via_service(context, wait_timeout, round).await;

        match result {
            Ok(true) => return,
            Ok(false) => {
                warn!("{log_name} fails to start. Retrying... ({round}/{max_attempts})");
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            Err(err) => {
                error!("{log_name} fails due to error: {:?}", err);
                break;
            }
        }
    }

    error!("{log_name} is unable to start");
    error!("Check {}", output_dir.display());
}

/// Load one composable node via ROS service call
///
/// NOTE: This function is deprecated and not used. Container-managed loading
/// (Phase 3 of the refactoring) uses the MemberCoordinator pattern instead.
#[allow(dead_code)]
async fn run_load_composable_node_via_service(
    context: &ComposableNodeContext,
    _timeout: Duration,
    _round: usize,
) -> eyre::Result<bool> {
    let log_name = &context.log_name;
    Err(eyre::eyre!(
        "{}: This function is deprecated. Use MemberCoordinator for composable node loading.",
        log_name
    ))
}

/// Wait for the completion of a node process.
async fn wait_for_node(
    log_name: &str,
    output_dir: &Path,
    mut child: tokio::process::Child,
) -> eyre::Result<()> {
    if let Some(pid) = child.id() {
        let pid_path = output_dir.join("pid");
        let mut pid_file = File::create(pid_path)?;
        writeln!(pid_file, "{pid}")?;
    }

    let status = child.wait().await?;
    save_node_status(&status, output_dir, log_name)?;
    eyre::Ok(())
}

/// Save the status code for a completed node process.
fn save_node_status(status: &ExitStatus, output_dir: &Path, log_name: &str) -> eyre::Result<()> {
    let status_path = output_dir.join("status");

    // Save status code
    {
        let mut status_file = File::create(status_path)?;
        if let Some(code) = status.code() {
            writeln!(status_file, "{code}",)?;
        } else {
            writeln!(status_file, "[none]")?;
        }
    }

    // Print status to the terminal
    if status.success() {
        if is_verbose() {
            info!("[{log_name}] finishes");
        }
    } else {
        match status.code() {
            Some(code) => {
                error!("{log_name} fails with code {code}.",);
                error!("Check {}", output_dir.display());
            }
            None => {
                error!("{log_name} fails.",);
                error!("Check {}", output_dir.display());
            }
        }
    }

    eyre::Ok(())
}

/// Save the status code for a completed composable node
/// loading/execution process.
/// Save status for service-based composable node loading
fn save_composable_node_service_status(
    success: bool,
    output_dir: &Path,
    log_name: &str,
) -> eyre::Result<()> {
    use std::io::Write;

    let status_path = output_dir.join("status");
    let mut status_file = File::create(status_path)?;

    // Write 0 for success, 1 for failure (mimics exit codes)
    let code = if success { 0 } else { 1 };
    writeln!(status_file, "{code}")?;

    if success {
        if is_verbose() {
            info!("{log_name} loading finishes successfully via service");
        }
    } else {
        error!("{log_name} fails via service");
        error!("Check {}", output_dir.display());
    }

    Ok(())
}
