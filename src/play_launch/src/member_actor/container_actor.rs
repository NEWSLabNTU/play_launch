//! Container actor with supervision of composable nodes
//!
//! ContainerActor manages the lifecycle of a container process and supervises
//! its composable nodes. When the container starts, it broadcasts its state to
//! composable nodes. When the container stops or fails, composable nodes are
//! automatically blocked.
//!
//! Phase 5: Provides both standalone function (run_container) and trait-based actor (ContainerActor).
//! Note: Containers have special spawning requirements (container_state_rx must be obtained before
//! spawning), so the coordinator still creates the actor directly to get the state receiver.

use super::{
    actor_traits::MemberActor,
    container_control::{ContainerControlEvent, CurrentLoad, LoadNodeResponse, LoadRequest},
    events::{ControlEvent, StateEvent},
    state::{ActorConfig, ContainerState, NodeState},
};
use crate::execution::context::NodeContext;
use eyre::{Context as _, Result};
use std::{
    collections::{HashMap, VecDeque},
    path::PathBuf,
    sync::{Arc, Mutex},
};
use tokio::{
    process::Command,
    sync::{mpsc, watch},
    time::{sleep, Duration},
};
use tracing::{debug, error, info, warn};

/// Standalone async function for running a container (Phase 5)
///
/// This is the standalone function version that will be used with FuturesUnordered.
/// Note: For containers that need composable node supervision, use ContainerActor directly
/// to obtain container_state_rx before spawning.
#[allow(clippy::too_many_arguments)]
pub async fn run_container(
    name: String,
    context: NodeContext,
    config: ActorConfig,
    control_rx: mpsc::Receiver<ControlEvent>,
    state_tx: mpsc::Sender<StateEvent>,
    shutdown_rx: watch::Receiver<bool>,
    process_registry: Option<Arc<Mutex<HashMap<u32, PathBuf>>>>,
    load_control_rx: mpsc::Receiver<ContainerControlEvent>,
    ros_node: Option<Arc<rclrs::Node>>,
) -> Result<()> {
    // Create the actor and run it (wrapper approach for Phase 5)
    let actor = ContainerActor::new(
        name,
        context,
        config,
        control_rx,
        state_tx,
        shutdown_rx,
        process_registry,
        load_control_rx,
        ros_node,
    );
    actor.run().await
}

/// Handle to a composable node actor
pub struct ComposableActorHandle {
    /// Name of the composable node
    pub name: String,
    /// Task handle
    pub task: tokio::task::JoinHandle<Result<()>>,
}

/// Container actor that supervises composable nodes
pub struct ContainerActor {
    /// Unique name for this actor
    name: String,
    /// Node execution context
    context: NodeContext,
    /// Actor configuration
    config: ActorConfig,
    /// Current state
    state: NodeState,
    /// Channel to receive control events
    control_rx: mpsc::Receiver<ControlEvent>,
    /// Channel to send state events
    state_tx: mpsc::Sender<StateEvent>,
    /// Shutdown signal receiver
    shutdown_rx: watch::Receiver<bool>,
    /// Process registry for I/O monitoring
    process_registry: Option<Arc<Mutex<HashMap<u32, PathBuf>>>>,
    /// Container state broadcast to composable nodes
    container_state_tx: watch::Sender<ContainerState>,
    /// Composable node actor handles
    composable_actors: Vec<ComposableActorHandle>,

    // LoadNode management (Phase 1: Container-managed loading)
    /// Channel to receive LoadNode requests from composable nodes
    load_control_rx: mpsc::Receiver<ContainerControlEvent>,
    /// ROS node for service calls (shared across all containers)
    ros_node: Option<Arc<rclrs::Node>>,
    /// ROS service client for LoadNode
    load_client: Option<rclrs::Client<composition_interfaces::srv::LoadNode>>,
    /// Queue of pending load requests
    pending_loads: VecDeque<LoadRequest>,
    /// Currently processing load
    current_load: Option<CurrentLoad>,
}

impl ContainerActor {
    /// Create a new container actor
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        name: String,
        context: NodeContext,
        config: ActorConfig,
        control_rx: mpsc::Receiver<ControlEvent>,
        state_tx: mpsc::Sender<StateEvent>,
        shutdown_rx: watch::Receiver<bool>,
        process_registry: Option<Arc<Mutex<HashMap<u32, PathBuf>>>>,
        load_control_rx: mpsc::Receiver<ContainerControlEvent>,
        ros_node: Option<Arc<rclrs::Node>>,
    ) -> Self {
        let (container_state_tx, _container_state_rx) = watch::channel(ContainerState::Pending);

        Self {
            name,
            context,
            config,
            state: NodeState::Pending,
            control_rx,
            state_tx,
            shutdown_rx,
            process_registry,
            container_state_tx,
            composable_actors: Vec::new(),
            load_control_rx,
            ros_node,
            load_client: None,
            pending_loads: VecDeque::new(),
            current_load: None,
        }
    }

    /// Get a receiver for container state (for composable nodes)
    pub fn container_state_rx(&self) -> watch::Receiver<ContainerState> {
        self.container_state_tx.subscribe()
    }

    /// Add a composable node actor handle
    pub fn add_composable_actor(&mut self, handle: ComposableActorHandle) {
        self.composable_actors.push(handle);
    }

    // LoadNode queue management methods (Phase 1)

    /// Start processing the next load request in the queue
    /// Spawns an async task to call the LoadNode service without blocking
    fn start_next_load(&mut self) {
        // Don't start if already processing a load
        if self.current_load.is_some() {
            return;
        }

        // Get next request from queue
        if let Some(request) = self.pending_loads.pop_front() {
            let start_time = std::time::Instant::now();
            let queue_wait_ms = start_time.duration_since(request.request_time).as_millis() as u64;

            debug!(
                "{}: Starting load for {} (queue wait: {}ms, {} requests remaining)",
                self.name,
                request.composable_name,
                queue_wait_ms,
                self.pending_loads.len()
            );

            // Spawn async task to call LoadNode service
            // This allows the select! loop to continue receiving new load requests
            let container_name = self.name.clone();
            let load_client = self.load_client.clone();
            let params = super::container_control::LoadParams {
                composable_name: request.composable_name.clone(),
                package: request.package.clone(),
                plugin: request.plugin.clone(),
                node_name: request.node_name.clone(),
                node_namespace: request.node_namespace.clone(),
                remap_rules: request.remap_rules.clone(),
                parameters: request.parameters.clone(),
                extra_args: request.extra_args.clone(),
                request_time: request.request_time,
            };

            let task = tokio::spawn(async move {
                Self::call_load_node_service(container_name, load_client, params, start_time).await
            });

            self.current_load = Some(CurrentLoad {
                request,
                start_time,
                task,
            });
        }
    }

    /// Call the LoadNode ROS service for a composable node
    /// This is a standalone function so it can be spawned as an async task
    async fn call_load_node_service(
        container_name: String,
        load_client: Option<rclrs::Client<composition_interfaces::srv::LoadNode>>,
        params: super::container_control::LoadParams,
        start_time: std::time::Instant,
    ) -> Result<LoadNodeResponse> {
        use eyre::Context;

        // Check if client exists
        let client = load_client.ok_or_else(|| {
            eyre::eyre!(
                "No LoadNode service client available for container {}",
                container_name
            )
        })?;

        // Build LoadNode request
        let ros_request = composition_interfaces::srv::LoadNode_Request {
            package_name: params.package.clone(),
            plugin_name: params.plugin.clone(),
            node_name: params.node_name.clone(),
            node_namespace: params.node_namespace.clone(),
            log_level: 0, // Default log level
            remap_rules: params.remap_rules,
            parameters: crate::ros::component_loader::convert_parameters_to_ros(
                &params.parameters,
            )?,
            extra_arguments: crate::ros::component_loader::convert_parameters_to_ros(
                &params.extra_args,
            )?,
        };

        // Wait for LoadNode service to be available
        // Container may have just started and service not yet registered
        debug!(
            "{}: Waiting for LoadNode service to be available for {}",
            container_name, params.composable_name
        );

        let service_wait_start = std::time::Instant::now();
        let service_timeout = std::time::Duration::from_secs(30);

        loop {
            match client.service_is_ready() {
                Ok(true) => break,
                Ok(false) => {
                    // Service not ready yet, continue waiting
                }
                Err(e) => {
                    warn!(
                        "{}: Error checking service readiness for {}: {:?}",
                        container_name, params.composable_name, e
                    );
                    // Continue waiting despite error
                }
            }

            if service_wait_start.elapsed() > service_timeout {
                return Err(eyre::eyre!(
                    "LoadNode service not available after 30s (container may not have started properly)"
                ));
            }

            // Sleep briefly before checking again
            tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
        }

        debug!(
            "{}: LoadNode service available after {}ms, calling service for {}/{} (node_name: {}, namespace: {})",
            container_name,
            service_wait_start.elapsed().as_millis(),
            params.package,
            params.plugin,
            params.node_name,
            params.node_namespace
        );

        // Call the service (NO timeout - wait indefinitely)
        // Composable node actor handles timeout on its side
        let response_future: rclrs::Promise<composition_interfaces::srv::LoadNode_Response> =
            client
                .call(&ros_request)
                .context("Failed to initiate LoadNode service call")?;

        match response_future.await {
            Ok(response) => {
                // Calculate timing metrics
                let service_call_ms = start_time.elapsed().as_millis() as u64;
                let queue_wait_ms =
                    start_time.duration_since(params.request_time).as_millis() as u64;
                let total_duration_ms = params.request_time.elapsed().as_millis() as u64;

                debug!(
                    "{}: LoadNode response for {}: success={}, unique_id={}, queue={}ms, service={}ms, total={}ms",
                    container_name,
                    params.composable_name,
                    response.success,
                    response.unique_id,
                    queue_wait_ms,
                    service_call_ms,
                    total_duration_ms
                );

                Ok(LoadNodeResponse {
                    success: response.success,
                    error_message: response.error_message.clone(),
                    full_node_name: response.full_node_name.clone(),
                    unique_id: response.unique_id,
                    timing: super::container_control::LoadTimingMetrics {
                        queue_wait_ms,
                        service_call_ms,
                        total_duration_ms,
                    },
                })
            }
            Err(e) => {
                warn!(
                    "{}: LoadNode service call error for {}: {:?}",
                    container_name, params.composable_name, e
                );
                Err(eyre::eyre!("Service call failed: {:?}", e))
            }
        }
    }

    /// Drain the load queue and respond with an error
    fn drain_queue(&mut self, error: &str) {
        // Cancel current load
        if let Some(load) = self.current_load.take() {
            debug!(
                "{}: Cancelling current load for {}: {}",
                self.name, load.request.composable_name, error
            );
            let _ = load.request.response_tx.send(Err(eyre::eyre!("{}", error)));
        }

        // Cancel all queued loads
        let queue_len = self.pending_loads.len();
        if queue_len > 0 {
            debug!(
                "{}: Draining {} queued load requests: {}",
                self.name, queue_len, error
            );
        }

        for request in self.pending_loads.drain(..) {
            let _ = request.response_tx.send(Err(eyre::eyre!("{}", error)));
        }
    }

    /// Build the full node name from namespace and name
    fn full_node_name(&self) -> String {
        let namespace = self.context.record.namespace.as_deref().unwrap_or("/");
        let name = self.context.record.name.as_deref().unwrap_or(&self.name);

        if namespace == "/" {
            format!("/{}", name)
        } else if namespace.ends_with('/') {
            format!("{}{}", namespace, name)
        } else {
            format!("{}/{}", namespace, name)
        }
    }

    /// Spawn the container process
    async fn spawn_process(&self) -> Result<tokio::process::Child> {
        let exec_context = self
            .context
            .to_exec_context(self.config.pgid)
            .context("Failed to create execution context")?;

        let mut command: Command = exec_context.command;

        debug!(
            "Spawning container process: {:?} (output_dir: {:?})",
            command, exec_context.output_dir
        );

        let child = command
            .spawn()
            .context("Failed to spawn container process")?;

        Ok(child)
    }

    /// Handle the Pending state
    async fn handle_pending(&mut self) -> Result<bool> {
        debug!("{}: Spawning container process", self.name);

        // Drain any pending loads (container not running)
        self.drain_queue("Container not running");

        // Broadcast pending state to composable nodes
        let _ = self.container_state_tx.send(ContainerState::Pending);

        match self.spawn_process().await {
            Ok(child) => {
                let pid = child
                    .id()
                    .ok_or_else(|| eyre::eyre!("Failed to get PID from spawned process"))?;

                debug!("{}: Container process started with PID {}", self.name, pid);

                // Register process for I/O monitoring
                if let Some(ref registry) = self.process_registry {
                    if let Ok(mut reg) = registry.lock() {
                        reg.insert(pid, self.config.output_dir.clone());
                    }
                }

                // Broadcast running state to composable nodes
                let _ = self
                    .container_state_tx
                    .send(ContainerState::Running { pid });

                // Send state event
                let _ = self
                    .state_tx
                    .send(StateEvent::Started {
                        name: self.name.clone(),
                        pid,
                    })
                    .await;

                // Create LoadNode service client now that container is running
                if self.load_client.is_none() {
                    let service_name = format!("{}/_container/load_node", self.full_node_name());
                    debug!(
                        "{}: Creating LoadNode service client for {}",
                        self.name, service_name
                    );

                    if let Some(ref ros_node) = self.ros_node {
                        match ros_node
                            .create_client::<composition_interfaces::srv::LoadNode>(&service_name)
                        {
                            Ok(client) => {
                                self.load_client = Some(client);
                                debug!(
                                    "{}: LoadNode service client created successfully",
                                    self.name
                                );
                            }
                            Err(e) => {
                                warn!(
                                    "{}: Failed to create LoadNode service client: {:#}",
                                    self.name, e
                                );
                                // Don't fail the container - composable nodes will get errors when trying to load
                            }
                        }
                    } else {
                        warn!(
                            "{}: No ROS node available for LoadNode service calls",
                            self.name
                        );
                    }
                }

                self.state = NodeState::Running { child, pid };
                Ok(true) // Continue running
            }
            Err(e) => {
                error!("{}: Failed to spawn container: {:#}", self.name, e);

                // Broadcast failed state to composable nodes
                let _ = self.container_state_tx.send(ContainerState::Failed);

                let _ = self
                    .state_tx
                    .send(StateEvent::Failed {
                        name: self.name.clone(),
                        error: e.to_string(),
                    })
                    .await;

                self.state = NodeState::Failed {
                    error: e.to_string(),
                };
                Ok(false) // Stop actor
            }
        }
    }

    /// Handle the Running state
    async fn handle_running(&mut self, mut child: tokio::process::Child, pid: u32) -> Result<bool> {
        debug!("{}: Container running with PID {}", self.name, pid);

        loop {
            tokio::select! {
                // Wait for child to exit
                status = child.wait() => {
                    let exit_code = status.ok().and_then(|s| s.code());
                    info!("{}: Container exited with code {:?}", self.name, exit_code);

                    // Unregister from process registry
                    if let Some(ref registry) = self.process_registry {
                        if let Ok(mut reg) = registry.lock() {
                            reg.remove(&pid);
                        }
                    }

                    // Drain load queue (container crashed)
                    self.drain_queue("Container crashed");

                    // Broadcast stopped state to composable nodes
                    let _ = self.container_state_tx.send(ContainerState::Stopped);

                    // Send state event
                    let _ = self.state_tx.send(StateEvent::Exited {
                        name: self.name.clone(),
                        exit_code,
                    }).await;

                    // Check if respawn is enabled
                    if self.config.respawn_enabled {
                        self.state = NodeState::Respawning {
                            exit_code,
                            attempt: 0,
                        };
                        return Ok(true); // Continue to respawn
                    } else {
                        self.state = NodeState::Stopped { exit_code };
                        let _ = self.state_tx.send(StateEvent::Terminated {
                            name: self.name.clone(),
                        }).await;
                        return Ok(false); // Stop actor
                    }
                }

                // Handle control events
                Some(event) = self.control_rx.recv() => {
                    match event {
                        ControlEvent::Stop => {
                            info!("{}: Received Stop command, killing container", self.name);
                            child.kill().await.ok();

                            // Wait for process to exit
                            let _ = child.wait().await;

                            // Unregister from process registry
                            if let Some(ref registry) = self.process_registry {
                                if let Ok(mut reg) = registry.lock() {
                                    reg.remove(&pid);
                                }
                            }

                            // Drain load queue
                            self.drain_queue("Container stopped");

                            // Broadcast stopped state
                            let _ = self.container_state_tx.send(ContainerState::Stopped);

                            self.state = NodeState::Stopped { exit_code: None };
                            let _ = self.state_tx.send(StateEvent::Terminated {
                                name: self.name.clone(),
                            }).await;
                            return Ok(false); // Stop actor
                        }
                        ControlEvent::Restart => {
                            info!("{}: Received Restart command, killing and respawning container", self.name);
                            child.kill().await.ok();

                            // Wait for process to exit
                            let _ = child.wait().await;

                            // Unregister from process registry
                            if let Some(ref registry) = self.process_registry {
                                if let Ok(mut reg) = registry.lock() {
                                    reg.remove(&pid);
                                }
                            }

                            // Drain load queue
                            self.drain_queue("Container restarting");

                            // Transition to Pending to respawn
                            self.state = NodeState::Pending;
                            return Ok(true); // Continue to respawn
                        }
                        _ => {
                            warn!("{}: Unhandled control event: {:?}", self.name, event);
                        }
                    }
                }

                // Handle LoadNode requests (Phase 1)
                Some(event) = self.load_control_rx.recv() => {
                    if let Some(request) = Option::<LoadRequest>::from(event) {
                        debug!("{}: Received load request for {}", self.name, request.composable_name);

                        // Queue the request
                        self.pending_loads.push_back(request);

                        // Start processing next load ONLY if not currently processing
                        // This spawns an async task so it doesn't block the select! loop
                        if self.current_load.is_none() {
                            self.start_next_load();
                        }
                    }
                }

                // Poll for current LoadNode service call completion
                result = async {
                    match &mut self.current_load {
                        Some(load) => (&mut load.task).await,
                        None => std::future::pending().await,
                    }
                }, if self.current_load.is_some() => {
                    // Take the current load
                    let load = self.current_load.take().unwrap();

                    // Handle the task result
                    match result {
                        Ok(service_result) => {
                            // Send the service call result to the composable node
                            let _ = load.request.response_tx.send(service_result);
                        }
                        Err(e) => {
                            // Task panicked
                            warn!(
                                "{}: LoadNode task panicked for {}: {:#}",
                                self.name, load.request.composable_name, e
                            );
                            let _ = load.request.response_tx.send(Err(eyre::eyre!("Task panicked: {:#}", e)));
                        }
                    }

                    // Start processing next load in queue
                    self.start_next_load();
                }

                // Check for shutdown signal
                _ = self.shutdown_rx.changed() => {
                    if *self.shutdown_rx.borrow() {
                        debug!("{}: Shutdown signal received, waiting for container to exit", self.name);

                        // Drain load queue before shutting down
                        self.drain_queue("Shutdown");

                        // On Unix, kill_process_group() already sent SIGTERM to all processes
                        // We just need to wait for this child to exit
                        #[cfg(unix)]
                        {
                            debug!("{}: Waiting for child to exit (killed by PGID)", self.name);
                            let _ = child.wait().await;
                        }
                        #[cfg(not(unix))]
                        {
                            // On non-Unix, we need to kill the child explicitly
                            child.kill().await.ok();
                            let _ = child.wait().await;
                        }

                        // Unregister from process registry
                        if let Some(ref registry) = self.process_registry {
                            if let Ok(mut reg) = registry.lock() {
                                reg.remove(&pid);
                            }
                        }

                        // Broadcast stopped state
                        let _ = self.container_state_tx.send(ContainerState::Stopped);

                        self.state = NodeState::Stopped { exit_code: None };
                        let _ = self.state_tx.send(StateEvent::Terminated {
                            name: self.name.clone(),
                        }).await;
                        return Ok(false); // Stop actor
                    }
                }
            }
        }
    }

    /// Handle the Respawning state
    async fn handle_respawning(&mut self, attempt: u32) -> Result<bool> {
        let delay = self.config.respawn_delay;
        info!(
            "{}: Respawning container after {:.1}s delay (attempt {})",
            self.name, delay, attempt
        );

        // Send respawning event
        let _ = self
            .state_tx
            .send(StateEvent::Respawning {
                name: self.name.clone(),
                attempt,
                delay,
            })
            .await;

        // Check if max attempts reached
        if let Some(max_attempts) = self.config.max_respawn_attempts {
            if attempt >= max_attempts {
                error!(
                    "{}: Max respawn attempts ({}) reached",
                    self.name, max_attempts
                );
                self.state = NodeState::Failed {
                    error: format!("Max respawn attempts ({}) reached", max_attempts),
                };

                // Broadcast failed state
                let _ = self.container_state_tx.send(ContainerState::Failed);

                let _ = self
                    .state_tx
                    .send(StateEvent::Failed {
                        name: self.name.clone(),
                        error: format!("Max respawn attempts ({}) reached", max_attempts),
                    })
                    .await;
                return Ok(false); // Stop actor
            }
        }

        // Wait for delay or control event or shutdown
        tokio::select! {
            _ = sleep(Duration::from_secs_f64(delay)) => {
                debug!("{}: Respawn delay complete, spawning container", self.name);
                self.state = NodeState::Pending;
                Ok(true) // Continue to spawn
            }
            Some(event) = self.control_rx.recv() => {
                match event {
                    ControlEvent::Stop => {
                        info!("{}: Stop requested during respawn delay", self.name);
                        self.state = NodeState::Stopped { exit_code: None };

                        // Broadcast stopped state
                        let _ = self.container_state_tx.send(ContainerState::Stopped);

                        let _ = self.state_tx.send(StateEvent::Terminated {
                            name: self.name.clone(),
                        }).await;
                        Ok(false) // Stop actor
                    }
                    _ => Ok(true) // Ignore other events during respawn
                }
            }
            _ = self.shutdown_rx.changed() => {
                if *self.shutdown_rx.borrow() {
                    info!("{}: Shutdown during respawn delay", self.name);
                    self.state = NodeState::Stopped { exit_code: None };

                    // Broadcast stopped state
                    let _ = self.container_state_tx.send(ContainerState::Stopped);

                    let _ = self.state_tx.send(StateEvent::Terminated {
                        name: self.name.clone(),
                    }).await;
                    Ok(false) // Stop actor
                } else {
                    Ok(true)
                }
            }
        }
    }

    /// Wait for all composable node actors to finish
    async fn wait_for_composable_actors(&mut self) {
        debug!(
            "{}: Waiting for {} composable node actors to finish",
            self.name,
            self.composable_actors.len()
        );

        // Take all handles and wait for them
        let handles = std::mem::take(&mut self.composable_actors);
        for handle in handles {
            match handle.task.await {
                Ok(Ok(())) => {
                    debug!("{}: Composable actor {} completed", self.name, handle.name);
                }
                Ok(Err(e)) => {
                    warn!(
                        "{}: Composable actor {} failed: {:#}",
                        self.name, handle.name, e
                    );
                }
                Err(e) if e.is_cancelled() => {
                    debug!("{}: Composable actor {} cancelled", self.name, handle.name);
                }
                Err(e) => {
                    error!(
                        "{}: Composable actor {} panicked: {:#}",
                        self.name, handle.name, e
                    );
                }
            }
        }
    }
}

impl MemberActor for ContainerActor {
    fn name(&self) -> &str {
        &self.name
    }

    async fn run(mut self) -> Result<()> {
        debug!("{}: Container actor started", self.name);

        loop {
            let should_continue = match self.state {
                NodeState::Pending => self.handle_pending().await?,
                NodeState::Running { .. } => {
                    // Extract child and pid by replacing state temporarily with Pending
                    // This avoids spawning a dummy "true" process
                    let (child, pid) = match std::mem::replace(&mut self.state, NodeState::Pending) {
                        NodeState::Running { child, pid } => (child, pid),
                        _ => unreachable!("State was Running before replace"),
                    };
                    self.handle_running(child, pid).await?
                }
                NodeState::Respawning { attempt, .. } => self.handle_respawning(attempt).await?,
                NodeState::Stopped { .. } => {
                    debug!("{}: Container stopped", self.name);
                    break;
                }
                NodeState::Failed { ref error } => {
                    error!("{}: Container failed: {}", self.name, error);
                    break;
                }
            };

            if !should_continue {
                break;
            }
        }

        // Wait for all composable nodes to finish
        self.wait_for_composable_actors().await;

        debug!("{}: Container actor finished", self.name);
        Ok(())
    }
}
