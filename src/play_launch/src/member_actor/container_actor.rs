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
    container_control::{
        ContainerControlEvent, CurrentLoad, CurrentUnload, LoadNodeResponse, LoadRequest,
    },
    events::{ControlEvent, StateEvent},
    state::{ActorConfig, BlockReason, ComposableState, ContainerState, NodeState},
};
use crate::execution::context::NodeContext;
use eyre::{Context as _, Result};
use std::{
    collections::{HashMap, VecDeque},
    path::PathBuf,
    sync::{Arc, Mutex},
    time::Instant,
};
use tokio::{
    process::Command,
    sync::{mpsc, watch},
    time::{sleep, Duration},
};
use tracing::{debug, error, info, warn};

/// Metadata for a composable node (Phase 12)
#[derive(Debug, Clone)]
pub struct ComposableNodeMetadata {
    /// Package containing the component
    pub package: String,
    /// Plugin name (fully qualified class name)
    pub plugin: String,
    /// ROS node name
    pub node_name: String,
    /// ROS namespace
    pub namespace: String,
    /// Remap rules
    pub remap_rules: Vec<String>,
    /// Parameters (key-value pairs)
    pub parameters: Vec<rcl_interfaces::msg::Parameter>,
    /// Extra arguments
    pub extra_args: Vec<rcl_interfaces::msg::Parameter>,
    /// Auto-load on container startup
    pub auto_load: bool,
}

/// Entry for a composable node managed by this container (Phase 12)
#[derive(Debug)]
struct ComposableNodeEntry {
    /// Metadata for this composable node
    metadata: ComposableNodeMetadata,
    /// Current state
    state: ComposableState,
    /// Unique ID from LoadNode response (if loaded)
    unique_id: Option<u64>,
    /// When the load started (for timeout detection)
    load_started_at: Option<Instant>,
    /// Whether a ListNodes verification has been requested
    list_nodes_requested: bool,
}

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
    shared_state: Arc<dashmap::DashMap<String, super::web_query::MemberState>>,
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
        shared_state,
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
    /// Container state broadcast to composable nodes (Phase 10)
    container_state_tx: watch::Sender<ContainerState>,
    /// Composable node actor handles (Phase 10, will be removed in Phase 12)
    #[allow(dead_code)]
    composable_actors: Vec<ComposableActorHandle>,

    // Phase 12: Container-managed composable nodes
    /// Composable nodes managed by this container (name -> entry)
    composable_nodes: HashMap<String, ComposableNodeEntry>,

    // LoadNode management (Phase 1: Container-managed loading)
    /// Channel to receive LoadNode requests from composable nodes
    load_control_rx: mpsc::Receiver<ContainerControlEvent>,
    /// ROS node for service calls (shared across all containers)
    ros_node: Option<Arc<rclrs::Node>>,
    /// ROS service client for LoadNode
    load_client: Option<rclrs::Client<composition_interfaces::srv::LoadNode>>,
    /// ROS service client for UnloadNode
    unload_client: Option<rclrs::Client<composition_interfaces::srv::UnloadNode>>,
    /// Queue of pending load requests
    pending_loads: VecDeque<LoadRequest>,
    /// Currently processing load
    current_load: Option<CurrentLoad>,
    /// Currently processing unload
    current_unload: Option<CurrentUnload>,
    /// Shared state map for direct state updates
    shared_state: Arc<dashmap::DashMap<String, super::web_query::MemberState>>,
}

impl ContainerActor {
    /// Convert ROS parameters to string tuples
    fn ros_params_to_strings(params: &[rcl_interfaces::msg::Parameter]) -> Vec<(String, String)> {
        params
            .iter()
            .map(|param| {
                let value_str = Self::parameter_value_to_string(&param.value);
                (param.name.clone(), value_str)
            })
            .collect()
    }

    /// Convert a ParameterValue to string representation
    fn parameter_value_to_string(value: &rcl_interfaces::msg::ParameterValue) -> String {
        use rcl_interfaces::msg::ParameterType;

        match value.type_ {
            ParameterType::PARAMETER_BOOL => value.bool_value.to_string(),
            ParameterType::PARAMETER_INTEGER => value.integer_value.to_string(),
            ParameterType::PARAMETER_DOUBLE => {
                // Format double to ensure it always has a decimal point
                // This prevents "1.0" from becoming "1" which would be interpreted as integer
                let s = value.double_value.to_string();
                if s.contains('.') || s.contains('e') || s.contains('E') {
                    s
                } else {
                    format!("{}.0", s)
                }
            }
            ParameterType::PARAMETER_STRING => value.string_value.clone(),
            ParameterType::PARAMETER_BYTE_ARRAY => format!("{:?}", value.byte_array_value),
            ParameterType::PARAMETER_BOOL_ARRAY => format!("{:?}", value.bool_array_value),
            ParameterType::PARAMETER_INTEGER_ARRAY => format!("{:?}", value.integer_array_value),
            ParameterType::PARAMETER_DOUBLE_ARRAY => format!("{:?}", value.double_array_value),
            ParameterType::PARAMETER_STRING_ARRAY => format!("{:?}", value.string_array_value),
            _ => String::new(),
        }
    }

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
        shared_state: Arc<dashmap::DashMap<String, super::web_query::MemberState>>,
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
            composable_nodes: HashMap::new(),
            load_control_rx,
            ros_node,
            load_client: None,
            unload_client: None,
            pending_loads: VecDeque::new(),
            current_load: None,
            current_unload: None,
            shared_state,
        }
    }

    /// Get a receiver for container state (for composable nodes)
    pub fn container_state_rx(&self) -> watch::Receiver<ContainerState> {
        self.container_state_tx.subscribe()
    }

    /// Add a composable node actor handle (Phase 10, deprecated in Phase 12)
    #[allow(dead_code)]
    pub fn add_composable_actor(&mut self, handle: ComposableActorHandle) {
        self.composable_actors.push(handle);
    }

    /// Add a composable node to be managed by this container (Phase 12)
    pub fn add_composable_node(&mut self, name: String, metadata: ComposableNodeMetadata) {
        let entry = ComposableNodeEntry {
            metadata,
            state: ComposableState::Blocked {
                reason: BlockReason::NotStarted,
            },
            unique_id: None,
            load_started_at: None,
            list_nodes_requested: false,
        };

        self.composable_nodes.insert(name, entry);
    }

    /// Update metadata.json to include composable node information (Phase 12)
    pub fn update_metadata_with_composables(&self) -> Result<()> {
        use std::fs;

        let metadata_path = self.config.output_dir.join("metadata.json");

        // Read existing metadata
        let mut metadata = if metadata_path.exists() {
            let metadata_str =
                fs::read_to_string(&metadata_path).context("Failed to read metadata.json")?;
            serde_json::from_str::<serde_json::Value>(&metadata_str)
                .context("Failed to parse metadata.json")?
        } else {
            serde_json::json!({})
        };

        // Add composable node information
        let composable_nodes_info: Vec<serde_json::Value> = self
            .composable_nodes
            .iter()
            .map(|(name, entry)| {
                serde_json::json!({
                    "name": name,
                    "package": entry.metadata.package,
                    "plugin": entry.metadata.plugin,
                    "node_name": entry.metadata.node_name,
                    "namespace": entry.metadata.namespace,
                    "auto_load": entry.metadata.auto_load,
                })
            })
            .collect();

        metadata["composable_nodes"] = serde_json::json!(composable_nodes_info);
        metadata["composable_node_count"] = serde_json::json!(self.composable_nodes.len());

        // Write back
        let updated_json =
            serde_json::to_string_pretty(&metadata).context("Failed to serialize metadata")?;
        fs::write(&metadata_path, updated_json).context("Failed to write metadata.json")?;

        debug!(
            "{}: Updated metadata.json with {} composable nodes",
            self.name,
            self.composable_nodes.len()
        );

        Ok(())
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
        let mut last_log_time = service_wait_start;

        loop {
            match client.service_is_ready() {
                Ok(true) => break,
                Ok(false) => {
                    // Service not ready yet, log periodically
                    if last_log_time.elapsed() > std::time::Duration::from_secs(2) {
                        debug!(
                            "{}: Still waiting for LoadNode service for {} ({}s elapsed)",
                            container_name,
                            params.composable_name,
                            service_wait_start.elapsed().as_secs()
                        );
                        last_log_time = std::time::Instant::now();
                    }
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

        // CRITICAL FIX: Add warmup delay after service becomes ready
        // Race condition: service_is_ready() returns true when service is registered in ROS graph,
        // but container executor may not be spinning/processing requests yet.
        // Without this delay, the service call can hang forever.
        // This is especially common after container restart when service registration happens
        // faster than executor startup.
        debug!(
            "{}: Waiting 200ms for container executor to start processing requests",
            container_name
        );
        tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;

        // Call the service (NO timeout - wait indefinitely)
        // Composable node actor handles timeout on its side
        debug!(
            "{}: Initiating LoadNode service call for {}",
            container_name, params.composable_name
        );

        let response_future: rclrs::Promise<composition_interfaces::srv::LoadNode_Response> =
            client
                .call(&ros_request)
                .context("Failed to initiate LoadNode service call")?;

        debug!(
            "{}: Waiting for LoadNode response for {}",
            container_name, params.composable_name
        );

        match response_future.await {
            Ok(response) => {
                // Calculate timing metrics
                let service_call_ms = start_time.elapsed().as_millis() as u64;
                let queue_wait_ms =
                    start_time.duration_since(params.request_time).as_millis() as u64;
                let total_duration_ms = params.request_time.elapsed().as_millis() as u64;

                if response.success {
                    debug!(
                        "{}: LoadNode SUCCESS for {}: unique_id={}, queue={}ms, service={}ms, total={}ms",
                        container_name,
                        params.composable_name,
                        response.unique_id,
                        queue_wait_ms,
                        service_call_ms,
                        total_duration_ms
                    );
                } else {
                    warn!(
                        "{}: LoadNode FAILED for {}: error='{}', queue={}ms, service={}ms, total={}ms",
                        container_name,
                        params.composable_name,
                        response.error_message,
                        queue_wait_ms,
                        service_call_ms,
                        total_duration_ms
                    );
                }

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
                    "{}: LoadNode service call ERROR for {}: {:?}",
                    container_name, params.composable_name, e
                );
                Err(eyre::eyre!("Service call failed: {:?}", e))
            }
        }
    }

    /// Call UnloadNode ROS service
    async fn call_unload_node_service(
        container_name: String,
        unload_client: Option<rclrs::Client<composition_interfaces::srv::UnloadNode>>,
        unique_id: u64,
    ) -> Result<super::container_control::UnloadNodeResponse> {
        use eyre::Context;

        // Check if client exists
        let client = unload_client.ok_or_else(|| {
            eyre::eyre!(
                "No UnloadNode service client available for container {}",
                container_name
            )
        })?;

        // Build UnloadNode request
        let ros_request = composition_interfaces::srv::UnloadNode_Request { unique_id };

        debug!(
            "{}: Calling UnloadNode service (unique_id: {})",
            container_name, unique_id
        );

        // Call the service
        let response_future: rclrs::Promise<composition_interfaces::srv::UnloadNode_Response> =
            client
                .call(&ros_request)
                .context("Failed to initiate UnloadNode service call")?;

        match response_future.await {
            Ok(response) => {
                debug!(
                    "{}: UnloadNode response: success={}, error_message={}",
                    container_name, response.success, response.error_message
                );

                Ok(super::container_control::UnloadNodeResponse {
                    success: response.success,
                    error_message: response.error_message.clone(),
                })
            }
            Err(e) => {
                warn!("{}: UnloadNode service call error: {:?}", container_name, e);
                Err(eyre::eyre!("Service call failed: {:?}", e))
            }
        }
    }

    /// Handle an UnloadNode request
    async fn handle_unload_request(
        &self,
        composable_name: String,
        unique_id: u64,
        response_tx: tokio::sync::oneshot::Sender<
            Result<super::container_control::UnloadNodeResponse>,
        >,
    ) {
        let container_name = self.name.clone();
        let unload_client = self.unload_client.clone();

        // Spawn service call as a task (so it doesn't block the actor)
        tokio::spawn(async move {
            let result =
                Self::call_unload_node_service(container_name, unload_client, unique_id).await;

            // Send response back
            if let Err(result) = response_tx.send(result) {
                warn!(
                    "Failed to send UnloadNode response for {}: {:?}",
                    composable_name, result
                );
            }
        });
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

        // Cancel current unload
        if let Some(unload) = self.current_unload.take() {
            debug!(
                "{}: Cancelling current unload for {}: {}",
                self.name, unload.composable_name, error
            );
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

    // Phase 12: Composable node control event handlers

    /// Handle LoadComposable control event
    async fn handle_load_composable(&mut self, name: &str) {
        debug!("{}: Handling LoadComposable for '{}'", self.name, name);

        // Check if composable node exists
        let entry = match self.composable_nodes.get_mut(name) {
            Some(e) => e,
            None => {
                warn!("{}: Composable node '{}' not found", self.name, name);
                return;
            }
        };

        // Check current state
        match &entry.state {
            ComposableState::Unloaded | ComposableState::Failed { .. } => {
                // Transition to Loading and start the load operation
                let started_at = Instant::now();
                entry.state = ComposableState::Loading { started_at };
                entry.load_started_at = Some(started_at);
                entry.list_nodes_requested = false;

                debug!("{}: Transitioning '{}' to Loading state", self.name, name);

                // Emit LoadStarted event
                let _ = self
                    .state_tx
                    .send(StateEvent::LoadStarted {
                        name: name.to_string(),
                    })
                    .await;

                // Update shared state directly for composable node
                self.shared_state
                    .insert(name.to_string(), super::web_query::MemberState::Loading);

                // Phase 12: Queue LoadNode request
                if let Some(entry) = self.composable_nodes.get(name) {
                    // Convert ROS parameters to string format
                    let parameters = Self::ros_params_to_strings(&entry.metadata.parameters);
                    let extra_args = Self::ros_params_to_strings(&entry.metadata.extra_args);

                    // Create LoadRequest and add to queue
                    let (response_tx, _response_rx) = tokio::sync::oneshot::channel();
                    let request = super::container_control::LoadRequest {
                        composable_name: name.to_string(),
                        package: entry.metadata.package.clone(),
                        plugin: entry.metadata.plugin.clone(),
                        node_name: entry.metadata.node_name.clone(),
                        node_namespace: entry.metadata.namespace.clone(),
                        remap_rules: entry.metadata.remap_rules.clone(),
                        parameters,
                        extra_args,
                        response_tx,
                        request_time: started_at,
                    };

                    self.pending_loads.push_back(request);
                    debug!(
                        "{}: Queued LoadNode request for '{}' (queue depth: {})",
                        self.name,
                        name,
                        self.pending_loads.len()
                    );

                    // Start processing if no load is currently in progress
                    if self.current_load.is_none() {
                        self.start_next_load();
                    }
                }
            }
            ComposableState::Loading { .. } => {
                debug!("{}: '{}' already loading, ignoring", self.name, name);
            }
            ComposableState::Unloading { .. } => {
                debug!(
                    "{}: '{}' is unloading, ignoring load request",
                    self.name, name
                );
            }
            ComposableState::Loaded { .. } => {
                debug!("{}: '{}' already loaded, ignoring", self.name, name);
            }
            ComposableState::Blocked { .. } => {
                warn!("{}: Cannot load '{}' - container blocked", self.name, name);
            }
        }
    }

    /// Handle UnloadComposable control event
    async fn handle_unload_composable(&mut self, name: &str) {
        debug!("{}: Handling UnloadComposable for '{}'", self.name, name);

        // Debug: Log all composable node states
        debug!("{}: Current composable node states:", self.name);
        for (node_name, node_entry) in self.composable_nodes.iter() {
            debug!("  - {}: {:?}", node_name, node_entry.state);
        }

        // Check if composable node exists
        let entry = match self.composable_nodes.get_mut(name) {
            Some(e) => e,
            None => {
                warn!(
                    "{}: Composable node '{}' not found in composable_nodes HashMap",
                    self.name, name
                );
                warn!(
                    "{}: Available composable nodes: {:?}",
                    self.name,
                    self.composable_nodes.keys().collect::<Vec<_>>()
                );
                return;
            }
        };

        // Check current state - only unload if loaded
        match &entry.state {
            ComposableState::Loaded { unique_id } => {
                let unique_id = *unique_id;
                let started_at = Instant::now();

                debug!(
                    "{}: Unloading '{}' (unique_id: {})",
                    self.name, name, unique_id
                );

                // Transition to Unloading state
                entry.state = ComposableState::Unloading { started_at };
                entry.load_started_at = Some(started_at);
                entry.list_nodes_requested = false;

                // Update shared_state to Unloading for Web UI
                self.shared_state
                    .insert(name.to_string(), super::web_query::MemberState::Unloading);

                // Spawn UnloadNode service call as a tracked task
                let container_name = self.name.clone();
                let unload_client = self.unload_client.clone();

                let task = tokio::spawn(async move {
                    Self::call_unload_node_service(container_name, unload_client, unique_id).await
                });

                // Track the unload operation (will be polled in select! loop)
                self.current_unload = Some(CurrentUnload {
                    composable_name: name.to_string(),
                    start_time: started_at,
                    task,
                });
            }
            _ => {
                warn!(
                    "{}: Cannot unload '{}' - not in Loaded state (current: {:?})",
                    self.name, name, entry.state
                );
            }
        }
    }

    /// Handle LoadAllComposables control event
    async fn handle_load_all_composables(&mut self) {
        debug!("{}: Handling LoadAllComposables", self.name);

        // Log state of all composable nodes for debugging
        let total_nodes = self.composable_nodes.len();
        let state_summary: std::collections::HashMap<String, usize> = self
            .composable_nodes
            .values()
            .fold(std::collections::HashMap::new(), |mut acc, entry| {
                let state_name = match &entry.state {
                    ComposableState::Blocked { .. } => "Blocked",
                    ComposableState::Unloaded => "Unloaded",
                    ComposableState::Loading { .. } => "Loading",
                    ComposableState::Unloading { .. } => "Unloading",
                    ComposableState::Loaded { .. } => "Loaded",
                    ComposableState::Failed { .. } => "Failed",
                };
                *acc.entry(state_name.to_string()).or_insert(0) += 1;
                acc
            });

        debug!(
            "{}: Composable node states: {} total ({:?})",
            self.name, total_nodes, state_summary
        );

        // Collect names of nodes to load (to avoid borrowing issues)
        let nodes_to_load: Vec<String> = self
            .composable_nodes
            .iter()
            .filter(|(_, entry)| {
                entry.metadata.auto_load
                    && matches!(
                        entry.state,
                        ComposableState::Unloaded | ComposableState::Failed { .. }
                    )
            })
            .map(|(name, _)| name.clone())
            .collect();

        if nodes_to_load.is_empty() {
            debug!(
                "{}: No composable nodes to load (all {} nodes already loaded or not marked for auto_load)",
                self.name, total_nodes
            );
        } else {
            debug!(
                "{}: Queueing {} composable nodes for loading: {:?}",
                self.name,
                nodes_to_load.len(),
                nodes_to_load
            );
        }

        // Load each node
        for name in nodes_to_load {
            self.handle_load_composable(&name).await;
        }
    }

    /// Handle UnloadAllComposables control event
    async fn handle_toggle_composable_auto_load(&mut self, name: &str, enabled: bool) {
        debug!(
            "{}: Handling ToggleComposableAutoLoad for '{}' (enabled: {})",
            self.name, name, enabled
        );

        // Find and update the composable node's auto_load setting
        if let Some(entry) = self.composable_nodes.get_mut(name) {
            entry.metadata.auto_load = enabled;
            debug!(
                "{}: Updated auto_load for '{}' to {}",
                self.name, name, enabled
            );
        } else {
            warn!(
                "{}: Cannot toggle auto_load for '{}': composable node not found",
                self.name, name
            );
        }
    }

    async fn handle_unload_all_composables(&mut self) {
        debug!("{}: Handling UnloadAllComposables", self.name);

        // Collect names of loaded nodes
        let nodes_to_unload: Vec<String> = self
            .composable_nodes
            .iter()
            .filter(|(_, entry)| matches!(entry.state, ComposableState::Loaded { .. }))
            .map(|(name, _)| name.clone())
            .collect();

        debug!(
            "{}: Unloading {} loaded composable nodes",
            self.name,
            nodes_to_unload.len()
        );

        // Unload each node
        for name in nodes_to_unload {
            self.handle_unload_composable(&name).await;
        }
    }

    /// Handle DiscoveredLoaded control event (from ListNodes verification)
    async fn handle_discovered_loaded(&mut self, unique_id: u64) {
        debug!(
            "{}: Handling DiscoveredLoaded (unique_id: {})",
            self.name, unique_id
        );

        // Find the composable node with this unique_id
        // Note: We need to match by unique_id since ListNodes returns actual loaded state
        let mut found = false;
        for (name, entry) in self.composable_nodes.iter_mut() {
            // Check if this node is in Loading state and matches the unique_id
            if let ComposableState::Loading { started_at } = &entry.state {
                // Transition to Loaded
                debug!(
                    "{}: Discovered '{}' as loaded (unique_id: {})",
                    self.name, name, unique_id
                );

                let load_duration = started_at.elapsed();
                entry.state = ComposableState::Loaded { unique_id };
                entry.unique_id = Some(unique_id);
                entry.load_started_at = None;

                // Emit LoadSucceeded event
                let full_node_name =
                    format!("{}/{}", entry.metadata.namespace, entry.metadata.node_name);
                let _ = self
                    .state_tx
                    .send(StateEvent::LoadSucceeded {
                        name: name.clone(),
                        full_node_name,
                        unique_id,
                    })
                    .await;

                // Update shared state directly for composable node
                self.shared_state.insert(
                    name.clone(),
                    super::web_query::MemberState::Loaded { unique_id },
                );

                debug!(
                    "{}: '{}' loaded successfully in {:.2}s",
                    self.name,
                    name,
                    load_duration.as_secs_f64()
                );

                found = true;
                break;
            }
        }

        if !found {
            debug!(
                "{}: No composable node in Loading state matches unique_id {}",
                self.name, unique_id
            );
        }
    }

    /// Check for composable nodes stuck in Loading state and trigger ListNodes verification
    ///
    /// Called periodically from the main loop to detect nodes that have exceeded
    /// the loading timeout and request verification via ListNodes query.
    async fn check_loading_timeouts(&mut self) {
        let timeout = Duration::from_secs(self.config.list_nodes_loading_timeout_secs);
        let now = Instant::now();

        for (name, entry) in self.composable_nodes.iter_mut() {
            // Check if node is in Loading state
            if let ComposableState::Loading { started_at } = &entry.state {
                let elapsed = now.duration_since(*started_at);

                // Check if timeout exceeded and ListNodes not yet requested
                if elapsed >= timeout && !entry.list_nodes_requested {
                    warn!(
                        "{}: Composable node '{}' stuck in Loading state for {:.1}s (timeout: {}s), requesting ListNodes verification",
                        self.name,
                        name,
                        elapsed.as_secs_f64(),
                        self.config.list_nodes_loading_timeout_secs
                    );

                    // Mark as requested to avoid spamming
                    entry.list_nodes_requested = true;

                    // Request ListNodes verification
                    let _ = self
                        .state_tx
                        .send(StateEvent::ListNodesRequested {
                            container_name: self.name.clone(),
                            requester: name.clone(),
                        })
                        .await;
                }
            }
        }
    }

    /// Transition all composable nodes to Blocked state
    ///
    /// Called when the container stops, fails, or shuts down to mark all
    /// composable nodes as unavailable.
    async fn transition_all_composables_to_blocked(&mut self, reason: BlockReason) {
        debug!(
            "{}: Transitioning all {} composable nodes to Blocked state (reason: {:?})",
            self.name,
            self.composable_nodes.len(),
            reason
        );

        for (name, entry) in self.composable_nodes.iter_mut() {
            // Only transition if not already blocked with this reason
            if entry.state != (ComposableState::Blocked { reason }) {
                entry.state = ComposableState::Blocked { reason };
                entry.unique_id = None;
                entry.load_started_at = None;
                entry.list_nodes_requested = false;

                // Emit Blocked event
                let _ = self
                    .state_tx
                    .send(StateEvent::Blocked {
                        name: name.clone(),
                        reason,
                    })
                    .await;

                // Update shared state directly for composable node
                use super::web_query::BlockReason as WebBlockReason;
                let web_reason = match reason {
                    BlockReason::NotStarted => WebBlockReason::ContainerNotStarted,
                    BlockReason::Stopped => WebBlockReason::ContainerStopped,
                    BlockReason::Failed => WebBlockReason::ContainerFailed,
                    BlockReason::Shutdown => WebBlockReason::Shutdown,
                };
                self.shared_state.insert(
                    name.clone(),
                    super::web_query::MemberState::Blocked { reason: web_reason },
                );
            }
        }
    }

    /// Handle the Pending state
    async fn handle_pending(&mut self) -> Result<bool> {
        debug!("{}: Spawning container process", self.name);

        // Drain any pending loads (container not running)
        self.drain_queue("Container not running");

        // Phase 12: Transition composable nodes to Blocked while container is starting
        self.transition_all_composables_to_blocked(BlockReason::NotStarted)
            .await;

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

                // Update shared state directly
                self.shared_state.insert(
                    self.name.clone(),
                    super::web_query::MemberState::Running { pid },
                );

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

                        // Also create UnloadNode service client
                        let unload_service_name =
                            format!("{}/_container/unload_node", self.full_node_name());
                        debug!(
                            "{}: Creating UnloadNode service client for {}",
                            self.name, unload_service_name
                        );

                        match ros_node.create_client::<composition_interfaces::srv::UnloadNode>(
                            &unload_service_name,
                        ) {
                            Ok(client) => {
                                self.unload_client = Some(client);
                                debug!(
                                    "{}: UnloadNode service client created successfully",
                                    self.name
                                );
                            }
                            Err(e) => {
                                warn!(
                                    "{}: Failed to create UnloadNode service client: {:#}",
                                    self.name, e
                                );
                                // Don't fail the container - composable nodes will get errors when trying to unload
                            }
                        }
                    } else {
                        warn!(
                            "{}: No ROS node available for LoadNode service calls",
                            self.name
                        );
                    }
                }

                // Phase 12: Transition blocked composable nodes to Unloaded state now that container is running
                let blocked_count = self
                    .composable_nodes
                    .values()
                    .filter(|e| matches!(e.state, ComposableState::Blocked { .. }))
                    .count();

                if blocked_count > 0 {
                    debug!(
                        "{}: Transitioning {} blocked composable nodes to Unloaded state (container now running)",
                        self.name, blocked_count
                    );
                }

                // Collect names of nodes transitioning from Blocked to Unloaded
                let unloaded_nodes: Vec<String> = self
                    .composable_nodes
                    .iter_mut()
                    .filter_map(|(name, entry)| {
                        if matches!(entry.state, ComposableState::Blocked { .. }) {
                            entry.state = ComposableState::Unloaded;
                            debug!(
                                "{}: Transitioned '{}' from Blocked to Unloaded",
                                self.name, name
                            );
                            Some(name.clone())
                        } else {
                            None
                        }
                    })
                    .collect();

                // Emit StateEvent::Unloaded and update shared state for each node
                for node_name in &unloaded_nodes {
                    let _ = self
                        .state_tx
                        .send(StateEvent::Unloaded {
                            name: node_name.clone(),
                        })
                        .await;

                    // Update shared state directly
                    self.shared_state
                        .insert(node_name.clone(), super::web_query::MemberState::Unloaded);

                    debug!(
                        "{}: Transitioned '{}' to Unloaded in shared state",
                        self.name, node_name
                    );
                }

                // Phase 12: Auto-load composable nodes marked with auto_load=true
                self.handle_load_all_composables().await;

                self.state = NodeState::Running { child, pid };
                Ok(true) // Continue running
            }
            Err(e) => {
                error!("{}: Failed to spawn container: {:#}", self.name, e);

                // Phase 12: Transition composable nodes to Blocked
                self.transition_all_composables_to_blocked(BlockReason::Failed)
                    .await;

                // Broadcast failed state to composable nodes
                let _ = self.container_state_tx.send(ContainerState::Failed);

                let _ = self
                    .state_tx
                    .send(StateEvent::Failed {
                        name: self.name.clone(),
                        error: e.to_string(),
                    })
                    .await;

                // Update shared state directly
                self.shared_state.insert(
                    self.name.clone(),
                    super::web_query::MemberState::Failed {
                        error: e.to_string(),
                    },
                );

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

        // Create timer for periodic timeout checks (every 5 seconds)
        let mut timeout_check_interval = tokio::time::interval(Duration::from_secs(5));
        timeout_check_interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

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

                    // Clear service client so it's recreated on restart
                    self.load_client = None;
                    self.unload_client = None;

                    // Phase 12: Transition composable nodes to Blocked
                    self.transition_all_composables_to_blocked(BlockReason::Failed).await;

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

                        // Update shared state directly
                        self.shared_state
                            .insert(self.name.clone(), super::web_query::MemberState::Stopped);

                        return Ok(true); // Keep actor alive to receive Start commands
                    }
                }

                // Handle control events
                Some(event) = self.control_rx.recv() => {
                    match event {
                        ControlEvent::Stop => {
                            debug!("{}: Received Stop command, killing container (PID: {})", self.name, pid);
                            child.kill().await.ok();

                            // Wait for process to exit
                            let _ = child.wait().await;
                            info!("{}: Container process terminated", self.name);

                            // Unregister from process registry
                            if let Some(ref registry) = self.process_registry {
                                if let Ok(mut reg) = registry.lock() {
                                    reg.remove(&pid);
                                }
                            }

                            // Drain load queue
                            self.drain_queue("Container stopped");

                            // Clear service client so it's recreated on restart
                            self.load_client = None;
                    self.unload_client = None;

                            // Phase 12: Transition composable nodes to Blocked
                            debug!("{}: Transitioning {} composable nodes to Blocked state (reason: Stopped)",
                                  self.name, self.composable_nodes.len());
                            self.transition_all_composables_to_blocked(BlockReason::Stopped).await;

                            // Broadcast stopped state
                            let _ = self.container_state_tx.send(ContainerState::Stopped);

                            self.state = NodeState::Stopped { exit_code: None };
                            let _ = self.state_tx.send(StateEvent::Terminated {
                                name: self.name.clone(),
                            }).await;

                            // Update shared state directly
                            self.shared_state
                                .insert(self.name.clone(), super::web_query::MemberState::Stopped);

                            return Ok(true); // Keep actor alive to receive Start commands
                        }
                        ControlEvent::Restart => {
                            debug!("{}: Received Restart command, killing and respawning container", self.name);
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

                            // Clear service client so it's recreated on restart
                            self.load_client = None;
                    self.unload_client = None;

                            // Phase 12: Transition composable nodes to Blocked (will be reloaded on restart)
                            self.transition_all_composables_to_blocked(BlockReason::Stopped).await;

                            // Transition to Pending to respawn
                            self.state = NodeState::Pending;
                            return Ok(true); // Continue to respawn
                        }
                        // Phase 12: Composable node control events
                        ControlEvent::LoadComposable { name } => {
                            self.handle_load_composable(&name).await;
                        }
                        ControlEvent::UnloadComposable { name } => {
                            self.handle_unload_composable(&name).await;
                        }
                        ControlEvent::LoadAllComposables => {
                            self.handle_load_all_composables().await;
                        }
                        ControlEvent::UnloadAllComposables => {
                            self.handle_unload_all_composables().await;
                        }
                        ControlEvent::DiscoveredLoaded { unique_id } => {
                            self.handle_discovered_loaded(unique_id).await;
                        }
                        ControlEvent::ToggleComposableAutoLoad { name, enabled } => {
                            self.handle_toggle_composable_auto_load(&name, enabled).await;
                        }
                        _ => {
                            warn!("{}: Unhandled control event: {:?}", self.name, event);
                        }
                    }
                }

                // Handle LoadNode and UnloadNode requests (Phase 1)
                Some(event) = self.load_control_rx.recv() => {
                    match event {
                        ContainerControlEvent::LoadNode {
                            composable_name,
                            package,
                            plugin,
                            node_name,
                            node_namespace,
                            remap_rules,
                            parameters,
                            extra_args,
                            response_tx,
                        } => {
                            debug!("{}: Received load request for {}", self.name, composable_name);

                            // Create load request
                            let request = LoadRequest {
                                composable_name,
                                package,
                                plugin,
                                node_name,
                                node_namespace,
                                remap_rules,
                                parameters,
                                extra_args,
                                response_tx,
                                request_time: std::time::Instant::now(),
                            };

                            // Queue the request
                            self.pending_loads.push_back(request);

                            // Start processing next load ONLY if not currently processing
                            // This spawns an async task so it doesn't block the select! loop
                            if self.current_load.is_none() {
                                self.start_next_load();
                            }
                        }
                        ContainerControlEvent::UnloadNode { composable_name, unique_id, response_tx } => {
                            debug!("{}: Received unload request for {} (unique_id: {})",
                                self.name, composable_name, unique_id);
                            // Handle unload request immediately (no queuing needed)
                            self.handle_unload_request(composable_name, unique_id, response_tx).await;
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
                    let composable_name = load.request.composable_name.clone();

                    // Handle the task result
                    match result {
                        Ok(service_result) => {
                            // Phase 12: Update composable node state directly
                            if let Some(entry) = self.composable_nodes.get_mut(&composable_name) {
                                match &service_result {
                                    Ok(response) if response.success => {
                                        entry.state = ComposableState::Loaded {
                                            unique_id: response.unique_id,
                                        };
                                        entry.unique_id = Some(response.unique_id);
                                        debug!(
                                            "{}: Successfully loaded composable node '{}' (unique_id: {})",
                                            self.name, composable_name, response.unique_id
                                        );

                                        // Update shared state directly for composable node
                                        self.shared_state.insert(
                                            composable_name.clone(),
                                            super::web_query::MemberState::Loaded { unique_id: response.unique_id },
                                        );

                                        // Emit LoadSucceeded event
                                        let _ = self.state_tx.send(StateEvent::LoadSucceeded {
                                            name: composable_name.clone(),
                                            full_node_name: response.full_node_name.clone(),
                                            unique_id: response.unique_id,
                                        }).await;
                                    }
                                    Ok(response) => {
                                        // LoadNode service returned failure
                                        entry.state = ComposableState::Failed {
                                            error: response.error_message.clone(),
                                        };
                                        warn!(
                                            "{}: Failed to load composable node '{}': {}",
                                            self.name, composable_name, response.error_message
                                        );

                                        // Update shared state directly for composable node
                                        self.shared_state.insert(
                                            composable_name.clone(),
                                            super::web_query::MemberState::Failed {
                                                error: response.error_message.clone(),
                                            },
                                        );

                                        // Emit Failed event
                                        let _ = self.state_tx.send(StateEvent::Failed {
                                            name: composable_name.clone(),
                                            error: response.error_message.clone(),
                                        }).await;
                                    }
                                    Err(e) => {
                                        // Service call error
                                        let error_msg = format!("{:#}", e);
                                        entry.state = ComposableState::Failed {
                                            error: error_msg.clone(),
                                        };
                                        warn!(
                                            "{}: LoadNode service call failed for '{}': {:#}",
                                            self.name, composable_name, e
                                        );

                                        // Update shared state directly for composable node
                                        self.shared_state.insert(
                                            composable_name.clone(),
                                            super::web_query::MemberState::Failed {
                                                error: error_msg.clone(),
                                            },
                                        );

                                        // Emit Failed event
                                        let _ = self.state_tx.send(StateEvent::Failed {
                                            name: composable_name.clone(),
                                            error: error_msg,
                                        }).await;
                                    }
                                }
                            }

                            // Legacy: Send response via channel (for old composable node actors, will be ignored in Phase 12)
                            let _ = load.request.response_tx.send(service_result);
                        }
                        Err(e) => {
                            // Task panicked
                            warn!(
                                "{}: LoadNode task panicked for {}: {:#}",
                                self.name, composable_name, e
                            );

                            // Phase 12: Update state to Failed
                            if let Some(entry) = self.composable_nodes.get_mut(&composable_name) {
                                entry.state = ComposableState::Failed {
                                    error: format!("Task panicked: {:#}", e),
                                };

                                // Emit Failed event
                                let _ = self.state_tx.send(StateEvent::Failed {
                                    name: composable_name.clone(),
                                    error: format!("Task panicked: {:#}", e),
                                }).await;
                            }

                            // Legacy: Send error via channel
                            let _ = load.request.response_tx.send(Err(eyre::eyre!("Task panicked: {:#}", e)));
                        }
                    }

                    // Start processing next load in queue
                    self.start_next_load();
                }

                // Poll for current UnloadNode service call completion
                result = async {
                    match &mut self.current_unload {
                        Some(unload) => (&mut unload.task).await,
                        None => std::future::pending().await,
                    }
                }, if self.current_unload.is_some() => {
                    // Take the current unload
                    let unload = self.current_unload.take().unwrap();
                    let composable_name = unload.composable_name.clone();

                    // Handle the task result
                    match result {
                        Ok(service_result) => {
                            // Phase 12: Update composable node state directly
                            if let Some(entry) = self.composable_nodes.get_mut(&composable_name) {
                                match &service_result {
                                    Ok(response) if response.success => {
                                        // Successfully unloaded
                                        entry.state = ComposableState::Unloaded;
                                        entry.unique_id = None;
                                        entry.load_started_at = None;

                                        debug!(
                                            "{}: Successfully unloaded composable node '{}'",
                                            self.name, composable_name
                                        );

                                        // Update shared state directly for composable node
                                        self.shared_state.insert(
                                            composable_name.clone(),
                                            super::web_query::MemberState::Unloaded,
                                        );

                                        // Emit Unloaded event
                                        let _ = self.state_tx.send(StateEvent::Unloaded {
                                            name: composable_name.clone(),
                                        }).await;
                                    }
                                    Ok(response) => {
                                        // UnloadNode service returned failure - keep in current state
                                        warn!(
                                            "{}: Failed to unload composable node '{}': {}",
                                            self.name, composable_name, response.error_message
                                        );
                                        // Transition back to Loaded state since unload failed
                                        // Note: We don't have unique_id anymore, so this is a problem
                                        // For now, keep in Unloading state (user can retry)
                                    }
                                    Err(e) => {
                                        // Service call error - keep in current state
                                        warn!(
                                            "{}: UnloadNode service call failed for '{}': {:#}",
                                            self.name, composable_name, e
                                        );
                                        // Keep in Unloading state for retry
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            // Task panicked
                            warn!(
                                "{}: UnloadNode task panicked for {}: {:#}",
                                self.name, composable_name, e
                            );
                        }
                    }
                }

                // Periodic timeout check for composable nodes stuck in Loading state
                _ = timeout_check_interval.tick() => {
                    self.check_loading_timeouts().await;
                }

                // Check for shutdown signal
                _ = self.shutdown_rx.changed() => {
                    if *self.shutdown_rx.borrow() {
                        debug!("{}: Shutdown signal received, waiting for container to exit", self.name);

                        // Drain load queue before shutting down
                        self.drain_queue("Shutdown");

                        // Phase 12: Transition composable nodes to Blocked
                        self.transition_all_composables_to_blocked(BlockReason::Shutdown).await;

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

                        // Update shared state directly
                        self.shared_state
                            .insert(self.name.clone(), super::web_query::MemberState::Stopped);

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

        // Update shared state directly
        self.shared_state.insert(
            self.name.clone(),
            super::web_query::MemberState::Respawning { attempt },
        );

        // Check if max attempts reached
        if let Some(max_attempts) = self.config.max_respawn_attempts {
            if attempt >= max_attempts {
                error!(
                    "{}: Max respawn attempts ({}) reached",
                    self.name, max_attempts
                );
                let error_msg = format!("Max respawn attempts ({}) reached", max_attempts);
                self.state = NodeState::Failed {
                    error: error_msg.clone(),
                };

                // Broadcast failed state
                let _ = self.container_state_tx.send(ContainerState::Failed);

                let _ = self
                    .state_tx
                    .send(StateEvent::Failed {
                        name: self.name.clone(),
                        error: error_msg.clone(),
                    })
                    .await;

                // Update shared state directly
                self.shared_state.insert(
                    self.name.clone(),
                    super::web_query::MemberState::Failed { error: error_msg },
                );

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
                        debug!("{}: Stop requested during respawn delay", self.name);

                        // Clear service client (should already be None, but be explicit)
                        self.load_client = None;
                    self.unload_client = None;

                        self.state = NodeState::Stopped { exit_code: None };

                        // Broadcast stopped state
                        let _ = self.container_state_tx.send(ContainerState::Stopped);

                        let _ = self.state_tx.send(StateEvent::Terminated {
                            name: self.name.clone(),
                        }).await;

                        // Update shared state directly
                        self.shared_state
                            .insert(self.name.clone(), super::web_query::MemberState::Stopped);

                        Ok(true) // Keep actor alive to receive Start commands
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

                    // Update shared state directly
                    self.shared_state
                        .insert(self.name.clone(), super::web_query::MemberState::Stopped);

                    Ok(false) // Stop actor
                } else {
                    Ok(true)
                }
            }
        }
    }

    /// Handle stopped/failed state - wait for control events
    ///
    /// Returns false if the actor should terminate (shutdown), true to continue
    async fn handle_stopped(&mut self) -> Result<bool> {
        // Wait for control event or shutdown signal
        tokio::select! {
            Some(event) = self.control_rx.recv() => {
                match event {
                    ControlEvent::Start => {
                        debug!("{}: Start requested", self.name);

                        // Transition to Pending to spawn container
                        match &self.state {
                            NodeState::Stopped { .. } | NodeState::Failed { .. } => {
                                self.state = NodeState::Pending;
                                debug!("{}: Transitioning to Pending for spawn", self.name);
                                Ok(true) // Continue to spawn
                            }
                            _ => {
                                // Should not happen, but handle gracefully
                                warn!("{}: Start requested in unexpected state", self.name);
                                Ok(true)
                            }
                        }
                    }
                    ControlEvent::Restart => {
                        debug!("{}: Restart requested", self.name);

                        // Transition to Pending to spawn container
                        self.state = NodeState::Pending;
                        Ok(true) // Continue to spawn
                    }
                    ControlEvent::Stop => {
                        // Already stopped, ignore
                        debug!("{}: Stop requested while already stopped", self.name);
                        Ok(true)
                    }
                    _ => {
                        warn!("{}: Unhandled control event in Stopped state: {:?}", self.name, event);
                        Ok(true)
                    }
                }
            }

            _ = self.shutdown_rx.changed() => {
                if *self.shutdown_rx.borrow() {
                    debug!("{}: Shutdown signal received in Stopped/Failed state", self.name);
                    Ok(false) // Terminate
                } else {
                    Ok(true) // Continue
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
                    let (child, pid) = match std::mem::replace(&mut self.state, NodeState::Pending)
                    {
                        NodeState::Running { child, pid } => (child, pid),
                        _ => unreachable!("State was Running before replace"),
                    };
                    self.handle_running(child, pid).await?
                }
                NodeState::Respawning { attempt, .. } => self.handle_respawning(attempt).await?,
                NodeState::Stopped { .. } | NodeState::Failed { .. } => {
                    // Keep actor alive to receive Start/Restart commands from Web UI
                    self.handle_stopped().await?
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
