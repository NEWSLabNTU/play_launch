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

mod component_events;
mod composable_nodes;
mod process_lifecycle;
mod service_calls;

use super::{
    actor_traits::MemberActor,
    container_control::{ContainerControlEvent, CurrentUnload, LoadCompletion, LoadRequest},
    events::{ControlEvent, StateEvent},
    state::{ActorConfig, BlockReason, ComposableState, ContainerState, NodeState},
};
use crate::execution::context::NodeContext;
use eyre::Result;
use rclrs::IntoPrimitiveOptions;
use std::{
    collections::{HashMap, VecDeque},
    path::PathBuf,
    sync::{Arc, Mutex},
    time::Instant,
};
use tokio::{
    sync::{mpsc, watch},
    time::Duration,
};
use tracing::{debug, error, warn};

/// Timeout for LoadNode/UnloadNode service calls.
const SERVICE_CALL_TIMEOUT: Duration = Duration::from_secs(30);

/// Interval for checking whether nodes stuck in Loading state should be
/// promoted to Loaded (handles DDS event loss).
const LOADING_TIMEOUT: Duration = Duration::from_secs(10);

/// Interval for checking composable node loading timeouts.
const LOADING_CHECK_INTERVAL: Duration = Duration::from_secs(5);

/// Brief warmup delay after service readiness before issuing LoadNode calls.
const POST_SERVICE_READY_WARMUP: Duration = Duration::from_millis(200);

/// Interval for logging service-not-ready status during service discovery.
const SERVICE_NOT_READY_LOG_INTERVAL: Duration = Duration::from_secs(2);

/// Poll interval while waiting for a container's LoadNode service to appear.
const SERVICE_POLL_INTERVAL: Duration = Duration::from_millis(100);

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
    /// Output directory for per-node logging (isolated mode)
    pub output_dir: PathBuf,
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
}

/// Bundled configuration for constructing a [`ContainerActor`].
///
/// Groups the non-channel parameters that configure a container actor,
/// reducing argument count on [`ContainerActor::new`] and [`run_container`].
pub struct ContainerActorParams {
    pub context: NodeContext,
    pub config: ActorConfig,
    pub process_registry: Option<Arc<Mutex<HashMap<u32, PathBuf>>>>,
    pub ros_node: Option<Arc<rclrs::Node>>,
    pub shared_state: Arc<dashmap::DashMap<String, super::web_query::MemberState>>,
    pub use_component_events: bool,
}

/// Standalone async function for running a container (Phase 5)
///
/// This is the standalone function version that will be used with FuturesUnordered.
/// Note: For containers that need composable node supervision, use ContainerActor directly
/// to obtain container_state_rx before spawning.
pub async fn run_container(
    name: String,
    params: ContainerActorParams,
    control_rx: mpsc::Receiver<ControlEvent>,
    state_tx: mpsc::Sender<StateEvent>,
    shutdown_rx: watch::Receiver<bool>,
    load_control_rx: mpsc::Receiver<ContainerControlEvent>,
) -> Result<()> {
    // Create the actor and run it (wrapper approach for Phase 5)
    let actor = ContainerActor::new(
        name,
        params,
        control_rx,
        state_tx,
        shutdown_rx,
        load_control_rx,
    );
    actor.run().await
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
    /// ROS subscription for ComponentEvent messages (Phase 19.4a)
    component_event_sub: Option<rclrs::Subscription<play_launch_msgs::msg::ComponentEvent>>,
    /// Receiver for ComponentEvent messages bridged from ROS callback
    component_event_rx:
        Option<tokio::sync::mpsc::UnboundedReceiver<play_launch_msgs::msg::ComponentEvent>>,
    /// Queue of pending load requests
    pending_loads: VecDeque<LoadRequest>,
    /// Channel for load completion notifications (parallel dispatch)
    load_completion_tx: mpsc::UnboundedSender<LoadCompletion>,
    load_completion_rx: mpsc::UnboundedReceiver<LoadCompletion>,
    /// Currently processing unload
    current_unload: Option<CurrentUnload>,
    /// Shared state map for direct state updates
    shared_state: Arc<dashmap::DashMap<String, super::web_query::MemberState>>,
    /// Whether to subscribe to ComponentEvent topic (only when using play_launch_container)
    use_component_events: bool,
}

impl ContainerActor {
    /// Create a new container actor
    pub fn new(
        name: String,
        params: ContainerActorParams,
        control_rx: mpsc::Receiver<ControlEvent>,
        state_tx: mpsc::Sender<StateEvent>,
        shutdown_rx: watch::Receiver<bool>,
        load_control_rx: mpsc::Receiver<ContainerControlEvent>,
    ) -> Self {
        let (container_state_tx, _container_state_rx) = watch::channel(ContainerState::Pending);
        let (load_completion_tx, load_completion_rx) = mpsc::unbounded_channel();

        Self {
            name,
            context: params.context,
            config: params.config,
            state: NodeState::Pending,
            control_rx,
            state_tx,
            shutdown_rx,
            process_registry: params.process_registry,
            container_state_tx,
            composable_nodes: HashMap::new(),
            load_control_rx,
            ros_node: params.ros_node,
            load_client: None,
            unload_client: None,
            component_event_sub: None,
            component_event_rx: None,
            pending_loads: VecDeque::new(),
            load_completion_tx,
            load_completion_rx,
            current_unload: None,
            shared_state: params.shared_state,
            use_component_events: params.use_component_events,
        }
    }

    /// Get a receiver for container state (for composable nodes)
    pub fn container_state_rx(&self) -> watch::Receiver<ContainerState> {
        self.container_state_tx.subscribe()
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
        };

        self.composable_nodes.insert(name, entry);
    }

    /// Register a process PID in the shared process registry for I/O monitoring.
    fn register_process(&self, pid: u32) {
        if let Some(ref registry) = self.process_registry {
            if let Ok(mut reg) = registry.lock() {
                reg.insert(pid, self.config.output_dir.clone());
            }
        }
    }

    /// Unregister a process PID from the shared process registry.
    fn unregister_process(&self, pid: u32) {
        if let Some(ref registry) = self.process_registry {
            if let Ok(mut reg) = registry.lock() {
                reg.remove(&pid);
            }
        }
    }

    // State machine handlers (handle_pending, handle_running) and the
    // MemberActor::run() implementation form the core of this module.
    // Process lifecycle methods are in process_lifecycle.rs.
    // Composable node management is in composable_nodes.rs.
    // ComponentEvent handling is in component_events.rs.
    // ROS service calls are in service_calls.rs.

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
                self.register_process(pid);

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

                        // Phase 19.4a: Create ComponentEvent subscription
                        // Only when using play_launch_container (not stock containers)
                        if self.use_component_events {
                            let event_topic =
                                format!("{}/_container/component_events", self.full_node_name());
                            debug!(
                                "{}: Creating ComponentEvent subscription on {}",
                                self.name, event_topic
                            );

                            let (event_tx, event_rx) = tokio::sync::mpsc::unbounded_channel();
                            match ros_node.create_subscription(
                                event_topic
                                    .as_str()
                                    .reliable()
                                    .transient_local()
                                    .keep_last(100),
                                move |msg: play_launch_msgs::msg::ComponentEvent| {
                                    let _ = event_tx.send(msg);
                                },
                            ) {
                                Ok(sub) => {
                                    self.component_event_sub = Some(sub);
                                    self.component_event_rx = Some(event_rx);
                                    debug!(
                                        "{}: ComponentEvent subscription created successfully",
                                        self.name
                                    );
                                }
                                Err(e) => {
                                    warn!(
                                        "{}: Failed to create ComponentEvent subscription: {:#}",
                                        self.name, e
                                    );
                                }
                            }
                        } else {
                            debug!(
                                "{}: Skipping ComponentEvent subscription (stock container)",
                                self.name
                            );
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

    /// Handle the Running state.
    ///
    /// Runs a `tokio::select!` loop that multiplexes child exit, control events,
    /// load/unload requests, ComponentEvents, loading timeouts, and shutdown.
    async fn handle_running(&mut self, mut child: tokio::process::Child, pid: u32) -> Result<bool> {
        debug!("{}: Container running with PID {}", self.name, pid);

        let mut loading_timeout_interval = tokio::time::interval(LOADING_CHECK_INTERVAL);
        loading_timeout_interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        // Skip the first immediate tick
        loading_timeout_interval.tick().await;

        loop {
            tokio::select! {
                status = child.wait() => {
                    let exit_code = status.ok().and_then(|s| s.code());
                    return self.handle_child_exit(exit_code, pid).await;
                }

                Some(event) = self.control_rx.recv() => {
                    match event {
                        ControlEvent::Stop => {
                            return self.handle_stop_command(&mut child, pid).await;
                        }
                        ControlEvent::Restart => {
                            return self.handle_restart_command(&mut child, pid).await;
                        }
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
                        ControlEvent::ToggleComposableAutoLoad { name, enabled } => {
                            self.handle_toggle_composable_auto_load(&name, enabled).await;
                        }
                        _ => {
                            warn!("{}: Unhandled control event: {:?}", self.name, event);
                        }
                    }
                }

                Some(event) = self.load_control_rx.recv() => {
                    self.handle_load_control_event(event).await;
                }

                Some(completion) = self.load_completion_rx.recv() => {
                    self.handle_load_completion(completion).await;
                }

                result = async {
                    match &mut self.current_unload {
                        Some(unload) => (&mut unload.task).await,
                        None => std::future::pending().await,
                    }
                }, if self.current_unload.is_some() => {
                    let unload = self.current_unload.take().unwrap();
                    self.handle_unload_completion(result, unload.composable_name).await;
                }

                Some(event) = async {
                    match &mut self.component_event_rx {
                        Some(rx) => rx.recv().await,
                        None => std::future::pending().await,
                    }
                } => {
                    self.handle_component_event(event).await;
                }

                _ = loading_timeout_interval.tick() => {
                    self.check_loading_timeouts().await;
                }

                _ = self.shutdown_rx.changed() => {
                    if *self.shutdown_rx.borrow() {
                        return self.handle_running_shutdown(&mut child, pid).await;
                    }
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

        debug!("{}: Container actor finished", self.name);
        Ok(())
    }
}
