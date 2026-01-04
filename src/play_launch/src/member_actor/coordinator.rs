//! Coordinator for managing member actors
//!
//! Split into two parts:
//! - MemberCoordinatorBuilder: Collects member definitions
//! - After spawning: MemberHandle (external control) + MemberRunner (completion waiting)

use super::{
    events::{ControlEvent, StateEvent},
    web_query::{HealthSummary, MemberState, MemberSummary, MemberType},
};
use eyre::{Context, Result};
use std::{collections::HashMap, path::PathBuf, sync::Arc};
use tokio::{
    sync::{mpsc, watch, Mutex},
    task::JoinHandle,
};
use tracing::debug;

/// Metadata about a member for web UI queries
#[derive(Clone)]
pub struct MemberMetadata {
    pub name: String,
    pub member_type: MemberType,
    pub package: Option<String>,
    pub executable: String,
    pub namespace: Option<String>,
    pub target_container: Option<String>,
    pub output_dir: PathBuf,
    pub respawn_enabled: Option<bool>,
    pub respawn_delay: Option<f64>,
    pub exec_name: Option<String>,
    pub node_name: Option<String>,
}

/// Definition of a regular node to be spawned
struct RegularNodeDefinition {
    name: String,
    context: crate::execution::context::NodeContext,
    config: super::state::ActorConfig,
    process_registry: Option<Arc<std::sync::Mutex<HashMap<u32, PathBuf>>>>,
    metadata: MemberMetadata,
}

/// Definition of a container to be spawned
struct ContainerDefinition {
    name: String,
    context: crate::execution::context::NodeContext,
    config: super::state::ActorConfig,
    process_registry: Option<Arc<std::sync::Mutex<HashMap<u32, PathBuf>>>>,
    metadata: MemberMetadata,
    /// Channel to send container state receiver back to caller
    state_tx: Option<tokio::sync::oneshot::Sender<watch::Receiver<super::state::ContainerState>>>,
}

/// Definition of a composable node to be spawned
struct ComposableNodeDefinition {
    name: String,
    context: crate::execution::context::ComposableNodeContext,
    config: super::composable_node_actor::ComposableActorConfig,
    target_container_name: String, // Will be matched with container during spawn()
    metadata: MemberMetadata,
}

/// Builder for collecting member definitions before spawning
pub struct MemberCoordinatorBuilder {
    regular_nodes: Vec<RegularNodeDefinition>,
    containers: Vec<ContainerDefinition>,
    composable_nodes: Vec<ComposableNodeDefinition>,
}

impl MemberCoordinatorBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            regular_nodes: Vec::new(),
            containers: Vec::new(),
            composable_nodes: Vec::new(),
        }
    }

    /// Add a regular node to be spawned later
    pub fn add_regular_node(
        &mut self,
        name: String,
        context: crate::execution::context::NodeContext,
        config: super::state::ActorConfig,
        process_registry: Option<Arc<std::sync::Mutex<HashMap<u32, PathBuf>>>>,
    ) {
        let metadata = MemberMetadata {
            name: name.clone(),
            member_type: MemberType::Node,
            package: context.record.package.clone(),
            executable: context.record.executable.clone(),
            namespace: context.record.namespace.clone(),
            target_container: None,
            output_dir: context.output_dir.clone(),
            respawn_enabled: context.record.respawn,
            respawn_delay: context.record.respawn_delay,
            exec_name: context.record.exec_name.clone(),
            node_name: context.record.name.clone(),
        };

        self.regular_nodes.push(RegularNodeDefinition {
            name,
            context,
            config,
            process_registry,
            metadata,
        });
    }

    /// Add a container to be spawned later
    /// Returns a oneshot receiver that will provide the container state watch receiver
    pub fn add_container(
        &mut self,
        name: String,
        context: crate::execution::context::NodeContext,
        config: super::state::ActorConfig,
        process_registry: Option<Arc<std::sync::Mutex<HashMap<u32, PathBuf>>>>,
    ) -> tokio::sync::oneshot::Receiver<watch::Receiver<super::state::ContainerState>> {
        let (state_tx, state_rx) = tokio::sync::oneshot::channel();

        let metadata = MemberMetadata {
            name: name.clone(),
            member_type: MemberType::Container,
            package: context.record.package.clone(),
            executable: context.record.executable.clone(),
            namespace: context.record.namespace.clone(),
            target_container: None,
            output_dir: context.output_dir.clone(),
            respawn_enabled: context.record.respawn,
            respawn_delay: context.record.respawn_delay,
            exec_name: context.record.exec_name.clone(),
            node_name: context.record.name.clone(),
        };

        self.containers.push(ContainerDefinition {
            name,
            context,
            config,
            process_registry,
            metadata,
            state_tx: Some(state_tx),
        });

        state_rx
    }

    /// Add a composable node to be spawned later
    /// The container must be added before calling spawn() so it can be matched
    pub fn add_composable_node(
        &mut self,
        name: String,
        context: crate::execution::context::ComposableNodeContext,
        config: super::composable_node_actor::ComposableActorConfig,
    ) {
        let target_container_name = context.record.target_container_name.clone();

        let metadata = MemberMetadata {
            name: name.clone(),
            member_type: MemberType::ComposableNode,
            package: Some(context.record.package.clone()),
            executable: String::new(),
            namespace: Some(context.record.namespace.clone()),
            target_container: Some(target_container_name.clone()),
            output_dir: context.output_dir.clone(),
            respawn_enabled: None,
            respawn_delay: None,
            exec_name: None,
            node_name: Some(context.record.node_name.clone()),
        };

        self.composable_nodes.push(ComposableNodeDefinition {
            name,
            context,
            config,
            target_container_name,
            metadata,
        });
    }

    /// Spawn all members and return handle + runner
    pub async fn spawn(self) -> (MemberHandle, MemberRunner) {
        let (state_tx, state_rx) = mpsc::channel(100);
        let (shutdown_tx, shutdown_rx) = watch::channel(false);

        let mut tasks = HashMap::new();
        let mut control_channels = HashMap::new();
        let mut metadata_map = HashMap::new();

        // Spawn regular nodes
        for def in self.regular_nodes {
            let (control_tx, control_rx) = mpsc::channel(10);

            let actor = super::regular_node_actor::RegularNodeActor::new(
                def.name.clone(),
                def.context,
                def.config,
                control_rx,
                state_tx.clone(),
                shutdown_rx.clone(),
                def.process_registry,
            );

            let task = tokio::spawn(async move {
                use super::actor_traits::MemberActor;
                actor.run().await
            });

            tasks.insert(def.name.clone(), task);
            control_channels.insert(def.name.clone(), control_tx);
            metadata_map.insert(def.name.clone(), def.metadata);
        }

        // Phase 2: Create ONE shared ROS node and executor for all containers
        // The executor spins in a dedicated thread to process LoadNode service callbacks
        let shared_ros_node = if !self.containers.is_empty() {
            use std::sync::mpsc as std_mpsc;
            let (node_tx, node_rx) = std_mpsc::channel();
            let shutdown_rx_executor = shutdown_rx.clone();

            // Spawn a dedicated thread for the ROS executor
            // This thread will spin the executor continuously
            let executor_thread = std::thread::spawn(move || {
                use rclrs::CreateBasicExecutor;
                use tracing::{debug, error};

                debug!("Starting ROS executor thread for container LoadNode service calls");

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
                let node = match executor.create_node("play_launch_containers") {
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
                    if shutdown_rx_executor.has_changed().is_ok() && *shutdown_rx_executor.borrow()
                    {
                        debug!("ROS executor thread received shutdown signal");
                        break;
                    }

                    // Spin once
                    executor.spin(rclrs::SpinOptions::spin_once());

                    // Small sleep to prevent busy-waiting
                    std::thread::sleep(std::time::Duration::from_millis(10));
                }

                debug!("ROS executor thread shutting down");
            });

            // The executor thread will run until shutdown signal is received
            // We don't need to explicitly join it - it will clean up automatically
            // when the shutdown signal is sent
            std::mem::forget(executor_thread); // Don't block on join

            // Wait for the node to be created
            match node_rx.recv() {
                Ok(node) => {
                    debug!("Shared ROS node created for all containers");
                    Some(node)
                }
                Err(e) => {
                    tracing::error!("Failed to receive node from executor thread: {:#}", e);
                    tracing::warn!("Containers will not be able to load composable nodes");
                    None
                }
            }
        } else {
            None
        };

        // Spawn containers and collect their state receivers and load control channels
        let mut container_state_map = HashMap::new();
        let mut container_load_control_map = HashMap::new();
        for def in self.containers {
            // Build the full node name BEFORE moving def.context
            // This matches what composable nodes use in target_container_name
            let container_name = {
                let namespace = def.context.record.namespace.as_deref().unwrap_or("/");
                let name = def
                    .context
                    .record
                    .name
                    .as_deref()
                    .expect("Container must have name");

                // Build full node name: namespace/name
                // If namespace is "/", result is "/name"
                // If namespace is "/foo", result is "/foo/name"
                if namespace == "/" {
                    format!("/{}", name)
                } else if namespace.ends_with('/') {
                    format!("{}{}", namespace, name)
                } else {
                    format!("{}/{}", namespace, name)
                }
            };

            let (control_tx, control_rx) = mpsc::channel(10);

            // Phase 2: Create load control channel for composable nodes
            let (load_control_tx, load_control_rx) =
                mpsc::channel::<super::container_control::ContainerControlEvent>(10);

            // Phase 2: Use the shared ROS node for this container
            let actor = super::container_actor::ContainerActor::new(
                def.name.clone(),
                def.context,
                def.config,
                control_rx,
                state_tx.clone(),
                shutdown_rx.clone(),
                def.process_registry,
                load_control_rx,
                shared_ros_node.clone(),
            );

            // Get container state receiver before spawning
            let container_state_rx = actor.container_state_rx();

            // Send container state receiver back to caller if requested
            if let Some(tx) = def.state_tx {
                let _ = tx.send(container_state_rx.clone());
            }

            // Store for composable nodes to use
            container_state_map.insert(container_name.clone(), container_state_rx);
            container_load_control_map.insert(container_name, load_control_tx);

            let task = tokio::spawn(async move {
                use super::actor_traits::MemberActor;
                actor.run().await
            });

            tasks.insert(def.name.clone(), task);
            control_channels.insert(def.name.clone(), control_tx);
            metadata_map.insert(def.name.clone(), def.metadata);
        }

        // Spawn composable nodes, matching with container state receivers and load control channels
        for def in self.composable_nodes {
            // Find the container state receiver and load control channel for this composable node
            let container_state_rx = container_state_map.get(&def.target_container_name);
            let load_control_tx = container_load_control_map.get(&def.target_container_name);

            if let (Some(container_state_rx), Some(load_control_tx)) =
                (container_state_rx, load_control_tx)
            {
                let (control_tx, control_rx) = mpsc::channel(10);

                let actor = super::composable_node_actor::ComposableNodeActor::new(
                    def.name.clone(),
                    def.context,
                    def.config,
                    control_rx,
                    state_tx.clone(),
                    container_state_rx.clone(),
                    shutdown_rx.clone(),
                    load_control_tx.clone(),
                );

                let task = tokio::spawn(async move {
                    use super::actor_traits::MemberActor;
                    actor.run().await
                });

                tasks.insert(def.name.clone(), task);
                control_channels.insert(def.name.clone(), control_tx);
                metadata_map.insert(def.name.clone(), def.metadata);
            } else {
                tracing::warn!(
                    "Container '{}' not found for composable node '{}', skipping",
                    def.target_container_name,
                    def.name
                );
            }
        }

        // state_tx and shutdown_rx are dropped here (actors have their clones)

        // Pre-populate state cache with all member names (using RwLock per MemberState)
        let mut state_entries = HashMap::new();
        for name in metadata_map.keys() {
            state_entries.insert(
                name.clone(),
                Arc::new(tokio::sync::RwLock::new(MemberState::Pending)),
            );
        }
        let state_cache = Arc::new(state_entries);

        let handle = MemberHandle {
            control_channels,
            metadata: metadata_map,
            state_cache: state_cache.clone(),
            shutdown_tx,
        };

        let runner = MemberRunner {
            tasks,
            state_rx,
            state_cache,
        };

        (handle, runner)
    }

    /// Get count of members to be spawned
    pub fn member_count(&self) -> usize {
        self.regular_nodes.len() + self.containers.len() + self.composable_nodes.len()
    }
}

impl Default for MemberCoordinatorBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// External handle for controlling and querying members
/// Wrap in Arc<MemberHandle> for sharing (not Clone-able itself)
pub struct MemberHandle {
    /// Control channels for sending commands to actors
    control_channels: HashMap<String, mpsc::Sender<ControlEvent>>,
    /// Member metadata (immutable after spawning)
    metadata: HashMap<String, MemberMetadata>,
    /// Cached state updated by runner (Arc for sharing with runner, RwLock per MemberState)
    state_cache: Arc<HashMap<String, Arc<tokio::sync::RwLock<MemberState>>>>,
    /// Shutdown signal broadcaster
    shutdown_tx: watch::Sender<bool>,
}

impl MemberHandle {
    /// Get list of all members with current state
    pub async fn list_members(&self) -> Vec<MemberSummary> {
        let mut summaries = Vec::new();

        for (name, meta) in self.metadata.iter() {
            // Read state from RwLock
            let state = if let Some(state_lock) = self.state_cache.get(name) {
                state_lock.read().await.clone()
            } else {
                MemberState::Pending
            };

            let pid = match &state {
                MemberState::Running { pid } => Some(*pid),
                _ => None,
            };

            // Read stderr info from log files
            let stderr_path = meta.output_dir.join("err");
            let (stderr_last_modified, stderr_size, stderr_preview) =
                if let Ok(metadata) = std::fs::metadata(&stderr_path) {
                    let size = metadata.len();
                    let modified = metadata.modified().ok().and_then(|t| {
                        t.duration_since(std::time::UNIX_EPOCH)
                            .ok()
                            .map(|d| d.as_secs())
                    });

                    // Read last 5 lines for preview
                    let preview = if size > 0 {
                        read_last_n_lines(&stderr_path, 5).ok()
                    } else {
                        None
                    };

                    (modified, size, preview)
                } else {
                    (None, 0, None)
                };

            summaries.push(MemberSummary {
                name: meta.name.clone(),
                member_type: meta.member_type,
                state,
                pid,
                package: meta.package.clone(),
                executable: meta.executable.clone(),
                namespace: meta.namespace.clone(),
                target_container: meta.target_container.clone(),
                is_container: meta.member_type == MemberType::Container,
                exec_name: meta.exec_name.clone(),
                node_name: meta.node_name.clone(),
                stderr_last_modified,
                stderr_size,
                stderr_preview,
                respawn_enabled: meta.respawn_enabled,
                respawn_delay: meta.respawn_delay,
                output_dir: meta.output_dir.clone(),
            });
        }

        summaries
    }

    /// Get detailed state for a specific member
    pub async fn get_member_state(&self, name: &str) -> Option<MemberSummary> {
        let meta = self.metadata.get(name)?;

        // Read state from RwLock
        let state = if let Some(state_lock) = self.state_cache.get(name) {
            state_lock.read().await.clone()
        } else {
            MemberState::Pending
        };

        let pid = match &state {
            MemberState::Running { pid } => Some(*pid),
            _ => None,
        };

        // Read stderr info from log files
        let stderr_path = meta.output_dir.join("err");
        let (stderr_last_modified, stderr_size, stderr_preview) =
            if let Ok(metadata) = std::fs::metadata(&stderr_path) {
                let size = metadata.len();
                let modified = metadata.modified().ok().and_then(|t| {
                    t.duration_since(std::time::UNIX_EPOCH)
                        .ok()
                        .map(|d| d.as_secs())
                });

                // Read last 5 lines for preview
                let preview = if size > 0 {
                    read_last_n_lines(&stderr_path, 5).ok()
                } else {
                    None
                };

                (modified, size, preview)
            } else {
                (None, 0, None)
            };

        Some(MemberSummary {
            name: meta.name.clone(),
            member_type: meta.member_type,
            state,
            pid,
            package: meta.package.clone(),
            executable: meta.executable.clone(),
            namespace: meta.namespace.clone(),
            target_container: meta.target_container.clone(),
            is_container: meta.member_type == MemberType::Container,
            exec_name: meta.exec_name.clone(),
            node_name: meta.node_name.clone(),
            stderr_last_modified,
            stderr_size,
            stderr_preview,
            respawn_enabled: meta.respawn_enabled,
            respawn_delay: meta.respawn_delay,
            output_dir: meta.output_dir.clone(),
        })
    }

    /// Get health summary statistics
    pub async fn get_health_summary(&self) -> HealthSummary {
        let mut summary = HealthSummary::default();

        for (name, meta) in self.metadata.iter() {
            // Read state from RwLock
            let state = if let Some(state_lock) = self.state_cache.get(name) {
                state_lock.read().await.clone()
            } else {
                MemberState::Pending
            };

            // Count by member type
            match meta.member_type {
                MemberType::Node => {
                    summary.nodes_total += 1;
                    match &state {
                        MemberState::Running { .. } | MemberState::Respawning { .. } => {
                            summary.nodes_running += 1;
                            summary.processes_running += 1;
                        }
                        MemberState::Stopped => {
                            summary.nodes_stopped += 1;
                            summary.processes_stopped += 1;
                        }
                        MemberState::Failed { .. } => {
                            summary.nodes_failed += 1;
                            summary.processes_stopped += 1;
                        }
                        MemberState::Pending => {
                            summary.processes_stopped += 1;
                        }
                        _ => {}
                    }
                }
                MemberType::Container => {
                    summary.containers_total += 1;
                    match &state {
                        MemberState::Running { .. } | MemberState::Respawning { .. } => {
                            summary.containers_running += 1;
                            summary.processes_running += 1;
                        }
                        MemberState::Stopped => {
                            summary.containers_stopped += 1;
                            summary.processes_stopped += 1;
                        }
                        MemberState::Failed { .. } => {
                            summary.containers_failed += 1;
                            summary.processes_stopped += 1;
                        }
                        MemberState::Pending => {
                            summary.processes_stopped += 1;
                        }
                        _ => {}
                    }
                }
                MemberType::ComposableNode => {
                    summary.composable_total += 1;
                    match &state {
                        MemberState::Loaded { .. } => {
                            summary.composable_loaded += 1;
                        }
                        MemberState::Failed { .. } => {
                            summary.composable_failed += 1;
                        }
                        MemberState::Pending | MemberState::Loading => {
                            summary.composable_pending += 1;
                        }
                        _ => {}
                    }
                }
            }

            // Count noisy nodes (stderr > 10KB)
            let stderr_path = meta.output_dir.join("err");
            if let Ok(metadata) = std::fs::metadata(&stderr_path) {
                if metadata.len() > 10 * 1024 {
                    summary.noisy += 1;
                }
            }
        }

        summary
    }

    /// Send a control event to a specific actor
    pub async fn send_control(&self, name: &str, event: ControlEvent) -> Result<()> {
        let control_tx = self
            .control_channels
            .get(name)
            .ok_or_else(|| eyre::eyre!("Actor not found: {}", name))?;

        control_tx
            .send(event)
            .await
            .context("Failed to send control event to actor")
    }

    /// Start a member (send Start control event)
    pub async fn start_member(&self, name: &str) -> Result<()> {
        self.send_control(name, ControlEvent::Start).await
    }

    /// Stop a member (send Stop control event)
    pub async fn stop_member(&self, name: &str) -> Result<()> {
        self.send_control(name, ControlEvent::Stop).await
    }

    /// Restart a member (send Restart control event)
    pub async fn restart_member(&self, name: &str) -> Result<()> {
        self.send_control(name, ControlEvent::Restart).await
    }

    /// Toggle respawn for a member
    pub async fn toggle_respawn(&self, name: &str, enabled: bool) -> Result<()> {
        self.send_control(name, ControlEvent::ToggleRespawn(enabled))
            .await
    }

    /// Broadcast shutdown signal to all actors
    pub fn shutdown(&self) -> Result<()> {
        self.shutdown_tx
            .send(true)
            .context("Failed to send shutdown signal")
    }

    /// Get the number of registered members
    pub fn member_count(&self) -> usize {
        self.metadata.len()
    }

    /// Check if a specific member is registered
    pub fn has_member(&self, name: &str) -> bool {
        self.metadata.contains_key(name)
    }
}

/// Runner that waits for all actors to complete
/// Takes mut self - no Arc<Mutex> needed!
pub struct MemberRunner {
    /// Task handles for all actors
    tasks: HashMap<String, JoinHandle<Result<()>>>,
    /// Receiver for state events from actors
    state_rx: mpsc::Receiver<StateEvent>,
    /// Shared state cache (updated for web UI queries, RwLock per MemberState)
    state_cache: Arc<HashMap<String, Arc<tokio::sync::RwLock<MemberState>>>>,
}

impl MemberRunner {
    /// Static helper to update state (for use in wait_for_completion)
    async fn update_state_static(
        event: &StateEvent,
        state_cache: &Arc<HashMap<String, Arc<tokio::sync::RwLock<MemberState>>>>,
    ) {
        let name = match event {
            StateEvent::Started { name, .. } => name,
            StateEvent::Exited { name, .. } => name,
            StateEvent::Respawning { name, .. } => name,
            StateEvent::Terminated { name } => name,
            StateEvent::Failed { name, .. } => name,
            StateEvent::LoadStarted { name } => name,
            StateEvent::LoadSucceeded { name, .. } => name,
            StateEvent::LoadFailed { name, .. } => name,
            StateEvent::Blocked { name, .. } => name,
        };

        // Get the RwLock for this specific member
        if let Some(state_lock) = state_cache.get(name) {
            let mut state = state_lock.write().await;

            *state = match event {
                StateEvent::Started { pid, .. } => MemberState::Running { pid: *pid },
                StateEvent::Exited { exit_code, .. } => {
                    if exit_code.is_some() {
                        MemberState::Failed {
                            error: format!("Exited with code {:?}", exit_code),
                        }
                    } else {
                        MemberState::Stopped
                    }
                }
                StateEvent::Respawning { attempt, .. } => {
                    MemberState::Respawning { attempt: *attempt }
                }
                StateEvent::Terminated { .. } => MemberState::Stopped,
                StateEvent::Failed { error, .. } => MemberState::Failed {
                    error: error.clone(),
                },
                StateEvent::LoadStarted { .. } => MemberState::Loading,
                StateEvent::LoadSucceeded { unique_id, .. } => MemberState::Loaded {
                    unique_id: *unique_id,
                },
                StateEvent::LoadFailed { error, .. } => MemberState::Failed {
                    error: error.clone(),
                },
                StateEvent::Blocked { reason, .. } => {
                    use super::{
                        state::BlockReason as ActorBlockReason,
                        web_query::BlockReason as WebBlockReason,
                    };

                    let web_reason = match reason {
                        ActorBlockReason::NotStarted => WebBlockReason::ContainerNotStarted,
                        ActorBlockReason::Stopped => WebBlockReason::ContainerStopped,
                        ActorBlockReason::Failed => WebBlockReason::ContainerFailed,
                        ActorBlockReason::Shutdown => WebBlockReason::Shutdown,
                    };

                    MemberState::Blocked { reason: web_reason }
                }
            };
        }
    }

    /// Update state cache based on a StateEvent
    async fn update_state(&self, event: &StateEvent) {
        Self::update_state_static(event, &self.state_cache).await;
    }

    /// Get the next state event (for web UI forwarding)
    pub async fn next_state_event(&mut self) -> Option<StateEvent> {
        self.state_rx.recv().await
    }

    /// Wait for all actors to complete (takes mut self!)
    pub async fn wait_for_completion(self) -> Result<()> {
        use futures::stream::{FuturesUnordered, StreamExt};

        // Extract fields from self
        let Self {
            tasks,
            mut state_rx,
            state_cache,
        } = self;

        // Track task count before moving into FuturesUnordered
        let task_count = tasks.len();
        tracing::debug!("wait_for_completion: starting with {} tasks", task_count);

        // Move tasks into FuturesUnordered for concurrent completion handling
        let mut task_futures = FuturesUnordered::from_iter(
            tasks
                .into_iter()
                .map(|(name, task)| async move { (name, task.await) }),
        );

        let mut errors = Vec::new();
        let mut remaining_tasks = task_count;
        tracing::debug!("wait_for_completion: remaining_tasks = {}", remaining_tasks);

        // Process state events and task completions concurrently
        loop {
            tokio::select! {
                // Process state events
                Some(event) = state_rx.recv() => {
                    tracing::debug!("State event: {:?}", event);
                    Self::update_state_static(&event, &state_cache).await;
                }

                // Process task completions
                Some((name, result)) = task_futures.next() => {
                    match result {
                        Ok(Ok(())) => {
                            tracing::debug!("Actor {} completed successfully", name);
                        }
                        Ok(Err(e)) => {
                            tracing::error!("Actor {} failed: {:#}", name, e);
                            errors.push(e);
                        }
                        Err(e) if e.is_cancelled() => {
                            tracing::debug!("Actor {} was cancelled", name);
                        }
                        Err(e) => {
                            tracing::error!("Actor {} panicked: {:#}", name, e);
                            errors.push(eyre::eyre!("Actor panicked: {}", e));
                        }
                    }

                    remaining_tasks -= 1;
                    if remaining_tasks == 0 {
                        break;
                    }
                }

                // All channels closed
                else => {
                    break;
                }
            }
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(eyre::eyre!(
                "Multiple actors failed: {}",
                errors
                    .iter()
                    .map(|e| e.to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            ))
        }
    }
}

/// Helper function to read last N lines from a file
fn read_last_n_lines(path: &std::path::Path, n: usize) -> Result<Vec<String>> {
    use std::io::{BufRead, BufReader};

    let file = std::fs::File::open(path)?;
    let reader = BufReader::new(file);
    let lines: Vec<String> = reader.lines().map_while(Result::ok).collect();

    let start = if lines.len() > n { lines.len() - n } else { 0 };

    Ok(lines[start..].to_vec())
}
