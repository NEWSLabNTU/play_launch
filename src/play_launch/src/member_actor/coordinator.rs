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
use tracing::{debug, warn};

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
    pub auto_load: Option<bool>, // For composable nodes: auto-load when container starts
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

/// Definition of a composable node to be spawned (Phase 12: managed by containers)
struct ComposableNodeDefinition {
    name: String,
    context: crate::execution::context::ComposableNodeContext,
    auto_load: bool,
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
            auto_load: None, // Not applicable for regular nodes
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
            auto_load: None, // Not applicable for containers
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
        auto_load: bool,
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
            auto_load: Some(auto_load),
        };

        self.composable_nodes.push(ComposableNodeDefinition {
            name,
            context,
            auto_load,
            target_container_name,
            metadata,
        });
    }

    /// Spawn all members and return handle + runner
    pub async fn spawn(
        self,
        shared_ros_node: Option<Arc<rclrs::Node>>,
    ) -> (MemberHandle, MemberRunner) {
        let (state_tx, state_rx) = mpsc::channel(100);
        let (shutdown_tx, shutdown_rx) = watch::channel(false);

        let mut tasks = HashMap::new();
        let mut control_channels = HashMap::new();
        let mut metadata_map = HashMap::new();

        // Initialize shared state map (will be populated before spawning actors)
        let shared_state = Arc::new(dashmap::DashMap::new());

        // Collect all metadata first before spawning actors
        // (We'll populate shared_state before spawning to avoid race conditions)

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
                shared_state.clone(),
            );

            let task = tokio::spawn(async move {
                use super::actor_traits::MemberActor;
                actor.run().await
            });

            tasks.insert(def.name.clone(), task);
            control_channels.insert(def.name.clone(), control_tx);
            metadata_map.insert(def.name.clone(), def.metadata);
        }

        // Spawn containers and collect their state receivers and load control channels
        // Use the shared ROS node passed from play() function
        // Phase 12: Keep containers in a HashMap so we can add composable nodes before spawning
        let mut container_actors: HashMap<String, super::container_actor::ContainerActor> =
            HashMap::new();
        let mut container_full_names = HashMap::new(); // member_name -> full_node_name
        let mut container_state_map = HashMap::new();
        let mut container_load_control_map = HashMap::new();
        let mut container_controls = HashMap::new(); // member_name -> control_tx

        // Track base names for deduplication (e.g., "container" -> 1, 2, 3...)
        let mut name_counts = HashMap::<String, usize>::new();

        for def in self.containers {
            // Generate unique member name FIRST (before creating actor)
            let base_name = def.name.clone();
            let count = name_counts.entry(base_name.clone()).or_insert(0);
            *count += 1;
            let unique_member_name = if *count == 1 {
                base_name
            } else {
                format!("{}_{}", base_name, count)
            };

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
            // IMPORTANT: Pass unique_member_name so actor reports state with it
            let actor = super::container_actor::ContainerActor::new(
                unique_member_name.clone(),
                def.context,
                def.config,
                control_rx,
                state_tx.clone(),
                shutdown_rx.clone(),
                def.process_registry,
                load_control_rx,
                shared_ros_node.clone(),
                shared_state.clone(),
            );

            // Get container state receiver before spawning
            let container_state_rx = actor.container_state_rx();

            // Send container state receiver back to caller if requested
            if let Some(tx) = def.state_tx {
                let _ = tx.send(container_state_rx.clone());
            }

            // Store for composable nodes to use
            container_state_map.insert(container_name.clone(), container_state_rx);
            container_load_control_map.insert(container_name.clone(), load_control_tx);

            // Phase 12: Store actor for adding composable nodes
            debug!(
                "Registering container: member_name='{}', full_name='{}'",
                unique_member_name, container_name
            );

            // Update metadata name to match unique member name (for Web UI)
            let mut metadata = def.metadata;
            metadata.name = unique_member_name.clone();

            container_full_names.insert(unique_member_name.clone(), container_name.clone());
            container_actors.insert(unique_member_name.clone(), actor);
            container_controls.insert(unique_member_name.clone(), control_tx);
            metadata_map.insert(unique_member_name, metadata);
        }

        // Phase 12: Add composable nodes as virtual members managed by containers
        // Declare virtual_member_routing HashMap here
        let mut virtual_member_routing = HashMap::new();

        for def in self.composable_nodes {
            // Normalize target_container_name to ensure it starts with "/"
            // Some nodes have "pointcloud_container" while containers use "/pointcloud_container"
            let normalized_target = if def.target_container_name.starts_with('/') {
                def.target_container_name.clone()
            } else {
                format!("/{}", def.target_container_name)
            };

            // Find the container member name for this full node name
            let container_member_name = container_full_names
                .iter()
                .find(|(_, full_name)| *full_name == &normalized_target)
                .map(|(member_name, _)| member_name.clone());

            if let Some(container_member_name) = container_member_name {
                // Get mutable reference to container actor
                if let Some(container_actor) = container_actors.get_mut(&container_member_name) {
                    // Convert parameters and extra_args
                    let parameters = match crate::ros::component_loader::convert_parameters_to_ros(
                        &def.context.record.params,
                    ) {
                        Ok(params) => params,
                        Err(e) => {
                            warn!(
                                "Failed to convert parameters for composable node '{}': {:#}",
                                def.name, e
                            );
                            Vec::new()
                        }
                    };

                    let extra_args = match crate::ros::component_loader::convert_parameters_to_ros(
                        &def.context
                            .record
                            .extra_args
                            .iter()
                            .map(|(k, v)| (k.clone(), v.clone()))
                            .collect::<Vec<_>>(),
                    ) {
                        Ok(args) => args,
                        Err(e) => {
                            warn!(
                                "Failed to convert extra_args for composable node '{}': {:#}",
                                def.name, e
                            );
                            Vec::new()
                        }
                    };

                    // Convert ComposableNodeContext to ComposableNodeMetadata
                    let metadata = super::container_actor::ComposableNodeMetadata {
                        package: def.context.record.package.clone(),
                        plugin: def.context.record.plugin.clone(),
                        node_name: def.context.record.node_name.clone(),
                        namespace: def.context.record.namespace.clone(),
                        remap_rules: def
                            .context
                            .record
                            .remaps
                            .iter()
                            .map(|(src, tgt)| format!("{}:={}", src, tgt))
                            .collect(),
                        parameters,
                        extra_args,
                        auto_load: def.auto_load,
                    };

                    // Add composable node to container
                    container_actor.add_composable_node(def.name.clone(), metadata);

                    // Add metadata for this virtual member
                    tracing::debug!(
                        "Inserting composable node '{}' into metadata_map (member_type: {:?})",
                        def.name,
                        def.metadata.member_type
                    );
                    metadata_map.insert(def.name.clone(), def.metadata.clone());

                    // Populate virtual member routing
                    virtual_member_routing.insert(def.name.clone(), container_member_name.clone());

                    debug!(
                        "Added composable node '{}' as virtual member of container '{}'",
                        def.name, container_member_name
                    );
                } else {
                    warn!(
                        "Container actor '{}' not found for composable node '{}', skipping",
                        container_member_name, def.name
                    );
                }
            } else {
                warn!(
                    "Container '{}' not found for composable node '{}', skipping (normalized: '{}')",
                    def.target_container_name, def.name, normalized_target
                );
                debug!(
                    "Available containers ({}): {:?}",
                    container_full_names.len(),
                    container_full_names.values().collect::<Vec<_>>()
                );
            }
        }

        // Phase 12: Now spawn all container actors
        for (member_name, actor) in container_actors {
            let control_tx = container_controls.remove(&member_name).unwrap();

            // Update metadata.json with composable node information
            if let Err(e) = actor.update_metadata_with_composables() {
                tracing::warn!(
                    "Failed to update metadata for container '{}': {:#}",
                    member_name,
                    e
                );
            }

            let task = tokio::spawn(async move {
                use super::actor_traits::MemberActor;
                actor.run().await
            });

            tasks.insert(member_name.clone(), task);
            control_channels.insert(member_name, control_tx);
        }

        // state_tx and shutdown_rx are dropped here (actors have their clones)

        // Populate shared state map with initial states for all members
        // Only insert if not already present (actors may have already written their states)
        for (name, meta) in metadata_map.iter() {
            // Composable nodes start in Unloaded state, others start in Pending
            let initial_state = if meta.member_type == MemberType::ComposableNode {
                MemberState::Unloaded
            } else {
                MemberState::Pending
            };
            // Use entry API to avoid overwriting actor state updates
            shared_state.entry(name.clone()).or_insert(initial_state);
        }

        // Phase 12: Build virtual member routing map for composable nodes
        // Maps composable node name -> parent container name
        // Populated during composable node registration above
        let virtual_member_routing = virtual_member_routing;

        let handle = MemberHandle {
            control_channels,
            metadata: Arc::new(tokio::sync::RwLock::new(metadata_map)),
            shared_state: shared_state.clone(),
            shutdown_tx,
            virtual_member_routing,
        };

        let runner = MemberRunner {
            tasks,
            state_rx,
            shared_state,
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
    /// Member metadata (mutable for dynamic config updates like respawn_enabled)
    metadata: Arc<tokio::sync::RwLock<HashMap<String, MemberMetadata>>>,
    /// Shared state map (actors write directly, Web UI reads)
    shared_state: Arc<dashmap::DashMap<String, MemberState>>,
    /// Shutdown signal broadcaster
    shutdown_tx: watch::Sender<bool>,
    /// Virtual member routing: maps composable node names to parent container names (Phase 12)
    virtual_member_routing: HashMap<String, String>,
}

impl MemberHandle {
    /// Get list of all members with current state
    pub async fn list_members(&self) -> Vec<MemberSummary> {
        let mut summaries = Vec::new();

        let metadata_guard = self.metadata.read().await;
        tracing::debug!(
            "list_members: metadata_guard has {} entries",
            metadata_guard.len()
        );
        for (name, meta) in metadata_guard.iter() {
            // Read state from DashMap
            let state = self
                .shared_state
                .get(name)
                .map(|entry| entry.value().clone())
                .unwrap_or(MemberState::Pending);

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
                auto_load: meta.auto_load,
                output_dir: meta.output_dir.clone(),
            });
        }

        summaries
    }

    /// Get detailed state for a specific member
    pub async fn get_member_state(&self, name: &str) -> Option<MemberSummary> {
        let metadata_guard = self.metadata.read().await;
        let meta = metadata_guard.get(name)?;

        // Read state from DashMap
        let state = self
            .shared_state
            .get(name)
            .map(|entry| entry.value().clone())
            .unwrap_or(MemberState::Pending);

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
            auto_load: meta.auto_load,
            output_dir: meta.output_dir.clone(),
        })
    }

    /// Get health summary statistics
    pub async fn get_health_summary(&self) -> HealthSummary {
        let mut summary = HealthSummary::default();

        let metadata_guard = self.metadata.read().await;
        for (name, meta) in metadata_guard.iter() {
            // Read state from DashMap
            let state = self
                .shared_state
                .get(name)
                .map(|entry| entry.value().clone())
                .unwrap_or(MemberState::Pending);

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
                            // Pending means not started yet, don't count as stopped
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
                            // Pending means not started yet, don't count as stopped
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
    ///
    /// Phase 12: For virtual members (composable nodes managed by containers),
    /// this translates and routes control events to the parent container.
    pub async fn send_control(&self, name: &str, event: ControlEvent) -> Result<()> {
        // Check if this is a virtual member (composable node)
        if let Some(parent_container) = self.virtual_member_routing.get(name) {
            // Translate control event for virtual member
            let translated_event = match event {
                ControlEvent::Start => ControlEvent::LoadComposable {
                    name: name.to_string(),
                },
                ControlEvent::Stop => ControlEvent::UnloadComposable {
                    name: name.to_string(),
                },
                ControlEvent::Restart => {
                    // For restart, send Unload followed by Load
                    // First unload
                    let control_tx =
                        self.control_channels.get(parent_container).ok_or_else(|| {
                            eyre::eyre!("Parent container not found: {}", parent_container)
                        })?;

                    control_tx
                        .send(ControlEvent::UnloadComposable {
                            name: name.to_string(),
                        })
                        .await
                        .context("Failed to send Unload to parent container")?;

                    // Then load
                    ControlEvent::LoadComposable {
                        name: name.to_string(),
                    }
                }
                ControlEvent::ToggleRespawn(_) => {
                    // Not supported for virtual members
                    warn!("ToggleRespawn not supported for virtual member: {}", name);
                    return Ok(());
                }
                ControlEvent::Load => ControlEvent::LoadComposable {
                    name: name.to_string(),
                },
                ControlEvent::Unload => ControlEvent::UnloadComposable {
                    name: name.to_string(),
                },
                ControlEvent::ToggleAutoLoad(enabled) => {
                    // Update metadata and forward to container so it can update internal state
                    let mut metadata_guard = self.metadata.write().await;
                    if let Some(meta) = metadata_guard.get_mut(name) {
                        meta.auto_load = Some(enabled);
                    }
                    drop(metadata_guard);

                    // Forward to container with composable node name included
                    ControlEvent::ToggleComposableAutoLoad {
                        name: name.to_string(),
                        enabled,
                    }
                }
                other => {
                    // Forward other events as-is
                    other
                }
            };

            // Send to parent container
            let control_tx = self
                .control_channels
                .get(parent_container)
                .ok_or_else(|| eyre::eyre!("Parent container not found: {}", parent_container))?;

            control_tx
                .send(translated_event)
                .await
                .context("Failed to send translated control event to parent container")
        } else {
            // Regular member - send directly
            let control_tx = self
                .control_channels
                .get(name)
                .ok_or_else(|| eyre::eyre!("Actor not found: {}", name))?;

            control_tx
                .send(event)
                .await
                .context("Failed to send control event to actor")
        }
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
        // Send control event to actor
        self.send_control(name, ControlEvent::ToggleRespawn(enabled))
            .await?;

        // Update metadata so web UI reflects the new state
        let mut metadata_guard = self.metadata.write().await;
        if let Some(meta) = metadata_guard.get_mut(name) {
            meta.respawn_enabled = Some(enabled);
        }

        Ok(())
    }

    /// Toggle auto-load for a composable node
    pub async fn toggle_auto_load(&self, name: &str, enabled: bool) -> Result<()> {
        // Send control event to actor
        self.send_control(name, ControlEvent::ToggleAutoLoad(enabled))
            .await?;

        // Update metadata so web UI reflects the new state
        let mut metadata_guard = self.metadata.write().await;
        if let Some(meta) = metadata_guard.get_mut(name) {
            if meta.member_type != MemberType::ComposableNode {
                return Err(eyre::eyre!("Auto-load only applies to composable nodes"));
            }
            meta.auto_load = Some(enabled);
        } else {
            return Err(eyre::eyre!("Member '{}' not found", name));
        }

        Ok(())
    }

    /// Request a composable node to load (retry loading)
    pub async fn load_member(&self, name: &str) -> Result<()> {
        // Immediately update shared_state to Loading for instant Web UI feedback
        self.shared_state
            .insert(name.to_string(), super::web_query::MemberState::Loading);

        self.send_control(name, ControlEvent::Load).await
    }

    /// Request a composable node to unload
    pub async fn unload_member(&self, name: &str) -> Result<()> {
        // Immediately update shared_state to Unloading for instant Web UI feedback
        self.shared_state
            .insert(name.to_string(), super::web_query::MemberState::Unloading);

        self.send_control(name, ControlEvent::Unload).await
    }

    /// Load all unloaded or failed composable nodes in a container
    pub async fn load_all_container_children(&self, container_name: &str) -> Result<usize> {
        // First verify the container exists and is a container
        let metadata_guard = self.metadata.read().await;
        if let Some(container_meta) = metadata_guard.get(container_name) {
            if container_meta.member_type != MemberType::Container {
                return Err(eyre::eyre!("'{}' is not a container", container_name));
            }
        } else {
            return Err(eyre::eyre!("Container '{}' not found", container_name));
        }

        // Use virtual_member_routing to find composable nodes that belong to this container
        // Count composable nodes that need loading (for return value)
        let mut nodes_to_load_count = 0;
        for (composable_name, parent_container) in self.virtual_member_routing.iter() {
            if parent_container == container_name {
                // Check if this node is in Unloaded or Failed state
                if let Some(entry) = self.shared_state.get(composable_name.as_str()) {
                    match entry.value() {
                        MemberState::Unloaded | MemberState::Failed { .. } => {
                            nodes_to_load_count += 1;
                        }
                        _ => {} // Skip nodes in other states
                    }
                }
            }
        }
        drop(metadata_guard); // Release lock before sending control

        // Send LoadAllComposables to container (Phase 12)
        self.send_control(container_name, ControlEvent::LoadAllComposables)
            .await?;

        Ok(nodes_to_load_count)
    }

    /// Unload all loaded composable nodes in a container
    pub async fn unload_all_container_children(&self, container_name: &str) -> Result<usize> {
        // First verify the container exists and is a container
        let metadata_guard = self.metadata.read().await;
        if let Some(container_meta) = metadata_guard.get(container_name) {
            if container_meta.member_type != MemberType::Container {
                return Err(eyre::eyre!("'{}' is not a container", container_name));
            }
        } else {
            return Err(eyre::eyre!("Container '{}' not found", container_name));
        }

        // Use virtual_member_routing to find composable nodes that belong to this container
        // Count composable nodes that are loaded (for return value)
        let mut nodes_to_unload_count = 0;
        for (composable_name, parent_container) in self.virtual_member_routing.iter() {
            if parent_container == container_name {
                // Check if this node is in Loaded state
                if let Some(entry) = self.shared_state.get(composable_name.as_str()) {
                    if let MemberState::Loaded { .. } = entry.value() {
                        nodes_to_unload_count += 1;
                    }
                }
            }
        }
        drop(metadata_guard); // Release lock before sending control

        // Send UnloadAllComposables to container (Phase 12)
        self.send_control(container_name, ControlEvent::UnloadAllComposables)
            .await?;

        Ok(nodes_to_unload_count)
    }

    /// Start all nodes and containers (not composable nodes - they're managed by containers)
    pub async fn start_all(&self) -> Result<usize> {
        let metadata_guard = self.metadata.read().await;
        let mut count = 0;

        for (name, meta) in metadata_guard.iter() {
            // Only send to regular nodes and containers
            if meta.member_type == MemberType::Node || meta.member_type == MemberType::Container {
                if let Some(tx) = self.control_channels.get(name) {
                    let _ = tx.send(ControlEvent::Start).await;
                    count += 1;
                }
            }
        }

        Ok(count)
    }

    /// Stop all nodes and containers (not composable nodes - they're managed by containers)
    pub async fn stop_all(&self) -> Result<usize> {
        let metadata_guard = self.metadata.read().await;
        let mut count = 0;

        for (name, meta) in metadata_guard.iter() {
            // Only send to regular nodes and containers
            if meta.member_type == MemberType::Node || meta.member_type == MemberType::Container {
                if let Some(tx) = self.control_channels.get(name) {
                    let _ = tx.send(ControlEvent::Stop).await;
                    count += 1;
                }
            }
        }

        Ok(count)
    }

    /// Restart all nodes and containers (not composable nodes - they're managed by containers)
    pub async fn restart_all(&self) -> Result<usize> {
        let metadata_guard = self.metadata.read().await;
        let mut count = 0;

        for (name, meta) in metadata_guard.iter() {
            // Only send to regular nodes and containers
            if meta.member_type == MemberType::Node || meta.member_type == MemberType::Container {
                if let Some(tx) = self.control_channels.get(name) {
                    let _ = tx.send(ControlEvent::Restart).await;
                    count += 1;
                }
            }
        }

        Ok(count)
    }

    /// Broadcast shutdown signal to all actors
    pub fn shutdown(&self) -> Result<()> {
        self.shutdown_tx
            .send(true)
            .context("Failed to send shutdown signal")
    }

    /// Get the number of registered members
    pub async fn member_count(&self) -> usize {
        self.metadata.read().await.len()
    }

    /// Check if a specific member is registered
    pub async fn has_member(&self, name: &str) -> bool {
        self.metadata.read().await.contains_key(name)
    }

    /// Get reference to the shared state map
    pub fn shared_state(&self) -> &Arc<dashmap::DashMap<String, MemberState>> {
        &self.shared_state
    }
}

/// Runner that waits for all actors to complete
/// Takes mut self - no Arc<Mutex> needed!
pub struct MemberRunner {
    /// Task handles for all actors
    tasks: HashMap<String, JoinHandle<Result<()>>>,
    /// Receiver for state events from actors
    state_rx: mpsc::Receiver<StateEvent>,
    /// Shared state map (actors write directly, runner only reads for logging)
    shared_state: Arc<dashmap::DashMap<String, MemberState>>,
}

impl MemberRunner {
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
            shared_state: _shared_state,
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
                // Drain state events (actors write directly to shared_state)
                Some(event) = state_rx.recv() => {
                    tracing::debug!("State event: {:?}", event);
                    // Actors update shared_state directly, no need to update from events
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
