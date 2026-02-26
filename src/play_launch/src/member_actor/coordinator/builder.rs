//! Builder for collecting member definitions before spawning

use super::{
    handle::MemberHandle, runner::MemberRunner, MemberMetadata, CONTROL_CHANNEL_SIZE,
    STATE_EVENT_CHANNEL_SIZE,
};
use crate::member_actor::web_query::{MemberState, MemberType};
use std::{collections::HashMap, path::PathBuf, sync::Arc};
use tokio::sync::{mpsc, watch};
use tracing::{debug, warn};

/// Definition of a regular node to be spawned
struct RegularNodeDefinition {
    name: String,
    context: crate::execution::context::NodeContext,
    config: crate::member_actor::state::ActorConfig,
    process_registry: Option<Arc<std::sync::Mutex<HashMap<u32, PathBuf>>>>,
    metadata: MemberMetadata,
}

/// Definition of a container to be spawned
struct ContainerDefinition {
    name: String,
    context: crate::execution::context::NodeContext,
    config: crate::member_actor::state::ActorConfig,
    process_registry: Option<Arc<std::sync::Mutex<HashMap<u32, PathBuf>>>>,
    metadata: MemberMetadata,
    /// Channel to send container state receiver back to caller
    state_tx: Option<
        tokio::sync::oneshot::Sender<watch::Receiver<crate::member_actor::state::ContainerState>>,
    >,
    /// Whether to subscribe to ComponentEvent topic (only for play_launch_container)
    use_component_events: bool,
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
        config: crate::member_actor::state::ActorConfig,
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
        config: crate::member_actor::state::ActorConfig,
        process_registry: Option<Arc<std::sync::Mutex<HashMap<u32, PathBuf>>>>,
        use_component_events: bool,
    ) -> tokio::sync::oneshot::Receiver<watch::Receiver<crate::member_actor::state::ContainerState>>
    {
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
            use_component_events,
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
        let (state_tx, state_rx) = mpsc::channel(STATE_EVENT_CHANNEL_SIZE);
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
            let (control_tx, control_rx) = mpsc::channel(CONTROL_CHANNEL_SIZE);

            let actor = crate::member_actor::regular_node_actor::RegularNodeActor::new(
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
                use crate::member_actor::actor_traits::MemberActor;
                actor.run().await
            });

            tasks.insert(def.name.clone(), task);
            control_channels.insert(def.name.clone(), control_tx);
            metadata_map.insert(def.name.clone(), def.metadata);
        }

        // Spawn containers and collect their state receivers and load control channels
        // Use the shared ROS node passed from play() function
        // Phase 12: Keep containers in a HashMap so we can add composable nodes before spawning
        let mut container_actors: HashMap<
            String,
            crate::member_actor::container_actor::ContainerActor,
        > = HashMap::new();
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

            let (control_tx, control_rx) = mpsc::channel(CONTROL_CHANNEL_SIZE);

            // Phase 2: Create load control channel for composable nodes
            let (load_control_tx, load_control_rx) =
                mpsc::channel::<crate::member_actor::container_control::ContainerControlEvent>(10);

            // Phase 2: Use the shared ROS node for this container
            // IMPORTANT: Pass unique_member_name so actor reports state with it
            let actor = crate::member_actor::container_actor::ContainerActor::new(
                unique_member_name.clone(),
                crate::member_actor::container_actor::ContainerActorParams {
                    context: def.context,
                    config: def.config,
                    process_registry: def.process_registry,
                    ros_node: shared_ros_node.clone(),
                    shared_state: shared_state.clone(),
                    use_component_events: def.use_component_events,
                },
                control_rx,
                state_tx.clone(),
                shutdown_rx.clone(),
                load_control_rx,
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
                    let parameters =
                        match crate::ros::parameter_conversion::convert_parameters_to_ros(
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

                    let extra_args =
                        match crate::ros::parameter_conversion::convert_parameters_to_ros(
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
                    let node_metadata =
                        crate::member_actor::container_actor::ComposableNodeMetadata {
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
                            output_dir: def.context.output_dir.clone(),
                        };

                    // Add composable node to container
                    container_actor.add_composable_node(def.name.clone(), node_metadata);

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
                use crate::member_actor::actor_traits::MemberActor;
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

        let handle = MemberHandle::new(
            control_channels,
            Arc::new(tokio::sync::RwLock::new(metadata_map)),
            shared_state.clone(),
            shutdown_tx,
            virtual_member_routing,
        );

        let runner = MemberRunner::new(tasks, state_rx, shared_state);

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
