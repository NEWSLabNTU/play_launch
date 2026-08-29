//! Builder for collecting member definitions before spawning

use super::{
    CONTROL_CHANNEL_SIZE, MemberMetadata, STATE_EVENT_CHANNEL_SIZE, handle::MemberHandle,
    runner::MemberRunner,
};
use crate::member_actor::{
    member_id::{IdAllocator, MemberKind},
    model::{BlockReason, MemberState, MemberType},
};
use std::{collections::HashMap, path::PathBuf, sync::Arc};
use tokio::sync::{mpsc, watch};
use tracing::{debug, warn};

/// Definition of a regular node to be spawned
struct RegularNodeDefinition {
    name: String,
    context: crate::execution::context::NodeContext,
    config: crate::member_actor::model::ActorConfig,
    process_registry: Option<Arc<std::sync::Mutex<HashMap<u32, PathBuf>>>>,
    metadata: MemberMetadata,
}

/// Definition of a container to be spawned
struct ContainerDefinition {
    name: String,
    context: crate::execution::context::NodeContext,
    config: crate::member_actor::model::ActorConfig,
    process_registry: Option<Arc<std::sync::Mutex<HashMap<u32, PathBuf>>>>,
    metadata: MemberMetadata,
    /// Channel to send container state receiver back to caller
    state_tx: Option<
        tokio::sync::oneshot::Sender<watch::Receiver<crate::member_actor::model::ContainerState>>,
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
    /// Phase 38.9: resolved posix scheduling for this composable (None = no tier).
    sched: Option<crate::execution::sched_apply::AppliedTier>,
}

/// Does `member_name` name one of the members declared `on_exit=Shutdown()`?
///
/// Matches the name as-is, and again with a trailing `-N` counter removed. The dump
/// strips that counter from `exec_name` ("talker-1" -> "talker") to match the Rust
/// parser, while the model keeps the launch-level name, so an UNNAMED node reaches the
/// coordinator as "dash-1" and appears in the dump-derived set as "dash". A node with a
/// real ROS name (the case that matters — SSv2's `scenario_test_runner`) matches on the
/// first comparison; this is only for the unnamed ones.
fn declares_shutdown_on_exit(
    declared: &std::collections::HashSet<String>,
    member_name: &str,
) -> bool {
    if declared.contains(member_name) {
        return true;
    }
    match member_name.rsplit_once('-') {
        Some((stem, counter))
            if !counter.is_empty() && counter.bytes().all(|b| b.is_ascii_digit()) =>
        {
            declared.contains(stem)
        }
        _ => false,
    }
}

/// Builder for collecting member definitions before spawning
pub struct MemberCoordinatorBuilder {
    regular_nodes: Vec<RegularNodeDefinition>,
    containers: Vec<ContainerDefinition>,
    composable_nodes: Vec<ComposableNodeDefinition>,
    /// Nodes launched with `on_exit=Shutdown()`: when one of these exits, the whole
    /// launch goes down with it. See `MemberRunner`.
    shutdown_on_exit: std::collections::HashSet<String>,
    /// LoadNode-path timings applied to every container (phase-52.2)
    load_timings: crate::member_actor::container_actor::LoadTimings,
}

impl MemberCoordinatorBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            regular_nodes: Vec::new(),
            containers: Vec::new(),
            composable_nodes: Vec::new(),
            shutdown_on_exit: std::collections::HashSet::new(),
            load_timings: Default::default(),
        }
    }

    /// Declare the members that carry `on_exit=Shutdown()`.
    ///
    /// Taken from the dump rather than from each node's record because on the model
    /// path the record is rebuilt from a `NodeInstance`, which does not carry
    /// launch-level handlers. See `MemberRunner::honour_on_exit_shutdown`.
    pub fn set_shutdown_on_exit(&mut self, names: std::collections::HashSet<String>) {
        self.shutdown_on_exit.extend(names);
    }

    /// Set the LoadNode-path timings applied to every container
    /// (config `composable_node_loading` + CLI overrides; phase-52.2).
    pub fn set_load_timings(&mut self, timings: crate::member_actor::container_actor::LoadTimings) {
        self.load_timings = timings;
    }

    /// Add a regular node to be spawned later
    pub fn add_regular_node(
        &mut self,
        name: String,
        context: crate::execution::context::NodeContext,
        config: crate::member_actor::model::ActorConfig,
        process_registry: Option<Arc<std::sync::Mutex<HashMap<u32, PathBuf>>>>,
    ) {
        let metadata = MemberMetadata {
            id: String::new(), // assigned by the IdAllocator in spawn() (phase-50)
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
            node_name: context
                .record
                .name
                .clone()
                .or(context.record.exec_name.clone()),
            auto_load: None, // Not applicable for regular nodes
        };

        if context.record.on_exit_shutdown == Some(true) {
            self.shutdown_on_exit.insert(name.clone());
        }

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
        config: crate::member_actor::model::ActorConfig,
        process_registry: Option<Arc<std::sync::Mutex<HashMap<u32, PathBuf>>>>,
        use_component_events: bool,
    ) -> tokio::sync::oneshot::Receiver<watch::Receiver<crate::member_actor::model::ContainerState>>
    {
        let (state_tx, state_rx) = tokio::sync::oneshot::channel();

        let metadata = MemberMetadata {
            id: String::new(), // assigned by the IdAllocator in spawn() (phase-50)
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
        sched: Option<crate::execution::sched_apply::AppliedTier>,
    ) {
        let target_container_name = context.record.target_container_name.clone();

        let metadata = MemberMetadata {
            id: String::new(), // assigned by the IdAllocator in spawn() (phase-50)
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
            sched,
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
        // Phase-50: every registry keys on the canonical collision-proof id;
        // bare display names collide (issue 0001) and silently overwrote.
        let mut id_alloc = IdAllocator::new();

        // Display names collected by `set_shutdown_on_exit` / `add_regular_node` become
        // canonical member ids here, because that is what actors report in
        // `StateEvent::Exited` — matching on the display name silently never fired.
        let mut shutdown_on_exit_ids = std::collections::HashSet::new();

        for def in self.regular_nodes {
            let member_id = id_alloc.allocate(
                MemberKind::Node,
                def.metadata.namespace.as_deref(),
                &def.name,
            );
            if declares_shutdown_on_exit(&self.shutdown_on_exit, &def.name) {
                shutdown_on_exit_ids.insert(member_id.clone());
            }
            let (control_tx, control_rx) = mpsc::channel(CONTROL_CHANNEL_SIZE);

            let actor = crate::member_actor::regular_node_actor::RegularNodeActor::new(
                member_id.clone(),
                def.context,
                def.config,
                control_rx,
                state_tx.clone(),
                shutdown_rx.clone(),
                def.process_registry,
            );

            let task = tokio::spawn(async move {
                use crate::member_actor::actor_traits::MemberActor;
                actor.run().await
            });

            tasks.insert(member_id.clone(), task);
            control_channels.insert(member_id.clone(), control_tx);
            let mut metadata = def.metadata;
            metadata.id = member_id.clone();
            metadata_map.insert(member_id, metadata);
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

        for def in self.containers {
            // Phase-50: canonical id replaces the container-only `_N` dedup.
            let unique_member_name = id_alloc.allocate(
                MemberKind::Container,
                def.metadata.namespace.as_deref(),
                &def.name,
            );

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
                    use_component_events: def.use_component_events,
                    timings: self.load_timings,
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

            let mut metadata = def.metadata;
            metadata.id = unique_member_name.clone();

            container_full_names.insert(unique_member_name.clone(), container_name.clone());
            container_actors.insert(unique_member_name.clone(), actor);
            container_controls.insert(unique_member_name.clone(), control_tx);
            metadata_map.insert(unique_member_name, metadata);
        }

        // Phase 12: Add composable nodes as virtual members managed by containers
        // Declare virtual_member_routing HashMap here
        let mut virtual_member_routing = HashMap::new();

        for def in self.composable_nodes {
            // Phase-50: canonical id (collision-proof across namespaces,
            // containers, and kinds).
            let member_id = id_alloc.allocate(
                MemberKind::Composable,
                def.metadata.namespace.as_deref(),
                &def.name,
            );
            // Normalize target_container_name to ensure it starts with "/"
            // Some nodes have "pointcloud_container" while containers use "/pointcloud_container"
            let normalized_target = if def.target_container_name.starts_with('/') {
                def.target_container_name.clone()
            } else {
                format!("/{}", def.target_container_name)
            };

            // Find the container member name for this full node name.
            // First try exact match, then fall back to suffix matching for relative
            // targets that may omit namespace prefix (e.g. "planning_container" should
            // match "/planning/planning_container").
            let container_member_name = container_full_names
                .iter()
                .find(|(_, full_name)| *full_name == &normalized_target)
                .or_else(|| {
                    let suffix = format!("/{}", def.target_container_name.trim_start_matches('/'));
                    container_full_names
                        .iter()
                        .find(|(_, full_name)| full_name.ends_with(&suffix))
                })
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
                            sched: def.sched.clone(),
                        };

                    // Add composable node to container (keyed by canonical id;
                    // the actor's shared_state/StateEvent writes use this key)
                    container_actor.add_composable_node(member_id.clone(), node_metadata);

                    // Add metadata for this virtual member
                    tracing::debug!(
                        "Inserting composable node '{}' into metadata_map (member_type: {:?})",
                        def.name,
                        def.metadata.member_type
                    );
                    let mut metadata = def.metadata.clone();
                    metadata.id = member_id.clone();
                    metadata_map.insert(member_id.clone(), metadata);

                    // Populate virtual member routing
                    virtual_member_routing.insert(member_id.clone(), container_member_name.clone());

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
                // Phase-50 (issue 0001 cause 2): REGISTER the orphan visibly
                // instead of dropping it — it shows in the web UI as
                // Blocked{ContainerNotFound} with its metadata intact.
                warn!(
                    "Container '{}' not found for composable node '{}' — registering as \
                     Blocked(ContainerNotFound) (normalized: '{}')",
                    def.target_container_name, def.name, normalized_target
                );
                debug!(
                    "Available containers ({}): {:?}",
                    container_full_names.len(),
                    container_full_names.values().collect::<Vec<_>>()
                );
                let mut metadata = def.metadata.clone();
                metadata.id = member_id.clone();
                metadata_map.insert(member_id.clone(), metadata);
                super::state_reducer::set(
                    &shared_state,
                    &member_id,
                    MemberState::Blocked {
                        reason: BlockReason::ContainerNotFound,
                    },
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
            // Never overwrites a state the reducer already derived
            super::state_reducer::init(&shared_state, name, initial_state);
        }

        // Phase-50: registered == spawned is the invariant that used to break
        // silently via bare-name key collisions (issue 0001). Count and check.
        // Orphaned composables (Blocked{ContainerNotFound}) are registered but
        // unrouted, so they're excluded from the routing side of the check.
        let mut n_nodes = 0usize;
        let mut n_containers = 0usize;
        let mut n_composables = 0usize;
        for meta in metadata_map.values() {
            match meta.member_type {
                MemberType::Node => n_nodes += 1,
                MemberType::Container => n_containers += 1,
                MemberType::ComposableNode => n_composables += 1,
            }
        }
        let orphaned = metadata_map
            .iter()
            .filter(|(id, meta)| {
                meta.member_type == MemberType::ComposableNode
                    && !virtual_member_routing.contains_key(*id)
            })
            .count();
        tracing::info!(
            "Registered {} members ({} nodes, {} containers, {} composables{})",
            metadata_map.len(),
            n_nodes,
            n_containers,
            n_composables,
            if orphaned > 0 {
                format!(", {} orphaned", orphaned)
            } else {
                String::new()
            }
        );
        let routed = tasks.len() + virtual_member_routing.len() + orphaned;
        if metadata_map.len() != routed {
            tracing::warn!(
                "MEMBER REGISTRY MISMATCH: {} registered vs {} spawned/routed — \
                 some members are invisible or unroutable (issue 0001 class)",
                metadata_map.len(),
                routed
            );
        }

        // Phase 12: Build virtual member routing map for composable nodes
        // Maps composable node name -> parent container name
        // Populated during composable node registration above
        let virtual_member_routing = virtual_member_routing;

        // Phase 24: Build node FQN map for parameter service calls
        // Regular nodes and containers: namespace + node_name
        // Composable nodes: populated later when LoadSucceeded arrives
        let mut node_fqn_map = HashMap::new();
        for (member_name, meta) in metadata_map.iter() {
            if (meta.member_type == MemberType::Node || meta.member_type == MemberType::Container)
                && let Some(node_name) = &meta.node_name
            {
                let namespace = meta.namespace.as_deref().unwrap_or("/");
                let fqn = if namespace == "/" {
                    format!("/{}", node_name)
                } else if namespace.ends_with('/') {
                    format!("{}{}", namespace, node_name)
                } else {
                    format!("{}/{}", namespace, node_name)
                };
                node_fqn_map.insert(member_name.clone(), fqn);
            }
        }
        let node_fqn_map = Arc::new(tokio::sync::RwLock::new(node_fqn_map));

        // Shared, because the runner needs to be able to pull the same lever the
        // handle does: a node with `on_exit=Shutdown()` exiting must stop every actor.
        let shutdown_tx = Arc::new(shutdown_tx);

        let handle = MemberHandle::new(
            control_channels,
            Arc::new(tokio::sync::RwLock::new(metadata_map)),
            shared_state.clone(),
            shutdown_tx.clone(),
            virtual_member_routing,
            shared_ros_node,
            node_fqn_map,
        );

        let runner = MemberRunner::new(
            tasks,
            state_rx,
            shared_state,
            shutdown_on_exit_ids,
            shutdown_tx,
        );

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

#[cfg(test)]
mod tests {
    use super::declares_shutdown_on_exit;
    use std::collections::HashSet;

    #[test]
    fn a_named_node_matches_directly() {
        let declared: HashSet<String> = ["scenario_test_runner".to_string()].into();
        assert!(declares_shutdown_on_exit(&declared, "scenario_test_runner"));
    }

    /// The dump strips the launch counter from `exec_name`, the model keeps it.
    #[test]
    fn an_unnamed_node_matches_with_its_counter_stripped() {
        let declared: HashSet<String> = ["dash".to_string()].into();
        assert!(declares_shutdown_on_exit(&declared, "dash-1"));
        assert!(declares_shutdown_on_exit(&declared, "dash-12"));
    }

    #[test]
    fn an_unrelated_member_does_not_match() {
        let declared: HashSet<String> = ["dash".to_string()].into();
        assert!(!declares_shutdown_on_exit(&declared, "sh-1"));
        assert!(!declares_shutdown_on_exit(&declared, "dashboard"));
    }

    /// A hyphen that is not a counter must not be stripped, or `foo-bar` would match a
    /// declaration of `foo`.
    #[test]
    fn a_hyphen_that_is_not_a_counter_is_left_alone() {
        let declared: HashSet<String> = ["foo".to_string()].into();
        assert!(!declares_shutdown_on_exit(&declared, "foo-bar"));
        assert!(!declares_shutdown_on_exit(&declared, "foo-"));
    }
}
