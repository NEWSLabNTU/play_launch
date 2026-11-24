//! Members added after the start (phase 106: a Reload that finds a node the
//! launch files did not have when the run began).
//!
//! The builder spawns every actor at once and then lets go of what it used to
//! build them. [`LateSpawner`] keeps the part a later actor needs: the id
//! allocator (so a new id never collides with one already handed out), the
//! state and shutdown channels, the LoadNode timings, and which container
//! answers to which full name. A new actor's task goes to the runner, which
//! waits for it like any other.
//!
//! A member added this way starts Excluded and stopped: Reload changes nothing
//! that runs, and the next Apply starts what the selection covers.

use super::{
    CONTROL_CHANNEL_SIZE, MemberHandle,
    builder::{
        composable_member_metadata, container_full_name, container_metadata, find_container,
        node_metadata,
    },
};
use crate::member_actor::{
    events::{ControlEvent, StateEvent},
    exit_policy::ExitPolicy,
    member_id::{IdAllocator, MemberKind},
    model::{ActorConfig, MemberState},
};
use eyre::{Result, bail};
use std::{collections::HashMap, path::PathBuf, sync::Arc};
use tokio::{
    sync::{mpsc, watch},
    task::JoinHandle,
};

type ProcessRegistry = Option<Arc<std::sync::Mutex<HashMap<u32, PathBuf>>>>;

/// An actor spawned after the start, for the runner to wait on.
pub(crate) struct LateTask {
    pub id: String,
    pub task: JoinHandle<Result<()>>,
    pub exit_policy: Option<ExitPolicy>,
}

/// What building an actor after the start needs.
pub(super) struct LateSpawner {
    pub id_alloc: IdAllocator,
    /// Weak, so the state stream still ends when every actor has.
    pub state_tx: mpsc::WeakSender<StateEvent>,
    pub shutdown_rx: watch::Receiver<bool>,
    pub load_timings: crate::member_actor::container_actor::LoadTimings,
    /// Container member id -> its full node name.
    pub container_full_names: HashMap<String, String>,
    pub tasks_tx: mpsc::UnboundedSender<LateTask>,
}

/// A member to add.
pub enum LateMember {
    Node {
        name: String,
        context: crate::execution::context::NodeContext,
        config: ActorConfig,
        process_registry: ProcessRegistry,
    },
    Container {
        name: String,
        context: crate::execution::context::NodeContext,
        config: ActorConfig,
        process_registry: ProcessRegistry,
        use_component_events: bool,
    },
    Composable {
        name: String,
        context: crate::execution::context::ComposableNodeContext,
        sched: Option<crate::execution::sched_apply::AppliedTask>,
    },
}

impl MemberHandle {
    /// Add a member to the running launch, Excluded and stopped, and return
    /// its id. A composable's container must be a member already (add
    /// containers first).
    pub async fn add_member(&self, member: LateMember) -> Result<String> {
        let mut guard = self.late.lock().await;
        let Some(late) = guard.as_mut() else {
            bail!("this run cannot add members");
        };
        let Some(state_tx) = late.state_tx.upgrade() else {
            bail!("the launch is ending");
        };
        match member {
            LateMember::Node {
                name,
                context,
                mut config,
                process_registry,
            } => {
                let mut metadata = node_metadata(&name, &context);
                let id =
                    late.id_alloc
                        .allocate(MemberKind::Node, metadata.namespace.as_deref(), &name);
                let exit_policy = ExitPolicy::of(&context.record);
                config.start_stopped = true;
                self.excluded.insert(id.clone());
                super::state_reducer::set(&self.shared_state, &id, MemberState::Stopped);
                let (control_tx, control_rx) = mpsc::channel(CONTROL_CHANNEL_SIZE);
                let actor = crate::member_actor::regular_node_actor::RegularNodeActor::new(
                    id.clone(),
                    context,
                    config,
                    control_rx,
                    state_tx,
                    late.shutdown_rx.clone(),
                    process_registry,
                );
                let task = tokio::spawn(async move {
                    use crate::member_actor::actor_traits::MemberActor;
                    actor.run().await
                });
                metadata.id = id.clone();
                self.register(&id, metadata, control_tx).await;
                let _ = late.tasks_tx.send(LateTask {
                    id: id.clone(),
                    task,
                    exit_policy,
                });
                Ok(id)
            }
            LateMember::Container {
                name,
                context,
                mut config,
                process_registry,
                use_component_events,
            } => {
                let mut metadata = container_metadata(&name, &context);
                let id = late.id_alloc.allocate(
                    MemberKind::Container,
                    metadata.namespace.as_deref(),
                    &name,
                );
                let exit_policy = ExitPolicy::of(&context.record);
                let full_name = container_full_name(&context);
                config.start_stopped = true;
                self.excluded.insert(id.clone());
                super::state_reducer::set(&self.shared_state, &id, MemberState::Stopped);
                let (control_tx, control_rx) = mpsc::channel(CONTROL_CHANNEL_SIZE);
                // The load channel is how a launch's own composables reach the
                // container at the start; a late one gets its composables
                // through control events, so its sender is not kept.
                let (_load_tx, load_rx) = mpsc::channel(10);
                let ros_node = self.ros_node();
                let actor = crate::member_actor::container_actor::ContainerActor::new(
                    id.clone(),
                    crate::member_actor::container_actor::ContainerActorParams {
                        context,
                        config,
                        process_registry,
                        ros_node,
                        use_component_events,
                        timings: late.load_timings,
                    },
                    control_rx,
                    state_tx,
                    late.shutdown_rx.clone(),
                    load_rx,
                );
                if let Err(e) = actor.update_metadata_with_composables() {
                    tracing::warn!("Failed to update metadata for container '{id}': {e:#}");
                }
                let task = tokio::spawn(async move {
                    use crate::member_actor::actor_traits::MemberActor;
                    actor.run().await
                });
                metadata.id = id.clone();
                late.container_full_names.insert(id.clone(), full_name);
                self.register(&id, metadata, control_tx).await;
                let _ = late.tasks_tx.send(LateTask {
                    id: id.clone(),
                    task,
                    exit_policy,
                });
                Ok(id)
            }
            LateMember::Composable {
                name,
                context,
                sched,
            } => {
                let target = context.record.target_container_name.clone();
                let Some(container) = find_container(&late.container_full_names, &target) else {
                    bail!("no container `{target}` for composable `{name}`");
                };
                let mut metadata = composable_member_metadata(&name, &context, false);
                let id = late.id_alloc.allocate(
                    MemberKind::Composable,
                    metadata.namespace.as_deref(),
                    &name,
                );
                let node = super::composable_metadata(&context, &name, false, sched, None);
                self.excluded.insert(id.clone());
                metadata.id = id.clone();
                self.metadata.write().await.insert(id.clone(), metadata);
                self.virtual_member_routing
                    .insert(id.clone(), container.clone());
                super::state_reducer::init(&self.shared_state, &id, MemberState::Unloaded);
                drop(guard);
                self.send_control(
                    &container,
                    ControlEvent::AddComposable {
                        name: id.clone(),
                        metadata: Box::new(node),
                    },
                )
                .await?;
                Ok(id)
            }
        }
    }

    async fn register(
        &self,
        id: &str,
        metadata: super::MemberMetadata,
        control_tx: mpsc::Sender<ControlEvent>,
    ) {
        if let Some(node_name) = &metadata.node_name {
            let fqn = crate::member_actor::member_id::fqn_join(
                metadata.namespace.as_deref().unwrap_or("/"),
                node_name,
            );
            self.node_fqn_map.write().await.insert(id.to_string(), fqn);
        }
        self.metadata.write().await.insert(id.to_string(), metadata);
        self.control_channels.insert(id.to_string(), control_tx);
    }
}
