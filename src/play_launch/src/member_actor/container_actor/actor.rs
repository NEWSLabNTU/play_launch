//! The container actor: process lifecycle + the run loop (phase-51.4).
//!
//! The former god struct is now a composition: this actor owns the child
//! process and container state machine; [`super::supervisor`] owns the
//! composables; [`super::ros_client`] owns the ROS clients and every
//! container service name.

use super::{
    ros_client::ContainerClients,
    supervisor::{ComposableNodeMetadata, ComposableSupervisor},
    timing::{LOADING_CHECK_INTERVAL, LoadTimings},
};
use crate::{
    execution::context::NodeContext,
    member_actor::{
        actor_traits::MemberActor,
        container_control::ContainerControlEvent,
        events::{ControlEvent, StateEvent, emit},
        model::{ActorConfig, BlockReason, ComposableState, ContainerState, NodeState},
    },
};
use eyre::Result;
use std::{
    collections::HashMap,
    path::PathBuf,
    sync::{Arc, Mutex},
};
use tokio::sync::{mpsc, watch};
use tracing::{debug, error, warn};

/// Bundled configuration for constructing a [`ContainerActor`].
///
/// Groups the non-channel parameters that configure a container actor,
/// reducing argument count on [`ContainerActor::new`].
pub struct ContainerActorParams {
    pub context: NodeContext,
    pub config: ActorConfig,
    pub process_registry: Option<Arc<Mutex<HashMap<u32, PathBuf>>>>,
    pub ros_node: Option<Arc<rclrs::Node>>,
    pub use_component_events: bool,
    /// Tunable LoadNode-path timings (phase-52.2)
    pub timings: LoadTimings,
}

/// Container actor that supervises composable nodes
pub struct ContainerActor {
    /// Unique name for this actor (canonical member id)
    pub(super) name: String,
    /// Node execution context
    pub(super) context: NodeContext,
    /// Actor configuration
    pub(super) config: ActorConfig,
    /// Current state
    pub(super) state: NodeState,
    /// Channel to receive control events
    pub(super) control_rx: mpsc::Receiver<ControlEvent>,
    /// Channel to send state events
    pub(super) state_tx: mpsc::Sender<StateEvent>,
    /// Shutdown signal receiver
    pub(super) shutdown_rx: watch::Receiver<bool>,
    /// Process registry for I/O monitoring
    pub(super) process_registry: Option<Arc<Mutex<HashMap<u32, PathBuf>>>>,
    /// Container state broadcast to composable nodes (Phase 10)
    pub(super) container_state_tx: watch::Sender<ContainerState>,
    /// Channel to receive LoadNode requests from composable nodes
    pub(super) load_control_rx: mpsc::Receiver<ContainerControlEvent>,
    /// ROS node for service calls (shared across all containers)
    pub(super) ros_node: Option<Arc<rclrs::Node>>,
    /// ROS clients + ComponentEvent subscription (created on start)
    pub(super) clients: ContainerClients,
    /// Composable-node supervision (map, queue, completions)
    pub(super) supervisor: ComposableSupervisor,
    /// Whether to subscribe to ComponentEvent topic (only when using play_launch_container)
    pub(super) use_component_events: bool,
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
        let supervisor = ComposableSupervisor::new(
            name.clone(),
            state_tx.clone(),
            params.timings,
            params.config.startup.clone(),
        );

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
            load_control_rx,
            ros_node: params.ros_node,
            clients: ContainerClients::default(),
            supervisor,
            use_component_events: params.use_component_events,
        }
    }

    /// Get a receiver for container state (for composable nodes)
    pub fn container_state_rx(&self) -> watch::Receiver<ContainerState> {
        self.container_state_tx.subscribe()
    }

    /// Add a composable node to be managed by this container (Phase 12)
    pub fn add_composable_node(&mut self, name: String, metadata: ComposableNodeMetadata) {
        self.supervisor.add_composable_node(name, metadata);
    }

    /// Register a process PID in the shared process registry for I/O monitoring.
    pub(super) fn register_process(&self, pid: u32) {
        if let Some(ref registry) = self.process_registry
            && let Ok(mut reg) = registry.lock()
        {
            reg.insert(pid, self.config.output_dir.clone());
        }
    }

    /// Unregister a process PID from the shared process registry.
    pub(super) fn unregister_process(&self, pid: u32) {
        if let Some(ref registry) = self.process_registry
            && let Ok(mut reg) = registry.lock()
        {
            reg.remove(&pid);
        }
    }

    /// Handle the Pending state
    async fn handle_pending(&mut self) -> Result<bool> {
        debug!("{}: Spawning container process", self.name);

        // Drain any pending loads (container not running)
        self.supervisor.drain_queue("Container not running");

        // Phase 12: Transition composable nodes to Blocked while container is starting
        self.supervisor
            .transition_all_composables_to_blocked(BlockReason::ContainerNotStarted)
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
                emit(
                    &self.state_tx,
                    StateEvent::Started {
                        name: self.name.clone(),
                        pid,
                    },
                )
                .await;

                // Create ROS clients now that the container is running
                if self.clients.load_client.is_none() {
                    if let Some(ros_node) = self.ros_node.clone() {
                        let fqn = self.full_node_name();
                        self.clients
                            .create(&self.name, &ros_node, &fqn, self.use_component_events);
                    } else {
                        warn!(
                            "{}: No ROS node available for LoadNode service calls",
                            self.name
                        );
                    }
                }

                // Phase 12: Transition blocked composable nodes to Unloaded state now that container is running
                let blocked_count = self
                    .supervisor
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
                    .supervisor
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

                // Emit StateEvent::Unloaded for each node
                for node_name in &unloaded_nodes {
                    emit(
                        &self.state_tx,
                        StateEvent::Unloaded {
                            name: node_name.clone(),
                        },
                    )
                    .await;
                }

                // Phase 12: Auto-load composable nodes marked with auto_load=true
                self.supervisor
                    .handle_load_all_composables(&self.clients)
                    .await;

                self.state = NodeState::Running { child, pid };
                Ok(true) // Continue running
            }
            Err(e) => {
                error!("{}: Failed to spawn container: {:#}", self.name, e);

                // Phase 12: Transition composable nodes to Blocked
                self.supervisor
                    .transition_all_composables_to_blocked(BlockReason::ContainerFailed)
                    .await;

                // Broadcast failed state to composable nodes
                let _ = self.container_state_tx.send(ContainerState::Failed);

                emit(
                    &self.state_tx,
                    StateEvent::Failed {
                        name: self.name.clone(),
                        error: e.to_string(),
                    },
                )
                .await;

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

        // Phase 38: apply resolved Linux scheduling to the container process
        // itself. Composable nodes loaded into it are scheduled separately,
        // per-process, on LOADED (phase 38.9, see component_events.rs).
        // Runs on every entry to Running, so respawns re-apply automatically.
        if self.config.sched_mode != crate::execution::sched_apply::SchedApplyMode::Off
            && let Some(tier) = &self.config.sched
            && let Err(e) = crate::execution::rt_helper_client::apply_sched(
                self.config.sched_helper.as_ref(),
                pid,
                tier,
            )
            .await
        {
            if self.config.sched_mode == crate::execution::sched_apply::SchedApplyMode::Strict {
                return Err(eyre::eyre!(
                    "sched apply failed for container {}: {e}",
                    self.name
                ));
            }
            warn!("{}: sched apply failed (pid {}): {}", self.name, pid, e);
        }

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
                            self.supervisor.handle_load_composable(&name, &self.clients).await;
                        }
                        ControlEvent::UnloadComposable { name } => {
                            self.supervisor.handle_unload_composable(&name, &self.clients).await;
                        }
                        ControlEvent::LoadAllComposables => {
                            self.supervisor.handle_load_all_composables(&self.clients).await;
                        }
                        ControlEvent::UnloadAllComposables => {
                            self.supervisor.handle_unload_all_composables(&self.clients).await;
                        }
                        ControlEvent::ToggleComposableAutoLoad { name, enabled } => {
                            self.supervisor.handle_toggle_composable_auto_load(&name, enabled).await;
                        }
                        _ => {
                            warn!("{}: Unhandled control event: {:?}", self.name, event);
                        }
                    }
                }

                Some(event) = self.load_control_rx.recv() => {
                    self.supervisor.handle_load_control_event(event, &self.clients).await;
                }

                Some(completion) = self.supervisor.load_completion_rx.recv() => {
                    let has_event_sub = self.clients.has_event_sub();
                    self.supervisor.handle_load_completion(completion, has_event_sub).await;
                }

                result = async {
                    match &mut self.supervisor.current_unload {
                        Some(unload) => (&mut unload.task).await,
                        None => std::future::pending().await,
                    }
                }, if self.supervisor.current_unload.is_some() => {
                    let unload = self.supervisor.current_unload.take().unwrap();
                    self.supervisor.handle_unload_completion(result, unload.composable_name).await;
                }

                Some(event) = async {
                    match &mut self.clients.component_event_rx {
                        Some(rx) => rx.recv().await,
                        None => std::future::pending().await,
                    }
                } => {
                    self.supervisor.handle_component_event(event, &self.config).await;
                }

                _ = loading_timeout_interval.tick() => {
                    self.supervisor.check_loading_timeouts().await;
                    self.supervisor.rescue_lost_loads(&self.clients).await;
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
