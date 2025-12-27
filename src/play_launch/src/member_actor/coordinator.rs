//! Coordinator for managing member actors
//!
//! The MemberCoordinator is a lightweight coordinator that:
//! - Spawns and tracks actor tasks
//! - Aggregates state events from all actors
//! - Broadcasts shutdown signals
//! - Provides control handles for individual actors

use super::{
    events::{ControlEvent, StateEvent},
    web_query::{HealthSummary, MemberState, MemberSummary, MemberType},
};
use eyre::{Context, Result};
use std::{collections::HashMap, path::PathBuf};
use tokio::{
    sync::{mpsc, watch},
    task::JoinHandle,
};

/// Handle to a running actor
pub struct ActorHandle {
    /// Task handle for the actor
    task: JoinHandle<Result<()>>,
    /// Channel to send control events to the actor
    control_tx: mpsc::Sender<ControlEvent>,
}

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

/// Actor entry combining handle and metadata
struct ActorEntry {
    handle: ActorHandle,
    metadata: MemberMetadata,
    /// Current state (updated via StateEvents)
    state: MemberState,
}

impl ActorHandle {
    /// Create a new actor handle
    pub fn new(task: JoinHandle<Result<()>>, control_tx: mpsc::Sender<ControlEvent>) -> Self {
        Self { task, control_tx }
    }

    /// Send a control event to the actor
    pub async fn send_control(&self, event: ControlEvent) -> Result<()> {
        self.control_tx
            .send(event)
            .await
            .context("Failed to send control event to actor")
    }

    /// Check if the actor task has finished
    pub fn is_finished(&self) -> bool {
        self.task.is_finished()
    }

    /// Abort the actor task
    pub fn abort(&self) {
        self.task.abort();
    }
}

/// Coordinator for managing member actors
///
/// The coordinator is responsible for:
/// - Spawning actor tasks
/// - Tracking actor handles
/// - Aggregating state events
/// - Broadcasting shutdown signals
/// - Providing web UI query interface
pub struct MemberCoordinator {
    /// Map of member name to actor entry (handle + metadata + state)
    actors: HashMap<String, ActorEntry>,
    /// Receiver for state events from all actors
    state_rx: mpsc::Receiver<StateEvent>,
    /// Sender for state events (cloned and given to actors)
    state_tx: mpsc::Sender<StateEvent>,
    /// Sender for shutdown signal (cloned and given to actors)
    shutdown_tx: watch::Sender<bool>,
    /// Receiver for shutdown signal (for internal use)
    shutdown_rx: watch::Receiver<bool>,
}

impl MemberCoordinator {
    /// Create a new coordinator
    pub fn new() -> Self {
        let (state_tx, state_rx) = mpsc::channel(100);
        let (shutdown_tx, shutdown_rx) = watch::channel(false);

        Self {
            actors: HashMap::new(),
            state_rx,
            state_tx,
            shutdown_tx,
            shutdown_rx,
        }
    }

    /// Get a state event sender for actors
    pub fn state_tx(&self) -> mpsc::Sender<StateEvent> {
        self.state_tx.clone()
    }

    /// Get a shutdown receiver for actors
    pub fn shutdown_rx(&self) -> watch::Receiver<bool> {
        self.shutdown_rx.clone()
    }

    /// Register an actor with the coordinator
    ///
    /// This stores the actor handle, metadata, and initial state for later use.
    pub fn register_actor(
        &mut self,
        name: String,
        handle: ActorHandle,
        metadata: MemberMetadata,
        initial_state: MemberState,
    ) {
        let entry = ActorEntry {
            handle,
            metadata,
            state: initial_state,
        };
        self.actors.insert(name, entry);
    }

    /// Update member state based on a StateEvent
    ///
    /// This is called internally when processing state events to keep
    /// the coordinator's view of actor state up to date.
    pub fn update_state(&mut self, event: &StateEvent) {
        match event {
            StateEvent::Started { name, pid } => {
                if let Some(entry) = self.actors.get_mut(name) {
                    entry.state = MemberState::Running { pid: *pid };
                }
            }
            StateEvent::Exited { name, exit_code } => {
                if let Some(entry) = self.actors.get_mut(name) {
                    if exit_code.is_some() {
                        entry.state = MemberState::Failed {
                            error: format!("Exited with code {:?}", exit_code),
                        };
                    } else {
                        entry.state = MemberState::Stopped;
                    }
                }
            }
            StateEvent::Respawning { name, attempt, .. } => {
                if let Some(entry) = self.actors.get_mut(name) {
                    entry.state = MemberState::Respawning { attempt: *attempt };
                }
            }
            StateEvent::Terminated { name } => {
                if let Some(entry) = self.actors.get_mut(name) {
                    entry.state = MemberState::Stopped;
                }
            }
            StateEvent::Failed { name, error } => {
                if let Some(entry) = self.actors.get_mut(name) {
                    entry.state = MemberState::Failed {
                        error: error.clone(),
                    };
                }
            }
            StateEvent::LoadStarted { name } => {
                if let Some(entry) = self.actors.get_mut(name) {
                    entry.state = MemberState::Loading;
                }
            }
            StateEvent::LoadSucceeded {
                name, unique_id, ..
            } => {
                if let Some(entry) = self.actors.get_mut(name) {
                    entry.state = MemberState::Loaded {
                        unique_id: *unique_id,
                    };
                }
            }
            StateEvent::LoadFailed { name, error } => {
                if let Some(entry) = self.actors.get_mut(name) {
                    entry.state = MemberState::Failed {
                        error: error.clone(),
                    };
                }
            }
            StateEvent::Blocked { name, reason } => {
                if let Some(entry) = self.actors.get_mut(name) {
                    use super::{
                        state::BlockReason as ActorBlockReason,
                        web_query::BlockReason as WebBlockReason,
                    };

                    let web_reason = match reason {
                        ActorBlockReason::Stopped => WebBlockReason::ContainerStopped,
                        ActorBlockReason::Failed => WebBlockReason::ContainerFailed,
                        ActorBlockReason::NotStarted => WebBlockReason::ContainerNotStarted,
                    };

                    entry.state = MemberState::Blocked { reason: web_reason };
                }
            }
        }
    }

    /// Spawn a regular node actor
    ///
    /// This creates and spawns a RegularNodeActor for the given node context.
    pub fn spawn_regular_node(
        &mut self,
        name: String,
        context: crate::execution::context::NodeContext,
        config: super::state::ActorConfig,
        process_registry: Option<
            std::sync::Arc<std::sync::Mutex<std::collections::HashMap<u32, std::path::PathBuf>>>,
        >,
    ) -> Result<()> {
        use super::regular_node_actor::RegularNodeActor;

        // Create channels for the actor
        let (control_tx, control_rx) = mpsc::channel(10);
        let state_tx = self.state_tx();
        let shutdown_rx = self.shutdown_rx();

        // Create metadata from context
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

        // Create the actor
        let actor = RegularNodeActor::new(
            name.clone(),
            context,
            config,
            control_rx,
            state_tx,
            shutdown_rx,
            process_registry,
        );

        // Spawn the actor task
        let task = tokio::spawn(async move {
            use super::actor_traits::MemberActor;
            actor.run().await
        });

        // Register the actor handle
        let handle = ActorHandle::new(task, control_tx);
        self.register_actor(name, handle, metadata, MemberState::Pending);

        Ok(())
    }

    /// Spawn a container actor
    ///
    /// This creates and spawns a ContainerActor for the given node context.
    /// Returns the container's state receiver for composable nodes to watch.
    #[allow(clippy::too_many_arguments)]
    pub fn spawn_container(
        &mut self,
        name: String,
        context: crate::execution::context::NodeContext,
        config: super::state::ActorConfig,
        process_registry: Option<
            std::sync::Arc<std::sync::Mutex<std::collections::HashMap<u32, std::path::PathBuf>>>,
        >,
    ) -> Result<watch::Receiver<super::state::ContainerState>> {
        use super::container_actor::ContainerActor;

        // Create channels for the actor
        let (control_tx, control_rx) = mpsc::channel(10);
        let state_tx = self.state_tx();
        let shutdown_rx = self.shutdown_rx();

        // Create metadata from context
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

        // Create the actor
        let actor = ContainerActor::new(
            name.clone(),
            context,
            config,
            control_rx,
            state_tx,
            shutdown_rx,
            process_registry,
        );

        // Get container state receiver before spawning
        let container_state_rx = actor.container_state_rx();

        // Spawn the actor task
        let task = tokio::spawn(async move {
            use super::actor_traits::MemberActor;
            actor.run().await
        });

        // Register the actor handle
        let handle = ActorHandle::new(task, control_tx);
        self.register_actor(name, handle, metadata, MemberState::Pending);

        Ok(container_state_rx)
    }

    /// Spawn a composable node actor
    ///
    /// This creates and spawns a ComposableNodeActor for the given composable node context.
    #[allow(clippy::too_many_arguments)]
    pub fn spawn_composable_node(
        &mut self,
        name: String,
        context: crate::execution::context::ComposableNodeContext,
        config: super::composable_node_actor::ComposableActorConfig,
        container_state_rx: watch::Receiver<super::state::ContainerState>,
        component_loader: crate::ros::component_loader::ComponentLoaderHandle,
    ) -> Result<()> {
        use super::composable_node_actor::ComposableNodeActor;

        // Create channels for the actor
        let (control_tx, control_rx) = mpsc::channel(10);
        let state_tx = self.state_tx();
        let shutdown_rx = self.shutdown_rx();

        // Create metadata from context
        let metadata = MemberMetadata {
            name: name.clone(),
            member_type: MemberType::ComposableNode,
            package: Some(context.record.package.clone()),
            executable: context.record.plugin.clone(),
            namespace: Some(context.record.namespace.clone()),
            target_container: Some(context.record.target_container_name.clone()),
            output_dir: context.output_dir.clone(),
            respawn_enabled: None, // Composable nodes don't have respawn
            respawn_delay: None,
            exec_name: None,
            node_name: Some(context.record.node_name.clone()),
        };

        // Create the actor
        let actor = ComposableNodeActor::new(
            name.clone(),
            context,
            config,
            control_rx,
            state_tx,
            container_state_rx,
            shutdown_rx,
            component_loader,
        );

        // Spawn the actor task
        let task = tokio::spawn(async move {
            use super::actor_traits::MemberActor;
            actor.run().await
        });

        // Register the actor handle
        let handle = ActorHandle::new(task, control_tx);
        self.register_actor(name, handle, metadata, MemberState::Pending);

        Ok(())
    }

    // ===== Web UI Query Methods =====

    /// Get list of all members with current state
    pub fn list_members(&self) -> Vec<MemberSummary> {
        self.actors
            .values()
            .map(|entry| {
                let pid = match &entry.state {
                    MemberState::Running { pid } => Some(*pid),
                    _ => None,
                };

                // Read stderr info from log files
                let stderr_path = entry.metadata.output_dir.join("err");
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

                MemberSummary {
                    name: entry.metadata.name.clone(),
                    member_type: entry.metadata.member_type,
                    state: entry.state.clone(),
                    pid,
                    package: entry.metadata.package.clone(),
                    executable: entry.metadata.executable.clone(),
                    namespace: entry.metadata.namespace.clone(),
                    target_container: entry.metadata.target_container.clone(),
                    is_container: entry.metadata.member_type == MemberType::Container,
                    exec_name: entry.metadata.exec_name.clone(),
                    node_name: entry.metadata.node_name.clone(),
                    stderr_last_modified,
                    stderr_size,
                    stderr_preview,
                    respawn_enabled: entry.metadata.respawn_enabled,
                    respawn_delay: entry.metadata.respawn_delay,
                    output_dir: entry.metadata.output_dir.clone(),
                }
            })
            .collect()
    }

    /// Get detailed state for a specific member
    pub fn get_member_state(&self, name: &str) -> Option<MemberSummary> {
        self.actors.get(name).map(|entry| {
            let pid = match &entry.state {
                MemberState::Running { pid } => Some(*pid),
                _ => None,
            };

            // Read stderr info from log files
            let stderr_path = entry.metadata.output_dir.join("err");
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

            MemberSummary {
                name: entry.metadata.name.clone(),
                member_type: entry.metadata.member_type,
                state: entry.state.clone(),
                pid,
                package: entry.metadata.package.clone(),
                executable: entry.metadata.executable.clone(),
                namespace: entry.metadata.namespace.clone(),
                target_container: entry.metadata.target_container.clone(),
                is_container: entry.metadata.member_type == MemberType::Container,
                exec_name: entry.metadata.exec_name.clone(),
                node_name: entry.metadata.node_name.clone(),
                stderr_last_modified,
                stderr_size,
                stderr_preview,
                respawn_enabled: entry.metadata.respawn_enabled,
                respawn_delay: entry.metadata.respawn_delay,
                output_dir: entry.metadata.output_dir.clone(),
            }
        })
    }

    /// Get health summary statistics
    pub fn get_health_summary(&self) -> HealthSummary {
        let mut summary = HealthSummary::default();

        for entry in self.actors.values() {
            // Count by member type
            match entry.metadata.member_type {
                MemberType::Node => {
                    summary.nodes_total += 1;
                    match &entry.state {
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
                    match &entry.state {
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
                    match &entry.state {
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
            let stderr_path = entry.metadata.output_dir.join("err");
            if let Ok(metadata) = std::fs::metadata(&stderr_path) {
                if metadata.len() > 10 * 1024 {
                    summary.noisy += 1;
                }
            }
        }

        summary
    }

    // ===== Control Methods =====

    /// Send a control event to a specific actor
    pub async fn send_control(&self, name: &str, event: ControlEvent) -> Result<()> {
        let entry = self
            .actors
            .get(name)
            .ok_or_else(|| eyre::eyre!("Actor not found: {}", name))?;

        entry.handle.send_control(event).await
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

    /// Wait for the next state event from any actor
    pub async fn next_state_event(&mut self) -> Option<StateEvent> {
        self.state_rx.recv().await
    }

    /// Process state events until all actors terminate
    ///
    /// This method listens for state events and returns when all actors
    /// have finished or a shutdown is requested.
    pub async fn wait_for_completion(&mut self) -> Result<()> {
        loop {
            tokio::select! {
                event = self.state_rx.recv() => {
                    match event {
                        Some(event) => {
                            tracing::debug!("State event: {:?}", event);

                            // Check if all actors are finished
                            if self.all_actors_finished() {
                                break;
                            }
                        }
                        None => {
                            // All state senders dropped, actors must be done
                            break;
                        }
                    }
                }
                _ = self.shutdown_rx.changed() => {
                    if *self.shutdown_rx.borrow() {
                        tracing::debug!("Shutdown signal received, waiting for actors to finish");
                        break;
                    }
                }
            }
        }

        // Wait for all actor tasks to complete
        self.join_all_actors().await
    }

    /// Check if all actors have finished
    fn all_actors_finished(&self) -> bool {
        self.actors.values().all(|entry| entry.handle.is_finished())
    }

    /// Join all actor tasks and collect results
    async fn join_all_actors(&mut self) -> Result<()> {
        let mut errors = Vec::new();

        // Take all actor entries (consume the hashmap)
        let actors = std::mem::take(&mut self.actors);

        for (name, entry) in actors {
            match entry.handle.task.await {
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

    /// Get the number of registered actors
    pub fn actor_count(&self) -> usize {
        self.actors.len()
    }

    /// Check if a specific actor is registered
    pub fn has_actor(&self, name: &str) -> bool {
        self.actors.contains_key(name)
    }

    /// Abort all actors immediately
    pub fn abort_all(&self) {
        for entry in self.actors.values() {
            entry.handle.abort();
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

impl Default for MemberCoordinator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_coordinator_creation() {
        let coordinator = MemberCoordinator::new();
        assert_eq!(coordinator.actor_count(), 0);
    }

    #[tokio::test]
    async fn test_actor_registration() {
        let mut coordinator = MemberCoordinator::new();
        let (control_tx, _control_rx) = mpsc::channel(10);
        let task = tokio::spawn(async { Ok(()) });
        let handle = ActorHandle::new(task, control_tx);

        let metadata = MemberMetadata {
            name: "test_actor".to_string(),
            member_type: MemberType::Node,
            package: Some("test_pkg".to_string()),
            executable: "test_exec".to_string(),
            namespace: Some("/".to_string()),
            target_container: None,
            output_dir: PathBuf::from("/tmp/test"),
            respawn_enabled: Some(false),
            respawn_delay: None,
            exec_name: None,
            node_name: None,
        };

        coordinator.register_actor(
            "test_actor".to_string(),
            handle,
            metadata,
            MemberState::Pending,
        );
        assert_eq!(coordinator.actor_count(), 1);
        assert!(coordinator.has_actor("test_actor"));
    }

    #[tokio::test]
    async fn test_shutdown_signal() {
        let coordinator = MemberCoordinator::new();
        let mut shutdown_rx = coordinator.shutdown_rx();

        assert!(!*shutdown_rx.borrow());

        coordinator.shutdown().unwrap();
        shutdown_rx.changed().await.unwrap();

        assert!(*shutdown_rx.borrow());
    }

    #[tokio::test]
    async fn test_state_event_channel() {
        let mut coordinator = MemberCoordinator::new();
        let state_tx = coordinator.state_tx();

        let event = StateEvent::Started {
            name: "test_node".to_string(),
            pid: 123,
        };

        state_tx.send(event.clone()).await.unwrap();

        let received = coordinator.next_state_event().await.unwrap();
        assert_eq!(received.member_name(), "test_node");
    }
}
