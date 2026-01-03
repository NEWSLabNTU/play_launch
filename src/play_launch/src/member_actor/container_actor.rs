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
    events::{ControlEvent, StateEvent},
    state::{ActorConfig, ContainerState, NodeState},
};
use crate::execution::context::NodeContext;
use eyre::{Context as _, Result};
use std::{
    collections::HashMap,
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

                            // Transition to Pending to respawn
                            self.state = NodeState::Pending;
                            return Ok(true); // Continue to respawn
                        }
                        _ => {
                            warn!("{}: Unhandled control event: {:?}", self.name, event);
                        }
                    }
                }

                // Check for shutdown signal
                _ = self.shutdown_rx.changed() => {
                    if *self.shutdown_rx.borrow() {
                        debug!("{}: Shutdown signal received, waiting for container to exit", self.name);

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
                NodeState::Running { ref mut child, pid } => {
                    // Take ownership of child to avoid borrow issues
                    let child = std::mem::replace(
                        child,
                        tokio::process::Command::new("true").spawn().unwrap(),
                    );
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
