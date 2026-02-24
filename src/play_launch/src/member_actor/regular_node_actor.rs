//! Regular node actor implementation
//!
//! This module implements actors for regular ROS nodes and containers.
//! Each actor is a self-contained task that manages its own lifecycle.
//!
//! Phase 5: Provides both standalone function (run_regular_node) and trait-based actor (RegularNodeActor).

use super::{
    actor_traits::MemberActor,
    events::{ControlEvent, StateEvent},
    state::{ActorConfig, NodeState},
};
use crate::{execution::context::NodeContext, util::logging::is_verbose};
use eyre::{Context as _, Result};
use std::{
    collections::HashMap,
    fs::File,
    io::Write,
    path::{Path, PathBuf},
    process::ExitStatus,
    sync::{Arc, Mutex},
    time::Duration,
};
use tokio::sync::{mpsc, watch};
use tracing::{debug, error, info, warn};

/// Standalone async function for running a regular node (Phase 5)
///
/// This is the standalone function version that will be used with FuturesUnordered.
/// It contains the same logic as RegularNodeActor::run() but without the trait.
#[allow(clippy::too_many_arguments)]
pub async fn run_regular_node(
    name: String,
    context: NodeContext,
    config: ActorConfig,
    control_rx: mpsc::Receiver<ControlEvent>,
    state_tx: mpsc::Sender<StateEvent>,
    shutdown_rx: watch::Receiver<bool>,
    process_registry: Option<Arc<Mutex<HashMap<u32, PathBuf>>>>,
    shared_state: Arc<dashmap::DashMap<String, super::web_query::MemberState>>,
) -> Result<()> {
    // Create the actor and run it (wrapper approach for Phase 5)
    let actor = RegularNodeActor::new(
        name,
        context,
        config,
        control_rx,
        state_tx,
        shutdown_rx,
        process_registry,
        shared_state,
    );
    actor.run().await
}

/// Actor for regular ROS nodes and containers
///
/// This actor manages the full lifecycle of a regular node:
/// - Spawning the process
/// - Monitoring for exit
/// - Respawning if configured
/// - Handling control commands (stop, restart, etc.)
pub struct RegularNodeActor {
    /// Member name (for logging and identification)
    name: String,
    /// Node context with all execution data
    context: NodeContext,
    /// Actor configuration (respawn settings, etc.)
    config: ActorConfig,
    /// Current state machine state
    state: NodeState,
    /// Channel to receive control events
    control_rx: mpsc::Receiver<ControlEvent>,
    /// Channel to send state events
    state_tx: mpsc::Sender<StateEvent>,
    /// Shutdown signal receiver
    shutdown_rx: watch::Receiver<bool>,
    /// Process registry for monitoring (optional)
    process_registry: Option<
        std::sync::Arc<std::sync::Mutex<std::collections::HashMap<u32, std::path::PathBuf>>>,
    >,
    /// Shared state map for direct state updates
    shared_state: std::sync::Arc<dashmap::DashMap<String, super::web_query::MemberState>>,
}

impl RegularNodeActor {
    /// Create a new regular node actor
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        name: String,
        context: NodeContext,
        config: ActorConfig,
        control_rx: mpsc::Receiver<ControlEvent>,
        state_tx: mpsc::Sender<StateEvent>,
        shutdown_rx: watch::Receiver<bool>,
        process_registry: Option<
            std::sync::Arc<std::sync::Mutex<std::collections::HashMap<u32, std::path::PathBuf>>>,
        >,
        shared_state: std::sync::Arc<dashmap::DashMap<String, super::web_query::MemberState>>,
    ) -> Self {
        Self {
            name,
            context,
            config,
            state: NodeState::Pending,
            control_rx,
            state_tx,
            shutdown_rx,
            process_registry,
            shared_state,
        }
    }

    /// Handle the Pending state - spawn the process
    async fn handle_pending(&mut self) -> Result<()> {
        // Check for shutdown before spawning
        if *self.shutdown_rx.borrow() {
            debug!("[{}] Shutdown signal detected in Pending state", self.name);
            self.transition_to_stopped(None).await?;
            return Ok(());
        }

        // Prepare execution context
        let exec = self
            .context
            .to_exec_context(self.config.pgid)
            .with_context(|| format!("Failed to prepare execution context for {}", self.name))?;

        let log_name = exec.log_name.clone();
        let output_dir = exec.output_dir.clone();
        let mut command = exec.command;

        // Spawn the process
        let child = match command.spawn() {
            Ok(child) => child,
            Err(err) => {
                error!("[{}] Unable to start: {}", log_name, err);
                error!("Check {}", output_dir.display());

                if !self.config.respawn_enabled {
                    self.transition_to_failed(format!("Spawn failed: {}", err))
                        .await?;
                    return Err(err.into());
                }

                // Transition to Respawning on spawn failure
                warn!(
                    "[{}] Will respawn in {:.1}s",
                    log_name, self.config.respawn_delay
                );
                self.transition_to_respawning(None, 0).await?;
                return Ok(());
            }
        };

        let pid = child.id().expect("Child process should have PID");

        // Register PID for monitoring
        if let Some(ref registry) = self.process_registry {
            if let Ok(mut reg) = registry.lock() {
                reg.insert(pid, output_dir.clone());
                debug!("[{}] Registered PID {} for monitoring", self.name, pid);
            }

            // Initialize metrics CSV
            if let Err(e) = crate::monitoring::resource_monitor::initialize_metrics_csv(&output_dir)
            {
                warn!("[{}] Failed to initialize metrics CSV: {}", self.name, e);
            }
        }

        // Write PID file
        if let Err(e) = write_pid_file(&output_dir, pid) {
            warn!("[{}] Failed to write PID file: {}", self.name, e);
        }

        // Transition to Running state
        self.state = NodeState::Running { child, pid };

        // Send Started event
        self.state_tx
            .send(StateEvent::Started {
                name: self.name.clone(),
                pid,
            })
            .await
            .ok();

        // Update shared state directly
        self.shared_state.insert(
            self.name.clone(),
            super::web_query::MemberState::Running { pid },
        );

        if is_verbose() {
            info!("[{}] Started with PID {}", self.name, pid);
        }

        Ok(())
    }

    /// Handle the Running state - wait for exit or control events
    ///
    /// Returns false if the actor should terminate
    async fn handle_running(&mut self) -> Result<bool> {
        let (mut child, pid) = match std::mem::replace(&mut self.state, NodeState::Pending) {
            NodeState::Running { child, pid } => (child, pid),
            _ => unreachable!("handle_running called in non-Running state"),
        };

        tokio::select! {
            // Process exited
            result = child.wait() => {
                // Unregister PID
                if let Some(ref registry) = self.process_registry {
                    if let Ok(mut reg) = registry.lock() {
                        reg.remove(&pid);
                        debug!("[{}] Unregistered PID {}", self.name, pid);
                    }
                }

                let exit_code = match result {
                    Ok(status) => {
                        self.save_status(&status)?;
                        status.code()
                    }
                    Err(err) => {
                        error!("[{}] Failed to wait for process: {}", self.name, err);
                        None
                    }
                };

                // Send Exited event
                self.state_tx.send(StateEvent::Exited {
                    name: self.name.clone(),
                    exit_code,
                }).await.ok();

                // Decide next state
                if self.config.respawn_enabled {
                    self.transition_to_respawning(exit_code, 0).await?;
                    Ok(true) // Continue running
                } else {
                    self.transition_to_stopped(exit_code).await?;
                    Ok(true) // Keep actor alive to allow Start/Restart from Web UI
                }
            }

            // Control event received
            Some(event) = self.control_rx.recv() => {
                // Put child back in state for control handler
                self.state = NodeState::Running { child, pid };
                self.handle_control_event(event).await
            }

            // Shutdown signal
            _ = self.shutdown_rx.changed() => {
                if *self.shutdown_rx.borrow() {
                    debug!("[{}] Shutdown signal received in Running state", self.name);

                    // On Unix, kill_process_group() already sent SIGTERM to all processes
                    // We just need to wait for this child to exit
                    #[cfg(unix)]
                    {
                        debug!("[{}] Waiting for child to exit (killed by PGID)", self.name);
                        match child.wait().await {
                            Ok(status) => {
                                debug!("[{}] Process exited with status: {:?}", self.name, status);
                            }
                            Err(e) => {
                                warn!("[{}] Failed to wait for process: {}", self.name, e);
                            }
                        }
                    }
                    #[cfg(not(unix))]
                    {
                        // On non-Unix, we need to kill the child explicitly
                        child.kill().await.ok();
                        if let Ok(status) = child.wait().await {
                            debug!("[{}] Process exited with status: {:?}", self.name, status);
                        }
                    }

                    // Unregister PID
                    if let Some(ref registry) = self.process_registry {
                        if let Ok(mut reg) = registry.lock() {
                            reg.remove(&pid);
                        }
                    }

                    self.transition_to_stopped(None).await?;
                    Ok(false) // Terminate
                } else {
                    // Put child back and continue
                    self.state = NodeState::Running { child, pid };
                    Ok(true)
                }
            }
        }
    }

    /// Handle the Respawning state - wait for delay then respawn
    ///
    /// Returns false if the actor should terminate
    async fn handle_respawning(&mut self) -> Result<bool> {
        let (exit_code, attempt) = match &self.state {
            NodeState::Respawning { exit_code, attempt } => (*exit_code, *attempt),
            _ => unreachable!("handle_respawning called in non-Respawning state"),
        };

        // Check if we've exceeded max attempts
        if let Some(max_attempts) = self.config.max_respawn_attempts {
            if attempt >= max_attempts {
                error!(
                    "[{}] Max respawn attempts ({}) reached",
                    self.name, max_attempts
                );
                self.transition_to_failed(format!(
                    "Max respawn attempts ({}) reached",
                    max_attempts
                ))
                .await?;
                return Ok(false);
            }
        }

        let delay = Duration::from_secs_f64(self.config.respawn_delay);

        // Send Respawning event
        self.state_tx
            .send(StateEvent::Respawning {
                name: self.name.clone(),
                attempt: attempt + 1,
                delay: self.config.respawn_delay,
            })
            .await
            .ok();

        // Update shared state directly
        self.shared_state.insert(
            self.name.clone(),
            super::web_query::MemberState::Respawning {
                attempt: attempt + 1,
            },
        );

        if is_verbose() {
            info!(
                "[{}] Respawning in {:.1}s (attempt {})",
                self.name,
                self.config.respawn_delay,
                attempt + 1
            );
        }

        // Wait for delay or shutdown/control event
        tokio::select! {
            _ = tokio::time::sleep(delay) => {
                // Delay finished, transition back to Pending to spawn again
                self.state = NodeState::Pending;
                Ok(true) // Continue to spawn
            }

            Some(event) = self.control_rx.recv() => {
                self.handle_control_event(event).await
            }

            _ = self.shutdown_rx.changed() => {
                if *self.shutdown_rx.borrow() {
                    debug!("[{}] Shutdown signal received during respawn delay", self.name);
                    self.transition_to_stopped(exit_code).await?;
                    Ok(false) // Terminate
                } else {
                    Ok(true) // Continue
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
                self.handle_control_event(event).await
            }

            _ = self.shutdown_rx.changed() => {
                if *self.shutdown_rx.borrow() {
                    debug!("[{}] Shutdown signal received in Stopped/Failed state", self.name);
                    Ok(false) // Terminate
                } else {
                    Ok(true) // Continue
                }
            }
        }
    }

    /// Handle control events
    ///
    /// Returns false if the actor should terminate
    async fn handle_control_event(&mut self, event: ControlEvent) -> Result<bool> {
        match event {
            ControlEvent::Start => {
                debug!("[{}] Start requested", self.name);

                // If not running, transition to Pending to spawn
                match &self.state {
                    NodeState::Stopped { .. } | NodeState::Failed { .. } => {
                        self.state = NodeState::Pending;
                        debug!("[{}] Transitioning to Pending for spawn", self.name);
                        Ok(true) // Continue to spawn
                    }
                    NodeState::Running { .. } => {
                        debug!("[{}] Already running, ignoring start request", self.name);
                        Ok(true) // Already running, ignore
                    }
                    _ => {
                        debug!(
                            "[{}] In transition state, ignoring start request",
                            self.name
                        );
                        Ok(true) // In transition, ignore
                    }
                }
            }

            ControlEvent::Stop => {
                debug!("[{}] Stop requested", self.name);

                // Kill process if running
                if let NodeState::Running { mut child, pid } =
                    std::mem::replace(&mut self.state, NodeState::Pending)
                {
                    child.kill().await.ok();

                    // Unregister PID
                    if let Some(ref registry) = self.process_registry {
                        if let Ok(mut reg) = registry.lock() {
                            reg.remove(&pid);
                        }
                    }
                }

                self.transition_to_stopped(None).await?;
                Ok(true) // Keep actor alive to receive Start commands
            }

            ControlEvent::Restart => {
                debug!("[{}] Restart requested", self.name);

                // Kill process if running
                if let NodeState::Running { mut child, pid } =
                    std::mem::replace(&mut self.state, NodeState::Pending)
                {
                    child.kill().await.ok();

                    // Unregister PID
                    if let Some(ref registry) = self.process_registry {
                        if let Ok(mut reg) = registry.lock() {
                            reg.remove(&pid);
                        }
                    }
                }

                // Transition to Pending to spawn again
                self.state = NodeState::Pending;
                Ok(true) // Continue to spawn
            }

            ControlEvent::ToggleRespawn(enabled) => {
                info!("[{}] Toggling respawn: {}", self.name, enabled);
                self.config.respawn_enabled = enabled;
                Ok(true) // Continue
            }

            #[cfg(unix)]
            ControlEvent::Kill(signal) => {
                info!("[{}] Sending signal {:?}", self.name, signal);

                if let NodeState::Running { pid, .. } = &self.state {
                    use nix::{sys::signal, unistd::Pid};

                    if let Err(e) = signal::kill(Pid::from_raw(*pid as i32), signal) {
                        warn!("[{}] Failed to send signal: {}", self.name, e);
                    }
                }

                Ok(true) // Continue
            }

            ControlEvent::LoadComposable { .. } => {
                warn!(
                    "[{}] LoadComposable not supported for regular nodes",
                    self.name
                );
                Ok(true) // Ignore
            }
            ControlEvent::UnloadComposable { .. } => {
                warn!(
                    "[{}] UnloadComposable not supported for regular nodes",
                    self.name
                );
                Ok(true) // Ignore
            }
            ControlEvent::LoadAllComposables => {
                warn!(
                    "[{}] LoadAllComposables not supported for regular nodes",
                    self.name
                );
                Ok(true) // Ignore
            }
            ControlEvent::UnloadAllComposables => {
                warn!(
                    "[{}] UnloadAllComposables not supported for regular nodes",
                    self.name
                );
                Ok(true) // Ignore
            }
            ControlEvent::ToggleComposableAutoLoad { .. } => {
                warn!(
                    "[{}] ToggleComposableAutoLoad not supported for regular nodes",
                    self.name
                );
                Ok(true) // Ignore
            }
            ControlEvent::Load => {
                warn!("[{}] Load not supported for regular nodes", self.name);
                Ok(true) // Ignore
            }
            ControlEvent::Unload => {
                warn!("[{}] Unload not supported for regular nodes", self.name);
                Ok(true) // Ignore
            }
            ControlEvent::ToggleAutoLoad(_) => {
                warn!(
                    "[{}] ToggleAutoLoad not supported for regular nodes",
                    self.name
                );
                Ok(true) // Ignore
            }
        }
    }

    /// Transition to Respawning state
    async fn transition_to_respawning(
        &mut self,
        exit_code: Option<i32>,
        attempt: u32,
    ) -> Result<()> {
        self.state = NodeState::Respawning { exit_code, attempt };
        Ok(())
    }

    /// Transition to Stopped state
    async fn transition_to_stopped(&mut self, exit_code: Option<i32>) -> Result<()> {
        self.state = NodeState::Stopped { exit_code };

        // Send Terminated event
        self.state_tx
            .send(StateEvent::Terminated {
                name: self.name.clone(),
            })
            .await
            .ok();

        // Update shared state directly
        self.shared_state
            .insert(self.name.clone(), super::web_query::MemberState::Stopped);

        if is_verbose() {
            info!("[{}] Stopped", self.name);
        }

        Ok(())
    }

    /// Transition to Failed state
    async fn transition_to_failed(&mut self, error: String) -> Result<()> {
        self.state = NodeState::Failed {
            error: error.clone(),
        };

        // Send Failed event
        self.state_tx
            .send(StateEvent::Failed {
                name: self.name.clone(),
                error: error.clone(),
            })
            .await
            .ok();

        // Update shared state directly
        self.shared_state.insert(
            self.name.clone(),
            super::web_query::MemberState::Failed { error },
        );

        error!("[{}] Failed", self.name);

        Ok(())
    }

    /// Save process exit status to file
    fn save_status(&self, status: &ExitStatus) -> Result<()> {
        let status_path = self.config.output_dir.join("status");
        let mut status_file = File::create(status_path)?;

        if let Some(code) = status.code() {
            writeln!(status_file, "{}", code)?;
        } else {
            writeln!(status_file, "[none]")?;
        }

        // Log status
        if status.success() {
            if is_verbose() {
                info!("[{}] Exited successfully", self.name);
            }
        } else {
            match status.code() {
                Some(code) => {
                    error!("[{}] Exited with code {}", self.name, code);
                    error!("Check {}", self.config.output_dir.display());
                }
                None => {
                    error!("[{}] Exited without code", self.name);
                    error!("Check {}", self.config.output_dir.display());
                }
            }
        }

        Ok(())
    }
}

impl MemberActor for RegularNodeActor {
    async fn run(mut self) -> Result<()> {
        loop {
            match &self.state {
                NodeState::Pending => {
                    self.handle_pending().await?;
                }
                NodeState::Running { .. } => {
                    if !self.handle_running().await? {
                        break; // Terminate
                    }
                }
                NodeState::Respawning { .. } => {
                    if !self.handle_respawning().await? {
                        break; // Terminate
                    }
                }
                NodeState::Stopped { .. } | NodeState::Failed { .. } => {
                    // Keep actor alive to receive Start/Restart commands from Web UI
                    if !self.handle_stopped().await? {
                        break; // Terminate on shutdown
                    }
                }
            }
        }

        Ok(())
    }

    fn name(&self) -> &str {
        &self.name
    }
}

/// Write PID to file
fn write_pid_file(output_dir: &Path, pid: u32) -> Result<()> {
    let pid_path = output_dir.join("pid");
    let mut pid_file = File::create(pid_path)?;
    writeln!(pid_file, "{}", pid)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_actor_creation() {
        // This is a placeholder test - full integration tests will be added
        // in the integration test suite
    }
}
