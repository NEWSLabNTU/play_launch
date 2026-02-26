//! Process lifecycle management for container actors.
//!
//! Methods for spawning and stopping the container process, handling child
//! exit, shutdown, restart, and respawn. Also includes metadata updates and
//! ROS client cleanup.

use super::ContainerActor;
use crate::member_actor::{
    events::{ControlEvent, StateEvent},
    state::{BlockReason, ContainerState, NodeState},
};
use eyre::{Context as _, Result};
use tokio::{
    process::Command,
    time::{sleep, Duration},
};
use tracing::{debug, error, info, warn};

impl ContainerActor {
    /// Update metadata.json to include composable node information (Phase 12).
    pub fn update_metadata_with_composables(&self) -> Result<()> {
        use std::fs;

        let metadata_path = self.config.output_dir.join("metadata.json");

        // Read existing metadata
        let mut metadata = if metadata_path.exists() {
            let metadata_str =
                fs::read_to_string(&metadata_path).context("Failed to read metadata.json")?;
            serde_json::from_str::<serde_json::Value>(&metadata_str)
                .context("Failed to parse metadata.json")?
        } else {
            serde_json::json!({})
        };

        // Add composable node information
        let composable_nodes_info: Vec<serde_json::Value> = self
            .composable_nodes
            .iter()
            .map(|(name, entry)| {
                serde_json::json!({
                    "name": name,
                    "package": entry.metadata.package,
                    "plugin": entry.metadata.plugin,
                    "node_name": entry.metadata.node_name,
                    "namespace": entry.metadata.namespace,
                    "auto_load": entry.metadata.auto_load,
                })
            })
            .collect();

        metadata["composable_nodes"] = serde_json::json!(composable_nodes_info);
        metadata["composable_node_count"] = serde_json::json!(self.composable_nodes.len());

        // Write back
        let updated_json =
            serde_json::to_string_pretty(&metadata).context("Failed to serialize metadata")?;
        fs::write(&metadata_path, updated_json).context("Failed to write metadata.json")?;

        debug!(
            "{}: Updated metadata.json with {} composable nodes",
            self.name,
            self.composable_nodes.len()
        );

        Ok(())
    }

    /// Clear ROS service clients and event subscription so they're recreated on restart.
    pub(super) fn clear_ros_clients(&mut self) {
        self.load_client = None;
        self.unload_client = None;
        self.component_event_sub = None;
        self.component_event_rx = None;
    }

    /// Build the full node name from namespace and name.
    pub(super) fn full_node_name(&self) -> String {
        let namespace = self.context.record.namespace.as_deref().unwrap_or("/");
        let name = self.context.record.name.as_deref().unwrap_or(&self.name);

        if namespace == "/" {
            format!("/{}", name)
        } else if namespace.ends_with('/') {
            format!("{}{}", namespace, name)
        } else {
            format!("{}/{}", namespace, name)
        }
    }

    /// Spawn the container process.
    pub(super) async fn spawn_process(&self) -> Result<tokio::process::Child> {
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

    /// Handle child process exit during Running state.
    ///
    /// Returns `Some(should_continue)` to exit the select loop, or `None` to
    /// keep looping (not used here, always returns Some).
    pub(super) async fn handle_child_exit(
        &mut self,
        exit_code: Option<i32>,
        pid: u32,
    ) -> Result<bool> {
        info!("{}: Container exited with code {:?}", self.name, exit_code);

        // Unregister from process registry
        if let Some(ref registry) = self.process_registry {
            if let Ok(mut reg) = registry.lock() {
                reg.remove(&pid);
            }
        }

        // Drain load queue (container crashed)
        self.drain_queue("Container crashed");

        self.clear_ros_clients();

        // Phase 12: Transition composable nodes to Blocked
        self.transition_all_composables_to_blocked(BlockReason::Failed)
            .await;

        // Broadcast stopped state to composable nodes
        let _ = self.container_state_tx.send(ContainerState::Stopped);

        // Send state event
        let _ = self
            .state_tx
            .send(StateEvent::Exited {
                name: self.name.clone(),
                exit_code,
            })
            .await;

        // Check if respawn is enabled
        if self.config.respawn_enabled {
            self.state = NodeState::Respawning {
                exit_code,
                attempt: 0,
            };
        } else {
            self.state = NodeState::Stopped { exit_code };
            let _ = self
                .state_tx
                .send(StateEvent::Terminated {
                    name: self.name.clone(),
                })
                .await;

            // Update shared state directly
            self.shared_state.insert(
                self.name.clone(),
                super::super::web_query::MemberState::Stopped,
            );
        }

        Ok(true) // Continue (respawn or keep alive for Start commands)
    }

    /// Handle Stop control event: kill container, clean up, transition to Stopped.
    pub(super) async fn handle_stop_command(
        &mut self,
        child: &mut tokio::process::Child,
        pid: u32,
    ) -> Result<bool> {
        debug!(
            "{}: Received Stop command, killing container (PID: {})",
            self.name, pid
        );
        child.kill().await.ok();

        // Wait for process to exit
        let _ = child.wait().await;
        info!("{}: Container process terminated", self.name);

        // Unregister from process registry
        if let Some(ref registry) = self.process_registry {
            if let Ok(mut reg) = registry.lock() {
                reg.remove(&pid);
            }
        }

        // Drain load queue
        self.drain_queue("Container stopped");

        self.clear_ros_clients();

        // Phase 12: Transition composable nodes to Blocked
        debug!(
            "{}: Transitioning {} composable nodes to Blocked state (reason: Stopped)",
            self.name,
            self.composable_nodes.len()
        );
        self.transition_all_composables_to_blocked(BlockReason::Stopped)
            .await;

        // Broadcast stopped state
        let _ = self.container_state_tx.send(ContainerState::Stopped);

        self.state = NodeState::Stopped { exit_code: None };
        let _ = self
            .state_tx
            .send(StateEvent::Terminated {
                name: self.name.clone(),
            })
            .await;

        // Update shared state directly
        self.shared_state.insert(
            self.name.clone(),
            super::super::web_query::MemberState::Stopped,
        );

        Ok(true) // Keep actor alive to receive Start commands
    }

    /// Handle Restart control event: kill container, clean up, transition to Pending.
    pub(super) async fn handle_restart_command(
        &mut self,
        child: &mut tokio::process::Child,
        pid: u32,
    ) -> Result<bool> {
        debug!(
            "{}: Received Restart command, killing and respawning container",
            self.name
        );
        child.kill().await.ok();

        // Wait for process to exit
        let _ = child.wait().await;

        // Unregister from process registry
        if let Some(ref registry) = self.process_registry {
            if let Ok(mut reg) = registry.lock() {
                reg.remove(&pid);
            }
        }

        // Drain load queue
        self.drain_queue("Container restarting");

        self.clear_ros_clients();

        // Phase 12: Transition composable nodes to Blocked (will be reloaded on restart)
        self.transition_all_composables_to_blocked(BlockReason::Stopped)
            .await;

        // Transition to Pending to respawn
        self.state = NodeState::Pending;
        Ok(true) // Continue to respawn
    }

    /// Handle shutdown signal during Running state.
    pub(super) async fn handle_running_shutdown(
        &mut self,
        child: &mut tokio::process::Child,
        pid: u32,
    ) -> Result<bool> {
        debug!(
            "{}: Shutdown signal received, waiting for container to exit",
            self.name
        );

        // Drain load queue before shutting down
        self.drain_queue("Shutdown");

        // Phase 12: Transition composable nodes to Blocked
        self.transition_all_composables_to_blocked(BlockReason::Shutdown)
            .await;

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
        let _ = self
            .state_tx
            .send(StateEvent::Terminated {
                name: self.name.clone(),
            })
            .await;

        // Update shared state directly
        self.shared_state.insert(
            self.name.clone(),
            super::super::web_query::MemberState::Stopped,
        );

        Ok(false) // Stop actor
    }

    /// Handle the Respawning state.
    pub(super) async fn handle_respawning(&mut self, attempt: u32) -> Result<bool> {
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

        // Update shared state directly
        self.shared_state.insert(
            self.name.clone(),
            super::super::web_query::MemberState::Respawning { attempt },
        );

        // Check if max attempts reached
        if let Some(max_attempts) = self.config.max_respawn_attempts {
            if attempt >= max_attempts {
                error!(
                    "{}: Max respawn attempts ({}) reached",
                    self.name, max_attempts
                );
                let error_msg = format!("Max respawn attempts ({}) reached", max_attempts);
                self.state = NodeState::Failed {
                    error: error_msg.clone(),
                };

                // Broadcast failed state
                let _ = self.container_state_tx.send(ContainerState::Failed);

                let _ = self
                    .state_tx
                    .send(StateEvent::Failed {
                        name: self.name.clone(),
                        error: error_msg.clone(),
                    })
                    .await;

                // Update shared state directly
                self.shared_state.insert(
                    self.name.clone(),
                    super::super::web_query::MemberState::Failed { error: error_msg },
                );

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
                        debug!("{}: Stop requested during respawn delay", self.name);

                        self.clear_ros_clients();

                        self.state = NodeState::Stopped { exit_code: None };

                        // Broadcast stopped state
                        let _ = self.container_state_tx.send(ContainerState::Stopped);

                        let _ = self.state_tx.send(StateEvent::Terminated {
                            name: self.name.clone(),
                        }).await;

                        // Update shared state directly
                        self.shared_state
                            .insert(self.name.clone(), super::super::web_query::MemberState::Stopped);

                        Ok(true) // Keep actor alive to receive Start commands
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

                    // Update shared state directly
                    self.shared_state
                        .insert(self.name.clone(), super::super::web_query::MemberState::Stopped);

                    Ok(false) // Stop actor
                } else {
                    Ok(true)
                }
            }
        }
    }

    /// Handle stopped/failed state - wait for control events.
    ///
    /// Returns false if the actor should terminate (shutdown), true to continue.
    pub(super) async fn handle_stopped(&mut self) -> Result<bool> {
        // Wait for control event or shutdown signal
        tokio::select! {
            Some(event) = self.control_rx.recv() => {
                match event {
                    ControlEvent::Start => {
                        debug!("{}: Start requested", self.name);

                        // Transition to Pending to spawn container
                        match &self.state {
                            NodeState::Stopped { .. } | NodeState::Failed { .. } => {
                                self.state = NodeState::Pending;
                                debug!("{}: Transitioning to Pending for spawn", self.name);
                                Ok(true) // Continue to spawn
                            }
                            _ => {
                                // Should not happen, but handle gracefully
                                warn!("{}: Start requested in unexpected state", self.name);
                                Ok(true)
                            }
                        }
                    }
                    ControlEvent::Restart => {
                        debug!("{}: Restart requested", self.name);

                        // Transition to Pending to spawn container
                        self.state = NodeState::Pending;
                        Ok(true) // Continue to spawn
                    }
                    ControlEvent::Stop => {
                        // Already stopped, ignore
                        debug!("{}: Stop requested while already stopped", self.name);
                        Ok(true)
                    }
                    _ => {
                        warn!("{}: Unhandled control event in Stopped state: {:?}", self.name, event);
                        Ok(true)
                    }
                }
            }

            _ = self.shutdown_rx.changed() => {
                if *self.shutdown_rx.borrow() {
                    debug!("{}: Shutdown signal received in Stopped/Failed state", self.name);
                    Ok(false) // Terminate
                } else {
                    Ok(true) // Continue
                }
            }
        }
    }
}
