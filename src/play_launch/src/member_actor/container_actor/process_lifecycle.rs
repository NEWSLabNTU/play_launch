//! Process lifecycle management for container actors.
//!
//! Methods for spawning and stopping the container process, handling child
//! exit, shutdown, restart, and respawn. Also includes metadata updates and
//! ROS client cleanup.

use super::ContainerActor;
use crate::member_actor::{
    events::{ControlEvent, StateEvent},
    model::{BlockReason, ContainerState, NodeState},
};
use eyre::{Context as _, Result};
use tokio::{
    process::Command,
    time::{Duration, sleep},
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
            .supervisor
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
        metadata["composable_node_count"] =
            serde_json::json!(self.supervisor.composable_nodes.len());

        // Write back
        let updated_json =
            serde_json::to_string_pretty(&metadata).context("Failed to serialize metadata")?;
        fs::write(&metadata_path, updated_json).context("Failed to write metadata.json")?;

        debug!(
            "{}: Updated metadata.json with {} composable nodes",
            self.name,
            self.supervisor.composable_nodes.len()
        );

        Ok(())
    }

    /// Clear ROS service clients and event subscription so they're recreated on restart.
    ///
    /// Phase 64: the private control channel goes with them. It belongs to one
    /// container PROCESS — the socketpair's far end died with it — so a
    /// respawn negotiates a fresh one rather than writing into a closed fd.
    pub(super) fn clear_ros_clients(&mut self) {
        self.clients.clear();
        self.control = None;
    }

    /// Phase 64: wait for the container to introduce itself on the control
    /// channel, and decide what the channel is worth.
    ///
    /// Three outcomes, all normal: a container that speaks the protocol and
    /// takes loads over it (`isolated`); one that speaks it for status only
    /// (`observable`, where loading has to happen on the manager's executor
    /// thread); and one that never answers — an older wheel's binary, or a
    /// stock container that never had the fd — which drops the channel and
    /// leaves the LoadNode path exactly as it was.
    pub(super) async fn negotiate_control_channel(&mut self) {
        let timeout = self.supervisor.timings.control_hello_timeout;
        let Some(channel) = self.control.as_mut() else {
            return;
        };
        if channel.await_hello(timeout).await {
            if channel.loads_over_socket() {
                info!(
                    "{}: loading composables over the private control channel \
                     (no LoadNode service calls)",
                    self.name
                );
            } else {
                debug!(
                    "{}: control channel up for status; loads stay on the LoadNode service",
                    self.name
                );
            }
        } else {
            info!(
                "{}: container did not answer the control channel within {}s — \
                 using the LoadNode service",
                self.name,
                timeout.as_secs()
            );
            self.control = None;
        }
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
    pub(super) async fn spawn_process(&mut self) -> Result<tokio::process::Child> {
        let exec_context = self
            .context
            .to_exec_context(self.config.pgid)
            .context("Failed to create execution context")?;

        let mut command: Command = exec_context.command;

        debug!(
            "Spawning container process: {:?} (output_dir: {:?})",
            command, exec_context.output_dir
        );

        // Phase 64: hand the container one end of a private socketpair, so the
        // load path does not have to travel the rmw service layer that every
        // other process on the machine is contending for during startup.
        //
        // Only for OUR binary: `use_component_events` is exactly the
        // "container-mode is not stock" flag, and a stock container has no
        // idea what the fd is for. It would inherit it harmlessly, never
        // speak, and `negotiate_control_channel` would drop the channel — but
        // there is no reason to create one.
        let child_end = if self.use_component_events && self.supervisor.timings.control_socket {
            match super::control_channel::ControlChannel::new(&self.name) {
                Ok((channel, child_end)) => {
                    self.control = Some(channel);
                    Some(child_end)
                }
                Err(e) => {
                    warn!(
                        "{}: could not create the control channel ({:#}) — \
                         falling back to the LoadNode service",
                        self.name, e
                    );
                    None
                }
            }
        } else {
            None
        };

        if let Some(child_end) = child_end.as_ref() {
            let fd = child_end.raw();
            command.env(
                crate::ipc::container_protocol::CONTROL_FD_ENV,
                fd.to_string(),
            );
            // Clear FD_CLOEXEC in the CHILD, between fork and exec, rather
            // than in this process: clearing it here would leak this
            // container's control fd into every other node spawned while the
            // flag was down, and each leaked copy holds the socket open so the
            // supervisor would never see EOF when the container died.
            //
            // Async-signal-safe: one `fcntl`, no allocation.
            unsafe {
                command.pre_exec(move || {
                    if libc::fcntl(fd, libc::F_SETFD, 0) < 0 {
                        return Err(std::io::Error::last_os_error());
                    }
                    Ok(())
                });
            }
        }

        // Phase 61: containers queue for a startup slot like any other
        // process. They are not exempt — a container is one of the more
        // expensive things to start, and in `isolated` mode each one goes on
        // to fork a process per composable it hosts.
        let permit = self
            .config
            .startup
            .admit(&self.name, self.config.startup_stage)
            .await;

        let child = command
            .spawn()
            .context("Failed to spawn container process")?;

        // On the error path above, `permit` drops and the slot is returned
        // immediately, which is what a failed spawn deserves.
        if let Some(pid) = child.id() {
            permit.hold_until_settled(pid, self.name.clone());
        }

        // The fork has happened, so the container has its own copy: close
        // ours. Holding it would keep a write end of the container's own
        // socket open in this process, and EOF — how the supervisor learns the
        // container is gone — would never arrive.
        drop(child_end);

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
        self.unregister_process(pid);

        // Drain load queue (container crashed)
        self.supervisor.drain_queue("Container crashed");

        self.clear_ros_clients();

        // Phase 12: Transition composable nodes to Blocked
        self.supervisor
            .transition_all_composables_to_blocked(BlockReason::ContainerFailed)
            .await;

        // Broadcast stopped state to composable nodes
        let _ = self.container_state_tx.send(ContainerState::Stopped);

        // Send state event
        crate::member_actor::events::emit(
            &self.state_tx,
            StateEvent::Exited {
                name: self.name.clone(),
                exit_code,
            },
        )
        .await;

        // Check if respawn is enabled
        if self.config.respawn_enabled {
            self.state = NodeState::Respawning {
                exit_code,
                attempt: 0,
            };
        } else {
            self.state = NodeState::Stopped { exit_code };
            crate::member_actor::events::emit(
                &self.state_tx,
                StateEvent::Terminated {
                    name: self.name.clone(),
                },
            )
            .await;
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
        self.unregister_process(pid);

        // Drain load queue
        self.supervisor.drain_queue("Container stopped");

        self.clear_ros_clients();

        // Phase 12: Transition composable nodes to Blocked
        debug!(
            "{}: Transitioning {} composable nodes to Blocked state (reason: Stopped)",
            self.name,
            self.supervisor.composable_nodes.len()
        );
        self.supervisor
            .transition_all_composables_to_blocked(BlockReason::ContainerStopped)
            .await;

        // Broadcast stopped state
        let _ = self.container_state_tx.send(ContainerState::Stopped);

        self.state = NodeState::Stopped { exit_code: None };
        crate::member_actor::events::emit(
            &self.state_tx,
            StateEvent::Terminated {
                name: self.name.clone(),
            },
        )
        .await;

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
        self.unregister_process(pid);

        // Drain load queue
        self.supervisor.drain_queue("Container restarting");

        self.clear_ros_clients();

        // Phase 12: Transition composable nodes to Blocked (will be reloaded on restart)
        self.supervisor
            .transition_all_composables_to_blocked(BlockReason::ContainerStopped)
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
        self.supervisor.drain_queue("Shutdown");

        // Phase 12: Transition composable nodes to Blocked
        self.supervisor
            .transition_all_composables_to_blocked(BlockReason::Shutdown)
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
        self.unregister_process(pid);

        // Broadcast stopped state
        let _ = self.container_state_tx.send(ContainerState::Stopped);

        self.state = NodeState::Stopped { exit_code: None };
        crate::member_actor::events::emit(
            &self.state_tx,
            StateEvent::Terminated {
                name: self.name.clone(),
            },
        )
        .await;

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
        crate::member_actor::events::emit(
            &self.state_tx,
            StateEvent::Respawning {
                name: self.name.clone(),
                attempt,
                delay,
            },
        )
        .await;

        // Check if max attempts reached
        if let Some(max_attempts) = self.config.max_respawn_attempts
            && attempt >= max_attempts
        {
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

            crate::member_actor::events::emit(
                &self.state_tx,
                StateEvent::Failed {
                    name: self.name.clone(),
                    error: error_msg.clone(),
                },
            )
            .await;

            return Ok(false); // Stop actor
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

                        crate::member_actor::events::emit(&self.state_tx, StateEvent::Terminated {
                            name: self.name.clone(),
                        }).await;

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

                    crate::member_actor::events::emit(&self.state_tx, StateEvent::Terminated {
                        name: self.name.clone(),
                    }).await;

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
