//! Composable node actor with container supervision
//!
//! ComposableNodeActor manages the lifecycle of a composable node that is loaded
//! into a container. It watches the parent container's state and automatically
//! loads/unloads based on container availability.
//!
//! ## Loading Strategy
//!
//! - **Single attempt**: No retry logic - if loading fails, the actor transitions to Blocked state
//! - **Container readiness**: Waits for container to reach Running state before loading
//! - **Concurrent loading**: Multiple actors can call load_node() concurrently, but all requests
//!   are serialized in the component loader's single blocking thread. This prevents resource
//!   depletion and ensures the container isn't overwhelmed.
//! - **One-by-one loading**: ROS2's LoadNode service only supports loading one node at a time.
//!   There's no bulk API - each node requires a separate service call.
//!
//! Phase 5: Provides both standalone function (run_composable_node) and trait-based actor (ComposableNodeActor).

use super::{
    actor_traits::MemberActor,
    container_control::{ContainerControlEvent, LoadNodeResponse},
    events::{ControlEvent, StateEvent},
    state::{BlockReason, ComposableState, ContainerState},
};
use crate::execution::context::ComposableNodeContext;
use eyre::{Context as _, Result};
use std::time::{Duration, Instant};
use tokio::sync::{mpsc, oneshot, watch};
use tracing::{debug, error, info, warn};

/// Standalone async function for running a composable node (Phase 5)
///
/// This is the standalone function version that will be used with FuturesUnordered.
/// It contains the same logic as ComposableNodeActor::run() but without the trait.
#[allow(clippy::too_many_arguments)]
pub async fn run_composable_node(
    name: String,
    context: ComposableNodeContext,
    config: ComposableActorConfig,
    control_rx: mpsc::Receiver<ControlEvent>,
    state_tx: mpsc::Sender<StateEvent>,
    container_rx: watch::Receiver<ContainerState>,
    shutdown_rx: watch::Receiver<bool>,
    load_control_tx: mpsc::Sender<ContainerControlEvent>,
) -> Result<()> {
    // Create the actor and run it (wrapper approach for Phase 5)
    let actor = ComposableNodeActor::new(
        name,
        context,
        config,
        control_rx,
        state_tx,
        container_rx,
        shutdown_rx,
        load_control_tx,
    );
    actor.run().await
}

/// Configuration for composable node actor
#[derive(Debug, Clone)]
pub struct ComposableActorConfig {
    /// Timeout for LoadNode service call (None = wait indefinitely)
    /// Some nodes (e.g., loading TensorRT models) may take 10+ minutes on Orin
    pub load_timeout: Option<Duration>,
    /// Automatically load when container starts (default: true)
    pub auto_load: bool,
    /// Timeout before triggering ListNodes verification (in seconds)
    /// If loading takes longer than this, we'll query the container to check if node is already loaded
    pub list_nodes_loading_timeout_secs: u64,
}

impl Default for ComposableActorConfig {
    fn default() -> Self {
        Self {
            load_timeout: Some(Duration::from_secs(30)),
            auto_load: true,
            list_nodes_loading_timeout_secs: 30,
        }
    }
}

/// Composable node actor
pub struct ComposableNodeActor {
    /// Unique name for this actor
    name: String,
    /// Composable node execution context
    context: ComposableNodeContext,
    /// Actor configuration
    config: ComposableActorConfig,
    /// Current state
    state: ComposableState,
    /// Channel to receive control events
    control_rx: mpsc::Receiver<ControlEvent>,
    /// Channel to send state events
    state_tx: mpsc::Sender<StateEvent>,
    /// Container state receiver (from parent container)
    container_rx: watch::Receiver<ContainerState>,
    /// Shutdown signal receiver
    shutdown_rx: watch::Receiver<bool>,
    /// Channel to send LoadNode requests to container actor
    load_control_tx: mpsc::Sender<ContainerControlEvent>,
}

impl ComposableNodeActor {
    /// Create a new composable node actor
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        name: String,
        context: ComposableNodeContext,
        config: ComposableActorConfig,
        control_rx: mpsc::Receiver<ControlEvent>,
        state_tx: mpsc::Sender<StateEvent>,
        container_rx: watch::Receiver<ContainerState>,
        shutdown_rx: watch::Receiver<bool>,
        load_control_tx: mpsc::Sender<ContainerControlEvent>,
    ) -> Self {
        Self {
            name,
            context,
            config,
            state: ComposableState::Unloaded,
            control_rx,
            state_tx,
            container_rx,
            shutdown_rx,
            load_control_tx,
        }
    }

    /// Send LoadNode request to container actor and return response channel
    /// Used by handle_loading() to initiate load without blocking
    async fn send_load_node_request(
        &mut self,
    ) -> Result<oneshot::Receiver<Result<LoadNodeResponse>>> {
        let record = &self.context.record;

        debug!(
            "{}: Sending LoadNode request to container {}",
            self.name, record.target_container_name
        );

        // Convert parameters to (name, value) tuples
        let parameters: Vec<(String, String)> = record
            .params
            .iter()
            .map(|(name, value)| (name.clone(), value.clone()))
            .collect();

        // Convert remappings to strings
        let remap_rules: Vec<String> = record
            .remaps
            .iter()
            .map(|(from, to)| format!("{}:={}", from, to))
            .collect();

        // Create response channel
        let (response_tx, response_rx) = oneshot::channel();

        // Send LoadNode request to container actor
        let load_event = ContainerControlEvent::LoadNode {
            composable_name: self.name.clone(),
            package: record.package.clone(),
            plugin: record.plugin.clone(),
            node_name: record.node_name.clone(),
            node_namespace: record.namespace.clone(),
            remap_rules,
            parameters,
            extra_args: vec![],
            response_tx,
        };

        self.load_control_tx
            .send(load_event)
            .await
            .context("Failed to send LoadNode request to container")?;

        Ok(response_rx)
    }

    /// Attempt to load the composable node via container actor
    /// Returns Ok(None) if shutdown occurred or timeout
    async fn load_node(&mut self) -> Result<Option<LoadNodeResponse>> {
        let record = &self.context.record;

        debug!(
            "{}: Sending LoadNode request to container {}",
            self.name, record.target_container_name
        );

        // Convert parameters to (name, value) tuples
        let parameters: Vec<(String, String)> = record
            .params
            .iter()
            .map(|(name, value)| (name.clone(), value.clone()))
            .collect();

        // Convert remappings to strings
        let remap_rules: Vec<String> = record
            .remaps
            .iter()
            .map(|(from, to)| format!("{}:={}", from, to))
            .collect();

        // Create response channel
        let (response_tx, response_rx) = oneshot::channel();

        // Send LoadNode request to container actor
        let load_event = ContainerControlEvent::LoadNode {
            composable_name: self.name.clone(),
            package: record.package.clone(),
            plugin: record.plugin.clone(),
            node_name: record.node_name.clone(),
            node_namespace: record.namespace.clone(),
            remap_rules,
            parameters,
            extra_args: vec![],
            response_tx,
        };

        self.load_control_tx
            .send(load_event)
            .await
            .context("Failed to send LoadNode request to container")?;

        // Wait for response with optional timeout
        let response_future = async {
            response_rx
                .await
                .context("Container actor dropped response channel")?
        };

        let result = if let Some(timeout_duration) = self.config.load_timeout {
            // Wait with timeout
            tokio::select! {
                result = response_future => result,
                _ = tokio::time::sleep(timeout_duration) => {
                    let error_msg = format!(
                        "LoadNode request timed out after {:?}",
                        timeout_duration
                    );
                    warn!("{}: {}", self.name, error_msg);
                    return Err(eyre::eyre!(error_msg));
                }
                _ = self.container_rx.changed() => {
                    let state = *self.container_rx.borrow();
                    if !state.is_running() {
                        debug!("{}: Container stopped during LoadNode wait", self.name);
                        return Err(eyre::eyre!("Container stopped during LoadNode"));
                    }
                    // Spurious wake or state unchanged, return to main loop
                    return Ok(None);
                }
                _ = self.shutdown_rx.changed() => {
                    if *self.shutdown_rx.borrow() {
                        debug!("{}: Shutdown during LoadNode wait", self.name);
                        return Ok(None);
                    }
                    // Spurious wake, return to main loop
                    return Ok(None);
                }
            }
        } else {
            // Wait indefinitely
            tokio::select! {
                result = response_future => result,
                _ = self.container_rx.changed() => {
                    let state = *self.container_rx.borrow();
                    if !state.is_running() {
                        debug!("{}: Container stopped during LoadNode wait", self.name);
                        return Err(eyre::eyre!("Container stopped during LoadNode"));
                    }
                    // Spurious wake or state unchanged, return to main loop
                    return Ok(None);
                }
                _ = self.shutdown_rx.changed() => {
                    if *self.shutdown_rx.borrow() {
                        debug!("{}: Shutdown during LoadNode wait", self.name);
                        return Ok(None);
                    }
                    // Spurious wake, return to main loop
                    return Ok(None)
                }
            }
        };

        // Process the response
        match result {
            Ok(response) => {
                if response.success {
                    debug!(
                        "{}: Successfully loaded with unique_id {}",
                        self.name, response.unique_id
                    );
                } else {
                    warn!(
                        "{}: LoadNode returned error: {}",
                        self.name, response.error_message
                    );
                }
                Ok(Some(response))
            }
            Err(e) => {
                warn!("{}: LoadNode request failed: {:#}", self.name, e);
                Err(e)
            }
        }
    }

    /// Write load timing metrics to CSV file
    fn write_load_timing_metrics(
        &self,
        timing: &super::container_control::LoadTimingMetrics,
    ) -> Result<()> {
        use std::{fs::OpenOptions, io::Write};

        let output_dir = &self.context.output_dir;
        let timing_file = output_dir.join("load_timing.csv");

        // Check if file exists to determine if we need to write headers
        let write_headers = !timing_file.exists();

        let mut file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&timing_file)?;

        if write_headers {
            writeln!(file, "queue_wait_ms,service_call_ms,total_duration_ms")?;
        }

        writeln!(
            file,
            "{},{},{}",
            timing.queue_wait_ms, timing.service_call_ms, timing.total_duration_ms
        )?;

        debug!(
            "{}: Wrote load timing metrics to {}",
            self.name,
            timing_file.display()
        );

        Ok(())
    }

    /// Attempt to unload the composable node via container actor
    async fn unload_node(
        &mut self,
        unique_id: u64,
    ) -> Result<super::container_control::UnloadNodeResponse> {
        debug!(
            "{}: Sending UnloadNode request to container (unique_id: {})",
            self.name, unique_id
        );

        // Create response channel
        let (response_tx, response_rx) = oneshot::channel();

        // Send UnloadNode request to container actor
        let unload_event = super::container_control::ContainerControlEvent::UnloadNode {
            composable_name: self.name.clone(),
            unique_id,
            response_tx,
        };

        self.load_control_tx
            .send(unload_event)
            .await
            .context("Failed to send UnloadNode request to container")?;

        // Wait for response
        match response_rx.await {
            Ok(Ok(response)) => {
                if response.success {
                    info!("{}: UnloadNode succeeded", self.name);
                } else {
                    warn!(
                        "{}: UnloadNode returned error: {}",
                        self.name, response.error_message
                    );
                }
                Ok(response)
            }
            Ok(Err(e)) => {
                warn!("{}: UnloadNode request failed: {:#}", self.name, e);
                Err(e)
            }
            Err(e) => {
                warn!("{}: UnloadNode response channel closed: {:#}", self.name, e);
                Err(eyre::eyre!("UnloadNode response channel closed: {:#}", e))
            }
        }
    }

    /// Handle the Unloaded state
    async fn handle_unloaded(&mut self) -> Result<bool> {
        debug!("{}: In Unloaded state, waiting for container", self.name);

        loop {
            // Check shutdown before processing
            if *self.shutdown_rx.borrow() {
                debug!("{}: Shutdown in Unloaded state", self.name);
                return Ok(false); // Stop actor
            }

            // Check current container state
            let container_state = *self.container_rx.borrow();

            match container_state {
                ContainerState::Running { .. } => {
                    // Check if auto-load is enabled
                    if self.config.auto_load {
                        debug!(
                            "{}: Container is running and auto-load enabled, transitioning to Loading",
                            self.name
                        );
                        self.state = ComposableState::Loading {
                            started_at: std::time::Instant::now(),
                        };
                        return Ok(true); // Continue to loading
                    } else {
                        debug!(
                            "{}: Container is running but auto-load disabled, staying in Unloaded",
                            self.name
                        );
                        // Stay in Unloaded state, wait for manual Load command or state changes
                        tokio::select! {
                            Some(event) = self.control_rx.recv() => {
                                match event {
                                    ControlEvent::Load => {
                                        debug!("{}: Received Load command in Unloaded state", self.name);
                                        self.state = ComposableState::Loading {
                                            started_at: std::time::Instant::now(),
                                        };
                                        return Ok(true); // Continue to loading
                                    }
                                    _ => {
                                        warn!("{}: Unhandled control event in Unloaded: {:?}", self.name, event);
                                    }
                                }
                            }
                            _ = self.container_rx.changed() => {
                                continue; // Check container state again
                            }
                            _ = self.shutdown_rx.changed() => {
                                if *self.shutdown_rx.borrow() {
                                    debug!("{}: Shutdown during Unloaded", self.name);
                                    return Ok(false); // Stop actor
                                }
                            }
                        }
                    }
                }
                ContainerState::Stopped => {
                    debug!("{}: Container stopped before loading", self.name);
                    self.state = ComposableState::Blocked {
                        reason: BlockReason::Stopped,
                    };
                    let _ = self
                        .state_tx
                        .send(StateEvent::Blocked {
                            name: self.name.clone(),
                            reason: BlockReason::Stopped,
                        })
                        .await;
                    return Ok(false); // Stop actor
                }
                ContainerState::Failed => {
                    debug!("{}: Container failed before loading", self.name);
                    self.state = ComposableState::Blocked {
                        reason: BlockReason::Failed,
                    };
                    let _ = self
                        .state_tx
                        .send(StateEvent::Blocked {
                            name: self.name.clone(),
                            reason: BlockReason::Failed,
                        })
                        .await;
                    return Ok(false); // Stop actor
                }
                ContainerState::Pending => {
                    // Wait for container state to change
                    tokio::select! {
                        _ = self.container_rx.changed() => {
                            continue; // Check new state in next iteration
                        }
                        _ = self.shutdown_rx.changed() => {
                            if *self.shutdown_rx.borrow() {
                                debug!("{}: Shutdown during Unloaded", self.name);
                                return Ok(false); // Stop actor
                            }
                        }
                    }
                }
            }
        }
    }

    /// Handle the Loading state
    /// Waits for container to be ready, then attempts to load once (no retries)
    async fn handle_loading(&mut self) -> Result<bool> {
        debug!("{}: In Loading state", self.name);

        // Check shutdown before starting
        if *self.shutdown_rx.borrow() {
            debug!("{}: Shutdown before loading started", self.name);
            return Ok(false); // Stop actor
        }

        // Check container state before attempting to load
        let container_state = *self.container_rx.borrow();
        if !container_state.is_ready() {
            warn!(
                "{}: Container not ready during loading (state: {:?})",
                self.name, container_state
            );
            self.state = ComposableState::Blocked {
                reason: BlockReason::Stopped,
            };
            return Ok(true); // Keep actor alive to detect container restart
        }

        let _ = self
            .state_tx
            .send(StateEvent::LoadStarted {
                name: self.name.clone(),
            })
            .await;

        // Get the started_at timestamp from state
        let started_at = if let ComposableState::Loading { started_at } = &self.state {
            *started_at
        } else {
            // Should not happen, but use current time as fallback
            Instant::now()
        };

        // Send LoadNode request and get response channel
        let mut response_rx = self.send_load_node_request().await?;

        // Track whether we've already sent a ListNodes verification request
        let mut list_nodes_requested = false;

        // Wait for response, with timeout-based ListNodes verification
        loop {
            let list_nodes_timeout =
                Duration::from_secs(self.config.list_nodes_loading_timeout_secs);
            let elapsed = started_at.elapsed();
            let remaining_until_verification = list_nodes_timeout.saturating_sub(elapsed);

            tokio::select! {
                // Wait for LoadNode response
                result = &mut response_rx => {
                    // Unwrap the channel recv result
                    let response_result = match result {
                        Ok(r) => r,
                        Err(e) => {
                            error!("{}: LoadNode response channel closed: {:#}", self.name, e);
                            self.state = ComposableState::Blocked {
                                reason: BlockReason::Failed,
                            };
                            let _ = self
                                .state_tx
                                .send(StateEvent::LoadFailed {
                                    name: self.name.clone(),
                                    error: format!("Response channel closed: {:#}", e),
                                })
                                .await;
                            return Ok(true); // Keep actor alive
                        }
                    };

                    // Unwrap the container actor result
                    match response_result {
                        Ok(response) if response.success => {
                            debug!("{}: LoadNode succeeded", self.name);
                            self.state = ComposableState::Loaded {
                                unique_id: response.unique_id,
                            };

                            // Write load timing metrics to file
                            if let Err(e) = self.write_load_timing_metrics(&response.timing) {
                                warn!(
                                    "{}: Failed to write load timing metrics: {:#}",
                                    self.name, e
                                );
                            }

                            let _ = self
                                .state_tx
                                .send(StateEvent::LoadSucceeded {
                                    name: self.name.clone(),
                                    full_node_name: response.full_node_name.clone(),
                                    unique_id: response.unique_id,
                                })
                                .await;

                            return Ok(true); // Continue to loaded state
                        }
                        Ok(response) => {
                            // Load failed
                            error!("{}: LoadNode failed: {}", self.name, response.error_message);

                            self.state = ComposableState::Blocked {
                                reason: BlockReason::Failed,
                            };

                            let _ = self
                                .state_tx
                                .send(StateEvent::LoadFailed {
                                    name: self.name.clone(),
                                    error: response.error_message.clone(),
                                })
                                .await;

                            return Ok(true); // Keep actor alive
                        }
                        Err(e) => {
                            // Container actor error (not channel closed)
                            error!("{}: LoadNode container error: {:#}", self.name, e);

                            self.state = ComposableState::Blocked {
                                reason: BlockReason::Failed,
                            };

                            let _ = self
                                .state_tx
                                .send(StateEvent::LoadFailed {
                                    name: self.name.clone(),
                                    error: format!("Container error: {:#}", e),
                                })
                                .await;

                            return Ok(true); // Keep actor alive
                        }
                    }
                }

                // Timeout for ListNodes verification
                _ = tokio::time::sleep(remaining_until_verification), if !list_nodes_requested => {
                    debug!(
                        "{}: Loading timeout reached ({}s), triggering ListNodes verification",
                        self.name, self.config.list_nodes_loading_timeout_secs
                    );

                    // Emit ListNodesRequested event
                    let _ = self
                        .state_tx
                        .send(StateEvent::ListNodesRequested {
                            container_name: self.context.record.target_container_name.clone(),
                            requester: self.name.clone(),
                        })
                        .await;

                    list_nodes_requested = true;
                    // Continue waiting for response or DiscoveredLoaded
                }

                // Handle control events
                Some(event) = self.control_rx.recv() => {
                    match event {
                        ControlEvent::DiscoveredLoaded { unique_id } => {
                            info!(
                                "{}: Discovered as already loaded (unique_id: {})",
                                self.name, unique_id
                            );

                            // Transition to Loaded state
                            self.state = ComposableState::Loaded { unique_id };

                            // Construct full node name from namespace and node_name
                            let full_node_name = if self.context.record.namespace == "/" {
                                format!("/{}", self.context.record.node_name)
                            } else if self.context.record.namespace.ends_with('/') {
                                format!("{}{}", self.context.record.namespace, self.context.record.node_name)
                            } else {
                                format!("{}/{}", self.context.record.namespace, self.context.record.node_name)
                            };

                            let _ = self
                                .state_tx
                                .send(StateEvent::LoadSucceeded {
                                    name: self.name.clone(),
                                    full_node_name,
                                    unique_id,
                                })
                                .await;

                            return Ok(true); // Continue to loaded state
                        }
                        ControlEvent::Stop => {
                            info!("{}: Stop requested during loading", self.name);
                            self.state = ComposableState::Blocked {
                                reason: BlockReason::Stopped,
                            };
                            let _ = self.state_tx.send(StateEvent::Blocked {
                                name: self.name.clone(),
                                reason: BlockReason::Stopped,
                            }).await;
                            return Ok(true); // Keep actor alive
                        }
                        _ => {
                            debug!("{}: Ignoring control event during loading: {:?}", self.name, event);
                        }
                    }
                }

                // Monitor container state changes
                _ = self.container_rx.changed() => {
                    let state = *self.container_rx.borrow();
                    if !state.is_running() {
                        warn!("{}: Container stopped during loading", self.name);
                        self.state = ComposableState::Blocked {
                            reason: BlockReason::Stopped,
                        };
                        let _ = self.state_tx.send(StateEvent::Blocked {
                            name: self.name.clone(),
                            reason: BlockReason::Stopped,
                        }).await;
                        return Ok(true); // Keep actor alive
                    }
                }

                // Check for shutdown signal
                _ = self.shutdown_rx.changed() => {
                    if *self.shutdown_rx.borrow() {
                        debug!("{}: Shutdown during loading", self.name);
                        return Ok(false); // Stop actor
                    }
                }
            }
        }
    }

    /// Handle the Loaded state
    async fn handle_loaded(&mut self, unique_id: u64) -> Result<bool> {
        debug!(
            "{}: In Loaded state (unique_id: {}), monitoring container",
            self.name, unique_id
        );

        loop {
            tokio::select! {
                // Monitor container state changes
                _ = self.container_rx.changed() => {
                    let container_state = *self.container_rx.borrow();
                    match container_state {
                        ContainerState::Running { .. } => {
                            // Container still running, continue monitoring
                            continue;
                        }
                        ContainerState::Stopped => {
                            info!("{}: Container stopped, transitioning to Blocked", self.name);
                            self.state = ComposableState::Blocked {
                                reason: BlockReason::Stopped,
                            };
                            let _ = self.state_tx.send(StateEvent::Blocked {
                                name: self.name.clone(),
                                reason: BlockReason::Stopped,
                            }).await;
                            return Ok(true); // Keep actor alive to detect container restart
                        }
                        ContainerState::Failed => {
                            info!("{}: Container failed, transitioning to Blocked", self.name);
                            self.state = ComposableState::Blocked {
                                reason: BlockReason::Failed,
                            };
                            let _ = self.state_tx.send(StateEvent::Blocked {
                                name: self.name.clone(),
                                reason: BlockReason::Failed,
                            }).await;
                            return Ok(true); // Keep actor alive to detect container restart
                        }
                        ContainerState::Pending => {
                            warn!("{}: Container unexpectedly transitioned to Pending", self.name);
                            self.state = ComposableState::Blocked {
                                reason: BlockReason::NotStarted,
                            };
                            let _ = self.state_tx.send(StateEvent::Blocked {
                                name: self.name.clone(),
                                reason: BlockReason::NotStarted,
                            }).await;
                            return Ok(true); // Keep actor alive to detect container restart
                        }
                    }
                }

                // Handle control events
                Some(event) = self.control_rx.recv() => {
                    match event {
                        ControlEvent::Stop => {
                            info!("{}: Received Stop command", self.name);
                            self.state = ComposableState::Blocked {
                                reason: BlockReason::Stopped,
                            };
                            let _ = self.state_tx.send(StateEvent::Blocked {
                                name: self.name.clone(),
                                reason: BlockReason::Stopped,
                            }).await;
                            return Ok(true); // Keep actor alive to detect container restart
                        }
                        ControlEvent::Load => {
                            info!("{}: Received Load command, transitioning to Unloaded", self.name);
                            self.state = ComposableState::Unloaded;
                            return Ok(true); // Continue to trigger loading
                        }
                        ControlEvent::ToggleAutoLoad(enabled) => {
                            info!("{}: Toggling auto-load: {}", self.name, enabled);
                            self.config.auto_load = enabled;
                            return Ok(true); // Continue
                        }
                        ControlEvent::Unload => {
                            info!("{}: Received Unload command, unloading node", self.name);
                            // Call unload_node method
                            match self.unload_node(unique_id).await {
                                Ok(response) => {
                                    if response.success {
                                        info!("{}: Successfully unloaded", self.name);
                                        // Transition to Unloaded state
                                        self.state = ComposableState::Unloaded;
                                        return Ok(true); // Continue
                                    } else {
                                        error!("{}: Unload failed: {}", self.name, response.error_message);
                                        self.state = ComposableState::Failed {
                                            error: response.error_message.clone(),
                                        };
                                        let _ = self.state_tx.send(StateEvent::LoadFailed {
                                            name: self.name.clone(),
                                            error: response.error_message,
                                        }).await;
                                        return Ok(true); // Keep actor alive
                                    }
                                }
                                Err(e) => {
                                    error!("{}: Unload error: {:#}", self.name, e);
                                    let error_msg = format!("{:#}", e);
                                    self.state = ComposableState::Failed {
                                        error: error_msg.clone(),
                                    };
                                    let _ = self.state_tx.send(StateEvent::LoadFailed {
                                        name: self.name.clone(),
                                        error: error_msg,
                                    }).await;
                                    return Ok(true); // Keep actor alive
                                }
                            }
                        }
                        _ => {
                            warn!("{}: Unhandled control event: {:?}", self.name, event);
                        }
                    }
                }

                // Check for shutdown signal
                _ = self.shutdown_rx.changed() => {
                    if *self.shutdown_rx.borrow() {
                        debug!("{}: Shutdown signal received", self.name);
                        return Ok(false); // Stop actor
                    }
                }
            }
        }
    }
}

impl MemberActor for ComposableNodeActor {
    fn name(&self) -> &str {
        &self.name
    }

    async fn run(mut self) -> Result<()> {
        debug!("{}: Composable node actor started", self.name);

        loop {
            let should_continue = match self.state {
                ComposableState::Unloaded => self.handle_unloaded().await?,
                ComposableState::Loading { .. } => self.handle_loading().await?,
                ComposableState::Loaded { unique_id } => self.handle_loaded(unique_id).await?,
                ComposableState::Failed { ref error } => {
                    debug!(
                        "{}: Failed (error: {}), waiting for container event or shutdown",
                        self.name, error
                    );
                    // Wait for container restart or shutdown
                    // In Failed state, we don't auto-retry - stay blocked until manual intervention
                    tokio::select! {
                        _ = self.container_rx.changed() => {
                            let state = *self.container_rx.borrow();
                            if state.is_running() {
                                // Container restarted - transition to Unloaded for fresh start
                                debug!("{}: Container restarted after failure, transitioning to Unloaded", self.name);
                                self.state = ComposableState::Unloaded;
                                true // Continue
                            } else {
                                true // Keep waiting
                            }
                        }
                        _ = self.shutdown_rx.changed() => {
                            if *self.shutdown_rx.borrow() {
                                debug!("{}: Shutdown signal in Failed state", self.name);
                                break;
                            }
                            true
                        }
                    }
                }
                ComposableState::Blocked { reason } => {
                    debug!(
                        "{}: Blocked (reason: {:?}), checking container state",
                        self.name, reason
                    );

                    // Check current container state first before waiting for changes
                    // This handles the case where container is already running when we enter Blocked
                    let current_state = *self.container_rx.borrow();
                    if current_state.is_running() {
                        debug!(
                            "{}: Container is already running, transitioning to Unloaded",
                            self.name
                        );
                        self.state = ComposableState::Unloaded;
                        true // Continue to Unloaded state
                    } else {
                        // Wait for container restart or shutdown
                        tokio::select! {
                            _ = self.container_rx.changed() => {
                                let state = *self.container_rx.borrow();
                                if state.is_running() {
                                    // Container restarted - transition to Unloaded for fresh start
                                    debug!("{}: Container restarted after being blocked, transitioning to Unloaded", self.name);
                                    self.state = ComposableState::Unloaded;
                                    true // Continue
                                } else {
                                    true // Keep waiting
                                }
                            }
                            _ = self.shutdown_rx.changed() => {
                                if *self.shutdown_rx.borrow() {
                                    debug!("{}: Shutdown signal in Blocked state", self.name);
                                    break;
                                }
                                true
                            }
                        }
                    }
                }
            };

            if !should_continue {
                break;
            }
        }

        debug!("{}: Composable node actor finished", self.name);
        Ok(())
    }
}
