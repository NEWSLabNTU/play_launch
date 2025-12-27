//! Composable node actor with container supervision
//!
//! ComposableNodeActor manages the lifecycle of a composable node that is loaded
//! into a container. It watches the parent container's state and automatically
//! loads/unloads based on container availability.
//!
//! Phase 5: Provides both standalone function (run_composable_node) and trait-based actor (ComposableNodeActor).

use super::{
    actor_traits::MemberActor,
    events::{ControlEvent, StateEvent},
    state::{BlockReason, ComposableState, ContainerState},
};
use crate::{
    execution::context::ComposableNodeContext,
    ros::component_loader::{ComponentLoaderHandle, LoadNodeResponse},
};
use eyre::{Context as _, Result};
use std::time::Duration;
use tokio::sync::{mpsc, watch};
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
    component_loader: ComponentLoaderHandle,
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
        component_loader,
    );
    actor.run().await
}

/// Configuration for composable node actor
#[derive(Debug, Clone)]
pub struct ComposableActorConfig {
    /// Timeout for LoadNode service call
    pub load_timeout: Duration,
    /// Maximum retry attempts for loading
    pub max_load_attempts: u32,
    /// Delay between retry attempts
    pub retry_delay: Duration,
}

impl Default for ComposableActorConfig {
    fn default() -> Self {
        Self {
            load_timeout: Duration::from_secs(30),
            max_load_attempts: 3,
            retry_delay: Duration::from_secs(2),
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
    /// Component loader handle
    component_loader: ComponentLoaderHandle,
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
        component_loader: ComponentLoaderHandle,
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
            component_loader,
        }
    }

    /// Attempt to load the composable node via LoadNode service
    /// Returns Ok(None) if shutdown occurred during the call
    async fn load_node(&mut self) -> Result<Option<LoadNodeResponse>> {
        let record = &self.context.record;

        debug!(
            "{}: Loading composable node {}/{} into container {}",
            self.name, record.package, record.plugin, record.target_container_name
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

        // Prepare service call future
        let service_call = self.component_loader.load_node(
            &record.target_container_name,
            &record.package,
            &record.plugin,
            &record.node_name,
            &record.namespace,
            remap_rules,
            parameters,
            vec![], // extra_args
            self.config.load_timeout,
        );

        // Race service call against shutdown signal
        tokio::select! {
            result = service_call => {
                let response = result.context("LoadNode service call failed")?;

                if response.success {
                    debug!(
                        "{}: Successfully loaded with unique_id {}",
                        self.name, response.unique_id
                    );
                } else {
                    warn!(
                        "{}: LoadNode service returned error: {}",
                        self.name, response.error_message
                    );
                }

                Ok(Some(response))
            }
            _ = self.shutdown_rx.changed() => {
                if *self.shutdown_rx.borrow() {
                    debug!("{}: Shutdown during LoadNode call", self.name);
                    Ok(None) // Signal shutdown occurred
                } else {
                    // Spurious wake, continue with original call
                    // This shouldn't happen often, but handle it gracefully
                    let response = self
                        .component_loader
                        .load_node(
                            &record.target_container_name,
                            &record.package,
                            &record.plugin,
                            &record.node_name,
                            &record.namespace,
                            record.remaps.iter().map(|(from, to)| format!("{}:={}", from, to)).collect(),
                            record.params.iter().map(|(name, value)| (name.clone(), value.clone())).collect(),
                            vec![],
                            self.config.load_timeout,
                        )
                        .await
                        .context("LoadNode service call failed")?;

                    Ok(Some(response))
                }
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
                    debug!(
                        "{}: Container is running, transitioning to Loading",
                        self.name
                    );
                    self.state = ComposableState::Loading;
                    return Ok(true); // Continue to loading
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
    async fn handle_loading(&mut self) -> Result<bool> {
        debug!("{}: In Loading state", self.name);

        // Check shutdown before starting
        if *self.shutdown_rx.borrow() {
            debug!("{}: Shutdown before loading started", self.name);
            return Ok(false); // Stop actor
        }

        let _ = self
            .state_tx
            .send(StateEvent::LoadStarted {
                name: self.name.clone(),
            })
            .await;

        // Retry loop
        for attempt in 0..self.config.max_load_attempts {
            // Check shutdown at start of each retry
            if *self.shutdown_rx.borrow() {
                debug!("{}: Shutdown during retry loop", self.name);
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
                return Ok(false); // Stop actor
            }

            // Attempt to load (cancellable on shutdown)
            let load_result = self.load_node().await;

            match load_result {
                Ok(None) => {
                    // Shutdown occurred during load
                    return Ok(false); // Stop actor
                }
                Ok(Some(response)) if response.success => {
                    debug!("{}: LoadNode succeeded", self.name);
                    self.state = ComposableState::Loaded {
                        unique_id: response.unique_id,
                    };

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
                Ok(Some(response)) => {
                    warn!(
                        "{}: LoadNode failed (attempt {}/{}): {}",
                        self.name,
                        attempt + 1,
                        self.config.max_load_attempts,
                        response.error_message
                    );

                    if attempt + 1 < self.config.max_load_attempts {
                        // Wait before retry (with shutdown check)
                        let delay = self.config.retry_delay;
                        tokio::select! {
                            _ = tokio::time::sleep(delay) => {},
                            _ = self.shutdown_rx.changed() => {
                                if *self.shutdown_rx.borrow() {
                                    debug!("{}: Shutdown during retry delay", self.name);
                                    return Ok(false); // Stop actor
                                }
                            }
                        }
                    }
                }
                Err(e) => {
                    error!(
                        "{}: LoadNode error (attempt {}/{}): {:#}",
                        self.name,
                        attempt + 1,
                        self.config.max_load_attempts,
                        e
                    );

                    if attempt + 1 < self.config.max_load_attempts {
                        // Wait before retry (with shutdown check)
                        let delay = self.config.retry_delay;
                        tokio::select! {
                            _ = tokio::time::sleep(delay) => {},
                            _ = self.shutdown_rx.changed() => {
                                if *self.shutdown_rx.borrow() {
                                    debug!("{}: Shutdown during retry delay", self.name);
                                    return Ok(false); // Stop actor
                                }
                            }
                        }
                    }
                }
            }
        }

        // All attempts failed
        error!(
            "{}: Failed to load after {} attempts",
            self.name, self.config.max_load_attempts
        );

        self.state = ComposableState::Blocked {
            reason: BlockReason::Failed,
        };

        let _ = self
            .state_tx
            .send(StateEvent::LoadFailed {
                name: self.name.clone(),
                error: format!(
                    "Failed to load after {} attempts",
                    self.config.max_load_attempts
                ),
            })
            .await;

        Ok(false) // Stop actor
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
                            return Ok(false); // Stop actor
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
                            return Ok(false); // Stop actor
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
                            return Ok(false); // Stop actor
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
                            return Ok(false); // Stop actor
                        }
                        _ => {
                            warn!("{}: Unhandled control event: {:?}", self.name, event);
                        }
                    }
                }

                // Check for shutdown signal
                _ = self.shutdown_rx.changed() => {
                    if *self.shutdown_rx.borrow() {
                        info!("{}: Shutdown signal received", self.name);
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
                ComposableState::Loading => self.handle_loading().await?,
                ComposableState::Loaded { unique_id } => self.handle_loaded(unique_id).await?,
                ComposableState::Blocked { reason } => {
                    debug!("{}: Blocked (reason: {:?})", self.name, reason);
                    break;
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
