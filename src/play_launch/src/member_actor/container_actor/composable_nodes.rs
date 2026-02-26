//! Composable node management for container actors.
//!
//! Methods for loading, unloading, and managing the lifecycle of composable
//! nodes within a container. Handles the LoadNode queue, dispatch of parallel
//! load requests, and state transitions for individual composable nodes.

use super::ContainerActor;
use crate::member_actor::{
    container_control::{ContainerControlEvent, LoadCompletion, LoadRequest},
    events::StateEvent,
    state::ComposableState,
};
use std::time::Instant;
use tracing::{debug, warn};

impl ContainerActor {
    /// Convert ROS parameters to string tuples.
    pub(super) fn ros_params_to_strings(
        params: &[rcl_interfaces::msg::Parameter],
    ) -> Vec<(String, String)> {
        params
            .iter()
            .map(|param| {
                let value_str = Self::parameter_value_to_string(&param.value);
                (param.name.clone(), value_str)
            })
            .collect()
    }

    /// Convert a ParameterValue to string representation.
    pub(super) fn parameter_value_to_string(value: &rcl_interfaces::msg::ParameterValue) -> String {
        use rcl_interfaces::msg::ParameterType;

        match value.type_ {
            ParameterType::PARAMETER_BOOL => value.bool_value.to_string(),
            ParameterType::PARAMETER_INTEGER => value.integer_value.to_string(),
            ParameterType::PARAMETER_DOUBLE => {
                // Format double to ensure it always has a decimal point
                // This prevents "1.0" from becoming "1" which would be interpreted as integer
                let s = value.double_value.to_string();
                if s.contains('.') || s.contains('e') || s.contains('E') {
                    s
                } else {
                    format!("{}.0", s)
                }
            }
            ParameterType::PARAMETER_STRING => value.string_value.clone(),
            ParameterType::PARAMETER_BYTE_ARRAY => format!("{:?}", value.byte_array_value),
            ParameterType::PARAMETER_BOOL_ARRAY => format!("{:?}", value.bool_array_value),
            ParameterType::PARAMETER_INTEGER_ARRAY => format!("{:?}", value.integer_array_value),
            ParameterType::PARAMETER_DOUBLE_ARRAY => format!("{:?}", value.double_array_value),
            ParameterType::PARAMETER_STRING_ARRAY => format!("{:?}", value.string_array_value),
            _ => String::new(),
        }
    }

    /// Dispatch ALL pending loads concurrently (no serialization gate).
    /// Each load is spawned as an independent tokio task that sends its
    /// result back via the load_completion channel.
    pub(super) fn dispatch_pending_loads(&mut self) {
        while let Some(request) = self.pending_loads.pop_front() {
            let start_time = std::time::Instant::now();
            let queue_wait_ms = start_time.duration_since(request.request_time).as_millis() as u64;

            debug!(
                "{}: Dispatching load for {} (queue wait: {}ms)",
                self.name, request.composable_name, queue_wait_ms
            );

            let tx = self.load_completion_tx.clone();
            let composable_name = request.composable_name.clone();
            let container_name = self.name.clone();
            let load_client = self.load_client.clone();
            let params = super::super::container_control::LoadParams {
                composable_name: request.composable_name,
                package: request.package,
                plugin: request.plugin,
                node_name: request.node_name,
                node_namespace: request.node_namespace,
                remap_rules: request.remap_rules,
                parameters: request.parameters,
                extra_args: request.extra_args,
                output_dir: request.output_dir,
                request_time: request.request_time,
            };

            tokio::spawn(async move {
                let result =
                    Self::call_load_node_service(container_name, load_client, params, start_time)
                        .await;
                let _ = tx.send(LoadCompletion {
                    composable_name,
                    result,
                });
            });
        }
    }

    /// Handle a completed LoadNode service call.
    pub(super) async fn handle_load_completion(&mut self, completion: LoadCompletion) {
        let composable_name = &completion.composable_name;
        let has_event_sub = self.component_event_sub.is_some();

        let Some(entry) = self.composable_nodes.get_mut(composable_name) else {
            warn!(
                "{}: Load completion for unknown composable '{}'",
                self.name, composable_name
            );
            return;
        };

        match completion.result {
            Ok(response) if response.success => {
                entry.unique_id = Some(response.unique_id);

                if !has_event_sub {
                    // Stock container fallback: service response = loaded
                    if matches!(entry.state, ComposableState::Loading { .. }) {
                        entry.state = ComposableState::Loaded {
                            unique_id: response.unique_id,
                        };
                        entry.load_started_at = None;

                        debug!(
                            "{}: Successfully loaded composable node '{}' (unique_id: {})",
                            self.name, composable_name, response.unique_id
                        );

                        self.shared_state.insert(
                            composable_name.clone(),
                            super::super::web_query::MemberState::Loaded {
                                unique_id: response.unique_id,
                            },
                        );

                        let _ = self
                            .state_tx
                            .send(StateEvent::LoadSucceeded {
                                name: composable_name.clone(),
                                full_node_name: response.full_node_name.clone(),
                                unique_id: response.unique_id,
                            })
                            .await;
                    }
                }
                // With event sub: state stays Loading, wait for ComponentEvent
            }
            Ok(response) => {
                // LoadNode returned failure
                if matches!(entry.state, ComposableState::Loading { .. }) {
                    entry.state = ComposableState::Failed {
                        error: response.error_message.clone(),
                    };
                    entry.load_started_at = None;

                    warn!(
                        "{}: Failed to load composable node '{}': {}",
                        self.name, composable_name, response.error_message
                    );

                    self.shared_state.insert(
                        composable_name.clone(),
                        super::super::web_query::MemberState::Failed {
                            error: response.error_message.clone(),
                        },
                    );

                    let _ = self
                        .state_tx
                        .send(StateEvent::LoadFailed {
                            name: composable_name.clone(),
                            error: response.error_message,
                        })
                        .await;
                }
            }
            Err(e) => {
                // Service call error/timeout
                if matches!(entry.state, ComposableState::Loading { .. }) {
                    let error_msg = format!("{:#}", e);
                    entry.state = ComposableState::Failed {
                        error: error_msg.clone(),
                    };
                    entry.load_started_at = None;

                    warn!(
                        "{}: LoadNode service call failed for '{}': {:#}",
                        self.name, composable_name, e
                    );

                    self.shared_state.insert(
                        composable_name.clone(),
                        super::super::web_query::MemberState::Failed {
                            error: error_msg.clone(),
                        },
                    );

                    let _ = self
                        .state_tx
                        .send(StateEvent::LoadFailed {
                            name: composable_name.clone(),
                            error: error_msg,
                        })
                        .await;
                }
            }
        }
    }

    /// Drain the load queue.
    ///
    /// Pending loads are dropped. In-flight spawned tasks will send completions
    /// to the channel, but handle_load_completion will see the composable node
    /// is Blocked and ignore them.
    pub(super) fn drain_queue(&mut self, error: &str) {
        // Cancel current unload
        if let Some(unload) = self.current_unload.take() {
            debug!(
                "{}: Cancelling current unload for {}: {}",
                self.name, unload.composable_name, error
            );
        }

        // Drop all queued loads (spawned tasks will complete harmlessly)
        let queue_len = self.pending_loads.len();
        if queue_len > 0 {
            debug!(
                "{}: Draining {} queued load requests: {}",
                self.name, queue_len, error
            );
        }
        self.pending_loads.clear();
    }

    /// Handle LoadComposable control event.
    pub(super) async fn handle_load_composable(&mut self, name: &str) {
        debug!("{}: Handling LoadComposable for '{}'", self.name, name);

        // Check if composable node exists
        let entry = match self.composable_nodes.get_mut(name) {
            Some(e) => e,
            None => {
                warn!("{}: Composable node '{}' not found", self.name, name);
                return;
            }
        };

        // Check current state
        match &entry.state {
            ComposableState::Unloaded | ComposableState::Failed { .. } => {
                // Transition to Loading and start the load operation
                let started_at = Instant::now();
                entry.state = ComposableState::Loading { started_at };
                entry.load_started_at = Some(started_at);

                debug!("{}: Transitioning '{}' to Loading state", self.name, name);

                // Emit LoadStarted event
                let _ = self
                    .state_tx
                    .send(StateEvent::LoadStarted {
                        name: name.to_string(),
                    })
                    .await;

                // Update shared state directly for composable node
                self.shared_state.insert(
                    name.to_string(),
                    super::super::web_query::MemberState::Loading,
                );

                // Phase 12: Queue LoadNode request and dispatch immediately
                if let Some(entry) = self.composable_nodes.get(name) {
                    // Convert ROS parameters to string format
                    let parameters = Self::ros_params_to_strings(&entry.metadata.parameters);
                    let extra_args = Self::ros_params_to_strings(&entry.metadata.extra_args);

                    let request = LoadRequest {
                        composable_name: name.to_string(),
                        package: entry.metadata.package.clone(),
                        plugin: entry.metadata.plugin.clone(),
                        node_name: entry.metadata.node_name.clone(),
                        node_namespace: entry.metadata.namespace.clone(),
                        remap_rules: entry.metadata.remap_rules.clone(),
                        parameters,
                        extra_args,
                        output_dir: entry.metadata.output_dir.clone(),
                        request_time: started_at,
                    };

                    self.pending_loads.push_back(request);
                    self.dispatch_pending_loads();
                }
            }
            ComposableState::Loading { .. } => {
                debug!("{}: '{}' already loading, ignoring", self.name, name);
            }
            ComposableState::Unloading { .. } => {
                debug!(
                    "{}: '{}' is unloading, ignoring load request",
                    self.name, name
                );
            }
            ComposableState::Loaded { .. } => {
                debug!("{}: '{}' already loaded, ignoring", self.name, name);
            }
            ComposableState::Blocked { .. } => {
                warn!("{}: Cannot load '{}' - container blocked", self.name, name);
            }
        }
    }

    /// Handle UnloadComposable control event.
    pub(super) async fn handle_unload_composable(&mut self, name: &str) {
        debug!("{}: Handling UnloadComposable for '{}'", self.name, name);

        // Debug: Log all composable node states
        debug!("{}: Current composable node states:", self.name);
        for (node_name, node_entry) in self.composable_nodes.iter() {
            debug!("  - {}: {:?}", node_name, node_entry.state);
        }

        // Check if composable node exists
        let entry = match self.composable_nodes.get_mut(name) {
            Some(e) => e,
            None => {
                warn!(
                    "{}: Composable node '{}' not found in composable_nodes HashMap",
                    self.name, name
                );
                warn!(
                    "{}: Available composable nodes: {:?}",
                    self.name,
                    self.composable_nodes.keys().collect::<Vec<_>>()
                );
                return;
            }
        };

        // Check current state - only unload if loaded
        match &entry.state {
            ComposableState::Loaded { unique_id } => {
                let unique_id = *unique_id;
                let started_at = Instant::now();

                debug!(
                    "{}: Unloading '{}' (unique_id: {})",
                    self.name, name, unique_id
                );

                // Transition to Unloading state
                entry.state = ComposableState::Unloading { started_at };
                entry.load_started_at = Some(started_at);

                // Update shared_state to Unloading for Web UI
                self.shared_state.insert(
                    name.to_string(),
                    super::super::web_query::MemberState::Unloading,
                );

                // Spawn UnloadNode service call as a tracked task
                let container_name = self.name.clone();
                let unload_client = self.unload_client.clone();

                let task = tokio::spawn(async move {
                    Self::call_unload_node_service(container_name, unload_client, unique_id).await
                });

                // Track the unload operation (will be polled in select! loop)
                self.current_unload = Some(super::super::container_control::CurrentUnload {
                    composable_name: name.to_string(),
                    start_time: started_at,
                    task,
                });
            }
            _ => {
                warn!(
                    "{}: Cannot unload '{}' - not in Loaded state (current: {:?})",
                    self.name, name, entry.state
                );
            }
        }
    }

    /// Handle LoadAllComposables control event.
    pub(super) async fn handle_load_all_composables(&mut self) {
        debug!("{}: Handling LoadAllComposables", self.name);

        // Log state of all composable nodes for debugging
        let total_nodes = self.composable_nodes.len();
        let state_summary: std::collections::HashMap<String, usize> = self
            .composable_nodes
            .values()
            .fold(std::collections::HashMap::new(), |mut acc, entry| {
                let state_name = match &entry.state {
                    ComposableState::Blocked { .. } => "Blocked",
                    ComposableState::Unloaded => "Unloaded",
                    ComposableState::Loading { .. } => "Loading",
                    ComposableState::Unloading { .. } => "Unloading",
                    ComposableState::Loaded { .. } => "Loaded",
                    ComposableState::Failed { .. } => "Failed",
                };
                *acc.entry(state_name.to_string()).or_insert(0) += 1;
                acc
            });

        debug!(
            "{}: Composable node states: {} total ({:?})",
            self.name, total_nodes, state_summary
        );

        // Collect names of nodes to load (to avoid borrowing issues)
        let nodes_to_load: Vec<String> = self
            .composable_nodes
            .iter()
            .filter(|(_, entry)| {
                entry.metadata.auto_load
                    && matches!(
                        entry.state,
                        ComposableState::Unloaded | ComposableState::Failed { .. }
                    )
            })
            .map(|(name, _)| name.clone())
            .collect();

        if nodes_to_load.is_empty() {
            debug!(
                "{}: No composable nodes to load (all {} nodes already loaded or not marked for auto_load)",
                self.name, total_nodes
            );
        } else {
            debug!(
                "{}: Queueing {} composable nodes for loading: {:?}",
                self.name,
                nodes_to_load.len(),
                nodes_to_load
            );
        }

        // Load each node
        for name in nodes_to_load {
            self.handle_load_composable(&name).await;
        }
    }

    /// Handle ToggleComposableAutoLoad control event.
    pub(super) async fn handle_toggle_composable_auto_load(&mut self, name: &str, enabled: bool) {
        debug!(
            "{}: Handling ToggleComposableAutoLoad for '{}' (enabled: {})",
            self.name, name, enabled
        );

        // Find and update the composable node's auto_load setting
        if let Some(entry) = self.composable_nodes.get_mut(name) {
            entry.metadata.auto_load = enabled;
            debug!(
                "{}: Updated auto_load for '{}' to {}",
                self.name, name, enabled
            );
        } else {
            warn!(
                "{}: Cannot toggle auto_load for '{}': composable node not found",
                self.name, name
            );
        }
    }

    /// Handle UnloadAllComposables control event.
    pub(super) async fn handle_unload_all_composables(&mut self) {
        debug!("{}: Handling UnloadAllComposables", self.name);

        // Collect names of loaded nodes
        let nodes_to_unload: Vec<String> = self
            .composable_nodes
            .iter()
            .filter(|(_, entry)| matches!(entry.state, ComposableState::Loaded { .. }))
            .map(|(name, _)| name.clone())
            .collect();

        debug!(
            "{}: Unloading {} loaded composable nodes",
            self.name,
            nodes_to_unload.len()
        );

        // Unload each node
        for name in nodes_to_unload {
            self.handle_unload_composable(&name).await;
        }
    }

    /// Handle a load control event (LoadNode / UnloadNode from the control channel).
    pub(super) async fn handle_load_control_event(&mut self, event: ContainerControlEvent) {
        match event {
            ContainerControlEvent::LoadNode {
                composable_name,
                package,
                plugin,
                node_name,
                node_namespace,
                remap_rules,
                parameters,
                extra_args,
                output_dir,
            } => {
                debug!(
                    "{}: Received load request for {}",
                    self.name, composable_name
                );

                let request = LoadRequest {
                    composable_name,
                    package,
                    plugin,
                    node_name,
                    node_namespace,
                    remap_rules,
                    parameters,
                    extra_args,
                    output_dir,
                    request_time: std::time::Instant::now(),
                };

                self.pending_loads.push_back(request);
                self.dispatch_pending_loads();
            }
            ContainerControlEvent::UnloadNode {
                composable_name,
                unique_id,
                response_tx,
            } => {
                debug!(
                    "{}: Received unload request for {} (unique_id: {})",
                    self.name, composable_name, unique_id
                );
                self.handle_unload_request(composable_name, unique_id, response_tx)
                    .await;
            }
        }
    }

    /// Handle completion of an UnloadNode service call task.
    pub(super) async fn handle_unload_completion(
        &mut self,
        result: Result<
            eyre::Result<super::super::container_control::UnloadNodeResponse>,
            tokio::task::JoinError,
        >,
        composable_name: String,
    ) {
        match result {
            Ok(service_result) => {
                // Phase 12: Update composable node state directly
                if let Some(entry) = self.composable_nodes.get_mut(&composable_name) {
                    match &service_result {
                        Ok(response) if response.success => {
                            // Successfully unloaded
                            entry.state = ComposableState::Unloaded;
                            entry.unique_id = None;
                            entry.load_started_at = None;

                            debug!(
                                "{}: Successfully unloaded composable node '{}'",
                                self.name, composable_name
                            );

                            // Update shared state directly for composable node
                            self.shared_state.insert(
                                composable_name.clone(),
                                super::super::web_query::MemberState::Unloaded,
                            );

                            // Emit Unloaded event
                            let _ = self
                                .state_tx
                                .send(StateEvent::Unloaded {
                                    name: composable_name.clone(),
                                })
                                .await;
                        }
                        Ok(response) => {
                            // UnloadNode service returned failure - keep in current state
                            warn!(
                                "{}: Failed to unload composable node '{}': {}",
                                self.name, composable_name, response.error_message
                            );
                            // Transition back to Loaded state since unload failed
                            // Note: We don't have unique_id anymore, so this is a problem
                            // For now, keep in Unloading state (user can retry)
                        }
                        Err(e) => {
                            // Service call error - keep in current state
                            warn!(
                                "{}: UnloadNode service call failed for '{}': {:#}",
                                self.name, composable_name, e
                            );
                            // Keep in Unloading state for retry
                        }
                    }
                }
            }
            Err(e) => {
                // Task panicked
                warn!(
                    "{}: UnloadNode task panicked for {}: {:#}",
                    self.name, composable_name, e
                );
            }
        }
    }
}
