//! ComponentEvent message handling for container actors.
//!
//! Processes ComponentEvent messages published by `play_launch_container`
//! to track composable node lifecycle (LOADED, LOAD_FAILED, UNLOADED, CRASHED).
//! Also handles loading timeout fallback for DDS event loss scenarios.

use super::{ContainerActor, LOADING_TIMEOUT};
use crate::member_actor::{
    events::StateEvent,
    state::{BlockReason, ComposableState},
};
use tracing::{debug, error, warn};

impl ContainerActor {
    /// Handle a ComponentEvent message from the container (Phase 19.5a).
    ///
    /// ComponentEvent is the primary source of truth for composable node state.
    /// Service responses are a secondary path (guarded to avoid redundant transitions).
    /// With parallel dispatch, we match by unique_id across ALL composable entries.
    pub(super) async fn handle_component_event(
        &mut self,
        event: play_launch_msgs::msg::ComponentEvent,
    ) {
        use play_launch_msgs::msg::ComponentEvent as CE;

        match event.event_type {
            CE::LOADED => {
                self.handle_component_loaded(&event).await;
            }
            CE::LOAD_FAILED => {
                self.handle_component_load_failed(&event).await;
            }
            CE::UNLOADED => {
                self.handle_component_unloaded(&event).await;
            }
            CE::CRASHED => {
                self.handle_component_crashed(&event).await;
            }
            _ => {} // Unknown event type, ignore
        }
    }

    /// Handle a LOADED ComponentEvent.
    async fn handle_component_loaded(&mut self, event: &play_launch_msgs::msg::ComponentEvent) {
        // Match by unique_id across ALL entries (parallel dispatch)
        let found = self
            .composable_nodes
            .iter()
            .find(|(_, e)| e.unique_id == Some(event.unique_id))
            .map(|(name, _)| name.clone());
        let Some(composable_name) = found else {
            return; // No matching entry, ignore
        };
        if let Some(entry) = self.composable_nodes.get_mut(&composable_name) {
            if !matches!(entry.state, ComposableState::Loading { .. }) {
                return; // Already handled by service response
            }
            entry.state = ComposableState::Loaded {
                unique_id: event.unique_id,
            };
            entry.load_started_at = None;

            debug!(
                "{}: ComponentEvent LOADED for '{}' (unique_id: {})",
                self.name, composable_name, event.unique_id
            );

            // Update shared state for Web UI
            self.shared_state.insert(
                composable_name.clone(),
                crate::member_actor::web_query::MemberState::Loaded {
                    unique_id: event.unique_id,
                },
            );

            // Emit LoadSucceeded event
            let full_node_name = event.full_node_name.clone();
            let _ = self
                .state_tx
                .send(StateEvent::LoadSucceeded {
                    name: composable_name,
                    full_node_name,
                    unique_id: event.unique_id,
                })
                .await;
        }
    }

    /// Handle a LOAD_FAILED ComponentEvent.
    async fn handle_component_load_failed(
        &mut self,
        event: &play_launch_msgs::msg::ComponentEvent,
    ) {
        // Match by unique_id across ALL entries (parallel dispatch)
        let found = self
            .composable_nodes
            .iter()
            .find(|(_, e)| e.unique_id == Some(event.unique_id))
            .map(|(name, _)| name.clone());
        let Some(composable_name) = found else {
            return;
        };
        if let Some(entry) = self.composable_nodes.get_mut(&composable_name) {
            if !matches!(entry.state, ComposableState::Loading { .. }) {
                return; // Already handled by service response
            }
            entry.state = ComposableState::Failed {
                error: event.error_message.clone(),
            };
            entry.load_started_at = None;

            warn!(
                "{}: ComponentEvent LOAD_FAILED for '{}': {}",
                self.name, composable_name, event.error_message
            );

            // Update shared state for Web UI
            self.shared_state.insert(
                composable_name.clone(),
                crate::member_actor::web_query::MemberState::Failed {
                    error: event.error_message.clone(),
                },
            );

            // Emit LoadFailed event
            let _ = self
                .state_tx
                .send(StateEvent::LoadFailed {
                    name: composable_name,
                    error: event.error_message.clone(),
                })
                .await;
        }
    }

    /// Handle an UNLOADED ComponentEvent.
    async fn handle_component_unloaded(&mut self, event: &play_launch_msgs::msg::ComponentEvent) {
        // Match by unique_id (node already has it stored)
        let entry = self
            .composable_nodes
            .iter_mut()
            .find(|(_, e)| e.unique_id == Some(event.unique_id));
        if let Some((name, entry)) = entry {
            let name = name.clone();
            entry.state = ComposableState::Unloaded;
            entry.unique_id = None;
            entry.load_started_at = None;

            debug!(
                "{}: ComponentEvent UNLOADED for '{}' (unique_id: {})",
                self.name, name, event.unique_id
            );

            // Update shared state for Web UI
            self.shared_state.insert(
                name.clone(),
                crate::member_actor::web_query::MemberState::Unloaded,
            );

            // Emit Unloaded event
            let _ = self.state_tx.send(StateEvent::Unloaded { name }).await;
        }
    }

    /// Handle a CRASHED ComponentEvent.
    async fn handle_component_crashed(&mut self, event: &play_launch_msgs::msg::ComponentEvent) {
        // Find composable node by unique_id
        let entry = self
            .composable_nodes
            .iter_mut()
            .find(|(_, e)| e.unique_id == Some(event.unique_id));

        let Some((name, entry)) = entry else {
            return;
        };
        let name = name.clone();

        error!(
            "{}: Composable node '{}' crashed: {}",
            self.name, name, event.error_message
        );

        entry.state = ComposableState::Failed {
            error: event.error_message.clone(),
        };
        entry.unique_id = None;

        // Update shared state for Web UI
        self.shared_state.insert(
            name.clone(),
            crate::member_actor::web_query::MemberState::Failed {
                error: format!("Crashed: {}", event.error_message),
            },
        );

        // Emit StateEvent for logging/Web UI
        let _ = self
            .state_tx
            .send(StateEvent::LoadFailed {
                name: name.clone(),
                error: format!("Crashed: {}", event.error_message),
            })
            .await;
    }

    /// Transition all composable nodes to Blocked state.
    ///
    /// Called when the container stops, fails, or shuts down to mark all
    /// composable nodes as unavailable.
    pub(super) async fn transition_all_composables_to_blocked(&mut self, reason: BlockReason) {
        debug!(
            "{}: Transitioning all {} composable nodes to Blocked state (reason: {:?})",
            self.name,
            self.composable_nodes.len(),
            reason
        );

        for (name, entry) in self.composable_nodes.iter_mut() {
            // Only transition if not already blocked with this reason
            if entry.state != (ComposableState::Blocked { reason }) {
                entry.state = ComposableState::Blocked { reason };
                entry.unique_id = None;
                entry.load_started_at = None;

                // Emit Blocked event
                let _ = self
                    .state_tx
                    .send(StateEvent::Blocked {
                        name: name.clone(),
                        reason,
                    })
                    .await;

                // Update shared state directly for composable node
                use crate::member_actor::web_query::BlockReason as WebBlockReason;
                let web_reason = match reason {
                    BlockReason::NotStarted => WebBlockReason::ContainerNotStarted,
                    BlockReason::Stopped => WebBlockReason::ContainerStopped,
                    BlockReason::Failed => WebBlockReason::ContainerFailed,
                    BlockReason::Shutdown => WebBlockReason::Shutdown,
                };
                self.shared_state.insert(
                    name.clone(),
                    crate::member_actor::web_query::MemberState::Blocked { reason: web_reason },
                );
            }
        }
    }

    /// Check for composable nodes stuck in Loading state and promote them to
    /// Loaded if the LoadNode service succeeded more than 10 seconds ago.
    /// This handles DDS event loss where ComponentEvent LOADED never arrives.
    pub(super) async fn check_loading_timeouts(&mut self) {
        let mut promoted = Vec::new();
        for (name, entry) in &self.composable_nodes {
            if let ComposableState::Loading { started_at } = &entry.state {
                // Only promote if LoadNode succeeded (we have a unique_id)
                // and the timeout has elapsed
                if let Some(uid) = entry.unique_id {
                    if started_at.elapsed() > LOADING_TIMEOUT {
                        promoted.push((name.clone(), uid));
                    }
                }
            }
        }

        for (name, unique_id) in promoted {
            warn!(
                "{}: ComponentEvent LOADED not received for '{}' (unique_id: {}) \
                 after {}s -- falling back to service response",
                self.name,
                name,
                unique_id,
                LOADING_TIMEOUT.as_secs()
            );

            if let Some(entry) = self.composable_nodes.get_mut(&name) {
                entry.state = ComposableState::Loaded { unique_id };
                entry.load_started_at = None;

                self.shared_state.insert(
                    name.clone(),
                    crate::member_actor::web_query::MemberState::Loaded { unique_id },
                );

                let _ = self
                    .state_tx
                    .send(StateEvent::LoadSucceeded {
                        name: name.clone(),
                        full_node_name: String::new(),
                        unique_id,
                    })
                    .await;
            }
        }
    }
}
