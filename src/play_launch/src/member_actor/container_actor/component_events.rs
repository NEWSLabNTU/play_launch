//! ComponentEvent message handling (phase-51.4: methods on the
//! `ComposableSupervisor`, which owns the composable entries).
//!
//! Processes ComponentEvent messages published by `play_launch_container`
//! to track composable node lifecycle (LOADED, LOAD_FAILED, UNLOADED, CRASHED).
//! Also handles loading timeout fallback for DDS event loss scenarios.

use super::supervisor::ComposableSupervisor;
use crate::member_actor::{
    events::{StateEvent, emit},
    model::{ActorConfig, BlockReason, ComposableState},
};
use tracing::{debug, error, warn};

/// Verify that `event.pid` still refers to the process that was actually
/// spawned, guarding against PID-reuse TOCTOU between the LOADED publish and
/// the sched apply: composables have no held `Child` handle (unlike regular
/// nodes/containers), so if the process died and the kernel recycled its PID
/// to an unrelated process, we must not set SCHED_FIFO on it.
///
/// `event.start_time == 0` means the container couldn't determine an
/// identity token (older container binary, or a CRASHED/LOAD_FAILED event) —
/// in that case there is nothing to verify against, so we keep today's
/// pid>0-only behavior for wire compatibility.
fn pid_identity_ok(event: &play_launch_msgs::msg::ComponentEvent) -> bool {
    event.start_time == 0
        || play_launch::sched::proc_start_time(event.pid as u32) == Some(event.start_time)
}

/// Expected fully-qualified node name for a composable entry
/// (namespace-joined; used for name-fallback ComponentEvent matching when
/// the LoadNode response was lost and no unique_id is known).
fn full_node_name_of(meta: &super::supervisor::ComposableNodeMetadata) -> String {
    if meta.namespace == "/" || meta.namespace.is_empty() {
        format!("/{}", meta.node_name)
    } else {
        format!(
            "{}/{}",
            meta.namespace.trim_end_matches('/'),
            meta.node_name
        )
    }
}

impl ComposableSupervisor {
    /// Handle a ComponentEvent message from the container (Phase 19.5a).
    ///
    /// ComponentEvent is the primary source of truth for composable node state.
    /// Service responses are a secondary path (guarded to avoid redundant transitions).
    /// With parallel dispatch, we match by unique_id across ALL composable entries.
    pub(super) async fn handle_component_event(
        &mut self,
        event: play_launch_msgs::msg::ComponentEvent,
        config: &ActorConfig,
    ) {
        use play_launch_msgs::msg::ComponentEvent as CE;

        match event.event_type {
            CE::LOADED => {
                self.handle_component_loaded(&event, config).await;
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
    async fn handle_component_loaded(
        &mut self,
        event: &play_launch_msgs::msg::ComponentEvent,
        config: &ActorConfig,
    ) {
        // Match by unique_id across ALL entries (parallel dispatch)
        let found = self
            .composable_nodes
            .iter()
            .find(|(_, e)| e.unique_id == Some(event.unique_id))
            .map(|(name, _)| name.clone())
            // Fallback: a lost LoadNode response leaves the entry Loading
            // with unique_id None — claim it by full node name.
            .or_else(|| {
                self.composable_nodes
                    .iter()
                    .find(|(_, e)| {
                        e.unique_id.is_none()
                            && matches!(e.state, ComposableState::Loading { .. })
                            && full_node_name_of(&e.metadata) == event.full_node_name
                    })
                    .map(|(name, _)| name.clone())
            });
        let Some(composable_name) = found else {
            return; // No matching entry, ignore
        };
        if let Some(entry) = self.composable_nodes.get_mut(&composable_name) {
            if !matches!(entry.state, ComposableState::Loading { .. }) {
                return; // Already handled by service response
            }
            entry.unique_id = Some(event.unique_id);
            entry.state = ComposableState::Loaded {
                unique_id: event.unique_id,
            };
            entry.load_started_at = None;
        } else {
            return;
        }

        debug!(
            "{}: ComponentEvent LOADED for '{}' (unique_id: {})",
            self.name(),
            composable_name,
            event.unique_id
        );

        // Phase 38.9: apply resolved Linux scheduling to this composable's
        // own process, now that ComponentEvent carries its pid. Observable/
        // stock mode and LOAD_FAILED events carry pid 0, hence the guard.
        let sched_tier = self
            .composable_nodes
            .get(&composable_name)
            .and_then(|e| e.metadata.sched.clone());
        if config.sched_mode != crate::execution::sched_apply::SchedApplyMode::Off
            && event.pid > 0
            && let Some(tier) = sched_tier.as_ref()
        {
            if !pid_identity_ok(event) {
                // The pid no longer matches the start_time captured at
                // spawn: the composable died and (post PID-wraparound)
                // the kernel may have handed this pid to an unrelated
                // process. Skip the apply — there is nothing to
                // schedule, and this is not a strict failure.
                warn!(
                    "{}: composable '{}' pid {} no longer matches its start_time \
                     -- skipping sched apply (pid reused or process gone)",
                    self.name(),
                    composable_name,
                    event.pid
                );
            } else {
                match crate::execution::rt_helper_client::apply_sched(
                    config.sched_helper.as_ref(),
                    event.pid as u32,
                    tier,
                )
                .await
                {
                    Ok(()) => debug!(
                        "{}: applied tier '{}' to composable '{}' (pid {})",
                        self.name(),
                        tier.tier_name,
                        composable_name,
                        event.pid
                    ),
                    Err(e) => {
                        // v1: a per-composable failure does not hard-abort the
                        // container mid-run (config errors already fail at
                        // SchedPlan::build; missing CAP is caught by the
                        // pre-spawn preflight). Strict logs loudly; Warn logs
                        // a warning.
                        if config.sched_mode
                            == crate::execution::sched_apply::SchedApplyMode::Strict
                        {
                            error!(
                                "{}: STRICT sched apply failed for composable '{}' (pid {}): {}",
                                self.name(),
                                composable_name,
                                event.pid,
                                e
                            );
                        } else {
                            warn!(
                                "{}: sched apply failed for composable '{}' (pid {}): {}",
                                self.name(),
                                composable_name,
                                event.pid,
                                e
                            );
                        }
                    }
                }
            }
        }

        // Emit LoadSucceeded event
        emit(
            self.state_tx(),
            StateEvent::LoadSucceeded {
                name: composable_name,
                full_node_name: event.full_node_name.clone(),
                unique_id: event.unique_id,
            },
        )
        .await;
    }

    /// Handle a LOAD_FAILED ComponentEvent.
    async fn handle_component_load_failed(
        &mut self,
        event: &play_launch_msgs::msg::ComponentEvent,
    ) {
        // Match by unique_id across ALL entries (parallel dispatch);
        // name-fallback for lost-LoadNode-response entries (unique_id None).
        // LOAD_FAILED may carry an empty full_node_name (ctor never ran), so
        // fall back further to package+plugin for a Loading no-id entry.
        let found = self
            .composable_nodes
            .iter()
            .find(|(_, e)| e.unique_id == Some(event.unique_id))
            .map(|(name, _)| name.clone())
            .or_else(|| {
                self.composable_nodes
                    .iter()
                    .find(|(_, e)| {
                        e.unique_id.is_none()
                            && matches!(e.state, ComposableState::Loading { .. })
                            && (full_node_name_of(&e.metadata) == event.full_node_name
                                || (event.full_node_name.is_empty()
                                    && e.metadata.package == event.package_name
                                    && e.metadata.plugin == event.plugin_name))
                    })
                    .map(|(name, _)| name.clone())
            });
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
        } else {
            return;
        }

        warn!(
            "{}: ComponentEvent LOAD_FAILED for '{}': {}",
            self.name(),
            composable_name,
            event.error_message
        );

        // Emit LoadFailed event
        emit(
            self.state_tx(),
            StateEvent::LoadFailed {
                name: composable_name,
                error: event.error_message.clone(),
            },
        )
        .await;
    }

    /// Handle an UNLOADED ComponentEvent.
    async fn handle_component_unloaded(&mut self, event: &play_launch_msgs::msg::ComponentEvent) {
        // Match by unique_id (node already has it stored)
        let entry = self
            .composable_nodes
            .iter_mut()
            .find(|(_, e)| e.unique_id == Some(event.unique_id));
        let Some((name, entry)) = entry else {
            return;
        };
        let name = name.clone();
        entry.state = ComposableState::Unloaded;
        entry.unique_id = None;
        entry.load_started_at = None;

        debug!(
            "{}: ComponentEvent UNLOADED for '{}' (unique_id: {})",
            self.name(),
            name,
            event.unique_id
        );

        // Emit Unloaded event
        emit(self.state_tx(), StateEvent::Unloaded { name }).await;
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

        entry.state = ComposableState::Failed {
            error: event.error_message.clone(),
        };
        entry.unique_id = None;

        error!(
            "{}: Composable node '{}' crashed: {}",
            self.name(),
            name,
            event.error_message
        );

        // Emit StateEvent for logging/Web UI
        emit(
            self.state_tx(),
            StateEvent::LoadFailed {
                name: name.clone(),
                error: format!("Crashed: {}", event.error_message),
            },
        )
        .await;
    }

    /// Transition all composable nodes to Blocked state.
    ///
    /// Called when the container stops, fails, or shuts down to mark all
    /// composable nodes as unavailable.
    pub(super) async fn transition_all_composables_to_blocked(&mut self, reason: BlockReason) {
        debug!(
            "{}: Transitioning all {} composable nodes to Blocked state (reason: {:?})",
            self.name(),
            self.composable_nodes.len(),
            reason
        );

        let mut blocked = Vec::new();
        for (name, entry) in self.composable_nodes.iter_mut() {
            // Only transition if not already blocked with this reason
            if entry.state != (ComposableState::Blocked { reason }) {
                entry.state = ComposableState::Blocked { reason };
                entry.unique_id = None;
                entry.load_started_at = None;
                blocked.push(name.clone());
            }
        }
        for name in blocked {
            // Emit Blocked event
            emit(self.state_tx(), StateEvent::Blocked { name, reason }).await;
        }
    }

    /// Rescue composable nodes stuck in Loading with NO unique_id — the load
    /// call's completion never arrived (lost LoadNode response deferred to a
    /// ComponentEvent that never came, or the spawned load task died). The
    /// entry would otherwise stay "pending" forever and the member silently
    /// never exists. Resolve via ListNodes: present → promote, absent →
    /// re-dispatch the load (safe: an answering container proves it is not
    /// still constructing the original request), busy → retry next tick.
    pub(super) async fn rescue_lost_loads(
        &mut self,
        clients: &super::ros_client::ContainerClients,
    ) {
        use super::ros_client::VerifyOutcome;

        let stuck: Vec<(String, String)> = self
            .composable_nodes
            .iter()
            .filter_map(|(name, entry)| match &entry.state {
                ComposableState::Loading { started_at }
                    if entry.unique_id.is_none()
                        && started_at.elapsed() > self.timings.total_budget =>
                {
                    let ns = entry.metadata.namespace.trim_end_matches('/');
                    Some((name.clone(), format!("{}/{}", ns, entry.metadata.node_name)))
                }
                _ => None,
            })
            .collect();

        for (name, expected_full_name) in stuck {
            let container_name = self.name().to_string();
            match super::ros_client::verify_component_loaded(
                &container_name,
                &clients.list_client,
                &expected_full_name,
            )
            .await
            {
                VerifyOutcome::Present(unique_id) => {
                    warn!(
                        "{}: '{}' was stuck Loading with a lost response; ListNodes \
                         confirms it loaded (unique_id: {})",
                        container_name, name, unique_id
                    );
                    if let Some(entry) = self.composable_nodes.get_mut(&name) {
                        entry.unique_id = Some(unique_id);
                        entry.state = ComposableState::Loaded { unique_id };
                        entry.load_started_at = None;
                        emit(
                            self.state_tx(),
                            StateEvent::LoadSucceeded {
                                name: name.clone(),
                                full_node_name: expected_full_name,
                                unique_id,
                            },
                        )
                        .await;
                    }
                }
                VerifyOutcome::Absent => {
                    warn!(
                        "{}: '{}' was stuck Loading past the {}s budget and the \
                         container does not have it — the load was lost; re-dispatching",
                        container_name,
                        name,
                        self.timings.total_budget.as_secs()
                    );
                    if let Some(entry) = self.composable_nodes.get_mut(&name) {
                        entry.state = ComposableState::Unloaded;
                        entry.load_started_at = None;
                    }
                    self.handle_load_composable(&name, clients).await;
                }
                VerifyOutcome::Unavailable => {
                    debug!(
                        "{}: '{}' stuck Loading, container busy; will re-verify",
                        container_name, name
                    );
                }
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
                if let Some(uid) = entry.unique_id
                    && started_at.elapsed() > self.timings.loading_event_timeout
                {
                    promoted.push((name.clone(), uid));
                }
            }
        }

        for (name, unique_id) in promoted {
            warn!(
                "{}: ComponentEvent LOADED not received for '{}' (unique_id: {}) \
                 after {}s -- falling back to service response",
                self.name(),
                name,
                unique_id,
                self.timings.loading_event_timeout.as_secs()
            );

            if let Some(entry) = self.composable_nodes.get_mut(&name) {
                entry.state = ComposableState::Loaded { unique_id };
                entry.load_started_at = None;

                emit(
                    self.state_tx(),
                    StateEvent::LoadSucceeded {
                        name: name.clone(),
                        full_node_name: String::new(),
                        unique_id,
                    },
                )
                .await;
            }
        }
    }
}
