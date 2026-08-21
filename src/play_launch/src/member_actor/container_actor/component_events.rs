//! ComponentEvent message handling (phase-51.4: methods on the
//! `ComposableSupervisor`, which owns the composable entries).
//!
//! Processes ComponentEvent messages published by `play_launch_container`
//! to track composable node lifecycle (LOADED, LOAD_FAILED, UNLOADED, CRASHED).
//! Also handles loading timeout fallback for DDS event loss scenarios.

use super::{ros_client::ContainerClients, supervisor::ComposableSupervisor};
use crate::member_actor::{
    events::{StateEvent, emit},
    model::{ActorConfig, BlockReason, ComposableState},
};
use std::time::Instant;
use tracing::{debug, error, info, warn};

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
fn pid_identity_ok(pid: i32, start_time: u64) -> bool {
    start_time == 0 || play_launch::sched::proc_start_time(pid as u32) == Some(start_time)
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

/// Which channel reported a composable's fate.
///
/// Kept in the log line rather than folded away, because from phase 64 there
/// are two and which one won is the first thing worth knowing when a load
/// looks wrong: the topic can drop a message under a startup storm, the socket
/// cannot. It also keeps `ComponentEvent LOADED for '<name>'` — the string
/// integration tests have grepped for since phase 19 — saying exactly what it
/// always said.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum LoadReport {
    ComponentEvent,
    ControlChannel,
}

impl LoadReport {
    pub(super) fn label(self) -> &'static str {
        match self {
            LoadReport::ComponentEvent => "ComponentEvent",
            LoadReport::ControlChannel => "control channel",
        }
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
        self.on_load_succeeded(
            LoadReport::ComponentEvent,
            event.unique_id,
            &event.full_node_name,
            event.pid,
            event.start_time,
            config,
        )
        .await;
    }

    /// A composable finished constructing — reported by whichever channel got
    /// there first: the `ComponentEvent` topic, or (phase 64) the private
    /// control socket.
    ///
    /// Both carry the same facts, and every transition below is guarded on the
    /// entry still being `Loading`, so a duplicate is a no-op. That is what
    /// lets the two coexist while a launch is mid-migration: the socket wins on
    /// speed, the topic is harmless when it arrives second, and neither has to
    /// know about the other.
    pub(super) async fn on_load_succeeded(
        &mut self,
        source: LoadReport,
        unique_id: u64,
        full_node_name: &str,
        pid: i32,
        start_time: u64,
        config: &ActorConfig,
    ) {
        // Match by unique_id across ALL entries (parallel dispatch)
        let found = self
            .composable_nodes
            .iter()
            .find(|(_, e)| e.unique_id == Some(unique_id))
            .map(|(name, _)| name.clone())
            // Fallback: a lost LoadNode response leaves the entry Loading
            // with unique_id None — claim it by full node name.
            //
            // NOT for a socket-tracked load. There the id is always known (the
            // container states it in `accepted`), so the fallback can only ever
            // match the WRONG thing: after a cancel-and-restart the topic still
            // delivers the previous attempt's event, and claiming the fresh
            // entry by name would let a dead attempt's outcome overwrite the
            // live one. Measured: a restarted composable was marked Failed by
            // the LOAD_FAILED of the attempt that had just been cancelled.
            .or_else(|| {
                self.composable_nodes
                    .iter()
                    .find(|(_, e)| {
                        e.tracking.is_none()
                            && e.unique_id.is_none()
                            && matches!(e.state, ComposableState::Loading { .. })
                            && full_node_name_of(&e.metadata) == full_node_name
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
            entry.unique_id = Some(unique_id);
            entry.state = ComposableState::Loaded { unique_id };
            entry.load_started_at = None;
        } else {
            return;
        }

        debug!(
            "{}: {} LOADED for '{}' (unique_id: {})",
            self.name(),
            source.label(),
            composable_name,
            unique_id
        );

        // Phase 38.9: apply resolved Linux scheduling to this composable's
        // own process, now that the load report carries its pid. Observable/
        // stock mode and failure reports carry pid 0, hence the guard.
        let sched_tier = self
            .composable_nodes
            .get(&composable_name)
            .and_then(|e| e.metadata.sched.clone());
        if config.sched_mode != crate::execution::sched_apply::SchedApplyMode::Off
            && pid > 0
            && let Some(tier) = sched_tier.as_ref()
        {
            if !pid_identity_ok(pid, start_time) {
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
                    pid
                );
            } else {
                match crate::execution::rt_helper_client::apply_sched(
                    config.sched_helper.as_ref(),
                    pid as u32,
                    tier,
                )
                .await
                {
                    Ok(()) => debug!(
                        "{}: applied tier '{}' to composable '{}' (pid {})",
                        self.name(),
                        tier.tier_name,
                        composable_name,
                        pid
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
                                pid,
                                e
                            );
                        } else {
                            warn!(
                                "{}: sched apply failed for composable '{}' (pid {}): {}",
                                self.name(),
                                composable_name,
                                pid,
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
                full_node_name: full_node_name.to_string(),
                unique_id,
            },
        )
        .await;
    }

    /// Handle a LOAD_FAILED ComponentEvent.
    async fn handle_component_load_failed(
        &mut self,
        event: &play_launch_msgs::msg::ComponentEvent,
    ) {
        self.on_load_failed(
            LoadReport::ComponentEvent,
            event.unique_id,
            &event.full_node_name,
            &event.package_name,
            &event.plugin_name,
            &event.error_message,
        )
        .await;
    }

    /// A composable failed to construct, from either report channel.
    pub(super) async fn on_load_failed(
        &mut self,
        source: LoadReport,
        unique_id: u64,
        full_node_name: &str,
        package: &str,
        plugin: &str,
        error: &str,
    ) {
        // Match by unique_id across ALL entries (parallel dispatch);
        // name-fallback for lost-LoadNode-response entries (unique_id None).
        // A failure may carry an empty full_node_name (ctor never ran), so
        // fall back further to package+plugin for a Loading no-id entry.
        let found = self
            .composable_nodes
            .iter()
            .find(|(_, e)| e.unique_id == Some(unique_id))
            .map(|(name, _)| name.clone())
            // Same restriction as the success path: a socket-tracked load
            // always has an id, so matching one by name or by package+plugin
            // could only attach a stale attempt's failure to a live one.
            .or_else(|| {
                self.composable_nodes
                    .iter()
                    .find(|(_, e)| {
                        e.tracking.is_none()
                            && e.unique_id.is_none()
                            && matches!(e.state, ComposableState::Loading { .. })
                            && (full_node_name_of(&e.metadata) == full_node_name
                                || (full_node_name.is_empty()
                                    && e.metadata.package == package
                                    && e.metadata.plugin == plugin))
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
                error: error.to_string(),
            };
            entry.load_started_at = None;
        } else {
            return;
        }

        warn!(
            "{}: {} LOAD_FAILED for '{}': {}",
            self.name(),
            source.label(),
            composable_name,
            error
        );

        // Emit LoadFailed event
        emit(
            self.state_tx(),
            StateEvent::LoadFailed {
                name: composable_name,
                error: error.to_string(),
            },
        )
        .await;
    }

    /// Handle an UNLOADED ComponentEvent.
    async fn handle_component_unloaded(&mut self, event: &play_launch_msgs::msg::ComponentEvent) {
        self.on_unloaded(LoadReport::ComponentEvent, event.unique_id)
            .await;
    }

    /// A composable was unloaded, from either report channel.
    pub(super) async fn on_unloaded(&mut self, source: LoadReport, unique_id: u64) {
        // Match by unique_id (node already has it stored)
        let entry = self
            .composable_nodes
            .iter_mut()
            .find(|(_, e)| e.unique_id == Some(unique_id));
        let Some((name, entry)) = entry else {
            return;
        };
        let name = name.clone();
        entry.state = ComposableState::Unloaded;
        entry.unique_id = None;
        entry.load_started_at = None;

        debug!(
            "{}: {} UNLOADED for '{}' (unique_id: {})",
            self.name(),
            source.label(),
            name,
            unique_id
        );

        // Emit Unloaded event
        emit(self.state_tx(), StateEvent::Unloaded { name }).await;
    }

    /// Handle a CRASHED ComponentEvent.
    async fn handle_component_crashed(&mut self, event: &play_launch_msgs::msg::ComponentEvent) {
        self.on_crashed(event.unique_id, &event.error_message).await;
    }

    /// A loaded composable's process died, from either report channel.
    pub(super) async fn on_crashed(&mut self, unique_id: u64, error: &str) {
        // Find composable node by unique_id
        let entry = self
            .composable_nodes
            .iter_mut()
            .find(|(_, e)| e.unique_id == Some(unique_id));

        let Some((name, entry)) = entry else {
            return;
        };
        let name = name.clone();

        entry.state = ComposableState::Failed {
            error: error.to_string(),
        };
        entry.unique_id = None;

        error!(
            "{}: Composable node '{}' crashed: {}",
            self.name(),
            name,
            error
        );

        // Emit StateEvent for logging/Web UI
        emit(
            self.state_tx(),
            StateEvent::LoadFailed {
                name: name.clone(),
                error: format!("Crashed: {}", error),
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
                VerifyOutcome::Absent if clients.has_event_sub() => {
                    // Phase 64 W2: `Absent` from OUR OWN container is not
                    // evidence of a lost load. `on_list_nodes` deliberately
                    // hides ids that are still constructing, so a composable
                    // that is merely slow answers exactly like one that never
                    // arrived — and re-dispatching it forks a SECOND process
                    // for a node that is alive and working. The reporter's
                    // launch had 14 lost LoadNode responses in one run, so
                    // this was one long constructor away from happening.
                    //
                    // The socket path can tell the difference because it asks
                    // (`docs/design/composable-load-lifecycle.md`); this path
                    // cannot, so it reports and waits.
                    warn!(
                        "{}: '{}' has been Loading for over {}s and ListNodes does not show it. \
                         That is ambiguous for this container — a constructor still running \
                         looks the same — so it is left alone. Enable the control channel \
                         (composable_node_loading.control_socket) to resolve this case.",
                        container_name,
                        name,
                        self.timings.total_budget.as_secs()
                    );
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
                    // A stock container's ListNodes is unambiguous: it cannot
                    // answer at all while a constructor holds its executor, so
                    // an answer that omits the node means the node is absent.
                    self.handle_load_composable(&name, clients, None).await;
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

    /// Promote composables stuck in `Loading` to `Loaded` when the
    /// `ComponentEvent` was lost — but only once ListNodes CONFIRMS the node
    /// is there.
    ///
    /// Issue 0019: this used to promote on the LoadNode response alone, after
    /// `loading_event_timeout` (10s). Under `--container-mode isolated` that
    /// response means only "spawn requested" — the container pre-assigns a
    /// unique_id and returns before the child is ready — so a node whose
    /// constructor was still running, or which the container later gave up on
    /// and killed, was reported as loaded anyway. That is how a 45s constructor
    /// produced `Startup complete: all nodes ready (composable 2/2)` at t+10s
    /// with one of the two dead, and how `motion_velocity_planner` counted
    /// toward `84/84 loaded` while absent from the ROS graph.
    ///
    /// A missing event is not evidence of success. `Unavailable` — the
    /// container's executor too busy to answer — is the normal state while a
    /// composable is being constructed, and is exactly the case that must NOT
    /// be read as loaded.
    /// Say what a still-loading composable is doing, so a long constructor is
    /// legible instead of indistinguishable from a hang.
    ///
    /// This is the whole reason a fixed ready timeout looked defensible: with
    /// nothing reporting, "1 pending" reads the same at 5 seconds and at 5
    /// minutes, so a deadline felt like the only way to find out. The container
    /// waits while the child is ALIVE and fails the moment it dies, which means
    /// "still Loading" already carries the liveness claim — the elapsed time is
    /// the only part missing, and this supervisor has it.
    ///
    /// Rate-limited per container rather than per composable: a launch with 84
    /// of them would otherwise print a wall of near-identical lines.
    pub(super) fn report_construction_progress(&mut self) {
        const REPORT_EVERY: std::time::Duration = std::time::Duration::from_secs(30);

        let now = Instant::now();
        if let Some(last) = self.last_progress_report
            && now.duration_since(last) < REPORT_EVERY
        {
            return;
        }

        let mut longest: Option<(&str, std::time::Duration)> = None;
        let mut pending = 0usize;
        for (name, entry) in &self.composable_nodes {
            if let ComposableState::Loading { started_at } = &entry.state {
                pending += 1;
                let elapsed = started_at.elapsed();
                if longest.is_none_or(|(_, d)| elapsed > d) {
                    longest = Some((name.as_str(), elapsed));
                }
            }
        }

        // Nothing pending: reset so the next slow load reports immediately
        // rather than waiting out a stale interval.
        let Some((name, elapsed)) = longest else {
            self.last_progress_report = None;
            return;
        };

        // Only worth saying once it has been a while — below that the load is
        // simply in flight and the startup line already covers it.
        if elapsed < REPORT_EVERY {
            return;
        }

        self.last_progress_report = Some(now);
        info!(
            "{}: {} composable(s) still constructing; longest '{}' at {}s \
             (the container waits while the child is alive)",
            self.name(),
            pending,
            name,
            elapsed.as_secs()
        );
    }

    pub(super) async fn check_loading_timeouts(&mut self, clients: &ContainerClients) {
        use super::ros_client::VerifyOutcome;

        let candidates: Vec<(String, u64, String)> = self
            .composable_nodes
            .iter()
            .filter_map(|(name, entry)| match &entry.state {
                ComposableState::Loading { started_at }
                    if entry.unique_id.is_some()
                        && started_at.elapsed() > self.timings.loading_event_timeout =>
                {
                    let ns = entry.metadata.namespace.trim_end_matches('/');
                    Some((
                        name.clone(),
                        entry.unique_id.unwrap(),
                        format!("{}/{}", ns, entry.metadata.node_name),
                    ))
                }
                _ => None,
            })
            .collect();

        for (name, unique_id, expected_full_name) in candidates {
            let container_name = self.name().to_string();
            match super::ros_client::verify_component_loaded(
                &container_name,
                &clients.list_client,
                &expected_full_name,
            )
            .await
            {
                VerifyOutcome::Present(confirmed_id) => {
                    warn!(
                        "{}: ComponentEvent LOADED not received for '{}' after {}s; \
                         ListNodes confirms it is loaded (unique_id: {})",
                        container_name,
                        name,
                        self.timings.loading_event_timeout.as_secs(),
                        confirmed_id
                    );
                    if let Some(entry) = self.composable_nodes.get_mut(&name) {
                        entry.unique_id = Some(confirmed_id);
                        entry.state = ComposableState::Loaded {
                            unique_id: confirmed_id,
                        };
                        entry.load_started_at = None;
                        emit(
                            self.state_tx(),
                            StateEvent::LoadSucceeded {
                                name: name.clone(),
                                full_node_name: expected_full_name.clone(),
                                unique_id: confirmed_id,
                            },
                        )
                        .await;
                    }
                }
                VerifyOutcome::Absent => {
                    // The container answered and does not have it. Leave it
                    // Loading rather than failing it here: `rescue_lost_loads`
                    // owns the give-up decision at `total_budget` and will
                    // re-dispatch. Saying so is the point — this used to be
                    // silently counted as a success.
                    debug!(
                        "{}: '{}' (unique_id: {}) not present per ListNodes — still loading, \
                         not promoting",
                        container_name, name, unique_id
                    );
                }
                VerifyOutcome::Unavailable => {
                    // The usual case for a long constructor: the container's
                    // executor cannot answer while it is building. Waiting is
                    // correct; assuming success is not.
                    debug!(
                        "{}: '{}' still Loading and the container is busy — leaving it alone",
                        container_name, name
                    );
                }
            }
        }
    }
}
