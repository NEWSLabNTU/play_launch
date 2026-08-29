//! Phase 64 — control-socket messages → composable transitions.
//!
//! The counterpart of [`super::component_events`], for the private channel to
//! our own container binary. Every terminal outcome lands in the SAME handler
//! the ComponentEvent path uses (`on_load_succeeded`, `on_load_failed`,
//! `on_unloaded`, `on_crashed`), so there is one state machine with two
//! entrances rather than two implementations that can drift.
//!
//! What is genuinely new here is the middle of a load: `Accepted` (the fact
//! the LoadNode response used to carry, delivered without a service call) and
//! `Constructing` (liveness while a constructor runs). The five mechanisms
//! that existed to reconstruct those facts through a congested rmw layer —
//! the 30 s timeout, the triple retry, the response-lost deferral, the 10 s
//! ComponentEvent wait and the ListNodes fallback — have nothing to do on this
//! path, because the container states them.

use super::{
    component_events::LoadReport, control_channel::ControlChannel, supervisor::ComposableSupervisor,
};
use crate::{
    ipc::container_protocol::{ContainerMsg, LoadPhase},
    member_actor::{
        events::{StateEvent, emit},
        model::{ActorConfig, ComposableState},
    },
};
use std::time::Instant;
use tracing::{debug, info, warn};

/// Don't print a liveness line more than this often per container: a container
/// with nine slow composables would otherwise produce a wall of them.
const PROGRESS_REPORT_EVERY: std::time::Duration = std::time::Duration::from_secs(30);

impl ComposableSupervisor {
    /// Handle one message from the container's control socket.
    pub(super) async fn handle_container_msg(
        &mut self,
        msg: ContainerMsg,
        config: &ActorConfig,
        control: &mut ControlChannel,
    ) {
        match msg {
            ContainerMsg::Hello { .. } => {
                // Negotiated in `await_hello`; a second one is meaningless.
            }
            ContainerMsg::Accepted { seq, unique_id } => {
                let Some(name) = control.take_pending(seq) else {
                    debug!(
                        "{}: control channel accepted seq {} we no longer track",
                        self.name(),
                        seq
                    );
                    return;
                };
                if let Some(entry) = self.composable_nodes.get_mut(&name)
                    && matches!(entry.state, ComposableState::Loading { .. })
                {
                    entry.unique_id = Some(unique_id);
                    if let Some(tracking) = entry.tracking.as_mut() {
                        tracking.unique_id = Some(unique_id);
                    }
                    debug!(
                        "{}: '{}' accepted by the container (unique_id {})",
                        self.name(),
                        name,
                        unique_id
                    );
                }
                // Acceptance is a report: it restarts the silence clock and
                // moves the entry out of the one state where a resend is safe.
                self.note_report(&name, Some(LoadPhase::Queued), 0, 0);
            }
            ContainerMsg::Rejected { seq, error } => {
                let Some(name) = control.take_pending(seq) else {
                    warn!(
                        "{}: control channel rejected seq {} we no longer track: {}",
                        self.name(),
                        seq,
                        error
                    );
                    return;
                };
                if let Some(entry) = self.composable_nodes.get_mut(&name)
                    && matches!(entry.state, ComposableState::Loading { .. })
                {
                    entry.state = ComposableState::Failed {
                        error: error.clone(),
                    };
                    entry.load_started_at = None;
                    warn!("{}: container refused '{}': {}", self.name(), name, error);
                    emit(self.state_tx(), StateEvent::LoadFailed { name, error }).await;
                }
            }
            ContainerMsg::Loaded {
                unique_id,
                full_node_name,
                pid,
                start_time,
            } => {
                if let Some(name) = self.name_for_unique_id(unique_id) {
                    self.note_report(&name, Some(LoadPhase::Loaded), pid, 0);
                }
                self.on_load_succeeded(
                    LoadReport::ControlChannel,
                    unique_id,
                    &full_node_name,
                    pid,
                    start_time,
                    config,
                )
                .await;
            }
            ContainerMsg::LoadFailed {
                unique_id,
                error,
                cancelled,
            } => {
                // A cancellation is not just a failure: it is the container
                // CONFIRMING that nothing is running for this id, which is the
                // precondition for a safe resend. Route it to the policy that
                // knows what the cancel was for.
                if cancelled && let Some(name) = self.name_for_unique_id(unique_id) {
                    self.on_cancel_confirmed(name, error, control).await;
                    return;
                }
                self.on_load_failed(LoadReport::ControlChannel, unique_id, "", "", "", &error)
                    .await;
            }
            ContainerMsg::Constructing {
                unique_id,
                pid,
                elapsed_ms,
                plugin,
                phase,
                cpu_ms,
            } => {
                if let Some(name) = self.name_for_unique_id(unique_id) {
                    self.note_report(&name, Some(phase), pid, cpu_ms);
                }
                self.report_socket_progress(unique_id, pid, elapsed_ms, &plugin, phase);
            }
            ContainerMsg::Status {
                seq,
                unique_id,
                phase,
                pid,
                elapsed_ms,
                cpu_ms,
                plugin,
                cancellable: _,
                full_node_name,
            } => {
                // Resolve by id where we have one, else by the seq the query
                // was asked under — the pre-acknowledgement case, where the
                // supervisor has no id at all.
                let name = self.name_for_unique_id(unique_id).or_else(|| {
                    seq.and_then(|seq| control.peek_pending(seq))
                        .map(str::to_string)
                });
                let Some(name) = name else {
                    debug!(
                        "{}: status for an unknown load (id {}, seq {:?}, plugin '{}')",
                        self.name(),
                        unique_id,
                        seq,
                        plugin
                    );
                    return;
                };
                self.on_status(
                    super::load_policy::StatusReport {
                        name,
                        phase,
                        pid,
                        elapsed_ms,
                        cpu_ms,
                        full_node_name,
                    },
                    config,
                    control,
                )
                .await;
            }
            ContainerMsg::Unloaded { unique_id, .. } => {
                self.on_unloaded(LoadReport::ControlChannel, unique_id)
                    .await;
            }
            ContainerMsg::Crashed {
                unique_id, error, ..
            } => {
                let name = self.name_for_unique_id(unique_id);
                self.on_crashed(unique_id, &error).await;
                // The container reaped the child and erased its id, so this is
                // the one retry that needs no confirmation step — it is
                // already confirmed.
                if let Some(name) = name {
                    self.schedule_crash_reload(&name, config);
                }
            }
        }
    }

    /// Report a constructor that is still running, said by the container
    /// rather than inferred from a timeout.
    ///
    /// This is the honest half of the acceptance criterion: a wedged composable
    /// keeps producing these lines with its own pid and elapsed time, so "slow"
    /// and "hung" look different to the operator, and neither is reported as a
    /// failure the load path invented.
    fn report_socket_progress(
        &mut self,
        unique_id: u64,
        pid: i32,
        elapsed_ms: u64,
        plugin: &str,
        phase: LoadPhase,
    ) {
        let label = self
            .name_for_unique_id(unique_id)
            .unwrap_or_else(|| plugin.to_string());

        let now = Instant::now();
        if let Some(last) = self.last_progress_report
            && now.duration_since(last) < PROGRESS_REPORT_EVERY
        {
            debug!(
                "{}: '{}' {} for {}s (pid {})",
                self.name(),
                label,
                describe(phase),
                elapsed_ms / 1000,
                pid
            );
            return;
        }
        self.last_progress_report = Some(now);
        if phase == LoadPhase::Queued {
            // Nothing has been forked yet: there is no pid to be alive, so the
            // honest report is about the queue, not about a process. Before
            // W2 this window (up to two minutes at the container's memory
            // gate) was silent and looked exactly like a lost load.
            info!(
                "{}: '{}' waiting {}s for a spawn slot in the container (nothing forked yet)",
                self.name(),
                label,
                elapsed_ms / 1000
            );
        } else {
            info!(
                "{}: '{}' still constructing after {}s (pid {}, alive — reported by the container)",
                self.name(),
                label,
                elapsed_ms / 1000,
                pid
            );
        }
    }

    /// The composable an id belongs to, if any.
    pub(super) fn name_for_unique_id(&self, unique_id: u64) -> Option<String> {
        self.composable_nodes
            .iter()
            .find(|(_, e)| e.unique_id == Some(unique_id))
            .map(|(n, _)| n.clone())
    }
}

fn describe(phase: LoadPhase) -> &'static str {
    match phase {
        LoadPhase::Queued => "waiting for a spawn slot",
        LoadPhase::Constructing => "constructing",
        LoadPhase::Loaded => "loaded",
        LoadPhase::Failed => "failed",
        LoadPhase::Unknown => "unknown to the container",
    }
}
