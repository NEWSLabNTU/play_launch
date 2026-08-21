//! Phase 64 W2 — when a load is declared lost, and when it may be retried.
//!
//! Design: `docs/design/composable-load-lifecycle.md`. The rules this file
//! implements, stated once so a future change can be checked against them:
//!
//! - **I1.** A resend requires a *confirmed absence* stated by the owner of the
//!   process. Never a timeout, never an inference from silence.
//! - **I2.** Time does not convert into failure while liveness holds. Only lost
//!   liveness, or an explicit operator policy, does.
//! - **I3.** Every wait is reported with its evidence — phase, pid, elapsed,
//!   CPU — so "slow versus stuck" is a human judgment with data.
//! - **I4.** Whoever owns the child kills the child. A retry is cancel →
//!   confirm → resend, so a double load is unrepresentable rather than
//!   unlikely.
//! - **I5.** "Cannot be measured" is its own state: never read as success
//!   (issue #0019), never as failure (the double-load hazard `rescue_lost_loads`
//!   still carries on the LoadNode path).
//!
//! Everything here runs on the container actor's existing 5 s tick. Each timer
//! below can only ever cause a QUESTION to the container; the container's
//! answer is what decides.

use super::{
    control_channel::ControlChannel,
    supervisor::{CancelIntent, ComposableSupervisor},
};
use crate::{
    cli::config::{ComposableRespawn, StallAction},
    ipc::container_protocol::LoadPhase,
    member_actor::{
        events::{StateEvent, emit},
        model::{ActorConfig, ComposableState},
    },
};
use std::time::{Duration, Instant};
use tracing::{debug, info, warn};

/// One composable's answer to "should anything happen to this load now?".
#[derive(Debug, Clone, PartialEq, Eq)]
enum SweepAction {
    /// Nothing is due.
    Wait,
    /// Ask the container by `seq` — the load was never acknowledged, so no id
    /// exists yet.
    QueryBySeq(u64),
    /// Ask the container by `unique_id`.
    QueryById(u64),
    /// A previous query went unanswered; ask again.
    Reprobe {
        seq: Option<u64>,
        unique_id: Option<u64>,
        unanswered: u32,
    },
    /// A crash-respawn delay has elapsed.
    RetryNow,
}

/// Decide what is due for one entry. Pure, so the policy is testable without a
/// container: everything it needs is the tracking record and the clock.
fn decide(
    tracking: &super::supervisor::LoadTracking,
    now: Instant,
    ack_timeout: Duration,
    report_timeout: Duration,
    probe_interval: Duration,
) -> SweepAction {
    // A cancel is out; the confirmation is what resolves it, and probing on
    // top would race the answer we are already waiting for.
    if tracking.cancel_intent.is_some() {
        return SweepAction::Wait;
    }

    if let Some(sent) = tracking.probe_sent_at {
        if now.duration_since(sent) >= probe_interval {
            return SweepAction::Reprobe {
                seq: tracking.phase.is_none().then_some(tracking.seq),
                unique_id: tracking.unique_id,
                unanswered: tracking.unanswered_probes,
            };
        }
        return SweepAction::Wait;
    }

    let quiet_for = now.duration_since(tracking.last_report);
    match tracking.phase {
        // Not acknowledged: the only state where a resend can be safe, because
        // no id was assigned and therefore nothing was forked.
        None if quiet_for >= ack_timeout => SweepAction::QueryBySeq(tracking.seq),
        None => SweepAction::Wait,
        // Acknowledged and then silent. `constructing` frames arrive every 15 s
        // while a child is alive, so silence past three of them means the
        // reporting stopped, not that the load did.
        Some(_) if quiet_for >= report_timeout => match tracking.unique_id {
            Some(id) => SweepAction::QueryById(id),
            None => SweepAction::QueryBySeq(tracking.seq),
        },
        Some(_) => SweepAction::Wait,
    }
}

impl ComposableSupervisor {
    /// The W2 sweep. Runs on the actor's 5 s tick whenever loads go over the
    /// socket, in place of the two `ListNodes` sweeps (which stay as the
    /// degraded-mode path when the socket itself is what broke).
    pub(super) async fn reconcile_socket_loads(&mut self, control: &mut ControlChannel) {
        let now = Instant::now();
        let ack = self.timings.ack_timeout;
        let report = self.timings.report_timeout;
        let probe = self.timings.probe_interval;

        // Collect first: the actions below need `&mut self`.
        let due: Vec<(String, SweepAction)> = self
            .composable_nodes
            .iter()
            .filter_map(|(name, entry)| {
                if let Some(at) = entry.retry_after
                    && now >= at
                {
                    return Some((name.clone(), SweepAction::RetryNow));
                }
                if !matches!(entry.state, ComposableState::Loading { .. }) {
                    return None;
                }
                let tracking = entry.tracking.as_ref()?;
                match decide(tracking, now, ack, report, probe) {
                    SweepAction::Wait => None,
                    action => Some((name.clone(), action)),
                }
            })
            .collect();

        for (name, action) in due {
            match action {
                SweepAction::Wait => {}
                SweepAction::QueryBySeq(seq) => {
                    debug!(
                        "{}: '{}' unacknowledged for {}s — asking the container",
                        self.name(),
                        name,
                        ack.as_secs()
                    );
                    control.send_query(Some(seq), None);
                    self.mark_probe_sent(&name, now);
                }
                SweepAction::QueryById(id) => {
                    debug!(
                        "{}: '{}' has been quiet for {}s — asking the container",
                        self.name(),
                        name,
                        report.as_secs()
                    );
                    control.send_query(None, Some(id));
                    self.mark_probe_sent(&name, now);
                }
                SweepAction::Reprobe {
                    seq,
                    unique_id,
                    unanswered,
                } => {
                    // An unanswered query is its own state (I5): the load is
                    // neither confirmed running nor confirmed gone, so it is
                    // reported and re-asked, never resolved by assumption.
                    if (unanswered + 1).is_power_of_two() {
                        warn!(
                            "{}: '{}' — {} control-channel queries unanswered; the load is \
                             neither confirmed running nor confirmed gone, so it is left alone",
                            self.name(),
                            name,
                            unanswered + 1
                        );
                    }
                    control.send_query(seq, unique_id);
                    self.mark_probe_sent(&name, now);
                    if let Some(t) = self.tracking_mut(&name) {
                        t.unanswered_probes = unanswered + 1;
                    }
                }
                SweepAction::RetryNow => {
                    info!("{}: reloading '{}' after its crash", self.name(), name);
                    self.start_socket_load(&name, control).await;
                }
            }
        }

        self.check_for_stalls(control).await;
    }

    /// Put an entry back into `Loading` and dispatch it over the socket.
    ///
    /// Used by the paths that restart a load from a terminal state (a crash
    /// reload); the ones that never left `Loading` — a lost load, a confirmed
    /// cancellation — dispatch directly.
    async fn start_socket_load(&mut self, name: &str, control: &mut ControlChannel) {
        let started_at = Instant::now();
        if let Some(entry) = self.composable_nodes.get_mut(name) {
            entry.retry_after = None;
            entry.unique_id = None;
            entry.state = ComposableState::Loading { started_at };
            entry.load_started_at = Some(started_at);
        }
        emit(
            self.state_tx(),
            StateEvent::LoadStarted {
                name: name.to_string(),
            },
        )
        .await;
        self.send_load_over_socket(name, control).await;
    }

    fn tracking_mut(&mut self, name: &str) -> Option<&mut super::supervisor::LoadTracking> {
        self.composable_nodes
            .get_mut(name)
            .and_then(|e| e.tracking.as_mut())
    }

    fn mark_probe_sent(&mut self, name: &str, now: Instant) {
        if let Some(t) = self.tracking_mut(name) {
            t.probe_sent_at = Some(now);
        }
    }

    /// Record whatever the container just said about a load. Any frame counts
    /// as a report: it is evidence the load is still being worked on.
    pub(super) fn note_report(
        &mut self,
        name: &str,
        phase: Option<LoadPhase>,
        pid: i32,
        cpu_ms: u64,
    ) {
        let now = Instant::now();
        if let Some(t) = self.tracking_mut(name) {
            t.last_report = now;
            t.probe_sent_at = None;
            t.unanswered_probes = 0;
            if let Some(phase) = phase {
                t.phase = Some(phase);
            }
            if pid > 0 {
                t.pid = pid;
            }
            if cpu_ms > 0 || t.cpu_sampled_at.is_some() {
                t.prev_cpu_ms = t.cpu_ms;
                t.cpu_ms = cpu_ms;
                t.cpu_sampled_at = Some(now);
            }
        }
    }

    /// The container answered a `Query`. This is the only place a load may be
    /// declared lost, and the only place a resend is authorised — because
    /// `Unknown` here is a statement by the process owner, not an inference.
    pub(super) async fn on_status(
        &mut self,
        name: String,
        phase: LoadPhase,
        pid: i32,
        elapsed_ms: u64,
        cpu_ms: u64,
        full_node_name: String,
        config: &ActorConfig,
        control: &mut ControlChannel,
    ) {
        let Some(entry) = self.composable_nodes.get(&name) else {
            return;
        };
        if !matches!(entry.state, ComposableState::Loading { .. }) {
            return; // already resolved by a frame that crossed this answer
        }

        self.note_report(&name, Some(phase), pid, cpu_ms);

        match phase {
            LoadPhase::Queued | LoadPhase::Constructing => {
                // The answer to "is it slow or gone?" is: slow. This is the
                // case the old ListNodes verification could not distinguish —
                // its `Absent` verdict covered both, which is what made a
                // resend able to double-load a working node.
                info!(
                    "{}: '{}' is {} at {}s (pid {}) — still in flight, not resending",
                    self.name(),
                    name,
                    if phase == LoadPhase::Queued {
                        "queued for a spawn slot"
                    } else {
                        "constructing"
                    },
                    elapsed_ms / 1000,
                    pid
                );
            }
            LoadPhase::Loaded => {
                let unique_id = self
                    .composable_nodes
                    .get(&name)
                    .and_then(|e| e.unique_id)
                    .unwrap_or(0);
                warn!(
                    "{}: '{}' was already loaded and we had not noticed — adopting the \
                     container's answer",
                    self.name(),
                    name
                );
                self.on_load_succeeded(
                    super::component_events::LoadReport::ControlChannel,
                    unique_id,
                    &full_node_name,
                    pid,
                    0,
                    config,
                )
                .await;
            }
            LoadPhase::Failed => {
                self.on_load_failed(
                    super::component_events::LoadReport::ControlChannel,
                    self.composable_nodes
                        .get(&name)
                        .and_then(|e| e.unique_id)
                        .unwrap_or(0),
                    "",
                    "",
                    "",
                    "the container reports this load as failed",
                )
                .await;
            }
            LoadPhase::Unknown => {
                self.handle_lost_load(name, control).await;
            }
        }
    }

    /// The container states it has no record of this load: nothing is queued
    /// for it and nothing is running. Only now is a resend safe (I1).
    async fn handle_lost_load(&mut self, name: String, control: &mut ControlChannel) {
        let attempts = self
            .composable_nodes
            .get(&name)
            .and_then(|e| e.tracking.as_ref())
            .map(|t| t.attempts)
            .unwrap_or(1);

        if attempts >= self.timings.max_load_attempts {
            let error = format!(
                "lost: the container has no record of this load, and {attempts} attempt(s) is \
                 the configured maximum"
            );
            warn!("{}: '{}' {}", self.name(), name, error);
            if let Some(entry) = self.composable_nodes.get_mut(&name) {
                entry.state = ComposableState::Failed {
                    error: error.clone(),
                };
                entry.load_started_at = None;
                entry.tracking = None;
            }
            emit(self.state_tx(), StateEvent::LoadFailed { name, error }).await;
            return;
        }

        warn!(
            "{}: '{}' was lost — the container has no record of it; resending (attempt {} of {})",
            self.name(),
            name,
            attempts + 1,
            self.timings.max_load_attempts
        );
        if let Some(entry) = self.composable_nodes.get_mut(&name) {
            entry.unique_id = None;
        }
        self.send_load_over_socket(&name, control).await;
    }

    /// A `LoadFailed` carrying `cancelled: true` — the container confirms that
    /// nothing is running for the id. This is the second half of the two-step
    /// that makes a retry safe (I4).
    pub(super) async fn on_cancel_confirmed(
        &mut self,
        name: String,
        error: String,
        control: &mut ControlChannel,
    ) {
        let intent = self
            .composable_nodes
            .get(&name)
            .and_then(|e| e.tracking.as_ref())
            .and_then(|t| t.cancel_intent);

        // The attempt budget applies to a restart too. Without this check a
        // node that stalls, is cancelled, and stalls again would cycle
        // forever: each iteration is individually safe (nothing is ever
        // double-loaded) and collectively useless.
        let attempts = self
            .composable_nodes
            .get(&name)
            .and_then(|e| e.tracking.as_ref())
            .map(|t| t.attempts)
            .unwrap_or(1);
        let restart_allowed = attempts < self.timings.max_load_attempts;

        match intent {
            Some(CancelIntent::Restart) if !restart_allowed => {
                let error = format!(
                    "stalled, and {attempts} attempt(s) is the configured maximum: {error}"
                );
                warn!("{}: '{}' {}", self.name(), name, error);
                if let Some(entry) = self.composable_nodes.get_mut(&name) {
                    entry.state = ComposableState::Failed {
                        error: error.clone(),
                    };
                    entry.load_started_at = None;
                    entry.tracking = None;
                }
                emit(self.state_tx(), StateEvent::LoadFailed { name, error }).await;
            }
            Some(CancelIntent::Restart) => {
                info!(
                    "{}: '{}' cancelled and confirmed gone — reloading (attempt {} of {})",
                    self.name(),
                    name,
                    attempts + 1,
                    self.timings.max_load_attempts
                );
                if let Some(entry) = self.composable_nodes.get_mut(&name) {
                    entry.unique_id = None;
                    if let Some(t) = entry.tracking.as_mut() {
                        t.cancel_intent = None;
                    }
                }
                self.send_load_over_socket(&name, control).await;
            }
            _ => {
                warn!("{}: '{}' cancelled: {}", self.name(), name, error);
                if let Some(entry) = self.composable_nodes.get_mut(&name) {
                    entry.state = ComposableState::Failed {
                        error: error.clone(),
                    };
                    entry.load_started_at = None;
                    entry.tracking = None;
                }
                emit(self.state_tx(), StateEvent::LoadFailed { name, error }).await;
            }
        }
    }

    /// A composable that crashed after loading. The id is confirmed gone — the
    /// container reaped the child and erased it — so this is the one retry
    /// that needs no confirmation step.
    pub(super) fn schedule_crash_reload(&mut self, name: &str, config: &ActorConfig) {
        if self.timings.composable_respawn != ComposableRespawn::OnCrash {
            return;
        }
        let container = self.name().to_string();
        let max = config.max_respawn_attempts;
        let delay = config.respawn_delay;
        let Some(entry) = self.composable_nodes.get_mut(name) else {
            return;
        };
        entry.crash_count += 1;
        let crash_count = entry.crash_count;
        if let Some(max) = max
            && crash_count > max
        {
            warn!(
                "{}: '{}' crashed {} times; not reloading again",
                container, name, crash_count
            );
            return;
        }
        entry.retry_after = Some(Instant::now() + Duration::from_secs_f64(delay));
        entry.tracking = None;
        info!(
            "{}: '{}' crashed; reloading in {:.1}s (attempt {})",
            container, name, delay, crash_count
        );
    }

    /// Stall detection. Evidence, never a clock alone — and disabled unless an
    /// operator has said what a stall means on their hardware.
    async fn check_for_stalls(&mut self, control: &mut ControlChannel) {
        if self.timings.stall_after.is_zero() {
            return;
        }
        let now = Instant::now();
        let threshold = self.timings.stall_cpu_threshold_pct;
        let stall_after = self.timings.stall_after;

        let stalled: Vec<(String, i32, u64, u64)> = self
            .composable_nodes
            .iter()
            .filter_map(|(name, entry)| {
                let ComposableState::Loading { started_at } = &entry.state else {
                    return None;
                };
                let tracking = entry.tracking.as_ref()?;
                if tracking.cancel_intent.is_some()
                    || tracking.phase != Some(LoadPhase::Constructing)
                    || tracking.pid <= 0
                {
                    return None;
                }
                let elapsed = started_at.elapsed();
                if elapsed < stall_after {
                    return None;
                }
                // Two samples are needed for a delta; one sample says nothing
                // about progress.
                let sampled_at = tracking.cpu_sampled_at?;
                let window = now.duration_since(sampled_at);
                let _ = window;
                let cpu_delta = tracking.cpu_ms.saturating_sub(tracking.prev_cpu_ms);
                let busy_pct = if elapsed.as_millis() == 0 {
                    100.0
                } else {
                    (cpu_delta as f64 / stall_after.as_millis() as f64) * 100.0
                };
                if busy_pct >= threshold {
                    return None;
                }
                Some((name.clone(), tracking.pid, elapsed.as_secs(), cpu_delta))
            })
            .collect();

        for (name, pid, elapsed_secs, cpu_delta) in stalled {
            let already_reported = self
                .composable_nodes
                .get(&name)
                .and_then(|e| e.tracking.as_ref())
                .map(|t| t.stall_reported)
                .unwrap_or(false);

            match self.timings.stall_action {
                StallAction::Report => {
                    if already_reported {
                        continue;
                    }
                    warn!(
                        "{}: '{}' has been constructing for {}s (pid {}) and used {}ms of CPU \
                         since the last report — it may be stalled. Nothing has been done about \
                         it: set composable_node_loading.stall_action to act.",
                        self.name(),
                        name,
                        elapsed_secs,
                        pid,
                        cpu_delta
                    );
                    if let Some(t) = self.tracking_mut(&name) {
                        t.stall_reported = true;
                    }
                }
                StallAction::Fail | StallAction::Restart => {
                    if already_reported {
                        continue; // a cancel is already out
                    }
                    let intent = if self.timings.stall_action == StallAction::Fail {
                        CancelIntent::Fail
                    } else {
                        CancelIntent::Restart
                    };
                    let unique_id = self.composable_nodes.get(&name).and_then(|e| e.unique_id);
                    let Some(unique_id) = unique_id else {
                        // No id: the container never accepted it, so there is
                        // nothing for it to cancel. The ack path owns this
                        // case.
                        continue;
                    };
                    warn!(
                        "{}: '{}' stalled at {}s (pid {}, {}ms CPU since the last report) — \
                         cancelling ({:?})",
                        self.name(),
                        name,
                        elapsed_secs,
                        pid,
                        cpu_delta,
                        intent
                    );
                    control.send_cancel(unique_id, "stalled: no CPU progress while constructing");
                    if let Some(t) = self.tracking_mut(&name) {
                        t.cancel_intent = Some(intent);
                        t.stall_reported = true;
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::member_actor::container_actor::supervisor::LoadTracking;

    const ACK: Duration = Duration::from_secs(5);
    const REPORT: Duration = Duration::from_secs(45);
    const PROBE: Duration = Duration::from_secs(15);

    fn tracking_at(phase: Option<LoadPhase>, quiet: Duration) -> LoadTracking {
        let mut t = LoadTracking::new(7, 1);
        t.phase = phase;
        t.last_report = Instant::now() - quiet;
        t
    }

    /// An unacknowledged load is the ONE case where nothing can have been
    /// forked, so it is the only one the ack timer may act on — and even then
    /// it only asks.
    #[test]
    fn unacknowledged_load_is_queried_by_seq() {
        let t = tracking_at(None, Duration::from_secs(6));
        assert_eq!(
            decide(&t, Instant::now(), ACK, REPORT, PROBE),
            SweepAction::QueryBySeq(7)
        );
    }

    #[test]
    fn a_fresh_load_is_left_alone() {
        let t = tracking_at(None, Duration::from_millis(200));
        assert_eq!(
            decide(&t, Instant::now(), ACK, REPORT, PROBE),
            SweepAction::Wait
        );
    }

    /// A constructor that reports every 15 s must never be probed for being
    /// slow — only for being SILENT, which is a different thing.
    #[test]
    fn a_reporting_constructor_is_never_probed() {
        let mut t = tracking_at(Some(LoadPhase::Constructing), Duration::from_secs(20));
        t.unique_id = Some(3);
        assert_eq!(
            decide(&t, Instant::now(), ACK, REPORT, PROBE),
            SweepAction::Wait
        );
    }

    #[test]
    fn silence_after_acceptance_is_queried_by_id() {
        let mut t = tracking_at(Some(LoadPhase::Constructing), Duration::from_secs(46));
        t.unique_id = Some(3);
        assert_eq!(
            decide(&t, Instant::now(), ACK, REPORT, PROBE),
            SweepAction::QueryById(3)
        );
    }

    /// While a query is outstanding the answer is what resolves it; asking
    /// again before `probe_interval` would just race it.
    #[test]
    fn an_outstanding_probe_suppresses_further_action() {
        let mut t = tracking_at(Some(LoadPhase::Constructing), Duration::from_secs(600));
        t.unique_id = Some(3);
        t.probe_sent_at = Some(Instant::now());
        assert_eq!(
            decide(&t, Instant::now(), ACK, REPORT, PROBE),
            SweepAction::Wait
        );

        t.probe_sent_at = Some(Instant::now() - Duration::from_secs(16));
        assert_eq!(
            decide(&t, Instant::now(), ACK, REPORT, PROBE),
            SweepAction::Reprobe {
                seq: None,
                unique_id: Some(3),
                unanswered: 0
            }
        );
    }

    /// A cancel is already the question; nothing else may be asked until it is
    /// answered, or the resend decision would race the confirmation.
    #[test]
    fn a_pending_cancel_suppresses_probing() {
        let mut t = tracking_at(Some(LoadPhase::Constructing), Duration::from_secs(600));
        t.unique_id = Some(3);
        t.cancel_intent = Some(CancelIntent::Restart);
        assert_eq!(
            decide(&t, Instant::now(), ACK, REPORT, PROBE),
            SweepAction::Wait
        );
    }
}
