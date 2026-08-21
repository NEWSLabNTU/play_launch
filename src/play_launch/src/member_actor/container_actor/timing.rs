//! Every Duration the container subsystem uses, in one place (phase-51.4).
//!
//! Phase-52.2: the load-path values are RUNTIME KNOBS — `LoadTimings` is
//! built from `composable_node_loading` config (+ CLI overrides) in
//! `replay` and threaded through the builder to every container actor.
//! Defaults equal the former consts. The remaining consts are internal
//! cadences with no tuning story.

use crate::cli::config::{ComposableNodeLoadingSettings, ComposableRespawn, StallAction};
use tokio::time::Duration;

/// Interval for checking composable node loading timeouts.
pub(super) const LOADING_CHECK_INTERVAL: Duration = Duration::from_secs(5);

/// Budget for the post-timeout ListNodes verification call.
pub(super) const LIST_VERIFY_TIMEOUT: Duration = Duration::from_secs(10);

/// Interval for logging service-not-ready status during service discovery.
pub(super) const SERVICE_NOT_READY_LOG_INTERVAL: Duration = Duration::from_secs(2);

/// Poll interval while waiting for a container's LoadNode service to appear.
pub(super) const SERVICE_POLL_INTERVAL: Duration = Duration::from_millis(100);

/// Tunable timings of the LoadNode path (phase-52.2, issue 0004).
#[derive(Debug, Clone, Copy)]
pub struct LoadTimings {
    /// Timeout for the FIRST LoadNode service call.
    pub service_call_timeout: Duration,
    /// Follow-up LoadNode attempts get a longer budget: a busy
    /// single-threaded container executes composable ctors serially, so a
    /// later load's response legitimately arrives after Σ(preceding ctor
    /// times) — seen live with Autoware's adapi container.
    pub retry_timeout: Duration,
    /// Total LoadNode attempts (first + retries) before giving up.
    pub max_attempts: usize,
    /// While a blocking-mode container is BUSY (a composable ctor can hold
    /// the executor for minutes — e.g. TensorRT engine loads), we keep
    /// polling ListNodes rather than resending LoadNode (NOT idempotent —
    /// a resend into a busy container double-loads once the ctor
    /// finishes). This caps the total wait per composable before declaring
    /// the container unresponsive.
    pub total_budget: Duration,
    /// Pause between ListNodes verification polls while the container is busy.
    pub verify_poll_interval: Duration,
    /// How long a composable may sit in Loading with a known unique_id
    /// before the stuck-Loading promoter treats the service response as
    /// authoritative (handles DDS ComponentEvent loss).
    pub loading_event_timeout: Duration,
    /// Brief warmup delay after service readiness before issuing LoadNode
    /// calls (service registration can precede executor spin-up).
    pub warmup: Duration,
    /// Phase 64: use the private control socket to `play_launch_container`
    /// instead of the LoadNode service, when the container is ours.
    pub control_socket: bool,
    /// How long to wait for the container's control-channel hello before
    /// falling back to the LoadNode service.
    pub control_hello_timeout: Duration,
    /// Phase 64 W2 — the coordination between failure detection, retry and
    /// constructors that legitimately take minutes
    /// (`docs/design/composable-load-lifecycle.md`). Every one of these causes
    /// a QUESTION to the container, never a verdict of its own; only
    /// `stall_*` can act, and only when opted in.
    pub ack_timeout: Duration,
    /// Silence from the container about a load, before asking.
    pub report_timeout: Duration,
    /// Re-ask cadence when a query itself went unanswered.
    pub probe_interval: Duration,
    /// Total attempts per composable, first included. Reachable only after a
    /// CONFIRMED absence, never on a clock.
    pub max_load_attempts: usize,
    /// Declare a constructor stalled after this long. Zero = never.
    pub stall_after: Duration,
    /// CPU share of one core below which a constructor is "not progressing".
    pub stall_cpu_threshold_pct: f64,
    pub stall_action: StallAction,
    pub composable_respawn: ComposableRespawn,
}

impl Default for LoadTimings {
    fn default() -> Self {
        Self {
            service_call_timeout: Duration::from_secs(30),
            retry_timeout: Duration::from_secs(60),
            max_attempts: 3,
            total_budget: Duration::from_secs(600),
            verify_poll_interval: Duration::from_secs(5),
            loading_event_timeout: Duration::from_secs(10),
            warmup: Duration::from_millis(200),
            control_socket: true,
            control_hello_timeout: Duration::from_secs(10),
            ack_timeout: Duration::from_secs(5),
            report_timeout: Duration::from_secs(45),
            probe_interval: Duration::from_secs(15),
            max_load_attempts: 2,
            stall_after: Duration::ZERO,
            stall_cpu_threshold_pct: 1.0,
            stall_action: StallAction::Report,
            composable_respawn: ComposableRespawn::Off,
        }
    }
}

impl LoadTimings {
    /// Build from the `composable_node_loading` config section.
    pub fn from_settings(s: &ComposableNodeLoadingSettings) -> Self {
        Self {
            service_call_timeout: Duration::from_millis(s.load_node_timeout_millis),
            retry_timeout: Duration::from_millis(s.load_retry_timeout_millis),
            max_attempts: s.load_node_attempts.max(1),
            total_budget: Duration::from_secs(s.load_total_budget_secs),
            verify_poll_interval: Duration::from_secs(s.load_verify_poll_interval_secs.max(1)),
            loading_event_timeout: Duration::from_secs(s.loading_event_timeout_secs),
            warmup: Duration::from_millis(s.post_service_ready_warmup_ms),
            control_socket: s.control_socket,
            control_hello_timeout: Duration::from_millis(s.control_hello_timeout_ms),
            ack_timeout: Duration::from_millis(s.ack_timeout_ms),
            report_timeout: Duration::from_secs(s.report_timeout_secs.max(1)),
            probe_interval: Duration::from_secs(s.probe_interval_secs.max(1)),
            max_load_attempts: s.max_load_attempts.max(1),
            stall_after: Duration::from_secs(s.stall_after_secs),
            stall_cpu_threshold_pct: s.stall_cpu_threshold_pct,
            stall_action: s.stall_action,
            composable_respawn: s.composable_respawn,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_former_consts() {
        let t = LoadTimings::default();
        assert_eq!(t.service_call_timeout, Duration::from_secs(30));
        assert_eq!(t.retry_timeout, Duration::from_secs(60));
        assert_eq!(t.max_attempts, 3);
        assert_eq!(t.total_budget, Duration::from_secs(600));
        assert_eq!(t.verify_poll_interval, Duration::from_secs(5));
        assert_eq!(t.loading_event_timeout, Duration::from_secs(10));
        assert_eq!(t.warmup, Duration::from_millis(200));
        assert!(t.control_socket);
        assert_eq!(t.control_hello_timeout, Duration::from_secs(10));
    }

    /// W2's defaults are all "wait and ask": nothing here may fail a load on
    /// its own, and the stall check is off until an operator turns it on.
    #[test]
    fn w2_defaults_are_passive() {
        let t = LoadTimings::default();
        assert_eq!(t.ack_timeout, Duration::from_secs(5));
        assert_eq!(t.report_timeout, Duration::from_secs(45));
        assert_eq!(t.probe_interval, Duration::from_secs(15));
        assert_eq!(
            t.max_load_attempts, 2,
            "one resend, after confirmation only"
        );
        assert_eq!(t.stall_after, Duration::ZERO, "stall detection is opt-in");
        assert_eq!(t.stall_action, StallAction::Report);
        assert_eq!(t.composable_respawn, ComposableRespawn::Off);
    }

    /// `max_load_attempts: 0` would mean "never load anything"; read it as the
    /// one attempt that must always be allowed.
    #[test]
    fn attempts_are_clamped_to_at_least_one() {
        let s = ComposableNodeLoadingSettings {
            max_load_attempts: 0,
            ..Default::default()
        };
        assert_eq!(LoadTimings::from_settings(&s).max_load_attempts, 1);
    }

    /// Phase 64: the socket is on by default and turns off from config alone
    /// — no CLI flag, no rebuild.
    #[test]
    fn control_socket_follows_settings() {
        let s = ComposableNodeLoadingSettings {
            control_socket: false,
            control_hello_timeout_ms: 2500,
            ..Default::default()
        };
        let t = LoadTimings::from_settings(&s);
        assert!(!t.control_socket);
        assert_eq!(t.control_hello_timeout, Duration::from_millis(2500));
    }

    #[test]
    fn from_settings_roundtrip() {
        let s = ComposableNodeLoadingSettings {
            load_total_budget_secs: 1200,
            load_node_attempts: 0, // clamped to 1
            ..Default::default()
        };
        let t = LoadTimings::from_settings(&s);
        assert_eq!(t.total_budget, Duration::from_secs(1200));
        assert_eq!(t.max_attempts, 1);
        assert_eq!(t.service_call_timeout, Duration::from_secs(30));
    }
}
