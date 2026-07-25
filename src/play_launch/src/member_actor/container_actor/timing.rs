//! Every Duration the container subsystem uses, in one place (phase-51.4).
//!
//! Phase-52.2: the load-path values are RUNTIME KNOBS — `LoadTimings` is
//! built from `composable_node_loading` config (+ CLI overrides) in
//! `replay` and threaded through the builder to every container actor.
//! Defaults equal the former consts. The remaining consts are internal
//! cadences with no tuning story.

use crate::cli::config::ComposableNodeLoadingSettings;
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
    }

    #[test]
    fn from_settings_roundtrip() {
        let mut s = ComposableNodeLoadingSettings::default();
        s.load_total_budget_secs = 1200;
        s.load_node_attempts = 0; // clamped to 1
        let t = LoadTimings::from_settings(&s);
        assert_eq!(t.total_budget, Duration::from_secs(1200));
        assert_eq!(t.max_attempts, 1);
        assert_eq!(t.service_call_timeout, Duration::from_secs(30));
    }
}
