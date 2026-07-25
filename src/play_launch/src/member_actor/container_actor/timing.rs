//! Every Duration the container subsystem uses, in one place (phase-51.4).
//!
//! Still consts — the config knobs land in phase-52 (issue 0004); this
//! file is where they will be populated from `Config` when they do.

use tokio::time::Duration;

/// Timeout for LoadNode/UnloadNode service calls.
pub(super) const SERVICE_CALL_TIMEOUT: Duration = Duration::from_secs(30);

/// Follow-up LoadNode attempts get a longer budget: a busy single-threaded
/// container executes composable ctors serially, so a later load's response
/// legitimately arrives after Σ(preceding ctor times) — seen live with
/// Autoware's adapi container (autoware_state/heartbeat) behind slow loads.
pub(super) const LOAD_RETRY_TIMEOUT: Duration = Duration::from_secs(60);

/// Total LoadNode attempts (first + retries) before giving up.
pub(super) const LOAD_MAX_ATTEMPTS: usize = 3;

/// Budget for the post-timeout ListNodes verification call.
pub(super) const LIST_VERIFY_TIMEOUT: Duration = Duration::from_secs(10);

/// While a blocking-mode container is BUSY (a composable ctor can hold the
/// executor for minutes — e.g. TensorRT engine loads), we keep polling
/// ListNodes rather than resending LoadNode (NOT idempotent — a resend into
/// a busy container double-loads once the ctor finishes). This caps the
/// total wait per composable before declaring the container unresponsive.
pub(super) const LOAD_TOTAL_BUDGET: Duration = Duration::from_secs(600);

/// Pause between ListNodes verification polls while the container is busy.
pub(super) const LOAD_VERIFY_POLL_INTERVAL: Duration = Duration::from_secs(5);

/// Interval for checking whether nodes stuck in Loading state should be
/// promoted to Loaded (handles DDS event loss).
pub(super) const LOADING_TIMEOUT: Duration = Duration::from_secs(10);

/// Interval for checking composable node loading timeouts.
pub(super) const LOADING_CHECK_INTERVAL: Duration = Duration::from_secs(5);

/// Brief warmup delay after service readiness before issuing LoadNode calls.
pub(super) const POST_SERVICE_READY_WARMUP: Duration = Duration::from_millis(200);

/// Interval for logging service-not-ready status during service discovery.
pub(super) const SERVICE_NOT_READY_LOG_INTERVAL: Duration = Duration::from_secs(2);

/// Poll interval while waiting for a container's LoadNode service to appear.
pub(super) const SERVICE_POLL_INTERVAL: Duration = Duration::from_millis(100);
