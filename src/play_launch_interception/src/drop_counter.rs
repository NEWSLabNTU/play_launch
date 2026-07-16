//! Ring-overflow drop counters (Phase 42.0).
//!
//! Every producer call site in `plugins/*.rs` previously did
//! `let _ = producer.push(&event)`, silently discarding the `Err(Full)`
//! returned when the SPSC ring is full (see `spsc_shm::Producer::push`).
//! Under sustained overflow (e.g. a bursty high-rate publisher hitting a
//! full ring during a consumer scheduling hiccup) this meant data loss
//! with zero signal — flagged as capability 5 ("NOT READY — no
//! instrumentation") in `.superpowers/sdd/p42-infra-readiness.md`.
//!
//! This module gives every push site two things, in order of
//! reliability:
//!
//! 1. **Process-exit stderr log** (always available, unconditionally
//!    accurate for *this* process): an `atexit` hook prints the total
//!    drop count once, the first time a producer is opened.
//! 2. **Best-effort in-band report** (`RingOverflowReport` event,
//!    opportunistic): periodically, when a push succeeds and the drop
//!    count has changed since the last report, we try to push one more
//!    event carrying the cumulative count. This can itself be dropped
//!    if the ring stays full, so it's a lower bound, not a guarantee —
//!    but it's the only way the *consumer* (play_launch, a different
//!    process) can learn about drops at all, short of scraping this
//!    process's stderr.
//!
//! A single process-wide counter (not per-plugin) is used deliberately:
//! all ring-buffer plugins (`FrontierPlugin`, `StatsPlugin`,
//! `QosNegotiationPlugin`, `DdsEventsPlugin`, `TopicNamesPlugin`) share
//! one `Producer` per process (see `try_open_producer` in `lib.rs`), so
//! a drop in one is indistinguishable from a drop in another from the
//! ring's point of view — they're all just "a slot wasn't available".
//! Per-plugin attribution would require tagging drops with a plugin id,
//! which isn't worth the complexity for a monitoring-only counter.

use std::sync::atomic::{AtomicU64, Ordering};

use crate::event::InterceptionEvent;
use spsc_shm::Producer;

/// Total `Err(Full)` results seen across all producer push call sites
/// in this process.
static DROPPED: AtomicU64 = AtomicU64::new(0);

/// Value of `DROPPED` as of the last successful `RingOverflowReport`
/// push. Used to avoid re-reporting an unchanged count.
static LAST_REPORTED: AtomicU64 = AtomicU64::new(0);

/// Push-site call counter, used to rate-limit how often we check
/// whether a report is due (checking on every single push would double
/// ring traffic for no benefit — drop counts change slowly relative to
/// event volume).
static CALLS: AtomicU64 = AtomicU64::new(0);

/// Check the report-due condition every N calls to `push_or_count`.
/// Chosen to match the existing periodic-check cadence used elsewhere
/// in this codebase (e.g. `RuleEngine::check_rate_hierarchy`'s
/// every-1024-publishes cadence) — frequent enough that a sustained
/// overflow is visible within a fraction of a second at typical
/// pub/take rates, rare enough not to add measurable overhead.
const REPORT_CHECK_INTERVAL: u64 = 256;

static ATEXIT_ONCE: std::sync::Once = std::sync::Once::new();

/// Push `event` through `producer`, counting (not just discarding) an
/// `Err(Full)` result. Every call site that used to do
/// `let _ = producer.push(&event)` should call this instead.
///
/// Also opportunistically attempts to push a `RingOverflowReport` event
/// (rate-limited to every `REPORT_CHECK_INTERVAL` calls) so the
/// play_launch consumer can learn about drops without needing this
/// process's stderr.
pub(crate) fn push_or_count(producer: &mut Producer<InterceptionEvent>, event: &InterceptionEvent) {
    if producer.push(event).is_err() {
        DROPPED.fetch_add(1, Ordering::Relaxed);
    }
    let calls = CALLS.fetch_add(1, Ordering::Relaxed) + 1;
    if calls.is_multiple_of(REPORT_CHECK_INTERVAL) {
        maybe_report(producer);
    }
}

/// If the drop count has changed since the last successful report,
/// attempt to push a `RingOverflowReport` event. Best-effort: if this
/// push also hits `Err(Full)`, we simply try again at the next
/// `REPORT_CHECK_INTERVAL` boundary — no recursion into
/// `push_or_count` (that would double-count this push as a plugin
/// event drop, which it isn't).
fn maybe_report(producer: &mut Producer<InterceptionEvent>) {
    let dropped = DROPPED.load(Ordering::Relaxed);
    if dropped != LAST_REPORTED.load(Ordering::Relaxed) {
        let event = InterceptionEvent::ring_overflow_report(dropped);
        if producer.push(&event).is_ok() {
            LAST_REPORTED.store(dropped, Ordering::Relaxed);
        }
    }
}

/// Total drops observed so far in this process (for the atexit log
/// line and for tests).
pub(crate) fn dropped_count() -> u64 {
    DROPPED.load(Ordering::Relaxed)
}

/// Register the process-exit stderr report, once per process. Safe to
/// call multiple times (e.g. once per plugin construction) — only the
/// first call actually registers the `atexit` hook.
pub(crate) fn install_atexit_report() {
    ATEXIT_ONCE.call_once(|| unsafe {
        libc::atexit(report_drops_at_exit);
    });
}

/// `atexit` callback: logs the final drop count to stderr if nonzero.
/// Silent when zero so well-behaved runs produce no extra noise.
extern "C" fn report_drops_at_exit() {
    let dropped = dropped_count();
    if dropped > 0 {
        eprintln!(
            "play_launch_interception: {dropped} event(s) dropped due to SPSC ring buffer \
             overflow in this process (frontier/stats/qos/dds/topic-name events share one ring); \
             increase `ring_capacity` in the interception config if this is a large fraction of \
             total event volume"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counts_drops_on_full_ring() {
        // Capacity 1: first push succeeds, ring is then full for a
        // consumer-less producer since nothing ever pops.
        let (shm_fd, event_fd) = spsc_shm::create::<InterceptionEvent>(1).unwrap();
        let mut producer = unsafe { Producer::<InterceptionEvent>::from_raw_fd(shm_fd) }.unwrap();

        let before = dropped_count();
        let event = InterceptionEvent::ring_overflow_report(0);
        // Ring capacity 1: this may or may not succeed depending on
        // implementation slack, so push enough events to guarantee at
        // least one Full.
        for _ in 0..8 {
            push_or_count(&mut producer, &event);
        }
        assert!(
            dropped_count() > before,
            "expected at least one drop after overfilling a capacity-1 ring"
        );

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }
}
