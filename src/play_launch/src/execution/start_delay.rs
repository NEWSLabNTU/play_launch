//! The launch `<timer>` start delay, at spawn time.
//!
//! A `<timer period="6.0">` in a launch file says its body starts six seconds
//! after the launch does. The parser resolves that into
//! `NodeRecord::start_delay_secs` (accumulated across nested timers), and it
//! reaches the runtime as `NodeInstance::start_delay_secs` on the SystemModel
//! — which is what `up` spawns from. This module is where the number finally
//! becomes a wait.
//!
//! # Why this is not a knob on the startup governor
//!
//! [`crate::execution::startup_governor`] already decides WHEN a member may
//! spawn, and it is tempting to express a start delay as one more gate there.
//! It is a different kind of statement, and folding the two would break both:
//!
//! - The governor's gates are POLICY, derived from the machine (memory floor,
//!   runnable ceiling, concurrency limit) and tunable by whoever runs the
//!   launch. A `<timer>` is the launch file's SEMANTICS: nav2's bringup
//!   staggers its lifecycle managers at 3 s, 6 s and 11 s because the nodes
//!   they manage must exist first, and no operator preference may shorten
//!   that. A configuration knob that can turn the gate off must never be able
//!   to turn this off.
//! - Every governor gate has a deliberate bypass — `max_gate_wait` admits a
//!   blocked member anyway rather than deadlock a launch (see that module's
//!   "Deadlock is not an option"). Bypassing a `<timer>` is not a degraded
//!   start, it is the wrong start.
//!
//! So the wait composes with the startup machinery instead of joining it. The
//! actor waits out its deadline FIRST and asks for admission SECOND, which
//! means a delayed member holds no concurrency permit while it waits, the
//! memory and load gates still apply to it when its time comes, and a member
//! that is both delayed and assigned a later `startup.order` stage starts when
//! both conditions are met rather than whichever the code happened to check.
//!
//! # Why an absolute instant
//!
//! ROS measures a `TimerAction` from the start of the launch. Carrying a
//! duration and sleeping it inside the actor would instead measure from
//! whenever that actor reached `Pending` — different per member, and biased by
//! exactly the startup bookkeeping the delay is supposed to be independent of.
//! [`deadline`] converts the delay once, against one epoch for the whole
//! launch, so members sharing a `<timer>` share an instant.
//!
//! The other half of that choice is respawn. A respawn re-enters the same
//! `Pending` path; a deadline in the past costs nothing there, where a
//! duration would make every restart of a delayed node wait the delay again —
//! a `<timer>` delays a first start, `respawn_delay` spaces the rest.

use std::time::Duration;

use tokio::{sync::watch, time::Instant};
use tracing::info;

/// Convert a launch-declared start delay into the instant before which a
/// member must not spawn, or `None` when it may spawn at once.
///
/// `epoch` is the launch's own start — ONE instant shared by every member of
/// the launch (see the module docs).
///
/// Nonsense is dropped rather than propagated: a non-finite or non-positive
/// delay yields `None`. That is not defensive dressing —
/// `Duration::from_secs_f64` PANICS on a negative or NaN input, and this
/// number reaches us from an arbitrary launch file through a `$(var ...)`
/// substitution, so the value has to be treated as untrusted at the one place
/// it is converted.
pub fn deadline(epoch: Instant, start_delay_secs: Option<f64>) -> Option<Instant> {
    let secs = start_delay_secs?;
    if !secs.is_finite() || secs <= 0.0 {
        return None;
    }
    Some(epoch + Duration::from_secs_f64(secs))
}

/// Outcome of [`wait_for_start_delay`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartDelay {
    /// The member may proceed to ask for a startup slot.
    Elapsed,
    /// The launch is shutting down; the member must not spawn.
    ShutDown,
}

/// Block until `start_after` has passed, or until the launch shuts down.
///
/// Returns [`StartDelay::Elapsed`] immediately when `start_after` is `None` or
/// already in the past — the overwhelmingly common case, and the respawn case.
pub async fn wait_for_start_delay(
    log_name: &str,
    start_after: Option<Instant>,
    shutdown_rx: &mut watch::Receiver<bool>,
) -> StartDelay {
    let Some(at) = start_after else {
        return StartDelay::Elapsed;
    };
    let remaining = at.saturating_duration_since(Instant::now());
    if remaining.is_zero() {
        return StartDelay::Elapsed;
    }

    // At `info`, not `debug`: a node that is not there yet is the single most
    // confusing thing a staged launch does, and "it is waiting, here is how
    // long" is the answer to the question the operator is about to ask.
    info!(
        "[{}] Waiting {:.1}s before starting (launch <timer>)",
        log_name,
        remaining.as_secs_f64()
    );

    // A shutdown during the wait must not be answered by starting the process
    // anyway once the timer fires: the 20 s member of a launch cancelled at
    // 2 s would otherwise spawn eighteen seconds into the teardown.
    tokio::select! {
        _ = tokio::time::sleep_until(at) => StartDelay::Elapsed,
        _ = shutdown_rx.changed() => {
            if *shutdown_rx.borrow() {
                StartDelay::ShutDown
            } else {
                // Spurious change (the channel also carries `false`): finish
                // the wait rather than treat it as a start signal.
                tokio::time::sleep_until(at).await;
                StartDelay::Elapsed
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test(start_paused = true)]
    async fn no_delay_declared_is_no_deadline() {
        let epoch = Instant::now();
        assert_eq!(deadline(epoch, None), None);
    }

    /// The launch file is not a trusted source of `f64`s, and
    /// `Duration::from_secs_f64` panics on two of these three.
    #[tokio::test(start_paused = true)]
    async fn nonsense_delays_do_not_panic_and_do_not_delay() {
        let epoch = Instant::now();
        assert_eq!(deadline(epoch, Some(-1.0)), None);
        assert_eq!(deadline(epoch, Some(f64::NAN)), None);
        assert_eq!(deadline(epoch, Some(0.0)), None);
    }

    /// Two members of the same `<timer>` get the SAME instant, whatever
    /// happens between their two constructions — the property that makes the
    /// delay mean "after the launch started" rather than "after this actor
    /// was built".
    #[tokio::test(start_paused = true)]
    async fn one_epoch_gives_members_of_one_timer_the_same_deadline() {
        let epoch = Instant::now();
        let a = deadline(epoch, Some(6.0)).unwrap();
        tokio::time::sleep(Duration::from_millis(20)).await;
        let b = deadline(epoch, Some(6.0)).unwrap();
        assert_eq!(a, b);
        assert_eq!(a - epoch, Duration::from_secs(6));
    }

    #[tokio::test(start_paused = true)]
    async fn an_undelayed_member_does_not_wait() {
        let (_tx, mut rx) = watch::channel(false);
        let before = Instant::now();
        assert_eq!(
            wait_for_start_delay("node", None, &mut rx).await,
            StartDelay::Elapsed
        );
        assert_eq!(Instant::now(), before, "an undelayed member slept");
    }

    #[tokio::test(start_paused = true)]
    async fn a_delayed_member_waits_exactly_its_delay() {
        let (_tx, mut rx) = watch::channel(false);
        let epoch = Instant::now();
        let at = deadline(epoch, Some(6.0));
        assert_eq!(
            wait_for_start_delay("node", at, &mut rx).await,
            StartDelay::Elapsed
        );
        assert_eq!(Instant::now().duration_since(epoch), Duration::from_secs(6));
    }

    /// The respawn case: the deadline is absolute, so a member that already
    /// waited it out once does not wait again.
    #[tokio::test(start_paused = true)]
    async fn an_elapsed_deadline_is_free() {
        let (_tx, mut rx) = watch::channel(false);
        let at = deadline(Instant::now(), Some(6.0));
        assert_eq!(
            wait_for_start_delay("node", at, &mut rx).await,
            StartDelay::Elapsed
        );

        let after_first_spawn = Instant::now();
        assert_eq!(
            wait_for_start_delay("node", at, &mut rx).await,
            StartDelay::Elapsed
        );
        assert_eq!(
            Instant::now(),
            after_first_spawn,
            "a respawn waited the start delay a second time"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn shutdown_during_the_wait_cancels_the_start() {
        let (tx, mut rx) = watch::channel(false);
        let epoch = Instant::now();
        let at = deadline(epoch, Some(20.0));

        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_secs(2)).await;
            tx.send(true).unwrap();
        });

        assert_eq!(
            wait_for_start_delay("node", at, &mut rx).await,
            StartDelay::ShutDown
        );
        assert_eq!(
            Instant::now().duration_since(epoch),
            Duration::from_secs(2),
            "the wait outlived the shutdown that cancelled it"
        );
    }
}
