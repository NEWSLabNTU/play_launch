//! Crash respawn policy for composable nodes.
//!
//! A composable whose process crashed after loading is the one retry that
//! needs no confirmation step: the container reaped the child and erased its
//! id, so reloading it cannot double-load. What it does need is a bound —
//! a component that dies in its constructor would otherwise be reloaded
//! forever. This module is that bound, kept free of the actor so it can be
//! tested with plain `Instant`s.
//!
//! Where the semantics come from: launch_ros' `ComposableNode` has no
//! `respawn`/`respawn_delay` of its own (nor does the parser carry one), so a
//! composable inherits its CONTAINER's. `composable_respawn: inherit` reloads
//! only under a `respawn="true"` container; `on-crash` reloads regardless.
//! Either way the first reload waits the container's `respawn_delay`.

use super::timing::LoadTimings;
use crate::cli::config::ComposableRespawn;
use std::{
    collections::VecDeque,
    time::{Duration, Instant},
};

/// Floor for the backoff base when the launch file's delay is zero: an
/// immediate first reload honours `respawn_delay="0"`, but a SECOND crash
/// inside the window means the immediate reload did not help.
const MIN_BACKOFF_BASE: Duration = Duration::from_secs(1);

/// Policy parameters, from `composable_node_loading` (+ CLI).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct RespawnPolicy {
    pub mode: ComposableRespawn,
    /// Reloads allowed within `window` (at least 1).
    pub max_restarts: u32,
    pub window: Duration,
    pub max_backoff: Duration,
}

/// Per-composable crash bookkeeping.
#[derive(Debug, Default, Clone)]
pub(super) struct CrashHistory {
    /// Crash instants still inside the window, oldest first.
    crashes: VecDeque<Instant>,
    /// Reloads scheduled over this composable's lifetime (surfaced as
    /// `restart_count` in /api/nodes). Never reset.
    pub restarts: u32,
}

impl CrashHistory {
    /// Forget recent crashes — an operator's manual load is a fresh start for
    /// the crash-loop bound. The lifetime counter is kept.
    pub fn reset_window(&mut self) {
        self.crashes.clear();
    }
}

/// What to do about one crash.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum CrashDecision {
    /// Respawn is off for this composable (mode, or container not respawn).
    Disabled,
    /// Reload after `delay`. `restart` is the lifetime count including this one.
    Reload { delay: Duration, restart: u32 },
    /// Too many crashes inside the window: leave it Failed.
    GiveUp { crashes_in_window: u32 },
}

impl RespawnPolicy {
    pub fn from_timings(t: &LoadTimings) -> Self {
        Self {
            mode: t.composable_respawn,
            max_restarts: t.composable_respawn_max_restarts,
            window: t.composable_respawn_window,
            max_backoff: t.composable_respawn_max_backoff,
        }
    }

    /// Decide on a crash at `now`. `container_respawn` / `base_delay` are the
    /// container's launch-file `respawn` / `respawn_delay` (as currently
    /// configured, so the web UI's respawn toggle on a container applies).
    pub fn on_crash(
        &self,
        history: &mut CrashHistory,
        now: Instant,
        container_respawn: bool,
        base_delay: Duration,
    ) -> CrashDecision {
        let enabled = match self.mode {
            ComposableRespawn::Off => false,
            ComposableRespawn::Inherit => container_respawn,
            ComposableRespawn::OnCrash => true,
        };
        if !enabled {
            return CrashDecision::Disabled;
        }

        while let Some(&oldest) = history.crashes.front() {
            if now.saturating_duration_since(oldest) > self.window {
                history.crashes.pop_front();
            } else {
                break;
            }
        }
        history.crashes.push_back(now);
        let in_window = history.crashes.len() as u32;

        if in_window > self.max_restarts.max(1) {
            return CrashDecision::GiveUp {
                crashes_in_window: in_window,
            };
        }

        history.restarts = history.restarts.saturating_add(1);
        CrashDecision::Reload {
            delay: self.delay_for(in_window, base_delay),
            restart: history.restarts,
        }
    }

    /// Delay before the reload for the `n`th crash inside the window (1-based):
    /// exactly `base` for the first, then `max(base, 1s) * 2^(n-1)`, capped at
    /// `max(max_backoff, base)` — the cap never undercuts the launch file.
    pub fn delay_for(&self, n: u32, base: Duration) -> Duration {
        if n <= 1 {
            return base;
        }
        let cap = self.max_backoff.max(base);
        let floor = base.max(MIN_BACKOFF_BASE);
        let factor = 1u32.checked_shl(n - 1).unwrap_or(u32::MAX);
        floor.checked_mul(factor).unwrap_or(cap).min(cap)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn policy(mode: ComposableRespawn) -> RespawnPolicy {
        RespawnPolicy {
            mode,
            max_restarts: 3,
            window: Duration::from_secs(60),
            max_backoff: Duration::from_secs(10),
        }
    }

    const S: fn(u64) -> Duration = Duration::from_secs;

    #[test]
    fn off_never_reloads() {
        let p = policy(ComposableRespawn::Off);
        let mut h = CrashHistory::default();
        assert_eq!(
            p.on_crash(&mut h, Instant::now(), true, S(1)),
            CrashDecision::Disabled
        );
        assert_eq!(h.restarts, 0);
    }

    #[test]
    fn inherit_follows_the_container() {
        let p = policy(ComposableRespawn::Inherit);
        let mut h = CrashHistory::default();
        let now = Instant::now();
        assert_eq!(
            p.on_crash(&mut h, now, false, S(2)),
            CrashDecision::Disabled
        );
        assert_eq!(
            p.on_crash(&mut h, now, true, S(2)),
            CrashDecision::Reload {
                delay: S(2),
                restart: 1
            }
        );
    }

    #[test]
    fn on_crash_ignores_the_container_flag() {
        let p = policy(ComposableRespawn::OnCrash);
        let mut h = CrashHistory::default();
        assert!(matches!(
            p.on_crash(&mut h, Instant::now(), false, S(0)),
            CrashDecision::Reload { .. }
        ));
    }

    /// The first reload honours the launch file's delay exactly — including 0.
    #[test]
    fn first_reload_waits_the_container_delay() {
        let p = policy(ComposableRespawn::OnCrash);
        assert_eq!(p.delay_for(1, S(0)), S(0));
        assert_eq!(
            p.delay_for(1, Duration::from_millis(2500)),
            Duration::from_millis(2500)
        );
    }

    #[test]
    fn repeats_back_off_exponentially_up_to_the_cap() {
        let p = policy(ComposableRespawn::OnCrash);
        // base 0 -> floor of 1 s for the backoff
        assert_eq!(p.delay_for(2, S(0)), S(2));
        assert_eq!(p.delay_for(3, S(0)), S(4));
        assert_eq!(p.delay_for(4, S(0)), S(8));
        assert_eq!(p.delay_for(5, S(0)), S(10), "capped at max_backoff");
        assert_eq!(p.delay_for(40, S(0)), S(10), "no overflow far past the cap");
        // base 3 s
        assert_eq!(p.delay_for(2, S(3)), S(6));
        assert_eq!(p.delay_for(3, S(3)), S(10));
    }

    /// A launch delay longer than the cap is never shortened.
    #[test]
    fn cap_never_undercuts_the_launch_delay() {
        let p = policy(ComposableRespawn::OnCrash);
        assert_eq!(p.delay_for(2, S(30)), S(30));
        assert_eq!(p.delay_for(6, S(30)), S(30));
    }

    #[test]
    fn gives_up_after_max_restarts_within_the_window() {
        let p = policy(ComposableRespawn::OnCrash);
        let mut h = CrashHistory::default();
        let t0 = Instant::now();
        for i in 0..3 {
            let d = p.on_crash(&mut h, t0 + S(i), true, S(0));
            assert!(
                matches!(d, CrashDecision::Reload { .. }),
                "crash {i}: {d:?}"
            );
        }
        assert_eq!(
            p.on_crash(&mut h, t0 + S(3), true, S(0)),
            CrashDecision::GiveUp {
                crashes_in_window: 4
            }
        );
        assert_eq!(h.restarts, 3, "a give-up is not a restart");
    }

    /// Crashes that age out of the window stop counting, so a node that
    /// crashes rarely is reloaded indefinitely, and its delay resets.
    #[test]
    fn old_crashes_age_out_of_the_window() {
        let p = policy(ComposableRespawn::OnCrash);
        let mut h = CrashHistory::default();
        let t0 = Instant::now();
        for i in 0..10u64 {
            assert_eq!(
                p.on_crash(&mut h, t0 + S(i * 61), true, S(1)),
                CrashDecision::Reload {
                    delay: S(1),
                    restart: i as u32 + 1
                }
            );
        }
    }

    #[test]
    fn manual_reset_clears_the_window_but_not_the_counter() {
        let p = policy(ComposableRespawn::OnCrash);
        let mut h = CrashHistory::default();
        let t0 = Instant::now();
        for i in 0..4 {
            p.on_crash(&mut h, t0 + S(i), true, S(0));
        }
        h.reset_window();
        assert_eq!(
            p.on_crash(&mut h, t0 + S(5), true, S(0)),
            CrashDecision::Reload {
                delay: S(0),
                restart: 4
            }
        );
    }

    #[test]
    fn max_restarts_zero_is_read_as_one() {
        let p = RespawnPolicy {
            max_restarts: 0,
            ..policy(ComposableRespawn::OnCrash)
        };
        let mut h = CrashHistory::default();
        let t0 = Instant::now();
        assert!(matches!(
            p.on_crash(&mut h, t0, true, S(0)),
            CrashDecision::Reload { .. }
        ));
        assert!(matches!(
            p.on_crash(&mut h, t0, true, S(0)),
            CrashDecision::GiveUp { .. }
        ));
    }
}
