//! Stopping one launched process at shutdown, as `ros2 launch` does (#0084).
//!
//! launch's `ExecuteProcess` shuts a process down with SIGINT, then, if it is
//! still running `sigterm_timeout` seconds later (default 5), SIGTERM, and
//! `sigkill_timeout` seconds after that (default 5), SIGKILL. Both timeouts
//! are per action, so a supervisor that needs a minute and a half to stop
//! its own children in order declares `sigterm_timeout="90"` and gets it.
//!
//! play_launch sends the SIGINT to the whole process group, staggered
//! (`pgid::stop_process_group_staggered`, which also leaves the ROS graph
//! first); this module is the rest of the ladder, run by each member's
//! actor for its own process, timed from the moment that SIGINT was sent.

use std::time::Duration;
use tracing::{debug, warn};

/// launch's default for both timeouts.
const DEFAULT_TIMEOUT: Duration = Duration::from_secs(5);

/// How long an actor waits for the group's SIGINT before sending its own:
/// longer than leaving the ROS graph (2 s) plus the stagger (1 s per scan)
/// normally take, so it is only reached when nothing signals the group.
const GROUP_SIGNAL_WAIT: Duration = Duration::from_secs(5);

/// The two waits of the ladder, from the process's launch record.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StopSchedule {
    /// From SIGINT to SIGTERM.
    pub sigterm: Duration,
    /// From SIGTERM to SIGKILL.
    pub sigkill: Duration,
}

impl Default for StopSchedule {
    fn default() -> Self {
        Self {
            sigterm: DEFAULT_TIMEOUT,
            sigkill: DEFAULT_TIMEOUT,
        }
    }
}

impl StopSchedule {
    /// The record's timeouts in seconds, `None` meaning launch's 5.
    pub fn new(sigterm_secs: Option<f64>, sigkill_secs: Option<f64>) -> Self {
        let secs = |s: Option<f64>| {
            s.filter(|s| s.is_finite() && *s >= 0.0)
                .map(Duration::from_secs_f64)
                .unwrap_or(DEFAULT_TIMEOUT)
        };
        Self {
            sigterm: secs(sigterm_secs),
            sigkill: secs(sigkill_secs),
        }
    }
}

fn secs(d: Duration) -> String {
    let s = d.as_secs_f64();
    if s.fract() == 0.0 {
        format!("{s:.0}")
    } else {
        format!("{s}")
    }
}

/// Bring `child` down at shutdown: wait for the group's SIGINT (sending one
/// to `pid` if none comes), give it `schedule.sigterm`, SIGTERM it, give it
/// `schedule.sigkill`, SIGKILL it. Returns once it has exited.
pub async fn stop_at_shutdown(
    child: &mut tokio::process::Child,
    pid: u32,
    name: &str,
    schedule: StopSchedule,
) {
    use nix::{
        sys::signal::{Signal, kill},
        unistd::Pid,
    };
    let target = Pid::from_raw(pid as i32);

    // The SIGINT. A child that exits before it is sent is done.
    tokio::select! {
        status = child.wait() => {
            debug!("[{name}] Exited before the shutdown's SIGINT: {status:?}");
            return;
        }
        signalled = super::pgid::group_signalled(GROUP_SIGNAL_WAIT) => {
            if !signalled {
                debug!("[{name}] The group was not signalled; sending SIGINT to pid {pid}");
                let _ = kill(target, Signal::SIGINT);
            }
        }
    }

    if let Ok(status) = tokio::time::timeout(schedule.sigterm, child.wait()).await {
        debug!("[{name}] Exited after SIGINT: {status:?}");
        return;
    }
    // launch's own wording.
    warn!(
        "[{name}] failed to terminate '{}' seconds after receiving 'SIGINT', escalating to 'SIGTERM'",
        secs(schedule.sigterm)
    );
    if let Err(e) = kill(target, Signal::SIGTERM) {
        debug!("[{name}] SIGTERM to pid {pid}: {e}");
    }

    if let Ok(status) = tokio::time::timeout(schedule.sigkill, child.wait()).await {
        debug!("[{name}] Exited after SIGTERM: {status:?}");
        return;
    }
    warn!(
        "[{name}] failed to terminate '{}' seconds after receiving 'SIGTERM', escalating to 'SIGKILL'",
        secs(schedule.sigkill)
    );
    let _ = child.kill().await;
    let _ = child.wait().await;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_schedule_defaults_to_five_and_five() {
        assert_eq!(StopSchedule::new(None, None), StopSchedule::default());
        let s = StopSchedule::new(Some(20.0), Some(0.5));
        assert_eq!(s.sigterm, Duration::from_secs(20));
        assert_eq!(s.sigkill, Duration::from_millis(500));
        assert_eq!(StopSchedule::new(Some(-1.0), None), StopSchedule::default());
    }

    /// A process that ignores SIGINT is stopped by the SIGTERM, at the
    /// schedule's time, not before. Nothing signals a group here, so the
    /// ladder sends its own SIGINT after the group wait.
    #[tokio::test]
    async fn the_ladder_escalates_on_schedule() {
        let mut child = tokio::process::Command::new("sh")
            .args(["-c", "trap '' INT; exec sleep 30"])
            .kill_on_drop(true)
            .spawn()
            .expect("spawn sh");
        let pid = child.id().unwrap();
        let start = std::time::Instant::now();
        let schedule = StopSchedule::new(Some(0.5), Some(5.0));
        stop_at_shutdown(&mut child, pid, "t", schedule).await;
        let took = start.elapsed();
        // Group wait (5 s) + sigterm (0.5 s); the SIGTERM ends it.
        assert!(
            took >= GROUP_SIGNAL_WAIT + Duration::from_millis(450)
                && took < GROUP_SIGNAL_WAIT + Duration::from_secs(4),
            "{took:?}"
        );
    }
}
