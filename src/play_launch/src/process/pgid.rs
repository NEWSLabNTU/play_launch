//! Process group ID management

use eyre::Context;
use tracing::{debug, warn};

/// Run the anchor process as an async tokio task (Phase 4)
/// Returns PGID via oneshot channel, then waits for shutdown
#[cfg(unix)]
pub async fn run_anchor_task(
    pgid_tx: tokio::sync::oneshot::Sender<i32>,
    mut shutdown_rx: tokio::sync::watch::Receiver<bool>,
) -> eyre::Result<()> {
    use std::os::unix::process::CommandExt;

    debug!("Starting anchor process task");

    // Spawn anchor process (minimal process that exits immediately to become zombie)
    let anchor = std::process::Command::new("true")
        .process_group(0) // Creates new PGID = anchor's PID
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .wrap_err("Failed to spawn anchor zombie process")?;

    let pgid = anchor.id() as i32;
    debug!("Anchor zombie process created with PGID: {}", pgid);

    // Send PGID back immediately
    let _ = pgid_tx.send(pgid);

    // Wait for shutdown signal
    shutdown_rx.changed().await.ok();
    if *shutdown_rx.borrow() {
        debug!("Anchor task received shutdown signal");
    }

    // Do NOT reap the anchor here. The group id stays valid only while the
    // group has a member, and the anchor IS that member; a spawn can still be
    // in flight when shutdown lands — an actor past its shutdown check but
    // not yet forked does `setpgid(0, pgid)` in the child, and that is EPERM
    // the instant this zombie is waited on (issue #0024: `run`'s web task
    // ended early, shutdown reaped the anchor, and the node still starting
    // failed with a bare "Operation not permitted" and an empty log). The
    // zombie costs one pid-table entry it has held for the whole run anyway.
    // The shutdown wait's `waitpid(-pgid)` sweep reaps it with everything
    // else in the group, and init reaps it if nothing does.
    drop(anchor);
    debug!("Anchor task exiting; anchor zombie left in place to keep the PGID valid");

    Ok(())
}

/// Kill an entire process group with a single signal
#[cfg(unix)]
pub fn kill_process_group(pgid: i32, signal: nix::sys::signal::Signal) {
    use nix::{sys::signal::killpg, unistd::Pid};

    match killpg(Pid::from_raw(pgid), signal) {
        Ok(_) => debug!("Sent {:?} to process group {}", signal, pgid),
        Err(e) => warn!("Failed to kill process group {}: {}", pgid, e),
    }
}

/// Longest a staggered stop may take, however many processes the group has.
const STAGGER_BUDGET: std::time::Duration = std::time::Duration::from_secs(1);
/// Gap between two processes' SIGTERM when the group is small enough.
const STAGGER_STEP: std::time::Duration = std::time::Duration::from_millis(5);

/// Stop a process group with SIGTERM, one process at a time.
///
/// One `killpg` makes every node leave the DDS domain in the same instant,
/// and stock rclcpp nodes then sometimes crash or hang in their own
/// teardown: a Fast DDS 2.6 race (`EDP::unpairWriterProxy`) while many
/// participants depart at once. Measured with 40 bare `demo_nodes_cpp`
/// nodes and no launcher, one SIGTERM each at once crashed a node in 5 of 15
/// runs, each crash leaving its participant's shared-memory files; the same
/// SIGTERMs a few milliseconds apart crashed none in 30.
///
/// Deepest first, so a child is signalled before the process that forked it:
/// an `isolated` container stops all its composables together the moment it
/// is told to stop, which would undo the stagger. The gap is
/// [`STAGGER_STEP`], shrunk so the whole group takes at most
/// [`STAGGER_BUDGET`]. The group is scanned again until a scan finds nobody
/// new, so a process forked meanwhile is not missed; where `/proc` lists
/// nobody, one `killpg` stands in.
#[cfg(target_os = "linux")]
pub async fn stop_process_group_staggered(pgid: i32) {
    use nix::{
        sys::signal::{Signal, kill},
        unistd::Pid,
    };

    let mut signalled = std::collections::HashSet::new();
    for scan in 0..10 {
        let pending: Vec<i32> = group_members_deepest_first(pgid)
            .into_iter()
            .filter(|pid| !signalled.contains(pid))
            .collect();
        if pending.is_empty() {
            if scan == 0 {
                kill_process_group(pgid, Signal::SIGTERM);
            }
            break;
        }
        let step = STAGGER_STEP.min(STAGGER_BUDGET / pending.len() as u32);
        debug!(
            "Stopping {} process(es) of group {pgid}, {step:?} apart",
            pending.len()
        );
        for pid in pending {
            signalled.insert(pid);
            let _ = kill(Pid::from_raw(pid), Signal::SIGTERM);
            tokio::time::sleep(step).await;
        }
    }
}

#[cfg(all(unix, not(target_os = "linux")))]
pub async fn stop_process_group_staggered(pgid: i32) {
    kill_process_group(pgid, nix::sys::signal::Signal::SIGTERM);
}

/// The live members of process group `pgid`, each after every member it
/// descends from has been listed below it: deepest first, then by pid.
#[cfg(target_os = "linux")]
fn group_members_deepest_first(pgid: i32) -> Vec<i32> {
    let Ok(dir) = std::fs::read_dir("/proc") else {
        return Vec::new();
    };
    // pid -> ppid, for the group's live members
    let mut parent = std::collections::HashMap::new();
    for entry in dir.flatten() {
        let Some(pid) = entry
            .file_name()
            .to_str()
            .and_then(|s| s.parse::<i32>().ok())
        else {
            continue;
        };
        let Ok(stat) = std::fs::read_to_string(format!("/proc/{pid}/stat")) else {
            continue;
        };
        // After the parenthesised comm: state, ppid, pgrp.
        let Some(rest) = stat.rfind(')').map(|i| &stat[i + 1..]) else {
            continue;
        };
        let mut f = rest.split_whitespace();
        let (Some(state), Some(ppid), Some(pgrp)) = (f.next(), f.next(), f.next()) else {
            continue;
        };
        if pgrp.parse::<i32>().ok() != Some(pgid) || state == "Z" || state == "X" {
            continue;
        }
        parent.insert(pid, ppid.parse::<i32>().unwrap_or(0));
    }
    let depth = |mut pid: i32| {
        let mut d = 0usize;
        while let Some(&pp) = parent.get(&pid) {
            if !parent.contains_key(&pp) || d > parent.len() {
                break;
            }
            d += 1;
            pid = pp;
        }
        d
    };
    let mut members: Vec<(usize, i32)> = parent.keys().map(|&pid| (depth(pid), pid)).collect();
    members.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
    members.into_iter().map(|(_, pid)| pid).collect()
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;
    use std::os::unix::process::CommandExt;

    /// The anchor exists to keep one process-group id valid for every spawn
    /// of the run, and a spawn can race the shutdown that ends the run: an
    /// actor past its shutdown check but not yet forked does
    /// `setpgid(0, pgid)` in the child, which is EPERM the moment the group
    /// has no member left. Issue #0024 — `run`'s web task ended early,
    /// shutdown reaped the anchor zombie, and the node it was still
    /// starting failed with a bare "Operation not permitted".
    ///
    /// So the group must still be joinable AFTER the anchor task has
    /// answered shutdown. The zombie costs a pid-table entry, which it held
    /// for the whole run anyway; reaping it is the shutdown wait's job.
    /// A child is signalled before the process that forked it, and every
    /// member of the group is signalled: a `sh` that forks a `sleep` and
    /// waits for it.
    #[cfg(target_os = "linux")]
    #[tokio::test]
    async fn a_staggered_stop_signals_children_first_and_everyone() {
        let mut parent = std::process::Command::new("sh")
            .args(["-c", "sleep 30 & wait"])
            .process_group(0)
            .spawn()
            .expect("spawn sh");
        let pgid = parent.id() as i32;
        let mut members = Vec::new();
        for _ in 0..100 {
            members = group_members_deepest_first(pgid);
            if members.len() == 2 {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        assert_eq!(members.len(), 2, "{members:?}");
        assert_eq!(
            members[1], pgid,
            "the forking shell comes last: {members:?}"
        );
        stop_process_group_staggered(pgid).await;
        // `sleep` goes first, so the shell's `wait` usually returns and it
        // exits 0 before its own SIGTERM lands; either way it is gone.
        parent.wait().expect("wait sh");
        for _ in 0..100 {
            if group_members_deepest_first(pgid).is_empty() {
                return;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        panic!(
            "group {pgid} still has {:?}",
            group_members_deepest_first(pgid)
        );
    }

    #[tokio::test]
    async fn the_anchor_group_is_still_joinable_after_shutdown() {
        let (pgid_tx, pgid_rx) = tokio::sync::oneshot::channel();
        let (shutdown_tx, shutdown_rx) = tokio::sync::watch::channel(false);
        let task = tokio::spawn(run_anchor_task(pgid_tx, shutdown_rx));
        let pgid = pgid_rx.await.expect("anchor task must report a pgid");

        shutdown_tx.send(true).unwrap();
        task.await.unwrap().unwrap();

        // A spawn that lost the race against shutdown joins the group now.
        let joined = std::process::Command::new("true")
            .process_group(pgid)
            .stdin(std::process::Stdio::null())
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn()
            .map(|mut probe| {
                let _ = probe.wait();
            });
        // Reap the anchor so the test process leaks nothing, whichever way
        // the assertion goes.
        unsafe {
            libc::waitpid(pgid, std::ptr::null_mut(), 0);
        }
        assert!(
            joined.is_ok(),
            "process group {pgid} vanished at shutdown: {:?}",
            joined.err()
        );
    }
}
