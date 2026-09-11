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
