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
    let mut anchor = std::process::Command::new("true")
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

    // Kill anchor process
    let _ = anchor.kill();
    debug!("Anchor process killed");

    Ok(())
}

/// Spawn an anchor zombie process to allocate and hold a process group ID
#[cfg(unix)]
pub fn spawn_anchor_process() -> eyre::Result<(std::process::Child, i32)> {
    use std::os::unix::process::CommandExt;

    // Spawn a minimal process that exits immediately to become a zombie
    let anchor = std::process::Command::new("true")
        .process_group(0) // Creates new PGID = anchor's PID
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .wrap_err("Failed to spawn anchor zombie process")?;

    let pgid = anchor.id() as i32;
    debug!("Anchor zombie process created with PGID: {}", pgid);

    // Don't call wait() - let it become a zombie to hold the PGID
    Ok((anchor, pgid))
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
