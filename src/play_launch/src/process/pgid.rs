//! Process group ID management

use eyre::Context;
use tracing::{debug, warn};

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
