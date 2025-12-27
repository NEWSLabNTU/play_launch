//! Process cleanup utilities

use std::sync::atomic::{AtomicBool, Ordering};
use tracing::debug;

/// Global flag to indicate graceful shutdown was completed (avoids duplicate SIGKILL in CleanupGuard)
pub static GRACEFUL_SHUTDOWN_DONE: AtomicBool = AtomicBool::new(false);

/// Kill all descendant processes of the current process recursively
#[cfg(unix)]
pub fn kill_all_descendants() {
    use super::tree::find_all_descendants;
    use std::{process, sync::Arc};

    // Skip if graceful shutdown was already completed
    if GRACEFUL_SHUTDOWN_DONE.load(Ordering::Relaxed) {
        debug!("Graceful shutdown already completed, skipping CleanupGuard force kill");
        return;
    }

    let my_pid = process::id();
    debug!("Killing all descendant processes of PID {}", my_pid);

    // Create a dummy cancellation token that's never cancelled (cleanup should always complete)
    let cancel_token = Arc::new(AtomicBool::new(false));

    // Find all descendant PIDs recursively (including grandchildren)
    let descendants = find_all_descendants(my_pid, &cancel_token);

    if !descendants.is_empty() {
        debug!(
            "Found {} descendant processes to terminate: {:?}",
            descendants.len(),
            descendants
        );

        // Kill them in reverse order (children before parents) with SIGTERM
        for &pid in descendants.iter().rev() {
            debug!("Sending SIGTERM to PID {}", pid);
            use nix::{
                sys::signal::{kill, Signal},
                unistd::Pid,
            };
            let _ = kill(Pid::from_raw(pid as i32), Signal::SIGTERM);
        }

        // Give processes time to terminate gracefully
        std::thread::sleep(std::time::Duration::from_millis(200));

        // Force kill any remaining processes with SIGKILL
        for &pid in descendants.iter().rev() {
            debug!("Sending SIGKILL to PID {}", pid);
            use nix::{
                sys::signal::{kill, Signal},
                unistd::Pid,
            };
            let _ = kill(Pid::from_raw(pid as i32), Signal::SIGKILL);
        }
    } else {
        debug!("No descendant processes found to terminate");
    }
}

#[cfg(not(unix))]
pub fn kill_all_descendants() {
    // No-op on non-Unix systems
}
