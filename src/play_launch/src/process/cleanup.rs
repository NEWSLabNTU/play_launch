//! Process cleanup utilities

use std::sync::atomic::AtomicBool;
use tracing::debug;

/// Grace period between SIGTERM and SIGKILL when killing descendant processes
const TERMINATION_GRACE_PERIOD: std::time::Duration = std::time::Duration::from_millis(200);

/// Kill all descendant processes of the current process recursively
#[cfg(unix)]
pub fn kill_all_descendants() {
    use super::tree::find_all_descendants;
    use std::{process, sync::Arc};

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

        // Debug: Check which processes are actually alive
        let mut alive_count = 0;
        for &pid in descendants.iter().take(10) {
            if let Ok(status) = std::fs::read_to_string(format!("/proc/{}/status", pid)) {
                let state = status
                    .lines()
                    .find(|line| line.starts_with("State:"))
                    .and_then(|line| line.split_whitespace().nth(1))
                    .unwrap_or("?");

                if let Ok(cmdline) = std::fs::read_to_string(format!("/proc/{}/cmdline", pid)) {
                    let cmdline = cmdline.replace('\0', " ");
                    debug!("  PID {} state={} cmdline: {}", pid, state, cmdline.trim());
                    if state != "Z" {
                        // Not a zombie
                        alive_count += 1;
                    }
                }
            }
        }
        if descendants.len() > 10 {
            debug!(
                "  ... and {} more processes (showing first 10, {} alive)",
                descendants.len() - 10,
                alive_count
            );
        } else {
            debug!(
                "  {} of {} processes are alive (non-zombie)",
                alive_count,
                descendants.len()
            );
        }

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
        std::thread::sleep(TERMINATION_GRACE_PERIOD);

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
