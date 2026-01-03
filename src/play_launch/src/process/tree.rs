//! Process tree traversal utilities

use std::sync::atomic::{AtomicBool, Ordering};

/// Check if a PID is a thread (not a process)
/// In Linux, threads have PID != TGID (thread group ID)
#[cfg(unix)]
fn is_thread(pid: u32) -> bool {
    // Read /proc/[pid]/status to get TGID
    if let Ok(status) = std::fs::read_to_string(format!("/proc/{}/status", pid)) {
        for line in status.lines() {
            if line.starts_with("Tgid:") {
                if let Some(tgid_str) = line.split_whitespace().nth(1) {
                    if let Ok(tgid) = tgid_str.parse::<u32>() {
                        // If PID != TGID, it's a thread
                        return pid != tgid;
                    }
                }
            }
        }
    }
    // If we can't read status, assume it's a process (be conservative)
    false
}

/// Recursively find all descendant PIDs of a given parent PID
#[cfg(unix)]
pub fn find_all_descendants(parent_pid: u32, cancel: &AtomicBool) -> Vec<u32> {
    use sysinfo::System;

    // Check cancellation before doing expensive work
    if cancel.load(Ordering::Relaxed) {
        return Vec::new();
    }

    let mut result = Vec::new();
    let mut sys = System::new();
    sys.refresh_processes(sysinfo::ProcessesToUpdate::All, true);

    // Find all processes that have parent_pid as their parent
    // IMPORTANT: Filter out threads - only include actual processes
    for (i, (pid, process)) in sys.processes().iter().enumerate() {
        // Check cancellation every 100 processes
        if i % 100 == 0 && cancel.load(Ordering::Relaxed) {
            return result;
        }

        // Skip threads: In Linux, threads have the same TGID as their parent process
        // Real processes have PID == TGID, threads have PID != TGID
        // sysinfo doesn't expose TGID directly, so we check /proc/[pid]/status
        let child_pid = pid.as_u32();
        if is_thread(child_pid) {
            continue;
        }

        if let Some(parent) = process.parent() {
            if parent.as_u32() == parent_pid {
                // Add this child
                result.push(child_pid);
                // Recursively find this child's descendants (grandchildren, etc.)
                result.extend(find_all_descendants(child_pid, cancel));

                // Check cancellation after recursive call
                if cancel.load(Ordering::Relaxed) {
                    return result;
                }
            }
        }
    }

    result
}
