//! Process tree traversal utilities

use std::sync::atomic::{AtomicBool, Ordering};

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
    for (i, (pid, process)) in sys.processes().iter().enumerate() {
        // Check cancellation every 100 processes
        if i % 100 == 0 && cancel.load(Ordering::Relaxed) {
            return result;
        }

        if let Some(parent) = process.parent() {
            if parent.as_u32() == parent_pid {
                let child_pid = pid.as_u32();
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
