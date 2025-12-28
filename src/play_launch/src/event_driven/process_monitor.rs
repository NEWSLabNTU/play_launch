//! Process monitoring component
//!
//! This module implements the ProcessMonitor, which is responsible for managing process
//! lifecycle monitoring. It owns Child handles and publishes exit events.
//!
//! # Design
//! - ProcessMonitor owns Child handles (members only store PIDs)
//! - Spawns one monitoring task per process
//! - Tasks are independent and self-cleaning
//! - Provides graceful shutdown (SIGTERM → wait → SIGKILL)

use super::events::{EventBus, MemberEvent};
use eyre::{eyre, Result};
use nix::{
    sys::signal::{self, Signal},
    unistd::Pid,
};
use std::{collections::HashMap, time::Duration};
use tokio::{
    process::Child,
    sync::{watch, Mutex},
    task::JoinHandle,
};
use tracing::{error, info, warn};

/// Process monitor manages lifecycle of spawned processes
///
/// # Responsibilities
/// - Owns Child handles (members don't store them)
/// - Spawns monitoring tasks that detect process exits
/// - Publishes ProcessExited events when processes terminate
/// - Provides process control (kill, graceful shutdown)
pub struct ProcessMonitor {
    /// Event bus for publishing exit events
    event_bus: EventBus,
    /// Shutdown signal receiver
    shutdown_rx: watch::Receiver<bool>,
    /// Tracking of monitoring tasks for cleanup
    tasks: Mutex<HashMap<String, JoinHandle<()>>>,
}

impl ProcessMonitor {
    /// Create a new ProcessMonitor
    pub fn new(event_bus: EventBus, shutdown_rx: watch::Receiver<bool>) -> Self {
        Self {
            event_bus,
            shutdown_rx,
            tasks: Mutex::new(HashMap::new()),
        }
    }

    /// Register a process for monitoring
    ///
    /// Spawns a background task that waits for the process to exit and publishes
    /// a ProcessExited event when it does.
    ///
    /// # Arguments
    /// - `name`: Unique name of the process
    /// - `child`: The spawned Child handle (will be consumed by the monitoring task)
    ///
    /// # Returns
    /// The PID of the process
    pub async fn register_process(&self, name: String, mut child: Child) -> Result<u32> {
        let pid = child.id().ok_or_else(|| eyre!("Failed to get PID"))?;

        let event_bus = self.event_bus.clone();
        let mut shutdown_rx = self.shutdown_rx.clone();
        let name_clone = name.clone();

        // Spawn monitoring task (consumes child via .wait())
        let handle = tokio::spawn(async move {
            tokio::select! {
                result = child.wait() => {
                    match result {
                        Ok(exit_status) => {
                            info!("Process {} (PID {}) exited with status: {:?}", name_clone, pid, exit_status);
                            if let Err(e) = event_bus.publish(MemberEvent::ProcessExited {
                                name: name_clone,
                                exit_code: exit_status.code(),
                            }) {
                                error!("Failed to publish ProcessExited event: {}", e);
                            }
                        }
                        Err(e) => {
                            error!("Failed to wait for process {} (PID {}): {}", name_clone, pid, e);
                        }
                    }
                }
                _ = shutdown_rx.changed() => {
                    info!("Shutdown requested, stopping monitor for {} (PID {})", name_clone, pid);
                }
            }
        });

        // Track the task for graceful shutdown
        self.tasks.lock().await.insert(name.clone(), handle);

        info!("Registered process {} with PID {}", name, pid);
        Ok(pid)
    }

    /// Kill a process by PID using the specified signal
    ///
    /// # Arguments
    /// - `pid`: Process ID to kill
    /// - `signal`: Signal to send (e.g., SIGTERM, SIGKILL)
    pub async fn kill_process(&self, pid: u32, sig: Signal) -> Result<()> {
        signal::kill(Pid::from_raw(pid as i32), sig)
            .map_err(|e| eyre!("Failed to send signal {:?} to PID {}: {}", sig, pid, e))?;

        Ok(())
    }

    /// Gracefully shutdown a process with SIGTERM → wait → SIGKILL pattern
    ///
    /// # Arguments
    /// - `name`: Name of the process (for logging)
    /// - `pid`: Process ID
    ///
    /// # Behavior
    /// 1. Send SIGTERM
    /// 2. Wait up to 5 seconds for process to exit
    /// 3. If still alive, send SIGKILL
    pub async fn shutdown_process(&self, name: &str, pid: u32) -> Result<()> {
        use signal::{SIGKILL, SIGTERM};

        // Send SIGTERM
        info!("Sending SIGTERM to process {} (PID {})", name, pid);
        self.kill_process(pid, SIGTERM).await?;

        // Wait up to 5 seconds for process to exit gracefully
        let start = tokio::time::Instant::now();
        let timeout = Duration::from_secs(5);

        while start.elapsed() < timeout {
            // Check if process still exists by trying to send signal 0 (null signal that just checks existence)
            // On Linux, sending signal 0 doesn't actually send anything, just checks permissions/existence
            match signal::kill(Pid::from_raw(pid as i32), None) {
                Err(nix::Error::ESRCH) => {
                    // Process is gone (ESRCH = no such process)
                    info!("Process {} (PID {}) exited gracefully", name, pid);
                    return Ok(());
                }
                Err(e) => {
                    // Other errors (e.g., permission denied) mean process might still exist
                    warn!(
                        "Error checking process {} (PID {}): {}. Assuming still alive.",
                        name, pid, e
                    );
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
                Ok(_) => {
                    // Process still exists, wait a bit
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
            }
        }

        // Force kill if still alive
        warn!(
            "Process {} (PID {}) did not exit gracefully, sending SIGKILL",
            name, pid
        );
        self.kill_process(pid, SIGKILL).await?;

        Ok(())
    }

    /// Abort all monitoring tasks (called on shutdown)
    #[allow(dead_code)]
    pub async fn shutdown(&self) {
        let tasks = self.tasks.lock().await;
        for (name, handle) in tasks.iter() {
            info!("Aborting monitoring task for {}", name);
            handle.abort();
        }
    }

    /// Remove a completed monitoring task from tracking
    ///
    /// Called internally when tasks complete
    #[allow(dead_code)]
    pub async fn remove_task(&self, name: &str) {
        self.tasks.lock().await.remove(name);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::sync::watch;

    #[tokio::test]
    async fn test_register_process() {
        let (event_bus, mut rx) = EventBus::new();
        let (_, shutdown_rx) = watch::channel(false);
        let monitor = ProcessMonitor::new(event_bus, shutdown_rx);

        // Spawn a process that exits immediately (using 'true' command)
        let child = tokio::process::Command::new("true").spawn().unwrap();
        let pid = child.id().unwrap();

        // Register process
        let registered_pid = monitor
            .register_process("test_process".to_string(), child)
            .await
            .unwrap();
        assert_eq!(registered_pid, pid);

        // Should receive ProcessExited event
        tokio::time::timeout(Duration::from_secs(1), async {
            let event = rx.recv().await.unwrap();
            match event {
                MemberEvent::ProcessExited { name, exit_code } => {
                    assert_eq!(name, "test_process");
                    assert_eq!(exit_code, Some(0));
                }
                _ => panic!("Expected ProcessExited event"),
            }
        })
        .await
        .unwrap();
    }

    #[tokio::test]
    async fn test_register_process_failure() {
        let (event_bus, mut rx) = EventBus::new();
        let (_, shutdown_rx) = watch::channel(false);
        let monitor = ProcessMonitor::new(event_bus, shutdown_rx);

        // Spawn a process that fails immediately (using 'false' command)
        let child = tokio::process::Command::new("false").spawn().unwrap();

        // Register process
        monitor
            .register_process("failing_process".to_string(), child)
            .await
            .unwrap();

        // Should receive ProcessExited event with non-zero exit code
        tokio::time::timeout(Duration::from_secs(1), async {
            let event = rx.recv().await.unwrap();
            match event {
                MemberEvent::ProcessExited { name, exit_code } => {
                    assert_eq!(name, "failing_process");
                    assert_eq!(exit_code, Some(1));
                }
                _ => panic!("Expected ProcessExited event"),
            }
        })
        .await
        .unwrap();
    }

    #[tokio::test]
    async fn test_kill_process() {
        let (event_bus, _) = EventBus::new();
        let (_, shutdown_rx) = watch::channel(false);
        let monitor = ProcessMonitor::new(event_bus, shutdown_rx);

        // Spawn a long-running process
        let mut child = tokio::process::Command::new("sleep")
            .arg("10")
            .spawn()
            .unwrap();
        let pid = child.id().unwrap();

        // Kill the process
        monitor.kill_process(pid, signal::SIGKILL).await.unwrap();

        // Process should exit
        let result = tokio::time::timeout(Duration::from_secs(1), child.wait())
            .await
            .unwrap()
            .unwrap();

        assert!(!result.success());
    }
}
