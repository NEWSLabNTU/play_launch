use std::io;
use std::os::unix::process::CommandExt;
use std::process::{Child, Command, ExitStatus};
use std::time::{Duration, Instant};

/// RAII wrapper that spawns a process in its own process group and guarantees
/// cleanup on drop — even if the test panics.
///
/// Linux features used:
/// - `setsid()`: puts the child in a new process group so we can kill the
///   entire tree with a single `kill(-pgid, ...)`.
/// - `PR_SET_PDEATHSIG(SIGKILL)`: tells the kernel to send SIGKILL to the
///   child if the parent (test) process dies unexpectedly.
pub struct ManagedProcess {
    child: Child,
    pgid: i32,
}

impl ManagedProcess {
    /// Spawn `cmd` in a new session/process group.
    pub fn spawn(cmd: &mut Command) -> io::Result<Self> {
        // Safety: setsid() and prctl() are async-signal-safe and only affect
        // the child (called between fork and exec).
        unsafe {
            cmd.pre_exec(|| {
                libc::setsid();
                libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGKILL);
                Ok(())
            });
        }

        let child = cmd.spawn()?;
        let pgid = child.id() as i32;
        Ok(Self { child, pgid })
    }

    /// Return the PID of the child process.
    pub fn id(&self) -> u32 {
        self.child.id()
    }

    /// Block until the child exits or `timeout` elapses.
    ///
    /// On timeout the process group is killed and a panic is raised (which
    /// triggers `Drop` cleanup for any other guards in scope).
    pub fn wait_with_timeout(&mut self, timeout: Duration) -> ExitStatus {
        let start = Instant::now();
        loop {
            match self.child.try_wait() {
                Ok(Some(status)) => return status,
                Ok(None) => {
                    if start.elapsed() >= timeout {
                        self.kill_group();
                        let _ = self.child.wait();
                        panic!("command timed out after {}s", timeout.as_secs());
                    }
                    std::thread::sleep(Duration::from_millis(100));
                }
                Err(e) => panic!("error waiting for child: {e}"),
            }
        }
    }

    /// Send SIGTERM then SIGKILL to the entire process group.
    fn kill_group(&self) {
        unsafe {
            libc::kill(-self.pgid, libc::SIGTERM);
        }
        std::thread::sleep(Duration::from_secs(2));
        unsafe {
            libc::kill(-self.pgid, libc::SIGKILL);
        }
    }
}

impl Drop for ManagedProcess {
    fn drop(&mut self) {
        // Only kill if the process is still running.
        match self.child.try_wait() {
            Ok(Some(_)) => {} // Already exited — nothing to do.
            _ => {
                self.kill_group();
                let _ = self.child.wait();
            }
        }
    }
}
