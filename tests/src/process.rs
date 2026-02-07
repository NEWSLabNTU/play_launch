use std::io;
use std::os::unix::process::CommandExt;
use std::process::{Child, Command};

/// RAII wrapper that spawns a process in its own process group and kills the
/// entire group on drop.  This prevents orphan ROS processes from lingering
/// after a test finishes (or panics).
pub struct ManagedProcess {
    child: Child,
    pgid: i32,
}

impl ManagedProcess {
    /// Spawn `cmd` in a new process group (via `setsid` on Linux).
    pub fn spawn(cmd: &mut Command) -> io::Result<Self> {
        // Safety: setsid() is async-signal-safe and only affects the child.
        unsafe {
            cmd.pre_exec(|| {
                libc::setsid();
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

    /// Wait for the child to exit and return its status.
    pub fn wait(&mut self) -> io::Result<std::process::ExitStatus> {
        self.child.wait()
    }
}

impl Drop for ManagedProcess {
    fn drop(&mut self) {
        // Send SIGTERM to the entire process group, then SIGKILL after a grace
        // period.  This mirrors the pattern from the project's justfile recipes.
        unsafe {
            libc::kill(-self.pgid, libc::SIGTERM);
        }
        std::thread::sleep(std::time::Duration::from_secs(2));
        unsafe {
            libc::kill(-self.pgid, libc::SIGKILL);
        }
        // Reap zombie.
        let _ = self.child.wait();
    }
}
