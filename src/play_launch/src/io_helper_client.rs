//! Client for communicating with the play_launch_io_helper daemon
//!
//! The I/O helper runs as a privileged child process with CAP_SYS_PTRACE,
//! allowing it to read /proc/[pid]/io files for processes that have capabilities
//! set (which clears their dumpable flag).
//!
//! Communication uses anonymous pipes (no filesystem sockets).

use eyre::{Context, Result};
use play_launch::ipc::{
    decode_message, encode_message, ProcIoError, ProcIoResult, Request, Response,
};
use std::{
    os::unix::io::FromRawFd,
    path::{Path, PathBuf},
    process::Stdio,
    time::Duration,
};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    process::{Child, Command},
};
use tracing::{debug, warn};

/// Client for communicating with the I/O helper daemon
pub struct IoHelperClient {
    request_writer: tokio::fs::File,  // Parent writes requests here
    response_reader: tokio::fs::File, // Parent reads responses here
    child: Option<Child>,
}

impl IoHelperClient {
    /// Spawn the helper daemon and connect via pipes
    ///
    /// This will:
    /// 1. Find the helper binary (same directory as current executable)
    /// 2. Create anonymous pipes for bidirectional communication
    /// 3. Spawn helper with FDs passed as arguments
    /// 4. Verify with a ping
    pub async fn spawn() -> Result<Self> {
        let helper_path = find_helper_binary()?;

        debug!("Spawning I/O helper: {:?}", helper_path);

        // Create pipes for bidirectional communication
        // Pipe 1: Parent writes requests → Helper reads requests
        let (request_reader_fd, request_writer_fd) = create_pipe()?;

        // Pipe 2: Helper writes responses → Parent reads responses
        let (response_reader_fd, response_writer_fd) = create_pipe()?;

        debug!(
            "Created pipes: request_pipe=[{},{}], response_pipe=[{},{}]",
            request_reader_fd, request_writer_fd, response_reader_fd, response_writer_fd
        );

        // Spawn helper process with FD arguments
        // Helper will read from request_reader_fd and write to response_writer_fd
        let child = Command::new(&helper_path)
            .arg("--request-fd")
            .arg(request_reader_fd.to_string())
            .arg("--response-fd")
            .arg(response_writer_fd.to_string())
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::inherit()) // Let helper errors show in our stderr
            .kill_on_drop(true) // Ensure helper dies with parent
            .spawn()
            .wrap_err_with(|| format!("Failed to spawn helper: {:?}", helper_path))?;

        // Convert FDs to tokio File handles for parent side
        // SAFETY: We just created these FDs via pipe(), we own them
        let request_writer_file = unsafe { std::fs::File::from_raw_fd(request_writer_fd) };
        let response_reader_file = unsafe { std::fs::File::from_raw_fd(response_reader_fd) };

        // Close child-side FDs in parent (helper has its own copies)
        unsafe {
            libc::close(request_reader_fd);
            libc::close(response_writer_fd);
        }

        // Convert to tokio async File handles
        let request_writer = tokio::fs::File::from_std(request_writer_file);
        let response_reader = tokio::fs::File::from_std(response_reader_file);

        // Create client
        let mut client = Self {
            request_writer,
            response_reader,
            child: Some(child),
        };

        // Give helper a moment to initialize
        tokio::time::sleep(Duration::from_millis(100)).await;

        // Verify helper is working
        client.ping().await?;

        debug!("I/O helper connected successfully via pipes");

        Ok(client)
    }

    /// Send a ping to verify helper is alive
    pub async fn ping(&mut self) -> Result<()> {
        let request = Request::Ping;
        let response = self.send_request(request).await?;

        match response {
            Response::Pong => Ok(()),
            Response::Error(e) => Err(eyre::eyre!("Helper error: {}", e)),
            _ => Err(eyre::eyre!("Unexpected response to ping: {:?}", response)),
        }
    }

    /// Read /proc/[pid]/io stats for a single PID
    #[allow(dead_code)] // Used by external API consumers, not within play_launch binary
    pub async fn read_proc_io(&mut self, pid: u32) -> Result<(u64, u64), ProcIoError> {
        let request = Request::ReadProcIo { pid };
        let response = self
            .send_request(request)
            .await
            .map_err(|e| ProcIoError::IoError(e.to_string()))?;

        match response {
            Response::ProcIo(result) => match result.result {
                Ok(stats) => Ok((stats.rchar, stats.wchar)),
                Err(e) => Err(e),
            },
            Response::Error(e) => Err(ProcIoError::IoError(format!("Helper error: {}", e))),
            _ => Err(ProcIoError::IoError(format!(
                "Unexpected response: {:?}",
                response
            ))),
        }
    }

    /// Read /proc/[pid]/io stats for multiple PIDs (batch request)
    pub async fn read_proc_io_batch(&mut self, pids: &[u32]) -> Result<Vec<ProcIoResult>> {
        if pids.is_empty() {
            return Ok(Vec::new());
        }

        let request = Request::ReadProcIoBatch {
            pids: pids.to_vec(),
        };

        let response = self.send_request(request).await?;

        match response {
            Response::ProcIoBatch(results) => Ok(results),
            Response::Error(e) => Err(eyre::eyre!("Helper error: {}", e)),
            _ => Err(eyre::eyre!("Unexpected response: {:?}", response)),
        }
    }

    /// Send a request and receive a response
    async fn send_request(&mut self, request: Request) -> Result<Response> {
        // Encode request
        let msg_buf = encode_message(&request).wrap_err("Failed to encode request")?;

        // Send request to helper via request pipe
        self.request_writer
            .write_all(&msg_buf)
            .await
            .wrap_err("Failed to send request")?;
        self.request_writer
            .flush()
            .await
            .wrap_err("Failed to flush request")?;

        // Read response length prefix from response pipe
        let mut len_buf = [0u8; 4];
        self.response_reader
            .read_exact(&mut len_buf)
            .await
            .wrap_err("Failed to read response length")?;

        let msg_len = u32::from_le_bytes(len_buf) as usize;

        if msg_len > 1024 * 1024 {
            return Err(eyre::eyre!("Response too large: {} bytes", msg_len));
        }

        // Read response payload
        let mut resp_buf = vec![0u8; msg_len];
        self.response_reader
            .read_exact(&mut resp_buf)
            .await
            .wrap_err("Failed to read response payload")?;

        // Decode response
        let response: Response = decode_message(&resp_buf).wrap_err("Failed to decode response")?;

        Ok(response)
    }

    /// Gracefully shutdown the helper
    #[allow(dead_code)] // Used for explicit cleanup, automatic cleanup via Drop also available
    pub async fn shutdown(mut self) -> Result<()> {
        // Send shutdown request
        debug!("Sending shutdown request to helper");
        if let Err(e) = self.send_request(Request::Shutdown).await {
            warn!("Failed to send shutdown request: {}", e);
        }

        // Wait for child to exit (with timeout)
        if let Some(mut child) = self.child.take() {
            tokio::select! {
                _ = tokio::time::sleep(Duration::from_secs(2)) => {
                    warn!("Helper did not exit gracefully, killing");
                    let _ = child.kill().await;
                }
                result = child.wait() => {
                    if let Err(e) = result {
                        warn!("Error waiting for helper: {}", e);
                    }
                }
            }
        }

        // Pipes are automatically closed when File handles are dropped
        // No cleanup needed (benefit of using pipes vs sockets)

        Ok(())
    }
}

impl Drop for IoHelperClient {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            // Force kill helper if still alive
            debug!("Killing helper process in drop");
            let _ = child.start_kill();
        }

        // Pipes are automatically closed when File handles are dropped
        // No manual cleanup needed (unlike sockets)
    }
}

/// Find the helper binary with fallback chain:
/// 1. PLAY_LAUNCH_IO_HELPER environment variable (development override)
/// 2. Same directory as current executable (pip install or colcon)
/// 3. ROS2 install paths (/opt/ros/humble/lib/play_launch/)
/// 4. PATH search (fallback)
fn find_helper_binary() -> Result<PathBuf> {
    // 1. Check environment variable (development override)
    if let Ok(path) = std::env::var("PLAY_LAUNCH_IO_HELPER") {
        let p = PathBuf::from(&path);
        if p.exists() {
            debug!("Using I/O helper from environment: {}", path);
            check_helper_capabilities(&p)?;
            return Ok(p);
        }
        warn!("PLAY_LAUNCH_IO_HELPER set but not found: {}", path);
    }

    // 2. Check same directory as current executable (pip install or colcon layout)
    if let Ok(exe_path) = std::env::current_exe() {
        if let Some(exe_dir) = exe_path.parent() {
            let helper_path = exe_dir.join("play_launch_io_helper");
            if helper_path.exists() {
                debug!("Using I/O helper from exe dir: {:?}", helper_path);
                check_helper_capabilities(&helper_path)?;
                return Ok(helper_path);
            }
        }
    }

    // 3. Check ROS2 install locations
    let ros2_paths = [
        "/opt/ros/humble/lib/play_launch/play_launch_io_helper",
        "/usr/lib/play_launch/play_launch_io_helper",
    ];

    for path_str in &ros2_paths {
        let path = PathBuf::from(path_str);
        if path.exists() {
            debug!("Using I/O helper from ROS2 path: {}", path_str);
            check_helper_capabilities(&path)?;
            return Ok(path);
        }
    }

    // 4. Search in PATH
    if let Ok(path) = which::which("play_launch_io_helper") {
        debug!("Using I/O helper from PATH: {:?}", path);
        check_helper_capabilities(&path)?;
        return Ok(path);
    }

    Err(eyre::eyre!(
        "Helper binary not found.\n\
         Searched locations:\n\
         - PLAY_LAUNCH_IO_HELPER environment variable\n\
         - Same directory as play_launch binary\n\
         - /opt/ros/humble/lib/play_launch/\n\
         - /usr/lib/play_launch/\n\
         - PATH\n\
         \n\
         Make sure play_launch_io_helper is installed.\n\
         For pip install: Binary should be in site-packages/play_launch/bin/\n\
         For colcon: Run 'just build'"
    ))
}

/// Check if helper binary has required capabilities
fn check_helper_capabilities(path: &Path) -> Result<()> {
    let output = std::process::Command::new("getcap").arg(path).output();

    match output {
        Ok(output) if output.status.success() => {
            let stdout = String::from_utf8_lossy(&output.stdout);
            if !stdout.contains("cap_sys_ptrace") {
                warn!(
                    "Helper binary does not have CAP_SYS_PTRACE capability set.\n\
                     I/O monitoring for privileged processes will not work.\n\
                     Run: sudo setcap cap_sys_ptrace+ep {:?}",
                    path
                );
            } else {
                debug!("Helper has CAP_SYS_PTRACE: {}", stdout.trim());
            }
            Ok(())
        }
        Ok(_) | Err(_) => {
            // getcap not available or failed - just warn
            warn!(
                "Could not verify capabilities on helper binary.\n\
                 Make sure CAP_SYS_PTRACE is set: sudo setcap cap_sys_ptrace+ep {:?}",
                path
            );
            Ok(())
        }
    }
}

/// Create an anonymous pipe
///
/// Returns (read_fd, write_fd)
fn create_pipe() -> Result<(std::os::unix::io::RawFd, std::os::unix::io::RawFd)> {
    let mut fds = [0i32; 2];

    // SAFETY: pipe() is a standard Unix syscall
    let ret = unsafe { libc::pipe(fds.as_mut_ptr()) };

    if ret != 0 {
        return Err(eyre::eyre!(
            "Failed to create pipe: {}",
            std::io::Error::last_os_error()
        ));
    }

    Ok((fds[0], fds[1]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_pipe() {
        let (read_fd, write_fd) = create_pipe().unwrap();
        assert!(read_fd >= 0);
        assert!(write_fd >= 0);
        assert_ne!(read_fd, write_fd);

        // Cleanup
        unsafe {
            libc::close(read_fd);
            libc::close(write_fd);
        }
    }
}
