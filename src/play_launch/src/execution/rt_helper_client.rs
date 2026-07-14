//! Client for communicating with the `play_launch_rt_helper` daemon
//! (phase 38.10, wave 3).
//!
//! The RT helper runs as a privileged child process holding `CAP_SYS_NICE`,
//! allowing it to apply Linux real-time scheduling (policy/priority/CPU
//! affinity) to processes on behalf of the unprivileged main `play_launch`
//! process, which spawns nodes without elevated capabilities.
//!
//! Structurally this mirrors `monitoring::io_helper_client::IoHelperClient`
//! (anonymous pipes, `kill_on_drop`, `Drop`, graceful `shutdown()`), but over
//! the independent `ipc::sched_protocol` wire protocol — see that module's
//! doc comment for why the two helpers must not share a protocol enum.
//!
//! [`RtHelperClient`] is `&mut self`-based over ordering-correlated pipes,
//! so a single client cannot be shared by N concurrent actors. [`SchedHelper`]
//! wraps it in an owner task reachable via an `mpsc` channel, giving a
//! `Clone + Debug` handle that many actors can hold at once (required because
//! `ActorConfig` derives `Debug`, and an `Arc<Mutex<..>>` would not satisfy
//! that on a type that isn't itself `Debug`).

use eyre::{Context, Result};
use play_launch::ipc::{SchedRequest, SchedResponse, decode_message, encode_message};
use play_launch::sched::{AppliedTier, SchedApplyError};
use std::{os::unix::io::FromRawFd, path::PathBuf, process::Stdio, time::Duration};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    process::{Child, Command},
    sync::{mpsc, oneshot},
};
use tracing::{debug, warn};

/// Delay after spawning helper to let it initialize before first ping
const HELPER_INIT_DELAY: Duration = Duration::from_millis(100);

/// Timeout for helper to exit gracefully during shutdown
const HELPER_SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(2);

/// Per-request timeout. A wedged or dead helper must surface as an error,
/// never hang the launch — every `send_request` call is wrapped in this.
const RT_REQUEST_TIMEOUT: Duration = Duration::from_secs(5);

/// Maximum IPC response size (1 MB)
const MAX_IPC_RESPONSE_SIZE: usize = 1_048_576;

/// Direct (non-cloneable) client for the RT helper daemon. See module docs —
/// most callers want [`SchedHelper`] instead, unless they specifically need
/// single-owner access (e.g. the owner task itself).
pub struct RtHelperClient {
    request_writer: tokio::fs::File, // Parent writes requests here
    response_reader: tokio::fs::File, // Parent reads responses here
    child: Option<Child>,
}

impl RtHelperClient {
    /// Spawn the helper daemon and connect via pipes.
    ///
    /// This will:
    /// 1. Find the helper binary (env override → exe dir → ROS path → PATH)
    /// 2. Create anonymous pipes for bidirectional communication
    /// 3. Spawn helper with FDs passed as arguments
    /// 4. Verify with a ping
    pub async fn spawn() -> Result<Self> {
        let helper_path = find_helper_binary()?;

        debug!("Spawning RT helper: {:?}", helper_path);

        // Pipe 1: Parent writes requests → Helper reads requests
        let (request_reader_fd, request_writer_fd) = create_pipe()?;
        // Pipe 2: Helper writes responses → Parent reads responses
        let (response_reader_fd, response_writer_fd) = create_pipe()?;

        debug!(
            "Created pipes: request_pipe=[{},{}], response_pipe=[{},{}]",
            request_reader_fd, request_writer_fd, response_reader_fd, response_writer_fd
        );

        let child = Command::new(&helper_path)
            .arg("--request-fd")
            .arg(request_reader_fd.to_string())
            .arg("--response-fd")
            .arg(response_writer_fd.to_string())
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::inherit()) // Let helper errors show in our stderr
            .kill_on_drop(true) // Reaped on panic/unwind — no zombie
            .spawn()
            .wrap_err_with(|| format!("Failed to spawn RT helper: {:?}", helper_path))?;

        // SAFETY: We just created these FDs via pipe(), we own them
        let request_writer_file = unsafe { std::fs::File::from_raw_fd(request_writer_fd) };
        let response_reader_file = unsafe { std::fs::File::from_raw_fd(response_reader_fd) };

        // Close child-side FDs in parent (helper has its own copies)
        unsafe {
            libc::close(request_reader_fd);
            libc::close(response_writer_fd);
        }

        let request_writer = tokio::fs::File::from_std(request_writer_file);
        let response_reader = tokio::fs::File::from_std(response_reader_file);

        let mut client = Self {
            request_writer,
            response_reader,
            child: Some(child),
        };

        // Give helper a moment to initialize
        tokio::time::sleep(HELPER_INIT_DELAY).await;

        // Verify helper is working
        client.ping().await?;

        debug!("RT helper connected successfully via pipes");

        Ok(client)
    }

    /// Send a ping to verify helper is alive.
    pub async fn ping(&mut self) -> Result<()> {
        let response = self.send_request(SchedRequest::Ping).await?;
        match response {
            SchedResponse::Pong => Ok(()),
            other => Err(eyre::eyre!("Unexpected response to ping: {:?}", other)),
        }
    }

    /// Apply `tier` to every thread of `pid` via the helper.
    ///
    /// IPC-level failures (encode/decode errors, broken pipe, timeout) are
    /// mapped to `SchedApplyError::Syscall` with a synthetic `rt_helper_ipc`
    /// call name and `errno: 0` — there is no real errno for "the helper
    /// didn't answer", but callers only branch on the variant shape (Warn
    /// logs and continues, Strict aborts), so this is sufficient without
    /// growing the shared error enum for a helper-transport-only condition.
    pub async fn apply(&mut self, pid: u32, tier: &AppliedTier) -> Result<(), SchedApplyError> {
        let request = SchedRequest::ApplySched {
            pid,
            tier: tier.clone(),
        };

        match self.send_request(request).await {
            Ok(SchedResponse::Applied(result)) => result,
            Ok(other) => {
                warn!("Unexpected response to ApplySched: {:?}", other);
                Err(ipc_error(pid))
            }
            Err(e) => {
                warn!("RT helper ApplySched request failed: {:#}", e);
                Err(ipc_error(pid))
            }
        }
    }

    /// Send a request and receive a response, bounded by [`RT_REQUEST_TIMEOUT`]
    /// so a wedged/dead helper surfaces as an error instead of hanging.
    async fn send_request(&mut self, request: SchedRequest) -> Result<SchedResponse> {
        match tokio::time::timeout(RT_REQUEST_TIMEOUT, self.send_request_untimed(request)).await {
            Ok(result) => result,
            Err(_) => Err(eyre::eyre!(
                "RT helper request timed out after {:?}",
                RT_REQUEST_TIMEOUT
            )),
        }
    }

    async fn send_request_untimed(&mut self, request: SchedRequest) -> Result<SchedResponse> {
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

        if msg_len > MAX_IPC_RESPONSE_SIZE {
            return Err(eyre::eyre!("Response too large: {} bytes", msg_len));
        }

        // Read response payload
        let mut resp_buf = vec![0u8; msg_len];
        self.response_reader
            .read_exact(&mut resp_buf)
            .await
            .wrap_err("Failed to read response payload")?;

        // Decode response
        let response: SchedResponse =
            decode_message(&resp_buf).wrap_err("Failed to decode response")?;

        Ok(response)
    }

    /// Gracefully shut down the helper: `Shutdown` → `ShutdownAck` →
    /// `child.wait()` with a timeout → escalate to `start_kill()` if the
    /// helper doesn't exit in time.
    pub async fn shutdown(mut self) -> Result<()> {
        debug!("Sending shutdown request to RT helper");
        if let Err(e) = self.send_request(SchedRequest::Shutdown).await {
            warn!("Failed to send shutdown request to RT helper: {}", e);
        }

        if let Some(mut child) = self.child.take() {
            tokio::select! {
                _ = tokio::time::sleep(HELPER_SHUTDOWN_TIMEOUT) => {
                    warn!("RT helper did not exit gracefully, killing");
                    let _ = child.kill().await;
                }
                result = child.wait() => {
                    if let Err(e) = result {
                        warn!("Error waiting for RT helper: {}", e);
                    }
                }
            }
        }

        // Pipes are automatically closed when File handles are dropped.
        Ok(())
    }
}

impl Drop for RtHelperClient {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            // Force kill helper if still alive (belt-and-suspenders — the
            // helper also sets PR_SET_PDEATHSIG itself, so an orphan should
            // never happen even if this Drop never ran).
            debug!("Killing RT helper process in drop");
            let _ = child.start_kill();
        }
    }
}

fn ipc_error(pid: u32) -> SchedApplyError {
    SchedApplyError::Syscall {
        pid,
        call: "rt_helper_ipc".to_string(),
        errno: 0,
    }
}

/// One outstanding `ApplySched` job routed through the [`SchedHelper`] owner
/// task.
struct SchedJob {
    pid: u32,
    tier: AppliedTier,
    reply: oneshot::Sender<Result<(), SchedApplyError>>,
}

/// Cloneable handle to the RT helper.
///
/// Backed by an `mpsc::Sender<SchedJob>` (not an `Arc<Mutex<RtHelperClient>>`)
/// so that:
/// - it is trivially `Clone` and `Debug` — `ActorConfig` derives `Debug`, and
///   an `Arc<tokio::Mutex<RtHelperClient>>` would not satisfy that without a
///   manual, potentially misleading `Debug` impl on the client itself;
/// - all traffic to the single-owner, ordering-correlated pipe pair is
///   serialized through one owner task, never interleaved across actors.
///
/// `Debug` is implemented manually (not derived) since `SchedJob` — the
/// channel's element type — intentionally doesn't derive `Debug` (it carries
/// a `oneshot::Sender`, which isn't diagnostically interesting to print).
#[derive(Clone)]
pub struct SchedHelper(mpsc::Sender<SchedJob>);

impl std::fmt::Debug for SchedHelper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SchedHelper").finish_non_exhaustive()
    }
}

/// Bound on the owner task's job queue. Generous relative to expected
/// concurrency (regular nodes + containers + composables applying roughly
/// once each at spawn/LOADED time) — this is not a steady-state stream.
const SCHED_JOB_QUEUE_SIZE: usize = 64;

impl SchedHelper {
    /// Spawn the RT helper and an owner task that serializes all traffic to
    /// it. Returns the cloneable handle plus the owner task's `JoinHandle`
    /// (the caller should hold onto the latter to await graceful shutdown).
    pub async fn spawn() -> Result<(SchedHelper, tokio::task::JoinHandle<()>)> {
        let mut client = RtHelperClient::spawn().await?;
        let (tx, mut rx) = mpsc::channel::<SchedJob>(SCHED_JOB_QUEUE_SIZE);

        let owner = tokio::spawn(async move {
            while let Some(job) = rx.recv().await {
                let result = client.apply(job.pid, &job.tier).await;
                // Ignore send errors: the caller may have given up waiting
                // (e.g. its own timeout elapsed first), which is fine.
                let _ = job.reply.send(result);
            }

            // All `SchedHelper` clones (senders) have been dropped — every
            // actor that held one has exited. Shut the helper down
            // gracefully rather than just letting `client` (and its
            // kill_on_drop child) fall out of scope.
            debug!("SchedHelper owner task: channel closed, shutting down RT helper");
            if let Err(e) = client.shutdown().await {
                warn!("RT helper graceful shutdown failed: {:#}", e);
            }
        });

        Ok((SchedHelper(tx), owner))
    }

    /// Apply `tier` to every thread of `pid`, routed through the owner task.
    ///
    /// If the owner task has already exited (channel closed, or it dropped
    /// our reply `oneshot` without answering — which shouldn't happen but is
    /// handled defensively), this returns a [`SchedApplyError`] rather than
    /// panicking.
    pub async fn apply(&self, pid: u32, tier: &AppliedTier) -> Result<(), SchedApplyError> {
        let (reply_tx, reply_rx) = oneshot::channel();
        let job = SchedJob {
            pid,
            tier: tier.clone(),
            reply: reply_tx,
        };

        if self.0.send(job).await.is_err() {
            warn!("RT helper owner task is gone; cannot apply sched for pid {pid}");
            return Err(ipc_error(pid));
        }

        match reply_rx.await {
            Ok(result) => result,
            Err(_) => {
                warn!(
                    "RT helper owner task dropped the reply channel for pid {pid} without answering"
                );
                Err(ipc_error(pid))
            }
        }
    }
}

/// Apply `tier` to `pid`, preferring the RT helper (non-root path) and
/// falling back to a direct in-process `apply_tier` call when no helper is
/// configured (keeps `sudo -E play_launch ...` working exactly as before).
///
/// Used by all three phase-38 apply sites (regular node, container,
/// composable-on-LOADED) so the helper-vs-direct choice isn't triplicated.
/// Each site's existing Off/Warn/Strict semantics are unchanged by this —
/// this function only decides HOW to apply, not WHETHER to.
pub async fn apply_sched(
    helper: Option<&SchedHelper>,
    pid: u32,
    tier: &AppliedTier,
) -> Result<(), SchedApplyError> {
    match helper {
        Some(h) => h.apply(pid, tier).await,
        None => crate::execution::sched_apply::apply_tier(pid, tier),
    }
}

/// Find the helper binary with fallback chain:
/// 1. `PLAY_LAUNCH_RT_HELPER` environment variable (development override)
/// 2. Same directory as current executable (pip install or colcon)
/// 3. ROS2 install paths (`/opt/ros/$ROS_DISTRO/lib/play_launch/`)
/// 4. PATH search (fallback — prefer the branches above; a bare `which`
///    resolution can land on an unrelated shim if one is ever installed
///    under this binary name, as happens today for `play_launch_io_helper`)
pub(crate) fn find_helper_binary() -> Result<PathBuf> {
    // 1. Check environment variable (development override)
    if let Ok(path) = std::env::var("PLAY_LAUNCH_RT_HELPER") {
        let p = PathBuf::from(&path);
        if p.exists() {
            debug!("Using RT helper from environment: {}", path);
            return Ok(p);
        }
        warn!("PLAY_LAUNCH_RT_HELPER set but not found: {}", path);
    }

    // 2. Check same directory as current executable (pip install or colcon layout)
    if let Ok(exe_path) = std::env::current_exe()
        && let Some(exe_dir) = exe_path.parent()
    {
        let helper_path = exe_dir.join("play_launch_rt_helper");
        if helper_path.exists() {
            debug!("Using RT helper from exe dir: {:?}", helper_path);
            return Ok(helper_path);
        }
    }

    // 3. Check ROS2 install locations (use $ROS_DISTRO if available, fallback to humble)
    let distro = std::env::var("ROS_DISTRO").unwrap_or_else(|_| "humble".to_string());
    let ros2_paths = [
        format!("/opt/ros/{}/lib/play_launch/play_launch_rt_helper", distro),
        "/usr/lib/play_launch/play_launch_rt_helper".to_string(),
    ];

    for path_str in &ros2_paths {
        let path = PathBuf::from(path_str.as_str());
        if path.exists() {
            debug!("Using RT helper from ROS2 path: {}", path_str);
            return Ok(path);
        }
    }

    // 4. Search in PATH
    if let Ok(path) = which::which("play_launch_rt_helper") {
        debug!("Using RT helper from PATH: {:?}", path);
        return Ok(path);
    }

    Err(eyre::eyre!(
        "RT helper binary not found.\n\
         Searched locations:\n\
         - PLAY_LAUNCH_RT_HELPER environment variable\n\
         - Same directory as play_launch binary\n\
         - /opt/ros/{distro}/lib/play_launch/\n\
         - /usr/lib/play_launch/\n\
         - PATH\n\
         \n\
         Make sure play_launch_rt_helper is installed.\n\
         For pip install: Binary should be in site-packages/play_launch/bin/\n\
         For colcon: Run 'just build'",
        distro = distro
    ))
}

/// Create an anonymous pipe. Returns (read_fd, write_fd).
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

        unsafe {
            libc::close(read_fd);
            libc::close(write_fd);
        }
    }

    fn fifo_tier(priority: i32) -> AppliedTier {
        AppliedTier {
            policy: play_launch::sched::SchedPolicy::Fifo,
            priority,
            core: None,
            tier_name: "test".to_string(),
        }
    }

    /// Full round-trip: main → IPC → helper → syscall → error back.
    ///
    /// This process is unprivileged in CI/dev environments (no CAP_SYS_NICE,
    /// not root), so `apply()` is expected to fail with `PermissionDenied` —
    /// the point of the test is that the failure comes back as a real
    /// `SchedApplyError` over IPC, not that scheduling actually applies.
    /// Skips gracefully if `play_launch_rt_helper` isn't found in this test
    /// environment (e.g. plain `cargo test` without the ROS/colcon overlay
    /// on `PATH` — see `find_helper_binary`'s search order).
    #[tokio::test]
    async fn sched_helper_round_trip_unprivileged() {
        if play_launch::sched::has_sched_privilege() {
            eprintln!(
                "skipping sched_helper_round_trip_unprivileged: running with CAP_SYS_NICE/root"
            );
            return;
        }

        let (helper, owner) = match SchedHelper::spawn().await {
            Ok(pair) => pair,
            Err(e) => {
                eprintln!(
                    "skipping sched_helper_round_trip_unprivileged: RT helper unavailable: {:#}",
                    e
                );
                return;
            }
        };

        let mut child = tokio::process::Command::new("sleep")
            .arg("30")
            .spawn()
            .expect("failed to spawn sleep child");
        let pid = child.id().expect("child should have a pid");

        // Give the kernel a moment to finish exec() before poking at it.
        tokio::time::sleep(Duration::from_millis(50)).await;

        let result = helper.apply(pid, &fifo_tier(10)).await;
        assert_eq!(result, Err(SchedApplyError::PermissionDenied { pid }));

        // Reap the child so no orphan leaks regardless of test outcome.
        let _ = child.kill().await;
        let _ = child.wait().await;

        // Drop the last handle so the owner task sees the channel close and
        // shuts the helper down gracefully; bound the wait so a stuck owner
        // task can't hang the test suite.
        drop(helper);
        let _ = tokio::time::timeout(Duration::from_secs(5), owner).await;
    }
}
