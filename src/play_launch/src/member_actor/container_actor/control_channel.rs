//! Phase 64 — the supervisor half of the private container control channel.
//!
//! One `socketpair(2)` per container process, created before the fork. The
//! supervisor keeps the parent end and hands the child end down as an
//! inherited fd; the container names it through
//! [`CONTROL_FD_ENV`](crate::ipc::container_protocol::CONTROL_FD_ENV).
//!
//! Why an inherited fd rather than a socket file: a unix socket PATH is capped
//! at 108 bytes, which a `play_log/<timestamp>/` directory plus a namespaced
//! container name can exceed; it needs a listener, a bind, a cleanup, and a
//! rendezvous timeout. A socketpair has none of that and gives the exact
//! lifetime we want — the channel exists while the process does, and EOF means
//! the container is gone.
//!
//! `FD_CLOEXEC` stays SET on the supervisor's copy and is cleared only in the
//! child being exec'd (from `pre_exec`, i.e. after that fork). Clearing it in
//! the parent — the way the interception fds are passed — would leak this
//! container's control fd into every unrelated node spawned in the same
//! window, and a leaked copy holds the socket open so the supervisor would
//! never see EOF when the container died.

use crate::ipc::container_protocol::{
    CONTROL_PROTOCOL_VERSION, ContainerMsg, MAX_FRAME_BYTES, SupervisorMsg, encode_frame,
};
use eyre::{Result, eyre};
use std::{
    collections::{HashMap, VecDeque},
    os::fd::{AsRawFd, OwnedFd, RawFd},
};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    sync::mpsc,
};
use tracing::{debug, warn};

/// Capacity of the container→supervisor bridge. Traffic is one accept, one
/// result and an occasional liveness line per composable, so this holds a
/// whole large container's startup several times over.
const INBOUND_CAPACITY: usize = 1024;

/// The child's end of the pair, kept alive until the container is spawned.
///
/// Dropping it closes the fd; that must happen AFTER the spawn (the fork
/// duplicates whatever is open at that moment) and it must happen, or the
/// supervisor holds a write end of the container's own socket and never sees
/// EOF.
pub(super) struct ChildEnd(OwnedFd);

impl ChildEnd {
    pub(super) fn raw(&self) -> RawFd {
        self.0.as_raw_fd()
    }
}

/// What the container told us about itself. Its pid is logged at negotiation
/// rather than stored: the actor already holds the `Child` it spawned, and a
/// socketpair cannot connect anything else.
#[derive(Debug, Clone, Copy)]
pub(super) struct ContainerHello {
    pub(super) loads_over_socket: bool,
}

/// The supervisor's end of one container's control channel.
pub(super) struct ControlChannel {
    outbound: mpsc::UnboundedSender<Vec<u8>>,
    inbound: mpsc::Receiver<ContainerMsg>,
    /// Messages that arrived before the `Hello` was consumed. Nothing should
    /// precede it, but dropping data because it arrived early is the kind of
    /// bug this channel exists to remove.
    buffered: VecDeque<ContainerMsg>,
    hello: Option<ContainerHello>,
    closed: bool,
    next_seq: u64,
    /// seq → composable member name, until the container answers
    /// `Accepted`/`Rejected`. Afterwards the entry's `unique_id` is the key.
    pending: HashMap<u64, String>,
}

impl ControlChannel {
    /// Create the pair and start the reader/writer tasks on the parent end.
    pub(super) fn new(container_name: &str) -> Result<(Self, ChildEnd)> {
        let (parent, child) = std::os::unix::net::UnixStream::pair()
            .map_err(|e| eyre!("socketpair for container control channel: {e}"))?;
        parent
            .set_nonblocking(true)
            .map_err(|e| eyre!("set_nonblocking on control channel: {e}"))?;
        let parent = tokio::net::UnixStream::from_std(parent)
            .map_err(|e| eyre!("tokio::net::UnixStream::from_std: {e}"))?;

        let child_fd: OwnedFd = child.into();

        let (reader, writer) = parent.into_split();
        let (inbound_tx, inbound_rx) = mpsc::channel(INBOUND_CAPACITY);
        let (outbound_tx, outbound_rx) = mpsc::unbounded_channel();

        tokio::spawn(read_loop(container_name.to_string(), reader, inbound_tx));
        tokio::spawn(write_loop(container_name.to_string(), writer, outbound_rx));

        let mut channel = Self {
            outbound: outbound_tx,
            inbound: inbound_rx,
            buffered: VecDeque::new(),
            hello: None,
            closed: false,
            next_seq: 1,
            pending: HashMap::new(),
        };
        channel.send(&SupervisorMsg::Hello {
            protocol: CONTROL_PROTOCOL_VERSION,
            supervisor_pid: std::process::id(),
        });

        Ok((channel, ChildEnd(child_fd)))
    }

    /// Whether loads for this container should go over the socket rather than
    /// the LoadNode service.
    pub(super) fn loads_over_socket(&self) -> bool {
        !self.closed && self.hello.is_some_and(|h| h.loads_over_socket)
    }

    pub(super) fn is_closed(&self) -> bool {
        self.closed
    }

    /// Wait for the container's `Hello`, or give up.
    ///
    /// The container sends it before `rclcpp::init`, so this normally returns
    /// in milliseconds. Giving up is not an error: a container binary from an
    /// older wheel inherits the fd and never speaks, and the LoadNode path is
    /// still there — which is the whole version-skew story.
    pub(super) async fn await_hello(&mut self, timeout: std::time::Duration) -> bool {
        if self.hello.is_some() {
            return true;
        }
        let deadline = tokio::time::Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
            if remaining.is_zero() {
                return false;
            }
            match tokio::time::timeout(remaining, self.inbound.recv()).await {
                Ok(Some(ContainerMsg::Hello {
                    protocol,
                    pid,
                    loads_over_socket,
                })) => {
                    if protocol != CONTROL_PROTOCOL_VERSION {
                        warn!(
                            "container control protocol v{} != supervisor v{} — \
                             falling back to the LoadNode service",
                            protocol, CONTROL_PROTOCOL_VERSION
                        );
                        self.closed = true;
                        return false;
                    }
                    debug!(
                        "container control channel negotiated (v{}, container pid {}, loads over socket: {})",
                        protocol, pid, loads_over_socket
                    );
                    self.hello = Some(ContainerHello { loads_over_socket });
                    return true;
                }
                Ok(Some(other)) => self.buffered.push_back(other),
                Ok(None) => {
                    self.closed = true;
                    return false;
                }
                Err(_) => return false,
            }
        }
    }

    /// Receive the next container message. Cancel-safe (usable in `select!`).
    ///
    /// `None` means the socket closed — the container process is gone, which
    /// the actor learns from `child.wait()` too.
    pub(super) async fn recv(&mut self) -> Option<ContainerMsg> {
        if let Some(msg) = self.buffered.pop_front() {
            return Some(msg);
        }
        let msg = self.inbound.recv().await;
        if msg.is_none() {
            self.closed = true;
        }
        msg
    }

    /// Queue a load request; returns the seq it was filed under.
    pub(super) fn send_load(
        &mut self,
        composable_name: &str,
        msg_fields: LoadFields,
    ) -> Result<u64> {
        if self.closed {
            return Err(eyre!("container control channel is closed"));
        }
        let seq = self.next_seq;
        self.next_seq += 1;
        self.pending.insert(seq, composable_name.to_string());
        self.send(&SupervisorMsg::Load {
            seq,
            package: msg_fields.package,
            plugin: msg_fields.plugin,
            node_name: msg_fields.node_name,
            node_namespace: msg_fields.node_namespace,
            remap_rules: msg_fields.remap_rules,
            parameters: msg_fields.parameters,
            extra_arguments: msg_fields.extra_arguments,
            log_dir: msg_fields.log_dir,
        });
        Ok(seq)
    }

    /// Resolve a `seq` back to the composable that asked for it.
    pub(super) fn take_pending(&mut self, seq: u64) -> Option<String> {
        self.pending.remove(&seq)
    }

    /// Same, without consuming it — a `Status` answered by `seq` arrives while
    /// the load is still waiting to be accepted, and consuming the mapping
    /// there would orphan the `Accepted` that follows.
    pub(super) fn peek_pending(&self, seq: u64) -> Option<&str> {
        self.pending.get(&seq).map(String::as_str)
    }

    /// Ask the container what state a load is in. Either key: `seq` before the
    /// load was acknowledged, `unique_id` after.
    pub(super) fn send_query(&mut self, seq: Option<u64>, unique_id: Option<u64>) {
        self.send(&SupervisorMsg::Query { seq, unique_id });
    }

    /// Ask the container to stop a load and destroy what it created. The
    /// answer — `LoadFailed { cancelled: true }`, or a `Status` saying nothing
    /// is running — is the precondition for a resend.
    pub(super) fn send_cancel(&mut self, unique_id: u64, reason: &str) {
        self.send(&SupervisorMsg::Cancel {
            unique_id,
            reason: reason.to_string(),
        });
    }

    fn send(&mut self, msg: &SupervisorMsg) {
        match encode_frame(msg) {
            Ok(frame) => {
                if self.outbound.send(frame).is_err() {
                    self.closed = true;
                }
            }
            Err(e) => warn!("failed to encode container control frame: {e}"),
        }
    }
}

/// The variable half of a `Load` message — grouped so `send_load` does not
/// take nine positional arguments.
pub(super) struct LoadFields {
    pub(super) package: String,
    pub(super) plugin: String,
    pub(super) node_name: String,
    pub(super) node_namespace: String,
    pub(super) remap_rules: Vec<String>,
    pub(super) parameters: Vec<crate::ipc::container_protocol::ControlParam>,
    pub(super) extra_arguments: Vec<crate::ipc::container_protocol::ControlParam>,
    pub(super) log_dir: String,
}

async fn read_loop(
    container_name: String,
    mut reader: tokio::net::unix::OwnedReadHalf,
    tx: mpsc::Sender<ContainerMsg>,
) {
    let mut len_buf = [0u8; 4];
    loop {
        if reader.read_exact(&mut len_buf).await.is_err() {
            break; // EOF or the container died
        }
        let len = u32::from_le_bytes(len_buf) as usize;
        if len > MAX_FRAME_BYTES {
            warn!(
                "{}: control frame of {} bytes exceeds the {} byte cap — closing the channel",
                container_name, len, MAX_FRAME_BYTES
            );
            break;
        }
        let mut payload = vec![0u8; len];
        if reader.read_exact(&mut payload).await.is_err() {
            break;
        }
        match serde_json::from_slice::<ContainerMsg>(&payload) {
            Ok(msg) => {
                if tx.send(msg).await.is_err() {
                    break; // actor gone
                }
            }
            Err(e) => {
                // A frame we cannot parse is not a reason to tear down a
                // working launch: a newer container may have added a message
                // kind. Skip it and keep reading — framing is intact.
                debug!(
                    "{}: unrecognised control frame ({}): {}",
                    container_name,
                    e,
                    String::from_utf8_lossy(&payload[..payload.len().min(200)])
                );
            }
        }
    }
    debug!("{}: control channel reader finished", container_name);
}

async fn write_loop(
    container_name: String,
    mut writer: tokio::net::unix::OwnedWriteHalf,
    mut rx: mpsc::UnboundedReceiver<Vec<u8>>,
) {
    while let Some(frame) = rx.recv().await {
        if let Err(e) = writer.write_all(&frame).await {
            debug!("{}: control channel write failed: {}", container_name, e);
            break;
        }
    }
    let _ = writer.shutdown().await;
}
