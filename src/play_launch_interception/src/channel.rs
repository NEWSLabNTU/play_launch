//! Unix datagram side-channel for frontier events.
//!
//! Opens a non-blocking, unbound Unix datagram socket connected to the path
//! in `PLAY_LAUNCH_INTERCEPTION_SOCKET`. Events are best-effort — dropped
//! datagrams are acceptable since frontiers are monotonically increasing.

use std::os::unix::net::UnixDatagram;
use std::sync::OnceLock;

/// A frontier event sent over the side-channel.
///
/// 17 bytes total (packed). Matches the layout expected by play_launch's
/// `FrontierListener`.
#[repr(C, packed)]
pub struct FrontierEvent {
    /// FNV-1a hash of the topic name.
    pub topic_hash: u64,
    /// Seconds component of `header.stamp`.
    pub stamp_sec: i32,
    /// Nanoseconds component of `header.stamp`.
    pub stamp_nanosec: u32,
    /// Event type: 0 = publish, 1 = take.
    pub event_type: u8,
}

pub const EVENT_PUBLISH: u8 = 0;
pub const EVENT_TAKE: u8 = 1;

/// Connected, non-blocking Unix datagram socket. `None` if the env var was
/// not set or socket creation failed.
static SOCKET: OnceLock<UnixDatagram> = OnceLock::new();

/// Initialize the side-channel from `PLAY_LAUNCH_INTERCEPTION_SOCKET`.
///
/// Returns `true` if the socket was created and connected successfully.
/// Must be called exactly once from the `#[ctor]` init function.
pub fn init() -> bool {
    let path = match std::env::var_os("PLAY_LAUNCH_INTERCEPTION_SOCKET") {
        Some(p) => p,
        None => return false,
    };

    let sock = match UnixDatagram::unbound() {
        Ok(s) => s,
        Err(_) => return false,
    };

    if sock.connect(&path).is_err() {
        return false;
    }

    if sock.set_nonblocking(true).is_err() {
        return false;
    }

    SOCKET.set(sock).is_ok()
}

/// Send a frontier event over the side-channel.
///
/// No-op if the channel was not initialized. Silently drops the event on
/// `EAGAIN` / `ENOBUFS` (best-effort).
#[inline]
pub fn send(event: &FrontierEvent) {
    let Some(sock) = SOCKET.get() else {
        return;
    };
    let bytes = unsafe {
        std::slice::from_raw_parts(
            event as *const FrontierEvent as *const u8,
            size_of::<FrontierEvent>(),
        )
    };
    // Ignore errors — best-effort.
    let _ = sock.send(bytes);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn event_size_is_17_bytes() {
        assert_eq!(size_of::<FrontierEvent>(), 17);
    }
}
