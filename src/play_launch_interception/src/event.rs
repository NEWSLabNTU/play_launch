//! Interception event types shared between the interceptor (producer) and
//! play_launch (consumer).
//!
//! These are `#[repr(C)]` + `Copy` so they can be stored directly in the
//! `spsc_shm` ring buffer with zero serialization overhead.

/// Event kind discriminator.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventKind {
    PublisherInit = 0,
    SubscriptionInit = 1,
    Publish = 2,
    Take = 3,
}

/// A single interception event (40 bytes, `#[repr(C)]`).
///
/// Written by plugins in the interception `.so`, read by play_launch's
/// `InterceptionListener` on the consumer side of the SPSC ring buffer.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct InterceptionEvent {
    /// What happened.
    pub kind: EventKind,
    pub _pad: [u8; 3],
    /// FNV-1a hash of the topic name.
    pub topic_hash: u64,
    /// Message `header.stamp.sec` (0 for init events or messages without stamp).
    pub stamp_sec: i32,
    /// Message `header.stamp.nanosec`.
    pub stamp_nanosec: u32,
    /// Opaque handle (publisher/subscription pointer as usize).
    pub handle: u64,
    /// `clock_gettime(CLOCK_MONOTONIC)` in nanoseconds. Used by play_launch to
    /// compute message rates and inter-message latency.
    pub monotonic_ns: u64,
}

const _: () = assert!(size_of::<InterceptionEvent>() == 40);

/// Get the current monotonic clock time in nanoseconds.
#[inline]
pub(crate) fn monotonic_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}
