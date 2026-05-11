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
    /// Phase 36.2: rmw_create_publisher fired. Field overload:
    ///   `_pad[0]` = reliability, `_pad[1]` = durability, `_pad[2]` = history,
    ///   `stamp_sec` = liveliness, `stamp_nanosec` = depth (u32),
    ///   `handle` = rmw_publisher_t*, `topic_hash` = topic FQN hash.
    QosDeclaredPub = 4,
    /// Phase 36.2: rmw_create_subscription fired. Same field overload as
    /// `QosDeclaredPub` with `handle` = rmw_subscription_t*.
    QosDeclaredSub = 5,
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

#[cfg(test)]
mod tests {
    use super::*;
    use rcl_interception_sys::qos::{durability, history, liveliness, reliability};

    #[test]
    fn qos_declared_event_field_overload_roundtrip() {
        let e = InterceptionEvent::qos_declared(
            EventKind::QosDeclaredPub,
            0xDEADBEEF,
            0x1234_5678_9ABC_DEF0,
            reliability::RELIABLE,
            durability::TRANSIENT_LOCAL,
            history::KEEP_LAST,
            liveliness::AUTOMATIC,
            5,
        );
        assert_eq!(e.kind, EventKind::QosDeclaredPub);
        assert_eq!(e._pad[0], reliability::RELIABLE as u8);
        assert_eq!(e._pad[1], durability::TRANSIENT_LOCAL as u8);
        assert_eq!(e._pad[2], history::KEEP_LAST as u8);
        assert_eq!(e.handle, 0xDEADBEEF);
        assert_eq!(e.topic_hash, 0x1234_5678_9ABC_DEF0);
        assert_eq!(e.stamp_sec, liveliness::AUTOMATIC);
        assert_eq!(e.stamp_nanosec, 5);
    }

    #[test]
    fn qos_declared_sub_kind() {
        let e = InterceptionEvent::qos_declared(
            EventKind::QosDeclaredSub,
            0,
            0,
            reliability::BEST_EFFORT,
            durability::VOLATILE,
            history::KEEP_LAST,
            liveliness::AUTOMATIC,
            10,
        );
        assert_eq!(e.kind, EventKind::QosDeclaredSub);
        assert_eq!(e._pad[0], reliability::BEST_EFFORT as u8);
    }
}

impl InterceptionEvent {
    /// Build a `QosDeclaredPub` or `QosDeclaredSub` event from a parsed
    /// QoS profile. `kind` must be one of the two QoS-declared variants.
    #[inline]
    pub fn qos_declared(
        kind: EventKind,
        handle: u64,
        topic_hash: u64,
        reliability: i32,
        durability: i32,
        history: i32,
        liveliness: i32,
        depth: u32,
    ) -> Self {
        debug_assert!(matches!(
            kind,
            EventKind::QosDeclaredPub | EventKind::QosDeclaredSub
        ));
        Self {
            kind,
            // Saturating clamp to u8 — enum discriminants are small (≤4).
            _pad: [
                reliability.clamp(0, 255) as u8,
                durability.clamp(0, 255) as u8,
                history.clamp(0, 255) as u8,
            ],
            topic_hash,
            stamp_sec: liveliness,
            stamp_nanosec: depth,
            handle,
            monotonic_ns: monotonic_ns(),
        }
    }
}

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
