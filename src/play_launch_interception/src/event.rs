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
    /// DDS event: publisher's offered QoS was found incompatible with a
    /// requesting subscription. Field overload:
    ///   `handle` = rmw_publisher_t*,
    ///   `stamp_sec` = total_count (i32),
    ///   `stamp_nanosec` = total_count_change as u32,
    ///   `_pad[0..2]` = last_policy_kind low 16 bits (little endian),
    ///   `_pad[2]` = 0 (unused).
    OfferedQosIncompatible = 6,
    /// DDS event: subscription's requested QoS was found incompatible.
    /// Same field overload as `OfferedQosIncompatible` with
    /// `handle` = rmw_subscription_t*.
    RequestedQosIncompatible = 7,
    /// DDS event: publisher missed its offered deadline. Field overload:
    ///   `handle` = rmw_publisher_t*,
    ///   `stamp_sec` = total_count,
    ///   `stamp_nanosec` = total_count_change as u32.
    OfferedDeadlineMissed = 8,
    /// DDS event: subscription missed its requested deadline. Same as
    /// `OfferedDeadlineMissed` with `handle` = rmw_subscription_t*.
    RequestedDeadlineMissed = 9,
    /// DDS event: publisher's liveliness was lost. Same as
    /// `OfferedDeadlineMissed` with `handle` = rmw_publisher_t*.
    LivelinessLost = 10,
    /// DDS event: subscription saw a liveliness change. Field overload:
    ///   `handle` = rmw_subscription_t*,
    ///   `stamp_sec` = alive_count,
    ///   `stamp_nanosec` = not_alive_count as u32,
    ///   `_pad[0]` = alive_count_change signed-clamped to i8,
    ///   `_pad[1]` = not_alive_count_change signed-clamped to i8.
    LivelinessChanged = 11,
    /// DDS event: subscription lost message(s). Field overload:
    ///   `handle` = rmw_subscription_t*,
    ///   `stamp_sec` = total_count (saturating cast from usize),
    ///   `stamp_nanosec` = total_count_change (saturating cast from usize).
    MessageLost = 12,
    /// Ships a chunk of a topic FQN string from the .so to the consumer
    /// so graph-deviation-runtime messages can show real topic names
    /// instead of hashes. Each chunk carries 24 bytes of payload; long
    /// names span multiple chunks ordered by `chunk_idx`. Field overload:
    ///   `_pad[0]` = chunk_idx (u8, 0-based),
    ///   `_pad[1]` = total_chunks (u8, 1-based),
    ///   `_pad[2]` = bytes valid in this chunk's payload (1-24),
    ///   `topic_hash` = FQN hash (chunks group by this),
    ///   `stamp_sec`, `stamp_nanosec`, `handle`, `monotonic_ns` reinterpreted
    ///   as 24 raw bytes of UTF-8 payload (little-endian byte order on the
    ///   producer matches the consumer because both run on the same host).
    TopicNameDeclared = 13,
    /// Ships a chunk of a runtime msg-type identity string
    /// (`"pkg/msg/Name"`) keyed by `topic_hash`. Same chunking + field
    /// overload as `TopicNameDeclared`. Lets the consumer build a
    /// `topic_hash → type_string` map for audit + contract backfill.
    TypeNameDeclared = 14,
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
    fn topic_name_chunk_roundtrip_full() {
        let payload = b"/abcd/efgh/ijkl/mnop/qrst"; // 25 bytes — last is dropped
        let e = InterceptionEvent::topic_name_chunk(0x1234, 0, 2, &payload[..24]);
        let (idx, total, n, buf) = e.decode_topic_name_chunk();
        assert_eq!(idx, 0);
        assert_eq!(total, 2);
        assert_eq!(n, 24);
        assert_eq!(&buf[..24], &payload[..24]);
    }

    #[test]
    fn topic_name_chunk_roundtrip_short() {
        let e = InterceptionEvent::topic_name_chunk(0xdeadbeef, 1, 2, b"tail");
        let (idx, total, n, buf) = e.decode_topic_name_chunk();
        assert_eq!(idx, 1);
        assert_eq!(total, 2);
        assert_eq!(n, 4);
        assert_eq!(&buf[..4], b"tail");
        // Padding bytes should be zero.
        assert_eq!(&buf[4..], &[0u8; 20][..]);
        assert_eq!(e.kind, EventKind::TopicNameDeclared);
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

impl InterceptionEvent {
    /// Build an `OfferedQosIncompatible` or `RequestedQosIncompatible`
    /// event. Encodes total counts in stamp fields, policy kind in pad.
    #[inline]
    pub fn qos_incompatible(
        kind: EventKind,
        handle: u64,
        topic_hash: u64,
        total_count: i32,
        total_count_change: i32,
        last_policy_kind: i32,
    ) -> Self {
        debug_assert!(matches!(
            kind,
            EventKind::OfferedQosIncompatible | EventKind::RequestedQosIncompatible
        ));
        let kind_bytes = (last_policy_kind as u16).to_le_bytes();
        Self {
            kind,
            _pad: [kind_bytes[0], kind_bytes[1], 0],
            topic_hash,
            stamp_sec: total_count,
            stamp_nanosec: total_count_change as u32,
            handle,
            monotonic_ns: monotonic_ns(),
        }
    }

    /// Build a deadline-missed or liveliness-lost event (simple
    /// `total_count + total_count_change` payload).
    #[inline]
    pub fn deadline_or_liveliness_lost(
        kind: EventKind,
        handle: u64,
        topic_hash: u64,
        total_count: i32,
        total_count_change: i32,
    ) -> Self {
        debug_assert!(matches!(
            kind,
            EventKind::OfferedDeadlineMissed
                | EventKind::RequestedDeadlineMissed
                | EventKind::LivelinessLost
        ));
        Self {
            kind,
            _pad: [0; 3],
            topic_hash,
            stamp_sec: total_count,
            stamp_nanosec: total_count_change as u32,
            handle,
            monotonic_ns: monotonic_ns(),
        }
    }

    /// Build a `LivelinessChanged` event.
    #[inline]
    pub fn liveliness_changed(
        handle: u64,
        topic_hash: u64,
        alive_count: i32,
        not_alive_count: i32,
        alive_count_change: i32,
        not_alive_count_change: i32,
    ) -> Self {
        Self {
            kind: EventKind::LivelinessChanged,
            _pad: [
                (alive_count_change.clamp(-128, 127) as i8) as u8,
                (not_alive_count_change.clamp(-128, 127) as i8) as u8,
                0,
            ],
            topic_hash,
            stamp_sec: alive_count,
            stamp_nanosec: not_alive_count as u32,
            handle,
            monotonic_ns: monotonic_ns(),
        }
    }

    /// Build a `MessageLost` event from `usize` counters.
    #[inline]
    pub fn message_lost(
        handle: u64,
        topic_hash: u64,
        total_count: usize,
        total_count_change: usize,
    ) -> Self {
        Self {
            kind: EventKind::MessageLost,
            _pad: [0; 3],
            topic_hash,
            stamp_sec: total_count.min(i32::MAX as usize) as i32,
            stamp_nanosec: total_count_change.min(u32::MAX as usize) as u32,
            handle,
            monotonic_ns: monotonic_ns(),
        }
    }
}

/// Chunk size used by `topic_name_chunk` — the 24-byte payload region
/// of the `InterceptionEvent` (stamp_sec + stamp_nanosec + handle +
/// monotonic_ns = 24 bytes).
pub const TOPIC_NAME_CHUNK_BYTES: usize = 24;

impl InterceptionEvent {
    /// Build a `TopicNameDeclared` or `TypeNameDeclared` event carrying
    /// one chunk of a string payload. `payload` must be ≤ 24 bytes;
    /// shorter slices are zero-padded. The whole event is a normal
    /// 40-byte `InterceptionEvent` so it goes through the same SPSC ring.
    #[inline]
    pub fn name_chunk(
        kind: EventKind,
        topic_hash: u64,
        chunk_idx: u8,
        total_chunks: u8,
        payload: &[u8],
    ) -> Self {
        debug_assert!(matches!(
            kind,
            EventKind::TopicNameDeclared | EventKind::TypeNameDeclared
        ));
        debug_assert!(payload.len() <= TOPIC_NAME_CHUNK_BYTES);
        let mut buf = [0u8; TOPIC_NAME_CHUNK_BYTES];
        let n = payload.len().min(TOPIC_NAME_CHUNK_BYTES);
        buf[..n].copy_from_slice(&payload[..n]);
        // Reinterpret the 24-byte buffer as the four fields. SAFETY:
        // `buf` is exactly 24 bytes and the four fields share the same
        // 24-byte layout per `#[repr(C)]`. Endianness is host-native;
        // .so + play_launch always run on the same host.
        let stamp_sec = i32::from_ne_bytes(buf[0..4].try_into().unwrap());
        let stamp_nanosec = u32::from_ne_bytes(buf[4..8].try_into().unwrap());
        let handle = u64::from_ne_bytes(buf[8..16].try_into().unwrap());
        let monotonic_ns = u64::from_ne_bytes(buf[16..24].try_into().unwrap());
        Self {
            kind,
            _pad: [chunk_idx, total_chunks, n as u8],
            topic_hash,
            stamp_sec,
            stamp_nanosec,
            handle,
            monotonic_ns,
        }
    }

    /// Back-compat alias for the `TopicNameDeclared` constructor.
    #[inline]
    pub fn topic_name_chunk(
        topic_hash: u64,
        chunk_idx: u8,
        total_chunks: u8,
        payload: &[u8],
    ) -> Self {
        Self::name_chunk(
            EventKind::TopicNameDeclared,
            topic_hash,
            chunk_idx,
            total_chunks,
            payload,
        )
    }

    /// Inverse of `topic_name_chunk`: copy the 24-byte payload back out
    /// into a fixed buffer plus the chunk metadata. Returns
    /// `(chunk_idx, total_chunks, byte_count, payload_buffer)`.
    #[inline]
    pub fn decode_topic_name_chunk(&self) -> (u8, u8, usize, [u8; TOPIC_NAME_CHUNK_BYTES]) {
        let mut buf = [0u8; TOPIC_NAME_CHUNK_BYTES];
        buf[0..4].copy_from_slice(&self.stamp_sec.to_ne_bytes());
        buf[4..8].copy_from_slice(&self.stamp_nanosec.to_ne_bytes());
        buf[8..16].copy_from_slice(&self.handle.to_ne_bytes());
        buf[16..24].copy_from_slice(&self.monotonic_ns.to_ne_bytes());
        (
            self._pad[0],
            self._pad[1],
            self._pad[2] as usize,
            buf,
        )
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
