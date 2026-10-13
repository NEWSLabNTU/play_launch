//! Publisher / subscription registry.
//!
//! Maps opaque `rcl_publisher_t*` / `rcl_subscription_t*` pointer values to
//! the metadata needed on the hot path: topic hash, stamp offset, and when
//! the next message is due a size sample.
//! Frontier tracking is now handled by plugins, not the registry.

use std::{
    collections::HashMap,
    sync::{
        LazyLock,
        atomic::{AtomicU64, Ordering},
    },
};

use parking_lot::RwLock;

// ---------------------------------------------------------------------------
// FNV-1a hash (64-bit)
// ---------------------------------------------------------------------------

const FNV_OFFSET: u64 = 14695981039346656037;
const FNV_PRIME: u64 = 1099511628211;

pub fn fnv1a(bytes: &[u8]) -> u64 {
    let mut hash = FNV_OFFSET;
    for &b in bytes {
        hash ^= b as u64;
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    hash
}

// ---------------------------------------------------------------------------
// Publisher / subscription registries
// ---------------------------------------------------------------------------

/// What the publish/take hooks need about an endpoint, per message.
#[derive(Clone, Copy)]
pub struct MsgEntry {
    pub topic_hash: u64,
    /// Byte offset of `header.stamp` (or a top-level `stamp`), when the
    /// message type has one.
    pub stamp_offset: Option<usize>,
    /// The endpoint's `rosidl_message_type_support_t*`, present only when
    /// THIS message is due a size sample (see [`size_sample_interval_ns`]).
    pub sample_with: Option<usize>,
}

struct EndpointRecord {
    topic_hash: u64,
    stamp_offset: Option<usize>,
    /// `rosidl_message_type_support_t*` as passed to `rcl_*_init`. rcl keeps
    /// it for the endpoint's lifetime, so it stays valid while the handle is.
    type_support: usize,
    /// Monotonic time before which no further size sample is taken.
    next_sample_ns: AtomicU64,
}

static PUB_REGISTRY: LazyLock<RwLock<HashMap<usize, EndpointRecord>>> =
    LazyLock::new(|| RwLock::new(HashMap::new()));

static SUB_REGISTRY: LazyLock<RwLock<HashMap<usize, EndpointRecord>>> =
    LazyLock::new(|| RwLock::new(HashMap::new()));

/// How often each endpoint serializes one message to learn its size, from
/// `PLAY_LAUNCH_INTERCEPTION_SIZE_SAMPLE_MS` (default 1000; 0 turns sampling
/// off). One sample a second per endpoint keeps the cost bounded however fast
/// the topic runs — a 30 Hz point cloud is serialized once, not thirty times.
pub fn size_sample_interval_ns() -> Option<u64> {
    static INTERVAL: LazyLock<Option<u64>> = LazyLock::new(|| {
        let ms = std::env::var("PLAY_LAUNCH_INTERCEPTION_SIZE_SAMPLE_MS")
            .ok()
            .and_then(|v| v.trim().parse::<u64>().ok())
            .unwrap_or(1000);
        (ms > 0).then(|| ms * 1_000_000)
    });
    *INTERVAL
}

fn register(
    map: &RwLock<HashMap<usize, EndpointRecord>>,
    ptr: usize,
    topic_name: &str,
    stamp_offset: Option<usize>,
    type_support: usize,
) {
    map.write().insert(
        ptr,
        EndpointRecord {
            topic_hash: fnv1a(topic_name.as_bytes()),
            stamp_offset,
            type_support,
            next_sample_ns: AtomicU64::new(0),
        },
    );
}

fn lookup(
    map: &RwLock<HashMap<usize, EndpointRecord>>,
    ptr: usize,
    now_ns: u64,
) -> Option<MsgEntry> {
    let map = map.read();
    let record = map.get(&ptr)?;
    let sample_with = match size_sample_interval_ns() {
        Some(interval) if record.type_support != 0 => {
            let due = record.next_sample_ns.load(Ordering::Relaxed);
            // The exchange makes the claim exclusive: of several threads
            // publishing on one endpoint at the due moment, one samples.
            (now_ns >= due
                && record
                    .next_sample_ns
                    .compare_exchange(due, now_ns + interval, Ordering::Relaxed, Ordering::Relaxed)
                    .is_ok())
            .then_some(record.type_support)
        }
        _ => None,
    };
    Some(MsgEntry {
        topic_hash: record.topic_hash,
        stamp_offset: record.stamp_offset,
        sample_with,
    })
}

/// Register a publisher. Called from `rcl_publisher_init` hook (cold path).
pub fn register_publisher(
    ptr: usize,
    topic_name: &str,
    stamp_offset: Option<usize>,
    type_support: usize,
) {
    register(&PUB_REGISTRY, ptr, topic_name, stamp_offset, type_support);
}

/// Look up a publisher for one message, claiming a size sample if one is
/// due at `now_ns`. `None` when the publisher was never registered.
pub fn lookup_publisher(ptr: usize, now_ns: u64) -> Option<MsgEntry> {
    lookup(&PUB_REGISTRY, ptr, now_ns)
}

/// Register a subscription. Called from `rcl_subscription_init` hook (cold path).
pub fn register_subscription(
    ptr: usize,
    topic_name: &str,
    stamp_offset: Option<usize>,
    type_support: usize,
) {
    register(&SUB_REGISTRY, ptr, topic_name, stamp_offset, type_support);
}

/// Subscription counterpart of [`lookup_publisher`].
pub fn lookup_subscription(ptr: usize, now_ns: u64) -> Option<MsgEntry> {
    lookup(&SUB_REGISTRY, ptr, now_ns)
}

// ---------------------------------------------------------------------------
// RMW entity registries (Phase 36 DDS events)
// ---------------------------------------------------------------------------
//
// rcl handles (`rcl_publisher_t*`, `rcl_subscription_t*`) are distinct
// from rmw handles (`rmw_publisher_t*`, `rmw_subscription_t*`). The DDS
// event hooks see only rmw handles. Track topic_hash by rmw ptr so
// `rmw_take_event` can correlate to a topic.

static RMW_PUB_REGISTRY: LazyLock<RwLock<HashMap<usize, u64>>> =
    LazyLock::new(|| RwLock::new(HashMap::new()));

static RMW_SUB_REGISTRY: LazyLock<RwLock<HashMap<usize, u64>>> =
    LazyLock::new(|| RwLock::new(HashMap::new()));

pub fn register_rmw_publisher(ptr: usize, topic_hash: u64) {
    RMW_PUB_REGISTRY.write().insert(ptr, topic_hash);
}

pub fn register_rmw_subscription(ptr: usize, topic_hash: u64) {
    RMW_SUB_REGISTRY.write().insert(ptr, topic_hash);
}

pub fn lookup_rmw_publisher(ptr: usize) -> Option<u64> {
    RMW_PUB_REGISTRY.read().get(&ptr).copied()
}

pub fn lookup_rmw_subscription(ptr: usize) -> Option<u64> {
    RMW_SUB_REGISTRY.read().get(&ptr).copied()
}

// ---------------------------------------------------------------------------
// DDS event handle registry — maps `rmw_event_t*` to entity ptr + side
// ---------------------------------------------------------------------------

/// Which side of a topic an event was registered against. Used by
/// `rmw_take_event` to look up topic_hash via the right rmw registry.
#[derive(Clone, Copy, Debug)]
pub enum EventSide {
    Publisher,
    Subscription,
}

#[derive(Clone, Copy, Debug)]
pub struct EventBinding {
    pub side: EventSide,
    pub entity_ptr: usize,
    pub event_type: i32,
}

static EVENT_REGISTRY: LazyLock<RwLock<HashMap<usize, EventBinding>>> =
    LazyLock::new(|| RwLock::new(HashMap::new()));

pub fn register_event(event_ptr: usize, binding: EventBinding) {
    EVENT_REGISTRY.write().insert(event_ptr, binding);
}

pub fn lookup_event(event_ptr: usize) -> Option<EventBinding> {
    EVENT_REGISTRY.read().get(&event_ptr).copied()
}

// ---------------------------------------------------------------------------
// Topic-name emission dedupe (Phase 36)
// ---------------------------------------------------------------------------
//
// We ship each topic FQN string from the .so to the consumer exactly
// once per (topic_hash) per process via TopicNameDeclared chunks. Track
// which hashes we've already emitted so repeated rcl_*_init calls on
// the same topic don't flood the ring.

use std::collections::HashSet;

static TOPIC_NAME_EMITTED: LazyLock<RwLock<HashSet<u64>>> =
    LazyLock::new(|| RwLock::new(HashSet::new()));

/// Returns true if this is the first time we've seen `topic_hash` in
/// this process. On `true`, the caller should emit the topic name to
/// the SPSC ring. Idempotent on subsequent calls with the same hash.
pub fn mark_topic_name_for_emission(topic_hash: u64) -> bool {
    TOPIC_NAME_EMITTED.write().insert(topic_hash)
}

static TYPE_NAME_EMITTED: LazyLock<RwLock<HashSet<u64>>> =
    LazyLock::new(|| RwLock::new(HashSet::new()));

/// Same as [`mark_topic_name_for_emission`] but for the runtime msg-type
/// identity string keyed by `topic_hash`.
pub fn mark_type_name_for_emission(topic_hash: u64) -> bool {
    TYPE_NAME_EMITTED.write().insert(topic_hash)
}

static NODE_NAME_EMITTED: LazyLock<RwLock<HashSet<u64>>> =
    LazyLock::new(|| RwLock::new(HashSet::new()));

/// Same as [`mark_topic_name_for_emission`] for a node FQN keyed by its hash.
pub fn mark_node_name_for_emission(node_hash: u64) -> bool {
    NODE_NAME_EMITTED.write().insert(node_hash)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fnv1a_deterministic() {
        assert_eq!(fnv1a(b"/chatter"), fnv1a(b"/chatter"));
        assert_ne!(fnv1a(b"/chatter"), fnv1a(b"/other"));
    }

    #[test]
    fn publisher_round_trip() {
        register_publisher(0x1000, "/test_pub", Some(16), 0);
        let entry = lookup_publisher(0x1000, 1).unwrap();
        assert_eq!(entry.stamp_offset, Some(16));
        assert_eq!(entry.topic_hash, fnv1a(b"/test_pub"));
    }

    #[test]
    fn publisher_without_stamp_is_still_found() {
        // A message type with no stamp is still traffic: the stats count it.
        register_publisher(0x2000, "/no_stamp", None, 0);
        let entry = lookup_publisher(0x2000, 1).unwrap();
        assert_eq!(entry.topic_hash, fnv1a(b"/no_stamp"));
        assert!(entry.stamp_offset.is_none());
    }

    #[test]
    fn unregistered_endpoint_is_not_found() {
        assert!(lookup_publisher(0xdead_0000, 1).is_none());
        assert!(lookup_subscription(0xdead_0000, 1).is_none());
    }

    #[test]
    fn subscription_round_trip() {
        register_subscription(0x3000, "/test_sub", Some(8), 0);
        let entry = lookup_subscription(0x3000, 1).unwrap();
        assert_eq!(entry.stamp_offset, Some(8));
        assert_eq!(entry.topic_hash, fnv1a(b"/test_sub"));
    }

    #[test]
    fn a_size_sample_is_due_once_per_interval() {
        // Default interval is 1 s (the env override is not set under test).
        let second = 1_000_000_000;
        register_publisher(0x5000, "/sampled", None, 0xabc);
        assert_eq!(
            lookup_publisher(0x5000, 10).unwrap().sample_with,
            Some(0xabc)
        );
        assert_eq!(lookup_publisher(0x5000, 11).unwrap().sample_with, None);
        assert_eq!(
            lookup_publisher(0x5000, 10 + second - 1)
                .unwrap()
                .sample_with,
            None
        );
        assert_eq!(
            lookup_publisher(0x5000, 10 + second).unwrap().sample_with,
            Some(0xabc)
        );
    }

    #[test]
    fn an_endpoint_without_type_support_is_never_sampled() {
        register_subscription(0x6000, "/untyped", None, 0);
        assert_eq!(lookup_subscription(0x6000, 10).unwrap().sample_with, None);
    }
}
