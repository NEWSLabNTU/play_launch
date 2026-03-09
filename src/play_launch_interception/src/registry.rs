//! Publisher / subscription registry.
//!
//! Maps opaque `rcl_publisher_t*` / `rcl_subscription_t*` pointer values to
//! the metadata needed on the hot path: topic hash, stamp offset, and (for
//! publishers) a frontier reference.

use std::collections::HashMap;
use std::sync::LazyLock;
use std::sync::atomic::AtomicU64;

use parking_lot::RwLock;

use crate::frontier;

// ---------------------------------------------------------------------------
// FNV-1a hash (64-bit)
// ---------------------------------------------------------------------------

const FNV_OFFSET: u64 = 14695981039346656037;
const FNV_PRIME: u64 = 1099511628211;

fn fnv1a(bytes: &[u8]) -> u64 {
    let mut hash = FNV_OFFSET;
    for &b in bytes {
        hash ^= b as u64;
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    hash
}

// ---------------------------------------------------------------------------
// Publisher registry
// ---------------------------------------------------------------------------

/// Info stored per publisher for the hot path.
#[derive(Clone, Copy)]
pub struct PubEntry {
    pub topic_hash: u64,
    pub stamp_offset: usize,
    pub frontier: &'static AtomicU64,
}

struct PubRecord {
    topic_hash: u64,
    stamp_offset: Option<usize>,
    frontier: &'static AtomicU64,
}

static PUB_REGISTRY: LazyLock<RwLock<HashMap<usize, PubRecord>>> =
    LazyLock::new(|| RwLock::new(HashMap::new()));

/// Register a publisher. Called from `rcl_publisher_init` hook (cold path).
pub fn register_publisher(ptr: usize, topic_name: &str, stamp_offset: Option<usize>) {
    let topic_hash = fnv1a(topic_name.as_bytes());
    let frontier = frontier::get_or_create(topic_hash);
    let mut map = PUB_REGISTRY.write();
    map.insert(
        ptr,
        PubRecord {
            topic_hash,
            stamp_offset,
            frontier,
        },
    );
}

/// Look up a publisher by pointer. Returns `Some(PubEntry)` only if the
/// publisher is registered AND its message type has a `header.stamp`.
/// Returns `None` otherwise — the hot path should skip this message.
pub fn lookup_publisher(ptr: usize) -> Option<PubEntry> {
    let map = PUB_REGISTRY.read();
    let record = map.get(&ptr)?;
    Some(PubEntry {
        topic_hash: record.topic_hash,
        stamp_offset: record.stamp_offset?,
        frontier: record.frontier,
    })
}

// ---------------------------------------------------------------------------
// Subscription registry
// ---------------------------------------------------------------------------

/// Info stored per subscription for the hot path.
#[derive(Clone, Copy)]
pub struct SubEntry {
    pub topic_hash: u64,
    pub stamp_offset: usize,
}

struct SubRecord {
    topic_hash: u64,
    stamp_offset: Option<usize>,
}

static SUB_REGISTRY: LazyLock<RwLock<HashMap<usize, SubRecord>>> =
    LazyLock::new(|| RwLock::new(HashMap::new()));

/// Register a subscription. Called from `rcl_subscription_init` hook (cold path).
pub fn register_subscription(ptr: usize, topic_name: &str, stamp_offset: Option<usize>) {
    let topic_hash = fnv1a(topic_name.as_bytes());
    let mut map = SUB_REGISTRY.write();
    map.insert(ptr, SubRecord { topic_hash, stamp_offset });
}

/// Look up a subscription by pointer. Returns `Some(SubEntry)` only if the
/// subscription is registered AND its message type has a `header.stamp`.
pub fn lookup_subscription(ptr: usize) -> Option<SubEntry> {
    let map = SUB_REGISTRY.read();
    let record = map.get(&ptr)?;
    Some(SubEntry {
        topic_hash: record.topic_hash,
        stamp_offset: record.stamp_offset?,
    })
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
        register_publisher(0x1000, "/test_pub", Some(16));
        let entry = lookup_publisher(0x1000).unwrap();
        assert_eq!(entry.stamp_offset, 16);
        assert_eq!(entry.topic_hash, fnv1a(b"/test_pub"));
    }

    #[test]
    fn publisher_without_stamp_returns_none() {
        register_publisher(0x2000, "/no_stamp", None);
        assert!(lookup_publisher(0x2000).is_none());
    }

    #[test]
    fn subscription_round_trip() {
        register_subscription(0x3000, "/test_sub", Some(8));
        let entry = lookup_subscription(0x3000).unwrap();
        assert_eq!(entry.stamp_offset, 8);
        assert_eq!(entry.topic_hash, fnv1a(b"/test_sub"));
    }
}
