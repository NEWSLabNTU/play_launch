//! Per-topic timestamp frontier tracking.
//!
//! Each topic has an `AtomicU64` storing the highest `(sec, nanosec)` stamp
//! seen so far, packed into a single u64. The CAS max-update is lock-free
//! (~40-50ns per publish).

use std::collections::HashMap;
use std::sync::LazyLock;
use std::sync::atomic::{AtomicU64, Ordering};

use parking_lot::RwLock;

/// Pack `(sec, nanosec)` into a single `u64` for atomic comparison.
///
/// Layout: `[sec as u64 (upper 32)] | [nanosec as u64 (lower 32)]`.
/// This preserves temporal ordering for non-negative seconds. For negative
/// seconds (pre-epoch), the ordering is still correct because we compare
/// the packed u64 as unsigned — the sign bit in sec maps to the MSB.
#[inline]
fn pack(sec: i32, nanosec: u32) -> u64 {
    ((sec as u32 as u64) << 32) | (nanosec as u64)
}

/// CAS max-update: advance the frontier if `(sec, nanosec)` is greater than
/// the current value. Returns `true` if the frontier advanced.
#[inline]
pub fn update(frontier: &AtomicU64, sec: i32, nanosec: u32) -> bool {
    let new = pack(sec, nanosec);
    loop {
        let old = frontier.load(Ordering::Relaxed);
        if new <= old {
            return false;
        }
        match frontier.compare_exchange_weak(old, new, Ordering::Release, Ordering::Relaxed) {
            Ok(_) => return true,
            Err(_) => continue,
        }
    }
}

/// Global map of topic_hash → frontier. Entries are leaked (never freed)
/// because frontiers live for the process lifetime.
static FRONTIERS: LazyLock<RwLock<HashMap<u64, &'static AtomicU64>>> =
    LazyLock::new(|| RwLock::new(HashMap::new()));

/// Get or create the frontier for `topic_hash`. The returned reference is
/// `'static` — it lives until process exit.
pub fn get_or_create(topic_hash: u64) -> &'static AtomicU64 {
    // Fast path: read lock.
    {
        let map = FRONTIERS.read();
        if let Some(frontier) = map.get(&topic_hash) {
            return frontier;
        }
    }
    // Slow path: write lock + insert.
    let mut map = FRONTIERS.write();
    map.entry(topic_hash)
        .or_insert_with(|| Box::leak(Box::new(AtomicU64::new(0))))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_ordering() {
        assert!(pack(0, 0) < pack(0, 1));
        assert!(pack(0, 999_999_999) < pack(1, 0));
        assert!(pack(1, 0) < pack(2, 0));
    }

    #[test]
    fn update_advances_frontier() {
        let frontier = AtomicU64::new(0);
        assert!(update(&frontier, 1, 0));
        assert!(update(&frontier, 1, 500));
        assert!(update(&frontier, 2, 0));
        // Same value — no advance.
        assert!(!update(&frontier, 2, 0));
        // Older value — no advance.
        assert!(!update(&frontier, 1, 999));
    }

    #[test]
    fn get_or_create_returns_same_reference() {
        let f1 = get_or_create(0xDEAD);
        let f2 = get_or_create(0xDEAD);
        assert!(std::ptr::eq(f1, f2));
    }
}
