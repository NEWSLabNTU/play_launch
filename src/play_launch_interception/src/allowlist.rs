//! Topic-FQN allowlist for blocking enforcement (Phase 36.7).
//!
//! When `PLAY_LAUNCH_INTERCEPTION_GRAPH_FILE` points at a newline-
//! separated file of fully-qualified topic names, the `rcl_publisher_init`
//! and `rcl_subscription_init` hooks return `RCL_RET_TOPIC_INVALID` for
//! any topic whose name isn't in the file. The hook does not call the
//! original rcl function — the publisher/subscription is never created.
//!
//! Activation is two-step:
//! 1. Set `PLAY_LAUNCH_INTERCEPTION_GRAPH_FILE=/path/to/graph.txt`.
//! 2. Set `PLAY_LAUNCH_INTERCEPTION_BLOCK_GRAPH=1` to enable blocking
//!    (otherwise the file is only used for warn-only check, but in
//!    practice the consumer-side `graph-deviation-runtime` already
//!    covers that — this module is strictly for blocking).
//!
//! Inert when either env var is unset: hooks pass through unchanged.

use std::collections::HashSet;
use std::sync::OnceLock;

use crate::registry::fnv1a;

/// Loaded allowlist. `None` = blocking disabled (hooks pass through).
static ALLOWLIST: OnceLock<Option<HashSet<u64>>> = OnceLock::new();

/// Lazily load the allowlist on first hook invocation. Returns `None`
/// if blocking is not active (env vars not set, file unreadable, or
/// blocking mode disabled).
pub fn try_load() -> Option<&'static HashSet<u64>> {
    ALLOWLIST.get_or_init(|| {
        if std::env::var("PLAY_LAUNCH_INTERCEPTION_BLOCK_GRAPH").ok().as_deref() != Some("1") {
            return None;
        }
        let path = std::env::var("PLAY_LAUNCH_INTERCEPTION_GRAPH_FILE").ok()?;
        let contents = std::fs::read_to_string(&path).ok()?;
        let mut set = HashSet::new();
        for line in contents.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with('#') {
                continue;
            }
            set.insert(fnv1a(trimmed.as_bytes()));
        }
        if set.is_empty() {
            return None;
        }
        eprintln!(
            "[play_launch_interception] blocking allowlist loaded: {} entries from {}",
            set.len(),
            path
        );
        Some(set)
    }).as_ref()
}

/// Check whether `topic_hash` is in the allowlist. Returns `true` when
/// the topic is authorized, `false` when blocking should fire. Always
/// returns `true` when blocking is not active (no allowlist loaded).
#[inline]
pub fn is_allowed(topic_hash: u64) -> bool {
    match try_load() {
        Some(set) => set.contains(&topic_hash),
        None => true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn parse_allowlist_file_format() {
        // Standalone parse logic test — bypasses try_load to avoid the
        // OnceLock global. Verifies file format: newline-separated,
        // comment lines ignored, blank lines ignored.
        let dir = std::env::temp_dir().join(format!(
            "play_launch_allowlist_test_{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("graph.txt");
        let mut f = std::fs::File::create(&path).unwrap();
        writeln!(f, "# header comment").unwrap();
        writeln!(f, "/chatter").unwrap();
        writeln!(f).unwrap();
        writeln!(f, "/sensor/raw").unwrap();
        writeln!(f, "  /trimmed/topic  ").unwrap();
        drop(f);

        // Re-implement parse path here (try_load's OnceLock prevents
        // re-running with new state inside one test process).
        let contents = std::fs::read_to_string(&path).unwrap();
        let mut set = HashSet::new();
        for line in contents.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with('#') {
                continue;
            }
            set.insert(fnv1a(trimmed.as_bytes()));
        }
        assert_eq!(set.len(), 3);
        assert!(set.contains(&fnv1a(b"/chatter")));
        assert!(set.contains(&fnv1a(b"/sensor/raw")));
        assert!(set.contains(&fnv1a(b"/trimmed/topic")));

        let _ = std::fs::remove_dir_all(&dir);
    }
}
