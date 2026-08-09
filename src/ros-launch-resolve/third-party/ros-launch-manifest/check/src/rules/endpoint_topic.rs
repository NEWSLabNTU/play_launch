//! Shared helper: resolve a node endpoint reference ("node/endpoint") to
//! the topic that wires it, within a single manifest.
//!
//! Several Vocabulary v2 rules (`once-durability`, `sync-feasibility`,
//! `queue-drain-rate`) need to go from a path's endpoint-name input/output
//! list back to the declared topic (for its `rate_hz`/`qos`). This is a
//! single-manifest, best-effort lookup — cross-scope topic resolution is
//! the `play_launch` `ManifestIndex` machinery's job, not this crate's.

use ros_launch_manifest_types::{Manifest, TopicDecl};

/// Find the topic that has `node_name/ep_name` as one of its publishers.
pub fn topic_for_pub<'m>(
    manifest: &'m Manifest,
    node_name: &str,
    ep_name: &str,
) -> Option<(&'m str, &'m TopicDecl)> {
    let ep_ref = format!("{node_name}/{ep_name}");
    manifest
        .topics
        .iter()
        .find(|(_, t)| t.publishers.iter().any(|p| p == &ep_ref))
        .map(|(name, t)| (name.as_str(), t))
}

/// Find the topic that has `node_name/ep_name` as one of its subscribers.
pub fn topic_for_sub<'m>(
    manifest: &'m Manifest,
    node_name: &str,
    ep_name: &str,
) -> Option<(&'m str, &'m TopicDecl)> {
    let ep_ref = format!("{node_name}/{ep_name}");
    manifest
        .topics
        .iter()
        .find(|(_, t)| t.subscribers.iter().any(|s| s == &ep_ref))
        .map(|(name, t)| (name.as_str(), t))
}
