//! Plugin dispatch — iterates compiled-in trait objects.

use crate::plugin::{InterceptionPlugin, Stamp};

/// Dispatch a publisher init event to all plugins.
#[inline]
pub(crate) fn dispatch_publisher_init(
    plugins: &[Box<dyn InterceptionPlugin>],
    handle: usize,
    topic: &str,
    topic_hash: u64,
    stamp_offset: Option<usize>,
    type_hash: Option<u64>,
) {
    for p in plugins {
        p.on_publisher_init(handle, topic, topic_hash, stamp_offset, type_hash);
    }
}

/// Dispatch a subscription init event to all plugins.
#[inline]
pub(crate) fn dispatch_subscription_init(
    plugins: &[Box<dyn InterceptionPlugin>],
    handle: usize,
    topic: &str,
    topic_hash: u64,
    stamp_offset: Option<usize>,
    type_hash: Option<u64>,
) {
    for p in plugins {
        p.on_subscription_init(handle, topic, topic_hash, stamp_offset, type_hash);
    }
}

/// Dispatch a publish event to all plugins. `payload_bytes` is the
/// message's serialized size, 0 when not measured.
#[inline]
pub(crate) fn dispatch_publish(
    plugins: &[Box<dyn InterceptionPlugin>],
    handle: usize,
    topic_hash: u64,
    stamp: Option<Stamp>,
    payload_bytes: u32,
) {
    for p in plugins {
        p.on_publish_sized(handle, topic_hash, stamp, payload_bytes);
    }
}

/// Dispatch a take event to all plugins. See [`dispatch_publish`].
#[inline]
pub(crate) fn dispatch_take(
    plugins: &[Box<dyn InterceptionPlugin>],
    handle: usize,
    topic_hash: u64,
    stamp: Option<Stamp>,
    payload_bytes: u32,
) {
    for p in plugins {
        p.on_take_sized(handle, topic_hash, stamp, payload_bytes);
    }
}

/// Dispatch the owning node of a newly created endpoint to all plugins.
#[inline]
pub(crate) fn dispatch_endpoint_owner(
    plugins: &[Box<dyn InterceptionPlugin>],
    handle: usize,
    node_fqn: &str,
    node_hash: u64,
) {
    for p in plugins {
        p.on_endpoint_owner(handle, node_fqn, node_hash);
    }
}
