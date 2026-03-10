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
) {
    for p in plugins {
        p.on_publisher_init(handle, topic, topic_hash, stamp_offset);
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
) {
    for p in plugins {
        p.on_subscription_init(handle, topic, topic_hash, stamp_offset);
    }
}

/// Dispatch a publish event to all plugins.
#[inline]
pub(crate) fn dispatch_publish(
    plugins: &[Box<dyn InterceptionPlugin>],
    handle: usize,
    topic_hash: u64,
    stamp: Option<Stamp>,
) {
    for p in plugins {
        p.on_publish(handle, topic_hash, stamp);
    }
}

/// Dispatch a take event to all plugins.
#[inline]
pub(crate) fn dispatch_take(
    plugins: &[Box<dyn InterceptionPlugin>],
    handle: usize,
    topic_hash: u64,
    stamp: Option<Stamp>,
) {
    for p in plugins {
        p.on_take(handle, topic_hash, stamp);
    }
}
