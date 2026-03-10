//! Internal trait for interception plugins.
//!
//! All implementations are compiled into the interception `.so` — no dynamic
//! loading. This trait exists for code organization, not for end users.

/// Timestamp extracted from a ROS message's `header.stamp`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Stamp {
    pub sec: i32,
    pub nanosec: u32,
}

/// Internal trait for interception plugins.
pub(crate) trait InterceptionPlugin: Send + Sync {
    /// Called once per publisher creation (cold path).
    fn on_publisher_init(
        &self,
        handle: usize,
        topic: &str,
        topic_hash: u64,
        stamp_offset: Option<usize>,
    );

    /// Called once per subscription creation (cold path).
    fn on_subscription_init(
        &self,
        handle: usize,
        topic: &str,
        topic_hash: u64,
        stamp_offset: Option<usize>,
    );

    /// Called on every `rcl_publish` (hot path). `stamp` is `None` if the
    /// message has no `header.stamp`.
    fn on_publish(&self, handle: usize, topic_hash: u64, stamp: Option<Stamp>);

    /// Called on every `rcl_take` (hot path). `stamp` is `None` if the
    /// message has no `header.stamp`.
    fn on_take(&self, handle: usize, topic_hash: u64, stamp: Option<Stamp>);
}
