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
    /// Called once per publisher creation (cold path). `type_hash` is
    /// the fnv1a hash of the runtime message type identity
    /// (`"pkg/msg/Name"`), or `None` if introspection isn't available.
    fn on_publisher_init(
        &self,
        handle: usize,
        topic: &str,
        topic_hash: u64,
        stamp_offset: Option<usize>,
        type_hash: Option<u64>,
    );

    /// Called once per subscription creation (cold path). See
    /// [`on_publisher_init`] for the `type_hash` semantics.
    fn on_subscription_init(
        &self,
        handle: usize,
        topic: &str,
        topic_hash: u64,
        stamp_offset: Option<usize>,
        type_hash: Option<u64>,
    );

    /// Called on every `rcl_publish` (hot path). `stamp` is `None` if the
    /// message has no `header.stamp`.
    fn on_publish(&self, handle: usize, topic_hash: u64, stamp: Option<Stamp>);

    /// Called on every `rcl_take` (hot path). `stamp` is `None` if the
    /// message has no `header.stamp`.
    fn on_take(&self, handle: usize, topic_hash: u64, stamp: Option<Stamp>);

    // -----------------------------------------------------------------
    // RMW-layer hooks (Phase 36.1). Default no-op so existing plugins
    // (FrontierPlugin, StatsPlugin) compile unchanged. New plugins
    // (QosNegotiationPlugin, DropMonitorPlugin) override these.
    // -----------------------------------------------------------------

    /// Called once per `rmw_create_publisher` (cold path).
    ///
    /// `qos` is the parsed policy (reliability, durability, history,
    /// depth, liveliness). `qos_hash` is a stable hash of the raw QoS
    /// bytes useful for fast profile-equality checks.
    fn on_rmw_publisher_created(
        &self,
        _rmw_pub_handle: usize,
        _topic: &str,
        _topic_hash: u64,
        _qos_hash: u64,
        _qos: Option<&rcl_interception_sys::qos::RmwQosProfile>,
    ) {
    }

    /// Called once per `rmw_create_subscription` (cold path).
    fn on_rmw_subscription_created(
        &self,
        _rmw_sub_handle: usize,
        _topic: &str,
        _topic_hash: u64,
        _qos_hash: u64,
        _qos: Option<&rcl_interception_sys::qos::RmwQosProfile>,
    ) {
    }

    /// Called on every `rmw_publish` (hot path). Lower-level than
    /// `on_publish` — sees the DDS write, not the rcl message.
    fn on_rmw_publish(&self, _rmw_pub_handle: usize, _monotonic_ns: u64) {}

    /// Called on every `rmw_take_with_info` (hot path). `taken` reflects
    /// whether a sample was actually available.
    fn on_rmw_take(&self, _rmw_sub_handle: usize, _monotonic_ns: u64, _taken: bool) {}
}
