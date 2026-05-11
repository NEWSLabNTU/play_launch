//! QoS-negotiation plugin (Phase 36.2).
//!
//! On every `rmw_create_publisher` and `rmw_create_subscription` call,
//! emits a `QosDeclared{Pub,Sub}` event to the shared SPSC ring with
//! the resolved QoS policy values (reliability, durability, history,
//! depth, liveliness). The play_launch consumer correlates pub vs sub
//! events on the same topic FQN and runs the offered ≥ requested
//! matrix — the same logic the static `qos-match` rule applies, now at
//! runtime.
//!
//! Activated when a shared memory producer is available (same env var
//! as the other plugins). Inert otherwise.

use std::sync::Arc;

use parking_lot::Mutex;

use crate::event::{EventKind, InterceptionEvent};
use crate::plugin::{InterceptionPlugin, Stamp};
use rcl_interception_sys::qos::RmwQosProfile;
use spsc_shm::Producer;

pub(crate) struct QosNegotiationPlugin {
    producer: Arc<Mutex<Producer<InterceptionEvent>>>,
}

impl QosNegotiationPlugin {
    pub fn new(producer: Arc<Mutex<Producer<InterceptionEvent>>>) -> Self {
        Self { producer }
    }

    /// Lookup a parsed QoS profile by handle. We cache the raw pointer
    /// passed to `on_rmw_*_created` rather than re-reading the C
    /// struct, but for now the RMW layer caller passes only the QoS
    /// hash to the plugin trait — see [`QosNegotiationPlugin::emit`].
    fn emit(&self, kind: EventKind, handle: usize, topic_hash: u64, qos: &RmwQosProfile) {
        let event = InterceptionEvent::qos_declared(
            kind,
            handle as u64,
            topic_hash,
            qos.reliability,
            qos.durability,
            qos.history,
            qos.liveliness,
            qos.depth as u32,
        );
        let _ = self.producer.lock().push(&event);
    }
}

impl InterceptionPlugin for QosNegotiationPlugin {
    // rcl-layer hooks — no-op. We act only at the RMW layer where the
    // actual DDS QoS is known.
    fn on_publisher_init(
        &self,
        _handle: usize,
        _topic: &str,
        _topic_hash: u64,
        _stamp_offset: Option<usize>,
    ) {
    }

    fn on_subscription_init(
        &self,
        _handle: usize,
        _topic: &str,
        _topic_hash: u64,
        _stamp_offset: Option<usize>,
    ) {
    }

    fn on_publish(&self, _handle: usize, _topic_hash: u64, _stamp: Option<Stamp>) {}
    fn on_take(&self, _handle: usize, _topic_hash: u64, _stamp: Option<Stamp>) {}

    fn on_rmw_publisher_created(
        &self,
        handle: usize,
        _topic: &str,
        topic_hash: u64,
        _qos_hash: u64,
        qos: Option<&RmwQosProfile>,
    ) {
        if let Some(q) = qos {
            self.emit(EventKind::QosDeclaredPub, handle, topic_hash, q);
        }
    }

    fn on_rmw_subscription_created(
        &self,
        handle: usize,
        _topic: &str,
        topic_hash: u64,
        _qos_hash: u64,
        qos: Option<&RmwQosProfile>,
    ) {
        if let Some(q) = qos {
            self.emit(EventKind::QosDeclaredSub, handle, topic_hash, q);
        }
    }

    fn on_rmw_publish(&self, _handle: usize, _monotonic_ns: u64) {}
    fn on_rmw_take(&self, _handle: usize, _monotonic_ns: u64, _taken: bool) {}
}
