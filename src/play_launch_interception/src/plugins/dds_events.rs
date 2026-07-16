//! DDS-event plugin (Phase 36 DDS events).
//!
//! Decodes status structs from `rmw_take_event` and forwards them to
//! the consumer-side `RuleEngine` as `InterceptionEvent` variants.
//! Each rmw event_type maps to one event kind:
//!
//! | rmw event_type                   | EventKind                     |
//! | -------------------------------- | ----------------------------- |
//! | REQUESTED_QOS_INCOMPATIBLE       | RequestedQosIncompatible      |
//! | OFFERED_QOS_INCOMPATIBLE         | OfferedQosIncompatible        |
//! | REQUESTED_DEADLINE_MISSED        | RequestedDeadlineMissed       |
//! | OFFERED_DEADLINE_MISSED          | OfferedDeadlineMissed         |
//! | LIVELINESS_LOST                  | LivelinessLost                |
//! | LIVELINESS_CHANGED               | LivelinessChanged             |
//! | MESSAGE_LOST                     | MessageLost                   |
//!
//! Activated alongside the other ring-buffer plugins (StatsPlugin,
//! QosNegotiationPlugin) when a shared memory producer is available.

use std::sync::Arc;

use parking_lot::Mutex;

use crate::event::{EventKind, InterceptionEvent};
use crate::plugin::{InterceptionPlugin, Stamp};
use rcl_interception_sys::events::{
    RmwDeadlineMissedStatus, RmwLivelinessChangedStatus, RmwLivelinessLostStatus,
    RmwMessageLostStatus, RmwQosIncompatibleEventStatus, event_type,
};
use spsc_shm::Producer;

pub(crate) struct DdsEventsPlugin {
    producer: Arc<Mutex<Producer<InterceptionEvent>>>,
}

impl DdsEventsPlugin {
    pub fn new(producer: Arc<Mutex<Producer<InterceptionEvent>>>) -> Self {
        Self { producer }
    }

    fn push(&self, event: InterceptionEvent) {
        crate::drop_counter::push_or_count(&mut self.producer.lock(), &event);
    }
}

impl InterceptionPlugin for DdsEventsPlugin {
    // rcl-layer and rmw create/publish/take hooks: no-op.
    fn on_publisher_init(
        &self,
        _handle: usize,
        _topic: &str,
        _topic_hash: u64,
        _stamp_offset: Option<usize>,
        _type_hash: Option<u64>,
    ) {
    }

    fn on_subscription_init(
        &self,
        _handle: usize,
        _topic: &str,
        _topic_hash: u64,
        _stamp_offset: Option<usize>,
        _type_hash: Option<u64>,
    ) {
    }

    fn on_publish(&self, _handle: usize, _topic_hash: u64, _stamp: Option<Stamp>) {}
    fn on_take(&self, _handle: usize, _topic_hash: u64, _stamp: Option<Stamp>) {}

    fn on_rmw_dds_event(
        &self,
        entity_ptr: usize,
        topic_hash: u64,
        event_type_disc: i32,
        status_ptr: *const u8,
    ) {
        if status_ptr.is_null() {
            return;
        }
        let handle = entity_ptr as u64;

        // SAFETY: the lib.rs hook only calls us after `rmw_take_event`
        // returned RMW_RET_OK with `taken == true`, which guarantees
        // `event_info` has been populated by the RMW implementation
        // with the status struct corresponding to the bound event_type.
        // The pointer is valid for the duration of the hook call.
        let event = unsafe {
            match event_type_disc {
                t if t == event_type::REQUESTED_QOS_INCOMPATIBLE => {
                    let s = &*(status_ptr as *const RmwQosIncompatibleEventStatus);
                    InterceptionEvent::qos_incompatible(
                        EventKind::RequestedQosIncompatible,
                        handle,
                        topic_hash,
                        s.total_count,
                        s.total_count_change,
                        s.last_policy_kind,
                    )
                }
                t if t == event_type::OFFERED_QOS_INCOMPATIBLE => {
                    let s = &*(status_ptr as *const RmwQosIncompatibleEventStatus);
                    InterceptionEvent::qos_incompatible(
                        EventKind::OfferedQosIncompatible,
                        handle,
                        topic_hash,
                        s.total_count,
                        s.total_count_change,
                        s.last_policy_kind,
                    )
                }
                t if t == event_type::REQUESTED_DEADLINE_MISSED => {
                    let s = &*(status_ptr as *const RmwDeadlineMissedStatus);
                    InterceptionEvent::deadline_or_liveliness_lost(
                        EventKind::RequestedDeadlineMissed,
                        handle,
                        topic_hash,
                        s.total_count,
                        s.total_count_change,
                    )
                }
                t if t == event_type::OFFERED_DEADLINE_MISSED => {
                    let s = &*(status_ptr as *const RmwDeadlineMissedStatus);
                    InterceptionEvent::deadline_or_liveliness_lost(
                        EventKind::OfferedDeadlineMissed,
                        handle,
                        topic_hash,
                        s.total_count,
                        s.total_count_change,
                    )
                }
                t if t == event_type::LIVELINESS_LOST => {
                    let s = &*(status_ptr as *const RmwLivelinessLostStatus);
                    InterceptionEvent::deadline_or_liveliness_lost(
                        EventKind::LivelinessLost,
                        handle,
                        topic_hash,
                        s.total_count,
                        s.total_count_change,
                    )
                }
                t if t == event_type::LIVELINESS_CHANGED => {
                    let s = &*(status_ptr as *const RmwLivelinessChangedStatus);
                    InterceptionEvent::liveliness_changed(
                        handle,
                        topic_hash,
                        s.alive_count,
                        s.not_alive_count,
                        s.alive_count_change,
                        s.not_alive_count_change,
                    )
                }
                t if t == event_type::MESSAGE_LOST => {
                    let s = &*(status_ptr as *const RmwMessageLostStatus);
                    InterceptionEvent::message_lost(
                        handle,
                        topic_hash,
                        s.total_count,
                        s.total_count_change,
                    )
                }
                _ => return,
            }
        };
        self.push(event);
    }
}
