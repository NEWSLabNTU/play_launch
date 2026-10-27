//! Service-call plugin (phase 95).
//!
//! Ships every service client and server a process creates, and every
//! request sent or taken and response sent or taken, so play_launch can pair
//! a request with its response (`call-unanswered` on the Health page) and
//! time the request leg (`measure`'s `max_transport`). A call is rare next to
//! a message, so this costs nothing on the topic hot path.

use std::sync::Arc;

use parking_lot::Mutex;

use crate::{
    event::{EventKind, InterceptionEvent, monotonic_ns},
    plugin::{InterceptionPlugin, Stamp},
};
use spsc_shm::Producer;

pub(crate) struct CallsPlugin {
    producer: Arc<Mutex<Producer<InterceptionEvent>>>,
}

impl CallsPlugin {
    pub fn new(producer: Arc<Mutex<Producer<InterceptionEvent>>>) -> Self {
        Self { producer }
    }
}

impl InterceptionPlugin for CallsPlugin {
    fn on_publisher_init(&self, _: usize, _: &str, _: u64, _: Option<usize>, _: Option<u64>) {}
    fn on_subscription_init(&self, _: usize, _: &str, _: u64, _: Option<usize>, _: Option<u64>) {}
    fn on_publish(&self, _: usize, _: u64, _: Option<Stamp>) {}
    fn on_take(&self, _: usize, _: u64, _: Option<Stamp>) {}

    fn on_call_endpoint_init(
        &self,
        handle: usize,
        is_client: bool,
        _service: &str,
        service_hash: u64,
        _node_fqn: Option<&str>,
        node_hash: u64,
    ) {
        let event = InterceptionEvent {
            kind: if is_client {
                EventKind::ServiceClientInit
            } else {
                EventKind::ServiceServerInit
            },
            _pad: [0; 3],
            topic_hash: service_hash,
            stamp_sec: 0,
            stamp_nanosec: 0,
            handle: handle as u64,
            monotonic_ns: monotonic_ns(),
            cpu_ns: node_hash,
            tid: 0,
            payload_bytes: 0,
        };
        crate::drop_counter::push_or_count(&mut self.producer.lock(), &event);
    }

    fn on_call(
        &self,
        kind: EventKind,
        handle: usize,
        service_hash: u64,
        seq: i64,
        guid_hash: u64,
        source_ns: i64,
    ) {
        let event =
            InterceptionEvent::call(kind, handle as u64, service_hash, seq, guid_hash, source_ns);
        crate::drop_counter::push_or_count(&mut self.producer.lock(), &event);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A request's sequence number, writer GUID hash and source timestamp
    /// survive the ring in the fields `EventKind::ServiceRequestTaken`
    /// documents; a missing timestamp stays 0/0.
    #[test]
    fn call_events_carry_their_fields() {
        let (shm_fd, _event_fd) = spsc_shm::create::<InterceptionEvent>(16).unwrap();
        let producer = unsafe { spsc_shm::Producer::from_raw_fd(shm_fd) }.unwrap();
        let mut consumer =
            unsafe { spsc_shm::Consumer::<InterceptionEvent>::from_raw_fd(shm_fd) }.unwrap();
        let plugin = CallsPlugin::new(Arc::new(Mutex::new(producer)));
        plugin.on_call_endpoint_init(0x10, true, "/svc", 0xAB, Some("/n"), 0xCD);
        plugin.on_call(
            EventKind::ServiceRequestTaken,
            0x20,
            0xAB,
            7,
            0x1122_3344_5566_7788,
            1_700_000_000_123_456_789,
        );
        plugin.on_call(EventKind::ServiceRequestSent, 0x10, 0xAB, 8, 0, 0);
        let init = consumer.pop().unwrap();
        assert_eq!(init.kind, EventKind::ServiceClientInit);
        assert_eq!(
            (init.handle, init.topic_hash, init.cpu_ns),
            (0x10, 0xAB, 0xCD)
        );
        let take = consumer.pop().unwrap();
        assert_eq!(take.kind, EventKind::ServiceRequestTaken);
        assert_eq!(take.cpu_ns, 7);
        assert_eq!(
            (u64::from(take.tid) << 32) | u64::from(take.payload_bytes),
            0x1122_3344_5566_7788
        );
        assert_eq!(
            (take.stamp_sec, take.stamp_nanosec),
            (1_700_000_000, 123_456_789)
        );
        let sent = consumer.pop().unwrap();
        assert_eq!((sent.cpu_ns, sent.stamp_sec, sent.stamp_nanosec), (8, 0, 0));
    }

    /// What a call hook adds on top of the rcl call it wraps: the endpoint
    /// lookup, the event and the ring push (`cargo test --release --
    /// --ignored call_hook_cost --nocapture`). Phase 95 measured it.
    #[test]
    #[ignore]
    fn call_hook_cost() {
        let (shm_fd, _event_fd) = spsc_shm::create::<InterceptionEvent>(1 << 16).unwrap();
        let producer = unsafe { spsc_shm::Producer::from_raw_fd(shm_fd) }.unwrap();
        let mut consumer =
            unsafe { spsc_shm::Consumer::<InterceptionEvent>::from_raw_fd(shm_fd) }.unwrap();
        let plugin = CallsPlugin::new(Arc::new(Mutex::new(producer)));
        for id in 0..64usize {
            crate::registry::register_call_endpoint(0x1000 + id, id as u64);
        }
        let n = 200_000u64;
        let mut total = std::time::Duration::ZERO;
        for round in 0..(n / 1000) {
            let t = std::time::Instant::now();
            for i in 0..1000u64 {
                let id = 0x1000 + ((round * 1000 + i) % 64) as usize;
                if let Some(h) = crate::registry::lookup_call_endpoint(id) {
                    plugin.on_call(EventKind::ServiceRequestTaken, id, h, i as i64, 7, 1);
                }
            }
            total += t.elapsed();
            while consumer.pop().is_some() {}
        }
        eprintln!(
            "call hook overhead: {:.1} ns per call",
            total.as_nanos() as f64 / n as f64
        );
    }
}
