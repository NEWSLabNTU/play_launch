//! Stats plugin — writes every publish/take event to the shared memory ring
//! buffer with a monotonic timestamp, enabling play_launch to compute message
//! rates, inter-message latency, and throughput.
//!
//! Always activated when a shared memory producer is available (same env var
//! as FrontierPlugin).

use std::sync::Arc;

use parking_lot::Mutex;

use crate::event::{EventKind, InterceptionEvent, monotonic_ns, thread_cpu_ns, tid};
use crate::plugin::{InterceptionPlugin, Stamp};
use spsc_shm::Producer;

pub(crate) struct StatsPlugin {
    producer: Arc<Mutex<Producer<InterceptionEvent>>>,
}

impl StatsPlugin {
    pub fn new(producer: Arc<Mutex<Producer<InterceptionEvent>>>) -> Self {
        Self { producer }
    }
}

impl InterceptionPlugin for StatsPlugin {
    fn on_publisher_init(
        &self,
        handle: usize,
        _topic: &str,
        topic_hash: u64,
        _stamp_offset: Option<usize>,
        type_hash: Option<u64>,
    ) {
        // Init events have no stamp; reuse the stamp_sec / stamp_nanosec
        // slots to carry the runtime msg-type hash (Phase 36 consistency
        // rule). 0 means "type unknown — introspection unavailable".
        let th = type_hash.unwrap_or(0);
        let event = InterceptionEvent {
            kind: EventKind::PublisherInit,
            _pad: [0; 3],
            topic_hash,
            stamp_sec: (th >> 32) as i32,
            stamp_nanosec: (th & 0xFFFF_FFFF) as u32,
            handle: handle as u64,
            monotonic_ns: monotonic_ns(),
            cpu_ns: 0,
            tid: 0,
            _pad2: [0; 4],
        };
        crate::drop_counter::push_or_count(&mut self.producer.lock(), &event);
    }

    fn on_subscription_init(
        &self,
        handle: usize,
        _topic: &str,
        topic_hash: u64,
        _stamp_offset: Option<usize>,
        type_hash: Option<u64>,
    ) {
        let th = type_hash.unwrap_or(0);
        let event = InterceptionEvent {
            kind: EventKind::SubscriptionInit,
            _pad: [0; 3],
            topic_hash,
            stamp_sec: (th >> 32) as i32,
            stamp_nanosec: (th & 0xFFFF_FFFF) as u32,
            handle: handle as u64,
            monotonic_ns: monotonic_ns(),
            cpu_ns: 0,
            tid: 0,
            _pad2: [0; 4],
        };
        crate::drop_counter::push_or_count(&mut self.producer.lock(), &event);
    }

    fn on_publish(&self, handle: usize, topic_hash: u64, stamp: Option<Stamp>) {
        let (sec, nanosec) = stamp
            .map(|s| (s.sec, s.nanosec))
            .unwrap_or((0, 0));
        let event = InterceptionEvent {
            kind: EventKind::Publish,
            _pad: [0; 3],
            topic_hash,
            stamp_sec: sec,
            stamp_nanosec: nanosec,
            handle: handle as u64,
            monotonic_ns: monotonic_ns(),
            // Phase 58 W2: the publishing thread's consumed CPU time.
            // Differenced against the take that started this path, it is
            // execution cost — what `budget_us` means — as opposed to
            // `monotonic_ns`, whose difference carries preemption too.
            cpu_ns: thread_cpu_ns(),
            tid: tid(),
            _pad2: [0; 4],
        };
        crate::drop_counter::push_or_count(&mut self.producer.lock(), &event);
    }

    fn on_take(&self, handle: usize, topic_hash: u64, stamp: Option<Stamp>) {
        let (sec, nanosec) = stamp
            .map(|s| (s.sec, s.nanosec))
            .unwrap_or((0, 0));
        let event = InterceptionEvent {
            kind: EventKind::Take,
            _pad: [0; 3],
            topic_hash,
            stamp_sec: sec,
            stamp_nanosec: nanosec,
            handle: handle as u64,
            monotonic_ns: monotonic_ns(),
            // Phase 58 W2: see `on_publish`. This is the "start" reading a
            // path's cost is measured from.
            cpu_ns: thread_cpu_ns(),
            tid: tid(),
            _pad2: [0; 4],
        };
        crate::drop_counter::push_or_count(&mut self.producer.lock(), &event);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn setup() -> (StatsPlugin, spsc_shm::Consumer<InterceptionEvent>, i32, i32) {
        let (shm_fd, event_fd) = spsc_shm::create::<InterceptionEvent>(64).unwrap();
        let producer = unsafe { spsc_shm::Producer::from_raw_fd(shm_fd) }.unwrap();
        let consumer = unsafe { spsc_shm::Consumer::<InterceptionEvent>::from_raw_fd(shm_fd) }.unwrap();
        let plugin = StatsPlugin::new(Arc::new(Mutex::new(producer)));
        (plugin, consumer, shm_fd, event_fd)
    }

    #[test]
    fn publish_writes_event() {
        let (plugin, mut consumer, shm_fd, event_fd) = setup();

        plugin.on_publish(0x100, 0xAAAA, Some(Stamp { sec: 10, nanosec: 500 }));

        let ev = consumer.pop().expect("should receive publish event");
        assert_eq!(ev.kind, EventKind::Publish);
        assert_eq!(ev.topic_hash, 0xAAAA);
        assert_eq!(ev.stamp_sec, 10);
        assert_eq!(ev.stamp_nanosec, 500);
        assert_eq!(ev.handle, 0x100);
        assert!(ev.monotonic_ns > 0);

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn take_writes_event() {
        let (plugin, mut consumer, shm_fd, event_fd) = setup();

        plugin.on_take(0x200, 0xBBBB, Some(Stamp { sec: 5, nanosec: 100 }));

        let ev = consumer.pop().expect("should receive take event");
        assert_eq!(ev.kind, EventKind::Take);
        assert_eq!(ev.topic_hash, 0xBBBB);
        assert_eq!(ev.stamp_sec, 5);
        assert_eq!(ev.stamp_nanosec, 100);

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn hot_path_events_carry_a_thread_cpu_reading_and_its_owner() {
        // Phase 58 W2: cost is a CPU-time delta, so both the reading and the
        // thread it belongs to must be on the event. A missing tid would let
        // a consumer subtract two unrelated threads' counters.
        let (plugin, mut consumer, shm_fd, event_fd) = setup();

        plugin.on_take(0x1, 0xAA, Some(Stamp { sec: 1, nanosec: 0 }));
        // Burn measurable CPU between the two hooks.
        let mut acc = 0u64;
        for i in 0..2_000_000u64 {
            acc = acc.wrapping_add(i);
        }
        std::hint::black_box(acc);
        plugin.on_publish(0x2, 0xBB, Some(Stamp { sec: 1, nanosec: 0 }));

        let take = consumer.pop().expect("take event");
        let publish = consumer.pop().expect("publish event");
        assert!(take.cpu_ns > 0, "take carried no CPU reading");
        assert!(
            publish.cpu_ns > take.cpu_ns,
            "CPU time must advance across work: {} -> {}",
            take.cpu_ns,
            publish.cpu_ns
        );
        assert_ne!(take.tid, 0);
        assert_eq!(take.tid, publish.tid, "same thread, same tid");

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn init_events_carry_no_cpu_reading() {
        // Init happens once, off the hot path, and has no invocation to
        // attribute cost to — a reading there would only invite differencing
        // it against something.
        let (plugin, mut consumer, shm_fd, event_fd) = setup();
        plugin.on_publisher_init(0x1, "/t", 0xAA, None, None);
        let ev = consumer.pop().expect("init event");
        assert_eq!(ev.cpu_ns, 0);
        assert_eq!(ev.tid, 0);
        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn publisher_init_writes_event() {
        let (plugin, mut consumer, shm_fd, event_fd) = setup();

        plugin.on_publisher_init(0x300, "/chatter", 0xCCCC, Some(16), Some(0xABCD_1234_5678));

        let ev = consumer.pop().expect("should receive init event");
        assert_eq!(ev.kind, EventKind::PublisherInit);
        assert_eq!(ev.topic_hash, 0xCCCC);
        assert_eq!(ev.handle, 0x300);
        // type_hash 0xABCD_1234_5678 splits across stamp_sec/stamp_nanosec
        let combined = ((ev.stamp_sec as u32 as u64) << 32) | (ev.stamp_nanosec as u64);
        assert_eq!(combined, 0xABCD_1234_5678);

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn subscription_init_writes_event() {
        let (plugin, mut consumer, shm_fd, event_fd) = setup();

        plugin.on_subscription_init(0x400, "/listener", 0xDDDD, Some(8), None);

        let ev = consumer.pop().expect("should receive init event");
        assert_eq!(ev.kind, EventKind::SubscriptionInit);
        assert_eq!(ev.topic_hash, 0xDDDD);
        assert_eq!(ev.handle, 0x400);

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn none_stamp_writes_zero() {
        let (plugin, mut consumer, shm_fd, event_fd) = setup();

        plugin.on_publish(0x100, 0xEEEE, None);
        let ev = consumer.pop().expect("should still write event");
        assert_eq!(ev.stamp_sec, 0);
        assert_eq!(ev.stamp_nanosec, 0);

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }

    #[test]
    fn monotonic_timestamps_increase() {
        let (plugin, mut consumer, shm_fd, event_fd) = setup();

        plugin.on_publish(0x100, 0x1111, Some(Stamp { sec: 1, nanosec: 0 }));
        plugin.on_publish(0x100, 0x1111, Some(Stamp { sec: 2, nanosec: 0 }));

        let ev1 = consumer.pop().unwrap();
        let ev2 = consumer.pop().unwrap();
        assert!(ev2.monotonic_ns >= ev1.monotonic_ns, "monotonic timestamps should not go backwards");

        unsafe {
            libc::close(shm_fd);
            libc::close(event_fd);
        }
    }
}
