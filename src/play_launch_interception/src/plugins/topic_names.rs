//! Topic-name announcement plugin (Phase 36).
//!
//! Ships each unique topic FQN from the .so to the play_launch
//! consumer via the SPSC ring so graph-deviation-runtime violations
//! can identify topics by name instead of just by hash. Each topic
//! string is emitted exactly once per process via one-or-more
//! `TopicNameDeclared` chunk events.

use std::sync::Arc;

use parking_lot::Mutex;

use crate::event::{InterceptionEvent, TOPIC_NAME_CHUNK_BYTES};
use crate::plugin::{InterceptionPlugin, Stamp};
use crate::registry;
use spsc_shm::Producer;

pub(crate) struct TopicNamesPlugin {
    producer: Arc<Mutex<Producer<InterceptionEvent>>>,
}

impl TopicNamesPlugin {
    pub fn new(producer: Arc<Mutex<Producer<InterceptionEvent>>>) -> Self {
        Self { producer }
    }

    /// Emit one topic FQN as 1+ TopicNameDeclared chunks. No-op when
    /// the hash was already announced earlier in this process.
    fn announce(&self, topic_hash: u64, topic: &str) {
        if !registry::mark_topic_name_for_emission(topic_hash) {
            return;
        }
        let bytes = topic.as_bytes();
        let total = bytes.len().div_ceil(TOPIC_NAME_CHUNK_BYTES).max(1);
        // Cap at 255 chunks (u8). Topic names that long are pathological;
        // truncate the tail rather than silently corrupting the ring.
        let total = total.min(255);
        let mut producer = self.producer.lock();
        for idx in 0..total {
            let start = idx * TOPIC_NAME_CHUNK_BYTES;
            let end = (start + TOPIC_NAME_CHUNK_BYTES).min(bytes.len());
            let chunk = &bytes[start..end];
            let event = InterceptionEvent::topic_name_chunk(
                topic_hash,
                idx as u8,
                total as u8,
                chunk,
            );
            let _ = producer.push(&event);
        }
    }
}

impl InterceptionPlugin for TopicNamesPlugin {
    fn on_publisher_init(
        &self,
        _handle: usize,
        topic: &str,
        topic_hash: u64,
        _stamp_offset: Option<usize>,
        _type_hash: Option<u64>,
    ) {
        self.announce(topic_hash, topic);
    }

    fn on_subscription_init(
        &self,
        _handle: usize,
        topic: &str,
        topic_hash: u64,
        _stamp_offset: Option<usize>,
        _type_hash: Option<u64>,
    ) {
        self.announce(topic_hash, topic);
    }

    fn on_publish(&self, _handle: usize, _topic_hash: u64, _stamp: Option<Stamp>) {}
    fn on_take(&self, _handle: usize, _topic_hash: u64, _stamp: Option<Stamp>) {}
}
