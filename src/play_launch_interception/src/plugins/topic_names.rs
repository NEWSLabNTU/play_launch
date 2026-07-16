//! Topic-name announcement plugin (Phase 36).
//!
//! Ships each unique topic FQN from the .so to the play_launch
//! consumer via the SPSC ring so graph-deviation-runtime violations
//! can identify topics by name instead of just by hash. Each topic
//! string is emitted exactly once per process via one-or-more
//! `TopicNameDeclared` chunk events.

use std::sync::Arc;

use parking_lot::Mutex;

use crate::event::{EventKind, InterceptionEvent, TOPIC_NAME_CHUNK_BYTES};
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

    /// Emit a UTF-8 string as 1+ chunk events keyed by `topic_hash`.
    /// Used by both `announce_topic_name` and `announce_type_name`.
    fn emit_chunks(&self, kind: EventKind, topic_hash: u64, payload: &[u8]) {
        let total = payload.len().div_ceil(TOPIC_NAME_CHUNK_BYTES).max(1);
        // Cap at 255 chunks (u8). Strings that long are pathological;
        // truncate the tail rather than silently corrupting the ring.
        let total = total.min(255);
        let mut producer = self.producer.lock();
        for idx in 0..total {
            let start = idx * TOPIC_NAME_CHUNK_BYTES;
            let end = (start + TOPIC_NAME_CHUNK_BYTES).min(payload.len());
            let event =
                InterceptionEvent::name_chunk(kind, topic_hash, idx as u8, total as u8, &payload[start..end]);
            crate::drop_counter::push_or_count(&mut producer, &event);
        }
    }

    /// Emit one topic FQN. No-op when the hash was already announced
    /// earlier in this process.
    fn announce_topic_name(&self, topic_hash: u64, topic: &str) {
        if !registry::mark_topic_name_for_emission(topic_hash) {
            return;
        }
        self.emit_chunks(EventKind::TopicNameDeclared, topic_hash, topic.as_bytes());
    }

    /// Emit one runtime msg-type identity (`"pkg/msg/Name"`) keyed by
    /// the topic_hash it belongs to. Same per-process dedupe as
    /// `announce_topic_name` but with a separate set.
    fn announce_type_name(&self, topic_hash: u64, type_identity: &str) {
        if !registry::mark_type_name_for_emission(topic_hash) {
            return;
        }
        self.emit_chunks(
            EventKind::TypeNameDeclared,
            topic_hash,
            type_identity.as_bytes(),
        );
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
        self.announce_topic_name(topic_hash, topic);
    }

    fn on_subscription_init(
        &self,
        _handle: usize,
        topic: &str,
        topic_hash: u64,
        _stamp_offset: Option<usize>,
        _type_hash: Option<u64>,
    ) {
        self.announce_topic_name(topic_hash, topic);
    }

    fn on_publish(&self, _handle: usize, _topic_hash: u64, _stamp: Option<Stamp>) {}
    fn on_take(&self, _handle: usize, _topic_hash: u64, _stamp: Option<Stamp>) {}

    fn on_type_identity_resolved(&self, topic_hash: u64, type_identity: &str) {
        self.announce_type_name(topic_hash, type_identity);
    }
}
