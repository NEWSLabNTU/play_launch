//! Ships TF edges (`crate::tf`) to play_launch, rate-limited: per
//! (publisher, frame pair), one `TfEdge` event a second carrying the count
//! since the last one, the latest stamp, and the latest transform's value
//! hash with how often it changed. The pair's names go once per process as
//! `TfFramesDeclared` chunks. A `/tf` at 1 kHz with ten frames costs ten
//! events a second, not ten thousand.

use std::{collections::HashMap, sync::Arc};

use parking_lot::Mutex;
use spsc_shm::Producer;

use crate::{
    event::{EventKind, InterceptionEvent, TOPIC_NAME_CHUNK_BYTES},
    plugin::{InterceptionPlugin, Stamp},
    tf::TfSample,
};

const EMIT_EVERY_NS: u64 = 1_000_000_000;

#[derive(Default)]
struct Edge {
    /// A value has been seen, so a different one is a change.
    seen: bool,
    count: u32,
    sec: i32,
    nanosec: u32,
    value_hash: u64,
    changes: u32,
    last_emit_ns: u64,
}

pub(crate) struct TfPlugin {
    producer: Arc<Mutex<Producer<InterceptionEvent>>>,
    edges: Mutex<HashMap<(usize, u64), Edge>>,
    named: Mutex<std::collections::HashSet<u64>>,
}

fn fnv(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for b in bytes {
        h ^= *b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

impl TfPlugin {
    pub fn new(producer: Arc<Mutex<Producer<InterceptionEvent>>>) -> Self {
        Self {
            producer,
            edges: Mutex::new(HashMap::new()),
            named: Mutex::new(Default::default()),
        }
    }

    fn announce(&self, pair_hash: u64, parent: &str, child: &str) {
        if !self.named.lock().insert(pair_hash) {
            return;
        }
        let payload = format!("{parent}\n{child}");
        let bytes = payload.as_bytes();
        let total = bytes.len().div_ceil(TOPIC_NAME_CHUNK_BYTES).clamp(1, 255);
        let mut producer = self.producer.lock();
        for idx in 0..total {
            let start = idx * TOPIC_NAME_CHUNK_BYTES;
            let end = (start + TOPIC_NAME_CHUNK_BYTES).min(bytes.len());
            let event = InterceptionEvent::name_chunk(
                EventKind::TfFramesDeclared,
                pair_hash,
                idx as u8,
                total as u8,
                &bytes[start..end],
            );
            crate::drop_counter::push_or_count(&mut producer, &event);
        }
    }
}

impl InterceptionPlugin for TfPlugin {
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

    /// Lifecycle transitions ride the same plugin: rare, one event each.
    fn on_lifecycle_transition(&self, handle: usize, start: u8, goal: u8) {
        let event = InterceptionEvent {
            kind: EventKind::LifecycleTransition,
            _pad: [start, goal, 0],
            topic_hash: 0,
            stamp_sec: 0,
            stamp_nanosec: 0,
            handle: handle as u64,
            monotonic_ns: crate::event::monotonic_ns(),
            cpu_ns: 0,
            tid: 0,
            payload_bytes: 0,
        };
        crate::drop_counter::push_or_count(&mut self.producer.lock(), &event);
    }

    fn on_tf(&self, handle: usize, is_static: bool, transforms: &[TfSample<'_>]) {
        let now = crate::event::monotonic_ns();
        let mut due = Vec::new();
        {
            let mut edges = self.edges.lock();
            for t in transforms {
                let mut key = Vec::with_capacity(t.parent.len() + t.child.len() + 1);
                key.extend_from_slice(t.parent.as_bytes());
                key.push(b'\n');
                key.extend_from_slice(t.child.as_bytes());
                let pair = fnv(&key);
                let e = edges.entry((handle, pair)).or_default();
                if e.seen && e.value_hash != t.value_hash {
                    e.changes += 1;
                }
                e.seen = true;
                e.count += 1;
                e.sec = t.sec;
                e.nanosec = t.nanosec;
                e.value_hash = t.value_hash;
                if e.last_emit_ns == 0 || now.saturating_sub(e.last_emit_ns) >= EMIT_EVERY_NS {
                    due.push((
                        pair,
                        t.parent.to_string(),
                        t.child.to_string(),
                        std::mem::take(e),
                    ));
                    let fresh = edges.get_mut(&(handle, pair)).unwrap();
                    fresh.last_emit_ns = now;
                    fresh.value_hash = t.value_hash;
                    fresh.seen = true;
                }
            }
        }
        for (pair, parent, child, e) in due {
            self.announce(pair, &parent, &child);
            let event = InterceptionEvent {
                kind: EventKind::TfEdge,
                _pad: [u8::from(is_static), 0, 0],
                topic_hash: pair,
                stamp_sec: e.sec,
                stamp_nanosec: e.nanosec,
                handle: handle as u64,
                monotonic_ns: now,
                cpu_ns: e.value_hash,
                tid: e.changes,
                payload_bytes: e.count,
            };
            crate::drop_counter::push_or_count(&mut self.producer.lock(), &event);
        }
    }
}
