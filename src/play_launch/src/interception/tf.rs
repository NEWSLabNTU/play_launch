//! TF as published (big-system-observability §H), from the interceptor's
//! `TfEdge` events: per (process, publisher, frame pair) a count, the latest
//! stamp, and whether the transform's value ever changed. play_launch does
//! not subscribe to `/tf`; the publishing process reads its own message.

use std::collections::{HashMap, HashSet};

use serde::Serialize;

use super::{InterceptionEvent, TopicNameAssembly, decode_topic_name_chunk};

#[derive(Debug, Clone)]
pub(super) struct Edge {
    pair: u64,
    is_static: bool,
    messages: u64,
    first_mono: u64,
    last_mono: u64,
    stamp_s: f64,
    changes: u64,
    last_change_mono: Option<u64>,
}

#[derive(Debug, Default)]
pub(super) struct TfBook {
    /// (child ring, publisher handle, pair) -> edge.
    edges: HashMap<(usize, u64, u64), Edge>,
    pairs: HashMap<u64, (String, String)>,
    chunks: HashMap<u64, TopicNameAssembly>,
}

impl TfBook {
    pub(super) fn observe_names(&mut self, e: &InterceptionEvent) {
        let (idx, total, n, buf) = decode_topic_name_chunk(e);
        let entry = self
            .chunks
            .entry(e.topic_hash)
            .or_insert_with(|| TopicNameAssembly::new(total.max(1)));
        entry.add(idx, &buf[..n]);
        if let Some(s) = entry.assembled() {
            if let Some((p, c)) = s.split_once('\n') {
                self.pairs
                    .entry(e.topic_hash)
                    .or_insert((p.to_string(), c.to_string()));
            }
            self.chunks.remove(&e.topic_hash);
        }
    }

    pub(super) fn observe_edge(&mut self, child: usize, e: &InterceptionEvent) {
        let edge = self
            .edges
            .entry((child, e.handle, e.topic_hash))
            .or_insert_with(|| Edge {
                pair: e.topic_hash,
                is_static: e._pad[0] != 0,
                messages: 0,
                first_mono: e.monotonic_ns,
                last_mono: e.monotonic_ns,
                stamp_s: 0.0,
                changes: 0,
                last_change_mono: None,
            });
        edge.messages += e.payload_bytes as u64;
        edge.last_mono = e.monotonic_ns;
        edge.stamp_s = e.stamp_sec as f64 + e.stamp_nanosec as f64 / 1e9;
        if e.tid > 0 {
            edge.changes += e.tid as u64;
            edge.last_change_mono = Some(e.monotonic_ns);
        }
    }

    pub(super) fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }

    /// The edges with their publisher's name, `node_of` mapping (ring,
    /// handle) to the node that created the publisher.
    pub(super) fn snapshot(
        &self,
        now_ns: u64,
        node_of: impl Fn(usize, u64) -> Option<String>,
    ) -> TfSnapshot {
        let secs = |ns: u64| ns as f64 / 1e9;
        let mut edges: Vec<TfEdgeView> = self
            .edges
            .iter()
            .filter_map(|((child, handle, _), e)| {
                let (parent, frame) = self.pairs.get(&e.pair)?;
                let life = secs(e.last_mono.saturating_sub(e.first_mono));
                Some(TfEdgeView {
                    parent: parent.clone(),
                    child: frame.clone(),
                    publisher: node_of(*child, *handle),
                    is_static: e.is_static,
                    messages: e.messages,
                    rate_hz: if life >= 1.0 {
                        e.messages as f64 / life
                    } else {
                        0.0
                    },
                    active_s: life,
                    since_last_s: secs(now_ns.saturating_sub(e.last_mono)),
                    stamp_s: e.stamp_s,
                    changes: e.changes,
                    since_change_s: e.last_change_mono.map(|m| secs(now_ns.saturating_sub(m))),
                })
            })
            .collect();
        edges.sort_by(|a, b| a.parent.cmp(&b.parent).then(a.child.cmp(&b.child)));
        TfSnapshot { edges }
    }
}

/// One edge as one publisher sends it.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct TfEdgeView {
    pub parent: String,
    pub child: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub publisher: Option<String>,
    pub is_static: bool,
    pub messages: u64,
    pub rate_hz: f64,
    /// Seconds between the first and the latest message.
    pub active_s: f64,
    pub since_last_s: f64,
    /// The latest transform's stamp, seconds (sim time under sim time).
    pub stamp_s: f64,
    /// Times the transform's value changed.
    pub changes: u64,
    pub since_change_s: Option<f64>,
}

#[derive(Debug, Clone, Default, Serialize, PartialEq)]
pub struct TfSnapshot {
    pub edges: Vec<TfEdgeView>,
}

impl TfSnapshot {
    /// Frames that are a parent and never a child: the tree's roots. A
    /// connected TF tree has one.
    pub fn roots(&self) -> Vec<String> {
        let children: HashSet<&str> = self.edges.iter().map(|e| e.child.as_str()).collect();
        let mut roots: Vec<String> = self
            .edges
            .iter()
            .map(|e| e.parent.as_str())
            .filter(|p| !children.contains(p))
            .map(String::from)
            .collect::<HashSet<_>>()
            .into_iter()
            .collect();
        roots.sort();
        roots
    }
}
