//! Per-endpoint traffic: every publisher and subscription a run created, with
//! how much went through it — including the ones nothing went through.
//!
//! `stats_summary.json` is keyed by TOPIC and built from traffic, so it can
//! say neither which endpoint on a topic carried the load nor that one carried
//! nothing: an endpoint with no messages produces no event that would put it
//! there. This keys by ENDPOINT — `(ring, handle)`, the rcl handle being unique
//! within one process and each process having its own ring — and is seeded by
//! the init event, so an endpoint exists here from the moment it was created.
//!
//! Size comes from `Publish`/`Take` events' `payload_bytes`: exact on every
//! message of the serialized paths, sampled about once a second per endpoint
//! on the typed and loaned paths (the `.so` serializes one message through
//! `rmw_serialize`). Total bytes are therefore an ESTIMATE, the message count
//! times the mean sampled size, and the output says how many samples it rests
//! on. A message whose size varies wildly between samples (a point cloud whose
//! point count changes) makes the estimate correspondingly rough.
//!
//! Blind spot: rclcpp's intra-process transport hands a message from a
//! publisher to a subscription in the same process without going through rcl,
//! so neither end sees it. With intra-process comms on, an endpoint reported
//! silent may not be.

use std::{collections::HashMap, path::Path};

use serde::Serialize;

use super::{EventKind, InterceptionEvent, TopicNameAssembly, decode_topic_name_chunk};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
#[serde(rename_all = "lowercase")]
pub(crate) enum Direction {
    Pub,
    Sub,
}

#[derive(Debug, Clone)]
pub(super) struct Endpoint {
    /// Model key of the process whose ring reported it.
    pub(super) member: String,
    pub(super) node_hash: Option<u64>,
    pub(super) direction: Direction,
    pub(super) topic_hash: u64,
    /// Monotonic time of the init event; the first message's when the init
    /// event was lost to ring overflow.
    pub(super) created_ns: u64,
    /// `true` when the endpoint was first seen through traffic rather than
    /// its init event, so `created_ns` is late and the rate is an upper bound.
    pub(super) seen_without_init: bool,
    pub(super) messages: u64,
    pub(super) last_message_ns: u64,
    pub(super) sampled_bytes: u64,
    pub(super) size_samples: u64,
    /// The last ten seconds, for the live view.
    pub(super) window: super::live::Window,
    /// The QoS the middleware applied, from `EndpointQos`.
    pub(super) qos: Option<super::live::Qos>,
}

impl Endpoint {
    fn new(
        member: &str,
        node_hash: Option<u64>,
        direction: Direction,
        topic_hash: u64,
        at_ns: u64,
    ) -> Self {
        Self {
            member: member.to_string(),
            node_hash,
            direction,
            topic_hash,
            created_ns: at_ns,
            seen_without_init: false,
            messages: 0,
            last_message_ns: 0,
            sampled_bytes: 0,
            size_samples: 0,
            window: Default::default(),
            qos: None,
        }
    }
}

/// One row of `endpoint_stats.json`.
#[derive(Debug, Serialize, PartialEq)]
pub(crate) struct EndpointRow {
    /// FQN of the node that created the endpoint; absent when rcl could not
    /// name it (or the announcement was lost to ring overflow).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub node: Option<String>,
    /// Model key of the process the endpoint lives in. For a container this
    /// is the container, and `node` says which composable.
    pub member: String,
    pub direction: Direction,
    /// Topic FQN; absent only when its announcement was lost.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub topic: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub msg_type: Option<String>,
    pub messages: u64,
    /// Seconds from creation to the end of the run.
    pub lifetime_s: f64,
    /// `messages / lifetime_s`: the average over the endpoint's whole life,
    /// silences included.
    pub rate_hz: f64,
    /// Seconds between the last message and the end of the run; absent for
    /// an endpoint that never carried one. A large value on a busy endpoint
    /// means it went quiet.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub since_last_message_s: Option<f64>,
    /// Mean serialized message size over `size_samples` measured messages.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mean_message_bytes: Option<f64>,
    pub size_samples: u64,
    /// `messages * mean_message_bytes`. Exact when every message was
    /// measured (the serialized paths), an estimate otherwise.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub est_total_bytes: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub est_bytes_per_s: Option<f64>,
    /// The endpoint was first seen through a message, its init event having
    /// been lost to ring overflow, so its lifetime is understated.
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub created_late: bool,
}

/// Aggregates per-endpoint traffic from every child's ring.
#[derive(Debug, Default)]
pub(crate) struct EndpointTracker {
    /// `(child index, handle) -> node hash`, from `EndpointOwner`, which
    /// precedes the endpoint's init event.
    pub(super) owners: HashMap<(usize, u64), u64>,
    pub(super) live: HashMap<(usize, u64), Endpoint>,
    /// Endpoints whose handle was later reused by a new endpoint.
    pub(super) retired: Vec<Endpoint>,
    pub(super) node_names: HashMap<u64, String>,
    pub(super) node_chunks: HashMap<u64, TopicNameAssembly>,
    /// `(child index, handle) -> QoS`, from `EndpointQos`, which also
    /// precedes the init event.
    pub(super) pending_qos: HashMap<(usize, u64), super::live::Qos>,
    /// `(child, topic hash, direction)` the middleware itself reported an
    /// incompatible-QoS event for.
    pub(super) dds_incompatible: std::collections::HashSet<(usize, u64, Direction)>,
}

impl EndpointTracker {
    pub(crate) fn observe(&mut self, child: usize, member: &str, event: &InterceptionEvent) {
        let key = (child, event.handle);
        match event.kind {
            EventKind::EndpointOwner => {
                self.owners.insert(key, event.topic_hash);
            }
            EventKind::NodeNameDeclared => {
                let (idx, total, n, buf) = decode_topic_name_chunk(event);
                let entry = self
                    .node_chunks
                    .entry(event.topic_hash)
                    .or_insert_with(|| TopicNameAssembly::new(total.max(1)));
                entry.add(idx, &buf[..n]);
                if let Some(name) = entry.assembled() {
                    self.node_names.entry(event.topic_hash).or_insert(name);
                    self.node_chunks.remove(&event.topic_hash);
                }
            }
            EventKind::EndpointQos => {
                let qos = super::live::Qos::from_event(event);
                if let Some(ep) = self.live.get_mut(&key)
                    && ep.topic_hash == event.topic_hash
                {
                    ep.qos = Some(qos);
                } else {
                    self.pending_qos.insert(key, qos);
                }
            }
            EventKind::OfferedQosIncompatible => {
                self.dds_incompatible
                    .insert((child, event.topic_hash, Direction::Pub));
            }
            EventKind::RequestedQosIncompatible => {
                self.dds_incompatible
                    .insert((child, event.topic_hash, Direction::Sub));
            }
            EventKind::PublisherInit | EventKind::SubscriptionInit => {
                let direction = match event.kind {
                    EventKind::PublisherInit => Direction::Pub,
                    _ => Direction::Sub,
                };
                // An address rcl hands out again after the old endpoint was
                // finalized: the old one's record is complete.
                if let Some(old) = self.live.remove(&key) {
                    self.retired.push(old);
                }
                let owner = self.owners.get(&key).copied();
                let mut ep = Endpoint::new(
                    member,
                    owner,
                    direction,
                    event.topic_hash,
                    event.monotonic_ns,
                );
                ep.qos = self.pending_qos.remove(&key);
                self.live.insert(key, ep);
            }
            EventKind::Publish | EventKind::Take => {
                let direction = match event.kind {
                    EventKind::Publish => Direction::Pub,
                    _ => Direction::Sub,
                };
                let owner = self.owners.get(&key).copied();
                let ep = self.live.entry(key).or_insert_with(|| {
                    let mut ep = Endpoint::new(
                        member,
                        owner,
                        direction,
                        event.topic_hash,
                        event.monotonic_ns,
                    );
                    ep.seen_without_init = true;
                    ep
                });
                ep.messages += 1;
                ep.last_message_ns = ep.last_message_ns.max(event.monotonic_ns);
                if event.payload_bytes > 0 {
                    ep.sampled_bytes += event.payload_bytes as u64;
                    ep.size_samples += 1;
                }
                ep.window.add(event.monotonic_ns, event.payload_bytes);
            }
            _ => {}
        }
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.live.is_empty() && self.retired.is_empty()
    }

    /// Rows for every endpoint, busiest first: by estimated bandwidth, then
    /// by message rate, so endpoints that never carried a message come last.
    /// `end_ns` is the run's end on the children's clock (`CLOCK_MONOTONIC`).
    pub(crate) fn rows(
        &self,
        end_ns: u64,
        topic_names: &HashMap<u64, String>,
        topic_types: &HashMap<u64, String>,
    ) -> Vec<EndpointRow> {
        let secs = |ns: u64| ns as f64 / 1e9;
        let mut rows: Vec<EndpointRow> = self
            .retired
            .iter()
            .chain(self.live.values())
            .map(|ep| {
                let end = end_ns.max(ep.last_message_ns).max(ep.created_ns);
                let lifetime_s = secs(end - ep.created_ns);
                let rate_hz = if lifetime_s > 0.0 {
                    ep.messages as f64 / lifetime_s
                } else {
                    0.0
                };
                let mean =
                    (ep.size_samples > 0).then(|| ep.sampled_bytes as f64 / ep.size_samples as f64);
                let est_total = mean.map(|m| (m * ep.messages as f64).round() as u64);
                EndpointRow {
                    node: ep.node_hash.and_then(|h| self.node_names.get(&h).cloned()),
                    member: ep.member.clone(),
                    direction: ep.direction,
                    topic: topic_names.get(&ep.topic_hash).cloned(),
                    msg_type: topic_types.get(&ep.topic_hash).cloned(),
                    messages: ep.messages,
                    lifetime_s,
                    rate_hz,
                    since_last_message_s: (ep.messages > 0).then(|| secs(end - ep.last_message_ns)),
                    mean_message_bytes: mean,
                    size_samples: ep.size_samples,
                    est_total_bytes: est_total,
                    est_bytes_per_s: est_total
                        .filter(|_| lifetime_s > 0.0)
                        .map(|b| b as f64 / lifetime_s),
                    created_late: ep.seen_without_init,
                }
            })
            .collect();
        rows.sort_by(|a, b| {
            let bw = |r: &EndpointRow| r.est_bytes_per_s.unwrap_or(0.0);
            bw(b)
                .total_cmp(&bw(a))
                .then(b.rate_hz.total_cmp(&a.rate_hz))
                .then_with(|| a.topic.cmp(&b.topic))
                .then_with(|| a.node.cmp(&b.node))
        });
        rows
    }
}

/// Write `endpoint_stats.json` and log what a reader would look for first:
/// how many endpoints never carried a message, and the busiest ones.
pub(crate) fn write(interception_dir: &Path, rows: &[EndpointRow]) -> eyre::Result<()> {
    let path = interception_dir.join("endpoint_stats.json");
    std::fs::write(&path, serde_json::to_string_pretty(rows)?)?;

    let count = |d: Direction| rows.iter().filter(|r| r.direction == d).count();
    let silent = |d: Direction| {
        rows.iter()
            .filter(|r| r.direction == d && r.messages == 0)
            .count()
    };
    tracing::info!(
        "Interception: {} publishers ({} never published), {} subscriptions ({} never received) -> {}",
        count(Direction::Pub),
        silent(Direction::Pub),
        count(Direction::Sub),
        silent(Direction::Sub),
        path.display()
    );
    for row in rows
        .iter()
        .filter(|r| r.direction == Direction::Pub && r.messages > 0)
        .take(3)
    {
        let size = match row.est_bytes_per_s {
            Some(bps) => format!(", {}/s", human_bytes(bps)),
            None => String::new(),
        };
        tracing::info!(
            "Interception: busiest publisher {} on {}: {:.1} msg/s{}",
            row.node.as_deref().unwrap_or(&row.member),
            row.topic.as_deref().unwrap_or("?"),
            row.rate_hz,
            size
        );
    }
    Ok(())
}

fn human_bytes(b: f64) -> String {
    const UNITS: [&str; 4] = ["B", "KiB", "MiB", "GiB"];
    let mut v = b;
    let mut unit = 0;
    while v >= 1024.0 && unit < UNITS.len() - 1 {
        v /= 1024.0;
        unit += 1;
    }
    format!("{v:.1} {}", UNITS[unit])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ev(
        kind: EventKind,
        handle: u64,
        topic_hash: u64,
        mono: u64,
        bytes: u32,
    ) -> InterceptionEvent {
        InterceptionEvent {
            kind,
            _pad: [0; 3],
            topic_hash,
            stamp_sec: 0,
            stamp_nanosec: 0,
            handle,
            monotonic_ns: mono,
            cpu_ns: 0,
            tid: 0,
            payload_bytes: bytes,
        }
    }

    const S: u64 = 1_000_000_000;

    fn names() -> (HashMap<u64, String>, HashMap<u64, String>) {
        let mut n = HashMap::new();
        n.insert(0xA, "/chatter".to_string());
        n.insert(0xB, "/silent".to_string());
        (n, HashMap::new())
    }

    #[test]
    fn an_endpoint_that_never_carried_a_message_still_has_a_row() {
        let mut t = EndpointTracker::default();
        t.observe(0, "talker", &ev(EventKind::PublisherInit, 0x10, 0xA, 0, 0));
        t.observe(0, "talker", &ev(EventKind::PublisherInit, 0x20, 0xB, 0, 0));
        for i in 0..10 {
            t.observe(
                0,
                "talker",
                &ev(EventKind::Publish, 0x10, 0xA, i * S / 10, 0),
            );
        }
        let (n, ty) = names();
        let rows = t.rows(10 * S, &n, &ty);
        assert_eq!(rows.len(), 2);
        let silent = rows
            .iter()
            .find(|r| r.topic.as_deref() == Some("/silent"))
            .unwrap();
        assert_eq!(silent.messages, 0);
        assert_eq!(silent.rate_hz, 0.0);
        assert_eq!(silent.since_last_message_s, None);
        assert_eq!(
            rows.last().unwrap().topic.as_deref(),
            Some("/silent"),
            "silent sorts last"
        );
        let busy = &rows[0];
        assert_eq!(busy.messages, 10);
        assert!(
            (busy.rate_hz - 1.0).abs() < 1e-9,
            "10 messages over a 10 s life"
        );
    }

    #[test]
    fn bytes_are_the_count_times_the_mean_sampled_size() {
        let mut t = EndpointTracker::default();
        t.observe(0, "m", &ev(EventKind::PublisherInit, 0x10, 0xA, 0, 0));
        // 100 messages, two of them sampled at 1000 and 3000 bytes.
        for i in 0..100 {
            let bytes = match i {
                0 => 1000,
                50 => 3000,
                _ => 0,
            };
            t.observe(
                0,
                "m",
                &ev(EventKind::Publish, 0x10, 0xA, i * S / 100, bytes),
            );
        }
        let (n, ty) = names();
        let row = &t.rows(4 * S, &n, &ty)[0];
        assert_eq!(row.size_samples, 2);
        assert_eq!(row.mean_message_bytes, Some(2000.0));
        assert_eq!(row.est_total_bytes, Some(200_000));
        assert_eq!(row.est_bytes_per_s, Some(50_000.0));
    }

    #[test]
    fn composables_in_one_process_are_told_apart_by_their_owner() {
        let mut t = EndpointTracker::default();
        let mut name_chunk = |hash: u64, name: &str| {
            let mut e = ev(EventKind::NodeNameDeclared, 0, hash, 0, 0);
            let mut buf = [0u8; 24];
            buf[..name.len()].copy_from_slice(name.as_bytes());
            e.stamp_sec = i32::from_ne_bytes(buf[0..4].try_into().unwrap());
            e.stamp_nanosec = u32::from_ne_bytes(buf[4..8].try_into().unwrap());
            e.handle = u64::from_ne_bytes(buf[8..16].try_into().unwrap());
            e.monotonic_ns = u64::from_ne_bytes(buf[16..24].try_into().unwrap());
            e._pad = [0, 1, name.len() as u8];
            t.observe(0, "container", &e);
        };
        name_chunk(0x1, "/a/first");
        name_chunk(0x2, "/a/second");
        for (handle, owner) in [(0x10, 0x1), (0x20, 0x2)] {
            t.observe(
                0,
                "container",
                &ev(EventKind::EndpointOwner, handle, owner, 0, 0),
            );
            t.observe(
                0,
                "container",
                &ev(EventKind::SubscriptionInit, handle, 0xA, 0, 0),
            );
        }
        t.observe(0, "container", &ev(EventKind::Take, 0x10, 0xA, S, 0));
        let (n, ty) = names();
        let rows = t.rows(2 * S, &n, &ty);
        assert_eq!(rows[0].node.as_deref(), Some("/a/first"));
        assert_eq!(rows[0].messages, 1);
        assert_eq!(rows[1].node.as_deref(), Some("/a/second"));
        assert_eq!(rows[1].messages, 0);
        assert!(rows.iter().all(|r| r.member == "container"));
    }

    #[test]
    fn the_same_handle_in_two_processes_is_two_endpoints() {
        let mut t = EndpointTracker::default();
        t.observe(0, "one", &ev(EventKind::PublisherInit, 0x10, 0xA, 0, 0));
        t.observe(1, "two", &ev(EventKind::SubscriptionInit, 0x10, 0xA, 0, 0));
        t.observe(1, "two", &ev(EventKind::Take, 0x10, 0xA, S, 0));
        let (n, ty) = names();
        let rows = t.rows(2 * S, &n, &ty);
        assert_eq!(rows.len(), 2);
        let sub = rows.iter().find(|r| r.direction == Direction::Sub).unwrap();
        assert_eq!((sub.member.as_str(), sub.messages), ("two", 1));
        let publ = rows.iter().find(|r| r.direction == Direction::Pub).unwrap();
        assert_eq!((publ.member.as_str(), publ.messages), ("one", 0));
    }

    #[test]
    fn a_reused_handle_starts_a_new_endpoint() {
        let mut t = EndpointTracker::default();
        t.observe(0, "m", &ev(EventKind::PublisherInit, 0x10, 0xA, 0, 0));
        t.observe(0, "m", &ev(EventKind::Publish, 0x10, 0xA, S, 0));
        t.observe(0, "m", &ev(EventKind::PublisherInit, 0x10, 0xB, 2 * S, 0));
        let (n, ty) = names();
        let rows = t.rows(3 * S, &n, &ty);
        assert_eq!(rows.len(), 2);
        let old = rows
            .iter()
            .find(|r| r.topic.as_deref() == Some("/chatter"))
            .unwrap();
        assert_eq!(old.messages, 1);
        let new = rows
            .iter()
            .find(|r| r.topic.as_deref() == Some("/silent"))
            .unwrap();
        assert_eq!(new.messages, 0);
        assert!((new.lifetime_s - 1.0).abs() < 1e-9);
    }

    #[test]
    fn traffic_without_an_init_event_is_kept_and_flagged() {
        // The init event can be lost to ring overflow; the messages are not
        // thrown away for it.
        let mut t = EndpointTracker::default();
        t.observe(0, "m", &ev(EventKind::Take, 0x10, 0xA, S, 0));
        let (n, ty) = names();
        let rows = t.rows(2 * S, &n, &ty);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].direction, Direction::Sub);
        assert!(rows[0].created_late);
    }
}
