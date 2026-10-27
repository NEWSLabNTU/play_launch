//! Service calls as the interceptor saw them (phase 95).
//!
//! The `.so` reports every service client and server a child creates and
//! every request sent or taken and response sent or taken (`EventKind`
//! 23-28). This pairs them:
//!
//! - a CLIENT's request is open from its send until the client takes the
//!   response with the same sequence number. One still open past the
//!   client's contract `timeout:` (10 s without one) is the Health page's
//!   `call_unanswered`: the server is gone, deaf, slow, or the request or the
//!   response was lost, which no middleware event reports;
//! - the REQUEST LEG is the time from the client's send to the server's
//!   take, both on `CLOCK_MONOTONIC`, which every process on the host shares.
//!   A server take is paired with its send by (service, sequence number);
//!   two clients of one service number their requests alike, so the take's
//!   `source_timestamp` (the client's `CLOCK_REALTIME` at the write) picks
//!   the send nearest it, which the send event records. The writer GUID
//!   cannot pair them: under Fast DDS a response's request id names another
//!   writer than the request's (measured). `measure` prints the maximum as
//!   `max_transport`;
//! - a server take carries the request's `source_timestamp` when the
//!   middleware fills it, which is what `srv.<e>.stale_after` relies on;
//!   counted per server, so a run says whether the RMW in use fills it.
//!
//! Rings drain in turn, so a server's take is often read before the client's
//! send, and a client's response before the server's take. Both orders are
//! held: an unpaired take waits (bounded), and a client keeps its last
//! answered sends.

use super::{EventKind, InterceptionEvent};
use serde::Serialize;
use std::collections::{BTreeMap, HashMap, VecDeque};

/// A request open longer than this is dropped from the open set and counted:
/// a client that gave up long ago is history, not a live issue.
const OPEN_EXPIRY_NS: u64 = 600 * 1_000_000_000;
/// Open requests kept per client; older ones are counted as expired.
const OPEN_CAP: usize = 1024;
/// Answered sends a client keeps for a server take read after the response.
const RECENT_CAP: usize = 256;
/// Server takes waiting for their client's send or GUID.
const PENDING_CAP: usize = 4096;
/// Request legs kept per client, for the percentiles.
const LEGS_CAP: usize = 10_000;

/// A send: `CLOCK_MONOTONIC` and `CLOCK_REALTIME`, ns.
#[derive(Debug, Clone, Copy)]
struct Send {
    mono: u64,
    real: i64,
    /// Its server take was already paired.
    timed: bool,
}

#[derive(Debug, Default)]
struct Client {
    member: String,
    node_hash: u64,
    service_hash: u64,
    /// seq -> send.
    open: BTreeMap<i64, Send>,
    /// (seq, send) of the last answered requests.
    recent: VecDeque<(i64, Send)>,
    sent: u64,
    answered: u64,
    expired: u64,
    /// Request legs, ns.
    legs: Vec<u64>,
    /// Send to response take, ns.
    round_trips: Vec<u64>,
}

#[derive(Debug, Default)]
struct Server {
    member: String,
    node_hash: u64,
    service_hash: u64,
    taken: u64,
    responded: u64,
    /// Takes whose `source_timestamp` was filled, and ones whose was not.
    source_filled: u64,
    source_zero: u64,
}

/// Every call of the run, by `(child index, handle)`.
#[derive(Debug, Default)]
pub(crate) struct CallBook {
    clients: HashMap<(usize, u64), Client>,
    servers: HashMap<(usize, u64), Server>,
    /// Server takes not yet paired with a send: (service hash, seq) ->
    /// (take time, source timestamp), oldest first.
    pending: VecDeque<(u64, i64, u64, i64)>,
}

/// A source timestamp further than this from every candidate send pairs
/// with none of them: it belongs to a client this run does not see.
const PAIR_WINDOW_NS: i64 = 1_000_000_000;

/// The event's stamp as nanoseconds, 0 when absent.
fn stamp_ns(e: &InterceptionEvent) -> i64 {
    i64::from(e.stamp_sec) * 1_000_000_000 + i64::from(e.stamp_nanosec)
}

fn push_capped(v: &mut Vec<u64>, x: u64) {
    if v.len() < LEGS_CAP {
        v.push(x);
    }
}

impl CallBook {
    /// No request was sent or taken.
    pub(crate) fn is_empty(&self) -> bool {
        self.clients.values().all(|c| c.sent == 0) && self.servers.values().all(|s| s.taken == 0)
    }

    pub(crate) fn observe(&mut self, child: usize, member: &str, e: &InterceptionEvent) {
        let key = (child, e.handle);
        match e.kind {
            EventKind::ServiceClientInit => {
                self.clients.insert(
                    key,
                    Client {
                        member: member.to_string(),
                        node_hash: e.cpu_ns,
                        service_hash: e.topic_hash,
                        ..Default::default()
                    },
                );
            }
            EventKind::ServiceServerInit => {
                self.servers.insert(
                    key,
                    Server {
                        member: member.to_string(),
                        node_hash: e.cpu_ns,
                        service_hash: e.topic_hash,
                        ..Default::default()
                    },
                );
            }
            EventKind::ServiceRequestSent => {
                let c = self.clients.entry(key).or_insert_with(|| Client {
                    member: member.to_string(),
                    service_hash: e.topic_hash,
                    ..Default::default()
                });
                c.sent += 1;
                let seq = e.cpu_ns as i64;
                let mut send = Send {
                    mono: e.monotonic_ns,
                    real: stamp_ns(e),
                    timed: false,
                };
                // A server take read before this send.
                let service = c.service_hash;
                if let Some(i) = self.pending.iter().position(|&(sv, q, _, src)| {
                    sv == service
                        && q == seq
                        && (src == 0 || send.real == 0 || (src - send.real).abs() <= PAIR_WINDOW_NS)
                }) {
                    let (_, _, take, _) = self.pending.remove(i).unwrap();
                    push_capped(&mut c.legs, take.saturating_sub(send.mono));
                    send.timed = true;
                }
                c.open.insert(seq, send);
                while c.open.len() > OPEN_CAP {
                    c.open.pop_first();
                    c.expired += 1;
                }
            }
            EventKind::ServiceResponseTaken => {
                let seq = e.cpu_ns as i64;
                let Some(c) = self.clients.get_mut(&key) else {
                    return;
                };
                let Some(sent) = c.open.remove(&seq) else {
                    return;
                };
                c.answered += 1;
                push_capped(&mut c.round_trips, e.monotonic_ns.saturating_sub(sent.mono));
                if !sent.timed {
                    c.recent.push_back((seq, sent));
                    if c.recent.len() > RECENT_CAP {
                        c.recent.pop_front();
                    }
                }
            }
            EventKind::ServiceRequestTaken => {
                let s = self.servers.entry(key).or_insert_with(|| Server {
                    member: member.to_string(),
                    service_hash: e.topic_hash,
                    ..Default::default()
                });
                s.taken += 1;
                if e.stamp_sec != 0 || e.stamp_nanosec != 0 {
                    s.source_filled += 1;
                } else {
                    s.source_zero += 1;
                }
                let seq = e.cpu_ns as i64;
                let src = stamp_ns(e);
                let service = e.topic_hash;
                // The sends this take could be: an untimed send of this
                // sequence number by a client of this service.
                let mut best: Option<((usize, u64), i64)> = None;
                let mut candidates = 0usize;
                for (ck, c) in &self.clients {
                    if c.service_hash != service {
                        continue;
                    }
                    let send = c
                        .open
                        .get(&seq)
                        .copied()
                        .or_else(|| c.recent.iter().find(|(q, _)| *q == seq).map(|(_, s)| *s));
                    let Some(send) = send.filter(|s| !s.timed) else {
                        continue;
                    };
                    candidates += 1;
                    let d = if src != 0 && send.real != 0 {
                        (src - send.real).abs()
                    } else {
                        0
                    };
                    if d <= PAIR_WINDOW_NS && best.is_none_or(|(_, b)| d < b) {
                        best = Some((*ck, d));
                    }
                }
                // Without a source timestamp only a sole candidate is sure.
                let sure = src != 0 || candidates == 1;
                if let Some((ck, _)) = best.filter(|_| sure)
                    && let Some(c) = self.clients.get_mut(&ck)
                {
                    let mono = if let Some(s) = c.open.get_mut(&seq) {
                        s.timed = true;
                        s.mono
                    } else {
                        let i = c.recent.iter().position(|(q, _)| *q == seq).unwrap();
                        c.recent.remove(i).unwrap().1.mono
                    };
                    push_capped(&mut c.legs, e.monotonic_ns.saturating_sub(mono));
                    return;
                }
                self.pending.push_back((service, seq, e.monotonic_ns, src));
                while self.pending.len() > PENDING_CAP {
                    self.pending.pop_front();
                }
            }
            EventKind::ServiceResponseSent => {
                if let Some(s) = self.servers.get_mut(&key) {
                    s.responded += 1;
                }
            }
            _ => {}
        }
    }

    /// Drop requests open past [`OPEN_EXPIRY_NS`], counting them.
    pub(crate) fn expire(&mut self, now_ns: u64) {
        for c in self.clients.values_mut() {
            while let Some((_, s)) = c.open.first_key_value() {
                if now_ns.saturating_sub(s.mono) <= OPEN_EXPIRY_NS {
                    break;
                }
                c.open.pop_first();
                c.expired += 1;
            }
        }
    }

    /// The calls as the Health page and `calls.json` read them.
    pub(crate) fn snapshot(
        &self,
        now_ns: u64,
        names: &HashMap<u64, String>,
        node_names: &HashMap<u64, String>,
    ) -> CallSnapshot {
        let node = |h: u64, member: &str| {
            node_names
                .get(&h)
                .cloned()
                .unwrap_or_else(|| member.to_string())
        };
        let service = |h: u64| names.get(&h).cloned().unwrap_or_else(|| format!("#{h:x}"));
        // A client that never sent and a server that never took (every
        // node's six parameter services) say nothing about a call.
        let mut clients: Vec<ClientRow> = self
            .clients
            .values()
            .filter(|c| c.sent > 0)
            .map(|c| ClientRow {
                node: node(c.node_hash, &c.member),
                node_known: node_names.contains_key(&c.node_hash),
                member: c.member.clone(),
                service: service(c.service_hash),
                sent: c.sent,
                answered: c.answered,
                open: c.open.len() as u64,
                oldest_open_ms: c
                    .open
                    .values()
                    .map(|s| s.mono)
                    .min()
                    .map(|t| now_ns.saturating_sub(t) as f64 / 1e6),
                expired: c.expired,
                request_leg: Dist::of(&c.legs),
                round_trip: Dist::of(&c.round_trips),
            })
            .collect();
        clients.sort_by(|a, b| (&a.node, &a.service).cmp(&(&b.node, &b.service)));
        let mut servers: Vec<ServerRow> = self
            .servers
            .values()
            .filter(|s| s.taken > 0)
            .map(|s| ServerRow {
                node: node(s.node_hash, &s.member),
                member: s.member.clone(),
                service: service(s.service_hash),
                taken: s.taken,
                responded: s.responded,
                source_timestamp_filled: s.source_filled,
                source_timestamp_zero: s.source_zero,
            })
            .collect();
        servers.sort_by(|a, b| (&a.node, &a.service).cmp(&(&b.node, &b.service)));
        CallSnapshot { clients, servers }
    }
}

/// Percentiles over a set of durations, milliseconds.
#[derive(Debug, Clone, Serialize, serde::Deserialize, PartialEq)]
pub struct Dist {
    pub n: usize,
    pub min_ms: f64,
    pub p50_ms: f64,
    pub p99_ms: f64,
    pub max_ms: f64,
}

impl Dist {
    fn of(v: &[u64]) -> Option<Dist> {
        if v.is_empty() {
            return None;
        }
        let mut s = v.to_vec();
        s.sort_unstable();
        let at = |q: f64| s[((s.len() - 1) as f64 * q).round() as usize] as f64 / 1e6;
        Some(Dist {
            n: s.len(),
            min_ms: s[0] as f64 / 1e6,
            p50_ms: at(0.5),
            p99_ms: at(0.99),
            max_ms: *s.last().unwrap() as f64 / 1e6,
        })
    }
}

/// One client, as read off the calls it made.
#[derive(Debug, Clone, Serialize, serde::Deserialize, PartialEq)]
pub struct ClientRow {
    /// The node that created the client, its member's name when unknown.
    pub node: String,
    /// Whether `node` is the node's real FQN (else the member's name).
    #[serde(default)]
    pub node_known: bool,
    pub member: String,
    pub service: String,
    pub sent: u64,
    pub answered: u64,
    /// Requests sent and not answered (and not yet expired).
    pub open: u64,
    pub oldest_open_ms: Option<f64>,
    /// Requests dropped from the open set unanswered: older than ten
    /// minutes, or past the per-client cap.
    pub expired: u64,
    /// Client send to server take, for answered requests.
    pub request_leg: Option<Dist>,
    /// Client send to response take.
    pub round_trip: Option<Dist>,
}

/// One server.
#[derive(Debug, Clone, Serialize, serde::Deserialize, PartialEq)]
pub struct ServerRow {
    pub node: String,
    pub member: String,
    pub service: String,
    pub taken: u64,
    pub responded: u64,
    /// Requests taken with a `source_timestamp`, and with none: whether the
    /// middleware in use fills what `srv.<e>.stale_after` needs.
    pub source_timestamp_filled: u64,
    pub source_timestamp_zero: u64,
}

/// Every client and server of the run (`interception/calls.json`).
#[derive(Debug, Clone, Default, Serialize, serde::Deserialize, PartialEq)]
pub struct CallSnapshot {
    pub clients: Vec<ClientRow>,
    pub servers: Vec<ServerRow>,
}

/// Write `calls.json` into `dir`.
pub(crate) fn write(dir: &std::path::Path, snap: &CallSnapshot) -> eyre::Result<()> {
    std::fs::write(dir.join("calls.json"), serde_json::to_vec_pretty(snap)?)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `real_s` is the send's realtime (on a send) or the source timestamp
    /// (on a take), whole seconds; 0 for none.
    fn ev(
        kind: EventKind,
        handle: u64,
        service: u64,
        seq: i64,
        real_s: i32,
        t: u64,
    ) -> InterceptionEvent {
        InterceptionEvent {
            kind,
            _pad: [0; 3],
            topic_hash: service,
            stamp_sec: real_s,
            stamp_nanosec: 0,
            handle,
            monotonic_ns: t,
            cpu_ns: seq as u64,
            tid: 0,
            payload_bytes: 0,
        }
    }

    const MS: u64 = 1_000_000;

    /// A request is open until its response is taken; the request leg is
    /// timed whichever ring is drained first; and two clients of one
    /// service, numbering their requests alike, are told apart by the
    /// take's source timestamp.
    #[test]
    fn requests_pair_with_responses_in_either_drain_order() {
        let mut b = CallBook::default();
        b.observe(
            0,
            "client",
            &ev(EventKind::ServiceClientInit, 1, 9, 0, 0, 0),
        );
        b.observe(2, "other", &ev(EventKind::ServiceClientInit, 3, 9, 0, 0, 0));
        b.observe(
            1,
            "server",
            &ev(EventKind::ServiceServerInit, 2, 9, 0, 0, 0),
        );
        // Request 1: the server ring is read first.
        b.observe(
            1,
            "server",
            &ev(EventKind::ServiceRequestTaken, 2, 9, 1, 100, 3 * MS),
        );
        b.observe(
            0,
            "client",
            &ev(EventKind::ServiceRequestSent, 1, 9, 1, 100, MS),
        );
        b.observe(
            0,
            "client",
            &ev(EventKind::ServiceResponseTaken, 1, 9, 1, 0, 5 * MS),
        );
        // Request 2: the client ring is read first (send and response), the
        // server's take after; the other client sent its own request 2 at
        // another time, and is not chosen.
        b.observe(
            2,
            "other",
            &ev(EventKind::ServiceRequestSent, 3, 9, 2, 300, 9 * MS),
        );
        b.observe(
            0,
            "client",
            &ev(EventKind::ServiceRequestSent, 1, 9, 2, 110, 10 * MS),
        );
        b.observe(
            0,
            "client",
            &ev(EventKind::ServiceResponseTaken, 1, 9, 2, 0, 14 * MS),
        );
        b.observe(
            1,
            "server",
            &ev(EventKind::ServiceRequestTaken, 2, 9, 2, 110, 11 * MS),
        );
        // Request 3: never answered.
        b.observe(
            0,
            "client",
            &ev(EventKind::ServiceRequestSent, 1, 9, 3, 120, 20 * MS),
        );
        let names = HashMap::from([(9u64, "/svc".to_string())]);
        let snap = b.snapshot(1020 * MS, &names, &HashMap::new());
        let c = snap.clients.iter().find(|c| c.member == "client").unwrap();
        let other = snap.clients.iter().find(|c| c.member == "other").unwrap();
        assert_eq!(
            other.request_leg, None,
            "the other client's request 2 is untimed"
        );
        assert_eq!((c.sent, c.answered, c.open), (3, 2, 1));
        assert_eq!(c.oldest_open_ms, Some(1000.0));
        let leg = c.request_leg.as_ref().unwrap();
        assert_eq!((leg.n, leg.min_ms, leg.max_ms), (2, 1.0, 2.0));
        assert_eq!(c.round_trip.as_ref().unwrap().max_ms, 4.0);
        assert_eq!(snap.servers[0].taken, 2);
        assert_eq!(snap.servers[0].source_timestamp_filled, 2);
        // Old requests leave the open set and are counted.
        b.expire(1020 * MS + OPEN_EXPIRY_NS);
        let snap = b.snapshot(0, &names, &HashMap::new());
        let c = snap.clients.iter().find(|c| c.member == "client").unwrap();
        assert_eq!((c.open, c.expired), (0, 1));
    }
}
