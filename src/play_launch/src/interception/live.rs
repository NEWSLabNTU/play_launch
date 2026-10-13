//! The live view of every endpoint: rates over the last seconds rather than
//! the run, the QoS each endpoint actually got, and what is wrong between
//! them. Feeds the web UI's Entities page through a 1 Hz snapshot
//! (`docs/design/live-entity-monitor.md`).
//!
//! Everything here is computed from state `EndpointTracker` already keeps;
//! the per-event cost of the live view is one bucket increment
//! ([`Window::add`]). The snapshot is built once a second.

use std::collections::{HashMap, HashSet};

use serde::Serialize;

use super::{
    EventKind, InterceptionEvent,
    endpoint_stats::{Direction, Endpoint, EndpointTracker, LatencySummary},
};

/// Seconds of history the live rates are computed over.
pub(crate) const WINDOW_S: u64 = 10;

const NS: u64 = 1_000_000_000;

// ---------------------------------------------------------------------------
// Rolling window
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, Default)]
struct Bucket {
    /// Which second (`monotonic_ns / 1e9`) this bucket currently holds.
    second: u64,
    messages: u64,
    sampled_bytes: u64,
    size_samples: u64,
}

/// One-second buckets of an endpoint's traffic: [`WINDOW_S`] complete
/// seconds plus the one in progress, which would otherwise overwrite the
/// oldest complete one (a 10-slot ring only ever holds nine).
const SLOTS: u64 = WINDOW_S + 1;

#[derive(Debug, Clone, Default)]
pub(crate) struct Window {
    buckets: [Bucket; SLOTS as usize],
}

impl Window {
    pub(crate) fn add(&mut self, at_ns: u64, payload_bytes: u32) {
        let second = at_ns / NS;
        let b = &mut self.buckets[(second % SLOTS) as usize];
        if b.second != second {
            *b = Bucket {
                second,
                ..Default::default()
            };
        }
        b.messages += 1;
        if payload_bytes > 0 {
            b.sampled_bytes += payload_bytes as u64;
            b.size_samples += 1;
        }
    }

    /// Totals over the complete seconds in `[now - n, now)`. The second in
    /// progress is left out so a rate never reads low for being half-counted.
    fn sum(&self, now_ns: u64, n: u64) -> Bucket {
        let now_s = now_ns / NS;
        let mut out = Bucket::default();
        for b in &self.buckets {
            if b.second < now_s && b.second + n >= now_s && b.messages > 0 {
                out.messages += b.messages;
                out.sampled_bytes += b.sampled_bytes;
                out.size_samples += b.size_samples;
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------
// QoS
// ---------------------------------------------------------------------------

/// The QoS an endpoint actually got, from an `EndpointQos` event. Enum
/// fields hold rmw's values (`rmw/types.h`); durations are ns with 0 meaning
/// unspecified and `u64::MAX` infinite — both "no bound" for the rules.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Qos {
    reliability: u8,
    durability: u8,
    history: u8,
    depth: u32,
    liveliness: u32,
    deadline_ns: u64,
    lease_ns: u64,
}

const RELIABILITY_RELIABLE: u8 = 1;
const RELIABILITY_BEST_EFFORT: u8 = 2;
const DURABILITY_TRANSIENT_LOCAL: u8 = 1;
const DURABILITY_VOLATILE: u8 = 2;

impl Qos {
    pub(crate) fn from_event(e: &InterceptionEvent) -> Self {
        debug_assert_eq!(e.kind, EventKind::EndpointQos);
        Self {
            reliability: e._pad[0],
            durability: e._pad[1],
            history: e._pad[2],
            depth: e.stamp_sec.max(0) as u32,
            liveliness: e.stamp_nanosec,
            deadline_ns: e.monotonic_ns,
            lease_ns: e.cpu_ns,
        }
    }

    /// A duration bound, or `None` for "no bound" (unspecified or infinite).
    fn bound(ns: u64) -> Option<u64> {
        (ns != 0 && ns != u64::MAX).then_some(ns)
    }

    fn view(&self) -> QosView {
        let reliability = match self.reliability {
            0 => "system_default",
            1 => "reliable",
            2 => "best_effort",
            4 => "best_available",
            _ => "unknown",
        };
        let durability = match self.durability {
            0 => "system_default",
            1 => "transient_local",
            2 => "volatile",
            4 => "best_available",
            _ => "unknown",
        };
        let history = match self.history {
            0 => "system_default",
            1 => "keep_last",
            2 => "keep_all",
            _ => "unknown",
        };
        let liveliness = match self.liveliness {
            0 => "system_default",
            1 => "automatic",
            2 => "manual_by_node",
            3 => "manual_by_topic",
            5 => "best_available",
            _ => "unknown",
        };
        let ms = |ns: u64| Self::bound(ns).map(|v| v as f64 / 1e6);
        QosView {
            reliability,
            durability,
            history,
            depth: self.depth,
            deadline_ms: ms(self.deadline_ns),
            liveliness,
            lease_ms: ms(self.lease_ns),
        }
    }
}

/// Why a publisher and a subscription with these profiles never match:
/// DDS's request-versus-offered rule, policy by policy. Empty when they do.
pub(crate) fn incompatibilities(publ: &Qos, sub: &Qos) -> Vec<(&'static str, String)> {
    let mut out = Vec::new();
    if publ.reliability == RELIABILITY_BEST_EFFORT && sub.reliability == RELIABILITY_RELIABLE {
        out.push((
            "reliability",
            "publisher offers best_effort, subscription requests reliable".to_string(),
        ));
    }
    if publ.durability == DURABILITY_VOLATILE && sub.durability == DURABILITY_TRANSIENT_LOCAL {
        out.push((
            "durability",
            "subscription requests transient_local (late-joiner data) from a volatile publisher"
                .to_string(),
        ));
    }
    let ms = |ns: u64| ns as f64 / 1e6;
    if let Some(requested) = Qos::bound(sub.deadline_ns) {
        match Qos::bound(publ.deadline_ns) {
            None => out.push((
                "deadline",
                format!(
                    "subscription requests a {:.0} ms deadline, publisher offers none",
                    ms(requested)
                ),
            )),
            Some(offered) if offered > requested => out.push((
                "deadline",
                format!(
                    "publisher offers a {:.0} ms deadline, subscription requests {:.0} ms",
                    ms(offered),
                    ms(requested)
                ),
            )),
            _ => {}
        }
    }
    // AUTOMATIC (1) < MANUAL_BY_NODE (2) < MANUAL_BY_TOPIC (3).
    if (1..=3).contains(&publ.liveliness)
        && (1..=3).contains(&sub.liveliness)
        && publ.liveliness < sub.liveliness
    {
        out.push((
            "liveliness",
            format!(
                "publisher offers {} liveliness, subscription requests {}",
                publ.view().liveliness,
                sub.view().liveliness
            ),
        ));
    }
    if let Some(requested) = Qos::bound(sub.lease_ns) {
        let offered = Qos::bound(publ.lease_ns);
        if offered.is_none_or(|o| o > requested) {
            out.push((
                "lease",
                format!(
                    "subscription requests a {:.0} ms liveliness lease, publisher offers {}",
                    ms(requested),
                    offered.map_or("none".to_string(), |o| format!("{:.0} ms", ms(o)))
                ),
            ));
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Snapshot
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, PartialEq)]
pub(crate) struct QosView {
    pub reliability: &'static str,
    pub durability: &'static str,
    pub history: &'static str,
    pub depth: u32,
    pub deadline_ms: Option<f64>,
    pub liveliness: &'static str,
    pub lease_ms: Option<f64>,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub(crate) struct Flag {
    /// `incompatible`, `advisory`, `silent`, `stale` or `confirmed`.
    pub kind: &'static str,
    pub code: &'static str,
    pub message: String,
}

#[derive(Debug, Clone, Serialize)]
pub(crate) struct LiveRow {
    pub id: String,
    pub node: Option<String>,
    pub member: String,
    pub direction: Direction,
    pub topic: Option<String>,
    pub msg_type: Option<String>,
    pub messages: u64,
    pub rate_1s: f64,
    pub rate_10s: f64,
    pub bytes_per_s: Option<f64>,
    pub mean_message_bytes: Option<f64>,
    pub since_last_message_s: Option<f64>,
    pub lifetime_s: f64,
    pub qos: Option<QosView>,
    /// Delivery latency over the last 10 to 20 s; absent for a subscription
    /// that took no stamped message.
    pub latency: Option<LatencySummary>,
    pub flags: Vec<Flag>,
}

#[derive(Debug, Clone, Default, Serialize)]
pub(crate) struct Totals {
    pub pubs: usize,
    pub subs: usize,
    pub silent_pubs: usize,
    pub silent_subs: usize,
    pub incompatible: usize,
    pub advisories: usize,
    pub ring_drops: u64,
    pub rows: usize,
}

/// One second's picture of every live endpoint, busiest first.
#[derive(Debug, Clone, Default, Serialize)]
pub struct EntitySnapshot {
    pub(crate) t_ms: u64,
    pub(crate) window_s: u64,
    pub(crate) totals: Totals,
    pub(crate) truncated: usize,
    pub(crate) rows: Vec<LiveRow>,
    /// Every node FQN the interceptor saw a process of this run create, for
    /// the Health page's graph comparison. Not sent to the page.
    #[serde(skip)]
    pub(crate) nodes: Vec<String>,
    /// Each node's first endpoint and first publish, for the startup
    /// timeline. Not sent to the page.
    #[serde(skip)]
    pub(crate) startup: Vec<super::endpoint_stats::StartupMark>,
    /// Lifecycle transitions per node, for the startup timeline.
    #[serde(skip)]
    pub(crate) lifecycle: Vec<super::endpoint_stats::LifecycleMark>,
    /// `/clock`, when something publishes it.
    pub(crate) clock: Option<ClockStatus>,
    /// TF edges as published; for the Health page, not sent with Entities.
    #[serde(skip)]
    pub(crate) tf: Option<super::tf::TfSnapshot>,
}

/// `/clock` as the Health page judges it.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct ClockStatus {
    /// The last value, seconds.
    pub value_s: f64,
    pub since_last_s: f64,
    /// Messages per second over its whole life.
    pub rate_hz: f64,
    pub messages: u64,
    pub backward_jumps: u64,
    /// Seconds since the last backward jump, and by how much it went back.
    pub last_backward_ago_s: Option<f64>,
    pub last_backward_by_s: Option<f64>,
}

impl EntitySnapshot {
    /// The first `limit` rows plus every flagged one further down (silent
    /// endpoints excepted: they are counted in the totals, and on a large
    /// system most of the list), in the original order.
    pub fn trimmed(&self, limit: usize) -> EntitySnapshot {
        let keep = |i: usize, r: &LiveRow| i < limit || r.flags.iter().any(|f| f.kind != "silent");
        let rows: Vec<LiveRow> = self
            .rows
            .iter()
            .enumerate()
            .filter(|(i, r)| keep(*i, r))
            .map(|(_, r)| r.clone())
            .collect();
        EntitySnapshot {
            t_ms: self.t_ms,
            window_s: self.window_s,
            totals: self.totals.clone(),
            truncated: self.rows.len() - rows.len(),
            rows,
            nodes: Vec::new(),
            startup: Vec::new(),
            lifecycle: Vec::new(),
            clock: self.clock.clone(),
            tf: self.tf.clone(),
        }
    }
}

/// Take rate below this share of the topic's publish rate is a deficit.
const DEFICIT_RATIO: f64 = 0.9;
/// Below this publish rate (Hz) a deficit is not judged: one late message
/// on a slow topic is not a pattern.
const DEFICIT_MIN_HZ: f64 = 1.0;
/// An endpoint that carried traffic and has been quiet this long is stale.
const STALE_AFTER_S: f64 = 10.0;
/// Fewer messages than this is a burst, not a stream that can go stale.
const STALE_MIN_MESSAGES: u64 = 5;
/// A stream must have run this long to be judged stale; a startup burst of
/// parameter declarations is many messages in a second or two.
const STALE_MIN_ACTIVE_S: f64 = 5.0;
/// Topics that carry events rather than streams: quiet is their normal state.
const EVENT_TOPICS: [&str; 2] = ["/parameter_events", "/rosout"];
/// Delivery latency (its p95) is judged against the larger of this and
/// [`LATENCY_PERIODS`] periods of the topic.
const LATENCY_FLOOR_MS: f64 = 100.0;
const LATENCY_PERIODS: f64 = 2.0;
/// Fewer matched takes than this in the window is not judged.
const LATENCY_MIN_SAMPLES: u64 = 20;

impl EndpointTracker {
    /// Build the live snapshot at `now_ns` (`CLOCK_MONOTONIC`).
    pub(crate) fn live_snapshot(
        &self,
        now_ns: u64,
        topic_names: &HashMap<u64, String>,
        topic_types: &HashMap<u64, String>,
        ring_drops: u64,
    ) -> EntitySnapshot {
        let eps: Vec<(&(usize, u64), &Endpoint)> = self.live.iter().collect();

        // Per-endpoint rates first: the topic-level checks need them.
        struct Rates {
            rate_1s: f64,
            rate_10s: f64,
            mean: Option<f64>,
        }
        let rates: Vec<Rates> = eps
            .iter()
            .map(|(_, ep)| {
                let age_s = now_ns.saturating_sub(ep.created_ns) / NS;
                let span = age_s.clamp(1, WINDOW_S);
                let last = ep.window.sum(now_ns, 1);
                let win = ep.window.sum(now_ns, span);
                let mean = if win.size_samples > 0 {
                    Some(win.sampled_bytes as f64 / win.size_samples as f64)
                } else if ep.size_samples > 0 {
                    Some(ep.sampled_bytes as f64 / ep.size_samples as f64)
                } else {
                    None
                };
                Rates {
                    rate_1s: last.messages as f64,
                    rate_10s: win.messages as f64 / span as f64,
                    mean,
                }
            })
            .collect();

        // Group endpoint indices by topic.
        let mut by_topic: HashMap<u64, (Vec<usize>, Vec<usize>)> = HashMap::new();
        for (i, (_, ep)) in eps.iter().enumerate() {
            let g = by_topic.entry(ep.topic_hash).or_default();
            match ep.direction {
                Direction::Pub => g.0.push(i),
                Direction::Sub => g.1.push(i),
            }
        }

        let mut flags: Vec<Vec<Flag>> = vec![Vec::new(); eps.len()];
        let label = |i: usize| -> String {
            let ep = eps[i].1;
            ep.node_hash
                .and_then(|h| self.node_names.get(&h).cloned())
                .unwrap_or_else(|| ep.member.clone())
        };
        for (topic, (pubs, subs)) in &by_topic {
            // Pairwise QoS. One flag per policy per endpoint, naming the first
            // counterpart and how many more there are.
            let mut found: HashMap<(usize, &'static str), (String, usize)> = HashMap::new();
            for &p in pubs {
                for &s in subs {
                    let (Some(pq), Some(sq)) = (eps[p].1.qos, eps[s].1.qos) else {
                        continue;
                    };
                    for (code, why) in incompatibilities(&pq, &sq) {
                        for (me, other) in [(p, s), (s, p)] {
                            found
                                .entry((me, code))
                                .and_modify(|e| e.1 += 1)
                                .or_insert_with(|| (format!("{why} ({})", label(other)), 0));
                        }
                    }
                }
            }
            for ((i, code), (msg, more)) in found {
                let message = if more > 0 {
                    format!("{msg} and {more} more")
                } else {
                    msg
                };
                flags[i].push(Flag {
                    kind: "incompatible",
                    code,
                    message,
                });
            }

            // Take deficit: a reliable subscription taking clearly less than
            // the topic's publishers send.
            let published: f64 = pubs.iter().map(|&p| rates[p].rate_10s).sum();
            // Event channels arrive in bursts (every node declaring its
            // parameters at once), which a subscriber still constructing
            // queues: not a stream to keep up with.
            let event_channel = topic_names
                .get(topic)
                .is_some_and(|t| EVENT_TOPICS.contains(&t.as_str()));
            if published >= DEFICIT_MIN_HZ && !event_channel {
                for &s in subs {
                    let sub = eps[s].1;
                    let reliable = sub
                        .qos
                        .is_none_or(|q| q.reliability == RELIABILITY_RELIABLE);
                    let settled = now_ns.saturating_sub(sub.created_ns) >= WINDOW_S * NS;
                    let blocked = flags[s].iter().any(|f| f.kind == "incompatible");
                    let took = rates[s].rate_10s;
                    if reliable && settled && !blocked && took < DEFICIT_RATIO * published {
                        let depth = sub.qos.filter(|q| q.history == 1).map(|q| q.depth);
                        let why = match depth {
                            Some(1) => "queue depth 1 keeps only the latest, so the rest are \
                                        overwritten: expected for a node that polls the latest \
                                        value, a loss for one that needs every message"
                                .to_string(),
                            Some(d) => {
                                format!("queue depth {d}: messages are being dropped or queued")
                            }
                            None => "messages are being dropped or queued".to_string(),
                        };
                        flags[s].push(Flag {
                            kind: "advisory",
                            code: "take_deficit",
                            message: format!(
                                "takes {took:.1}/s of the {published:.1}/s published; {why}"
                            ),
                        });
                    }
                }
            }
        }

        // Delivery latency: a subscription whose recent p99 is well past
        // both a floor and the topic's own period.
        let latencies: Vec<Option<LatencySummary>> = eps
            .iter()
            .map(|(_, ep)| {
                let l = ep.latency.as_deref()?;
                LatencySummary::of(&l.recent(now_ns), l)
            })
            .collect();
        for (topic, (pubs, subs)) in &by_topic {
            // Event channels: `ParameterEvent` carries a stamp, and the
            // startup burst of parameter declarations queues behind every
            // node still constructing. Measured on Autoware: one issue per
            // subscriber of `/parameter_events`, each open 15-20 s.
            if topic_names
                .get(topic)
                .is_some_and(|t| EVENT_TOPICS.contains(&t.as_str()))
            {
                continue;
            }
            let published: f64 = pubs.iter().map(|&p| rates[p].rate_10s).sum();
            let period_ms = (published > 0.0).then(|| 1000.0 / published);
            let limit = LATENCY_FLOOR_MS.max(period_ms.map_or(0.0, |p| LATENCY_PERIODS * p));
            for &s in subs {
                let Some(l) = &latencies[s] else { continue };
                // Judged on p95, shown with p99: with a few dozen samples the
                // p99 IS the largest one, and a single late message (the
                // first after discovery) flagged a subscriber that keeps up.
                let Some(p95) = l.p95_ms else { continue };
                if l.samples >= LATENCY_MIN_SAMPLES && p95 > limit {
                    let p50 = l.p50_ms.unwrap_or(0.0);
                    let p99 = l.p99_ms.unwrap_or(p95);
                    flags[s].push(Flag {
                        kind: "advisory",
                        code: "delivery_latency",
                        message: format!(
                            "messages wait {p50:.0} ms (p50), {p95:.0} ms (p95), {p99:.0} ms \
                             (p99) between publish and take, over {limit:.0} ms; the \
                             subscriber's executor is behind or its queue is holding old \
                             messages"
                        ),
                    });
                }
            }
        }

        let mut totals = Totals {
            ring_drops,
            ..Default::default()
        };
        let mut rows: Vec<LiveRow> = Vec::with_capacity(eps.len());
        for (i, ((child, handle), ep)) in eps.iter().enumerate() {
            let r = &rates[i];
            let mut fl = std::mem::take(&mut flags[i]);
            let since_last =
                (ep.messages > 0).then(|| now_ns.saturating_sub(ep.last_message_ns) as f64 / 1e9);
            if ep.messages == 0 {
                fl.push(Flag {
                    kind: "silent",
                    code: "silent",
                    message: match ep.direction {
                        Direction::Pub => "never published".to_string(),
                        Direction::Sub => "never received a message".to_string(),
                    },
                });
            } else if let Some(age) = since_last.filter(|a| {
                // A stream that stopped, not an endpoint that spoke a few
                // times at startup (`/parameter_events`, `/rosout`): at least
                // STALE_MIN_MESSAGES, and quiet for five of its own average
                // intervals as well as STALE_AFTER_S.
                let active_s = ep.last_message_ns.saturating_sub(ep.created_ns) as f64 / 1e9;
                let interval = active_s / ep.messages as f64;
                let event_channel = topic_names
                    .get(&ep.topic_hash)
                    .is_some_and(|t| EVENT_TOPICS.contains(&t.as_str()));
                !event_channel
                    && ep.messages >= STALE_MIN_MESSAGES
                    && active_s >= STALE_MIN_ACTIVE_S
                    && *a >= STALE_AFTER_S.max(5.0 * interval)
            }) {
                fl.push(Flag {
                    kind: "stale",
                    code: "stale",
                    message: format!("no message for {age:.0} s"),
                });
            }
            if self
                .dds_incompatible
                .contains(&(*child, ep.topic_hash, ep.direction))
            {
                fl.push(Flag {
                    kind: "confirmed",
                    code: "dds_confirmed",
                    message: "the middleware reported an incompatible-QoS event on this topic"
                        .to_string(),
                });
            }
            match ep.direction {
                Direction::Pub => totals.pubs += 1,
                Direction::Sub => totals.subs += 1,
            }
            if ep.messages == 0 {
                match ep.direction {
                    Direction::Pub => totals.silent_pubs += 1,
                    Direction::Sub => totals.silent_subs += 1,
                }
            }
            totals.incompatible += fl.iter().filter(|f| f.kind == "incompatible").count();
            totals.advisories += fl.iter().filter(|f| f.kind == "advisory").count();
            rows.push(LiveRow {
                id: format!("{child}:{handle:x}"),
                node: ep.node_hash.and_then(|h| self.node_names.get(&h).cloned()),
                member: ep.member.clone(),
                direction: ep.direction,
                topic: topic_names.get(&ep.topic_hash).cloned(),
                msg_type: topic_types.get(&ep.topic_hash).cloned(),
                messages: ep.messages,
                rate_1s: r.rate_1s,
                rate_10s: r.rate_10s,
                bytes_per_s: r.mean.map(|m| m * r.rate_10s),
                mean_message_bytes: r.mean,
                since_last_message_s: since_last,
                lifetime_s: now_ns.saturating_sub(ep.created_ns) as f64 / 1e9,
                qos: ep.qos.map(|q| q.view()),
                latency: latencies[i].clone(),
                flags: fl,
            });
        }
        // Incompatibilities were counted once per side; report pairs' sides
        // as rows flagged, which is what the page shows.
        rows.sort_by(|a, b| {
            let bw = |r: &LiveRow| r.bytes_per_s.unwrap_or(0.0);
            bw(b)
                .total_cmp(&bw(a))
                .then(b.rate_10s.total_cmp(&a.rate_10s))
                .then_with(|| a.topic.cmp(&b.topic))
                .then_with(|| a.id.cmp(&b.id))
        });
        totals.rows = rows.len();
        EntitySnapshot {
            t_ms: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_millis() as u64),
            window_s: WINDOW_S,
            totals,
            truncated: 0,
            rows,
            nodes: self.node_names.values().cloned().collect(),
            startup: self.startup_marks(topic_names),
            lifecycle: self.lifecycle_marks(),
            tf: (!self.tf.is_empty()).then(|| {
                self.tf.snapshot(now_ns, |child, handle| {
                    self.owners
                        .get(&(child, handle))
                        .and_then(|h| self.node_names.get(h).cloned())
                })
            }),
            clock: (self.clock.messages > 0).then(|| {
                let c = &self.clock;
                let secs = |ns: u64| ns as f64 / 1e9;
                let life = secs(c.last_mono.saturating_sub(c.first_mono));
                ClockStatus {
                    value_s: c.last_value_ns as f64 / 1e9,
                    since_last_s: secs(now_ns.saturating_sub(c.last_mono)),
                    rate_hz: if life > 0.0 {
                        (c.messages - 1) as f64 / life
                    } else {
                        0.0
                    },
                    messages: c.messages,
                    backward_jumps: c.backward_jumps,
                    last_backward_ago_s: (c.backward_jumps > 0)
                        .then(|| secs(now_ns.saturating_sub(c.last_backward_mono))),
                    last_backward_by_s: (c.backward_jumps > 0)
                        .then(|| c.last_backward_ns as f64 / 1e9),
                }
            }),
        }
    }
}

/// Topics touched, for tests and callers that want the set.
#[allow(dead_code)]
pub(crate) fn topics(snapshot: &EntitySnapshot) -> HashSet<String> {
    snapshot
        .rows
        .iter()
        .filter_map(|r| r.topic.clone())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ev(kind: EventKind, handle: u64, topic_hash: u64, mono: u64) -> InterceptionEvent {
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
            payload_bytes: 0,
        }
    }

    fn qos_ev(handle: u64, topic: u64, rel: u8, dur: u8, depth: i32) -> InterceptionEvent {
        let mut e = ev(EventKind::EndpointQos, handle, topic, 0);
        e._pad = [rel, dur, 1];
        e.stamp_sec = depth;
        e.stamp_nanosec = 1;
        e
    }

    fn names() -> (HashMap<u64, String>, HashMap<u64, String>) {
        let mut n = HashMap::new();
        n.insert(0xA, "/a".to_string());
        (n, HashMap::new())
    }

    #[test]
    fn the_window_reports_the_last_seconds_not_the_run() {
        let mut w = Window::default();
        // 100 messages in second 5, then 10 a second through second 15.
        for _ in 0..100 {
            w.add(5 * NS, 0);
        }
        for s in 6..16 {
            for _ in 0..10 {
                w.add(s * NS + 1, 0);
            }
        }
        // At t = 16.5 s: last complete second is 15, the window is 6..=15.
        assert_eq!(w.sum(16 * NS + NS / 2, 1).messages, 10);
        assert_eq!(w.sum(16 * NS + NS / 2, 10).messages, 100);
        // The second in progress is not counted, and does not evict the
        // oldest complete second (a 10-slot ring would: rates read 9/10).
        w.add(16 * NS + 2, 0);
        assert_eq!(w.sum(16 * NS + NS / 2, 1).messages, 10);
        assert_eq!(w.sum(16 * NS + NS / 2, 10).messages, 100);
    }

    #[test]
    fn reliability_and_durability_mismatches_are_incompatible() {
        let reliable_volatile = Qos::from_event(&qos_ev(0, 0, 1, 2, 10));
        let best_effort = Qos::from_event(&qos_ev(0, 0, 2, 2, 10));
        let transient = Qos::from_event(&qos_ev(0, 0, 1, 1, 1));
        let codes = |p: &Qos, s: &Qos| -> Vec<&str> {
            incompatibilities(p, s)
                .into_iter()
                .map(|(c, _)| c)
                .collect()
        };
        assert_eq!(codes(&best_effort, &reliable_volatile), ["reliability"]);
        assert!(
            codes(&reliable_volatile, &best_effort).is_empty(),
            "offering more is fine"
        );
        assert_eq!(codes(&reliable_volatile, &transient), ["durability"]);
        assert!(codes(&transient, &reliable_volatile).is_empty());
    }

    #[test]
    fn deadline_and_lease_bounds_follow_request_versus_offered() {
        let mut p = Qos::from_event(&qos_ev(0, 0, 1, 2, 10));
        let mut s = p;
        s.deadline_ns = 100_000_000;
        assert_eq!(incompatibilities(&p, &s)[0].0, "deadline", "none offered");
        p.deadline_ns = 50_000_000;
        assert!(
            incompatibilities(&p, &s).is_empty(),
            "50 ms offered meets 100 ms"
        );
        p.deadline_ns = 200_000_000;
        assert_eq!(incompatibilities(&p, &s)[0].0, "deadline");
        p.deadline_ns = u64::MAX;
        s.deadline_ns = 0;
        assert!(
            incompatibilities(&p, &s).is_empty(),
            "infinite meets unrequested"
        );
        s.lease_ns = 1_000_000_000;
        p.lease_ns = 2_000_000_000;
        assert_eq!(incompatibilities(&p, &s)[0].0, "lease");
    }

    #[test]
    fn a_mismatched_pair_flags_both_sides_and_a_slow_reliable_sub_is_a_deficit() {
        let mut t = EndpointTracker::default();
        // Topic 0xA: one best-effort publisher at 20 Hz; a reliable sub
        // (incompatible) and a best-effort sub that takes half.
        t.observe(0, "pub", &qos_ev(0x1, 0xA, 2, 2, 10));
        t.observe(0, "pub", &ev(EventKind::PublisherInit, 0x1, 0xA, 0));
        t.observe(1, "rel", &qos_ev(0x2, 0xA, 1, 2, 10));
        t.observe(1, "rel", &ev(EventKind::SubscriptionInit, 0x2, 0xA, 0));
        t.observe(2, "be", &qos_ev(0x3, 0xA, 2, 2, 1));
        t.observe(2, "be", &ev(EventKind::SubscriptionInit, 0x3, 0xA, 0));
        // And topic 0xB: reliable pub at 20 Hz, reliable sub taking 5 Hz.
        t.observe(3, "p2", &qos_ev(0x4, 0xB, 1, 2, 10));
        t.observe(3, "p2", &ev(EventKind::PublisherInit, 0x4, 0xB, 0));
        t.observe(4, "s2", &qos_ev(0x5, 0xB, 1, 2, 1));
        t.observe(4, "s2", &ev(EventKind::SubscriptionInit, 0x5, 0xB, 0));
        for s in 0..20u64 {
            for k in 0..20u64 {
                let at = s * NS + k * (NS / 20);
                t.observe(0, "pub", &ev(EventKind::Publish, 0x1, 0xA, at));
                t.observe(3, "p2", &ev(EventKind::Publish, 0x4, 0xB, at));
                if k % 4 == 0 {
                    t.observe(4, "s2", &ev(EventKind::Take, 0x5, 0xB, at));
                }
                if k % 2 == 0 {
                    t.observe(2, "be", &ev(EventKind::Take, 0x3, 0xA, at));
                }
            }
        }
        let (mut n, ty) = names();
        n.insert(0xB, "/b".to_string());
        let snap = t.live_snapshot(20 * NS + 1, &n, &ty, 0);
        let row = |member: &str| snap.rows.iter().find(|r| r.member == member).unwrap();
        let codes =
            |member: &str| -> Vec<&str> { row(member).flags.iter().map(|f| f.code).collect() };

        assert_eq!(codes("pub"), ["reliability"]);
        assert_eq!(
            codes("rel"),
            ["reliability", "silent"],
            "nothing reaches a subscription DDS never matched"
        );
        assert!(
            codes("be").is_empty(),
            "a best-effort subscriber may drop without it being a deficit: {:?}",
            row("be").flags
        );
        assert_eq!(codes("s2"), ["take_deficit"]);
        assert!(
            row("s2").flags[0]
                .message
                .contains("queue depth 1 keeps only the latest")
        );
        assert!((row("p2").rate_10s - 20.0).abs() < 1e-9);
        assert!((row("s2").rate_10s - 5.0).abs() < 1e-9);
        assert_eq!(snap.totals.incompatible, 2);
        assert_eq!(snap.totals.advisories, 1);
        assert_eq!(row("rel").flags[0].kind, "incompatible");
    }

    #[test]
    fn health_turns_a_mismatch_into_one_issue_per_topic() {
        let mut t = EndpointTracker::default();
        // 0xA: best-effort pub, two reliable subs (one incompatible pair each).
        t.observe(0, "pub", &qos_ev(0x1, 0xA, 2, 2, 10));
        t.observe(0, "pub", &ev(EventKind::PublisherInit, 0x1, 0xA, 0));
        for (child, h) in [(1, 0x2), (2, 0x3)] {
            t.observe(child, "sub", &qos_ev(h, 0xA, 1, 2, 10));
            t.observe(child, "sub", &ev(EventKind::SubscriptionInit, h, 0xA, 0));
        }
        // 0xB: reliable pub at 20 Hz, a depth-1 sub taking 5 Hz.
        t.observe(3, "p2", &qos_ev(0x4, 0xB, 1, 2, 10));
        t.observe(3, "p2", &ev(EventKind::PublisherInit, 0x4, 0xB, 0));
        t.observe(4, "s2", &qos_ev(0x5, 0xB, 1, 2, 1));
        t.observe(4, "s2", &ev(EventKind::SubscriptionInit, 0x5, 0xB, 0));
        for s in 0..20u64 {
            for k in 0..20u64 {
                let at = s * NS + k * (NS / 20);
                t.observe(0, "pub", &ev(EventKind::Publish, 0x1, 0xA, at));
                t.observe(3, "p2", &ev(EventKind::Publish, 0x4, 0xB, at));
                if k % 4 == 0 {
                    t.observe(4, "s2", &ev(EventKind::Take, 0x5, 0xB, at));
                }
            }
        }
        let (mut n, ty) = names();
        n.insert(0xB, "/b".to_string());
        let snap = t.live_snapshot(20 * NS + 1, &n, &ty, 0);
        let conds = crate::health::entity_conditions(&snap);
        let kinds = |k: &str| conds.iter().filter(|c| c.kind == k).count();
        assert_eq!(
            kinds("qos_incompatible"),
            1,
            "one issue for the topic, not one per side"
        );
        assert_eq!(kinds("take_deficit"), 1);
        assert_eq!(kinds("silent"), 1, "silent endpoints are one summary");
        let q = conds.iter().find(|c| c.kind == "qos_incompatible").unwrap();
        assert_eq!(q.subject.name, "/a");
        assert!(q.detail.contains("reliability"), "{}", q.detail);
    }

    #[test]
    fn a_subscription_that_takes_late_is_flagged_and_a_prompt_one_is_not() {
        let mut t = EndpointTracker::default();
        let (n, ty) = names();
        t.observe(0, "talker", &ev(EventKind::PublisherInit, 1, 0xA, 0));
        t.observe(1, "fast", &ev(EventKind::SubscriptionInit, 2, 0xA, 0));
        t.observe(2, "slow", &ev(EventKind::SubscriptionInit, 3, 0xA, 0));
        let stamped = |kind, handle, i: u64, mono| {
            let mut e = ev(kind, handle, 0xA, mono);
            e.stamp_sec = 1000;
            e.stamp_nanosec = i as u32;
            e
        };
        // 20 Hz for 20 s; the fast subscriber takes 1 ms after each publish,
        // the slow one 300 ms after, read before the publish half the time
        // (its ring drained first).
        let ms = NS / 1000;
        for i in 1..=400u64 {
            let at = i * 50 * ms;
            if i % 2 == 0 {
                t.observe(2, "slow", &stamped(EventKind::Take, 3, i, at + 300 * ms));
            }
            t.observe(0, "talker", &stamped(EventKind::Publish, 1, i, at));
            // One late take among prompt ones: the first after discovery.
            let late = if i == 300 { 150 * ms } else { ms };
            t.observe(1, "fast", &stamped(EventKind::Take, 2, i, at + late));
            if i % 2 == 1 {
                t.observe(2, "slow", &stamped(EventKind::Take, 3, i, at + 300 * ms));
            }
            t.settle(at + 300 * ms);
        }
        let snap = t.live_snapshot(20 * NS + NS / 2, &n, &ty, 0);
        let row = |m: &str| snap.rows.iter().find(|r| r.member == m).unwrap();
        let fast = row("fast").latency.clone().unwrap();
        let slow = row("slow").latency.clone().unwrap();
        assert!(fast.p50_ms.unwrap() <= 1.25, "{fast:?}");
        assert!(
            fast.max_ms.unwrap() >= 150.0,
            "the outlier is in the window: {fast:?}"
        );
        assert!((300.0..=375.0).contains(&slow.p99_ms.unwrap()), "{slow:?}");
        assert_eq!(slow.coverage, 1.0, "every take matched: {slow:?}");
        assert!(row("fast").flags.is_empty(), "{:?}", row("fast").flags);
        assert_eq!(row("slow").flags[0].code, "delivery_latency");
        assert!(row("talker").latency.is_none(), "a publisher has none");
    }

    #[test]
    fn event_channels_are_judged_for_neither_latency_nor_deficit() {
        let mut t = EndpointTracker::default();
        let mut n = HashMap::new();
        n.insert(0xA, "/parameter_events".to_string());
        t.observe(0, "p", &ev(EventKind::PublisherInit, 1, 0xA, 0));
        t.observe(1, "s", &ev(EventKind::SubscriptionInit, 2, 0xA, 0));
        let ms = NS / 1000;
        for i in 1..=400u64 {
            let at = i * 50 * ms;
            let mut p = ev(EventKind::Publish, 1, 0xA, at);
            p.stamp_sec = 7;
            p.stamp_nanosec = i as u32;
            t.observe(0, "p", &p);
            if i % 2 == 0 {
                let mut k = ev(EventKind::Take, 2, 0xA, at + 400 * ms);
                k.stamp_sec = 7;
                k.stamp_nanosec = i as u32;
                t.observe(1, "s", &k);
            }
        }
        let snap = t.live_snapshot(20 * NS + NS / 2, &n, &HashMap::new(), 0);
        let s = snap.rows.iter().find(|r| r.member == "s").unwrap();
        assert!(s.latency.is_some(), "measured, shown");
        assert!(s.flags.is_empty(), "but not judged: {:?}", s.flags);
    }

    #[test]
    fn a_startup_burst_is_not_a_stream_that_went_stale() {
        let mut t = EndpointTracker::default();
        // Three messages at startup (`/parameter_events`), then nothing.
        t.observe(0, "m", &ev(EventKind::PublisherInit, 1, 0xA, 0));
        for k in 0..3 {
            t.observe(0, "m", &ev(EventKind::Publish, 1, 0xA, k));
        }
        // A 10 Hz stream for 10 s, then nothing.
        t.observe(0, "m", &ev(EventKind::PublisherInit, 2, 0xA, 0));
        for k in 0..100 {
            t.observe(0, "m", &ev(EventKind::Publish, 2, 0xA, k * NS / 10));
        }
        let (n, ty) = names();
        let snap = t.live_snapshot(40 * NS, &n, &ty, 0);
        let flags = |id: &str| -> Vec<&str> {
            let r = snap.rows.iter().find(|r| r.id == id).unwrap();
            r.flags.iter().map(|f| f.code).collect()
        };
        assert!(flags("0:1").is_empty(), "{:?}", flags("0:1"));
        assert_eq!(flags("0:2"), ["stale"]);
    }

    #[test]
    fn trimming_keeps_flagged_rows_past_the_limit() {
        let mut t = EndpointTracker::default();
        for h in 0..5u64 {
            t.observe(0, "m", &ev(EventKind::PublisherInit, h, 0xA, 0));
            for k in 0..(10 - h) {
                t.observe(0, "m", &ev(EventKind::Publish, h, 0xA, k * NS));
            }
        }
        t.observe(0, "m", &ev(EventKind::SubscriptionInit, 9, 0xA, 0));
        let (n, ty) = names();
        let snap = t.live_snapshot(30 * NS, &n, &ty, 0);
        // Every publisher went stale at t = 30 s, so all are flagged.
        let trimmed = snap.trimmed(2);
        assert_eq!(trimmed.rows.len(), 5, "stale rows survive the trim");
        assert_eq!(trimmed.truncated, 1, "the silent subscription is cut");
        assert_eq!(trimmed.totals.rows, 6);
    }
}
