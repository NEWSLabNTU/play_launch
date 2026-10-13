//! Data-flow chains found in the running system, with no contract
//! (`docs/design/data-flow-chains.md`, phase 88 C1): CARRIED edges.
//!
//! Node N took a message stamped `s` on topic A and later published `s` on
//! topic B, so A →N→ B is a real edge and `t_publish − t_take` is N's latency
//! for it, queueing included. The evidence is grounded in the node: a stamp
//! two topics merely share (a diagnostic about an object list, siblings
//! stamped from one input) is no edge unless one node took it on one and
//! published it on the other.
//!
//! A node's takes and publishes come through one ring, its process's, in the
//! order they happened, so unlike delivery latency (`latency.rs`) a publish
//! never arrives before the take it carries a stamp from.
//!
//! A node is `(ring, node hash)` from `EndpointOwner`: the composables of one
//! container are separate nodes here.
//!
//! C2 adds SAMPLED edges: a publish by N on thread T that carried no stamp N
//! took is attributed to the latest take N made on T, stamped or not. That
//! says the data flowed (the input sat in a buffer the callback could read)
//! and not what triggered the output: a timer node with several inputs is
//! attributed to whichever it handled last. So a sampled edge is reported
//! apart from the carried ones, with its `share` of the output's publishes,
//! and only once seen [`CHAIN_MIN_SAMPLES`] times. Its time is from the
//! node's latest take of the input on ANY thread to the publish: a lower
//! bound on the hop's staleness, since the node cannot have used data newer
//! than it took. The thread's own latest take is no bound at all on a
//! multi-threaded executor: on a driven Autoware it read 4 to 10 s for the
//! behavior planner's status topics, published from a thread that had not
//! taken the map for seconds while others had. Keying the latest take by the
//! node as well as the thread is the guard that the input is among the
//! node's own subscriptions: composables sharing an executor thread do not
//! lend each other inputs.
//!
//! C3 propagates an ORIGIN with each publish: the time of the oldest
//! publish behind it along the hops that produced it, and the topic that
//! publish was on. A carried hop passes on the origin of the message whose
//! stamp it carries; a sampled hop passes on the origin of the input it is
//! attributed to (the thread's latest take), as the node's latest take of
//! that input. A publish with neither starts a new origin. `t_publish −
//! origin` is the topic's DATA AGE, kept as a histogram per topic, with the
//! share of publishes whose age passes through a sampled hop or an input
//! whose own publish was not seen (`estimated_share`).
//!
//! Two departures from the design, both forced by feedback. The design took
//! the OLDEST origin among a node's sampled inputs; that is dominated by any
//! state taken once (a gear command, a route) and, around a loop, never
//! refreshes: a simulator's state is older than the command that used it,
//! which is older than the state, and the minimum sticks at the first
//! publish. The attributed input is the one C2 already reports. And an
//! origin remembers the last [`ORIGIN_PATH`] topics it passed through; a
//! hop that would bring it back to one of them closes a loop, and the
//! output starts a new origin instead. So in the simulator → controller →
//! simulator loop the state is born where it is published, and the
//! command's age is how old the state behind it is.
//!
//! An input's origin is the one recorded at its publish, and the publish of
//! a message another process took may not have been read yet: each process
//! has its own ring and they drain in turn. A publish whose input is not
//! resolved waits, like a take in `latency.rs`, and is retried after each
//! drain pass ([`ChainBook::settle`]). Without that, a hop across processes
//! a few microseconds after its input restarted the origin: on Autoware the
//! scenario selector's trajectory read 0 ms of age with 283 ms of planning
//! behind it.

use std::collections::{HashMap, HashSet, VecDeque};

use serde::Serialize;

use super::latency::Latency;

/// Stamps remembered per node, latest take of each.
const STAMPS_PER_NODE: usize = 1024;
/// An edge needs this many observations to join a chain.
pub(crate) const CHAIN_MIN_SAMPLES: u64 = 20;
/// Chains reported, longest first.
const CHAINS_SHOWN: usize = 20;
/// Paths walked while looking for the longest; bounds a dense graph.
const PATHS_WALKED: usize = 20_000;
/// Topics every node publishes on with stamps of its own making.
const INFRASTRUCTURE: [&str; 2] = ["/rosout", "/parameter_events"];

/// Origins remembered per topic, by stamp. A take is resolved at its node's
/// next publish, so this need only outlast a few messages of queueing.
const ORIGINS_PER_TOPIC: usize = 64;
/// Topics an origin remembers having passed through, for loop detection.
const ORIGIN_PATH: usize = 8;

/// The topic hashes of [`INFRASTRUCTURE`] (the `.so` hashes names with
/// FNV-1a). A take on one never becomes a node's attributed input: every
/// node takes `/parameter_events` once at startup, and a timer node whose
/// thread took nothing since read 12 s of data age on a driven Autoware.
static INFRASTRUCTURE_HASHES: std::sync::LazyLock<[u64; 2]> =
    std::sync::LazyLock::new(|| INFRASTRUCTURE.map(crate::health::fnv));

/// `(ring, node hash)`; hash 0 when the endpoint's owner was not announced.
pub(crate) type NodeKey = (usize, u64);

#[derive(Debug, Default)]
struct Taken {
    /// stamp -> (topic, take time).
    at: HashMap<u64, (u64, u64)>,
    order: VecDeque<u64>,
}

/// Where the data behind a publish began.
#[derive(Debug, Clone, Copy)]
struct Origin {
    ns: u64,
    /// The topic the oldest publish was on.
    source: u64,
    /// A sampled hop, or an input whose publish was not seen, on the way.
    estimated: bool,
    /// The last topics passed through, newest last.
    path: [u64; ORIGIN_PATH],
    len: u8,
}

impl Origin {
    fn born(topic: u64, ns: u64, estimated: bool) -> Self {
        let mut path = [0; ORIGIN_PATH];
        path[0] = topic;
        Origin {
            ns,
            source: topic,
            estimated,
            path,
            len: 1,
        }
    }

    fn visited(&self, topic: u64) -> bool {
        self.path[..self.len as usize].contains(&topic)
    }

    fn through(mut self, topic: u64) -> Self {
        if (self.len as usize) == ORIGIN_PATH {
            self.path.copy_within(1.., 0);
            self.len -= 1;
        }
        self.path[self.len as usize] = topic;
        self.len += 1;
        self
    }
}

/// A publish waiting for its input's origin.
#[derive(Debug)]
struct Pending {
    topic: u64,
    stamp: Option<u64>,
    at_ns: u64,
    /// (input, its stamp, take time, sampled hop).
    from: Option<(u64, Option<u64>, u64, bool)>,
}

/// Publishes held for their input's origin, at most; beyond, the take time
/// stands in at once.
const PENDING_MAX: usize = 65_536;
/// How long a publish waits for its input's publish to be read.
const PENDING_EXPIRY_NS: u64 = 1_000_000_000;

#[derive(Debug, Default)]
struct Origins {
    at: HashMap<u64, Origin>,
    order: VecDeque<u64>,
}

#[derive(Debug, Default)]
struct Age {
    latency: Latency,
    estimated: u64,
    /// source topic -> publishes.
    sources: HashMap<u64, u64>,
}

#[derive(Debug, Default)]
struct Edge {
    latency: Latency,
    first_ns: u64,
    last_ns: u64,
    /// The latest observation's take-to-publish time.
    last_latency_ns: u64,
}

#[derive(Debug, Default)]
pub(crate) struct ChainBook {
    taken: HashMap<NodeKey, Taken>,
    /// (input topic, node, output topic) -> edge.
    edges: HashMap<(u64, NodeKey, u64), Edge>,
    /// (node, thread) -> topic of the latest take on that thread.
    last_take: HashMap<(NodeKey, u32), u64>,
    /// (node, topic) -> the node's latest take of it, on any thread, and
    /// that take's stamp.
    latest_input: HashMap<(NodeKey, u64), (u64, Option<u64>)>,
    /// topic -> stamp -> origin, of its recent publishes.
    origins: HashMap<u64, Origins>,
    /// topic -> data age.
    ages: HashMap<u64, Age>,
    /// Publishes whose input's publish was not read yet, oldest first.
    pending: VecDeque<Pending>,
    /// (input topic, node, output topic) -> sampled edge.
    sampled: HashMap<(u64, NodeKey, u64), Edge>,
    /// (node, output topic) -> publishes, for a sampled edge's share.
    outputs: HashMap<(NodeKey, u64), u64>,
}

impl Edge {
    fn observe(&mut self, at_ns: u64, latency_ns: u64) {
        if self.first_ns == 0 {
            self.first_ns = at_ns;
        }
        self.last_ns = at_ns;
        self.last_latency_ns = latency_ns;
        self.latency.add(at_ns, latency_ns);
    }
}

impl ChainBook {
    /// A take by `node` on thread `tid` (0 when the `.so` did not say).
    pub(crate) fn take(
        &mut self,
        node: NodeKey,
        tid: u32,
        topic: u64,
        stamp: Option<u64>,
        at_ns: u64,
    ) {
        if tid != 0 && !INFRASTRUCTURE_HASHES.contains(&topic) {
            self.last_take.insert((node, tid), topic);
            self.latest_input.insert((node, topic), (at_ns, stamp));
        }
        let Some(stamp) = stamp else { return };
        let t = self.taken.entry(node).or_default();
        if t.at.insert(stamp, (topic, at_ns)).is_none() {
            if t.order.len() >= STAMPS_PER_NODE
                && let Some(old) = t.order.pop_front()
            {
                t.at.remove(&old);
            }
            t.order.push_back(stamp);
        }
    }

    /// A publish: a carried edge when `node` took this stamp, else a sampled
    /// edge from the latest take on the same thread.
    pub(crate) fn publish(
        &mut self,
        node: NodeKey,
        tid: u32,
        topic: u64,
        stamp: Option<u64>,
        at_ns: u64,
    ) {
        if INFRASTRUCTURE_HASHES.contains(&topic) {
            return;
        }
        *self.outputs.entry((node, topic)).or_default() += 1;
        let carried = stamp
            .and_then(|s| self.taken.get(&node).and_then(|t| t.at.get(&s)))
            .copied();
        // The input's origin: (input, its stamp, take time, sampled?).
        let mut from: Option<(u64, Option<u64>, u64, bool)> = None;
        if let Some((input, take_ns)) = carried {
            if input != topic && take_ns <= at_ns {
                self.edges
                    .entry((input, node, topic))
                    .or_default()
                    .observe(at_ns, at_ns - take_ns);
                from = Some((input, stamp, take_ns, false));
            }
        } else if tid != 0
            && let Some(&input) = self.last_take.get(&(node, tid))
        {
            let (take_ns, take_stamp) = self.latest_input[&(node, input)];
            if input != topic && take_ns <= at_ns {
                self.sampled
                    .entry((input, node, topic))
                    .or_default()
                    .observe(at_ns, at_ns - take_ns);
                from = Some((input, take_stamp, take_ns, true));
            }
        }
        let p = Pending {
            topic,
            stamp,
            at_ns,
            from,
        };
        // A stamped input whose publish has not been read yet: its process's
        // ring may simply drain after this one. Wait for the pass to end.
        if self.pending.is_empty() && self.ready(&p) {
            self.finish(&p, false);
        } else if self.pending.len() < PENDING_MAX {
            self.pending.push_back(p);
        } else {
            self.finish(&p, true);
        }
    }

    /// Whether a publish's input origin can be resolved now.
    fn ready(&self, p: &Pending) -> bool {
        match p.from {
            Some((input, Some(s), _, _)) => self
                .origins
                .get(&input)
                .is_some_and(|r| r.at.contains_key(&s)),
            _ => true,
        }
    }

    /// Resolve publishes that waited for their input's publish; call after
    /// each drain pass. In time order, repeatedly, since one may feed the
    /// next; one waiting past [`PENDING_EXPIRY_NS`] (its input published
    /// outside the run, or forgotten) falls back to its take time.
    pub(crate) fn settle(&mut self, now_ns: u64) {
        if self.pending.is_empty() {
            return;
        }
        self.pending.make_contiguous().sort_by_key(|p| p.at_ns);
        loop {
            let mut progress = false;
            let mut rest = VecDeque::with_capacity(self.pending.len());
            while let Some(p) = self.pending.pop_front() {
                if self.ready(&p) {
                    self.finish(&p, false);
                    progress = true;
                } else if now_ns.saturating_sub(p.at_ns) > PENDING_EXPIRY_NS {
                    self.finish(&p, true);
                    progress = true;
                } else {
                    rest.push_back(p);
                }
            }
            self.pending = rest;
            if !progress || self.pending.is_empty() {
                break;
            }
        }
    }

    /// Record a publish's data age and remember its origin. `fallback`: the
    /// input's origin is not coming, so its take time stands in.
    fn finish(&mut self, p: &Pending, fallback: bool) {
        let from = p.from.map(|(input, s, take_ns, sampled)| {
            let mut o = if fallback {
                Origin::born(input, take_ns, true)
            } else {
                self.resolve(input, s, take_ns)
            };
            o.estimated |= sampled;
            o
        });
        let (topic, at_ns) = (p.topic, p.at_ns);
        let origin = match from {
            Some(o) if !o.visited(topic) && o.ns <= at_ns => {
                let age = self.ages.entry(topic).or_default();
                age.latency.add(at_ns, at_ns - o.ns);
                age.estimated += o.estimated as u64;
                *age.sources.entry(o.source).or_default() += 1;
                o.through(topic)
            }
            _ => Origin::born(topic, at_ns, false),
        };
        if let Some(stamp) = p.stamp {
            let r = self.origins.entry(topic).or_default();
            if r.at.insert(stamp, origin).is_none() {
                if r.order.len() >= ORIGINS_PER_TOPIC
                    && let Some(old) = r.order.pop_front()
                {
                    r.at.remove(&old);
                }
                r.order.push_back(stamp);
            }
        }
    }

    /// The origin of the message a node took on `input`: the one recorded
    /// when it was published, else (unstamped, or its publisher unseen) the
    /// take itself, marked estimated.
    fn resolve(&self, input: u64, stamp: Option<u64>, take_ns: u64) -> Origin {
        stamp
            .and_then(|s| self.origins.get(&input)?.at.get(&s).copied())
            .unwrap_or_else(|| Origin::born(input, take_ns, true))
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.edges.is_empty() && self.sampled.is_empty()
    }

    /// The edges with names, and the longest chains over those seen at
    /// least [`CHAIN_MIN_SAMPLES`] times.
    pub(crate) fn snapshot(
        &self,
        now_ns: u64,
        topic_names: &HashMap<u64, String>,
        node_name: impl Fn(NodeKey) -> String,
    ) -> ChainSnapshot {
        let secs = |ns: u64| ns as f64 / 1e9;
        let mut edges: Vec<CarriedEdge> = Vec::new();
        for ((a, node, b), e) in &self.edges {
            let (Some(from), Some(to)) = (topic_names.get(a), topic_names.get(b)) else {
                continue;
            };
            if INFRASTRUCTURE.contains(&from.as_str()) || INFRASTRUCTURE.contains(&to.as_str()) {
                continue;
            }
            let run = &e.latency.run;
            let recent = e.latency.recent(now_ns);
            let life = secs(e.last_ns.saturating_sub(e.first_ns));
            edges.push(CarriedEdge {
                from: from.clone(),
                node: node_name(*node),
                to: to.clone(),
                samples: run.samples(),
                rate_hz: if life >= 1.0 {
                    (run.samples() - 1) as f64 / life
                } else {
                    0.0
                },
                p50_ms: run.quantile_ms(0.5),
                p99_ms: run.quantile_ms(0.99),
                max_ms: run.max_ms(),
                recent_samples: recent.samples(),
                recent_p50_ms: recent.quantile_ms(0.5),
                last_ms: e.last_latency_ns as f64 / 1e6,
                since_last_s: secs(now_ns.saturating_sub(e.last_ns)),
            });
        }
        edges.sort_by(|x, y| {
            y.samples
                .cmp(&x.samples)
                .then_with(|| x.from.cmp(&y.from))
                .then_with(|| x.to.cmp(&y.to))
                .then_with(|| x.node.cmp(&y.node))
        });
        let chains = longest_chains(&edges);

        let mut sampled: Vec<SampledEdge> = Vec::new();
        for (k @ (a, node, b), e) in &self.sampled {
            if self.edges.contains_key(k) || e.latency.run.samples() < CHAIN_MIN_SAMPLES {
                continue;
            }
            let (Some(from), Some(to)) = (topic_names.get(a), topic_names.get(b)) else {
                continue;
            };
            if INFRASTRUCTURE.contains(&from.as_str()) || INFRASTRUCTURE.contains(&to.as_str()) {
                continue;
            }
            let run = &e.latency.run;
            let outputs = self.outputs.get(&(*node, *b)).copied().unwrap_or(0);
            sampled.push(SampledEdge {
                from: from.clone(),
                node: node_name(*node),
                to: to.clone(),
                samples: run.samples(),
                share: if outputs > 0 {
                    run.samples() as f64 / outputs as f64
                } else {
                    0.0
                },
                p50_ms: run.quantile_ms(0.5),
                p99_ms: run.quantile_ms(0.99),
                max_ms: run.max_ms(),
                last_ms: e.last_latency_ns as f64 / 1e6,
                since_last_s: secs(now_ns.saturating_sub(e.last_ns)),
            });
        }
        sampled.sort_by(|x, y| {
            x.to.cmp(&y.to)
                .then_with(|| x.node.cmp(&y.node))
                .then_with(|| y.samples.cmp(&x.samples))
                .then_with(|| x.from.cmp(&y.from))
        });
        let mut ages: Vec<TopicAge> = Vec::new();
        for (t, a) in &self.ages {
            let run = &a.latency.run;
            if run.samples() < CHAIN_MIN_SAMPLES {
                continue;
            }
            let Some(topic) = topic_names.get(t) else {
                continue;
            };
            if INFRASTRUCTURE.contains(&topic.as_str()) {
                continue;
            }
            let (source, n) = a
                .sources
                .iter()
                .max_by_key(|(s, n)| (**n, std::cmp::Reverse(**s)))
                .map(|(s, n)| (*s, *n))
                .unwrap_or_default();
            let recent = a.latency.recent(now_ns);
            ages.push(TopicAge {
                topic: topic.clone(),
                samples: run.samples(),
                p50_ms: run.quantile_ms(0.5),
                p99_ms: run.quantile_ms(0.99),
                max_ms: run.max_ms(),
                recent_p50_ms: recent.quantile_ms(0.5),
                estimated_share: a.estimated as f64 / run.samples() as f64,
                source: topic_names
                    .get(&source)
                    .cloned()
                    .unwrap_or_else(|| format!("{source:#x}")),
                source_share: n as f64 / run.samples() as f64,
            });
        }
        ages.sort_by(|x, y| x.topic.cmp(&y.topic));
        ChainSnapshot {
            edges,
            sampled,
            chains,
            ages,
        }
    }
}

/// One carried edge: `node` took a stamp on `from` and published it on `to`.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct CarriedEdge {
    pub from: String,
    pub node: String,
    pub to: String,
    pub samples: u64,
    /// Observations per second over the edge's life.
    pub rate_hz: f64,
    /// Take to publish in the node, over the run.
    pub p50_ms: Option<f64>,
    pub p99_ms: Option<f64>,
    pub max_ms: Option<f64>,
    /// Over the last 10 to 20 s.
    pub recent_samples: u64,
    pub recent_p50_ms: Option<f64>,
    /// The latest observation.
    pub last_ms: f64,
    pub since_last_s: f64,
}

/// One sampled edge: `node` published on `to` with no stamp it took, on the
/// thread whose latest take was on `from`. The data flowed; the trigger is
/// unknown, and a node with several inputs shows one edge per input.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct SampledEdge {
    pub from: String,
    pub node: String,
    pub to: String,
    pub samples: u64,
    /// Of `node`'s publishes on `to`, the fraction attributed to `from`.
    pub share: f64,
    /// The node's latest take of `from` to the publish: a lower bound on the
    /// hop's staleness.
    pub p50_ms: Option<f64>,
    pub p99_ms: Option<f64>,
    pub max_ms: Option<f64>,
    pub last_ms: f64,
    pub since_last_s: f64,
}

/// How old the data behind a topic's publishes is: publish time minus the
/// origin propagated along carried and sampled hops.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct TopicAge {
    pub topic: String,
    pub samples: u64,
    pub p50_ms: Option<f64>,
    pub p99_ms: Option<f64>,
    pub max_ms: Option<f64>,
    /// Over the last 10 to 20 s.
    pub recent_p50_ms: Option<f64>,
    /// Publishes whose age passed through a sampled hop, or through an input
    /// whose publish was not seen: approximate, not measured.
    pub estimated_share: f64,
    /// The topic the oldest data most often began on, and how often.
    pub source: String,
    pub source_share: f64,
}

/// A chain of topics joined by carried edges, source first.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Chain {
    pub topics: Vec<String>,
    /// The node of each hop; one fewer than `topics`.
    pub nodes: Vec<String>,
    /// Sum of the hops' median latencies.
    pub p50_ms: f64,
}

#[derive(Debug, Clone, Default, Serialize, PartialEq)]
pub struct ChainSnapshot {
    /// Carried edges.
    pub edges: Vec<CarriedEdge>,
    /// Sampled edges seen at least [`CHAIN_MIN_SAMPLES`] times, minus those
    /// a carried edge already states.
    pub sampled: Vec<SampledEdge>,
    pub chains: Vec<Chain>,
    /// Data age per topic, for topics with an upstream seen at least
    /// [`CHAIN_MIN_SAMPLES`] times.
    pub ages: Vec<TopicAge>,
}

/// One hop of a path between two topics.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Hop {
    pub from: String,
    pub node: String,
    pub to: String,
    /// `carried` (the stamp crossed the node: measured) or `sampled` (the
    /// thread's latest input: the data flowed, the time is a lower bound).
    pub kind: &'static str,
    pub samples: u64,
    pub p50_ms: Option<f64>,
    pub p99_ms: Option<f64>,
    /// A sampled hop's share of the node's publishes on `to`.
    pub share: Option<f64>,
}

/// A path of hops from a source topic to a sink topic.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct ChainPath {
    pub hops: Vec<Hop>,
    /// Sums of the hops' medians and 99th percentiles. The second bounds the
    /// path's p99 from above only if the hops' worst cases coincide.
    pub p50_ms: f64,
    pub p99_ms: f64,
    pub carried_hops: usize,
    pub sampled_hops: usize,
}

/// Paths shown between a source and a sink.
pub const PATHS_SHOWN: usize = 10;
/// Hops in a path, at most.
const PATH_MAX_HOPS: usize = 16;

impl ChainSnapshot {
    /// Simple paths from `from` to `to` over carried edges seen at least
    /// [`CHAIN_MIN_SAMPLES`] times and the sampled edges reported, fewest
    /// hops first, then fastest. `truncated` when the walk hit its bound.
    pub fn paths(&self, from: &str, to: &str) -> (Vec<ChainPath>, bool) {
        let mut succ: HashMap<&str, Vec<Hop>> = HashMap::new();
        for e in self.edges.iter().filter(|e| e.samples >= CHAIN_MIN_SAMPLES) {
            succ.entry(e.from.as_str()).or_default().push(Hop {
                from: e.from.clone(),
                node: e.node.clone(),
                to: e.to.clone(),
                kind: "carried",
                samples: e.samples,
                p50_ms: e.p50_ms,
                p99_ms: e.p99_ms,
                share: None,
            });
        }
        for e in &self.sampled {
            succ.entry(e.from.as_str()).or_default().push(Hop {
                from: e.from.clone(),
                node: e.node.clone(),
                to: e.to.clone(),
                kind: "sampled",
                samples: e.samples,
                p50_ms: e.p50_ms,
                p99_ms: e.p99_ms,
                share: Some(e.share),
            });
        }
        let mut found: Vec<Vec<&Hop>> = Vec::new();
        let mut walked = 0usize;
        #[allow(clippy::too_many_arguments)]
        fn walk<'a>(
            at: &'a str,
            to: &str,
            succ: &'a HashMap<&str, Vec<Hop>>,
            path: &mut Vec<&'a Hop>,
            seen: &mut HashSet<&'a str>,
            found: &mut Vec<Vec<&'a Hop>>,
            walked: &mut usize,
        ) {
            *walked += 1;
            if *walked > PATHS_WALKED || path.len() >= PATH_MAX_HOPS {
                return;
            }
            for h in succ.get(at).into_iter().flatten() {
                if h.to == to {
                    path.push(h);
                    found.push(path.clone());
                    path.pop();
                    continue;
                }
                if seen.insert(h.to.as_str()) {
                    path.push(h);
                    walk(h.to.as_str(), to, succ, path, seen, found, walked);
                    path.pop();
                    seen.remove(h.to.as_str());
                }
            }
        }
        if from != to {
            let mut seen = HashSet::from([from]);
            walk(
                from,
                to,
                &succ,
                &mut Vec::new(),
                &mut seen,
                &mut found,
                &mut walked,
            );
        }
        let truncated = walked > PATHS_WALKED;
        let mut paths: Vec<ChainPath> = found
            .into_iter()
            .map(|hops| {
                let carried_hops = hops.iter().filter(|h| h.kind == "carried").count();
                ChainPath {
                    p50_ms: hops.iter().filter_map(|h| h.p50_ms).sum(),
                    p99_ms: hops.iter().filter_map(|h| h.p99_ms).sum(),
                    carried_hops,
                    sampled_hops: hops.len() - carried_hops,
                    hops: hops.into_iter().cloned().collect(),
                }
            })
            .collect();
        paths.sort_by(|a, b| {
            a.hops
                .len()
                .cmp(&b.hops.len())
                .then(a.p50_ms.total_cmp(&b.p50_ms))
        });
        paths.truncate(PATHS_SHOWN);
        (paths, truncated)
    }
}

/// The longest simple paths over well-observed edges, starting at topics no
/// such edge leads to (or anywhere in a cycle with no entry). Paths that are
/// a prefix or suffix of a longer one reported are dropped.
fn longest_chains(edges: &[CarriedEdge]) -> Vec<Chain> {
    let mut succ: HashMap<&str, Vec<&CarriedEdge>> = HashMap::new();
    let mut has_parent: HashSet<&str> = HashSet::new();
    for e in edges.iter().filter(|e| e.samples >= CHAIN_MIN_SAMPLES) {
        succ.entry(e.from.as_str()).or_default().push(e);
        has_parent.insert(e.to.as_str());
    }
    let mut roots: Vec<&str> = succ
        .keys()
        .copied()
        .filter(|t| !has_parent.contains(t))
        .collect();
    if roots.is_empty() {
        roots = succ.keys().copied().collect();
    }
    roots.sort();

    let mut found: Vec<Vec<&CarriedEdge>> = Vec::new();
    let mut walked = 0usize;
    fn walk<'a>(
        at: &'a str,
        succ: &HashMap<&'a str, Vec<&'a CarriedEdge>>,
        path: &mut Vec<&'a CarriedEdge>,
        seen: &mut HashSet<&'a str>,
        found: &mut Vec<Vec<&'a CarriedEdge>>,
        walked: &mut usize,
    ) {
        let next: Vec<&&CarriedEdge> = succ
            .get(at)
            .map(|v| v.iter().filter(|e| !seen.contains(e.to.as_str())).collect())
            .unwrap_or_default();
        if next.is_empty() || *walked >= PATHS_WALKED {
            *walked += 1;
            if !path.is_empty() {
                found.push(path.clone());
            }
            return;
        }
        for e in next {
            seen.insert(e.to.as_str());
            path.push(e);
            walk(e.to.as_str(), succ, path, seen, found, walked);
            path.pop();
            seen.remove(e.to.as_str());
        }
    }
    for r in roots {
        let mut seen = HashSet::from([r]);
        walk(
            r,
            &succ,
            &mut Vec::new(),
            &mut seen,
            &mut found,
            &mut walked,
        );
    }

    found.sort_by_key(|p| std::cmp::Reverse(p.len()));
    let mut out: Vec<Chain> = Vec::new();
    for p in found {
        let topics: Vec<String> = std::iter::once(p[0].from.clone())
            .chain(p.iter().map(|e| e.to.clone()))
            .collect();
        let covered = out.iter().any(|c| {
            c.topics
                .windows(topics.len())
                .any(|w| w == topics.as_slice())
        });
        if covered {
            continue;
        }
        out.push(Chain {
            nodes: p.iter().map(|e| e.node.clone()).collect(),
            p50_ms: p.iter().filter_map(|e| e.p50_ms).sum(),
            topics,
        });
        if out.len() >= CHAINS_SHOWN {
            break;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    fn names() -> HashMap<u64, String> {
        [(1, "/a"), (2, "/b"), (3, "/c"), (4, "/d"), (9, "/rosout")]
            .into_iter()
            .map(|(h, n)| (h, n.to_string()))
            .collect()
    }

    fn label(n: NodeKey) -> String {
        format!("n{}", n.1)
    }

    #[test]
    fn a_stamp_taken_and_republished_by_one_node_is_an_edge() {
        let mut b = ChainBook::default();
        b.take((0, 1), 0, 1, Some(7), 100 * MS);
        b.publish((0, 1), 0, 2, Some(7), 103 * MS);
        let s = b.snapshot(110 * MS, &names(), label);
        assert_eq!(s.edges.len(), 1);
        let e = &s.edges[0];
        assert_eq!(
            (e.from.as_str(), e.node.as_str(), e.to.as_str()),
            ("/a", "n1", "/b")
        );
        assert_eq!(e.last_ms, 3.0);
    }

    #[test]
    fn a_stamp_shared_by_two_nodes_is_no_edge() {
        // Node 1 took it; node 2 publishes the same stamp from elsewhere.
        let mut b = ChainBook::default();
        b.take((0, 1), 0, 1, Some(7), 100 * MS);
        b.publish((0, 2), 0, 2, Some(7), 103 * MS);
        // Same node, same topic in and out: a relay, not a hop between topics.
        b.take((1, 3), 0, 3, Some(8), 100 * MS);
        b.publish((1, 3), 0, 3, Some(8), 101 * MS);
        assert!(b.is_empty());
    }

    #[test]
    fn infrastructure_topics_are_not_edges() {
        let mut b = ChainBook::default();
        b.take((0, 1), 0, 1, Some(7), 100 * MS);
        b.publish((0, 1), 0, 9, Some(7), 101 * MS);
        assert!(!b.is_empty());
        assert!(b.snapshot(110 * MS, &names(), label).edges.is_empty());
    }

    #[test]
    fn well_observed_edges_join_into_the_longest_chain() {
        let mut b = ChainBook::default();
        for i in 1..=CHAIN_MIN_SAMPLES {
            let t = i * 100 * MS;
            b.take((0, 1), 0, 1, Some(i), t);
            b.publish((0, 1), 0, 2, Some(i), t + MS);
            b.take((0, 2), 0, 2, Some(i), t + 2 * MS);
            b.publish((0, 2), 0, 3, Some(i), t + 5 * MS);
            b.take((0, 3), 0, 3, Some(i), t + 6 * MS);
            b.publish((0, 3), 0, 4, Some(i), t + 7 * MS);
        }
        // A side branch seen too rarely to count.
        b.take((0, 4), 0, 2, Some(999), 50 * MS);
        b.publish((0, 4), 0, 9, Some(999), 51 * MS);
        let s = b.snapshot(10_000 * MS, &names(), label);
        assert_eq!(s.chains.len(), 1, "{:?}", s.chains);
        let c = &s.chains[0];
        assert_eq!(c.topics, ["/a", "/b", "/c", "/d"]);
        assert_eq!(c.nodes, ["n1", "n2", "n3"]);
    }

    #[test]
    fn the_stamps_a_node_remembers_are_bounded() {
        let mut b = ChainBook::default();
        for s in 1..=(STAMPS_PER_NODE as u64 + 1) {
            b.take((0, 1), 0, 1, Some(s), s * MS);
        }
        b.publish((0, 1), 0, 2, Some(1), 10_000 * MS);
        assert!(b.is_empty(), "stamp 1 was forgotten");
        b.publish((0, 1), 0, 2, Some(2), 10_000 * MS);
        assert!(!b.is_empty());
    }

    #[test]
    fn a_timer_node_with_two_inputs_is_sampled_from_both_never_carried() {
        // Node 1 takes /a and /b on thread 7 and publishes /c from a timer
        // with stamps of its own: /c is attributed to whichever came last.
        let mut b = ChainBook::default();
        for i in 1..=40u64 {
            let t = i * 100 * MS;
            let input = if i % 4 == 0 { 2 } else { 1 };
            b.take((0, 1), 7, input, Some(i), t);
            b.publish((0, 1), 7, 3, Some(10_000 + i), t + 2 * MS);
        }
        let s = b.snapshot(10_000 * MS, &names(), label);
        assert!(s.edges.is_empty(), "nothing carried: {:?}", s.edges);
        let got: Vec<(&str, &str, u64)> = s
            .sampled
            .iter()
            .map(|e| (e.from.as_str(), e.to.as_str(), e.samples))
            .collect();
        assert_eq!(got, [("/a", "/c", 30)], "/b seen 10 times, under the floor");
        assert_eq!(s.sampled[0].share, 0.75);
        assert_eq!(s.sampled[0].last_ms, 2.0);
    }

    #[test]
    fn a_carried_publish_is_not_also_sampled_and_threads_are_apart() {
        let mut b = ChainBook::default();
        for i in 1..=CHAIN_MIN_SAMPLES {
            let t = i * 100 * MS;
            b.take((0, 1), 7, 1, Some(i), t);
            b.publish((0, 1), 7, 2, Some(i), t + MS);
            // Another thread of the same node publishes /d, unstamped.
            b.publish((0, 1), 8, 4, None, t + MS);
            // Another node on thread 7 of the same process.
            b.publish((0, 2), 7, 4, None, t + MS);
        }
        let s = b.snapshot(10_000 * MS, &names(), label);
        assert_eq!(s.edges.len(), 1);
        assert!(s.sampled.is_empty(), "{:?}", s.sampled);
    }

    #[test]
    fn a_sampled_hop_is_timed_from_the_inputs_latest_take_on_any_thread() {
        let mut b = ChainBook::default();
        for i in 1..=CHAIN_MIN_SAMPLES {
            let t = i * 1000 * MS;
            // Thread 7 took /a long ago; thread 8 took it again just now.
            b.take((0, 1), 7, 1, None, t);
            b.take((0, 1), 8, 1, None, t + 900 * MS);
            b.publish((0, 1), 7, 2, None, t + 901 * MS);
        }
        let s = b.snapshot(100_000 * MS, &names(), label);
        assert_eq!(s.sampled[0].last_ms, 1.0);
    }

    #[test]
    fn an_unstamped_take_still_samples() {
        let mut b = ChainBook::default();
        for i in 1..=CHAIN_MIN_SAMPLES {
            let t = i * 100 * MS;
            b.take((0, 1), 7, 1, None, t);
            b.publish((0, 1), 7, 2, None, t + 3 * MS);
        }
        let s = b.snapshot(10_000 * MS, &names(), label);
        assert_eq!(s.sampled.len(), 1);
        assert_eq!(s.sampled[0].share, 1.0);
    }

    fn age<'a>(s: &'a ChainSnapshot, topic: &str) -> &'a TopicAge {
        s.ages
            .iter()
            .find(|a| a.topic == topic)
            .unwrap_or_else(|| panic!("no age for {topic}: {:?}", s.ages))
    }

    #[test]
    fn age_runs_from_the_first_publish_along_carried_hops() {
        // /a published by node 1, carried by node 2 to /b and node 3 to /c.
        let mut b = ChainBook::default();
        for i in 1..=CHAIN_MIN_SAMPLES {
            let t = i * 100 * MS;
            b.publish((0, 1), 1, 1, Some(i), t);
            b.take((0, 2), 2, 1, Some(i), t + MS);
            b.publish((0, 2), 2, 2, Some(i), t + 3 * MS);
            b.take((0, 3), 3, 2, Some(i), t + 4 * MS);
            b.publish((0, 3), 3, 3, Some(i), t + 7 * MS);
        }
        let s = b.snapshot(10_000 * MS, &names(), label);
        let c = age(&s, "/c");
        assert_eq!(c.max_ms, Some(7.0));
        assert_eq!((c.source.as_str(), c.estimated_share), ("/a", 0.0));
        assert_eq!(age(&s, "/b").max_ms, Some(3.0));
        assert!(
            s.ages.iter().all(|a| a.topic != "/a"),
            "a source has no age"
        );
    }

    #[test]
    fn a_sampled_hop_passes_its_inputs_origin_on_as_estimated() {
        // Node 2 takes /a and publishes /b from a timer, restamped.
        let mut b = ChainBook::default();
        for i in 1..=CHAIN_MIN_SAMPLES {
            let t = i * 100 * MS;
            b.publish((0, 1), 1, 1, Some(i), t);
            b.take((0, 2), 2, 1, Some(i), t + MS);
            b.publish((0, 2), 2, 2, Some(1000 + i), t + 10 * MS);
        }
        let s = b.snapshot(10_000 * MS, &names(), label);
        let a = age(&s, "/b");
        assert_eq!(a.max_ms, Some(10.0));
        assert_eq!((a.source.as_str(), a.estimated_share), ("/a", 1.0));
    }

    #[test]
    fn a_loop_restarts_the_origin_instead_of_growing_it() {
        // Node 1 publishes /a from what it took on /b; node 2 publishes /b
        // from what it took on /a. Every hop sampled, each 5 ms.
        let mut b = ChainBook::default();
        let mut t = 0;
        for i in 1..=200u64 {
            t += 5 * MS;
            b.publish((0, 1), 1, 1, Some(2 * i), t);
            b.take((0, 2), 2, 1, Some(2 * i), t + MS);
            t += 5 * MS;
            b.publish((0, 2), 2, 2, Some(2 * i + 1), t);
            b.take((0, 1), 1, 2, Some(2 * i + 1), t + MS);
        }
        let s = b.snapshot(t + MS, &names(), label);
        for a in &s.ages {
            assert!(a.max_ms.unwrap() <= 10.0, "{a:?}");
        }
    }

    #[test]
    fn a_take_on_parameter_events_is_never_the_attributed_input() {
        let pe = crate::health::fnv("/parameter_events");
        let mut names = names();
        names.insert(pe, "/parameter_events".into());
        let mut b = ChainBook::default();
        b.take((0, 1), 7, 1, Some(1), 0);
        b.take((0, 1), 7, pe, Some(2), 10 * MS);
        for i in 1..=CHAIN_MIN_SAMPLES {
            b.publish((0, 1), 7, 2, None, (100 + i) * MS);
        }
        let s = b.snapshot(10_000 * MS, &names, label);
        assert_eq!(s.sampled.len(), 1);
        assert_eq!(s.sampled[0].from, "/a");
    }

    #[test]
    fn paths_join_carried_and_sampled_hops_fewest_first() {
        // /a -carried-> /b -sampled-> /c, and a longer /a -> /d -> /b too.
        let mut b = ChainBook::default();
        for i in 1..=CHAIN_MIN_SAMPLES {
            let t = i * 100 * MS;
            b.take((0, 1), 1, 1, Some(i), t);
            b.publish((0, 1), 1, 2, Some(i), t + MS);
            b.take((0, 2), 2, 2, Some(i), t + 2 * MS);
            b.publish((0, 2), 2, 3, Some(10_000 + i), t + 6 * MS);
            b.take((0, 3), 3, 1, Some(i), t + 2 * MS);
            b.publish((0, 3), 3, 4, Some(i), t + 3 * MS);
            b.take((0, 4), 4, 4, Some(i), t + 4 * MS);
            b.publish((0, 4), 4, 2, Some(i), t + 5 * MS);
        }
        let s = b.snapshot(10_000 * MS, &names(), label);
        let (paths, truncated) = s.paths("/a", "/c");
        assert!(!truncated);
        assert_eq!(paths.len(), 2, "{paths:?}");
        let kinds: Vec<&str> = paths[0].hops.iter().map(|h| h.kind).collect();
        assert_eq!(kinds, ["carried", "sampled"]);
        assert_eq!((paths[0].carried_hops, paths[0].sampled_hops), (1, 1));
        assert_eq!(paths[0].hops[1].node, "n2");
        assert_eq!(paths[1].hops.len(), 3);
        assert!(s.paths("/c", "/a").0.is_empty());
        assert!(s.paths("/a", "/a").0.is_empty());
    }

    #[test]
    fn a_publish_read_before_its_inputs_publish_waits_for_it() {
        // Node 2's ring drains first: its take of /a and publish of /b are
        // read before node 1's publish of /a that started them.
        let mut b = ChainBook::default();
        for i in 1..=CHAIN_MIN_SAMPLES {
            let t = i * 100 * MS;
            b.take((1, 2), 2, 1, Some(i), t + MS);
            b.publish((1, 2), 2, 2, Some(i), t + 3 * MS);
            b.take((2, 3), 3, 2, Some(i), t + 4 * MS);
            b.publish((2, 3), 3, 3, Some(i), t + 6 * MS);
            b.publish((0, 1), 1, 1, Some(i), t);
            b.settle(t + 10 * MS);
        }
        let s = b.snapshot(10_000 * MS, &names(), label);
        let c = age(&s, "/c");
        assert_eq!((c.source.as_str(), c.max_ms), ("/a", Some(6.0)), "{c:?}");
        assert_eq!(c.estimated_share, 0.0);
    }

    #[test]
    fn a_publish_whose_input_never_appears_falls_back_after_a_second() {
        let mut b = ChainBook::default();
        for i in 1..=CHAIN_MIN_SAMPLES {
            let t = i * 100 * MS;
            b.take((1, 2), 2, 1, Some(i), t);
            b.publish((1, 2), 2, 2, Some(i), t + 3 * MS);
        }
        b.settle(CHAIN_MIN_SAMPLES * 100 * MS + 10 * MS);
        assert!(
            b.snapshot(0, &names(), label).ages.is_empty(),
            "still waiting"
        );
        b.settle(10_000 * MS);
        let s = b.snapshot(10_000 * MS, &names(), label);
        let a = age(&s, "/b");
        assert_eq!((a.source.as_str(), a.max_ms), ("/a", Some(3.0)));
        assert_eq!(a.estimated_share, 1.0);
    }
}
