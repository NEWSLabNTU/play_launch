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

/// `(ring, node hash)`; hash 0 when the endpoint's owner was not announced.
pub(crate) type NodeKey = (usize, u64);

#[derive(Debug, Default)]
struct Taken {
    /// stamp -> (topic, take time).
    at: HashMap<u64, (u64, u64)>,
    order: VecDeque<u64>,
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
}

impl ChainBook {
    pub(crate) fn take(&mut self, node: NodeKey, topic: u64, stamp: u64, at_ns: u64) {
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

    pub(crate) fn publish(&mut self, node: NodeKey, topic: u64, stamp: u64, at_ns: u64) {
        let Some(&(input, take_ns)) = self.taken.get(&node).and_then(|t| t.at.get(&stamp)) else {
            return;
        };
        if input == topic || take_ns > at_ns {
            return;
        }
        let e = self.edges.entry((input, node, topic)).or_default();
        if e.first_ns == 0 {
            e.first_ns = at_ns;
        }
        e.last_ns = at_ns;
        e.last_latency_ns = at_ns - take_ns;
        e.latency.add(at_ns, at_ns - take_ns);
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.edges.is_empty()
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
        ChainSnapshot { edges, chains }
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
    pub edges: Vec<CarriedEdge>,
    pub chains: Vec<Chain>,
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
        b.take((0, 1), 1, 7, 100 * MS);
        b.publish((0, 1), 2, 7, 103 * MS);
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
        b.take((0, 1), 1, 7, 100 * MS);
        b.publish((0, 2), 2, 7, 103 * MS);
        // Same node, same topic in and out: a relay, not a hop between topics.
        b.take((1, 3), 3, 8, 100 * MS);
        b.publish((1, 3), 3, 8, 101 * MS);
        assert!(b.is_empty());
    }

    #[test]
    fn infrastructure_topics_are_not_edges() {
        let mut b = ChainBook::default();
        b.take((0, 1), 1, 7, 100 * MS);
        b.publish((0, 1), 9, 7, 101 * MS);
        assert!(!b.is_empty());
        assert!(b.snapshot(110 * MS, &names(), label).edges.is_empty());
    }

    #[test]
    fn well_observed_edges_join_into_the_longest_chain() {
        let mut b = ChainBook::default();
        for i in 1..=CHAIN_MIN_SAMPLES {
            let t = i * 100 * MS;
            b.take((0, 1), 1, i, t);
            b.publish((0, 1), 2, i, t + MS);
            b.take((0, 2), 2, i, t + 2 * MS);
            b.publish((0, 2), 3, i, t + 5 * MS);
            b.take((0, 3), 3, i, t + 6 * MS);
            b.publish((0, 3), 4, i, t + 7 * MS);
        }
        // A side branch seen too rarely to count.
        b.take((0, 4), 2, 999, 50 * MS);
        b.publish((0, 4), 9, 999, 51 * MS);
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
            b.take((0, 1), 1, s, s * MS);
        }
        b.publish((0, 1), 2, 1, 10_000 * MS);
        assert!(b.is_empty(), "stamp 1 was forgotten");
        b.publish((0, 1), 2, 2, 10_000 * MS);
        assert!(!b.is_empty());
    }
}
