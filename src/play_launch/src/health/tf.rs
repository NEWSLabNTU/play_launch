//! TF health (big-system-observability §H), from the edges the interceptor
//! reads on publish (`interception::TfSnapshot`). A TF tree breaks in a few
//! ways that look like a perception or planning bug from downstream:
//!
//! - `tf_two_parents` (error): a frame published under two parents. tf2
//!   keeps the last one it heard, so the frame jumps between subtrees.
//! - `tf_two_publishers` (warning): one edge from two nodes. The values
//!   interleave; usually two launches of the same `robot_state_publisher`.
//! - `tf_disconnected` (warning): more than one root, so a lookup between
//!   frames under different roots fails ("not part of the same tree"); most
//!   often a missing static transform.
//! - `tf_stale` (warning): a dynamic edge that stopped. Every lookup through
//!   it starts failing with extrapolation errors as its last stamp ages.
//! - `tf_static_at_rate` (info): a static edge republished continually.
//!   `/tf_static` is latched; sending it at rate costs every subscriber.
//! - `tf_never_moves` (info): a dynamic edge whose value never changed —
//!   it could be static, and costs a `/tf` message every time it is sent.

use std::collections::{BTreeMap, BTreeSet};

use super::{Condition, Severity, Subject};
use crate::interception::TfSnapshot;

/// A dynamic edge is stale after this long, or five of its own intervals.
const STALE_AFTER_S: f64 = 2.0;
/// A static edge "at rate": this many messages, at this rate or faster.
const STATIC_AT_RATE_MESSAGES: u64 = 20;
const STATIC_AT_RATE_HZ: f64 = 0.5;
/// A dynamic edge "never moves": this many messages over this long.
const NEVER_MOVES_MESSAGES: u64 = 50;
const NEVER_MOVES_S: f64 = 30.0;

fn subject(name: String) -> Subject {
    Subject {
        kind: "frame",
        name,
    }
}

fn who(p: &Option<String>) -> &str {
    p.as_deref().unwrap_or("?")
}

pub fn tf_conditions(tf: &TfSnapshot) -> Vec<Condition> {
    let mut out = Vec::new();

    // Per child frame: its parents and, per edge, its publishers.
    let mut parents: BTreeMap<&str, BTreeSet<&str>> = BTreeMap::new();
    let mut publishers: BTreeMap<(&str, &str), BTreeSet<&str>> = BTreeMap::new();
    for e in &tf.edges {
        parents.entry(&e.child).or_default().insert(&e.parent);
        publishers
            .entry((&e.parent, &e.child))
            .or_default()
            .insert(who(&e.publisher));
    }
    for (child, ps) in &parents {
        if ps.len() > 1 {
            out.push(Condition {
                key: format!("tf_two_parents:{child}"),
                kind: "tf_two_parents",
                severity: Severity::Error,
                subject: subject(child.to_string()),
                title: format!("published under {} parents", ps.len()),
                detail: format!(
                    "parents: {}. tf2 keeps whichever it heard last, so the frame jumps \
                     between subtrees",
                    ps.iter().copied().collect::<Vec<_>>().join(", ")
                ),
                link: None,
            });
        }
    }
    for ((parent, child), nodes) in &publishers {
        if nodes.len() > 1 {
            out.push(Condition {
                key: format!("tf_two_publishers:{parent}:{child}"),
                kind: "tf_two_publishers",
                severity: Severity::Warning,
                subject: subject(format!("{parent} → {child}")),
                title: format!("published by {} nodes", nodes.len()),
                detail: format!(
                    "{}: their values interleave",
                    nodes.iter().copied().collect::<Vec<_>>().join(", ")
                ),
                link: None,
            });
        }
    }
    let roots = tf.roots();
    if roots.len() > 1 {
        out.push(Condition {
            key: "tf_disconnected".to_string(),
            kind: "tf_disconnected",
            severity: Severity::Warning,
            subject: subject(roots.join(", ")),
            title: format!("the TF tree has {} roots", roots.len()),
            detail: format!(
                "roots: {}. A lookup between frames under different roots fails; usually \
                 a static transform nobody publishes",
                roots.join(", ")
            ),
            link: None,
        });
    }
    for e in &tf.edges {
        let edge = format!("{} → {}", e.parent, e.child);
        let by = who(&e.publisher);
        if !e.is_static && e.messages >= 5 {
            let interval = if e.rate_hz > 0.0 {
                1.0 / e.rate_hz
            } else {
                0.0
            };
            if e.since_last_s > STALE_AFTER_S.max(5.0 * interval) {
                out.push(Condition {
                    key: format!("tf_stale:{}:{}:{by}", e.parent, e.child),
                    kind: "tf_stale",
                    severity: Severity::Warning,
                    subject: subject(edge.clone()),
                    title: format!("not published for {:.0} s", e.since_last_s),
                    detail: format!(
                        "{by} sent it at {:.1} Hz; lookups through it now fail with \
                         extrapolation errors",
                        e.rate_hz
                    ),
                    link: None,
                });
            }
            if e.changes == 0 && e.messages >= NEVER_MOVES_MESSAGES && e.active_s >= NEVER_MOVES_S {
                out.push(Condition {
                    key: format!("tf_never_moves:{}:{}:{by}", e.parent, e.child),
                    kind: "tf_never_moves",
                    severity: Severity::Info,
                    subject: subject(edge.clone()),
                    title: format!("sent {} times on /tf, never changed", e.messages),
                    detail: format!(
                        "{by} publishes a constant transform at {:.0} Hz: on /tf_static it \
                         would be sent once",
                        e.rate_hz
                    ),
                    link: None,
                });
            }
        }
        if e.is_static && e.messages >= STATIC_AT_RATE_MESSAGES && e.rate_hz >= STATIC_AT_RATE_HZ {
            out.push(Condition {
                key: format!("tf_static_at_rate:{}:{}:{by}", e.parent, e.child),
                kind: "tf_static_at_rate",
                severity: Severity::Info,
                subject: subject(edge),
                title: format!("static, but sent at {:.1} Hz", e.rate_hz),
                detail: format!(
                    "{by} republishes it on /tf_static, which is latched: once is enough, \
                     and every subscriber processes each copy"
                ),
                link: None,
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interception::TfEdgeView;

    fn edge(parent: &str, child: &str, by: &str) -> TfEdgeView {
        TfEdgeView {
            parent: parent.into(),
            child: child.into(),
            publisher: Some(by.into()),
            is_static: false,
            messages: 100,
            rate_hz: 10.0,
            active_s: 10.0,
            since_last_s: 0.05,
            stamp_s: 0.0,
            changes: 99,
            since_change_s: Some(0.05),
        }
    }

    fn kinds(edges: Vec<TfEdgeView>) -> Vec<&'static str> {
        let mut k: Vec<&str> = tf_conditions(&TfSnapshot { edges })
            .iter()
            .map(|c| c.kind)
            .collect();
        k.sort();
        k
    }

    #[test]
    fn a_healthy_tree_reports_nothing() {
        let k = kinds(vec![
            edge("map", "odom", "/loc"),
            edge("odom", "base_link", "/odom"),
            TfEdgeView {
                is_static: true,
                messages: 1,
                rate_hz: 0.0,
                changes: 0,
                ..edge("base_link", "lidar", "/rsp")
            },
        ]);
        assert!(k.is_empty(), "{k:?}");
    }

    #[test]
    fn two_parents_two_publishers_and_two_roots() {
        let k = kinds(vec![
            edge("map", "base_link", "/a"),
            edge("odom", "base_link", "/b"),
            edge("base_link", "lidar", "/c"),
            edge("base_link", "lidar", "/d"),
        ]);
        assert_eq!(
            k,
            ["tf_disconnected", "tf_two_parents", "tf_two_publishers"],
            "map and odom are both roots"
        );
    }

    #[test]
    fn stale_constant_and_static_at_rate() {
        let stale = TfEdgeView {
            since_last_s: 10.0,
            ..edge("map", "base_link", "/a")
        };
        let constant = TfEdgeView {
            changes: 0,
            active_s: 60.0,
            messages: 600,
            ..edge("base_link", "imu", "/b")
        };
        let static_rate = TfEdgeView {
            is_static: true,
            changes: 0,
            ..edge("base_link", "camera", "/c")
        };
        assert_eq!(
            kinds(vec![stale, constant, static_rate]),
            ["tf_never_moves", "tf_stale", "tf_static_at_rate"]
        );
    }
}
