//! Chains on the Health page (`docs/design/data-flow-chains.md`, phase 88
//! C1): from the carried edges the interception consumer finds
//! (`interception::ChainSnapshot`).
//!
//! - `stale_stamp` (info): a node keeps publishing a stamp it took long ago
//!   — more than ten of the output's own periods, and at least a second —
//!   because the input was taken once and the output repeats it. Right for a
//!   state stamped when it changed; wrong for a command or measurement a
//!   consumer judges by its stamp's age, which discards it or acts on old
//!   data. Info, because only the author knows which: a driven Autoware has
//!   three, all of this shape — `vehicle_cmd_gate` re-sending
//!   `/system/emergency/gear_cmd`'s stamp on `/control/command/gear_cmd`
//!   (25.9 s in one run), its operation mode, and the emergency stop
//!   operator re-sending a control command's stamp 47.7 s old.

use super::{Condition, Link, Severity, Subject};
use crate::interception::ChainSnapshot;

/// A stamp is stale past this many periods of the output, or this floor.
const STALE_PERIODS: f64 = 10.0;
const STALE_FLOOR_MS: f64 = 1000.0;
/// Observations in the last 10 to 20 s before an edge is judged.
const STALE_MIN_RECENT: u64 = 5;

pub fn chain_conditions(snap: &ChainSnapshot) -> Vec<Condition> {
    let mut out = Vec::new();
    for e in &snap.edges {
        if e.recent_samples < STALE_MIN_RECENT {
            continue;
        }
        let period_ms = if e.rate_hz > 0.0 {
            1000.0 / e.rate_hz
        } else {
            0.0
        };
        let limit = STALE_FLOOR_MS.max(STALE_PERIODS * period_ms);
        let Some(p50) = e.recent_p50_ms else { continue };
        if p50 <= limit || e.last_ms <= limit {
            continue;
        }
        out.push(Condition {
            key: format!("stale_stamp:{}:{}:{}", e.node, e.from, e.to),
            kind: "stale_stamp",
            severity: Severity::Info,
            subject: Subject {
                kind: "topic",
                name: e.to.clone(),
            },
            title: format!("re-sends a stamp taken {:.1} s earlier", e.last_ms / 1000.0),
            detail: format!(
                "{} publishes {} at {:.1} Hz with the stamp of a message it took on {} \
                 {:.1} s before, and has not taken that stamp since; downstream it looks \
                 that old. Right for a state stamped when it changed, wrong for data \
                 judged by its age",
                e.node,
                e.to,
                e.rate_hz,
                e.from,
                e.last_ms / 1000.0
            ),
            link: Some(Link {
                view: "entities",
                target: Some(e.to.clone()),
            }),
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interception::CarriedEdge;

    fn edge(rate_hz: f64, recent_p50_ms: f64, last_ms: f64) -> CarriedEdge {
        CarriedEdge {
            from: "/system/emergency/gear_cmd".into(),
            node: "/control/vehicle_cmd_gate".into(),
            to: "/control/command/gear_cmd".into(),
            samples: 200,
            rate_hz,
            p50_ms: Some(recent_p50_ms),
            p99_ms: Some(recent_p50_ms),
            max_ms: Some(recent_p50_ms),
            recent_samples: 100,
            recent_p50_ms: Some(recent_p50_ms),
            last_ms,
            since_last_s: 0.1,
        }
    }

    #[test]
    fn an_old_stamp_re_sent_at_rate_is_stale() {
        let snap = ChainSnapshot {
            edges: vec![edge(10.0, 25_900.0, 25_950.0)],
            chains: vec![],
        };
        let c = chain_conditions(&snap);
        assert_eq!(c.len(), 1);
        assert_eq!(c[0].kind, "stale_stamp");
        assert_eq!(c[0].subject.name, "/control/command/gear_cmd");
    }

    #[test]
    fn a_slow_hop_on_a_slow_topic_is_not() {
        // 0.2 Hz output: ten periods is 50 s.
        let snap = ChainSnapshot {
            edges: vec![edge(0.2, 3_000.0, 3_000.0), edge(10.0, 40.0, 40.0)],
            chains: vec![],
        };
        assert!(chain_conditions(&snap).is_empty());
    }
}
