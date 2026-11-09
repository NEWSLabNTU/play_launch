//! Values and levels at run time (phase 102 M2; rlm v0.3.13).
//!
//! The interceptor sends each declared field of each published message as a
//! `FieldValue` event (`interception::fields`). Here those values do two
//! things:
//!
//! - a value function (`of:` + `when:`) is LOST while its predicate holds on
//!   its latest sample, and found again when it no longer does, so a mode
//!   is lost by value as well as by silence;
//! - a level the system fell to is ACTIVE from the moment it was entered
//!   until the mode above it is available again, and from the level's
//!   derived bound on, every message on a topic it commands must satisfy
//!   its predicates (`safe-state-violated`) and the topic must not go
//!   silent unless its consumer states what it does on silence
//!   (`final-element-silent`).

use std::collections::{HashMap, HashSet, VecDeque};

use ros_launch_manifest_model as model;

/// The FNV-1a hash the interceptor sends for a string field.
fn fnv1a(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for b in bytes {
        h ^= *b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

/// Does `pred` hold on a field value the interceptor encoded as (`kind`,
/// `bits`)? `None` when it cannot be judged: a message constant name (the
/// model carries the name, not its value) or a value of the wrong kind.
pub(super) fn eval(pred: &model::PredicateContract, kind: u8, bits: u64) -> Option<bool> {
    use model::{CompareOp as Op, PredicateValueContract as V};
    let number = || -> Option<f64> {
        match kind {
            1 => Some(f64::from_bits(bits)),
            2 => Some(bits as i64 as f64),
            3 => Some(bits as f64),
            4 => Some(if bits != 0 { 1.0 } else { 0.0 }),
            _ => None,
        }
    };
    let cmp = |a: f64, b: f64| match pred.op {
        Op::Equals => a == b,
        Op::NotEquals => a != b,
        Op::Lt => a < b,
        Op::Le => a <= b,
        Op::Gt => a > b,
        Op::Ge => a >= b,
    };
    match &pred.value {
        V::Bool(b) => {
            let v = (kind == 4).then_some(bits != 0)?;
            match pred.op {
                Op::Equals => Some(v == *b),
                Op::NotEquals => Some(v != *b),
                _ => None,
            }
        }
        V::Int(i) => Some(cmp(number()?, *i as f64)),
        V::Float(f) => Some(cmp(number()?, *f)),
        V::Str(s) => {
            let same = (kind == 5).then(|| bits == fnv1a(s.as_bytes()))?;
            match pred.op {
                Op::Equals => Some(same),
                Op::NotEquals => Some(!same),
                _ => None,
            }
        }
        V::Constant { .. } => None,
    }
}

/// The value as a reader would write it, for a message.
pub(super) fn show(kind: u8, bits: u64) -> String {
    match kind {
        1 => format!("{}", f64::from_bits(bits)),
        2 => format!("{}", bits as i64),
        3 => format!("{bits}"),
        4 => format!("{}", bits != 0),
        5 => format!("a string (fnv1a {bits:016x})"),
        _ => "an unknown value".to_string(),
    }
}

/// The observer's state for values and levels.
#[derive(Debug, Default)]
pub(super) struct Levels {
    /// Value functions currently lost by value.
    pub value_lost: HashSet<String>,
    /// Active levels: mode key to the time it was entered.
    pub active: HashMap<String, u64>,
    /// Findings already reported for the current entry of a level, by
    /// `(level, what)`, so a 50 Hz command stream reports once per entry.
    pub reported: HashSet<(String, String)>,
    /// Commanded topics' last publish, and their recent gaps.
    pub last_pub: HashMap<String, u64>,
    pub gaps: HashMap<String, VecDeque<u64>>,
}

impl Levels {
    /// Note a publish on a topic some level commands.
    pub fn note_publish(&mut self, topic: &str, now: u64) {
        if let Some(prev) = self.last_pub.get(topic) {
            let g = self.gaps.entry(topic.to_string()).or_default();
            g.push_back(now.saturating_sub(*prev));
            if g.len() > 32 {
                g.pop_front();
            }
        }
        self.last_pub.insert(topic.to_string(), now);
    }

    /// How long `topic` may stay silent before a level judges it silent:
    /// ten of its median gaps, at least 50 ms; 1 s before four gaps are
    /// known. The rule the hazard observer uses for a guard.
    pub fn silence_ns(&self, topic: &str) -> u64 {
        match self.gaps.get(topic) {
            Some(g) if g.len() >= 4 => {
                let mut v: Vec<u64> = g.iter().copied().collect();
                v.sort_unstable();
                (v[v.len() / 2] * 10).max(50_000_000)
            }
            _ => 1_000_000_000,
        }
    }

    /// A level was entered at `now` (once; a second entry keeps the first).
    pub fn enter(&mut self, level: &str, now: u64) {
        self.active.entry(level.to_string()).or_insert(now);
    }

    /// A level was left: forget its entry and what it reported.
    pub fn leave(&mut self, level: &str) {
        self.active.remove(level);
        self.reported.retain(|(l, _)| l != level);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use model::{CompareOp, PredicateContract, PredicateValueContract as V};

    fn p(op: CompareOp, value: V) -> PredicateContract {
        PredicateContract {
            field: "x".into(),
            op,
            value,
        }
    }

    #[test]
    fn predicates_judge_each_encoding() {
        let zero = p(CompareOp::Equals, V::Int(0));
        assert_eq!(eval(&zero, 1, 0.0f64.to_bits()), Some(true));
        assert_eq!(eval(&zero, 1, 0.3f64.to_bits()), Some(false));
        let le = p(CompareOp::Le, V::Float(0.3));
        assert_eq!(eval(&le, 1, 0.3f64.to_bits()), Some(true));
        assert_eq!(eval(&le, 2, (-4i64) as u64), Some(true));
        let flag = p(CompareOp::Equals, V::Bool(true));
        assert_eq!(eval(&flag, 4, 1), Some(true));
        assert_eq!(eval(&flag, 1, 1), None, "a bool predicate on a number");
        let s = p(CompareOp::NotEquals, V::Str("MANUAL".into()));
        assert_eq!(eval(&s, 5, fnv1a(b"MANUAL")), Some(false));
        assert_eq!(eval(&s, 5, fnv1a(b"AUTO")), Some(true));
        let c = p(
            CompareOp::Equals,
            V::Constant {
                constant: "MANUAL".into(),
            },
        );
        assert_eq!(
            eval(&c, 3, 1),
            None,
            "a constant's value is not in the model"
        );
    }

    #[test]
    fn a_level_reports_once_per_entry() {
        let mut l = Levels::default();
        l.enter("stopped", 10);
        l.enter("stopped", 20);
        assert_eq!(l.active["stopped"], 10);
        assert!(l.reported.insert(("stopped".into(), "/cmd".into())));
        assert!(!l.reported.insert(("stopped".into(), "/cmd".into())));
        l.leave("stopped");
        assert!(l.reported.insert(("stopped".into(), "/cmd".into())));
        for t in [0, 100, 200, 300, 400] {
            l.note_publish("/cmd", t * 1_000_000);
        }
        assert_eq!(l.silence_ns("/cmd"), 1_000_000_000);
    }
}
