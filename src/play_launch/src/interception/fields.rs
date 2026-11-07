//! Declared message fields read at run time (phase 102 M0).
//!
//! A contract's predicates (`hazards.<h>.when`, `functions.<f>.when`, and
//! phase 100's `modes.<m>.commands`) name fields of messages. The
//! interceptor reads exactly those, by introspection, and sends each value
//! as a `FieldValue` event (`play_launch_interception::fields`). This module
//! is the consumer's half: which fields the model asks for, the environment
//! value that tells each child, and a per-field record of what was seen,
//! written to `interception/fields.json` at shutdown.
//!
//! A field's index is its position in the list handed to every child, so
//! both sides number them from the one list built here.

use std::{
    collections::{BTreeSet, HashMap},
    path::Path,
};

use ros_launch_manifest_model::SystemModel;
use serde::Serialize;

use super::InterceptionEvent;

/// The environment variable the interceptor reads the list from.
pub const ENV: &str = "PLAY_LAUNCH_INTERCEPTION_FIELDS";

/// The interceptor's limit: an index is one byte.
const MAX_FIELDS: usize = 256;

/// One field the model asks the run to read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeclaredField {
    pub topic: String,
    /// Dotted path within the message (`data`, `status.mode`).
    pub path: String,
    /// The contract keys that read it (`hazards.collision.when`).
    pub sources: Vec<String>,
}

/// Every field a predicate in the model reads, deduplicated by (topic,
/// path), in a stable order.
pub fn declared_fields(model: &SystemModel) -> Vec<DeclaredField> {
    let mut by_key: std::collections::BTreeMap<(String, String), BTreeSet<String>> =
        Default::default();
    let contracts = &model.contracts;
    for (name, h) in &contracts.hazards {
        let Some(pred) = &h.when else { continue };
        for m in &h.monitors {
            for topic in &m.members {
                by_key
                    .entry((topic.clone(), pred.field.clone()))
                    .or_default()
                    .insert(format!("hazards.{name}.when"));
            }
        }
    }
    for (name, f) in &contracts.functions {
        let Some(pred) = &f.when else { continue };
        for topic in &f.members {
            by_key
                .entry((topic.clone(), pred.field.clone()))
                .or_default()
                .insert(format!("functions.{name}.when"));
        }
    }
    by_key
        .into_iter()
        .take(MAX_FIELDS)
        .map(|((topic, path), sources)| DeclaredField {
            topic,
            path,
            sources: sources.into_iter().collect(),
        })
        .collect()
}

/// The list as the interceptor reads it: one `topic<TAB>path` per line.
pub fn env_value(fields: &[DeclaredField]) -> String {
    fields
        .iter()
        .map(|f| format!("{}\t{}", f.topic, f.path))
        .collect::<Vec<_>>()
        .join("\n")
}

/// A value as the interceptor encoded it
/// (`play_launch_interception::fields::ValueKind`).
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(untagged)]
pub enum FieldValue {
    Float(f64),
    Int(i64),
    Uint(u64),
    Bool(bool),
    /// FNV-1a of a string's bytes; compare with the hash of the constant.
    StrHash(String),
}

impl FieldValue {
    /// Decode an event's (kind byte, value bits), or `None` for a kind this
    /// build does not know.
    pub fn decode(kind: u8, bits: u64) -> Option<Self> {
        Some(match kind {
            1 => Self::Float(f64::from_bits(bits)),
            2 => Self::Int(bits as i64),
            3 => Self::Uint(bits),
            4 => Self::Bool(bits != 0),
            5 => Self::StrHash(format!("fnv1a:{bits:016x}")),
            _ => return None,
        })
    }
}

#[derive(Debug, Default)]
struct Seen {
    messages: u64,
    changes: u64,
    last: Option<(u8, u64)>,
    publishers: BTreeSet<(usize, u64)>,
}

/// What the run saw of each declared field.
#[derive(Debug, Default)]
pub struct FieldBook {
    seen: HashMap<u8, Seen>,
}

impl FieldBook {
    /// Fold one `FieldValue` event from child `child`.
    pub fn observe(&mut self, child: usize, event: &InterceptionEvent) {
        let index = event._pad[0];
        let value = (event._pad[1], event.cpu_ns);
        let s = self.seen.entry(index).or_default();
        s.messages += 1;
        if s.last.is_some_and(|last| last != value) {
            s.changes += 1;
        }
        s.last = Some(value);
        s.publishers.insert((child, event.handle));
    }

    pub fn is_empty(&self) -> bool {
        self.seen.is_empty()
    }

    /// One row per declared field, seen or not: a declared field nothing
    /// published is as much a finding as a value.
    pub fn rows(&self, declared: &[DeclaredField]) -> Vec<FieldRow> {
        declared
            .iter()
            .enumerate()
            .map(|(i, f)| {
                let s = self.seen.get(&(i as u8));
                FieldRow {
                    topic: f.topic.clone(),
                    field: f.path.clone(),
                    sources: f.sources.clone(),
                    messages: s.map_or(0, |s| s.messages),
                    changes: s.map_or(0, |s| s.changes),
                    publishers: s.map_or(0, |s| s.publishers.len()),
                    last: s
                        .and_then(|s| s.last)
                        .and_then(|(k, b)| FieldValue::decode(k, b)),
                }
            })
            .collect()
    }
}

/// One row of `fields.json`.
#[derive(Debug, Serialize)]
pub struct FieldRow {
    pub topic: String,
    pub field: String,
    pub sources: Vec<String>,
    pub messages: u64,
    /// How many messages carried a value other than the one before.
    pub changes: u64,
    pub publishers: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last: Option<FieldValue>,
}

/// Write `fields.json` into the run's `interception/` directory.
pub fn write(interception_dir: &Path, rows: &[FieldRow]) -> eyre::Result<()> {
    let path = interception_dir.join("fields.json");
    std::fs::write(&path, serde_json::to_string_pretty(rows)?)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interception::EventKind;

    fn event(index: u8, kind: u8, bits: u64, handle: u64) -> InterceptionEvent {
        InterceptionEvent {
            kind: EventKind::FieldValue,
            _pad: [index, kind, 0],
            topic_hash: 1,
            stamp_sec: 0,
            stamp_nanosec: 0,
            handle,
            monotonic_ns: 0,
            cpu_ns: bits,
            tid: 0,
            payload_bytes: 0,
        }
    }

    #[test]
    fn counts_messages_changes_and_publishers() {
        let declared = vec![
            DeclaredField {
                topic: "/too_close".into(),
                path: "data".into(),
                sources: vec!["hazards.collision.when".into()],
            },
            DeclaredField {
                topic: "/mode".into(),
                path: "mode".into(),
                sources: vec![],
            },
        ];
        let mut book = FieldBook::default();
        for (bits, handle) in [(0, 7), (0, 7), (1, 7), (1, 8), (0, 7)] {
            book.observe(0, &event(0, 4, bits, handle));
        }
        let rows = book.rows(&declared);
        assert_eq!(rows[0].messages, 5);
        assert_eq!(rows[0].changes, 2);
        assert_eq!(rows[0].publishers, 2);
        assert_eq!(rows[0].last, Some(FieldValue::Bool(false)));
        assert_eq!(rows[1].messages, 0, "a declared field nobody sent is a row");
        assert_eq!(env_value(&declared), "/too_close\tdata\n/mode\tmode");
    }

    #[test]
    fn decodes_each_kind() {
        assert_eq!(
            FieldValue::decode(1, 2.5f64.to_bits()),
            Some(FieldValue::Float(2.5))
        );
        assert_eq!(
            FieldValue::decode(2, (-4i64) as u64),
            Some(FieldValue::Int(-4))
        );
        assert_eq!(FieldValue::decode(9, 0), None);
    }
}
