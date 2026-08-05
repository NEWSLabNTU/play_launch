//! Chrome Trace Event export for interception events (phase 57 W1).
//!
//! The interception layer already timestamps every `rcl_publish`/`rcl_take`
//! with `CLOCK_MONOTONIC`; until now that only ever became summary JSON.
//! `metrics.csv` samples every 2 s, which is three orders of magnitude too
//! coarse to see scheduling, so there was no way to look at *when* things
//! actually ran.
//!
//! Output loads directly in `chrome://tracing` (and Perfetto's legacy-JSON
//! importer). Format reference: Chrome Trace Event Format, the "JSON Array"
//! variant — a bare array needs no trailing `]`, so a trace truncated by a
//! crash still opens.
//!
//! What this can and cannot show:
//!
//! - CAN: which node published or took a message, when, and how long a
//!   message took to travel a chain end to end.
//! - CANNOT: CPU occupancy or preemption. Interception sees messages, not
//!   thread run-time. A gap in a row means "no message", not "descheduled" —
//!   worth remembering before reading a stall into one.
//!
//! Flow arrows are keyed on the message header stamp, which is the only
//! identity shared by a publish and the take that receives it. Messages
//! without a `std_msgs/Header` carry stamp 0 and cannot be linked; they still
//! appear as instants, just unconnected.

use std::{collections::HashMap, io::Write, path::Path};

use super::{EventKind, InterceptionEvent};

/// One row per node, keyed by the ring's owning node name.
///
/// Chrome sorts rows by pid/tid, so a stable assignment keeps a trace
/// comparable across runs — which matters here, because the whole point is
/// diffing an RT-on run against an RT-off one.
#[derive(Default)]
pub struct TraceRecorder {
    events: Vec<serde_json::Value>,
    pids: HashMap<String, u32>,
    next_pid: u32,
    /// Stamp -> the flow id already issued for it, so a publish and every
    /// take of the same message share one arrow.
    flows: HashMap<u64, u64>,
    next_flow: u64,
    dropped_unstamped: u64,
}

impl TraceRecorder {
    pub fn new() -> Self {
        Self {
            next_pid: 1,
            next_flow: 1,
            ..Default::default()
        }
    }

    fn pid_for(&mut self, node: &str) -> u32 {
        if let Some(p) = self.pids.get(node) {
            return *p;
        }
        let p = self.next_pid;
        self.next_pid += 1;
        self.pids.insert(node.to_string(), p);
        // Name the row so Chrome shows the node, not a bare number.
        self.events.push(serde_json::json!({
            "name": "process_name",
            "ph": "M",
            "pid": p,
            "tid": 0,
            "args": { "name": node },
        }));
        p
    }

    /// Stable per-message identity: the header stamp. Zero means the message
    /// had no header, so it cannot be correlated.
    fn stamp_key(event: &InterceptionEvent) -> Option<u64> {
        let key = ((event.stamp_sec as u64) << 32) | event.stamp_nanosec as u64;
        (event.stamp_sec != 0 || event.stamp_nanosec != 0).then_some(key)
    }

    /// Record a publish or take. Other event kinds (init, name declarations,
    /// overflow reports) carry no timing and are skipped.
    pub fn observe(&mut self, node: &str, event: &InterceptionEvent, topic: Option<&str>) {
        let (phase_name, is_publish) = match event.kind {
            EventKind::Publish => ("publish", true),
            EventKind::Take => ("take", false),
            _ => return,
        };

        let pid = self.pid_for(node);
        // Chrome timestamps are microseconds; the ring gives nanoseconds.
        let ts = event.monotonic_ns / 1_000;
        let topic_name = topic
            .map(str::to_string)
            .unwrap_or_else(|| format!("topic#{:x}", event.topic_hash));

        self.events.push(serde_json::json!({
            "name": format!("{phase_name} {topic_name}"),
            "cat": phase_name,
            "ph": "i",
            "s": "t",
            "ts": ts,
            "pid": pid,
            "tid": 1,
        }));

        match Self::stamp_key(event) {
            Some(key) => {
                // `s` opens a flow at the publish, `f` closes it at each take.
                // Chrome draws the arrow between them, which is what makes a
                // multi-hop chain legible at a glance.
                let (id, ph) = if is_publish {
                    let id = self.next_flow;
                    self.next_flow += 1;
                    self.flows.insert(key, id);
                    (id, "s")
                } else {
                    match self.flows.get(&key) {
                        Some(id) => (*id, "f"),
                        // A take whose publish we never saw — the publisher
                        // started before tracing, or its ring overflowed.
                        None => return,
                    }
                };
                self.events.push(serde_json::json!({
                    "name": topic_name,
                    "cat": "msg",
                    "ph": ph,
                    "bp": "e",
                    "id": id,
                    "ts": ts,
                    "pid": pid,
                    "tid": 1,
                }));
            }
            None => self.dropped_unstamped += 1,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    /// Number of publish/take events that could not be linked because the
    /// message had no header stamp. Reported so an all-unlinked trace is
    /// visibly explained rather than looking like an empty result.
    pub fn unstamped(&self) -> u64 {
        self.dropped_unstamped
    }

    pub fn write(&self, path: &Path) -> eyre::Result<()> {
        let mut f = std::fs::File::create(path)?;
        // JSON Array form on purpose: no trailing bracket is required, so a
        // trace cut short by a crash still loads.
        writeln!(f, "[")?;
        for (i, ev) in self.events.iter().enumerate() {
            let comma = if i + 1 == self.events.len() { "" } else { "," };
            writeln!(f, "  {}{}", serde_json::to_string(ev)?, comma)?;
        }
        writeln!(f, "]")?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ev(kind: EventKind, hash: u64, sec: i32, nsec: u32, mono: u64) -> InterceptionEvent {
        InterceptionEvent {
            kind,
            _pad: [0; 3],
            topic_hash: hash,
            stamp_sec: sec,
            stamp_nanosec: nsec,
            handle: 0,
            monotonic_ns: mono,
        }
    }

    /// A publish and the take of the SAME message must share one flow id, or
    /// Chrome draws no arrow and the chain is invisible.
    #[test]
    fn publish_and_take_of_one_message_share_a_flow() {
        let mut t = TraceRecorder::new();
        t.observe(
            "/a",
            &ev(EventKind::Publish, 7, 1, 500, 1_000_000),
            Some("/scan"),
        );
        t.observe(
            "/b",
            &ev(EventKind::Take, 7, 1, 500, 2_000_000),
            Some("/scan"),
        );

        let flows: Vec<_> = t
            .events
            .iter()
            .filter(|e| e["cat"] == "msg")
            .map(|e| {
                (
                    e["ph"].as_str().unwrap().to_string(),
                    e["id"].as_u64().unwrap(),
                )
            })
            .collect();
        assert_eq!(flows.len(), 2, "one open and one close: {flows:?}");
        assert_eq!(flows[0].0, "s");
        assert_eq!(flows[1].0, "f");
        assert_eq!(flows[0].1, flows[1].1, "same flow id both ends");
    }

    /// Two nodes get two rows; a row is named after the node so a reader sees
    /// `/safety/brake_controller`, not `pid 2`.
    #[test]
    fn each_node_gets_a_named_row() {
        let mut t = TraceRecorder::new();
        t.observe("/a", &ev(EventKind::Publish, 1, 1, 0, 10), Some("/x"));
        t.observe("/b", &ev(EventKind::Publish, 2, 2, 0, 20), Some("/y"));

        let names: Vec<&str> = t
            .events
            .iter()
            .filter(|e| e["ph"] == "M")
            .map(|e| e["args"]["name"].as_str().unwrap())
            .collect();
        assert_eq!(names, vec!["/a", "/b"]);
    }

    /// Unstamped messages cannot be correlated. They must still appear, and
    /// the count must be reported — an all-unlinked trace should explain
    /// itself rather than look empty.
    #[test]
    fn unstamped_messages_are_counted_not_silently_dropped() {
        let mut t = TraceRecorder::new();
        t.observe("/a", &ev(EventKind::Publish, 1, 0, 0, 10), Some("/x"));

        assert_eq!(t.unstamped(), 1);
        assert!(
            t.events.iter().any(|e| e["ph"] == "i"),
            "the event itself must still be visible"
        );
        assert!(
            !t.events.iter().any(|e| e["cat"] == "msg"),
            "but with no flow arrow"
        );
    }
}
