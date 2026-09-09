//! Report the topic endpoints a process actually creates.
//!
//! The model's topic graph is INFERRED — from contracts an author wrote, and
//! (phase 76) from the `~/input/` / `~/output/` remap convention. This is the
//! measurement that can confirm or contradict it: `rcl_publisher_init` and
//! `rcl_subscription_init` name the node, the topic and the direction, as
//! fact.
//!
//! Why not read it off `events.jsonl`: that file records MESSAGES. A
//! subscription that never receives one, on a pipeline whose sensor is absent,
//! is indistinguishable there from a subscription that does not exist — so
//! grading a graph by traffic grades the run's coverage, not the graph. An
//! endpoint exists the moment it is created, whether or not anything ever
//! flows through it.
//!
//! Written in the init hooks, which already resolve the node's name and expand
//! the topic to the canonical FQN (remap rules applied), so this costs one
//! hash-set lookup per init call and nothing on the hot path.
//!
//! Same file discipline as [`crate::node_identity`]: `O_APPEND`, one
//! `write(2)` per line, every child of a launch appending to one file.
//! Inert unless `PLAY_LAUNCH_INTERCEPTION_ENDPOINT_FILE` is set.
//!
//! Format, tab-separated:
//!
//! ```text
//! <model key>\t<pid>\t<node FQN>\t<pub|sub>\t<topic FQN>
//! ```
//!
//! The model key travels alongside the real FQN for the reason issue #0017
//! gives: for a node the launch file did not name they are different strings,
//! and the model can only be joined on the first.

use std::{
    collections::HashSet,
    io::Write,
    sync::{Mutex, OnceLock},
};

struct Sink {
    file: Mutex<std::fs::File>,
    member: String,
    /// `(direction, topic)` pairs already written by this process. A container
    /// in `stock` or `observable` mode hosts many nodes in one process, and a
    /// node may create the same endpoint more than once over its life.
    seen: Mutex<HashSet<String>>,
}

static SINK: OnceLock<Option<Sink>> = OnceLock::new();

fn sink() -> Option<&'static Sink> {
    SINK.get_or_init(|| {
        let path = std::env::var("PLAY_LAUNCH_INTERCEPTION_ENDPOINT_FILE").ok()?;
        let member = std::env::var("PLAY_LAUNCH_INTERCEPTION_MEMBER").unwrap_or_default();
        let file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&path)
            .ok()?;
        Some(Sink {
            file: Mutex::new(file),
            member,
            seen: Mutex::new(HashSet::new()),
        })
    })
    .as_ref()
}

/// Record that this process created `direction` endpoint on `topic`, from the
/// node at `node_fqn`.
///
/// Cheap and idempotent: after the first call for a given
/// `(node, direction, topic)` it is a hash-set lookup.
pub fn observe(node_fqn: &str, direction: &str, topic: &str) {
    let Some(sink) = sink() else {
        return;
    };
    if node_fqn.is_empty() || topic.is_empty() {
        return;
    }

    let key = format!("{node_fqn}\t{direction}\t{topic}");
    {
        let mut seen = match sink.seen.lock() {
            Ok(g) => g,
            Err(p) => p.into_inner(),
        };
        if !seen.insert(key.clone()) {
            return;
        }
    }

    let line = format!("{}\t{}\t{}\n", sink.member, std::process::id(), key);
    let mut file = match sink.file.lock() {
        Ok(g) => g,
        Err(p) => p.into_inner(),
    };
    // Best-effort by design, like the identity sink: this is a diagnostic aid
    // and a failed write must never disturb the node it is describing.
    let _ = file.write_all(line.as_bytes());
    let _ = file.flush();
}
