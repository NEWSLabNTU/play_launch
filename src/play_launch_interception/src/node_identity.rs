//! Report the node names a process actually registers (issue #0017).
//!
//! For a `<node>` the launch file did not name, the model key is built from
//! the EXECUTABLE while the process registers whatever name it was compiled
//! with — `/…/autoware_ekf_localizer_node-1` in the model against
//! `/…/ekf_localizer` in the graph. play_launch cannot know the second from
//! the launch file, and must not impose the first (forcing `-r __node:=`
//! renames the node away from its own default; bug `af7c524`).
//!
//! Inside the process the answer is simply there. `rcl_publisher_init` and
//! `rcl_subscription_init` both receive an `rcl_node_t*`, and the hooks
//! already call `rcl_node_get_name`/`rcl_node_get_namespace` on it to expand
//! relative topic names. So this costs one string compare per init call and
//! nothing at all on the hot path.
//!
//! Why a file rather than the ring buffer: `InterceptionEvent` is a fixed
//! 56-byte record that carries topics as FNV-1a hashes, which works because
//! the consumer already knows the topic names and can hash them to compare.
//! That is exactly what does not hold here — the whole point is that
//! play_launch does NOT know the name, so a hash it cannot invert reports
//! nothing.
//!
//! The file is opened `O_APPEND` and each line is written with a single
//! `write(2)`. Every child of a launch appends to one file concurrently, and
//! `O_APPEND` writes below `PIPE_BUF` do not interleave.
//!
//! Inert unless `PLAY_LAUNCH_INTERCEPTION_IDENTITY_FILE` is set, which
//! play_launch sets only when interception is enabled. A node that never
//! creates a publisher or subscription is never seen; hooking `rcl_node_init`
//! directly would make this earlier and unconditional, and is the obvious
//! next step if such a node turns up.

use std::{
    collections::HashSet,
    io::Write,
    sync::{Mutex, OnceLock},
};

/// The identity sink, resolved once from the environment.
struct Sink {
    file: Mutex<std::fs::File>,
    /// The model key play_launch spawned this process under. Half of the
    /// mapping — without it the consumer would have to join by PID, and a
    /// container hosting several nodes has one PID and many names.
    member: String,
    /// FQNs already written by this process. A container in `stock` or
    /// `observable` mode hosts many nodes in one process, and each of them
    /// creates many publishers.
    seen: Mutex<HashSet<String>>,
}

static SINK: OnceLock<Option<Sink>> = OnceLock::new();

fn sink() -> Option<&'static Sink> {
    SINK.get_or_init(|| {
        let path = std::env::var("PLAY_LAUNCH_INTERCEPTION_IDENTITY_FILE").ok()?;
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

/// Record that this process registered a node at `namespace` + `name`.
///
/// Called from the publisher/subscription init hooks, which already hold the
/// resolved name and namespace. Cheap and idempotent: after the first call for
/// a given FQN it is a hash-set lookup.
pub fn observe(name: &str, namespace: &str) {
    let Some(sink) = sink() else {
        return;
    };
    if name.is_empty() {
        return;
    }

    // `rcl_node_get_namespace` returns "/" at the root, so joining naively
    // would give "//name".
    let ns = namespace.trim_end_matches('/');
    let fqn = format!("{ns}/{name}");

    {
        let mut seen = match sink.seen.lock() {
            Ok(g) => g,
            Err(p) => p.into_inner(),
        };
        if !seen.insert(fqn.clone()) {
            return;
        }
    }

    let line = format!("{}\t{}\t{}\n", sink.member, std::process::id(), fqn);
    let mut file = match sink.file.lock() {
        Ok(g) => g,
        Err(p) => p.into_inner(),
    };
    // Best-effort by design. This is a diagnostic aid; a failed write must
    // never disturb the node it is describing.
    let _ = file.write_all(line.as_bytes());
    let _ = file.flush();
}
