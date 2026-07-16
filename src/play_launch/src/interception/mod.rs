//! RCL interception consumer — reads events from SPSC ring buffers shared with
//! child processes via LD_PRELOAD'd `libplay_launch_interception.so`.
//!
//! This module handles the play_launch (parent) side:
//! - Creating shared memory ring buffers + eventfds per child process
//! - Injecting `LD_PRELOAD` and fd env vars into spawned commands
//! - Polling ring buffers for `InterceptionEvent`s
//! - Aggregating frontier state and message statistics
//! - Writing summary files on shutdown

use crate::cli::config::InterceptionSettings;
use eyre::Context;
use serde::Serialize;
use std::{
    collections::HashMap,
    os::fd::RawFd,
    path::{Path, PathBuf},
};
use tracing::{debug, info};

// ---------------------------------------------------------------------------
// InterceptionEvent — must match the #[repr(C)] layout in the .so
// ---------------------------------------------------------------------------

/// Event kind discriminator (matches play_launch_interception::event::EventKind).
/// Variants are set via `#[repr(u8)]` from shared memory, not constructed by Rust code.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
pub enum EventKind {
    PublisherInit = 0,
    SubscriptionInit = 1,
    Publish = 2,
    Take = 3,
    /// Phase 36.2: rmw_create_publisher event with parsed QoS profile.
    /// Field overload: `_pad[0]` = reliability, `_pad[1]` = durability,
    /// `_pad[2]` = history, `stamp_sec` = liveliness, `stamp_nanosec`
    /// = depth, `handle` = rmw_publisher_t*, `topic_hash` = topic FQN
    /// hash.
    QosDeclaredPub = 4,
    /// Phase 36.2: rmw_create_subscription event with parsed QoS profile.
    /// Same field overload as `QosDeclaredPub`.
    QosDeclaredSub = 5,
    /// Phase 36 DDS events: publisher's offered QoS was incompatible.
    OfferedQosIncompatible = 6,
    /// Phase 36 DDS events: subscription's requested QoS was incompatible.
    RequestedQosIncompatible = 7,
    /// Phase 36 DDS events: publisher missed offered deadline.
    OfferedDeadlineMissed = 8,
    /// Phase 36 DDS events: subscription missed requested deadline.
    RequestedDeadlineMissed = 9,
    /// Phase 36 DDS events: publisher's liveliness lost.
    LivelinessLost = 10,
    /// Phase 36 DDS events: subscription saw liveliness change.
    LivelinessChanged = 11,
    /// Phase 36 DDS events: subscription lost message(s).
    MessageLost = 12,
    /// Phase 36 polish: chunk of a topic FQN string. `_pad[0]` =
    /// chunk_idx, `_pad[1]` = total_chunks, `_pad[2]` = byte_count,
    /// `topic_hash` = FQN hash, four numeric fields reinterpreted as
    /// 24 raw payload bytes (see `event::topic_name_chunk`).
    TopicNameDeclared = 13,
    /// Phase 36 polish: chunk of a runtime msg-type identity string
    /// (`"pkg/msg/Name"`), keyed by `topic_hash`. Same chunk encoding
    /// as `TopicNameDeclared`.
    TypeNameDeclared = 14,
    /// Phase 42.0: periodic, best-effort report of this process's
    /// cumulative SPSC ring-overflow drop count (i.e. how many times a
    /// producer's `push()` returned `Err(Full)` since process start).
    /// Field overload: `monotonic_ns` = cumulative dropped-event count
    /// (NOT a timestamp), `topic_hash` = 0 (sentinel, not a real
    /// topic). Emitted opportunistically by the `.so` whenever a push
    /// succeeds and the drop count has changed since the last report —
    /// so it can itself be dropped under sustained overflow and is not
    /// guaranteed delivery. See `docs/roadmap/phase-29-rcl_interception.md`
    /// ("Drop counters") for the full design + caveats.
    RingOverflowReport = 15,
    /// Frontier-side publish notification, emitted by `FrontierPlugin`
    /// only when its per-topic stamp frontier advances. Distinct from
    /// `Publish` (`StatsPlugin`'s unconditional per-message event) so
    /// this consumer doesn't count both plugins' events toward
    /// `stats.pub_count` — see
    /// `play_launch_interception::event::EventKind::FrontierPublish`
    /// for the full rationale (fixes the stamped-topic 2x rate bug).
    FrontierPublish = 16,
}

/// Interception event (40 bytes, matches play_launch_interception::event::InterceptionEvent).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct InterceptionEvent {
    pub kind: EventKind,
    pub _pad: [u8; 3],
    pub topic_hash: u64,
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    pub handle: u64,
    pub monotonic_ns: u64,
}

const _: () = assert!(size_of::<InterceptionEvent>() == 40);

// ---------------------------------------------------------------------------
// Per-child consumer
// ---------------------------------------------------------------------------

/// Holds the consumer side of a shared memory ring buffer for one child process.
pub struct ChildConsumer {
    pub consumer: spsc_shm::Consumer<InterceptionEvent>,
    /// eventfd for async wakeup (not yet used — using timer-based polling).
    #[allow(dead_code)]
    pub event_fd: RawFd,
    /// shm fd (kept open for the child to inherit).
    pub shm_fd: RawFd,
}

// ---------------------------------------------------------------------------
// .so resolution
// ---------------------------------------------------------------------------

/// Try to find `libplay_launch_interception.so`.
///
/// Search order:
/// 1. `PLAY_LAUNCH_INTERCEPTION_SO` env var
/// 2. Relative to the current executable: `../lib/libplay_launch_interception.so`
/// 3. In `src/play_launch_interception/target/release/`
/// 4. In `src/play_launch_interception/target/debug/`
pub fn find_interception_so() -> Option<PathBuf> {
    // 1. Explicit env var
    if let Ok(path) = std::env::var("PLAY_LAUNCH_INTERCEPTION_SO") {
        let p = PathBuf::from(path);
        if p.exists() {
            return Some(p);
        }
    }

    // 2. Relative to binary
    if let Ok(exe) = std::env::current_exe()
        && let Some(dir) = exe.parent()
    {
        let candidate = dir.join("../lib/libplay_launch_interception.so");
        if candidate.exists() {
            return Some(candidate);
        }
    }

    // 3. Development paths (relative to repo root)
    for suffix in &[
        "src/play_launch_interception/target/release/libplay_launch_interception.so",
        "src/play_launch_interception/target/debug/libplay_launch_interception.so",
    ] {
        let candidate = PathBuf::from(suffix);
        if candidate.exists() {
            return Some(candidate);
        }
    }

    None
}

// ---------------------------------------------------------------------------
// Setup: create ring buffer fds and inject env vars
// ---------------------------------------------------------------------------

/// Create a shared memory ring buffer + eventfd for a child process and inject
/// the necessary env vars into the command's environment.
///
/// Returns a `ChildConsumer` that the `InterceptionListener` will poll.
pub fn setup_child_interception(
    env: &mut HashMap<String, String>,
    so_path: &Path,
    config: &InterceptionSettings,
    allowlist_path: Option<&Path>,
) -> eyre::Result<ChildConsumer> {
    let (shm_fd, event_fd) =
        spsc_shm::create::<InterceptionEvent>(config.ring_capacity).wrap_err("spsc_shm::create")?;

    // Clear FD_CLOEXEC so fds survive exec
    unsafe {
        libc::fcntl(shm_fd, libc::F_SETFD, 0);
        libc::fcntl(event_fd, libc::F_SETFD, 0);
    }

    // Create consumer (parent side) — mmap the shared memory
    let consumer = unsafe { spsc_shm::Consumer::<InterceptionEvent>::from_raw_fd(shm_fd) }
        .wrap_err("Consumer::from_raw_fd")?;

    // Inject env vars
    env.insert("LD_PRELOAD".to_string(), so_path.display().to_string());
    env.insert(
        "PLAY_LAUNCH_INTERCEPTION_SHM_FD".to_string(),
        shm_fd.to_string(),
    );
    env.insert(
        "PLAY_LAUNCH_INTERCEPTION_EVENT_FD".to_string(),
        event_fd.to_string(),
    );

    // Phase 36.7: when blocking is enabled, inject the allowlist path
    // and the block flag. Children's `rcl_publisher_init` /
    // `rcl_subscription_init` hooks refuse topics not in the file.
    if let Some(path) = allowlist_path {
        env.insert(
            "PLAY_LAUNCH_INTERCEPTION_GRAPH_FILE".to_string(),
            path.display().to_string(),
        );
        env.insert(
            "PLAY_LAUNCH_INTERCEPTION_BLOCK_GRAPH".to_string(),
            "1".to_string(),
        );
    }

    Ok(ChildConsumer {
        consumer,
        event_fd,
        shm_fd,
    })
}

// ---------------------------------------------------------------------------
// Aggregation state
// ---------------------------------------------------------------------------

#[derive(Debug, Default, Serialize)]
struct FrontierState {
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    pub event_count: u64,
}

#[derive(Debug, Default, Serialize)]
struct TopicStats {
    pub pub_count: u64,
    pub take_count: u64,
    pub first_monotonic_ns: u64,
    pub last_monotonic_ns: u64,
}

// ---------------------------------------------------------------------------
// Topic name / type reassembly (Phase 42.0)
// ---------------------------------------------------------------------------

/// In-flight reassembly state for a multi-chunk string announcement
/// (`TopicNameDeclared` / `TypeNameDeclared`). The `.so` splits long
/// strings into 24-byte chunks (see `EventKind::TopicNameDeclared`);
/// this buffers them per `topic_hash` until all chunks arrive.
///
/// Shared between this module (frontier/stats name catalog) and
/// `runtime_enforcement::RuleEngine` (graph-deviation messages +
/// `discovered_topic_types.tsv`) since both decode the same wire
/// chunk format independently.
#[derive(Debug)]
pub(crate) struct TopicNameAssembly {
    pub(crate) total: u8,
    parts: Vec<Option<Vec<u8>>>,
}

impl TopicNameAssembly {
    pub(crate) fn new(total: u8) -> Self {
        Self {
            total,
            parts: vec![None; total as usize],
        }
    }

    pub(crate) fn add(&mut self, idx: u8, bytes: &[u8]) {
        if (idx as usize) < self.parts.len() && self.parts[idx as usize].is_none() {
            self.parts[idx as usize] = Some(bytes.to_vec());
        }
    }

    pub(crate) fn assembled(&self) -> Option<String> {
        if self.parts.iter().any(|p| p.is_none()) {
            return None;
        }
        let mut out = Vec::with_capacity(self.total as usize * 24);
        for bytes in self.parts.iter().flatten() {
            out.extend_from_slice(bytes);
        }
        String::from_utf8(out).ok()
    }
}

/// Mirror of `play_launch_interception::event::InterceptionEvent::decode_topic_name_chunk`
/// — the .so and play_launch are separate crates so the binary-level
/// layout is shared but the helper isn't reachable. Both sides use
/// host-native byte order so a direct copy of the numeric fields is
/// safe.
#[inline]
pub(crate) fn decode_topic_name_chunk(event: &InterceptionEvent) -> (u8, u8, usize, [u8; 24]) {
    let mut buf = [0u8; 24];
    buf[0..4].copy_from_slice(&event.stamp_sec.to_ne_bytes());
    buf[4..8].copy_from_slice(&event.stamp_nanosec.to_ne_bytes());
    buf[8..16].copy_from_slice(&event.handle.to_ne_bytes());
    buf[16..24].copy_from_slice(&event.monotonic_ns.to_ne_bytes());
    (event._pad[0], event._pad[1], event._pad[2] as usize, buf)
}

/// Accumulates `topic_hash -> name` (and `topic_hash -> msg type`) maps
/// from `TopicNameDeclared` / `TypeNameDeclared` chunk events, for
/// inclusion in `frontier_summary.json` / `stats_summary.json`.
///
/// Kept separate from `RuleEngine::discovered_names` /
/// `discovered_types` (`runtime_enforcement/mod.rs`) — that map only
/// exists when a `RuleEngine` is instantiated (contracts/manifest
/// audit, Phase 36/40), whereas this one is always populated whenever
/// interception is enabled, regardless of whether contracts are in
/// use. This is precisely the gap identified in
/// `.superpowers/sdd/p42-infra-readiness.md` capability 4: previously
/// the only way to resolve a topic hash to a name was the RuleEngine's
/// incidental `discovered_topic_types.tsv` side effect.
#[derive(Debug, Default)]
struct NameCatalog {
    names: HashMap<u64, String>,
    types: HashMap<u64, String>,
    name_chunks: HashMap<u64, TopicNameAssembly>,
    type_chunks: HashMap<u64, TopicNameAssembly>,
}

impl NameCatalog {
    fn observe(&mut self, event: &InterceptionEvent) {
        match event.kind {
            EventKind::TopicNameDeclared => {
                let (idx, total, n, buf) = decode_topic_name_chunk(event);
                let entry = self
                    .name_chunks
                    .entry(event.topic_hash)
                    .or_insert_with(|| TopicNameAssembly::new(total.max(1)));
                entry.add(idx, &buf[..n]);
                if let Some(name) = entry.assembled() {
                    self.names.entry(event.topic_hash).or_insert(name);
                    self.name_chunks.remove(&event.topic_hash);
                }
            }
            EventKind::TypeNameDeclared => {
                let (idx, total, n, buf) = decode_topic_name_chunk(event);
                let entry = self
                    .type_chunks
                    .entry(event.topic_hash)
                    .or_insert_with(|| TopicNameAssembly::new(total.max(1)));
                entry.add(idx, &buf[..n]);
                if let Some(ty) = entry.assembled() {
                    self.types.entry(event.topic_hash).or_insert(ty);
                    self.type_chunks.remove(&event.topic_hash);
                }
            }
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------
// Interception task
// ---------------------------------------------------------------------------

/// Run the interception consumer task.
///
/// Polls all child consumers at 10ms intervals, aggregates frontier and stats
/// data, and writes summary files on shutdown.
///
/// Phase 36.3: when `rule_engine` is `Some`, every event is also fed
/// to the runtime enforcement engine. The engine writes violations to
/// `<log_dir>/runtime_violations.jsonl` and flags `strict_violated`
/// in `EnforceMode::Strict` so the caller can trigger shutdown.
pub async fn run_interception_task(
    mut consumers: Vec<ChildConsumer>,
    log_dir: PathBuf,
    config: InterceptionSettings,
    mut shutdown_signal: tokio::sync::watch::Receiver<bool>,
    mut rule_engine: Option<crate::runtime_enforcement::RuleEngine>,
    mut lifecycle_rx: Option<
        tokio::sync::mpsc::UnboundedReceiver<(String, crate::runtime_enforcement::LifecycleState)>,
    >,
) -> eyre::Result<()> {
    debug!(
        "Interception task started ({} consumers, frontier={}, stats={})",
        consumers.len(),
        config.frontier,
        config.stats,
    );

    let mut frontiers: HashMap<u64, FrontierState> = HashMap::new();
    let mut stats: HashMap<u64, TopicStats> = HashMap::new();
    let mut names = NameCatalog::default();
    // Phase 42.0: per-child (per-process) latest reported cumulative
    // drop count, keyed by index into `consumers`. `RingOverflowReport`
    // carries a cumulative-since-process-start count, so later reports
    // simply overwrite earlier ones for the same child; the final
    // events-dropped estimate is the sum across all children. This is
    // a lower bound (best-effort reports can themselves be dropped),
    // never an overestimate.
    let mut drop_counts: HashMap<usize, u64> = HashMap::new();
    let mut total_events: u64 = 0;

    let poll_interval = tokio::time::Duration::from_millis(10);

    loop {
        // Drain any pending lifecycle updates first so subsequent rule
        // observations see the latest state.
        if let Some(rx) = lifecycle_rx.as_mut() {
            while let Ok((fqn, state)) = rx.try_recv() {
                if let Some(re) = rule_engine.as_mut() {
                    re.set_lifecycle_state(&fqn, state);
                }
            }
        }

        tokio::select! {
            _ = tokio::time::sleep(poll_interval) => {
                // Drain all consumers
                for (child_idx, child) in consumers.iter_mut().enumerate() {
                    while let Some(event) = child.consumer.pop() {
                        total_events += 1;
                        debug!(
                            "Interception event: kind={:?} topic_hash={:#x} stamp={}.{} handle={:#x} mono={}",
                            event.kind, event.topic_hash, event.stamp_sec, event.stamp_nanosec, event.handle, event.monotonic_ns
                        );
                        process_event(&event, &config, &mut frontiers, &mut stats);
                        names.observe(&event);
                        if event.kind == EventKind::RingOverflowReport {
                            drop_counts.insert(child_idx, event.monotonic_ns);
                        }
                        if let Some(re) = rule_engine.as_mut() {
                            re.observe(&event);
                        }
                    }
                }
            }
            _ = shutdown_signal.changed() => {
                if *shutdown_signal.borrow() {
                    break;
                }
            }
        }
    }

    // Final drain
    for (child_idx, child) in consumers.iter_mut().enumerate() {
        while let Some(event) = child.consumer.pop() {
            total_events += 1;
            process_event(&event, &config, &mut frontiers, &mut stats);
            names.observe(&event);
            if event.kind == EventKind::RingOverflowReport {
                drop_counts.insert(child_idx, event.monotonic_ns);
            }
            if let Some(re) = rule_engine.as_mut() {
                re.observe(&event);
            }
        }
    }

    if let Some(re) = rule_engine.as_mut() {
        re.flush();
    }

    // Close fds
    for child in &consumers {
        unsafe {
            libc::close(child.shm_fd);
            libc::close(child.event_fd);
        }
    }

    // Write summaries
    if total_events > 0 {
        let total_dropped: u64 = drop_counts.values().sum();
        write_summaries(
            &log_dir,
            &frontiers,
            &stats,
            total_events,
            &names.names,
            &names.types,
            total_dropped,
        )?;
    } else {
        debug!("Interception task: no events received");
    }

    debug!(
        "Interception task shutdown (processed {} events)",
        total_events
    );
    Ok(())
}

fn process_event(
    event: &InterceptionEvent,
    config: &InterceptionSettings,
    frontiers: &mut HashMap<u64, FrontierState>,
    stats: &mut HashMap<u64, TopicStats>,
) {
    match event.kind {
        EventKind::FrontierPublish => {
            // Emitted only by FrontierPlugin, only when its frontier
            // advances. Feeds frontier state exclusively — never
            // `stats.pub_count` — so it can't double-count alongside
            // StatsPlugin's unconditional `Publish` event below (the
            // stamped-topic 2x rate bug fixed in Phase 42.x).
            if config.frontier {
                let frontier = frontiers.entry(event.topic_hash).or_default();
                // Max-update (same logic as in the .so)
                let new = pack(event.stamp_sec, event.stamp_nanosec);
                let old = pack(frontier.stamp_sec, frontier.stamp_nanosec);
                if new > old {
                    frontier.stamp_sec = event.stamp_sec;
                    frontier.stamp_nanosec = event.stamp_nanosec;
                }
                frontier.event_count += 1;
            }
        }
        EventKind::Publish => {
            // Emitted unconditionally by StatsPlugin, once per message
            // (stamped or not). Feeds pub_count/rate stats exclusively —
            // frontier state comes only from `FrontierPublish` above.
            if config.stats {
                let ts = stats.entry(event.topic_hash).or_default();
                ts.pub_count += 1;
                if ts.first_monotonic_ns == 0 {
                    ts.first_monotonic_ns = event.monotonic_ns;
                }
                ts.last_monotonic_ns = event.monotonic_ns;
            }
        }
        EventKind::Take => {
            if config.stats {
                let ts = stats.entry(event.topic_hash).or_default();
                ts.take_count += 1;
                if ts.first_monotonic_ns == 0 {
                    ts.first_monotonic_ns = event.monotonic_ns;
                }
                ts.last_monotonic_ns = event.monotonic_ns;
            }
        }
        EventKind::PublisherInit | EventKind::SubscriptionInit => {
            // Init events don't affect frontier/stats aggregation
        }
        EventKind::QosDeclaredPub | EventKind::QosDeclaredSub => {
            // Phase 36.2: forwarded to the RuleEngine (Phase 36.3) by
            // the dispatcher in commands/replay.rs. Not aggregated
            // into frontier or stats summaries.
        }
        EventKind::OfferedQosIncompatible
        | EventKind::RequestedQosIncompatible
        | EventKind::OfferedDeadlineMissed
        | EventKind::RequestedDeadlineMissed
        | EventKind::LivelinessLost
        | EventKind::LivelinessChanged
        | EventKind::MessageLost => {
            // Phase 36: forwarded to the RuleEngine. Not aggregated
            // into frontier or stats summaries.
        }
        EventKind::TopicNameDeclared | EventKind::TypeNameDeclared => {
            // Phase 42.0: handled by `NameCatalog::observe` (called
            // alongside this function in `run_interception_task`) and
            // also forwarded to the RuleEngine. Not aggregated into
            // frontier or stats *state*, but folded into the summary
            // JSON at write time via the name/type catalog.
        }
        EventKind::RingOverflowReport => {
            // Phase 42.0: handled directly in `run_interception_task`'s
            // drain loops (keyed by child index) so we can distinguish
            // which child's ring is reporting. Not aggregated here.
        }
    }
}

#[inline]
fn pack(sec: i32, nanosec: u32) -> u64 {
    ((sec as u32 as u64) << 32) | (nanosec as u64)
}

fn write_summaries(
    log_dir: &Path,
    frontiers: &HashMap<u64, FrontierState>,
    stats: &HashMap<u64, TopicStats>,
    total_events: u64,
    names: &HashMap<u64, String>,
    types: &HashMap<u64, String>,
    total_dropped: u64,
) -> eyre::Result<()> {
    let interception_dir = log_dir.join("interception");
    std::fs::create_dir_all(&interception_dir)?;

    // Frontier summary
    if !frontiers.is_empty() {
        #[derive(Serialize)]
        struct FrontierOutput {
            stamp_sec: i32,
            stamp_nanosec: u32,
            event_count: u64,
            // Phase 42.0: additive fields — populated from
            // `TopicNameDeclared`/`TypeNameDeclared` events observed
            // in this run. Omitted (not `null`) when the hash was
            // never announced, so old consumers keyed only on
            // `stamp_sec`/`stamp_nanosec`/`event_count` see no schema
            // change.
            #[serde(skip_serializing_if = "Option::is_none")]
            name: Option<String>,
            #[serde(skip_serializing_if = "Option::is_none")]
            msg_type: Option<String>,
        }

        let frontier_output: HashMap<u64, FrontierOutput> = frontiers
            .iter()
            .map(|(&hash, f)| {
                (
                    hash,
                    FrontierOutput {
                        stamp_sec: f.stamp_sec,
                        stamp_nanosec: f.stamp_nanosec,
                        event_count: f.event_count,
                        name: names.get(&hash).cloned(),
                        msg_type: types.get(&hash).cloned(),
                    },
                )
            })
            .collect();

        let mut value = serde_json::to_value(&frontier_output)?;
        insert_meta(&mut value, total_dropped);

        let path = interception_dir.join("frontier_summary.json");
        let json = serde_json::to_string_pretty(&value)?;
        std::fs::write(&path, json)?;
        info!(
            "Interception: {} topic frontiers written to {}",
            frontiers.len(),
            path.display()
        );
    }

    // Stats summary
    if !stats.is_empty() {
        #[derive(Serialize)]
        struct StatsOutput {
            pub_count: u64,
            take_count: u64,
            duration_ms: f64,
            avg_pub_rate_hz: f64,
            // Phase 42.0: additive, see `FrontierOutput` above.
            #[serde(skip_serializing_if = "Option::is_none")]
            name: Option<String>,
            #[serde(skip_serializing_if = "Option::is_none")]
            msg_type: Option<String>,
        }

        let stats_output: HashMap<u64, StatsOutput> = stats
            .iter()
            .map(|(&hash, ts)| {
                let duration_ns = ts.last_monotonic_ns.saturating_sub(ts.first_monotonic_ns);
                let duration_ms = duration_ns as f64 / 1_000_000.0;
                let duration_secs = duration_ms / 1000.0;
                let avg_pub_rate_hz = if duration_secs > 0.0 {
                    ts.pub_count as f64 / duration_secs
                } else {
                    0.0
                };
                (
                    hash,
                    StatsOutput {
                        pub_count: ts.pub_count,
                        take_count: ts.take_count,
                        duration_ms,
                        avg_pub_rate_hz,
                        name: names.get(&hash).cloned(),
                        msg_type: types.get(&hash).cloned(),
                    },
                )
            })
            .collect();

        let mut value = serde_json::to_value(&stats_output)?;
        insert_meta(&mut value, total_dropped);

        let path = interception_dir.join("stats_summary.json");
        let json = serde_json::to_string_pretty(&value)?;
        std::fs::write(&path, json)?;
        info!(
            "Interception: {} topic stats written to {}",
            stats_output.len(),
            path.display()
        );
    }

    if total_dropped > 0 {
        info!(
            "Interception: {} event(s) known-dropped to ring overflow (best-effort estimate, \
             lower bound — see `_events_dropped_total_best_effort` in the summaries)",
            total_dropped
        );
    }

    info!("Interception: {} total events processed", total_events);

    Ok(())
}

/// Inserts a top-level `_events_dropped_total_best_effort` key into a
/// `HashMap<u64, _>`-shaped JSON object. The key is a non-numeric
/// string, so it can never collide with a topic-hash key (those always
/// serialize as decimal-digit strings) — existing consumers that
/// iterate "every key is a topic hash" need a one-line allowlist/skip
/// for this one metadata key, but nothing about the *shape* changes.
///
/// Value is a lower-bound estimate: it only counts drops that a
/// producer *also* managed to report back through the (possibly full)
/// ring before the process exited. See "Drop counters" in
/// `docs/roadmap/phase-29-rcl_interception.md` for the full design.
fn insert_meta(value: &mut serde_json::Value, total_dropped: u64) {
    if let serde_json::Value::Object(map) = value {
        map.insert(
            "_events_dropped_total_best_effort".to_string(),
            serde_json::Value::from(total_dropped),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cli::config::InterceptionSettings;

    /// Build a synthetic event the same way FrontierPlugin's shared-memory
    /// transport does for an advancing stamp: `FrontierPublish`, not `Publish`.
    fn frontier_publish_event(topic_hash: u64, sec: i32, nanosec: u32, mono: u64) -> InterceptionEvent {
        InterceptionEvent {
            kind: EventKind::FrontierPublish,
            _pad: [0; 3],
            topic_hash,
            stamp_sec: sec,
            stamp_nanosec: nanosec,
            handle: 0x100,
            monotonic_ns: mono,
        }
    }

    /// Build a synthetic event the same way StatsPlugin does for every
    /// message (stamped or not): plain `Publish`.
    fn stats_publish_event(topic_hash: u64, sec: i32, nanosec: u32, mono: u64) -> InterceptionEvent {
        InterceptionEvent {
            kind: EventKind::Publish,
            _pad: [0; 3],
            topic_hash,
            stamp_sec: sec,
            stamp_nanosec: nanosec,
            handle: 0x100,
            monotonic_ns: mono,
        }
    }

    fn stats_take_event(topic_hash: u64, sec: i32, nanosec: u32, mono: u64) -> InterceptionEvent {
        InterceptionEvent {
            kind: EventKind::Take,
            _pad: [0; 3],
            topic_hash,
            stamp_sec: sec,
            stamp_nanosec: nanosec,
            handle: 0x200,
            monotonic_ns: mono,
        }
    }

    /// Regression test for the stamped-topic 2x rate bug: a single
    /// stamped message previously produced *two* ring events counted as
    /// two publishes — one plain `Publish` from `StatsPlugin` (emitted
    /// unconditionally per message) and one from `FrontierPlugin` (which,
    /// before this fix, also emitted `Publish` whenever its frontier
    /// advanced — which is true for essentially every real message).
    /// `process_event` counted every `Publish` it saw, doubling
    /// `stats.pub_count`. With `FrontierPlugin` now emitting the distinct
    /// `FrontierPublish` kind, one stamped message → one `FrontierPublish`
    /// (frontier-only) + one `Publish` (stats-only) → `pub_count == 1`.
    #[test]
    fn stamped_message_through_both_plugins_counts_once() {
        let config = InterceptionSettings::default();
        let mut frontiers: HashMap<u64, FrontierState> = HashMap::new();
        let mut stats: HashMap<u64, TopicStats> = HashMap::new();

        let topic_hash = 0xABCD_EF01;

        // Simulate one real publish call dispatched to both plugins, as
        // `plugin_dispatch::dispatch_publish` does: FrontierPlugin's
        // event first (order shouldn't matter), then StatsPlugin's.
        let frontier_ev = frontier_publish_event(topic_hash, 10, 500, 1_000);
        let stats_ev = stats_publish_event(topic_hash, 10, 500, 1_000);

        process_event(&frontier_ev, &config, &mut frontiers, &mut stats);
        process_event(&stats_ev, &config, &mut frontiers, &mut stats);

        let ts = stats.get(&topic_hash).expect("stats entry present");
        assert_eq!(ts.pub_count, 1, "one message must count as exactly one publish");

        let frontier = frontiers.get(&topic_hash).expect("frontier entry present");
        assert_eq!(frontier.event_count, 1, "frontier advance must be counted exactly once");
        assert_eq!(frontier.stamp_sec, 10);
        assert_eq!(frontier.stamp_nanosec, 500);
    }

    /// Same double-count regression, but for takes: `FrontierPlugin`'s
    /// shared-memory transport no longer emits anything for `on_take`
    /// (see `plugins/frontier.rs`), so only `StatsPlugin`'s `Take` event
    /// reaches the consumer for a given message.
    #[test]
    fn stamped_take_through_both_plugins_counts_once() {
        let config = InterceptionSettings::default();
        let mut frontiers: HashMap<u64, FrontierState> = HashMap::new();
        let mut stats: HashMap<u64, TopicStats> = HashMap::new();

        let topic_hash = 0x1234_5678;

        // FrontierPlugin emits nothing for take (shm mode) — only
        // StatsPlugin's Take event is observed by the consumer.
        let stats_ev = stats_take_event(topic_hash, 5, 100, 2_000);
        process_event(&stats_ev, &config, &mut frontiers, &mut stats);

        let ts = stats.get(&topic_hash).expect("stats entry present");
        assert_eq!(ts.take_count, 1, "one message must count as exactly one take");
    }

    /// Multiple stamped messages in sequence should each count once,
    /// exercising the full advance-then-plateau frontier pattern.
    #[test]
    fn multiple_stamped_publishes_count_linearly() {
        let config = InterceptionSettings::default();
        let mut frontiers: HashMap<u64, FrontierState> = HashMap::new();
        let mut stats: HashMap<u64, TopicStats> = HashMap::new();
        let topic_hash = 0x9999;

        for i in 0..5u32 {
            let frontier_ev = frontier_publish_event(topic_hash, 0, i, 1_000 + i as u64);
            let stats_ev = stats_publish_event(topic_hash, 0, i, 1_000 + i as u64);
            process_event(&frontier_ev, &config, &mut frontiers, &mut stats);
            process_event(&stats_ev, &config, &mut frontiers, &mut stats);
        }

        assert_eq!(stats.get(&topic_hash).unwrap().pub_count, 5);
        assert_eq!(frontiers.get(&topic_hash).unwrap().event_count, 5);
    }

    /// `config.frontier = false` must fully disable frontier aggregation
    /// (including for `FrontierPublish`), while stats still accumulate.
    #[test]
    fn frontier_disabled_skips_frontier_publish() {
        let config = InterceptionSettings {
            frontier: false,
            ..InterceptionSettings::default()
        };
        let mut frontiers: HashMap<u64, FrontierState> = HashMap::new();
        let mut stats: HashMap<u64, TopicStats> = HashMap::new();
        let topic_hash = 0x4242;

        let frontier_ev = frontier_publish_event(topic_hash, 1, 0, 1);
        let stats_ev = stats_publish_event(topic_hash, 1, 0, 1);
        process_event(&frontier_ev, &config, &mut frontiers, &mut stats);
        process_event(&stats_ev, &config, &mut frontiers, &mut stats);

        assert!(frontiers.is_empty(), "frontier tracking disabled by config");
        assert_eq!(stats.get(&topic_hash).unwrap().pub_count, 1);
    }
}
