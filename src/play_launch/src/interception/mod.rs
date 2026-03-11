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
// Interception task
// ---------------------------------------------------------------------------

/// Run the interception consumer task.
///
/// Polls all child consumers at 10ms intervals, aggregates frontier and stats
/// data, and writes summary files on shutdown.
pub async fn run_interception_task(
    mut consumers: Vec<ChildConsumer>,
    log_dir: PathBuf,
    config: InterceptionSettings,
    mut shutdown_signal: tokio::sync::watch::Receiver<bool>,
) -> eyre::Result<()> {
    debug!(
        "Interception task started ({} consumers, frontier={}, stats={})",
        consumers.len(),
        config.frontier,
        config.stats,
    );

    let mut frontiers: HashMap<u64, FrontierState> = HashMap::new();
    let mut stats: HashMap<u64, TopicStats> = HashMap::new();
    let mut total_events: u64 = 0;

    let poll_interval = tokio::time::Duration::from_millis(10);

    loop {
        tokio::select! {
            _ = tokio::time::sleep(poll_interval) => {
                // Drain all consumers
                for child in &mut consumers {
                    while let Some(event) = child.consumer.pop() {
                        total_events += 1;
                        debug!(
                            "Interception event: kind={:?} topic_hash={:#x} stamp={}.{} handle={:#x} mono={}",
                            event.kind, event.topic_hash, event.stamp_sec, event.stamp_nanosec, event.handle, event.monotonic_ns
                        );
                        process_event(&event, &config, &mut frontiers, &mut stats);
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
    for child in &mut consumers {
        while let Some(event) = child.consumer.pop() {
            total_events += 1;
            process_event(&event, &config, &mut frontiers, &mut stats);
        }
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
        write_summaries(&log_dir, &frontiers, &stats, total_events)?;
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
        EventKind::Publish => {
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
) -> eyre::Result<()> {
    let interception_dir = log_dir.join("interception");
    std::fs::create_dir_all(&interception_dir)?;

    // Frontier summary
    if !frontiers.is_empty() {
        let path = interception_dir.join("frontier_summary.json");
        let json = serde_json::to_string_pretty(frontiers)?;
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
                    },
                )
            })
            .collect();

        let path = interception_dir.join("stats_summary.json");
        let json = serde_json::to_string_pretty(&stats_output)?;
        std::fs::write(&path, json)?;
        info!(
            "Interception: {} topic stats written to {}",
            stats_output.len(),
            path.display()
        );
    }

    info!("Interception: {} total events processed", total_events);

    Ok(())
}
