//! What play_launch itself costs, per subsystem (`docs/design/health-page.md`
//! §G).
//!
//! Every tokio worker is named `play_launch-worker` and tasks migrate between
//! them, so per-thread CPU cannot say which subsystem spent it. Instead each
//! subsystem wraps its synchronous work in [`section`], which reads the
//! calling thread's CPU clock on entry and exit (`CLOCK_THREAD_CPUTIME_ID`,
//! ~85 ns a read, phase 58 W2) and adds the difference to that subsystem's
//! counter. Time spent awaiting is not CPU and is not counted, which is the
//! point: a section measures work, not latency.
//!
//! A section must not span an `.await`: the task may resume on another thread,
//! and the two clock reads would belong to different threads. Wrap the
//! synchronous stretch between awaits instead.

use std::sync::atomic::{AtomicU64, Ordering};

use serde::Serialize;

/// The subsystems whose cost is attributed. Fixed so the counters are plain
/// statics with no lookup on the hot path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Subsystem {
    /// `/diagnostics` processing: conversion, registry, CSV.
    Diagnostics,
    /// Draining the children's interception rings and aggregating.
    Interception,
    /// Building the Entities snapshot.
    Entities,
    /// The resource monitor's sampling tick.
    Monitor,
    /// ROS graph queries (`/api/graph`, node topics).
    Graph,
    /// Web request handlers and SSE serialisation.
    Web,
    /// The Health page's log watcher.
    Logs,
}

impl Subsystem {
    pub const ALL: [Subsystem; 7] = [
        Subsystem::Diagnostics,
        Subsystem::Interception,
        Subsystem::Entities,
        Subsystem::Monitor,
        Subsystem::Graph,
        Subsystem::Web,
        Subsystem::Logs,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Subsystem::Diagnostics => "diagnostics",
            Subsystem::Interception => "interception",
            Subsystem::Entities => "entities",
            Subsystem::Monitor => "monitor",
            Subsystem::Graph => "graph",
            Subsystem::Web => "web",
            Subsystem::Logs => "logs",
        }
    }
}

const N: usize = Subsystem::ALL.len();
static CPU_NS: [AtomicU64; N] = [const { AtomicU64::new(0) }; N];
static CALLS: [AtomicU64; N] = [const { AtomicU64::new(0) }; N];

fn thread_cpu_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: valid clock id and a writable timespec.
    unsafe { libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, &mut ts) };
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

/// Counts the CPU the current thread spends until it is dropped.
#[must_use = "a section measures until it is dropped"]
pub struct Section {
    index: usize,
    start: u64,
}

impl Drop for Section {
    fn drop(&mut self) {
        let spent = thread_cpu_ns().saturating_sub(self.start);
        CPU_NS[self.index].fetch_add(spent, Ordering::Relaxed);
        CALLS[self.index].fetch_add(1, Ordering::Relaxed);
    }
}

/// Start counting `subsystem`'s CPU on this thread. Must not span an `.await`.
pub fn section(subsystem: Subsystem) -> Section {
    let index = Subsystem::ALL
        .iter()
        .position(|s| *s == subsystem)
        .unwrap_or(0);
    Section {
        index,
        start: thread_cpu_ns(),
    }
}

/// CPU spent so far by `subsystem`, in ns, and how many sections it ran.
pub fn total(subsystem: Subsystem) -> (u64, u64) {
    let i = Subsystem::ALL
        .iter()
        .position(|s| *s == subsystem)
        .unwrap_or(0);
    (
        CPU_NS[i].load(Ordering::Relaxed),
        CALLS[i].load(Ordering::Relaxed),
    )
}

/// The process's own CPU (utime + stime, ns), RSS (bytes) and thread count,
/// from `/proc/self/stat`.
pub fn process() -> Option<(u64, u64, u64)> {
    let stat = std::fs::read_to_string("/proc/self/stat").ok()?;
    // Fields after the command, which may contain spaces, end at ')'.
    let rest = &stat[stat.rfind(')')? + 2..];
    let f: Vec<&str> = rest.split_whitespace().collect();
    // rest[0] is field 3 (state): utime = 14, stime = 15, threads = 20,
    // rss = 24 (pages).
    let at = |field: usize| f.get(field - 3).and_then(|v| v.parse::<u64>().ok());
    let tck = unsafe { libc::sysconf(libc::_SC_CLK_TCK) }.max(1) as u64;
    let page = unsafe { libc::sysconf(libc::_SC_PAGESIZE) }.max(1) as u64;
    let cpu_ns = (at(14)? + at(15)?) * 1_000_000_000 / tck;
    Some((cpu_ns, at(24)? * page, at(20)?))
}

/// One reading of everything, for differencing.
#[derive(Debug, Clone, Default)]
pub struct Reading {
    pub at_ns: u64,
    pub process_cpu_ns: u64,
    pub rss_bytes: u64,
    pub threads: u64,
    pub subsystems: [u64; N],
    /// CPU of the rclrs executor thread (named [`RCLRS_THREAD_NAME`]), which
    /// runs every subscription callback and message conversion.
    pub rclrs_cpu_ns: u64,
}

/// The name given to the thread that spins the shared rclrs executor.
pub const RCLRS_THREAD_NAME: &str = "pl-rclrs";
/// The thread spinning the `/diagnostics` executor. Shares the prefix, so the
/// rclrs share counts both (and the wait-set threads they spawn, which
/// inherit the name).
pub const RCLRS_DIAG_THREAD_NAME: &str = "pl-rclrs-diag";

fn named_thread_cpu_ns(name: &str) -> u64 {
    let tck = unsafe { libc::sysconf(libc::_SC_CLK_TCK) }.max(1) as u64;
    let Ok(dir) = std::fs::read_dir("/proc/self/task") else {
        return 0;
    };
    let mut total = 0;
    for task in dir.flatten() {
        let path = task.path();
        let Ok(comm) = std::fs::read_to_string(path.join("comm")) else {
            continue;
        };
        if !comm.trim_end().starts_with(name) {
            continue;
        }
        if let Ok(stat) = std::fs::read_to_string(path.join("stat"))
            && let Some(close) = stat.rfind(')')
        {
            let f: Vec<&str> = stat[close + 2..].split_whitespace().collect();
            let ticks: u64 = [11, 12]
                .iter()
                .filter_map(|&i| f.get(i).and_then(|v| v.parse::<u64>().ok()))
                .sum();
            total += ticks * 1_000_000_000 / tck;
        }
    }
    total
}

pub fn read() -> Reading {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    let (process_cpu_ns, rss_bytes, threads) = process().unwrap_or_default();
    let mut subsystems = [0; N];
    for (i, s) in Subsystem::ALL.iter().enumerate() {
        subsystems[i] = total(*s).0;
    }
    Reading {
        at_ns: ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64,
        process_cpu_ns,
        rss_bytes,
        threads,
        subsystems,
        rclrs_cpu_ns: named_thread_cpu_ns(RCLRS_THREAD_NAME),
    }
}

/// CPU shares over the interval between two readings, as % of one core.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Usage {
    pub cpu_percent: f64,
    pub rss_bytes: u64,
    pub threads: u64,
    /// Per subsystem, % of one core, busiest first. The remainder of
    /// `cpu_percent` is unattributed (the rclrs executor, the runtime itself,
    /// and anything not yet wrapped in a section).
    pub subsystems: Vec<(&'static str, f64)>,
    /// The rclrs executor thread, % of one core: subscription callbacks and
    /// ROS message conversion, which no section can wrap.
    pub rclrs_percent: f64,
}

pub fn usage(before: &Reading, after: &Reading) -> Usage {
    let span = after.at_ns.saturating_sub(before.at_ns).max(1) as f64;
    let pct = |ns: u64| ns as f64 / span * 100.0;
    let mut subsystems: Vec<(&'static str, f64)> = Subsystem::ALL
        .iter()
        .enumerate()
        .map(|(i, s)| {
            (
                s.name(),
                pct(after.subsystems[i].saturating_sub(before.subsystems[i])),
            )
        })
        .collect();
    subsystems.sort_by(|a, b| b.1.total_cmp(&a.1));
    Usage {
        cpu_percent: pct(after.process_cpu_ns.saturating_sub(before.process_cpu_ns)),
        rss_bytes: after.rss_bytes,
        threads: after.threads,
        subsystems,
        rclrs_percent: pct(after.rclrs_cpu_ns.saturating_sub(before.rclrs_cpu_ns)),
    }
}

/// Log the run's totals: what each subsystem cost over the whole run.
pub fn log_totals(since: &Reading) {
    let now = read();
    let u = usage(since, &now);
    let parts: Vec<String> = u
        .subsystems
        .iter()
        .map(|(n, p)| format!("{n} {p:.2}%"))
        .collect();
    tracing::info!(
        "play_launch's own CPU over the run: {:.1}% of a core (rclrs executor {:.2}%, {})",
        u.cpu_percent,
        u.rclrs_percent,
        parts.join(", ")
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_section_counts_the_cpu_it_spans() {
        let (before, calls_before) = total(Subsystem::Graph);
        {
            let _s = section(Subsystem::Graph);
            let mut acc = 0u64;
            for i in 0..5_000_000u64 {
                acc = acc.wrapping_add(i * i);
            }
            std::hint::black_box(acc);
        }
        let (after, calls_after) = total(Subsystem::Graph);
        assert!(after > before, "a busy section must add CPU time");
        assert_eq!(calls_after, calls_before + 1);
    }

    #[test]
    fn process_reading_is_sane() {
        // utime/stime tick at CLK_TCK (100 Hz): burn past one tick first.
        let start = std::time::Instant::now();
        let mut acc = 0u64;
        while start.elapsed() < std::time::Duration::from_millis(50) {
            acc = std::hint::black_box(acc.wrapping_add(1));
        }
        let (cpu, rss, threads) = process().expect("/proc/self/stat readable");
        assert!(cpu > 0, "cpu {cpu}");
        assert!(rss > 0 && threads >= 1);
    }

    #[test]
    fn usage_is_a_share_of_the_interval() {
        let a = Reading {
            at_ns: 0,
            process_cpu_ns: 0,
            subsystems: [0; N],
            ..Default::default()
        };
        let mut b = a.clone();
        b.at_ns = 1_000_000_000;
        b.process_cpu_ns = 300_000_000;
        b.subsystems[0] = 250_000_000;
        let u = usage(&a, &b);
        assert!((u.cpu_percent - 30.0).abs() < 1e-9);
        assert_eq!(u.subsystems[0], ("diagnostics", 25.0));
    }
}
