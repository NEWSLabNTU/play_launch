//! Resource trends per member (`docs/design/big-system-observability.md`
//! §K): the Nodes page shows CPU and memory NOW, and a leak is a trend, not a
//! level — a node at 800 MB is fine if it was at 800 MB an hour ago.
//!
//! The monitor feeds every sample it already takes ([`TrendBook::observe`]);
//! this keeps, per member, RSS and open fds sampled about once a
//! [`TrendConfig::sample_every`] over a [`TrendConfig::window`], fits a line
//! through each, and publishes a [`ResourceSnapshot`] the Health page reads.
//! A respawn (new pid) starts the series again: the new process's memory is
//! not the old one's growth.
//!
//! Not kept: context switches. `/proc/<pid>/status` counts the main thread
//! only, and a process total means reading every thread's file every tick,
//! the cost this module's monitor was just relieved of.

use std::{
    collections::{HashMap, VecDeque},
    path::Path,
    time::{Duration, Instant},
};

use serde::Serialize;

/// The default window: a leak worth reporting shows over half an hour.
const DEFAULT_WINDOW: Duration = Duration::from_secs(30 * 60);
/// Samples kept per window.
const SAMPLES_PER_WINDOW: u32 = 30;

#[derive(Debug, Clone, Copy)]
pub struct TrendConfig {
    pub window: Duration,
    pub sample_every: Duration,
}

impl TrendConfig {
    pub fn with_window(window: Duration) -> Self {
        Self {
            window,
            sample_every: window / SAMPLES_PER_WINDOW,
        }
    }

    /// `PLAY_LAUNCH_RESOURCE_TREND_WINDOW_S` overrides the 30 min window
    /// (tests use seconds).
    pub fn from_env() -> Self {
        let window = std::env::var("PLAY_LAUNCH_RESOURCE_TREND_WINDOW_S")
            .ok()
            .and_then(|v| v.parse::<u64>().ok())
            .filter(|&s| s > 0)
            .map_or(DEFAULT_WINDOW, Duration::from_secs);
        Self::with_window(window)
    }
}

/// A least-squares line through `(seconds, value)` points.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct Fit {
    /// Change per minute.
    pub per_min: f64,
    /// How well a line explains the points, 0 to 1: a leak is steady, a
    /// cache filling then plateauing is not.
    pub r2: f64,
    /// Seconds between the first and last point.
    pub span_s: f64,
    /// The fitted change over the span.
    pub growth: f64,
}

pub fn fit(points: &[(f64, f64)]) -> Option<Fit> {
    if points.len() < 3 {
        return None;
    }
    let n = points.len() as f64;
    let (mx, my) = points
        .iter()
        .fold((0.0, 0.0), |(a, b), (x, y)| (a + x / n, b + y / n));
    let (mut sxy, mut sxx, mut syy) = (0.0, 0.0, 0.0);
    for (x, y) in points {
        sxy += (x - mx) * (y - my);
        sxx += (x - mx) * (x - mx);
        syy += (y - my) * (y - my);
    }
    if sxx == 0.0 {
        return None;
    }
    let slope = sxy / sxx;
    let r2 = if syy == 0.0 {
        0.0
    } else {
        (sxy * sxy) / (sxx * syy)
    };
    let span_s = points.last()?.0 - points.first()?.0;
    Some(Fit {
        per_min: slope * 60.0,
        r2,
        span_s,
        growth: slope * span_s,
    })
}

#[derive(Debug)]
struct MemberTrend {
    pid: u32,
    started: Instant,
    last_sample: Option<Instant>,
    rss: VecDeque<(f64, f64)>,
    fds: VecDeque<(f64, f64)>,
    latest: MemberResources,
    throttle_prev: Option<(u64, u64)>,
}

/// One member's resources and trends, as the Health page shows them.
#[derive(Debug, Clone, Default, Serialize, PartialEq)]
pub struct MemberResources {
    /// The member's log directory name.
    pub name: String,
    pub pid: u32,
    pub rss_bytes: u64,
    pub threads: u32,
    pub fds: u32,
    /// RSS fitted over the window, bytes per minute.
    pub rss_trend: Option<Fit>,
    pub fd_trend: Option<Fit>,
    /// Share of the period the member's cgroup spent throttled by the cpu
    /// controller since the previous sample; absent without one.
    pub cpu_throttled_percent: Option<f64>,
    /// The cgroup's `memory.events`, `high`/`max`/`oom_kill`, when it has one.
    pub memory_high_events: Option<u64>,
    pub memory_max_events: Option<u64>,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct ResourceSnapshot {
    pub window_s: u64,
    /// Busiest growth first.
    pub members: Vec<MemberResources>,
}

/// Per-member series, fed by the monitor.
#[derive(Debug)]
pub struct TrendBook {
    cfg: TrendConfig,
    members: HashMap<String, MemberTrend>,
}

/// What the monitor measured for one member this tick.
#[derive(Debug, Clone, Copy, Default)]
pub struct Sample {
    pub pid: u32,
    pub rss_bytes: u64,
    pub threads: u32,
    pub fds: u32,
}

impl TrendBook {
    pub fn new(cfg: TrendConfig) -> Self {
        Self {
            cfg,
            members: HashMap::new(),
        }
    }

    /// Record a tick's sample for the member logging to `output_dir`, with
    /// its cgroup when it has one of ours.
    pub fn observe(
        &mut self,
        output_dir: &Path,
        sample: Sample,
        cgroup: Option<&Path>,
        now: Instant,
    ) {
        let name = output_dir.file_name().map_or_else(
            || output_dir.display().to_string(),
            |n| n.to_string_lossy().into_owned(),
        );
        let t = self
            .members
            .entry(name.clone())
            .or_insert_with(|| MemberTrend::new(sample.pid, now));
        if t.pid != sample.pid {
            *t = MemberTrend::new(sample.pid, now);
        }
        if t.last_sample
            .is_none_or(|l| now.duration_since(l) >= self.cfg.sample_every)
        {
            t.last_sample = Some(now);
            let x = now.duration_since(t.started).as_secs_f64();
            let keep_after = x - self.cfg.window.as_secs_f64();
            for (series, v) in [
                (&mut t.rss, sample.rss_bytes as f64),
                (&mut t.fds, sample.fds as f64),
            ] {
                series.push_back((x, v));
                while series.front().is_some_and(|(px, _)| *px < keep_after) {
                    series.pop_front();
                }
            }
        }
        let (rss, fds): (Vec<_>, Vec<_>) = (
            t.rss.iter().copied().collect(),
            t.fds.iter().copied().collect(),
        );
        let mut latest = MemberResources {
            name,
            pid: sample.pid,
            rss_bytes: sample.rss_bytes,
            threads: sample.threads,
            fds: sample.fds,
            rss_trend: fit(&rss),
            fd_trend: fit(&fds),
            ..Default::default()
        };
        if let Some(group) = cgroup {
            if let Some((throttled, usage)) =
                crate::execution::cgroup::CgroupTree::cpu_throttled(group)
            {
                if let Some((pt, pu)) = t.throttle_prev {
                    let dt = throttled.saturating_sub(pt) as f64;
                    let du = usage.saturating_sub(pu) as f64;
                    latest.cpu_throttled_percent = (dt + du > 0.0).then(|| dt * 100.0 / (dt + du));
                }
                t.throttle_prev = Some((throttled, usage));
            }
            if let Some(ev) = crate::execution::cgroup::MemoryEvents::read(group) {
                latest.memory_high_events = Some(ev.high);
                latest.memory_max_events = Some(ev.max);
            }
        }
        t.latest = latest;
    }

    /// Forget members that left the monitored set.
    pub fn retain(&mut self, live: impl Fn(&str) -> bool) {
        self.members.retain(|k, _| live(k));
    }

    pub fn snapshot(&self) -> ResourceSnapshot {
        let mut members: Vec<MemberResources> =
            self.members.values().map(|t| t.latest.clone()).collect();
        let growth = |m: &MemberResources| m.rss_trend.map_or(0.0, |f| f.per_min);
        members.sort_by(|a, b| growth(b).total_cmp(&growth(a)).then(a.name.cmp(&b.name)));
        ResourceSnapshot {
            window_s: self.cfg.window.as_secs(),
            members,
        }
    }
}

impl MemberTrend {
    fn new(pid: u32, now: Instant) -> Self {
        Self {
            pid,
            started: now,
            last_sample: None,
            rss: VecDeque::new(),
            fds: VecDeque::new(),
            latest: MemberResources::default(),
            throttle_prev: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MB: f64 = 1024.0 * 1024.0;

    #[test]
    fn a_steady_line_fits_exactly_and_noise_does_not() {
        let line: Vec<(f64, f64)> = (0..30)
            .map(|i| (i as f64 * 60.0, 100.0 + 2.0 * i as f64))
            .collect();
        let f = fit(&line).unwrap();
        assert!((f.per_min - 2.0).abs() < 1e-9, "{f:?}");
        assert!((f.r2 - 1.0).abs() < 1e-9);
        assert!((f.growth - 58.0).abs() < 1e-9);
        let flat: Vec<(f64, f64)> = (0..30)
            .map(|i| (i as f64, if i % 2 == 0 { 1.0 } else { 3.0 }))
            .collect();
        assert!(fit(&flat).unwrap().r2 < 0.1);
        assert!(fit(&line[..2]).is_none(), "two points are not a trend");
    }

    #[test]
    fn samples_are_thinned_windowed_and_reset_by_a_respawn() {
        let cfg = TrendConfig::with_window(Duration::from_secs(30));
        let mut b = TrendBook::new(cfg);
        let t0 = Instant::now();
        let dir = Path::new("/log/node/leaky");
        for i in 0..120u64 {
            // Four ticks a second, 1 MB a second.
            let now = t0 + Duration::from_millis(250 * i);
            let s = Sample {
                pid: 7,
                rss_bytes: (100.0 * MB + i as f64 / 4.0 * MB) as u64,
                threads: 3,
                fds: 10,
            };
            b.observe(dir, s, None, now);
        }
        let snap = b.snapshot();
        let m = &snap.members[0];
        assert_eq!(m.name, "leaky");
        let f = m.rss_trend.unwrap();
        assert!((f.per_min / MB - 60.0).abs() < 1.0, "{f:?}");
        assert!(f.span_s <= 30.0 && f.span_s >= 25.0, "windowed: {f:?}");
        assert_eq!(m.fd_trend.unwrap().per_min, 0.0);

        b.observe(
            dir,
            Sample {
                pid: 8,
                ..Default::default()
            },
            None,
            t0 + Duration::from_secs(31),
        );
        assert!(
            b.snapshot().members[0].rss_trend.is_none(),
            "a new process starts over"
        );
    }
}
