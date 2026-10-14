//! The Health page's book of what is wrong now (`docs/design/health-page.md`).
//!
//! Sources report the conditions true at this moment; [`HealthBook`] turns
//! them into issues with a lifetime. A condition seen for the first time opens
//! an issue; one absent for [`CLOSE_AFTER_ABSENT`] consecutive reports closes
//! it, so a condition flickering at the report boundary does not open and
//! close every tick. Closed issues are kept, most recent first, so "what went
//! wrong ten minutes ago" survives, and every open and close is appended to
//! `play_log/<ts>/health.jsonl` so a bundle can explain its run afterwards.
//!
//! Sources today: the Entities snapshot (QoS, take deficits, delivery latency, stale and silent
//! endpoints), member exits, logs, the ROS graph against the run, and
//! play_launch's own cost.

use std::{
    collections::{HashMap, VecDeque},
    io::Write,
    path::{Path, PathBuf},
    sync::Arc,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use serde::Serialize;
use tokio::sync::watch;

use crate::{interception::EntitySnapshot, util::self_cost};

pub mod chains;
pub mod dds_report;
pub mod discovery;
pub mod graph;
pub mod host;
pub mod logs;
pub mod params;
pub mod startup;
pub mod tf;

/// FNV-1a, for stable issue keys built from free text.
pub(crate) fn fnv(s: &str) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for b in s.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

/// Reports a condition may be missing before its issue closes.
pub const CLOSE_AFTER_ABSENT: u32 = 2;
/// Closed issues kept in memory (the journal keeps all of them).
pub const CLOSED_KEPT: usize = 500;
/// Closed issues sent to the page.
pub const CLOSED_SENT: usize = 50;
/// play_launch above this share of a core over [`SELF_COST_WINDOW`] is an
/// issue: it is the supervisor, not the workload.
pub const SELF_COST_LIMIT_PERCENT: f64 = 10.0;
pub const SELF_COST_WINDOW: Duration = Duration::from_secs(60);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    Info,
    Warning,
    Error,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Subject {
    /// `endpoint`, `topic`, `node`, `member` or `process`.
    pub kind: &'static str,
    pub name: String,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Link {
    /// The web UI view holding the evidence.
    pub view: &'static str,
    pub target: Option<String>,
}

/// A condition a source reports as true now. Its `key` is its identity
/// across reports.
#[derive(Debug, Clone, PartialEq)]
pub struct Condition {
    pub key: String,
    pub kind: &'static str,
    pub severity: Severity,
    pub subject: Subject,
    pub title: String,
    pub detail: String,
    pub link: Option<Link>,
}

#[derive(Debug, Clone, Serialize)]
pub struct Issue {
    pub key: String,
    pub kind: &'static str,
    pub severity: Severity,
    pub subject: Subject,
    pub title: String,
    pub detail: String,
    pub opened_ms: u64,
    pub last_seen_ms: u64,
    pub closed_ms: Option<u64>,
    pub link: Option<Link>,
    #[serde(skip)]
    absent: u32,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct Counts {
    pub error: usize,
    pub warning: usize,
    pub info: usize,
}

/// What the page shows: open issues, recently closed ones, and
/// play_launch's own cost.
#[derive(Debug, Clone, Default, Serialize)]
pub struct HealthSnapshot {
    pub t_ms: u64,
    pub counts: Counts,
    pub open: Vec<Issue>,
    pub closed: Vec<Issue>,
    pub self_cost: Option<self_cost::Usage>,
    /// Members by log rate, busiest first, with their top message signatures.
    pub logs: Vec<logs::MemberLogView>,
    /// The host and middleware setup, read once a minute.
    pub host: Option<host::HostFacts>,
    /// Members' resources and trends, fastest-growing first (top
    /// [`RESOURCES_SENT`]).
    pub resources: Option<crate::monitoring::trends::ResourceSnapshot>,
    /// The last parameter probe: per node, values that differ from the
    /// launch, overrides it never declared, and its `use_sim_time`.
    pub params: Option<params::ParamsReport>,
    /// TF edges as published, when the run intercepts.
    pub tf: Option<crate::interception::TfSnapshot>,
    /// Participants on the domain against the middleware's limit, and drops.
    pub discovery: Option<discovery::DiscoveryView>,
    /// The DDS report as text (`GET /api/health/dds_report`); not sent with
    /// the issues.
    #[serde(skip)]
    pub dds_report: String,
}

fn unix_ms(t: SystemTime) -> u64 {
    t.duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_millis() as u64)
}

#[derive(Default)]
pub struct HealthBook {
    open: HashMap<String, Issue>,
    closed: VecDeque<Issue>,
    journal: Option<std::fs::File>,
}

impl HealthBook {
    /// A book that journals to `<log_dir>/health.jsonl` (best effort: a
    /// journal that cannot be opened is warned about once and skipped).
    pub fn with_journal(log_dir: &Path) -> Self {
        let path = log_dir.join("health.jsonl");
        let journal = match std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&path)
        {
            Ok(f) => Some(f),
            Err(e) => {
                tracing::warn!("Health: cannot write {}: {e}", path.display());
                None
            }
        };
        Self {
            journal,
            ..Default::default()
        }
    }

    fn journal(&mut self, event: &str, issue: &Issue) {
        if let Some(f) = self.journal.as_mut() {
            #[derive(Serialize)]
            struct Line<'a> {
                event: &'a str,
                #[serde(flatten)]
                issue: &'a Issue,
            }
            if let Ok(line) = serde_json::to_string(&Line { event, issue }) {
                let _ = writeln!(f, "{line}");
            }
        }
    }

    /// Apply one report: every condition true `now`, from every source.
    pub fn report(&mut self, conditions: Vec<Condition>, now: SystemTime) {
        let now_ms = unix_ms(now);
        let mut seen = std::collections::HashSet::with_capacity(conditions.len());
        for c in conditions {
            seen.insert(c.key.clone());
            match self.open.get_mut(&c.key) {
                Some(issue) => {
                    issue.last_seen_ms = now_ms;
                    issue.absent = 0;
                    issue.title = c.title;
                    issue.detail = c.detail;
                    issue.severity = c.severity;
                    issue.link = c.link;
                }
                None => {
                    let issue = Issue {
                        key: c.key.clone(),
                        kind: c.kind,
                        severity: c.severity,
                        subject: c.subject,
                        title: c.title,
                        detail: c.detail,
                        opened_ms: now_ms,
                        last_seen_ms: now_ms,
                        closed_ms: None,
                        link: c.link,
                        absent: 0,
                    };
                    self.journal("open", &issue);
                    self.open.insert(c.key, issue);
                }
            }
        }
        let mut to_close = Vec::new();
        for (key, issue) in self.open.iter_mut() {
            if !seen.contains(key) {
                issue.absent += 1;
                if issue.absent >= CLOSE_AFTER_ABSENT {
                    to_close.push(key.clone());
                }
            }
        }
        for key in to_close {
            if let Some(mut issue) = self.open.remove(&key) {
                issue.closed_ms = Some(now_ms);
                self.journal("close", &issue);
                self.closed.push_front(issue);
                self.closed.truncate(CLOSED_KEPT);
            }
        }
    }

    pub fn snapshot(
        &self,
        now: SystemTime,
        self_cost: Option<self_cost::Usage>,
        logs: Vec<logs::MemberLogView>,
    ) -> HealthSnapshot {
        let mut open: Vec<Issue> = self.open.values().cloned().collect();
        open.sort_by(|a, b| {
            b.severity
                .cmp(&a.severity)
                .then(b.opened_ms.cmp(&a.opened_ms))
                .then_with(|| a.key.cmp(&b.key))
        });
        let mut counts = Counts::default();
        for i in &open {
            match i.severity {
                Severity::Error => counts.error += 1,
                Severity::Warning => counts.warning += 1,
                Severity::Info => counts.info += 1,
            }
        }
        HealthSnapshot {
            t_ms: unix_ms(now),
            counts,
            open,
            closed: self.closed.iter().take(CLOSED_SENT).cloned().collect(),
            self_cost,
            logs,
            host: None,
            resources: None,
            params: None,
            tf: None,
            discovery: None,
            dds_report: String::new(),
        }
    }
}

// ---------------------------------------------------------------------------
// Sources
// ---------------------------------------------------------------------------

/// Conditions from the Entities snapshot. One issue per incompatible TOPIC
/// (not per endpoint: both sides of a mismatch are one problem), one per
/// deficient or stale endpoint, and one summary for silent endpoints — most
/// of which are fine (Autoware has ~860), so they are counted, not listed.
pub fn entity_conditions(snap: &EntitySnapshot) -> Vec<Condition> {
    let mut out = Vec::new();
    let label =
        |node: &Option<String>, member: &str| node.clone().unwrap_or_else(|| member.to_string());

    let mut incompatible: HashMap<String, Vec<String>> = HashMap::new();
    let mut confirmed: std::collections::HashSet<String> = Default::default();
    let mut silent = 0usize;
    for r in &snap.rows {
        let topic = r.topic.clone().unwrap_or_else(|| "?".to_string());
        let who = label(&r.node, &r.member);
        let dir = match r.direction {
            crate::interception::Direction::Pub => "pub",
            crate::interception::Direction::Sub => "sub",
        };
        for f in &r.flags {
            match f.kind {
                "incompatible" => incompatible
                    .entry(topic.clone())
                    .or_default()
                    .push(format!("{dir} {who}: {}: {}", f.code, f.message)),
                "confirmed" => {
                    confirmed.insert(topic.clone());
                }
                "advisory" if f.code == "take_deficit" => out.push(Condition {
                    key: format!("take_deficit:{who}:{topic}"),
                    kind: "take_deficit",
                    severity: Severity::Warning,
                    subject: Subject {
                        kind: "endpoint",
                        name: format!("{who} ← {topic}"),
                    },
                    title: format!("takes fewer messages than {topic} publishes"),
                    detail: f.message.clone(),
                    link: Some(Link {
                        view: "entities",
                        target: Some(topic.clone()),
                    }),
                }),
                "advisory" if f.code == "delivery_latency" => out.push(Condition {
                    key: format!("delivery_latency:{who}:{topic}"),
                    kind: "delivery_latency",
                    severity: Severity::Warning,
                    subject: Subject {
                        kind: "endpoint",
                        name: format!("{who} ← {topic}"),
                    },
                    title: format!("takes {topic} late"),
                    detail: f.message.clone(),
                    link: Some(Link {
                        view: "entities",
                        target: Some(topic.clone()),
                    }),
                }),
                "stale" => out.push(Condition {
                    key: format!("stale:{dir}:{who}:{topic}"),
                    kind: "stale",
                    severity: Severity::Warning,
                    subject: Subject {
                        kind: "endpoint",
                        name: format!("{who} {} {topic}", if dir == "pub" { "→" } else { "←" }),
                    },
                    title: if dir == "pub" {
                        "stopped publishing".to_string()
                    } else {
                        "stopped receiving".to_string()
                    },
                    detail: f.message.clone(),
                    link: Some(Link {
                        view: "entities",
                        target: Some(topic.clone()),
                    }),
                }),
                "silent" => silent += 1,
                _ => {}
            }
        }
    }
    for (topic, mut lines) in incompatible {
        lines.sort();
        lines.dedup();
        let dds = if confirmed.contains(&topic) {
            " The middleware confirmed the incompatibility."
        } else {
            ""
        };
        out.push(Condition {
            key: format!("qos_incompatible:{topic}"),
            kind: "qos_incompatible",
            severity: Severity::Error,
            subject: Subject {
                kind: "topic",
                name: topic.clone(),
            },
            title: "publisher and subscription QoS never match: nothing is delivered".to_string(),
            detail: format!("{}.{dds}", lines.join("; ")),
            link: Some(Link {
                view: "entities",
                target: Some(topic),
            }),
        });
    }
    if silent > 0 {
        out.push(Condition {
            key: "silent:endpoints".to_string(),
            kind: "silent",
            severity: Severity::Info,
            subject: Subject {
                kind: "process",
                name: "all endpoints".to_string(),
            },
            title: if silent == 1 {
                "1 endpoint has never carried a message".to_string()
            } else {
                format!("{silent} endpoints have never carried a message")
            },
            detail: "Usually fine (unused outputs, inputs whose source is not running); \
                     a sensor input in this list is not."
                .to_string(),
            link: Some(Link {
                view: "entities",
                target: None,
            }),
        });
    }
    out
}

/// How long an OOM kill stays an open issue after it happened.
pub const OOM_ISSUE_MS: u64 = 10 * 60 * 1000;
/// Exits within `RECENT_EXIT_WINDOW_MS` that make a crash loop.
pub const CRASH_LOOP_EXITS: u32 = 3;

/// `SIGSEGV` for 11, and so on; the number when it is not a common one.
pub fn signal_name(signal: i32) -> String {
    let name = match signal {
        1 => "SIGHUP",
        2 => "SIGINT",
        3 => "SIGQUIT",
        4 => "SIGILL",
        5 => "SIGTRAP",
        6 => "SIGABRT",
        7 => "SIGBUS",
        8 => "SIGFPE",
        9 => "SIGKILL",
        11 => "SIGSEGV",
        13 => "SIGPIPE",
        14 => "SIGALRM",
        15 => "SIGTERM",
        _ => return format!("signal {signal}"),
    };
    name.to_string()
}

/// How an exit reads: "SIGSEGV after 12 s", "code 1 after 3 s".
pub fn describe_exit(e: &crate::member_actor::coordinator::ExitRecord) -> String {
    let how = match (e.signal, e.code) {
        _ if e.oom_killed == Some(true) => "killed by the OOM killer".to_string(),
        (Some(sig), _) => signal_name(sig),
        (None, Some(code)) => format!("code {code}"),
        (None, None) => "no exit status".to_string(),
    };
    match e.uptime_ms {
        Some(ms) if ms < 120_000 => format!("{how} after {:.0} s", ms as f64 / 1000.0),
        Some(ms) => format!("{how} after {:.0} min", ms as f64 / 60_000.0),
        None => how,
    }
}

/// Conditions from the members: crash loops, OOM kills, members that crashed
/// and stayed down, and members that failed to start. `stopped_by_user`
/// says whether a user stopped a member, which makes `Stopped` expected.
pub fn member_conditions(
    members: &[crate::member_actor::model::member_view::MemberSummary],
    stopped_by_user: impl Fn(&str) -> bool,
    now_ms: u64,
) -> Vec<Condition> {
    use crate::member_actor::model::member_view::MemberState;
    let mut out = Vec::new();
    for m in members {
        let link = Some(Link {
            view: "nodes",
            target: Some(m.id.clone()),
        });
        let subject = Subject {
            kind: "member",
            name: m.name.clone(),
        };
        if m.recent_exits >= CRASH_LOOP_EXITS {
            let last = m.last_exit.as_ref().map(describe_exit).unwrap_or_default();
            out.push(Condition {
                key: format!("crash_loop:{}", m.id),
                kind: "crash_loop",
                severity: Severity::Error,
                subject: subject.clone(),
                title: format!("exited {} times in 5 min", m.recent_exits),
                detail: format!("last: {last}"),
                link: link.clone(),
            });
        }
        if let Some(e) = &m.last_exit
            && e.oom_killed == Some(true)
            && now_ms.saturating_sub(e.at_ms) < OOM_ISSUE_MS
        {
            out.push(Condition {
                key: format!("oom_kill:{}", m.id),
                kind: "oom_kill",
                severity: Severity::Error,
                subject: subject.clone(),
                title: "killed by the kernel's OOM killer".to_string(),
                detail: format!(
                    "{}; its cgroup's memory.events oom_kill counter rose",
                    describe_exit(e)
                ),
                link: link.clone(),
            });
        }
        match &m.state {
            MemberState::Stopped => {
                if let Some(e) = &m.last_exit {
                    let clean = e.signal.is_none() && e.code == Some(0);
                    if !clean && !stopped_by_user(&m.id) {
                        out.push(Condition {
                            key: format!("down:{}", m.id),
                            kind: "down",
                            severity: Severity::Error,
                            subject: subject.clone(),
                            title: "exited and is not restarting".to_string(),
                            detail: format!("{} (respawn is off for it)", describe_exit(e)),
                            link: link.clone(),
                        });
                    }
                }
            }
            MemberState::Failed { error } => out.push(Condition {
                key: format!("failed:{}", m.id),
                kind: "failed",
                severity: Severity::Error,
                subject,
                title: "failed".to_string(),
                detail: error.clone(),
                link,
            }),
            _ => {}
        }
    }
    out
}

/// A self-cost condition when play_launch has used more than
/// [`SELF_COST_LIMIT_PERCENT`] of a core over the window.
pub fn self_cost_condition(window: &self_cost::Usage) -> Option<Condition> {
    if window.cpu_percent <= SELF_COST_LIMIT_PERCENT {
        return None;
    }
    let mut parts: Vec<(String, f64)> = window
        .subsystems
        .iter()
        .map(|(n, p)| (n.to_string(), *p))
        .collect();
    parts.push(("rclrs executor".to_string(), window.rclrs_percent));
    let attributed: f64 = parts.iter().map(|(_, p)| p).sum();
    parts.push((
        "unattributed".to_string(),
        (window.cpu_percent - attributed).max(0.0),
    ));
    parts.sort_by(|a, b| b.1.total_cmp(&a.1));
    let top = &parts[0];
    Some(Condition {
        key: "self_cost:play_launch".to_string(),
        kind: "self_cost",
        severity: Severity::Warning,
        subject: Subject {
            kind: "process",
            name: "play_launch".to_string(),
        },
        title: format!(
            "play_launch is using {:.0}% of a core (largest: {} {:.0}%)",
            window.cpu_percent, top.0, top.1
        ),
        detail: parts
            .iter()
            .filter(|(_, p)| *p >= 0.1)
            .map(|(n, p)| format!("{n} {p:.1}%"))
            .collect::<Vec<_>>()
            .join(", "),
        link: Some(Link {
            view: "health",
            target: None,
        }),
    })
}

/// Ask every node whose name the run knows for the parameters its launch
/// sets, one at a time (one short-lived client each, so the probe adds one
/// pair of endpoints to discovery at a time, not one per node).
async fn probe_parameters(
    node: &Arc<rclrs::Node>,
    model: &ros_launch_manifest_model::SystemModel,
    fqns: &HashMap<String, String>,
) -> params::ParamsReport {
    let mut nodes = Vec::new();
    for (member, fqn, inst) in params::probe_targets(model, fqns) {
        // A probe still going when the run ends would ask dying nodes.
        if crate::util::teardown::shutting_down() {
            break;
        }
        let expected = params::expected_params(inst, fqn);
        let names = params::probe_names(&expected);
        match crate::ros::parameter_proxy::get_values(node, fqn, &names, Duration::from_secs(2))
            .await
        {
            Ok(values) => nodes.push(params::compare(fqn, member, &expected, &names, &values)),
            Err(e) => nodes.push(params::NodeParams {
                fqn: fqn.to_string(),
                member: member.to_string(),
                error: Some(format!("{e:#}")),
                ..Default::default()
            }),
        }
    }
    let mut report = params::ParamsReport {
        probed_ms: unix_ms(SystemTime::now()),
        nodes,
        clock_published: None,
        clock: None,
        broadcast: Vec::new(),
    };
    report.separate_broadcasts();
    report
}

/// Members sent to the page in the Resources section.
pub const RESOURCES_SENT: usize = 15;
/// A trend counts once it covers this share of the window…
const TREND_MIN_SPAN: f64 = 2.0 / 3.0;
/// …and a line explains it this well: a leak is steady; a cache that fills
/// and plateaus, or a sawtooth of allocation and release, is not.
const TREND_MIN_R2: f64 = 0.8;
/// RSS growth reported: over 1 MiB a minute, and at least 10 MiB in all.
const RSS_LEAK_PER_MIN: f64 = (1 << 20) as f64;
const RSS_LEAK_MIN_GROWTH: f64 = (10 << 20) as f64;
/// Open files growing by at least this many over the window.
const FD_LEAK_MIN_GROWTH: f64 = 20.0;
/// A cgroup throttled by the cpu controller for this share of the period.
const THROTTLED_PERCENT: f64 = 20.0;

fn resources_view(
    snap: &crate::monitoring::trends::ResourceSnapshot,
) -> crate::monitoring::trends::ResourceSnapshot {
    crate::monitoring::trends::ResourceSnapshot {
        window_s: snap.window_s,
        members: snap.members.iter().take(RESOURCES_SENT).cloned().collect(),
    }
}

/// "12 min", or "40 s" under two minutes (the window is seconds in tests).
fn span_text(s: f64) -> String {
    if s < 120.0 {
        format!("{s:.0} s")
    } else {
        format!("{:.0} min", s / 60.0)
    }
}

/// `memory_growth`, `fd_growth`, `cpu_throttled`, `memory_high` per member.
pub fn resource_conditions(snap: &crate::monitoring::trends::ResourceSnapshot) -> Vec<Condition> {
    let window = snap.window_s as f64;
    let steady = |f: &crate::monitoring::trends::Fit| {
        f.span_s >= TREND_MIN_SPAN * window && f.r2 >= TREND_MIN_R2
    };
    let mib = |b: f64| b / (1u64 << 20) as f64;
    let mut out = Vec::new();
    for m in &snap.members {
        let subject = Subject {
            kind: "member",
            name: m.name.clone(),
        };
        let link = Some(Link {
            view: "nodes",
            target: Some(m.name.clone()),
        });
        if let Some(f) = m.rss_trend
            && steady(&f)
            && f.per_min > RSS_LEAK_PER_MIN
            && f.growth >= RSS_LEAK_MIN_GROWTH
        {
            out.push(Condition {
                key: format!("memory_growth:{}", m.name),
                kind: "memory_growth",
                severity: Severity::Warning,
                subject: subject.clone(),
                title: format!(
                    "memory grew {:.0} MiB in {} ({:.1} MiB/min)",
                    mib(f.growth),
                    span_text(f.span_s),
                    mib(f.per_min)
                ),
                detail: format!(
                    "now {:.0} MiB, growing steadily (r² {:.2}): a leak, or a buffer \
                     with no bound",
                    mib(m.rss_bytes as f64),
                    f.r2
                ),
                link: link.clone(),
            });
        }
        if let Some(f) = m.fd_trend
            && steady(&f)
            && f.growth >= FD_LEAK_MIN_GROWTH
        {
            out.push(Condition {
                key: format!("fd_growth:{}", m.name),
                kind: "fd_growth",
                severity: Severity::Warning,
                subject: subject.clone(),
                title: format!(
                    "open files grew by {:.0} in {}",
                    f.growth,
                    span_text(f.span_s)
                ),
                detail: format!(
                    "{} open now; a descriptor leak ends at the process's limit, where \
                     every open, socket and DDS participant fails",
                    m.fds
                ),
                link: link.clone(),
            });
        }
        if let Some(p) = m.cpu_throttled_percent
            && p >= THROTTLED_PERCENT
        {
            out.push(Condition {
                key: format!("cpu_throttled:{}", m.name),
                kind: "cpu_throttled",
                severity: Severity::Warning,
                subject: subject.clone(),
                title: format!("throttled {p:.0}% of the time by its cgroup's cpu limit"),
                detail: "cpu.max is below what it uses: its callbacks wait".to_string(),
                link: link.clone(),
            });
        }
        if let Some(n) = m.memory_high_events
            && n > 0
        {
            out.push(Condition {
                key: format!("memory_high:{}", m.name),
                kind: "memory_high",
                severity: Severity::Info,
                subject,
                title: format!("reclaimed {n} times at memory.high"),
                detail: "its cgroup's memory.high was reached: allocations stall while \
                         the kernel reclaims"
                    .to_string(),
                link,
            });
        }
    }
    out
}

/// `startup_timeline.json`, written once, when the run reaches startup
/// complete.
fn write_startup_timeline(log_dir: &Path, t: &startup::StartupTimeline) {
    let path = log_dir.join("startup_timeline.json");
    match serde_json::to_string_pretty(t) {
        Ok(json) => {
            if let Err(e) = std::fs::write(&path, json) {
                tracing::warn!("could not write {}: {e}", path.display());
            }
        }
        Err(e) => tracing::warn!("could not serialize the startup timeline: {e}"),
    }
    if let Some(last) = t
        .last_ready
        .as_ref()
        .and_then(|id| t.rows.iter().find(|r| &r.id == id))
    {
        tracing::debug!(
            "startup timeline: complete at {} ms; last ready {} at {} ms",
            t.complete_ms.unwrap_or(0),
            last.name,
            last.ready_ms.unwrap_or(0)
        );
    }
}

fn is_up(state: &crate::member_actor::model::member_view::MemberState) -> bool {
    use crate::member_actor::model::member_view::MemberState;
    matches!(
        state,
        MemberState::Running { .. } | MemberState::Loaded { .. }
    )
}

/// Each member's part in the graph comparison. A member has a known node FQN
/// when the launch named it (a ROS node with a package, a container, a loaded
/// composable); a plain `<executable>` and a node the launch did not name
/// are accounted for by namespace (`graph.rs`).
fn expected_members(
    list: &[crate::member_actor::model::member_view::MemberSummary],
    fqns: &HashMap<String, String>,
    up_since: &HashMap<String, (Option<u32>, std::time::Instant)>,
    now: std::time::Instant,
) -> Vec<graph::Expected> {
    use crate::member_actor::model::member_view::MemberType;
    list.iter()
        .map(|m| {
            let fqn = m.ros_name_known.then(|| fqns.get(&m.id).cloned()).flatten();
            let unnamed_ns = (m.member_type != MemberType::ComposableNode && fqn.is_none())
                .then(|| m.namespace.clone().unwrap_or_else(|| "/".to_string()));
            graph::Expected {
                member_id: m.id.clone(),
                member_name: m.name.clone(),
                fqn,
                unnamed_ns,
                up_for: up_since.get(&m.id).map(|(_, t)| now.duration_since(*t)),
            }
        })
        .collect()
}

/// The Health task: once a second, gather conditions from every source,
/// update the book, and publish a snapshot.
/// What the Health task reads and writes, handed over by `up`.
pub struct HealthInputs {
    pub log_dir: PathBuf,
    pub entities: Option<watch::Receiver<Arc<EntitySnapshot>>>,
    pub members: Arc<std::sync::OnceLock<Arc<crate::member_actor::MemberHandle>>>,
    pub tx: watch::Sender<Arc<HealthSnapshot>>,
    pub startup_book: Arc<std::sync::Mutex<startup::StartupBook>>,
    pub startup_tx: watch::Sender<Arc<startup::StartupTimeline>>,
    pub trends: watch::Receiver<Arc<crate::monitoring::trends::ResourceSnapshot>>,
    /// The model the run was launched from: the parameters it sets.
    pub model: Arc<ros_launch_manifest_model::SystemModel>,
    pub shutdown: watch::Receiver<bool>,
}

pub async fn run_health_task(inputs: HealthInputs) {
    let HealthInputs {
        log_dir,
        entities,
        members,
        tx,
        startup_book,
        startup_tx,
        trends,
        model,
        mut shutdown,
    } = inputs;
    let mut book = HealthBook::with_journal(&log_dir);
    let dds_env = discovery::dds_env_splits(&model);
    let mut startup_drops = discovery::StartupDrops::default();
    let mut last_report = String::new();
    let domain_id: u32 = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let mut log_watcher = logs::LogWatcher::default();
    let mut ticks: u64 = 0;
    let mut tick = tokio::time::interval(Duration::from_secs(1));
    // Self-cost readings for the display (last few seconds) and the rule
    // (the last SELF_COST_WINDOW).
    let mut readings: VecDeque<self_cost::Reading> = VecDeque::new();
    // The graph comparison: a read every GRAPH_PERIOD on a blocking thread,
    // its conditions reported every tick until the next read replaces them.
    let mut graph_job: Option<tokio::task::JoinHandle<eyre::Result<graph::GraphFacts>>> = None;
    let mut graph_conds: Vec<Condition> = Vec::new();
    let mut dangling: std::collections::HashSet<String> = Default::default();
    // Since when each member has been up, and as which pid: a respawn starts
    // the clock again.
    let mut up_since: HashMap<String, (Option<u32>, std::time::Instant)> = HashMap::new();
    let own_prefix = format!("/play_launch_{}", std::process::id());
    // Lifecycle nodes the launch asks to activate: ready means active.
    let autostart_active: std::collections::HashSet<String> = model
        .structure
        .nodes
        .iter()
        .filter(|(_, n)| {
            n.lifecycle
                && matches!(
                    n.lifecycle_autostart,
                    Some(ros_launch_manifest_model::Autostart::Active)
                )
        })
        .map(|(fqn, _)| fqn.clone())
        .collect();
    // Parameters against the launch (J) and the time base (I): a probe once
    // startup completes, then every PROBE_PERIOD.
    let mut params_job: Option<tokio::task::JoinHandle<params::ParamsReport>> = None;
    let mut params_report: Option<params::ParamsReport> = None;
    let mut next_probe: Option<std::time::Instant> = None;
    let mut clock_published: Option<bool> = None;
    let mut graph_nodes: Option<usize> = None;
    let mut discovery_view: Option<discovery::DiscoveryView> = None;
    // Host checks: a read every HOST_PERIOD on a blocking thread.
    let mut host_state = Some(host::HostState::default());
    let mut host_job: Option<tokio::task::JoinHandle<(host::HostFacts, host::HostState)>> = None;
    let mut host_facts: Option<host::HostFacts> = None;
    loop {
        tokio::select! {
            _ = tick.tick() => {}
            _ = shutdown.changed() => {
                if *shutdown.borrow() {
                    if let Some(job) = params_job.take() {
                        job.abort();
                    }
                    break;
                }
            }
        }
        let now = SystemTime::now();
        readings.push_back(self_cost::read());
        while readings.len() > SELF_COST_WINDOW.as_secs() as usize + 1 {
            readings.pop_front();
        }
        let shown = readings
            .iter()
            .rev()
            .nth(5)
            .or(readings.front())
            .map(|r| self_cost::usage(r, readings.back().unwrap()));

        let mut conditions = Vec::new();
        ticks += 1;
        // Members exist once the coordinator is built, after this task starts.
        if let Some(handle) = members.get() {
            let list = handle.list_members().await;
            conditions.extend(member_conditions(
                &list,
                |id| handle.stopped_by_user(id),
                unix_ms(now),
            ));
            if ticks.is_multiple_of(logs::LOG_TICK.as_secs().max(1)) {
                let _cost = self_cost::section(self_cost::Subsystem::Logs);
                log_watcher.tick(
                    list.iter()
                        .map(|m| (m.id.as_str(), m.name.as_str(), m.output_dir.as_path())),
                    unix_ms(now),
                );
            }
            conditions.extend(log_watcher.conditions(unix_ms(now)));

            let mono = std::time::Instant::now();
            for m in &list {
                if is_up(&m.state) {
                    let e = up_since.entry(m.id.clone()).or_insert((m.pid, mono));
                    if e.0 != m.pid {
                        *e = (m.pid, mono);
                    }
                } else {
                    up_since.remove(&m.id);
                }
            }
            if graph_job.as_ref().is_some_and(|j| j.is_finished()) {
                match graph_job.take().unwrap().await {
                    Ok(Ok(facts)) => {
                        let fqns = handle.node_fqn_map().read().await.clone();
                        let expected = expected_members(&list, &fqns, &up_since, mono);
                        let ours = graph::Ours {
                            announced: entities
                                .as_ref()
                                .map(|rx| rx.borrow().nodes.iter().cloned().collect())
                                .unwrap_or_default(),
                            own_prefix: own_prefix.clone(),
                        };
                        graph_conds = graph::graph_conditions(&facts, &expected, &ours, &dangling);
                        graph_nodes = Some(facts.nodes.len());
                        clock_published = Some(
                            facts
                                .topics
                                .iter()
                                .any(|t| t.name == "/clock" && !t.pubs.is_empty()),
                        );
                        dangling = graph::dangling_subscribed(&facts);
                    }
                    Ok(Err(e)) => tracing::debug!("health: graph read failed: {e:#}"),
                    Err(e) => tracing::debug!("health: graph read panicked: {e}"),
                }
            }
            if graph_job.is_none()
                && ticks.is_multiple_of(graph::GRAPH_PERIOD.as_secs())
                && let Some(node) = handle.ros_node()
            {
                graph_job = Some(tokio::task::spawn_blocking(move || {
                    graph::read_graph(&node)
                }));
            }
            conditions.extend(graph_conds.iter().cloned());

            // The startup timeline: rebuilt every tick (O(members + nodes)),
            // judged and written only until startup completes.
            let complete = startup_book.lock().unwrap().is_complete();
            {
                let fqns = handle.node_fqn_map().read().await.clone();
                let summary_done = handle.get_health_summary().await.startup_complete();
                let marks = entities
                    .as_ref()
                    .map(|rx| rx.borrow().startup.clone())
                    .unwrap_or_default();
                let lifecycle_marks = entities
                    .as_ref()
                    .map(|rx| rx.borrow().lifecycle.clone())
                    .unwrap_or_default();
                let timeline = {
                    let mut book = startup_book.lock().unwrap();
                    if summary_done {
                        book.mark_complete();
                    }
                    book.timeline(
                        &list,
                        &fqns,
                        &marks,
                        &lifecycle_marks,
                        &autostart_active,
                        entities.is_some(),
                        crate::runtime_enforcement::monotonic_now_ns(),
                    )
                };
                if !complete {
                    conditions.extend(startup::startup_conditions(&timeline));
                    if timeline.complete_ms.is_some() {
                        write_startup_timeline(&log_dir, &timeline);
                    }
                }
                let _ = startup_tx.send(Arc::new(timeline));
            }

            let now_i = std::time::Instant::now();
            if next_probe.is_none() && startup_book.lock().unwrap().is_complete() {
                next_probe = Some(now_i + params::probe_delay());
            }
            if params_job.as_ref().is_some_and(|j| j.is_finished()) {
                match params_job.take().unwrap().await {
                    Ok(r) => params_report = Some(r),
                    Err(e) => tracing::debug!("health: parameter probe panicked: {e}"),
                }
            }
            if params_job.is_none()
                && next_probe.is_some_and(|t| now_i >= t)
                && let Some(node) = handle.ros_node()
            {
                next_probe = Some(now_i + params::PROBE_PERIOD);
                // Only members whose ROS name the run knows: an unnamed node's
                // key (`aggregator_node-1`, #0017) is not a name a service has.
                // Not containers: a component manager starts no parameter
                // services (upstream's, and so ours), so asking one waits out
                // the timeout and learns nothing.
                let known: std::collections::HashSet<&str> = list
                    .iter()
                    .filter(|m| m.ros_name_known && !m.is_container)
                    .map(|m| m.id.as_str())
                    .collect();
                let fqns: HashMap<String, String> = handle
                    .node_fqn_map()
                    .read()
                    .await
                    .iter()
                    .filter(|(id, _)| known.contains(id.as_str()))
                    .map(|(a, b)| (a.clone(), b.clone()))
                    .collect();
                let model = model.clone();
                params_job = Some(tokio::spawn(async move {
                    probe_parameters(&node, &model, &fqns).await
                }));
            }
            if let Some(r) = params_report.as_mut() {
                r.clock_published = clock_published;
                conditions.extend(params::param_conditions(r));
                let sim_users = r
                    .nodes
                    .iter()
                    .filter(|n| n.use_sim_time == Some(true))
                    .count();
                r.clock = entities.as_ref().and_then(|rx| rx.borrow().clock.clone());
                conditions.extend(params::clock_conditions(r.clock.as_ref(), sim_users));
            }
        }
        let mut tf_view = None;
        if let Some(rx) = entities.as_ref() {
            let snap = rx.borrow().clone();
            conditions.extend(entity_conditions(&snap));
            conditions.extend(host::large_message_conditions(
                &snap.rows,
                host_facts.as_ref(),
            ));
            if let Some(t) = snap.tf.as_ref() {
                conditions.extend(tf::tf_conditions(t));
                tf_view = Some(t.clone());
            }
            if let Some(c) = snap.chains.as_ref() {
                conditions.extend(chains::chain_conditions(c));
            }
        }
        let mut host_read = false;
        if host_job.as_ref().is_some_and(|j| j.is_finished()) {
            match host_job.take().unwrap().await {
                Ok((facts, state)) => {
                    host_read = true;
                    host_facts = Some(facts);
                    host_state = Some(state);
                }
                // The read panicked and took its state with it: start over,
                // or the host is never read again.
                Err(e) => {
                    tracing::debug!("health: host read panicked: {e}");
                    host_state = Some(host::HostState::default());
                }
            }
        }
        if host_job.is_none()
            && (ticks == 1 || ticks.is_multiple_of(host::HOST_PERIOD.as_secs()))
            && let Some(mut state) = host_state.take()
        {
            let dir = log_dir.clone();
            host_job = Some(tokio::task::spawn_blocking(move || {
                (host::read(&dir, &mut state), state)
            }));
        }
        if let Some(h) = host_facts.as_ref() {
            conditions.extend(host::host_conditions(h));
        }
        conditions.extend(discovery::dds_env_conditions(&dds_env));
        let resources = trends.borrow().clone();
        conditions.extend(resource_conditions(&resources));
        if let Some(handle) = members.get() {
            let list = handle.list_members().await;
            let processes: HashMap<String, u32> = resources
                .members
                .iter()
                .map(|m| (m.name.clone(), m.processes))
                .collect();
            let v = discovery::view(&list, &processes, host_facts.as_ref(), graph_nodes);
            conditions.extend(discovery::discovery_conditions(&v));
            let complete = startup_book.lock().unwrap().is_complete();
            startup_drops.sample(std::time::Instant::now(), domain_id, complete);
            discovery::discovery_pressure(&v, &startup_drops, &mut conditions);
            discovery_view = Some(v);
        }
        if readings.len() > SELF_COST_WINDOW.as_secs() as usize
            && let Some(c) = self_cost_condition(&self_cost::usage(
                readings.front().unwrap(),
                readings.back().unwrap(),
            ))
        {
            conditions.push(c);
        }
        book.report(conditions, now);
        let mut snap = book.snapshot(now, shown, log_watcher.views(10));
        snap.host = host_facts.clone();
        snap.resources = Some(resources_view(&resources));
        snap.params = params_report.clone();
        snap.tf = tf_view;
        snap.discovery = discovery_view.clone();
        snap.dds_report = dds_report::render(&dds_report::ReportInputs {
            host: host_facts.as_ref(),
            discovery: discovery_view.as_ref(),
            dds_env: &dds_env,
            startup_drops: &startup_drops,
            open: &snap.open,
            now_ms: unix_ms(now),
        });
        // In the bundle too: each time the host is read, and at the end.
        if host_read {
            write_dds_report(&log_dir, &snap.dds_report);
        }
        last_report = snap.dds_report.clone();
        let _ = tx.send(Arc::new(snap));
    }
    write_dds_report(&log_dir, &last_report);
}

fn write_dds_report(log_dir: &std::path::Path, text: &str) {
    if text.is_empty() {
        return;
    }
    if let Err(e) = std::fs::write(log_dir.join("dds_report.txt"), text) {
        tracing::debug!("health: could not write dds_report.txt: {e}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cond(key: &str, sev: Severity) -> Condition {
        Condition {
            key: key.to_string(),
            kind: "test",
            severity: sev,
            subject: Subject {
                kind: "topic",
                name: key.to_string(),
            },
            title: format!("title {key}"),
            detail: String::new(),
            link: None,
        }
    }

    fn at(s: u64) -> SystemTime {
        UNIX_EPOCH + Duration::from_secs(1_000 + s)
    }

    impl HealthBook {
        fn snapshot_t(&self, now: SystemTime, c: Option<self_cost::Usage>) -> HealthSnapshot {
            self.snapshot(now, c, Vec::new())
        }
    }

    #[test]
    fn a_steady_rise_is_a_leak_and_a_noisy_one_or_a_short_one_is_not() {
        use crate::monitoring::trends::{Fit, MemberResources, ResourceSnapshot};
        let mib = (1u64 << 20) as f64;
        let member = |name: &str, fit: Fit, fds: Option<Fit>| MemberResources {
            name: name.into(),
            rss_bytes: 500 << 20,
            rss_trend: Some(fit),
            fd_trend: fds,
            ..Default::default()
        };
        let leak = Fit {
            per_min: 2.0 * mib,
            r2: 0.98,
            span_s: 1700.0,
            growth: 56.0 * mib,
        };
        let snap = ResourceSnapshot {
            window_s: 1800,
            members: vec![
                member(
                    "leaky",
                    leak,
                    Some(Fit {
                        per_min: 1.0,
                        r2: 0.95,
                        span_s: 1700.0,
                        growth: 28.0,
                    }),
                ),
                member("noisy", Fit { r2: 0.3, ..leak }, None),
                member(
                    "young",
                    Fit {
                        span_s: 300.0,
                        ..leak
                    },
                    None,
                ),
                member(
                    "slow",
                    Fit {
                        per_min: 0.2 * mib,
                        growth: 6.0 * mib,
                        ..leak
                    },
                    None,
                ),
            ],
        };
        let c = resource_conditions(&snap);
        let keys: Vec<&str> = c.iter().map(|c| c.key.as_str()).collect();
        assert_eq!(keys, ["memory_growth:leaky", "fd_growth:leaky"], "{c:?}");
    }

    #[test]
    fn a_condition_opens_once_and_closes_after_two_absent_reports() {
        let mut book = HealthBook::default();
        book.report(vec![cond("a", Severity::Error)], at(0));
        book.report(vec![cond("a", Severity::Error)], at(1));
        let s = book.snapshot_t(at(1), None);
        assert_eq!(s.open.len(), 1);
        assert_eq!(s.open[0].opened_ms, unix_ms(at(0)), "not reopened");
        assert_eq!(s.open[0].last_seen_ms, unix_ms(at(1)));

        book.report(vec![], at(2));
        assert_eq!(
            book.snapshot_t(at(2), None).open.len(),
            1,
            "one absent report is a flicker"
        );
        book.report(vec![], at(3));
        let s = book.snapshot_t(at(3), None);
        assert!(s.open.is_empty());
        assert_eq!(s.closed.len(), 1);
        assert_eq!(s.closed[0].closed_ms, Some(unix_ms(at(3))));
    }

    #[test]
    fn a_flicker_does_not_close_and_reopen() {
        let mut book = HealthBook::default();
        book.report(vec![cond("a", Severity::Warning)], at(0));
        book.report(vec![], at(1));
        book.report(vec![cond("a", Severity::Warning)], at(2));
        let s = book.snapshot_t(at(2), None);
        assert_eq!(s.open.len(), 1);
        assert!(s.closed.is_empty());
        assert_eq!(s.open[0].opened_ms, unix_ms(at(0)));
    }

    #[test]
    fn open_issues_sort_by_severity_and_counts_add_up() {
        let mut book = HealthBook::default();
        book.report(
            vec![
                cond("i", Severity::Info),
                cond("e", Severity::Error),
                cond("w", Severity::Warning),
            ],
            at(0),
        );
        let s = book.snapshot_t(at(0), None);
        let keys: Vec<&str> = s.open.iter().map(|i| i.key.as_str()).collect();
        assert_eq!(keys, ["e", "w", "i"]);
        assert_eq!((s.counts.error, s.counts.warning, s.counts.info), (1, 1, 1));
    }

    #[test]
    fn the_journal_records_opens_and_closes() {
        let dir = tempfile::tempdir().unwrap();
        let mut book = HealthBook::with_journal(dir.path());
        book.report(vec![cond("a", Severity::Error)], at(0));
        book.report(vec![], at(1));
        book.report(vec![], at(2));
        drop(book);
        let body = std::fs::read_to_string(dir.path().join("health.jsonl")).unwrap();
        let events: Vec<String> = body
            .lines()
            .map(|l| {
                serde_json::from_str::<serde_json::Value>(l).unwrap()["event"]
                    .as_str()
                    .unwrap()
                    .to_string()
            })
            .collect();
        assert_eq!(events, ["open", "close"]);
    }

    fn member(
        id: &str,
        state: crate::member_actor::MemberState,
        last_exit: Option<crate::member_actor::coordinator::ExitRecord>,
        recent_exits: u32,
    ) -> crate::member_actor::MemberSummary {
        crate::member_actor::MemberSummary {
            id: id.to_string(),
            name: id.rsplit('/').next().unwrap().to_string(),
            member_type: crate::member_actor::MemberType::Node,
            state,
            pid: None,
            package: Some("pkg".into()),
            executable: "exe".into(),
            namespace: None,
            target_container: None,
            is_container: false,
            exec_name: None,
            node_name: None,
            ros_name_known: false,
            stderr_last_modified: None,
            stderr_size: 0,
            stderr_preview: None,
            respawn_enabled: Some(false),
            respawn_delay: None,
            auto_load: None,
            restart_count: None,
            output_dir: std::path::PathBuf::from("/nonexistent"),
            last_exit,
            recent_exits,
        }
    }

    fn exit(
        at_ms: u64,
        code: Option<i32>,
        signal: Option<i32>,
        oom: Option<bool>,
    ) -> crate::member_actor::coordinator::ExitRecord {
        crate::member_actor::coordinator::ExitRecord {
            at_ms,
            code,
            signal,
            uptime_ms: Some(12_000),
            oom_killed: oom,
        }
    }

    #[test]
    fn crashes_and_ooms_and_down_members_become_issues() {
        use crate::member_actor::MemberState;
        let now = 1_000_000;
        let members = vec![
            // Three exits in five minutes, still respawning.
            member(
                "node:/loop",
                MemberState::Respawning { attempt: 3 },
                Some(exit(now - 1000, None, Some(11), None)),
                3,
            ),
            // OOM-killed a minute ago.
            member(
                "node:/oom",
                MemberState::Running { pid: 2 },
                Some(exit(now - 60_000, None, Some(9), Some(true))),
                1,
            ),
            // Crashed with respawn off: down.
            member(
                "node:/down",
                MemberState::Stopped,
                Some(exit(now - 5000, None, Some(6), None)),
                1,
            ),
            // Stopped by a user after a crash: not an issue.
            member(
                "node:/user",
                MemberState::Stopped,
                Some(exit(now - 5000, None, Some(6), None)),
                1,
            ),
            // Exited cleanly: not an issue.
            member(
                "node:/clean",
                MemberState::Stopped,
                Some(exit(now - 5000, Some(0), None, None)),
                1,
            ),
            // Spawn failure.
            member(
                "node:/failed",
                MemberState::Failed {
                    error: "no such executable".into(),
                },
                None,
                0,
            ),
        ];
        let conds = member_conditions(&members, |id| id == "node:/user", now);
        let mut keys: Vec<&str> = conds.iter().map(|c| c.key.as_str()).collect();
        keys.sort();
        assert_eq!(
            keys,
            [
                "crash_loop:node:/loop",
                "down:node:/down",
                "failed:node:/failed",
                "oom_kill:node:/oom"
            ]
        );
        let lp = conds.iter().find(|c| c.kind == "crash_loop").unwrap();
        assert!(lp.detail.contains("SIGSEGV after 12 s"), "{}", lp.detail);
        let oom = conds.iter().find(|c| c.kind == "oom_kill").unwrap();
        assert!(oom.detail.contains("OOM killer"), "{}", oom.detail);
        let down = conds.iter().find(|c| c.kind == "down").unwrap();
        assert!(down.detail.contains("SIGABRT"), "{}", down.detail);
    }

    #[test]
    fn an_old_oom_kill_is_no_longer_open() {
        use crate::member_actor::MemberState;
        let now = 100 * 60 * 1000;
        let m = member(
            "node:/oom",
            MemberState::Running { pid: 2 },
            Some(exit(now - OOM_ISSUE_MS - 1, None, Some(9), Some(true))),
            1,
        );
        assert!(member_conditions(&[m], |_| false, now).is_empty());
    }

    #[test]
    fn self_cost_names_the_largest_share() {
        let u = self_cost::Usage {
            cpu_percent: 30.0,
            rss_bytes: 0,
            threads: 1,
            subsystems: vec![("diagnostics", 0.4), ("monitor", 6.0)],
            rclrs_percent: 2.0,
        };
        let c = self_cost_condition(&u).unwrap();
        assert!(c.title.contains("30%"), "{}", c.title);
        assert!(c.title.contains("unattributed"), "{}", c.title);
        let under = self_cost::Usage {
            cpu_percent: 5.0,
            ..u
        };
        assert!(self_cost_condition(&under).is_none());
    }
}
