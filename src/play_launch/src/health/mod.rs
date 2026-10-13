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
//! Sources today: the Entities snapshot (QoS, take deficits, stale and silent
//! endpoints) and play_launch's own cost. Crash loops, logs, graph-versus-
//! model and startup land as their own sources.

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

    pub fn snapshot(&self, now: SystemTime, self_cost: Option<self_cost::Usage>) -> HealthSnapshot {
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

/// The Health task: once a second, gather conditions from every source,
/// update the book, and publish a snapshot.
pub async fn run_health_task(
    log_dir: PathBuf,
    entities: Option<watch::Receiver<Arc<EntitySnapshot>>>,
    tx: watch::Sender<Arc<HealthSnapshot>>,
    mut shutdown: watch::Receiver<bool>,
) {
    let mut book = HealthBook::with_journal(&log_dir);
    let mut tick = tokio::time::interval(Duration::from_secs(1));
    // Self-cost readings for the display (last few seconds) and the rule
    // (the last SELF_COST_WINDOW).
    let mut readings: VecDeque<self_cost::Reading> = VecDeque::new();
    loop {
        tokio::select! {
            _ = tick.tick() => {}
            _ = shutdown.changed() => {
                if *shutdown.borrow() {
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
        if let Some(rx) = entities.as_ref() {
            let snap = rx.borrow().clone();
            conditions.extend(entity_conditions(&snap));
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
        let _ = tx.send(Arc::new(book.snapshot(now, shown)));
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

    #[test]
    fn a_condition_opens_once_and_closes_after_two_absent_reports() {
        let mut book = HealthBook::default();
        book.report(vec![cond("a", Severity::Error)], at(0));
        book.report(vec![cond("a", Severity::Error)], at(1));
        let s = book.snapshot(at(1), None);
        assert_eq!(s.open.len(), 1);
        assert_eq!(s.open[0].opened_ms, unix_ms(at(0)), "not reopened");
        assert_eq!(s.open[0].last_seen_ms, unix_ms(at(1)));

        book.report(vec![], at(2));
        assert_eq!(
            book.snapshot(at(2), None).open.len(),
            1,
            "one absent report is a flicker"
        );
        book.report(vec![], at(3));
        let s = book.snapshot(at(3), None);
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
        let s = book.snapshot(at(2), None);
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
        let s = book.snapshot(at(0), None);
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
