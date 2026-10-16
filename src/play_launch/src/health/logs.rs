//! Log rates and error bursts per member (`docs/design/health-page.md` W4).
//!
//! Every [`LOG_TICK`] the watcher reads the bytes each member's `out` and `err`
//! files gained since the last read — never the whole file, and at most
//! [`MAX_READ_PER_TICK`] per file: past that it counts bytes without reading
//! them, because reading a flood to measure it would be the flood's cost twice.
//! Each line is classified by the rcutils prefix (`[ERROR] [stamp] [logger]:`)
//! and its message reduced to a signature — numbers, hex and quoted strings
//! replaced — so a line repeated with a different counter is one row with a
//! count.
//!
//! `/rosout` is deliberately not subscribed: a per-message subscription in
//! play_launch is exactly the cost the diagnostics fix removed. The files are
//! what every node writes anyway.

use std::{
    collections::{HashMap, VecDeque},
    io::{Read, Seek, SeekFrom},
    path::{Path, PathBuf},
    time::Duration,
};

use serde::Serialize;

use super::{Condition, Link, Severity, Subject};

/// How often the files are read.
pub const LOG_TICK: Duration = Duration::from_secs(2);
/// Most bytes read from one file in one tick; beyond it, bytes are counted.
pub const MAX_READ_PER_TICK: u64 = 1024 * 1024;
/// Ticks of history kept: 60 s.
const WINDOW_TICKS: usize = 30;
/// Signatures kept per member, least recently seen evicted first.
const SIGNATURES_KEPT: usize = 50;
/// A member logging faster than this, averaged over 30 s, is flooding.
pub const FLOOD_LINES_PER_S: f64 = 100.0;
/// ... or writing more than this many bytes a minute.
pub const FLOOD_BYTES_PER_MIN: u64 = 1024 * 1024;
/// More ERROR/FATAL lines than this in the last minute is a burst.
pub const ERROR_BURST_PER_MIN: u64 = 10;
/// An error signature first seen this long after the watcher started is new.
pub const NEW_ERROR_GRACE_MS: u64 = 60_000;
/// How long a new error stays an open issue.
pub const NEW_ERROR_OPEN_MS: u64 = 120_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Level {
    Plain,
    Debug,
    Info,
    Warn,
    Error,
    Fatal,
}

/// Strip ANSI escape sequences (rcutils colours its output when asked to).
fn strip_ansi(line: &str) -> std::borrow::Cow<'_, str> {
    if !line.contains('\x1b') {
        return std::borrow::Cow::Borrowed(line);
    }
    let mut out = String::with_capacity(line.len());
    let mut chars = line.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '\x1b' {
            if chars.peek() == Some(&'[') {
                chars.next();
                for c in chars.by_ref() {
                    if c.is_ascii_alphabetic() {
                        break;
                    }
                }
            }
        } else {
            out.push(c);
        }
    }
    std::borrow::Cow::Owned(out)
}

/// The level and message of an rcutils-formatted line
/// (`[LEVEL] [stamp] [logger]: message`); `Plain` and the whole line otherwise.
pub fn classify(line: &str) -> (Level, &str) {
    let Some(rest) = line.strip_prefix('[') else {
        return (Level::Plain, line);
    };
    let Some(end) = rest.find(']') else {
        return (Level::Plain, line);
    };
    let level = match &rest[..end] {
        "DEBUG" => Level::Debug,
        "INFO" => Level::Info,
        "WARN" | "WARNING" => Level::Warn,
        "ERROR" => Level::Error,
        "FATAL" => Level::Fatal,
        _ => return (Level::Plain, line),
    };
    // The message follows the logger's "]: ".
    let message = rest[end..]
        .find("]: ")
        .map_or(&rest[end + 1..], |i| &rest[end + i + 3..]);
    (level, message.trim())
}

/// A message with its variable parts replaced: digit runs (and hex) by `#`,
/// quoted strings by `"#"`. Paths and topic names are kept — they say which.
pub fn signature(message: &str) -> String {
    let mut out = String::with_capacity(message.len());
    let mut chars = message.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '0'..='9' => {
                // 0x1f, 12.5, 1e-3: one token.
                while let Some(&n) = chars.peek() {
                    if n.is_ascii_hexdigit() || n == '.' || n == 'x' || n == 'e' || n == '-' {
                        chars.next();
                    } else {
                        break;
                    }
                }
                out.push('#');
            }
            '\'' | '"' => {
                let quote = c;
                for n in chars.by_ref() {
                    if n == quote {
                        break;
                    }
                }
                out.push(quote);
                out.push('#');
                out.push(quote);
            }
            _ => out.push(c),
        }
    }
    out
}

#[derive(Debug, Clone, Serialize)]
pub struct SignatureView {
    pub level: Level,
    pub count: u64,
    pub first_ms: u64,
    pub last_ms: u64,
    /// One real line with this signature.
    pub example: String,
}

#[derive(Debug, Clone, Copy, Default)]
struct TickCounts {
    lines: u64,
    bytes: u64,
    errors: u64,
    warnings: u64,
}

#[derive(Debug, Default)]
struct MemberLog {
    name: String,
    window: VecDeque<TickCounts>,
    signatures: HashMap<String, SignatureView>,
    /// Bytes counted without being read (a flood past MAX_READ_PER_TICK).
    unread_bytes: u64,
    total_lines: u64,
    total_bytes: u64,
}

impl MemberLog {
    fn sum(&self, ticks: usize) -> TickCounts {
        let mut t = TickCounts::default();
        for c in self.window.iter().rev().take(ticks) {
            t.lines += c.lines;
            t.bytes += c.bytes;
            t.errors += c.errors;
            t.warnings += c.warnings;
        }
        t
    }

    fn note(&mut self, level: Level, message: &str, line: &str, now_ms: u64) {
        let key = signature(message);
        if let Some(sig) = self.signatures.get_mut(&key) {
            sig.count += 1;
            sig.last_ms = now_ms;
            return;
        }
        if self.signatures.len() >= SIGNATURES_KEPT
            && let Some(oldest) = self
                .signatures
                .iter()
                .min_by_key(|(_, s)| s.last_ms)
                .map(|(k, _)| k.clone())
        {
            self.signatures.remove(&oldest);
        }
        let mut example: String = line.chars().take(300).collect();
        if example.len() < line.len() {
            example.push('…');
        }
        self.signatures.insert(
            key,
            SignatureView {
                level,
                count: 1,
                first_ms: now_ms,
                last_ms: now_ms,
                example,
            },
        );
    }
}

struct FileState {
    member: String,
    offset: u64,
    /// The inode `offset` is into. A respawn renames the dead process's file
    /// to `err.N` and starts a new `err` (`execution::context::
    /// rotate_previous_incarnation`); a new file that grew past the old offset
    /// within one tick is not shorter, so only the inode tells them apart.
    ino: Option<u64>,
    /// The bytes after the last newline, carried to the next read.
    partial: Vec<u8>,
}

/// One member's log activity, as the Health page shows it.
#[derive(Debug, Clone, Serialize)]
pub struct MemberLogView {
    pub member: String,
    pub name: String,
    pub lines_per_s: f64,
    pub bytes_per_s: f64,
    pub errors_per_min: u64,
    pub warnings_per_min: u64,
    pub total_lines: u64,
    pub total_bytes: u64,
    /// Bytes counted but not read because they arrived faster than the
    /// watcher reads; their lines are not in the counts.
    pub unread_bytes: u64,
    /// Most frequent signatures first (warnings and errors before others).
    pub signatures: Vec<SignatureView>,
}

#[derive(Default)]
pub struct LogWatcher {
    files: HashMap<PathBuf, FileState>,
    members: HashMap<String, MemberLog>,
    started_ms: Option<u64>,
}

impl LogWatcher {
    /// Read what every member's `out` and `err` gained since the last tick.
    /// `members` is `(member id, display name, output dir)`. A file shared by
    /// several members (composables logging through their container) is
    /// attributed to the first that names it.
    pub fn tick<'a>(
        &mut self,
        members: impl IntoIterator<Item = (&'a str, &'a str, &'a Path)>,
        now_ms: u64,
    ) {
        self.started_ms.get_or_insert(now_ms);
        let mut counts: HashMap<String, TickCounts> = HashMap::new();
        for (id, name, dir) in members {
            let log = self.members.entry(id.to_string()).or_default();
            log.name = name.to_string();
            for stream in ["out", "err"] {
                let path = dir.join(stream);
                self.files.entry(path).or_insert_with(|| FileState {
                    member: id.to_string(),
                    offset: 0,
                    ino: None,
                    partial: Vec::new(),
                });
            }
            counts.entry(id.to_string()).or_default();
        }
        for (path, state) in self.files.iter_mut() {
            let Some(log) = self.members.get_mut(&state.member) else {
                continue;
            };
            let c = counts.entry(state.member.clone()).or_default();
            read_new(path, state, log, c, now_ms);
        }
        for (id, c) in counts {
            if let Some(log) = self.members.get_mut(&id) {
                log.window.push_back(c);
                while log.window.len() > WINDOW_TICKS {
                    log.window.pop_front();
                }
                log.total_lines += c.lines;
                log.total_bytes += c.bytes;
            }
        }
    }

    /// Members by log rate, busiest first, with their top signatures. Every
    /// member with an error in the last minute is included past `limit`.
    pub fn views(&self, limit: usize) -> Vec<MemberLogView> {
        let tick_s = LOG_TICK.as_secs_f64();
        let mut views: Vec<MemberLogView> = self
            .members
            .iter()
            .map(|(id, log)| {
                let w = log.sum(15); // 30 s
                let m = log.sum(WINDOW_TICKS); // 60 s
                let span = log.window.len().clamp(1, 15) as f64 * tick_s;
                let mut signatures: Vec<SignatureView> = log.signatures.values().cloned().collect();
                signatures.sort_by(|a, b| {
                    let rank = |l: Level| (l >= Level::Warn) as u8;
                    rank(b.level)
                        .cmp(&rank(a.level))
                        .then(b.count.cmp(&a.count))
                });
                signatures.truncate(5);
                MemberLogView {
                    member: id.clone(),
                    name: log.name.clone(),
                    lines_per_s: w.lines as f64 / span,
                    bytes_per_s: w.bytes as f64 / span,
                    errors_per_min: m.errors,
                    warnings_per_min: m.warnings,
                    total_lines: log.total_lines,
                    total_bytes: log.total_bytes,
                    unread_bytes: log.unread_bytes,
                    signatures,
                }
            })
            .collect();
        views.sort_by(|a, b| {
            b.bytes_per_s
                .total_cmp(&a.bytes_per_s)
                .then(b.lines_per_s.total_cmp(&a.lines_per_s))
        });
        let mut kept: Vec<MemberLogView> = Vec::new();
        for (i, v) in views.into_iter().enumerate() {
            // By bytes, not lines: a flood past the read cap has no counted lines
            // and is exactly the member that must show.
            if (i < limit && v.total_bytes > 0) || v.errors_per_min > 0 {
                kept.push(v);
            }
        }
        kept
    }

    /// Log floods, error bursts and new error signatures.
    pub fn conditions(&self, now_ms: u64) -> Vec<Condition> {
        let mut out = Vec::new();
        let tick_s = LOG_TICK.as_secs_f64();
        let started = self.started_ms.unwrap_or(now_ms);
        for (id, log) in &self.members {
            let subject = Subject {
                kind: "member",
                name: log.name.clone(),
            };
            let link = Some(Link {
                view: "nodes",
                target: Some(id.clone()),
            });
            let half = log.sum(15);
            let minute = log.sum(WINDOW_TICKS);
            let span = log.window.len().clamp(1, 15) as f64 * tick_s;
            let lines_per_s = half.lines as f64 / span;
            let full_minute = log.window.len() >= WINDOW_TICKS;
            let bytes_per_min = minute.bytes;
            if log.window.len() >= 15
                && (lines_per_s > FLOOD_LINES_PER_S
                    || (full_minute && bytes_per_min > FLOOD_BYTES_PER_MIN))
            {
                out.push(Condition {
                    key: format!("log_flood:{id}"),
                    kind: "log_flood",
                    severity: Severity::Warning,
                    subject: subject.clone(),
                    title: format!(
                        "logging {:.0} lines/s ({:.0} KiB/min)",
                        lines_per_s,
                        bytes_per_min as f64 / 1024.0
                    ),
                    detail: top_example(log, Level::Plain).unwrap_or_default(),
                    link: link.clone(),
                });
            }
            if minute.errors > ERROR_BURST_PER_MIN {
                out.push(Condition {
                    key: format!("error_burst:{id}"),
                    kind: "error_burst",
                    severity: Severity::Warning,
                    subject: subject.clone(),
                    title: format!("{} ERROR lines in the last minute", minute.errors),
                    detail: top_example(log, Level::Error).unwrap_or_default(),
                    link: link.clone(),
                });
            }
            for (sig, view) in &log.signatures {
                if view.level >= Level::Error
                    && view.first_ms >= started + NEW_ERROR_GRACE_MS
                    && now_ms.saturating_sub(view.first_ms) < NEW_ERROR_OPEN_MS
                {
                    out.push(Condition {
                        key: format!("new_error:{id}:{}", super::fnv(sig)),
                        kind: "new_error",
                        severity: Severity::Warning,
                        subject: subject.clone(),
                        title: "an error not seen earlier in this run".to_string(),
                        detail: format!("{} (×{})", view.example, view.count),
                        link: link.clone(),
                    });
                }
            }
        }
        out
    }
}

/// The most frequent example at or above `at_least`.
fn top_example(log: &MemberLog, at_least: Level) -> Option<String> {
    log.signatures
        .values()
        .filter(|s| s.level >= at_least)
        .max_by_key(|s| s.count)
        .map(|s| format!("{} (×{})", s.example, s.count))
}

fn read_new(
    path: &Path,
    state: &mut FileState,
    log: &mut MemberLog,
    c: &mut TickCounts,
    now_ms: u64,
) {
    let Ok(meta) = std::fs::metadata(path) else {
        return;
    };
    let len = meta.len();
    let ino = Some(std::os::unix::fs::MetadataExt::ino(&meta));
    if len < state.offset || (state.ino.is_some() && state.ino != ino) {
        // Truncated or replaced: start over.
        state.offset = 0;
        state.partial.clear();
    }
    state.ino = ino;
    let new = len - state.offset;
    if new == 0 {
        return;
    }
    c.bytes += new;
    if new > MAX_READ_PER_TICK {
        // Too much to read: count it, skip it.
        log.unread_bytes += new;
        state.offset = len;
        state.partial.clear();
        return;
    }
    let Ok(mut file) = std::fs::File::open(path) else {
        return;
    };
    if file.seek(SeekFrom::Start(state.offset)).is_err() {
        return;
    }
    let mut buf = std::mem::take(&mut state.partial);
    let before = buf.len();
    if file.by_ref().take(new).read_to_end(&mut buf).is_err() {
        buf.truncate(before);
        state.partial = buf;
        return;
    }
    state.offset += (buf.len() - before) as u64;
    let Some(last_nl) = buf.iter().rposition(|&b| b == b'\n') else {
        state.partial = buf;
        return;
    };
    state.partial = buf.split_off(last_nl + 1);
    for raw in String::from_utf8_lossy(&buf).lines() {
        let line = strip_ansi(raw);
        c.lines += 1;
        let (level, message) = classify(&line);
        match level {
            Level::Error | Level::Fatal => c.errors += 1,
            Level::Warn => c.warnings += 1,
            _ => {}
        }
        // Warnings and errors always; INFO and unformatted lines only while
        // there is room, so chatter never evicts a warning.
        let keep = level >= Level::Warn || log.signatures.len() < SIGNATURES_KEPT;
        if keep {
            log.note(level, message, &line, now_ms);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn rcutils_lines_are_classified_and_signatures_drop_the_variable_parts() {
        let (l, m) = classify(
            "[ERROR] [1791840660.932751449] [planner]: no path found after 12 tries (goal '/map/goal_3')",
        );
        assert_eq!(l, Level::Error);
        assert_eq!(m, "no path found after 12 tries (goal '/map/goal_3')");
        assert_eq!(signature(m), "no path found after # tries (goal '#')");
        assert_eq!(classify("plain stderr text").0, Level::Plain);
        assert_eq!(classify("[WARN] [1.0] [a.b]: x").0, Level::Warn);
        assert_eq!(
            signature("took 0x1f3 at 12.50 ms"),
            signature("took 0x2aa at 7.25 ms"),
            "hex and decimals collapse"
        );
        assert_eq!(strip_ansi("\x1b[31m[ERROR] x\x1b[0m"), "[ERROR] x");
    }

    fn write(path: &Path, s: &str) {
        let mut f = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .unwrap();
        f.write_all(s.as_bytes()).unwrap();
    }

    #[test]
    fn only_new_bytes_are_read_and_repeats_collapse_to_one_signature() {
        let dir = tempfile::tempdir().unwrap();
        let mut w = LogWatcher::default();
        write(
            &dir.path().join("err"),
            "[ERROR] [1.0] [n]: failed 1\n[ERROR] [1.0] [n]: failed 2\npart",
        );
        w.tick([("node:/n", "n", dir.path())], 1000);
        write(
            &dir.path().join("err"),
            "ial line\n[ERROR] [1.0] [n]: failed 3\n",
        );
        w.tick([("node:/n", "n", dir.path())], 3000);
        let v = &w.views(10)[0];
        assert_eq!(v.total_lines, 4, "the partial line is counted once, whole");
        let errors: Vec<&SignatureView> = v
            .signatures
            .iter()
            .filter(|s| s.level == Level::Error)
            .collect();
        assert_eq!(errors.len(), 1, "{errors:?}");
        assert_eq!(errors[0].count, 3);
        assert!(v.signatures.iter().any(|s| s.example == "partial line"));
    }

    /// A respawn renames the dead process's `err` to `err.1` and the new
    /// process writes a fresh `err`. Within one tick it can write MORE than
    /// the old offset, so the file is not shorter and only its inode says it
    /// is a different file: reading from the old offset would skip its start.
    #[test]
    fn a_file_replaced_by_a_respawn_is_read_from_its_start() {
        let dir = tempfile::tempdir().unwrap();
        let mut w = LogWatcher::default();
        write(&dir.path().join("err"), "[ERROR] [1.0] [n]: first life\n");
        w.tick([("node:/n", "n", dir.path())], 1000);
        std::fs::rename(dir.path().join("err"), dir.path().join("err.1")).unwrap();
        write(
            &dir.path().join("err"),
            "[ERROR] [1.0] [n]: second life begins here\n[INFO] [1.0] [n]: more\n",
        );
        w.tick([("node:/n", "n", dir.path())], 3000);
        let v = &w.views(10)[0];
        assert_eq!(v.total_lines, 3, "{:?}", v.signatures);
        assert!(
            v.signatures
                .iter()
                .any(|s| s.example.contains("second life begins here")),
            "{:?}",
            v.signatures
        );
    }

    #[test]
    fn a_flood_past_the_read_cap_is_counted_not_read() {
        let dir = tempfile::tempdir().unwrap();
        let mut w = LogWatcher::default();
        w.tick([("node:/n", "n", dir.path())], 0);
        let big = "x".repeat(MAX_READ_PER_TICK as usize + 10) + "\n";
        write(&dir.path().join("out"), &big);
        w.tick([("node:/n", "n", dir.path())], 2000);
        let v = &w.views(10)[0];
        assert_eq!(v.unread_bytes, big.len() as u64);
        assert_eq!(v.total_bytes, big.len() as u64);
    }

    #[test]
    fn bursts_floods_and_new_errors_become_conditions() {
        let dir = tempfile::tempdir().unwrap();
        let mut w = LogWatcher::default();
        // 15 ticks (30 s) of 300 lines each: 150 lines/s, a flood; 12
        // errors in the last minute, a burst.
        for t in 0..15u64 {
            let mut s = String::new();
            for i in 0..300 {
                s.push_str(&format!("[INFO] [1.0] [n]: tick {i}\n"));
            }
            if t < 12 {
                s.push_str("[ERROR] [1.0] [n]: lost connection\n");
            }
            write(&dir.path().join("out"), &s);
            w.tick([("node:/n", "n", dir.path())], t * 2000);
        }
        // After the grace period, a never-seen error.
        write(
            &dir.path().join("err"),
            "[ERROR] [1.0] [n]: camera disconnected\n",
        );
        w.tick([("node:/n", "n", dir.path())], NEW_ERROR_GRACE_MS + 5000);
        let kinds: Vec<&str> = w
            .conditions(NEW_ERROR_GRACE_MS + 5000)
            .iter()
            .map(|c| c.kind)
            .collect();
        assert!(kinds.contains(&"log_flood"), "{kinds:?}");
        assert!(kinds.contains(&"error_burst"), "{kinds:?}");
        assert!(kinds.contains(&"new_error"), "{kinds:?}");
        assert_eq!(
            kinds.iter().filter(|k| **k == "new_error").count(),
            1,
            "lost connection is not new"
        );
    }
}
