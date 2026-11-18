//! The runs under a `play_log/` directory: listing, sizing by item class,
//! and removal (#0098). Shared by `play_launch log list|prune`, the opt-in
//! retention at the start of a run (`play_log.keep_runs`,
//! `play_log.max_total_mb`) and the reminder printed when the directory is
//! large (`play_log.remind_mb`).
//!
//! Two rules hold everywhere here. Only a directory named like a run
//! (`YYYY-MM-DD_HH-MM-SS[-N]`, local time) is ever touched, never `latest` or
//! anything else a user put beside the runs. And a run whose `run_info.json`
//! names a live play_launch process is never removed, wholly or in part: two
//! launches can share one log directory.

use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
};

/// A kind of file in a run, removable on its own (`log prune --class`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, clap::ValueEnum)]
pub enum ItemClass {
    /// `diagnostics.csv`.
    Diagnostics,
    /// `system_stats.csv` and every member's `metrics.csv`.
    Metrics,
    /// `interception/events.jsonl`, the per-message record.
    Events,
    /// Everything else under `interception/` (removing it removes `events`
    /// too).
    Interception,
    /// Each member's `out` and `err`, the earlier processes' `out.N`/`err.N`
    /// included.
    NodeLogs,
    /// `play_launch.log`, the launcher's own log.
    LauncherLog,
    /// Every `params_files/` directory.
    Params,
    /// Anything else: metadata, pids, cmdlines, the startup timeline.
    /// Listed, never removed by class.
    #[value(skip)]
    Other,
}

impl ItemClass {
    pub fn name(self) -> &'static str {
        match self {
            Self::Diagnostics => "diagnostics",
            Self::Metrics => "metrics",
            Self::Events => "events",
            Self::Interception => "interception",
            Self::NodeLogs => "node-logs",
            Self::LauncherLog => "launcher-log",
            Self::Params => "params",
            Self::Other => "other",
        }
    }

    /// Whether removing `self` removes a file of class `of`.
    fn covers(self, of: ItemClass) -> bool {
        self == of || (self == Self::Interception && of == Self::Events)
    }
}

/// The class of a file, from its path relative to the run.
pub fn class_of(rel: &Path) -> ItemClass {
    let parts: Vec<&str> = rel.iter().filter_map(|p| p.to_str()).collect();
    let file = parts.last().copied().unwrap_or("");
    if parts.contains(&"params_files") {
        return ItemClass::Params;
    }
    match parts.as_slice() {
        ["diagnostics.csv"] => ItemClass::Diagnostics,
        ["system_stats.csv"] => ItemClass::Metrics,
        ["play_launch.log"] => ItemClass::LauncherLog,
        ["interception", "events.jsonl"] => ItemClass::Events,
        ["interception", ..] => ItemClass::Interception,
        ["node" | "load_node", ..] if file == "metrics.csv" => ItemClass::Metrics,
        ["node" | "load_node", ..] if is_process_log(file) => ItemClass::NodeLogs,
        _ => ItemClass::Other,
    }
}

/// `out`, `err`, `out.N`, `err.N`.
fn is_process_log(file: &str) -> bool {
    let (stem, n) = file.split_once('.').unwrap_or((file, ""));
    matches!(stem, "out" | "err") && n.bytes().all(|c| c.is_ascii_digit())
}

/// One run directory.
#[derive(Debug, Clone)]
pub struct Run {
    pub path: PathBuf,
    /// The directory name, which is the run's local start time.
    pub name: String,
    /// Bytes per item class.
    pub sizes: BTreeMap<ItemClass, u64>,
    /// A live play_launch is writing it.
    pub in_use: bool,
}

impl Run {
    pub fn total(&self) -> u64 {
        self.sizes.values().sum()
    }
}

/// Every run under `base`, oldest first. `sized` walks each run for its
/// per-class sizes; without it `sizes` is empty.
pub fn list_runs(base: &Path, sized: bool) -> Vec<Run> {
    let Ok(entries) = fs::read_dir(base) else {
        return Vec::new();
    };
    let mut runs: Vec<Run> = entries
        .filter_map(|e| e.ok())
        .filter(|e| e.file_type().is_ok_and(|t| t.is_dir()))
        .filter_map(|e| {
            let name = e.file_name().into_string().ok()?;
            is_run_name(&name).then(|| {
                let path = e.path();
                let mut sizes = BTreeMap::new();
                if sized {
                    walk(&path, Path::new(""), &mut |rel, len| {
                        *sizes.entry(class_of(rel)).or_insert(0) += len;
                    });
                }
                Run {
                    in_use: in_use(&path),
                    path,
                    name,
                    sizes,
                }
            })
        })
        .collect();
    runs.sort_by(|a, b| a.name.cmp(&b.name));
    runs
}

fn walk(dir: &Path, rel: &Path, f: &mut dyn FnMut(&Path, u64)) {
    let Ok(entries) = fs::read_dir(dir) else {
        return;
    };
    for e in entries.filter_map(|e| e.ok()) {
        let rel = rel.join(e.file_name());
        match e.file_type() {
            Ok(t) if t.is_dir() => walk(&e.path(), &rel, f),
            Ok(t) if t.is_file() => f(&rel, e.metadata().map(|m| m.len()).unwrap_or(0)),
            _ => {}
        }
    }
}

/// Which runs a prune selects. Each limit selects independently and the
/// selection is their union; a run in use is never selected.
#[derive(Debug, Clone, Default)]
pub struct Selection {
    /// Runs that started before this run-name timestamp
    /// (`YYYY-MM-DD_HH-MM-SS`).
    pub before: Option<String>,
    /// Keep the newest this many runs.
    pub keep: Option<usize>,
    /// Keep the newest runs that fit in this many bytes together.
    pub max_total: Option<u64>,
}

impl Selection {
    pub fn is_empty(&self) -> bool {
        self.before.is_none() && self.keep.is_none() && self.max_total.is_none()
    }
}

/// The runs `sel` selects from `runs` (oldest first, sized when
/// `max_total` is set), oldest first.
pub fn select<'a>(runs: &'a [Run], sel: &Selection) -> Vec<&'a Run> {
    let free: Vec<&Run> = runs.iter().filter(|r| !r.in_use).collect();
    let mut chosen = vec![false; free.len()];
    if let Some(cut) = &sel.before {
        for (i, r) in free.iter().enumerate() {
            if r.name[..19] < cut[..] {
                chosen[i] = true;
            }
        }
    }
    if let Some(keep) = sel.keep {
        let excess = free.len().saturating_sub(keep);
        chosen[..excess].iter_mut().for_each(|c| *c = true);
    }
    if let Some(cap) = sel.max_total {
        // Newest first, keep while it fits.
        let mut kept = 0u64;
        for i in (0..free.len()).rev() {
            kept += free[i].total();
            if kept > cap {
                chosen[..=i].iter_mut().for_each(|c| *c = true);
                break;
            }
        }
    }
    free.into_iter()
        .zip(chosen)
        .filter_map(|(r, c)| c.then_some(r))
        .collect()
}

/// Remove `classes` from `run`, or the whole run when `classes` is empty.
/// Returns the bytes removed.
pub fn remove(run: &Run, classes: &[ItemClass]) -> std::io::Result<u64> {
    if in_use(&run.path) {
        return Ok(0);
    }
    if classes.is_empty() {
        let mut total = 0;
        walk(&run.path, Path::new(""), &mut |_, len| total += len);
        fs::remove_dir_all(&run.path)?;
        return Ok(total);
    }
    let mut doomed: Vec<(PathBuf, u64)> = Vec::new();
    walk(&run.path, Path::new(""), &mut |rel, len| {
        if classes.iter().any(|c| c.covers(class_of(rel))) {
            doomed.push((run.path.join(rel), len));
        }
    });
    let mut total = 0;
    for (p, len) in doomed {
        fs::remove_file(&p)?;
        total += len;
    }
    Ok(total)
}

/// Opt-in retention at the start of a run (`play_log.keep_runs`,
/// `play_log.max_total_mb`): remove the oldest earlier runs so that, with
/// the run about to be created, at most `keep_runs` remain and the earlier
/// ones total at most `max_total_mb`. Both `None` removes nothing, which is
/// the default. Returns the directories removed, oldest first.
pub fn prune_runs(
    base: &Path,
    keep_runs: Option<usize>,
    max_total_mb: Option<u64>,
) -> Vec<PathBuf> {
    if keep_runs.is_none() && max_total_mb.is_none() {
        return Vec::new();
    }
    let runs = list_runs(base, max_total_mb.is_some());
    let sel = Selection {
        before: None,
        // The new run counts toward `keep_runs`.
        keep: keep_runs.map(|k| k.saturating_sub(1)),
        max_total: max_total_mb.map(|mb| mb.saturating_mul(1_000_000)),
    };
    select(&runs, &sel)
        .into_iter()
        .filter(|r| match remove(r, &[]) {
            Ok(_) => true,
            Err(e) => {
                tracing::warn!("play_log: could not remove {}: {e}", r.path.display());
                false
            }
        })
        .map(|r| r.path.clone())
        .collect()
}

/// The reminder printed at the start of a run when `base` holds more than
/// `remind_mb` (`play_log.remind_mb`, 0 = never). Walks the directory, so
/// call it off the startup path.
pub fn reminder(base: &Path, remind_mb: u64) -> Option<String> {
    if remind_mb == 0 {
        return None;
    }
    let runs = list_runs(base, true);
    let total: u64 = runs.iter().map(Run::total).sum();
    if total <= remind_mb.saturating_mul(1_000_000) {
        return None;
    }
    let diag: u64 = runs
        .iter()
        .filter_map(|r| r.sizes.get(&ItemClass::Diagnostics))
        .sum();
    let hint = if diag * 2 > total {
        format!(
            ", {} of it diagnostics.csv (`play_launch log prune --class diagnostics --yes`)",
            human(diag)
        )
    } else {
        String::new()
    };
    Some(format!(
        "{} holds {} in {} run(s){hint}. `play_launch log list --log-dir {}` shows them; \
         `play_launch log prune --before 7d --yes` removes runs older than a week. \
         (`play_log.remind_mb: 0` silences this)",
        base.display(),
        human(total),
        runs.len(),
        base.display()
    ))
}

/// `1.2 GB`, `340 MB`, `12 kB`, decimal units.
pub fn human(bytes: u64) -> String {
    let b = bytes as f64;
    if b >= 1e9 {
        format!("{:.1} GB", b / 1e9)
    } else if b >= 1e6 {
        format!("{:.0} MB", b / 1e6)
    } else if b >= 1e3 {
        format!("{:.0} kB", b / 1e3)
    } else {
        format!("{bytes} B")
    }
}

/// `YYYY-MM-DD_HH-MM-SS`, optionally followed by `-N`.
pub fn is_run_name(name: &str) -> bool {
    let b = name.as_bytes();
    if b.len() < 19 {
        return false;
    }
    let shape = b"dddd-dd-dd_dd-dd-dd";
    let stamp_ok = shape.iter().zip(b).all(|(s, c)| match s {
        b'd' => c.is_ascii_digit(),
        s => s == c,
    });
    let suffix = &name[19..];
    stamp_ok
        && (suffix.is_empty()
            || suffix
                .strip_prefix('-')
                .is_some_and(|n| !n.is_empty() && n.bytes().all(|c| c.is_ascii_digit())))
}

/// Whether the run's `run_info.json` names a live play_launch process.
fn in_use(run: &Path) -> bool {
    let Ok(text) = fs::read_to_string(run.join("run_info.json")) else {
        return false;
    };
    let Some(pid) = serde_json::from_str::<serde_json::Value>(&text)
        .ok()
        .and_then(|v| v["pid"].as_u64())
    else {
        return false;
    };
    if pid == std::process::id() as u64 {
        return true;
    }
    fs::read_to_string(format!("/proc/{pid}/comm"))
        .is_ok_and(|c| c.trim_end().starts_with("play_launch"))
}

/// A `--before` value as a run-name timestamp: `YYYY-MM-DD`,
/// `YYYY-MM-DD_HH-MM-SS`, or an age (`7d`, `12h`, `30m`, `2w`) back from
/// `now`.
pub fn parse_before(s: &str, now: chrono::DateTime<chrono::Local>) -> Result<String, String> {
    if s.len() == 10 && is_run_name(&format!("{s}_00-00-00")) {
        return Ok(format!("{s}_00-00-00"));
    }
    if s.len() == 19 && is_run_name(s) {
        return Ok(s.to_string());
    }
    let (n, unit) = s.split_at(s.len().saturating_sub(1));
    let n: i64 = n.parse().map_err(|_| {
        format!("`{s}`: expected YYYY-MM-DD, YYYY-MM-DD_HH-MM-SS or an age like 7d")
    })?;
    let secs = match unit {
        "m" => 60,
        "h" => 3600,
        "d" => 86_400,
        "w" => 7 * 86_400,
        _ => return Err(format!("`{s}`: an age ends in m, h, d or w")),
    };
    let cut = now - chrono::Duration::seconds(n.saturating_mul(secs));
    Ok(cut.format("%Y-%m-%d_%H-%M-%S").to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(base: &Path, name: &str, files: &[(&str, usize)]) {
        for (rel, bytes) in files {
            let p = base.join(name).join(rel);
            fs::create_dir_all(p.parent().unwrap()).unwrap();
            fs::write(p, vec![b'x'; *bytes]).unwrap();
        }
        fs::create_dir_all(base.join(name)).unwrap();
    }

    fn left(base: &Path) -> Vec<String> {
        let mut v: Vec<String> = fs::read_dir(base)
            .unwrap()
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .collect();
        v.sort();
        v
    }

    #[test]
    fn classes() {
        for (p, c) in [
            ("diagnostics.csv", ItemClass::Diagnostics),
            ("system_stats.csv", ItemClass::Metrics),
            ("node/a/metrics.csv", ItemClass::Metrics),
            ("load_node/b/metrics.csv", ItemClass::Metrics),
            ("node/a/out", ItemClass::NodeLogs),
            ("node/a/err.3", ItemClass::NodeLogs),
            ("load_node/b/out", ItemClass::NodeLogs),
            ("node/a/outline", ItemClass::Other),
            ("node/a/pid", ItemClass::Other),
            ("node/a/params_files/x.yaml", ItemClass::Params),
            ("params_files/y.yaml", ItemClass::Params),
            ("interception/events.jsonl", ItemClass::Events),
            ("interception/stats_summary.json", ItemClass::Interception),
            ("play_launch.log", ItemClass::LauncherLog),
            ("health.jsonl", ItemClass::Other),
        ] {
            assert_eq!(class_of(Path::new(p)), c, "{p}");
        }
    }

    #[test]
    fn nothing_is_removed_by_default() {
        let t = tempfile::tempdir().unwrap();
        run(t.path(), "2026-01-01_00-00-00", &[("out", 10)]);
        assert!(prune_runs(t.path(), None, None).is_empty());
        assert_eq!(left(t.path()).len(), 1);
    }

    /// `keep_runs` counts the run about to start; only run-shaped
    /// directories are candidates, and a symlink never is.
    #[test]
    fn keep_runs_removes_the_oldest_runs_only() {
        let t = tempfile::tempdir().unwrap();
        for n in [
            "2026-01-01_00-00-00",
            "2026-01-01_00-00-00-1",
            "2026-01-02_00-00-00",
            "2026-01-03_00-00-00",
        ] {
            run(t.path(), n, &[("out", 10)]);
        }
        fs::create_dir(t.path().join("notes")).unwrap();
        std::os::unix::fs::symlink("2026-01-03_00-00-00", t.path().join("latest")).unwrap();
        assert_eq!(prune_runs(t.path(), Some(3), None).len(), 2);
        assert_eq!(
            left(t.path()),
            vec![
                "2026-01-02_00-00-00",
                "2026-01-03_00-00-00",
                "latest",
                "notes"
            ]
        );
    }

    #[test]
    fn max_total_mb_removes_oldest_until_under_the_cap() {
        let t = tempfile::tempdir().unwrap();
        run(t.path(), "2026-01-01_00-00-00", &[("out", 600_000)]);
        run(t.path(), "2026-01-02_00-00-00", &[("out", 600_000)]);
        run(t.path(), "2026-01-03_00-00-00", &[("out", 300_000)]);
        prune_runs(t.path(), None, Some(1));
        assert_eq!(
            left(t.path()),
            vec!["2026-01-02_00-00-00", "2026-01-03_00-00-00"]
        );
    }

    /// A run another live play_launch is writing is kept whatever the
    /// limit, and its files are not removed by class either.
    #[test]
    fn a_run_in_use_is_never_touched() {
        let t = tempfile::tempdir().unwrap();
        run(t.path(), "2026-01-01_00-00-00", &[("diagnostics.csv", 10)]);
        fs::write(
            t.path().join("2026-01-01_00-00-00/run_info.json"),
            format!("{{\"pid\":{}}}", std::process::id()),
        )
        .unwrap();
        assert!(prune_runs(t.path(), Some(1), Some(0)).is_empty());
        let runs = list_runs(t.path(), true);
        assert!(runs[0].in_use);
        assert_eq!(remove(&runs[0], &[ItemClass::Diagnostics]).unwrap(), 0);
        assert!(
            t.path()
                .join("2026-01-01_00-00-00/diagnostics.csv")
                .exists()
        );
    }

    /// `--before` selects by start time; `--class` removes only those files
    /// and keeps the run; `interception` covers `events`.
    #[test]
    fn prune_by_date_and_by_class() {
        let t = tempfile::tempdir().unwrap();
        let files = [
            ("diagnostics.csv", 100),
            ("node/a/out", 10),
            ("node/a/metrics.csv", 5),
            ("interception/events.jsonl", 50),
            ("interception/stats_summary.json", 1),
        ];
        run(t.path(), "2026-01-01_00-00-00", &files);
        run(t.path(), "2026-01-09_00-00-00", &files);
        let runs = list_runs(t.path(), true);
        assert_eq!(runs[0].sizes[&ItemClass::Diagnostics], 100);
        assert_eq!(runs[0].sizes[&ItemClass::Events], 50);
        let sel = Selection {
            before: Some("2026-01-05_00-00-00".into()),
            ..Default::default()
        };
        let chosen = select(&runs, &sel);
        assert_eq!(chosen.len(), 1);
        assert_eq!(chosen[0].name, "2026-01-01_00-00-00");
        let freed = remove(
            chosen[0],
            &[ItemClass::Diagnostics, ItemClass::Interception],
        )
        .unwrap();
        assert_eq!(freed, 151);
        let after = list_runs(t.path(), true);
        let classes: Vec<_> = after[0].sizes.keys().copied().collect();
        assert_eq!(classes, vec![ItemClass::Metrics, ItemClass::NodeLogs]);
        assert_eq!(after[1].total(), 166);
    }

    #[test]
    fn before_values() {
        use chrono::TimeZone;
        let now = chrono::Local
            .with_ymd_and_hms(2026, 11, 18, 12, 0, 0)
            .unwrap();
        assert_eq!(
            parse_before("2026-11-01", now).unwrap(),
            "2026-11-01_00-00-00"
        );
        assert_eq!(
            parse_before("2026-11-01_08-30-00", now).unwrap(),
            "2026-11-01_08-30-00"
        );
        assert_eq!(parse_before("7d", now).unwrap(), "2026-11-11_12-00-00");
        assert_eq!(parse_before("12h", now).unwrap(), "2026-11-18_00-00-00");
        assert!(parse_before("7x", now).is_err());
        assert!(parse_before("yesterday", now).is_err());
    }

    #[test]
    fn the_reminder_names_the_size_and_the_command() {
        let t = tempfile::tempdir().unwrap();
        run(
            t.path(),
            "2026-01-01_00-00-00",
            &[("diagnostics.csv", 3_000_000)],
        );
        assert!(reminder(t.path(), 5).is_none());
        assert!(reminder(t.path(), 0).is_none());
        let m = reminder(t.path(), 1).unwrap();
        assert!(m.contains("holds 3 MB in 1 run(s)"), "{m}");
        assert!(m.contains("--class diagnostics"), "{m}");
    }

    #[test]
    fn run_names() {
        assert!(is_run_name("2026-11-18_00-39-23"));
        assert!(is_run_name("2026-11-18_00-39-23-12"));
        assert!(!is_run_name("latest"));
        assert!(!is_run_name("2026-11-18_00-39-23-"));
        assert!(!is_run_name("2026-11-18_00-39-23x"));
    }
}
