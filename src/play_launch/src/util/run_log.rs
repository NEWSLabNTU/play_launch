//! The launcher's own log, written into the run's `log_dir` (issue #0023).
//!
//! A bundle from a failed launch used to hold every node's `out`/`err`,
//! `metrics.csv`, `diagnostics.csv` — and nothing from play_launch itself.
//! The one process that decided what to spawn, what to load, and what to
//! give up on wrote its account only to the terminal, which is gone by the
//! time anyone asks. Six lost composable loads on the golf cart were
//! undiagnosable for exactly that reason.
//!
//! Two constraints shape this module:
//!
//! - **The subscriber is installed before the directory exists.** `launch`
//!   parses the launch file (seconds, with warnings worth keeping) before
//!   `play()` creates `play_log/<ts>/`; `up` loads and validates the model
//!   first. Creating the directory earlier would move `create_log_dir` out of
//!   the shared engine into three callers and leave an empty timestamped
//!   directory plus a moved `latest` symlink behind every failed parse. So the
//!   file layer is registered at startup with a writer that BUFFERS until
//!   [`attach`] hands it the directory, then drains the buffer into the file
//!   ahead of everything that follows. Nothing emitted before the directory
//!   existed is lost.
//! - **It must never take a launch down.** A run whose log could not be
//!   opened is a run with a warning, not a failed run; every error here is
//!   reported once and swallowed.
//!
//! The file takes `debug` from this crate and the resolver regardless of the
//! terminal's `RUST_LOG` (see [`file_filter`] for the measurement behind
//! that), so a bundle carries the detail a reproduction would have needed —
//! without the terminal having to be noisy on the happy path.

use serde::Serialize;
use std::{
    fs::File,
    io::{self, Write},
    path::{Path, PathBuf},
    sync::{Mutex, OnceLock},
};
use tracing_subscriber::fmt::MakeWriter;

/// Name of the launcher's log inside `play_log/<ts>/`.
pub const LOG_FILE_NAME: &str = "play_launch.log";
/// Name of the run-identity record beside it.
pub const RUN_INFO_FILE_NAME: &str = "run_info.json";
/// Where a `--config` file is copied to, so the bundle can be re-run with the
/// same runtime configuration.
pub const CONFIG_COPY_FILE_NAME: &str = "config.yaml";

/// Environment variable that overrides the file's level filter, with the
/// same syntax as `RUST_LOG`. The terminal keeps `RUST_LOG`; the two are
/// independent so a quiet terminal can still produce a detailed bundle.
pub const FILE_FILTER_ENV: &str = "PLAY_LAUNCH_LOG_FILE";

/// Default filter for the file — see [`file_filter`].
pub const DEFAULT_FILE_FILTER: &str = "play_launch=debug,ros_launch_resolve=debug,info";

/// How much may accumulate before the directory exists. The parse phase of
/// the largest launch measured (Autoware, 119 nodes) produces well under
/// 100 KiB at debug; anything past this cap is counted and reported at
/// attach time rather than growing without bound in a run that never
/// creates its directory.
const PRE_ATTACH_CAP_BYTES: usize = 4 * 1024 * 1024;

enum State {
    /// The directory does not exist yet: hold every line.
    Buffering { buf: Vec<u8>, dropped_bytes: usize },
    /// Lines go straight to the file.
    Attached(File),
    /// The file could not be opened (reported once); discard.
    Disabled,
}

struct RunLog {
    state: Mutex<State>,
}

static RUN_LOG: OnceLock<RunLog> = OnceLock::new();

fn run_log() -> &'static RunLog {
    RUN_LOG.get_or_init(|| RunLog {
        state: Mutex::new(State::Buffering {
            buf: Vec::new(),
            dropped_bytes: 0,
        }),
    })
}

/// `MakeWriter` for the file layer: every event is written through the
/// global [`RunLog`], whichever state it is in.
#[derive(Debug, Clone, Copy, Default)]
pub struct RunLogWriter;

/// One event's writer. Holds no lock across events: the lock is taken per
/// `write`, and `tracing_subscriber::fmt` writes a formatted event in ONE
/// `write_all`, so a line is never interleaved with another thread's.
pub struct RunLogHandle;

impl Write for RunLogHandle {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        let log = run_log();
        let mut state = log.state.lock().unwrap_or_else(|e| e.into_inner());
        match &mut *state {
            State::Buffering { buf, dropped_bytes } => {
                if buf.len() + bytes.len() <= PRE_ATTACH_CAP_BYTES {
                    buf.extend_from_slice(bytes);
                } else {
                    *dropped_bytes += bytes.len();
                }
            }
            State::Attached(file) => {
                // A write failure (disk full, directory removed under us)
                // must not recurse into tracing from inside a tracing
                // writer; it is silently dropped and the file simply ends.
                if file.write_all(bytes).is_err() {
                    *state = State::Disabled;
                }
            }
            State::Disabled => {}
        }
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl<'a> MakeWriter<'a> for RunLogWriter {
    type Writer = RunLogHandle;

    fn make_writer(&'a self) -> Self::Writer {
        RunLogHandle
    }
}

/// The file layer's filter: [`FILE_FILTER_ENV`] if set, else
/// [`DEFAULT_FILE_FILTER`].
///
/// Why `debug`, and why crate-scoped. Measured on the `container_events`
/// fixture (one container, two composables, 20 s, web UI + monitoring +
/// diagnostics all on): INFO alone is ~21 lines; `play_launch=debug` is 174
/// lines / 30 KB; a global `debug` is 179 lines / 31 KB. Debug is ~8x the
/// lines of INFO and still tens of kilobytes for a run, which is nothing
/// beside the per-node `out`/`err` already in the bundle — and it is the
/// level at which the load path narrates every dispatch, acceptance and
/// timeout, which is what #0023 needed and did not have. The five extra
/// lines under a global `debug` are the whole third-party contribution today
/// (`hyper`, `tower`, `rclrs` only speak at `trace`, or under request load
/// the launcher never generates), so scoping costs nothing now; it is chosen
/// so that a future chatty dependency cannot turn the file into its own log,
/// and because `play_launch=debug` is what every integration test already
/// asks for on the terminal. `EnvFilter` matches targets by string prefix, so
/// `play_launch=` also covers `play_launch_parser` and
/// `play_launch_parser_pyload` (12 and 1 of the lines above).
pub fn file_filter() -> tracing_subscriber::EnvFilter {
    let spec = std::env::var(FILE_FILTER_ENV).unwrap_or_else(|_| DEFAULT_FILE_FILTER.to_string());
    tracing_subscriber::EnvFilter::try_new(&spec).unwrap_or_else(|e| {
        // The subscriber is not up yet, so this cannot be a `warn!`.
        eprintln!(
            "warning: {FILE_FILTER_ENV}={spec:?} is not a valid filter ({e}); using {DEFAULT_FILE_FILTER:?}"
        );
        tracing_subscriber::EnvFilter::new(DEFAULT_FILE_FILTER)
    })
}

/// What identifies a run — written as the log's header and as
/// [`RUN_INFO_FILE_NAME`], because the version of play_launch that produced
/// a bundle was the second thing #0023 could not answer.
#[derive(Debug, Clone, Serialize)]
pub struct RunInfo {
    pub version: String,
    pub started_at: String,
    pub pid: u32,
    pub cwd: Option<PathBuf>,
    pub argv: Vec<String>,
    /// `--config` as given on the command line, if any.
    pub config_path: Option<PathBuf>,
    /// Where that file was copied inside the bundle, if it could be.
    pub config_copy: Option<String>,
    /// The filter the log file was written under.
    pub log_filter: String,
}

impl RunInfo {
    /// Capture the current process. `config_path` is the `--config` argument
    /// the verb received (all three run verbs carry one).
    pub fn capture(config_path: Option<&Path>) -> Self {
        Self {
            version: env!("CARGO_PKG_VERSION").to_string(),
            // UTC, because that is the clock every tracing line below it is
            // stamped with; a local-time header beside UTC lines makes the
            // reader do arithmetic to correlate them.
            started_at: chrono::Utc::now().to_rfc3339(),
            pid: std::process::id(),
            cwd: std::env::current_dir().ok(),
            argv: std::env::args().collect(),
            config_path: config_path.map(Path::to_path_buf),
            config_copy: None,
            log_filter: std::env::var(FILE_FILTER_ENV)
                .unwrap_or_else(|_| DEFAULT_FILE_FILTER.to_string()),
        }
    }

    fn header(&self) -> String {
        let mut s = String::new();
        s.push_str(&format!("# play_launch {} — launcher log\n", self.version));
        s.push_str(&format!("# started: {}\n", self.started_at));
        s.push_str(&format!("# pid: {}\n", self.pid));
        if let Some(cwd) = &self.cwd {
            s.push_str(&format!("# cwd: {}\n", cwd.display()));
        }
        s.push_str(&format!("# argv: {}\n", shell_join(&self.argv)));
        match (&self.config_path, &self.config_copy) {
            (Some(path), Some(copy)) => s.push_str(&format!(
                "# config: {} (copied to {copy})\n",
                path.display()
            )),
            (Some(path), None) => s.push_str(&format!("# config: {}\n", path.display())),
            (None, _) => s.push_str("# config: none (defaults)\n"),
        }
        s.push_str(&format!("# filter: {}\n", self.log_filter));
        s
    }
}

/// Quote argv the way a shell would need it, so the header line can be
/// pasted back.
fn shell_join(argv: &[String]) -> String {
    argv.iter()
        .map(|a| {
            if a.is_empty()
                || a.chars()
                    .any(|c| c.is_whitespace() || matches!(c, '\'' | '"' | '$' | '`' | '\\'))
            {
                format!("'{}'", a.replace('\'', "'\\''"))
            } else {
                a.clone()
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}

/// Open `<log_dir>/play_launch.log`, write the header, drain everything
/// logged so far into it, and write `run_info.json` (copying the `--config`
/// file beside it when there is one). Returns the log path.
///
/// Idempotent in effect: a second call while attached replaces nothing and
/// returns the existing path's sibling — but no verb calls it twice.
pub fn attach(log_dir: &Path, mut info: RunInfo) -> io::Result<PathBuf> {
    let log_path = log_dir.join(LOG_FILE_NAME);

    // The config copy is best-effort and recorded in the header, so it is
    // done before the header is written.
    if let Some(path) = info.config_path.as_deref() {
        match std::fs::copy(path, log_dir.join(CONFIG_COPY_FILE_NAME)) {
            Ok(_) => info.config_copy = Some(CONFIG_COPY_FILE_NAME.to_string()),
            Err(e) => tracing::warn!(
                "could not copy {} into the log directory: {}",
                path.display(),
                e
            ),
        }
    }

    let mut file = File::create(&log_path)?;
    file.write_all(info.header().as_bytes())?;

    {
        let log = run_log();
        let mut state = log.state.lock().unwrap_or_else(|e| e.into_inner());
        if let State::Buffering { buf, dropped_bytes } = &mut *state {
            file.write_all(buf)?;
            if *dropped_bytes > 0 {
                file.write_all(
                    format!(
                        "# NOTE: {} bytes logged before this directory existed exceeded the \
                         {} byte buffer and were dropped\n",
                        dropped_bytes, PRE_ATTACH_CAP_BYTES
                    )
                    .as_bytes(),
                )?;
            }
        }
        *state = State::Attached(file);
    }

    let info_path = log_dir.join(RUN_INFO_FILE_NAME);
    if let Err(e) = serde_json::to_string_pretty(&info)
        .map_err(io::Error::other)
        .and_then(|json| std::fs::write(&info_path, json))
    {
        tracing::warn!("could not write {}: {}", info_path.display(), e);
    }

    Ok(log_path)
}

/// [`attach`], reporting rather than propagating: the log directory exists
/// and the launch proceeds whether or not its own log could be opened.
pub fn attach_or_warn(log_dir: &Path, info: RunInfo) {
    match attach(log_dir, info) {
        Ok(path) => tracing::info!("Launcher log: {}", path.display()),
        Err(e) => {
            // Nothing written before this point can reach a file now; say so
            // once and stop buffering, so a long run does not hold a buffer
            // it will never drain.
            let log = run_log();
            *log.state.lock().unwrap_or_else(|e| e.into_inner()) = State::Disabled;
            tracing::warn!(
                "could not open {} in {} ({}); this run's launcher log is the terminal only",
                LOG_FILE_NAME,
                log_dir.display(),
                e
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // The global is process-wide, so these tests share it and run in one
    // thread of control: `serial` by construction (a single test), not by
    // attribute.
    #[test]
    fn buffered_lines_land_in_the_file_after_the_header_in_order() {
        let dir = tempfile::tempdir().unwrap();
        let mut h = RunLogHandle;
        h.write_all(b"first line\n").unwrap();
        h.write_all(b"second line\n").unwrap();

        let info = RunInfo {
            version: "9.9.9-test".into(),
            started_at: "2026-09-11T00:00:00+00:00".into(),
            pid: 4242,
            cwd: None,
            argv: vec!["play_launch".into(), "launch".into(), "a b".into()],
            config_path: None,
            config_copy: None,
            log_filter: DEFAULT_FILE_FILTER.into(),
        };
        let path = attach(dir.path(), info).unwrap();
        assert_eq!(path, dir.path().join(LOG_FILE_NAME));

        // A line written AFTER attach goes straight through.
        h.write_all(b"third line\n").unwrap();

        let text = std::fs::read_to_string(&path).unwrap();
        let header_end = text.find("first line").expect("buffered line present");
        let header = &text[..header_end];
        assert!(header.starts_with("# play_launch 9.9.9-test"), "{header}");
        assert!(header.contains("# pid: 4242"), "{header}");
        assert!(
            header.contains("# argv: play_launch launch 'a b'"),
            "{header}"
        );
        assert!(header.contains("# config: none"), "{header}");
        let first = text.find("first line").unwrap();
        let second = text.find("second line").unwrap();
        let third = text.find("third line").unwrap();
        assert!(first < second && second < third, "{text}");

        let info: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(dir.path().join(RUN_INFO_FILE_NAME)).unwrap(),
        )
        .unwrap();
        assert_eq!(info["version"], "9.9.9-test");
        assert_eq!(info["argv"][2], "a b");
    }

    #[test]
    fn shell_join_quotes_only_what_needs_it() {
        let argv = vec![
            "play_launch".to_string(),
            "up".to_string(),
            "m.yaml".to_string(),
            "arg:=it's".to_string(),
            String::new(),
        ];
        assert_eq!(
            shell_join(&argv),
            "play_launch up m.yaml 'arg:=it'\\''s' ''"
        );
    }

    #[test]
    fn default_filter_parses() {
        tracing_subscriber::EnvFilter::try_new(DEFAULT_FILE_FILTER).unwrap();
    }
}
