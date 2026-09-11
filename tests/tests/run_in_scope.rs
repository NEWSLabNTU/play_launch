//! Issue #0024 — `play_launch run` with its DEFAULT features must start its
//! node, in a plain shell and inside a `systemd-run --user --scope`.
//!
//! The report was `Unable to start: Operation not permitted (os error 1)`
//! with an empty node log, inside a scope, while `launch` in another scope
//! was fine. The scope was incidental. `run` wired its web server to a
//! throwaway shutdown channel whose sender died at once, so the server
//! logged `Web server shutting down...` a millisecond after starting, the
//! main loop read a finished background task as the end of the run,
//! shutdown reaped the anchor zombie that holds the run's process group,
//! and the node still being spawned failed `setpgid(0, pgid)` in the child
//! — a race the reporter's loaded machine lost every time and an idle one
//! wins almost every time, which is why every earlier `run` test here
//! passed: they all use `--disable-all`, and without the web UI the run
//! never ended early.
//!
//! So these tests run with the defaults ON (the web UI on an ephemeral
//! port, so concurrent tests cannot collide) and read the launcher's log
//! BEFORE stopping it, because a `Web server shutting down...` after our
//! SIGTERM is the correct one.

use play_launch_tests::fixtures;
use play_launch_tests::process::ManagedProcess;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::{Duration, Instant};
use tempfile::TempDir;

static UNIT_SEQ: AtomicU32 = AtomicU32::new(0);

/// Whether this host can put a process in a transient user scope at all.
fn user_scope_available() -> bool {
    Command::new("systemd-run")
        .args(["--user", "--scope", "--quiet", "--collect", "--", "true"])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// `play_launch run demo_nodes_cpp talker` with the default feature set,
/// optionally wrapped in a transient user scope. The environment is exactly
/// what `fixtures::play_launch_cmd` would give the bare binary — copied
/// onto whichever program actually starts.
fn talker_cmd(work: &Path, in_scope: bool) -> Command {
    let env = fixtures::install_env();
    let base = fixtures::play_launch_cmd(&env);
    let bin = fixtures::play_launch_bin();

    let mut cmd = if in_scope {
        let unit = format!(
            "play-launch-test-{}-{}",
            std::process::id(),
            UNIT_SEQ.fetch_add(1, Ordering::Relaxed)
        );
        let mut c = Command::new("systemd-run");
        c.args(["--user", "--scope", "--quiet", "--collect", "--unit", &unit, "--"]);
        c.arg(&bin);
        c
    } else {
        Command::new(&bin)
    };
    cmd.env_clear();
    for (k, v) in base.get_envs() {
        if let Some(v) = v {
            cmd.env(k, v);
        }
    }
    cmd.current_dir(work);
    // Defaults ON. Port 0 gives the web UI an ephemeral port: the fixed
    // 8080 would collide with any other play_launch on the host, and a
    // bind failure is just another way for the web task to end early.
    cmd.args(["run", "demo_nodes_cpp", "talker", "--web-addr", "127.0.0.1:0"]);
    // `run` writes its log to STDOUT (`main.rs::logs_to_stderr` sends only
    // the model-emitting verbs to stderr), so both streams go to one file.
    let log = std::fs::File::create(work.join("launcher.log")).unwrap();
    cmd.stdin(Stdio::null());
    cmd.stdout(log.try_clone().unwrap());
    cmd.stderr(log);
    cmd
}

/// The talker's `err` file once it exists, wherever `run` put the log dir.
fn talker_err(work: &Path) -> Option<PathBuf> {
    let runs = std::fs::read_dir(work.join("play_log")).ok()?;
    runs.flatten()
        .map(|e| e.path().join("node/talker/err"))
        .find(|p| p.is_file())
}

/// Start the run, wait until the talker publishes, and return the
/// launcher's stderr as it stood at that moment (before shutdown).
fn run_until_talker_publishes(in_scope: bool) -> (String, String) {
    let work = TempDir::new().expect("tempdir");
    let mut cmd = talker_cmd(work.path(), in_scope);
    let proc = ManagedProcess::spawn(&mut cmd).expect("failed to start play_launch");

    let deadline = Instant::now() + Duration::from_secs(30);
    let mut node_err = String::new();
    while Instant::now() < deadline {
        if let Some(p) = talker_err(work.path()) {
            node_err = std::fs::read_to_string(&p).unwrap_or_default();
            if node_err.contains("Publishing:") {
                break;
            }
        }
        // A failed spawn ends the run; stop waiting on it.
        let launcher = std::fs::read_to_string(work.path().join("launcher.log")).unwrap_or_default();
        if launcher.contains("Unable to start") {
            break;
        }
        std::thread::sleep(Duration::from_millis(250));
    }
    let launcher = std::fs::read_to_string(work.path().join("launcher.log")).unwrap_or_default();
    // Graceful stop: the guard SIGTERMs the whole group, then reaps.
    drop(proc);
    (launcher, node_err)
}

fn assert_run_healthy(launcher: &str, node_err: &str, where_: &str) {
    assert!(
        node_err.contains("Publishing:"),
        "{where_}: the talker never published.\nnode err: {node_err}\nlauncher log: {launcher}"
    );
    assert!(
        !launcher.contains("Unable to start"),
        "{where_}: the node failed to spawn.\nlauncher log: {launcher}"
    );
    assert!(
        !launcher.contains("Web server shutting down"),
        "{where_}: the web server ended before the run did — the run.rs wiring of #0024 is back.\n\
         launcher log: {launcher}"
    );
    assert!(
        !launcher.contains("Background task finished early"),
        "{where_}: a background task ended while the node was running.\nlauncher log: {launcher}"
    );
}

/// The plain-shell half: with the defaults on, `run` keeps its web UI and
/// its node alive. This is the direct regression for the wiring, and it
/// needs no scope to fail on the old binary.
#[test]
fn run_with_default_features_keeps_web_ui_and_node_alive() {
    let (launcher, node_err) = run_until_talker_publishes(false);
    assert_run_healthy(&launcher, &node_err, "plain shell");
}

/// The reporter's setup: `run` inside `systemd-run --user --scope`.
#[test]
fn run_starts_its_node_inside_a_user_scope() {
    if !user_scope_available() {
        eprintln!(
            "SKIP: run_starts_its_node_inside_a_user_scope: \
             `systemd-run --user --scope -- true` does not work on this host"
        );
        return;
    }
    let (launcher, node_err) = run_until_talker_publishes(true);
    assert_run_healthy(&launcher, &node_err, "inside a user scope");
}
