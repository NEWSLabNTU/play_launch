//! How a launch ends, against stock `ros2 launch` (#0084, #0090).
//!
//! - #0084: on shutdown each process gets SIGINT; if it is still running its
//!   action's `sigterm_timeout` later (default 5 s), SIGTERM; and
//!   `sigkill_timeout` after that (default 5 s), SIGKILL. play_launch sent
//!   SIGTERM first, stopped the process "directly" 2 s later and SIGKILLed it
//!   5 s after that, whatever the launch file declared.
//! - #0090: an `OnProcessExit` handler whose `on_exit` is a FUNCTION of the
//!   exit event, returning a shutdown for exit code 2 only. `ros2 launch`
//!   ends the launch on that exit; play_launch ignored the handler and the
//!   process respawned forever. On another code it must still respawn.
//!
//! Each launcher is started in its own session (`ManagedProcess`) and is
//! stopped the way a terminal's Ctrl-C stops it: SIGINT to its whole process
//! group. `ros2 launch` then sends no SIGINT of its own (the terminal did);
//! play_launch's processes are in a group of their own, so it forwards one.
//! The exit is observed without `ManagedProcess`'s escalating cleanup, which
//! would SIGKILL at 2 s and hide the very ladder under test (see
//! `run_signal.rs`).

use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    process::{Command, Stdio},
    time::{Duration, Instant},
};

use play_launch_tests::{fixtures, process::ManagedProcess};

fn fixture(name: &str) -> PathBuf {
    fixtures::repo_root()
        .join("tests/fixtures/shutdown")
        .join(name)
}

fn ros2(env: &HashMap<String, String>) -> Option<PathBuf> {
    env.get("PATH").and_then(|path| {
        std::env::split_paths(path)
            .map(|p| p.join("ros2"))
            .find(|p| p.is_file())
    })
}

/// How to start the system under test.
#[derive(Clone, Copy, Debug)]
enum Launcher {
    Stock,
    PlayLaunch(&'static str),
    /// `resolve --parser rust -o m.yaml`, then `up m.yaml`.
    ResolveThenUp,
}

/// A started launcher, with where its processes write and its console log.
struct Run {
    proc: ManagedProcess,
    log: PathBuf,
    started: Instant,
}

fn start(launcher: Launcher, file: &Path, args: &[&str], out: &Path) -> Option<Run> {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("SKIP: shutdown_semantics: ROS env not available");
        return None;
    }
    std::fs::create_dir_all(out).unwrap();
    let mut cmd = match launcher {
        Launcher::Stock => {
            let Some(ros2) = ros2(&env) else {
                eprintln!("SKIP: shutdown_semantics: `ros2` is not on the ROS PATH");
                return None;
            };
            let mut cmd = Command::new(ros2);
            fixtures::apply_test_env(&mut cmd, &env);
            cmd.arg("launch").arg(file).args(args);
            cmd
        }
        Launcher::PlayLaunch(parser) => {
            let mut cmd = fixtures::play_launch_cmd(&env);
            cmd.args(["launch", "--parser", parser])
                .arg(file)
                .args(args);
            cmd
        }
        Launcher::ResolveThenUp => {
            let model = out.join("model.yaml");
            let mut resolve = fixtures::play_launch_cmd(&env);
            resolve
                .env("PL_PROBE_OUT", out)
                .args(["resolve", "--parser", "rust"])
                .arg(file)
                .args(args)
                .arg("-o")
                .arg(&model);
            let o = resolve.output().expect("run resolve");
            assert!(
                o.status.success(),
                "resolve failed:\n{}",
                String::from_utf8_lossy(&o.stderr)
            );
            let mut cmd = fixtures::play_launch_cmd(&env);
            cmd.arg("up").arg(&model);
            cmd
        }
    };
    if !matches!(launcher, Launcher::Stock) {
        cmd.args([
            "--disable-web-ui",
            "--disable-monitoring",
            "--disable-diagnostics",
        ]);
    }
    let log = out.join("console.log");
    cmd.current_dir(out)
        .env("PL_PROBE_OUT", out)
        .env("ROS_LOG_DIR", out.join("roslog"))
        .env("RUST_LOG", "play_launch=debug")
        .stdout(Stdio::from(std::fs::File::create(&log).unwrap()))
        .stderr(Stdio::from(
            std::fs::File::create(out.join("console.err")).unwrap(),
        ));
    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn launcher");
    Some(Run {
        proc,
        log,
        started: Instant::now(),
    })
}

impl Run {
    fn console(&self) -> String {
        let text = std::fs::read_to_string(&self.log).unwrap_or_default()
            + &std::fs::read_to_string(self.log.with_file_name("console.err")).unwrap_or_default();
        text[text.len().saturating_sub(4000)..].to_string()
    }

    /// Wait for the launcher to exit by itself, up to `deadline` from start.
    fn exit_within(&mut self, deadline: Duration) -> Option<Duration> {
        loop {
            if self.proc.try_wait().is_some() {
                return Some(self.started.elapsed());
            }
            if self.started.elapsed() > deadline {
                return None;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    /// A terminal's Ctrl-C: SIGINT to the launcher's process group.
    fn ctrl_c(&self) {
        unsafe {
            libc::kill(-(self.proc.id() as i32), libc::SIGINT);
        }
    }
}

/// The probe's events, as seconds after its START.
fn events(path: &Path) -> Vec<(String, f64)> {
    let text = std::fs::read_to_string(path).unwrap_or_default();
    let mut start = None;
    let mut out = Vec::new();
    for line in text.lines() {
        let mut parts = line.split_whitespace();
        let (Some(name), Some(t)) = (parts.next(), parts.next()) else {
            continue;
        };
        let t: f64 = t.parse().unwrap();
        if name == "START" {
            start = Some(t);
        } else if let Some(s) = start {
            out.push((name.to_string(), t - s));
        }
    }
    out
}

fn wait_for(path: &Path, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if std::fs::read_to_string(path).is_ok_and(|t| t.contains("START")) {
            return true;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    false
}

/// What a Ctrl-C does to the two probes, and when the launcher exits, in
/// seconds after the Ctrl-C.
struct Ladder {
    /// Each probe's signals: (name, seconds after the Ctrl-C).
    signals: HashMap<&'static str, Vec<(String, f64)>>,
    exit_after: f64,
}

fn ladder(launcher: Launcher) -> Option<Ladder> {
    let tmp = tempfile::TempDir::new().unwrap();
    let out = tmp.path().canonicalize().unwrap();
    let mut run = start(launcher, &fixture("timeouts.launch.py"), &[], &out)?;
    for name in ["declared", "default"] {
        assert!(
            wait_for(&out.join(name), Duration::from_secs(30)),
            "{launcher:?}: probe {name} never started:\n{}",
            run.console()
        );
    }
    // Discovery and output settle; then the Ctrl-C, timed against the probes'
    // own START lines, which are on the same clock as `time.monotonic()`.
    std::thread::sleep(Duration::from_secs(2));
    let mono_now = {
        let mut ts = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        ts.tv_sec as f64 + ts.tv_nsec as f64 / 1e9
    };
    let ctrl_c_at = Instant::now();
    run.ctrl_c();
    let mut exit_after = None;
    while ctrl_c_at.elapsed() < Duration::from_secs(25) {
        if run.proc.try_wait().is_some() {
            exit_after = Some(ctrl_c_at.elapsed().as_secs_f64());
            break;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    let exit_after = exit_after.unwrap_or_else(|| {
        panic!(
            "{launcher:?}: still running 25 s after Ctrl-C:\n{}",
            run.console()
        )
    });
    let mut signals = HashMap::new();
    for name in ["declared", "default"] {
        let text = std::fs::read_to_string(out.join(name)).unwrap();
        let start: f64 = text
            .lines()
            .find_map(|l| l.strip_prefix("START "))
            .unwrap()
            .parse()
            .unwrap();
        // Relative to the Ctrl-C rather than to START.
        let offset = mono_now - start;
        signals.insert(
            name,
            events(&out.join(name))
                .into_iter()
                .map(|(n, t)| (n, t - offset))
                .collect(),
        );
    }
    Some(Ladder {
        signals,
        exit_after,
    })
}

/// SIGINT at once, SIGTERM at `term`, and nothing after (SIGKILL is not
/// observable from inside), each within `slack`.
fn assert_rungs(who: &str, name: &str, got: &[(String, f64)], term: f64, slack: f64) {
    // A terminal's SIGINT and the launcher's own may both arrive (`ros2
    // launch` without a TTY counts as noninteractive and sends one too).
    let mut got = got.to_vec();
    got.dedup_by(|b, a| a.0 == b.0);
    let names: Vec<&str> = got.iter().map(|(n, _)| n.as_str()).collect();
    assert_eq!(
        names,
        ["SIGINT", "SIGTERM"],
        "{who}: {name} got {got:?}, want SIGINT then SIGTERM"
    );
    assert!(
        got[0].1 < 1.5,
        "{who}: {name}'s SIGINT came at {:.2}s",
        got[0].1
    );
    assert!(
        (got[1].1 - term).abs() < slack,
        "{who}: {name}'s SIGTERM came at {:.2}s, want {term}s",
        got[1].1
    );
}

fn check_ladder(who: &str, l: &Ladder) {
    assert_rungs(who, "declared", &l.signals["declared"], 2.0, 1.0);
    assert_rungs(who, "default", &l.signals["default"], 5.0, 1.0);
    // The launch ends once the last probe is SIGKILLed: 5 s + 5 s. play_launch
    // then spends up to 2 s draining its own tasks, as before #0084.
    assert!(
        (9.0..14.0).contains(&l.exit_after),
        "{who}: exited {:.2}s after Ctrl-C, want about 10s",
        l.exit_after
    );
}

#[test]
fn ros2_launch_stops_on_the_declared_timeouts() {
    if let Some(l) = ladder(Launcher::Stock) {
        check_ladder("ros2 launch", &l);
    }
}

#[test]
fn play_launch_stops_on_the_declared_timeouts() {
    if let Some(l) = ladder(Launcher::PlayLaunch("rust")) {
        check_ladder("play_launch (rust)", &l);
    }
}

#[test]
fn play_launch_python_parser_stops_on_the_declared_timeouts() {
    if let Some(l) = ladder(Launcher::PlayLaunch("python")) {
        check_ladder("play_launch (python)", &l);
    }
}

#[test]
fn up_from_a_model_stops_on_the_declared_timeouts() {
    if let Some(l) = ladder(Launcher::ResolveThenUp) {
        check_ladder("play_launch up", &l);
    }
}

fn starts(out: &Path) -> usize {
    std::fs::read_to_string(out.join("starts"))
        .map(|t| t.lines().count())
        .unwrap_or(0)
}

/// Exit code 2: one start, and the launch ends by itself.
fn ends_on_the_handlers_code(launcher: Launcher) {
    let tmp = tempfile::TempDir::new().unwrap();
    let out = tmp.path().canonicalize().unwrap();
    let Some(mut run) = start(launcher, &fixture("exit_handler.launch.py"), &[], &out) else {
        return;
    };
    let Some(after) = run.exit_within(Duration::from_secs(30)) else {
        panic!(
            "{launcher:?}: still running 30 s after start, {} start(s); the handler \
             shuts down on exit code 2:\n{}",
            starts(&out),
            run.console()
        );
    };
    assert_eq!(
        starts(&out),
        1,
        "{launcher:?}: the process must not respawn from the exit that ends the launch \
         (exited after {after:?}):\n{}",
        run.console()
    );
}

/// Exit code 1: the handler returns nothing, so the process respawns, and
/// the launch stays up until stopped.
fn respawns_on_another_code(launcher: Launcher) {
    let tmp = tempfile::TempDir::new().unwrap();
    let out = tmp.path().canonicalize().unwrap();
    let Some(mut run) = start(
        launcher,
        &fixture("exit_handler.launch.py"),
        &["code:=1"],
        &out,
    ) else {
        return;
    };
    let start = Instant::now();
    while starts(&out) < 3 && start.elapsed() < Duration::from_secs(30) {
        assert!(
            run.proc.try_wait().is_none(),
            "{launcher:?}: the launch ended on exit code 1, which the handler does not \
             shut down on:\n{}",
            run.console()
        );
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        starts(&out) >= 3,
        "{launcher:?}: {} start(s) in 30 s; the process respawns every second:\n{}",
        starts(&out),
        run.console()
    );
    run.ctrl_c();
    assert!(
        run.exit_within(run.started.elapsed() + Duration::from_secs(20))
            .is_some(),
        "{launcher:?}: did not stop after Ctrl-C:\n{}",
        run.console()
    );
}

#[test]
fn ros2_launch_ends_on_the_handlers_code() {
    ends_on_the_handlers_code(Launcher::Stock);
}

#[test]
fn rust_parser_ends_on_the_handlers_code() {
    ends_on_the_handlers_code(Launcher::PlayLaunch("rust"));
}

#[test]
fn python_parser_ends_on_the_handlers_code() {
    ends_on_the_handlers_code(Launcher::PlayLaunch("python"));
}

#[test]
fn up_from_a_model_ends_on_the_handlers_code() {
    ends_on_the_handlers_code(Launcher::ResolveThenUp);
}

#[test]
fn ros2_launch_respawns_on_another_code() {
    respawns_on_another_code(Launcher::Stock);
}

#[test]
fn rust_parser_respawns_on_another_code() {
    respawns_on_another_code(Launcher::PlayLaunch("rust"));
}

#[test]
fn python_parser_respawns_on_another_code() {
    respawns_on_another_code(Launcher::PlayLaunch("python"));
}
