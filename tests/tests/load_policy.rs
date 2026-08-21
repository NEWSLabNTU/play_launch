//! Phase 64 W2 — failure detection, retry, and constructors that take minutes.
//!
//! These tests are about the DECISION, not the outcome: given the same silence,
//! does play_launch resend, fail, or wait? Each one pins a case where the two
//! wrong answers cost differently — a resend forks a second process for a node
//! that is alive and working, a wrong give-up loses a node that was fine.
//!
//! Design: `docs/design/composable-load-lifecycle.md`.

use std::process::Stdio;
use std::time::{Duration, Instant};

use play_launch_tests::fixtures;
use play_launch_tests::process::ManagedProcess;

fn launch_path(name: &str) -> String {
    fixtures::test_workspace_path("parallel_loading")
        .join(format!("launch/{name}.launch.xml"))
        .to_str()
        .unwrap()
        .to_string()
}

fn container_events_launch() -> String {
    fixtures::test_workspace_path("container_events")
        .join("launch/container_events.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

/// Log fragments that would mean the supervisor decided to load something
/// again. Deliberately anchored: the correct behaviour prints "not resending",
/// and a marker matching that substring would pass this assertion by accident.
const RESEND_MARKERS: &[&str] = &["; resending", "re-dispatching", "— reloading", "reloading '"];

struct Run {
    _proc: ManagedProcess,
    _tmp: tempfile::TempDir,
    stdout: std::path::PathBuf,
    session: u32,
}

impl Run {
    fn text(&self) -> String {
        std::fs::read_to_string(&self.stdout).unwrap_or_default()
    }

    fn wait_for(&self, pattern: &str, timeout: Duration) -> bool {
        let start = Instant::now();
        loop {
            if self.text().contains(pattern) {
                return true;
            }
            if start.elapsed() >= timeout {
                return false;
            }
            std::thread::sleep(Duration::from_millis(500));
        }
    }

    /// How many `component_node` children exist for a plugin, in THIS test's
    /// session only. The number that matters: two is the double load every
    /// rule in the design exists to prevent.
    fn children_for(&self, plugin: &str) -> usize {
        let out = std::process::Command::new("pgrep")
            .args(["-s", &self.session.to_string(), "-f", plugin])
            .output();
        match out {
            Ok(o) => String::from_utf8_lossy(&o.stdout).lines().count(),
            Err(_) => 0,
        }
    }

    fn assert_no_resend(&self) {
        let text = self.text();
        for marker in RESEND_MARKERS {
            assert!(
                !text.contains(marker),
                "the supervisor decided to load again ('{marker}') without a confirmed \
                 absence:\n{text}"
            );
        }
    }
}

/// `config` is written to a temp file when non-empty; `drop_faults` sets the
/// container's test-only fault injection.
fn launch(launch_file: &str, config: &str, drop_faults: Option<&str>) -> Run {
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let work_dir = tmp.path().to_path_buf();
    let stdout = work_dir.join("stdout.log");
    let stdout_file = std::fs::File::create(&stdout).expect("create stdout log");
    let stderr_file = std::fs::File::create(work_dir.join("stderr.log")).expect("create stderr");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--container-mode",
        "isolated",
    ]);
    if !config.is_empty() {
        let path = work_dir.join("policy.yaml");
        std::fs::write(&path, config).expect("write config");
        cmd.args(["--config", path.to_str().unwrap()]);
    }
    if let Some(faults) = drop_faults {
        cmd.env("PLAY_LAUNCH_CONTROL_DROP", faults);
    }
    cmd.arg(launch_file);
    cmd.stdout(Stdio::from(stdout_file));
    cmd.stderr(Stdio::from(stderr_file));
    cmd.env("RUST_LOG", "play_launch=debug");

    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    let session = proc.id();
    Run {
        _proc: proc,
        _tmp: tmp,
        stdout,
        session,
    }
}

/// A load frame the container never received. Nothing was accepted, so nothing
/// was forked — the one case where a resend cannot double anything, and the
/// only automatic one there is.
///
/// The resend is still gated on the container SAYING it has no record: the ack
/// timeout only causes the question.
#[test]
fn test_a_load_the_container_never_saw_is_resent_once() {
    let run = launch(&container_events_launch(), "", Some("first_load"));

    assert!(
        run.wait_for("the container has no record of it", Duration::from_secs(60)),
        "the lost load was never diagnosed:\n{}",
        run.text()
    );
    assert!(
        run.wait_for("Startup complete", Duration::from_secs(60)),
        "the resent load never completed:\n{}",
        run.text()
    );

    let text = run.text();
    assert!(
        text.contains("composable 2/2"),
        "expected both composables loaded after the resend:\n{text}"
    );
    // Exactly one resend, and it names the evidence rather than a timeout.
    assert_eq!(
        text.matches("the container has no record of it").count(),
        1,
        "expected exactly one lost-load diagnosis:\n{text}"
    );
    assert_eq!(
        run.children_for("composition::Talker"),
        1,
        "expected exactly one Talker process after the resend"
    );
    assert_eq!(
        run.children_for("composition::Listener"),
        1,
        "expected exactly one Listener process after the resend"
    );
}

/// The container is alive but never answers a query. That is a third state —
/// neither running nor gone — and it must never be resolved by assumption.
#[test]
fn test_unanswered_queries_never_authorise_a_resend() {
    let run = launch(
        &container_events_launch(),
        "composable_node_loading:\n  ack_timeout_ms: 1000\n  probe_interval_secs: 1\n",
        Some("first_load,status"),
    );

    assert!(
        run.wait_for("queries unanswered", Duration::from_secs(60)),
        "an unanswered probe was never reported:\n{}",
        run.text()
    );

    // Give the sweep several more chances to do the wrong thing.
    std::thread::sleep(Duration::from_secs(10));

    let text = run.text();
    assert!(
        !text.contains("the container has no record of it"),
        "silence was read as a confirmed absence:\n{text}"
    );
    run.assert_no_resend();
    assert!(
        text.contains("left alone"),
        "the unanswered load was not reported as unresolved:\n{text}"
    );
}

/// A constructor that legitimately takes a long time. The supervisor probes it
/// because it went quiet, gets `constructing` back, and leaves it alone.
///
/// This is the regression test for the double-load hazard: the ListNodes
/// verification this replaces answered `Absent` for exactly this case, because
/// the isolated container hides mid-construction ids.
#[test]
fn test_a_slow_constructor_is_probed_and_left_alone() {
    let run = launch(
        &launch_path("slow_construct_20s"),
        "composable_node_loading:\n  report_timeout_secs: 5\n  probe_interval_secs: 5\n",
        None,
    );

    assert!(
        run.wait_for("still in flight, not resending", Duration::from_secs(45)),
        "the slow constructor was never probed, or the answer was not acted on:\n{}",
        run.text()
    );
    assert_eq!(
        run.children_for("VoxelGrid|SlowLoader"),
        run.children_for("SlowLoader"),
        "sanity: pattern match is stable"
    );
    assert_eq!(
        run.children_for("SlowLoader"),
        1,
        "a probed constructor must not be duplicated"
    );

    assert!(
        run.wait_for("Startup complete", Duration::from_secs(60)),
        "the slow constructor never finished:\n{}",
        run.text()
    );
    run.assert_no_resend();
}

/// A constructor that is alive, past its budget, and burning no CPU. With
/// `stall_action: restart` the supervisor cancels it, waits for the container
/// to CONFIRM nothing is running, and only then reloads — never the other
/// order.
#[test]
fn test_stall_restart_cancels_before_it_reloads() {
    let run = launch(
        &launch_path("slow_construct_60s"),
        "composable_node_loading:\n  report_timeout_secs: 5\n  probe_interval_secs: 5\n  \
         stall_after_secs: 10\n  stall_action: restart\n",
        None,
    );

    assert!(
        run.wait_for("stalled at", Duration::from_secs(90)),
        "the stall was never detected:\n{}",
        run.text()
    );
    assert!(
        run.wait_for("cancelled and confirmed gone", Duration::from_secs(60)),
        "the reload did not wait for the container's confirmation:\n{}",
        run.text()
    );

    let text = run.text();
    let cancel_at = text.find("cancelling").expect("a cancel must precede the reload");
    let reload_at = text
        .find("cancelled and confirmed gone")
        .expect("the confirmation must be logged");
    assert!(
        cancel_at < reload_at,
        "the reload was ordered before the cancellation:\n{text}"
    );
    assert!(
        run.children_for("SlowLoader") <= 1,
        "cancel -> confirm -> resend must never leave two processes"
    );

    // The attempt budget bounds the cycle: a node that stalls again is failed
    // rather than restarted forever.
    assert!(
        run.wait_for("is the configured maximum", Duration::from_secs(120)),
        "the restart cycle was not bounded by max_load_attempts:\n{}",
        run.text()
    );
}

/// A constructor that never returns. Nothing may fail it, nothing may retry
/// it, and the run must say what it is waiting for instead of repeating a bare
/// count forever.
#[test]
fn test_a_wedged_constructor_is_reported_not_failed() {
    let run = launch(
        &launch_path("wedged_construct"),
        "startup:\n  stage_timeout_secs: 20\n",
        None,
    );

    assert!(
        run.wait_for("Startup still incomplete after", Duration::from_secs(90)),
        "the run never said what it was waiting for:\n{}",
        run.text()
    );

    let text = run.text();
    assert!(
        text.contains("wedged_node"),
        "the waiting-on report did not name the composable:\n{text}"
    );
    assert!(
        text.contains("nothing has been failed or reloaded on their account"),
        "the report did not say that nothing was done about it:\n{text}"
    );
    assert!(
        text.contains("still constructing after"),
        "no liveness evidence was reported for the wedged constructor:\n{text}"
    );
    assert!(
        !text.contains("LOAD_FAILED"),
        "a wedged constructor was reported as a failure:\n{text}"
    );
    run.assert_no_resend();
    assert_eq!(
        run.children_for("SlowLoader"),
        1,
        "the wedged constructor must be left exactly as it is"
    );
}
