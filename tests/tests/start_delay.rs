//! A launch `<timer>` delays the start of what it encloses, and nothing else.
//!
//! The parser has understood `<timer>` since the action was implemented, but
//! understanding it only ever reached the model as a note: `NodeInstance` had
//! no start-delay field, so every delayed node was spawned at t=0 and a nav2
//! bringup's carefully staggered lifecycle managers all came up at once. This
//! is the end-to-end check that the number now survives parser → SystemModel →
//! spawn, measured where it matters — on the wall clock of a real launch.
//!
//! The evidence is each member's `pid` file, written immediately after its
//! process is spawned (`regular_node_actor::write_pid_file`). Its mtime is the
//! spawn instant, independent of any log format.
//!
//! A composable node is the second test here, and a different mechanism: it
//! has no process of its own, so its `<timer>` delays the LoadNode request its
//! container issues rather than a spawn. Its evidence is its own output
//! directory, created at the instant the load is sent
//! (`ComposableSupervisor::send_load_over_socket`, and the LoadNode path does
//! the same) — a DIRECTORY's mtime, so that a chatty composable appending to
//! `out` cannot move it afterwards.

use std::{
    path::{Path, PathBuf},
    process::Stdio,
    time::{Duration, Instant, SystemTime},
};

use play_launch_tests::{fixtures, process::ManagedProcess};

/// The declared `<timer period="…">` in the fixture.
const PERIOD: Duration = Duration::from_secs(5);

fn launch_file(name: &str) -> String {
    fixtures::test_workspace_path("simple_test")
        .join("launch")
        .join(name)
        .to_str()
        .unwrap()
        .to_string()
}

/// Wait for `path` to exist and return its mtime. Polls fast enough that the
/// "not yet" assertions below are meaningful.
///
/// Used on a file for a node (its `pid`) and on a directory for a composable
/// (its output directory, created when the load is sent): a directory's mtime
/// moves only when an entry is added, so neither can be pushed forward by a
/// later write.
fn wait_for_file(path: &Path, timeout: Duration) -> SystemTime {
    let start = Instant::now();
    loop {
        if let Ok(meta) = std::fs::metadata(path) {
            return meta.modified().expect("mtime");
        }
        assert!(
            start.elapsed() < timeout,
            "{} never appeared within {timeout:?}",
            path.display()
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Both talkers are the same executable from the same package; only a
/// `<timer period="5.0">` separates them. The delayed one must start
/// approximately five seconds after the prompt one — and must not be running
/// before that.
#[test]
fn test_a_timer_delays_the_node_it_encloses() {
    let env = fixtures::install_env();

    let work_tmp = tempfile::TempDir::new().expect("tempdir");
    let work_dir: PathBuf = work_tmp.path().to_path_buf();
    let stdout = work_dir.join("stdout.log");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        &launch_file("timer_delay.launch.xml"),
    ]);
    cmd.stdout(Stdio::from(std::fs::File::create(&stdout).unwrap()));
    cmd.stderr(Stdio::from(
        std::fs::File::create(work_dir.join("stderr.log")).unwrap(),
    ));

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let play_log = work_dir.join("play_log/latest");
    let prompt_pid = play_log.join("node/prompt/pid");
    let delayed_pid = play_log.join("node/delayed/pid");

    // The undelayed node comes up on the usual startup path; the generous
    // timeout covers a cold ament index, not the delay under test.
    let prompt_at = wait_for_file(&prompt_pid, Duration::from_secs(60));

    // The delay is a WAIT, not a reordering: at the moment the undelayed node
    // is running, the delayed one must not have been spawned at all. This is
    // the assertion that failed before the field existed.
    assert!(
        !delayed_pid.exists(),
        "the delayed node was spawned alongside the undelayed one — the \
         <timer> did nothing"
    );

    let delayed_at = wait_for_file(&delayed_pid, PERIOD + Duration::from_secs(60));
    let gap = delayed_at
        .duration_since(prompt_at)
        .expect("the delayed node started before the undelayed one");

    // Lower bound tight, upper bound loose on purpose: starting EARLY is the
    // bug this test exists to catch, while starting late is a busy machine.
    // The slack below the period absorbs the fact that the two timestamps are
    // taken at spawn, and the prompt node's own spawn happens some
    // milliseconds after the epoch the delay is measured from.
    assert!(
        gap >= PERIOD - Duration::from_millis(1500),
        "the delayed node started {gap:?} after the undelayed one, short of \
         the declared {PERIOD:?}"
    );
    assert!(
        gap < PERIOD + Duration::from_secs(30),
        "the delayed node started {gap:?} after the undelayed one — far \
         longer than the declared {PERIOD:?}"
    );
}

/// The composable case: one container, two `composition::Talker`s declared
/// the same way, and a `<timer period="5.0">` around the second.
///
/// The property under test is not "the delayed one is late" but "the delayed
/// one is late AND nothing else is": the container comes up on the usual path
/// and its undelayed composable loads with it, so what the timer deferred is
/// one LoadNode request rather than the whole container. That distinction is
/// exactly what was missing — the delay reached the model, and the runtime
/// loaded both composables the moment the container was ready.
#[test]
fn test_a_timer_delays_the_composable_node_it_encloses() {
    let env = fixtures::install_env();

    let work_tmp = tempfile::TempDir::new().expect("tempdir");
    let work_dir: PathBuf = work_tmp.path().to_path_buf();
    let stdout = work_dir.join("stdout.log");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        &launch_file("timer_composable.launch.xml"),
    ]);
    cmd.stdout(Stdio::from(std::fs::File::create(&stdout).unwrap()));
    cmd.stderr(Stdio::from(
        std::fs::File::create(work_dir.join("stderr.log")).unwrap(),
    ));

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let play_log = work_dir.join("play_log/latest");
    let prompt_dir = play_log.join("load_node/prompt");
    let delayed_dir = play_log.join("load_node/delayed");

    // The undelayed composable loads as soon as the container is up. The
    // generous timeout covers starting a container process, not the delay
    // under test.
    let prompt_at = wait_for_file(&prompt_dir, Duration::from_secs(90));

    // The container is up and loading — and the delayed composable has not
    // been asked for. This is the assertion that failed before the container
    // actor could defer an individual load.
    assert!(
        !delayed_dir.exists(),
        "the delayed composable was loaded alongside the undelayed one — the \
         <timer> did nothing"
    );

    let delayed_at = wait_for_file(&delayed_dir, PERIOD + Duration::from_secs(90));
    let gap = delayed_at
        .duration_since(prompt_at)
        .expect("the delayed composable loaded before the undelayed one");

    // Same bounds as the node case, and for the same reason: loading EARLY is
    // the bug, loading late is a busy machine. The slack below the period
    // absorbs the container's own startup, which happens after the epoch the
    // delay is measured from and before the prompt composable's load.
    assert!(
        gap >= PERIOD - Duration::from_millis(1500),
        "the delayed composable loaded {gap:?} after the undelayed one, short \
         of the declared {PERIOD:?}"
    );
    assert!(
        gap < PERIOD + Duration::from_secs(30),
        "the delayed composable loaded {gap:?} after the undelayed one — far \
         longer than the declared {PERIOD:?}"
    );
}
