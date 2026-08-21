//! Phase 64 — the private control channel to `play_launch_container`.
//!
//! What these tests pin down is not "composables load" (every other container
//! suite covers that) but WHICH PATH loaded them, and that the five mechanisms
//! built to survive a congested rmw layer are absent from the socket path
//! rather than merely quiet on it.

use std::process::Stdio;
use std::time::Duration;

use play_launch_tests::fixtures;
use play_launch_tests::process::ManagedProcess;

fn launch_file() -> String {
    fixtures::test_workspace_path("container_events")
        .join("launch/container_events.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

/// One composable with a 20s constructor — long enough to cross the
/// container's 15s liveness interval.
fn slow_launch_file() -> String {
    fixtures::test_workspace_path("parallel_loading")
        .join("launch/slow_construct_20s.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

/// Log lines that only ever appear when a load went through the ROS service.
const LOAD_NODE_SERVICE_MARKERS: &[&str] = &[
    "Initiating LoadNode service call",
    "Waiting for LoadNode service to be available",
];

/// The false alarms this phase exists to remove. Every one of these was seen
/// on a launch where nothing had actually failed.
const LOAD_PATH_ALARMS: &[&str] = &[
    "LoadNode service call timed out",
    "LoadNode response lost",
    "ComponentEvent LOADED not received",
    "confirmed loaded via ListNodes after timeout",
];

struct Run {
    _proc: ManagedProcess,
    _tmp: tempfile::TempDir,
    stdout: std::path::PathBuf,
}

impl Run {
    fn text(&self) -> String {
        std::fs::read_to_string(&self.stdout).unwrap_or_default()
    }

    /// Wait until the log contains `pattern`, or give up.
    fn wait_for(&self, pattern: &str, timeout: Duration) -> bool {
        let start = std::time::Instant::now();
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
}

/// Launch the two-composable fixture with debug logging captured to a file.
fn launch(extra_args: &[&str]) -> Run {
    launch_file_with(&launch_file(), extra_args)
}

/// Launch a specific file with debug logging captured to a file.
fn launch_file_with(launch_path: &str, extra_args: &[&str]) -> Run {
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
    ]);
    cmd.args(extra_args);
    cmd.arg(launch_path);
    cmd.stdout(Stdio::from(stdout_file));
    cmd.stderr(Stdio::from(stderr_file));
    cmd.env("RUST_LOG", "play_launch=debug");

    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    Run {
        _proc: proc,
        _tmp: tmp,
        stdout,
    }
}

/// The default path: our own isolated container, loaded over the socket, with
/// the LoadNode service untouched.
#[test]
fn test_isolated_loads_over_the_control_channel() {
    let run = launch(&["--container-mode", "isolated"]);

    assert!(
        run.wait_for("Startup complete", Duration::from_secs(60)),
        "startup never completed:\n{}",
        run.text()
    );

    let text = run.text();
    assert!(
        text.contains("loading composables over the private control channel"),
        "the container never negotiated the control channel:\n{text}"
    );
    assert!(
        text.contains("control channel LOADED for"),
        "no load was reported over the control channel:\n{text}"
    );
    assert!(
        text.contains("composable 2/2"),
        "expected both composables loaded:\n{text}"
    );

    for marker in LOAD_NODE_SERVICE_MARKERS {
        assert!(
            !text.contains(marker),
            "the LoadNode service was still used ({marker}) — the socket path is supposed to \
             replace it entirely:\n{text}"
        );
    }
    for alarm in LOAD_PATH_ALARMS {
        assert!(
            !text.contains(alarm),
            "load-path alarm '{alarm}' on a launch where nothing failed:\n{text}"
        );
    }
}

/// The escape hatch. `control_socket: false` puts every load back on the ROS
/// service, unchanged — which is also how the version-skew fallback behaves,
/// so this is the one test that keeps that path exercised.
#[test]
fn test_control_socket_can_be_turned_off() {
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let config = tmp.path().join("no_socket.yaml");
    std::fs::write(
        &config,
        "composable_node_loading:\n  control_socket: false\n",
    )
    .expect("write config");

    let run = launch(&[
        "--container-mode",
        "isolated",
        "--config",
        config.to_str().unwrap(),
    ]);

    assert!(
        run.wait_for("Startup complete", Duration::from_secs(60)),
        "startup never completed with the socket disabled:\n{}",
        run.text()
    );

    let text = run.text();
    assert!(
        text.contains("composable 2/2"),
        "expected both composables loaded over LoadNode:\n{text}"
    );
    assert!(
        LOAD_NODE_SERVICE_MARKERS
            .iter()
            .any(|m| text.contains(m)),
        "expected the LoadNode service path with control_socket: false:\n{text}"
    );
    assert!(
        !text.contains("control channel LOADED for"),
        "the control channel was used despite control_socket: false:\n{text}"
    );
}

/// `observable` is our binary too, so the channel comes up — but loading there
/// runs on the manager's executor, so the container answers
/// `loads_over_socket: false` and LoadNode stays in charge. The socket still
/// carries status.
#[test]
fn test_observable_keeps_load_node_and_reports_on_the_socket() {
    let run = launch(&["--container-mode", "observable"]);

    assert!(
        run.wait_for("Startup complete", Duration::from_secs(60)),
        "startup never completed in observable mode:\n{}",
        run.text()
    );

    let text = run.text();
    assert!(
        text.contains("control channel up for status"),
        "observable container did not negotiate a status-only channel:\n{text}"
    );
    assert!(
        !text.contains("loading composables over the private control channel"),
        "observable must not take loads over the socket:\n{text}"
    );
    assert!(
        LOAD_NODE_SERVICE_MARKERS
            .iter()
            .any(|m| text.contains(m)),
        "expected observable loads to use the LoadNode service:\n{text}"
    );
    assert!(
        text.contains("composable 2/2"),
        "expected both composables loaded:\n{text}"
    );
}

/// A stock container gets no channel at all — it would not know what to do
/// with the fd, and this is the compatibility guarantee the phase makes.
#[test]
fn test_stock_container_gets_no_control_channel() {
    let run = launch(&["--container-mode", "stock"]);

    assert!(
        run.wait_for("Startup complete", Duration::from_secs(60)),
        "startup never completed in stock mode:\n{}",
        run.text()
    );

    let text = run.text();
    assert!(
        !text.contains("control channel"),
        "a stock container was given a control channel:\n{text}"
    );
    assert!(
        text.contains("composable 2/2"),
        "expected both composables loaded:\n{text}"
    );
}

/// The honest half: a composable that is merely SLOW must look different from
/// one that is wedged, and the difference must come from what the container
/// reports rather than from a deadline the supervisor invented.
///
/// The old path could only say "no result yet" and then guess; this one names
/// the pid and the elapsed time every 15s for as long as the child is alive,
/// which is the same evidence for a 20s constructor and for one that never
/// returns — the operator gets the elapsed time and decides.
#[test]
fn test_slow_constructor_reports_liveness_not_a_timeout() {
    let run = launch_file_with(&slow_launch_file(), &["--container-mode", "isolated"]);

    assert!(
        run.wait_for("still constructing after", Duration::from_secs(45)),
        "no liveness report for a 20s constructor:\n{}",
        run.text()
    );

    let text = run.text();
    assert!(
        text.contains("reported by the container"),
        "the liveness line did not come from the container:\n{text}"
    );
    for alarm in LOAD_PATH_ALARMS {
        assert!(
            !text.contains(alarm),
            "a slow constructor produced the alarm '{alarm}':\n{text}"
        );
    }

    assert!(
        run.wait_for("Startup complete", Duration::from_secs(60)),
        "the slow composable never finished loading:\n{}",
        run.text()
    );
    assert!(
        run.text().contains("control channel LOADED for"),
        "the load result did not arrive over the control channel:\n{}",
        run.text()
    );
}
