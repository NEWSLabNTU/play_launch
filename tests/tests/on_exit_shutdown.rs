//! `on_exit=Shutdown()` ends the launch when that process exits, under either
//! parser -- as `ros2 launch` does.
//!
//! This is how a launch says "this process is required": SSv2's
//! `scenario_test_runner.launch.py` declares its runner with
//! `on_exit=ShutdownOnce()` (an `EmitEvent` of `launch.events.Shutdown`), so a
//! finished scenario takes its whole stack down. The Python dump path honoured
//! it (779d68e5); the Rust parser recorded no `on_exit` at all, and a scenario
//! stack resolved with it stayed up after its runner exited -- 58 minutes, in
//! the carla-scenario-bridge run that found it (2026-10-11).
//!
//! The fixture: a long-lived `sleep 300` beside a short `sleep 2` whose
//! `on_exit` shuts down. The launch must end in seconds, not at 300 s.

use std::{
    process::Stdio,
    time::{Duration, Instant},
};

use play_launch_tests::{fixtures, process::ManagedProcess};

const LAUNCH_PY: &str = r#"
from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, Shutdown
from launch.events import Shutdown as ShutdownEvent


class ShutdownOnce(EmitEvent):
    """SSv2's shape: a subclass of EmitEvent carrying the Shutdown event."""

    def __init__(self, **kwargs):
        super().__init__(event=ShutdownEvent(reason='required'), **kwargs)


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['sleep', '300'], name='bystander'),
        ExecuteProcess(cmd=['sleep', '2'], name='required', on_exit=ON_EXIT),
    ])
"#;

fn ends_on_required_exit(parser: &str, on_exit: &str) {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let work_tmp = tempfile::TempDir::new().expect("tempdir");
    let work_dir = work_tmp.path();
    let launch = work_dir.join("required.launch.py");
    std::fs::write(&launch, LAUNCH_PY.replace("ON_EXIT", on_exit)).expect("write launch");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--parser",
        parser,
        launch.to_str().unwrap(),
    ]);
    let stdout = work_dir.join("stdout.log");
    cmd.stdout(Stdio::from(std::fs::File::create(&stdout).unwrap()));
    cmd.stderr(Stdio::from(
        std::fs::File::create(work_dir.join("stderr.log")).unwrap(),
    ));

    let started = Instant::now();
    let mut proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    let deadline = Duration::from_secs(60);
    while proc.try_wait().is_none() {
        if started.elapsed() > deadline {
            let out = std::fs::read_to_string(&stdout).unwrap_or_default();
            panic!(
                "--parser {parser}, on_exit={on_exit}: the launch was still up {deadline:?} after \
                 start; the required process exits at 2 s.\n{}",
                &out[out.len().saturating_sub(3000)..]
            );
        }
        std::thread::sleep(Duration::from_millis(200));
    }
}

#[test]
fn rust_parser_shutdown_action() {
    ends_on_required_exit("rust", "Shutdown()");
}

#[test]
fn rust_parser_emit_event_subclass() {
    ends_on_required_exit("rust", "ShutdownOnce()");
}

#[test]
fn rust_parser_list_of_actions() {
    ends_on_required_exit("rust", "[ShutdownOnce()]");
}

#[test]
fn python_parser_shutdown_action() {
    ends_on_required_exit("python", "Shutdown()");
}
