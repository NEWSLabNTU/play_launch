//! `on_exit=Shutdown()` ends the launch when that process exits, under either
//! parser, every frontend and both ways of starting a system -- as `ros2
//! launch` does.
//!
//! This is how a launch says "this process is required": SSv2's
//! `scenario_test_runner.launch.py` declares its runner with
//! `on_exit=ShutdownOnce()` (an `EmitEvent` of `launch.events.Shutdown`), so a
//! finished scenario takes its whole stack down. The Python dump path honoured
//! it (779d68e5); the Rust parser recorded no `on_exit` at all, and a scenario
//! stack resolved with it stayed up after its runner exited -- 58 minutes, in
//! the carla-scenario-bridge run that found it (2026-10-11).
//!
//! The fixtures: a long-lived `sleep 300` beside a short `sleep` whose
//! `on_exit` shuts down. The launch must end when the short one does, not at
//! 300 s, and not before.

use std::{
    path::Path,
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

/// The XML frontend's spelling (#0064): stock `ExecuteProcess.parse` turns
/// `on_exit="shutdown"` into `on_exit=[Shutdown()]`.
const LAUNCH_XML: &str = r#"<launch>
  <executable cmd="sleep 300" name="bystander"/>
  <executable cmd="sleep 2" name="required" on_exit="shutdown"/>
</launch>
"#;

/// #0065: two UNNAMED processes with the same executable. Only the 3 s one is
/// required. The required set used to be matched by bare name with the `-N`
/// counter stripped, so both were `sleep` and the 1 s one ended the launch.
const LAUNCH_SAME_STEM_PY: &str = r#"
from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['sleep', '300'], name='bystander'),
        ExecuteProcess(cmd=['sleep', '1']),
        ExecuteProcess(cmd=['sleep', '3'], on_exit=Shutdown()),
    ])
"#;

/// How the system is started.
#[derive(Clone, Copy)]
enum Start {
    /// `play_launch launch <file>`.
    Launch,
    /// `play_launch resolve <file> -o m.yaml`, then `play_launch up m.yaml`
    /// (#0063): the flag must survive the model.
    ResolveThenUp,
}

struct Case<'a> {
    label: &'a str,
    file_name: &'a str,
    content: &'a str,
    parser: &'a str,
    start: Start,
    /// When the required process exits; the launch must not end before.
    required_after: Duration,
    /// The `[node:/…]` member the on_exit handler must fire for.
    required_member: &'a str,
}

fn run_case(case: Case) {
    let Case {
        label,
        file_name,
        content,
        parser,
        start,
        required_after,
        required_member,
    } = case;
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let work_tmp = tempfile::TempDir::new().expect("tempdir");
    let work_dir = work_tmp.path();
    let launch = work_dir.join(file_name);
    std::fs::write(&launch, content).expect("write launch");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_dir);
    match start {
        Start::Launch => {
            cmd.args(["launch", "--parser", parser]).arg(&launch);
        }
        Start::ResolveThenUp => {
            let model = work_dir.join("model.yaml");
            let out = fixtures::play_launch_cmd(&env)
                .current_dir(work_dir)
                .args(["resolve", "--parser", parser])
                .arg(&launch)
                .arg("-o")
                .arg(&model)
                .output()
                .expect("run resolve");
            assert!(
                out.status.success(),
                "{label}: resolve failed:\n{}",
                String::from_utf8_lossy(&out.stderr)
            );
            cmd.arg("up").arg(&model);
        }
    }
    cmd.args([
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
    ]);
    let stdout = work_dir.join("stdout.log");
    cmd.stdout(Stdio::from(std::fs::File::create(&stdout).unwrap()));
    cmd.stderr(Stdio::from(
        std::fs::File::create(work_dir.join("stderr.log")).unwrap(),
    ));

    let started = Instant::now();
    let mut proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    let deadline = Duration::from_secs(60);
    let status = loop {
        if let Some(status) = proc.try_wait() {
            break status;
        }
        if started.elapsed() > deadline {
            let out = std::fs::read_to_string(&stdout).unwrap_or_default();
            panic!(
                "{label}: the launch was still up {deadline:?} after start; the required \
                 process exits at {required_after:?}.\n{}",
                &out[out.len().saturating_sub(3000)..]
            );
        }
        std::thread::sleep(Duration::from_millis(200));
    };
    let elapsed = started.elapsed();
    check_ended_by_required_exit(
        label,
        work_dir,
        &stdout,
        status,
        elapsed,
        required_after,
        required_member,
    );
}

/// Ending quickly is not enough: a launch that fails to parse also ends
/// quickly. Each check rules out one way of exiting for the wrong reason.
fn check_ended_by_required_exit(
    label: &str,
    work_dir: &Path,
    stdout: &Path,
    status: std::process::ExitStatus,
    elapsed: Duration,
    required_after: Duration,
    required_member: &str,
) {
    let out = std::fs::read_to_string(stdout).unwrap_or_default();
    let tail = &out[out.len().saturating_sub(3000)..];
    assert!(
        status.success(),
        "{label}: play_launch exited with {status}, not 0.\n{tail}"
    );
    assert!(
        elapsed >= required_after,
        "{label}: play_launch ended after {elapsed:?}, before the required process could \
         have exited at {required_after:?}.\n{tail}"
    );
    let fired = format!("[{required_member}] exited (code 0) and was declared on_exit=Shutdown");
    assert!(
        out.contains(&fired),
        "{label}: the launch ended without the on_exit handler firing for \
         '{required_member}'.\n{tail}"
    );
    // The bystander must have run, and must not outlive the launch.
    let pid_file = work_dir.join("play_log/latest/node/bystander/pid");
    let bystander: i32 = std::fs::read_to_string(&pid_file)
        .unwrap_or_else(|e| {
            panic!(
                "{label}: {} unreadable ({e}): the bystander never started.\n{tail}",
                pid_file.display()
            )
        })
        .trim()
        .parse()
        .expect("bystander pid");
    let reap_start = Instant::now();
    while unsafe { libc::kill(bystander, 0) } == 0 && reap_start.elapsed() < Duration::from_secs(3)
    {
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        unsafe { libc::kill(bystander, 0) } != 0,
        "{label}: bystander {bystander} outlived the launch"
    );
}

fn py_case<'a>(label: &'a str, content: &'a str, parser: &'a str, start: Start) -> Case<'a> {
    Case {
        label,
        file_name: "required.launch.py",
        content,
        parser,
        start,
        required_after: Duration::from_secs(2),
        required_member: "node:/required",
    }
}

fn ends_on_required_exit(parser: &str, on_exit: &str) {
    let content = LAUNCH_PY.replace("ON_EXIT", on_exit);
    let label = format!("--parser {parser}, on_exit={on_exit}");
    run_case(py_case(&label, &content, parser, Start::Launch));
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

// The Python dumper used to RUN both `sleep`s at parse time and record
// nothing (#0066); this case passed only because the parse took about two
// seconds. The checks in `check_ended_by_required_exit` are what exposed it.
#[test]
fn python_parser_shutdown_action() {
    ends_on_required_exit("python", "Shutdown()");
}

/// #0064: the XML frontend honours `on_exit="shutdown"`.
#[test]
fn xml_on_exit_shutdown() {
    run_case(Case {
        label: "XML on_exit=\"shutdown\"",
        file_name: "required.launch.xml",
        content: LAUNCH_XML,
        parser: "rust",
        start: Start::Launch,
        required_after: Duration::from_secs(2),
        required_member: "node:/required",
    });
}

/// #0063: `resolve` then `up` honours it, because the model carries it.
#[test]
fn up_from_a_resolved_model() {
    let content = LAUNCH_PY.replace("ON_EXIT", "Shutdown()");
    run_case(py_case(
        "resolve then up",
        &content,
        "rust",
        Start::ResolveThenUp,
    ));
}

/// #0063 for the XML frontend too.
#[test]
fn up_from_a_resolved_xml_model() {
    run_case(Case {
        label: "XML, resolve then up",
        file_name: "required.launch.xml",
        content: LAUNCH_XML,
        parser: "rust",
        start: Start::ResolveThenUp,
        required_after: Duration::from_secs(2),
        required_member: "node:/required",
    });
}

/// #0065: an unrelated process with the same executable must not end it. The
/// launch has to outlast the 1 s `sleep` and end with the 3 s one.
#[test]
fn a_same_stem_bystander_does_not_end_the_launch() {
    run_case(Case {
        label: "same-stem bystander",
        file_name: "same_stem.launch.py",
        content: LAUNCH_SAME_STEM_PY,
        parser: "rust",
        start: Start::Launch,
        required_after: Duration::from_secs(3),
        // The second unnamed `sleep`: the id allocator numbers collisions `#N`.
        required_member: "node:/sleep#2",
    });
}
