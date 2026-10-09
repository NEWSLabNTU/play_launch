//! Issue #0061 — `play_launch run` exits on SIGTERM, and takes its node with it.
//!
//! `run` handled the first SIGTERM by signalling the process group and the
//! run-level watch channel, and never called `member_handle.shutdown()`. The
//! node exited, its actor moved to `Stopped` — which deliberately stays alive
//! so the web UI can restart it — and waited on the actors' OWN shutdown
//! channel, which nothing sent on. The runner never completed, the drain after
//! the main loop waited on it forever, and `run` sat there until SIGKILL.
//!
//! Every other `run` test missed it because `ManagedProcess`'s cleanup sends
//! SIGTERM, waits two seconds and then SIGKILLs: a hang and a clean exit look
//! the same from there. Worse, the SIGKILL is what leaked nodes: they live in
//! `run`'s own process group, not the test's, so a killed `run` left its
//! talker publishing. So this test sends SIGTERM ONLY, measures the exit, and
//! checks the node is gone too.

use std::{
    process::Stdio,
    time::{Duration, Instant},
};

use play_launch_tests::{fixtures, process::ManagedProcess};

/// Long enough for a clean shutdown on a loaded CI runner; far shorter than the
/// forever this used to take.
const EXIT_DEADLINE: Duration = Duration::from_secs(15);

fn pid_alive(pid: i32) -> bool {
    // Signal 0 checks existence without delivering anything. A zombie still
    // answers, so a reaped-or-gone process is the only `false`.
    unsafe { libc::kill(pid, 0) == 0 }
}

fn wait_for_pid_file(path: &std::path::Path, timeout: Duration) -> i32 {
    let start = Instant::now();
    loop {
        if let Ok(text) = std::fs::read_to_string(path)
            && let Ok(pid) = text.trim().parse::<i32>()
        {
            return pid;
        }
        assert!(
            start.elapsed() < timeout,
            "{} never appeared within {timeout:?} — the node did not start",
            path.display()
        );
        std::thread::sleep(Duration::from_millis(100));
    }
}

fn exits_on(signal: libc::c_int, name: &str) {
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let work_dir = tmp.path().to_path_buf();

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir)
        .args(["run", "--disable-all", "demo_nodes_cpp", "talker"])
        .env("RUST_LOG", "play_launch=debug")
        .stdout(Stdio::from(
            std::fs::File::create(work_dir.join("stdout.log")).unwrap(),
        ))
        .stderr(Stdio::from(
            std::fs::File::create(work_dir.join("stderr.log")).unwrap(),
        ));
    let mut proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch run");

    let talker_pid = wait_for_pid_file(
        &work_dir.join("play_log/latest/node/talker/pid"),
        Duration::from_secs(30),
    );
    assert!(pid_alive(talker_pid), "talker {talker_pid} not running");

    // To `run` ALONE, not the group: that is `kill <pid>`, and it is the case
    // where `run` has to bring its own children down.
    let start = Instant::now();
    unsafe {
        libc::kill(proc.id() as i32, signal);
    }

    let status = loop {
        if let Some(status) = proc.try_wait() {
            break status;
        }
        if start.elapsed() > EXIT_DEADLINE {
            // stdout, not `play_launch.log`: the launcher's own log is written
            // through a buffer that a process stuck in this exact state has
            // not flushed, so it read back empty on the run that caught this.
            let log = std::fs::read_to_string(work_dir.join("stdout.log")).unwrap_or_default();
            let tail: Vec<_> = log.lines().rev().take(15).collect();
            panic!(
                "`play_launch run` still running {EXIT_DEADLINE:?} after {name} \
                 (issue #0061). Last log lines:\n{}",
                tail.into_iter().rev().collect::<Vec<_>>().join("\n")
            );
        }
        std::thread::sleep(Duration::from_millis(100));
    };
    eprintln!("{name}: exited after {:?} with {status}", start.elapsed());

    // The node must not outlive the launcher. Give the kernel a moment to
    // finish reaping before calling it a leak.
    let reap_start = Instant::now();
    while pid_alive(talker_pid) && reap_start.elapsed() < Duration::from_secs(3) {
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        !pid_alive(talker_pid),
        "talker {talker_pid} outlived `play_launch run` after {name}"
    );
}

#[test]
fn run_exits_on_sigterm() {
    exits_on(libc::SIGTERM, "SIGTERM");
}

#[test]
fn run_exits_on_sigint() {
    exits_on(libc::SIGINT, "SIGINT");
}
