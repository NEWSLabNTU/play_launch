use std::process::Stdio;

use play_launch_tests::fixtures;
use play_launch_tests::fixtures::array_len;
use play_launch_tests::process::ManagedProcess;

fn launch_file() -> String {
    fixtures::test_workspace_path("container_events")
        .join("launch/container_events.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

fn isolated_launch_file() -> String {
    fixtures::test_workspace_path("container_events")
        .join("launch/isolated_crash.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

// ---- Dump tests ----

#[test]
fn test_dump_container_events_rust() {
    let env = fixtures::install_env();
    let (record, _tmp) = fixtures::dump_launch(&env, &launch_file(), "rust");

    assert_eq!(array_len(&record, "node"), 0, "expected 0 standalone nodes");
    assert_eq!(array_len(&record, "container"), 1, "expected 1 container");
    assert_eq!(
        array_len(&record, "load_node"),
        2,
        "expected 2 composable nodes"
    );
}

#[test]
fn test_dump_container_events_python() {
    let env = fixtures::install_env();
    let (record, _tmp) = fixtures::dump_launch(&env, &launch_file(), "python");

    assert_eq!(array_len(&record, "node"), 0, "expected 0 standalone nodes");
    assert_eq!(array_len(&record, "container"), 1, "expected 1 container");
    assert_eq!(
        array_len(&record, "load_node"),
        2,
        "expected 2 composable nodes"
    );
}

// ---- Parser parity ----

#[test]
fn test_parser_parity_container_events() {
    let env = fixtures::install_env();
    let launch = launch_file();
    let (rust, _r) = fixtures::dump_launch(&env, &launch, "rust");
    let (python, _p) = fixtures::dump_launch(&env, &launch, "python");

    for key in ["node", "container", "load_node"] {
        assert_eq!(
            array_len(&rust, key),
            array_len(&python, key),
            "{key} count mismatch: rust={}, python={}",
            array_len(&rust, key),
            array_len(&python, key)
        );
    }
}

// ---- Isolated dump test (args attribute) ----

#[test]
fn test_dump_isolated_has_args() {
    let env = fixtures::install_env();
    let (record, _tmp) = fixtures::dump_launch(&env, &isolated_launch_file(), "rust");

    assert_eq!(array_len(&record, "container"), 1, "expected 1 container");

    // Verify the container's cmd contains --isolated
    let container = &record["container"][0];
    let cmd = container["cmd"]
        .as_array()
        .expect("cmd should be an array");
    let has_isolated = cmd.iter().any(|v| v.as_str() == Some("--isolated"));
    assert!(
        has_isolated,
        "expected --isolated in container cmd, got: {:?}",
        cmd
    );

    // Verify args field is set
    let args = container["args"]
        .as_array()
        .expect("args should be an array");
    assert_eq!(args.len(), 1);
    assert_eq!(args[0].as_str(), Some("--isolated"));
}

// ---- Launch test ----

#[test]
fn test_launch_container_events() {
    let env = fixtures::install_env();
    let launch = launch_file();

    // Dump to get expected process count (1 container)
    let (record, _tmp) = fixtures::dump_launch(&env, &launch, "rust");
    let expected = array_len(&record, "node") + array_len(&record, "container");
    assert_eq!(expected, 1, "expected 1 process (the container)");

    // Use a temp work directory to avoid stale play_log/latest from prior tests
    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let work_dir = work_tmp.path();

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        &launch,
    ]);

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let play_log = work_dir.join("play_log/latest");
    fixtures::wait_for_processes(&play_log, expected, std::time::Duration::from_secs(30));

    let actual = fixtures::count_cmdline_files(&play_log);
    assert_eq!(
        actual, expected,
        "process count mismatch: actual={actual}, expected={expected}"
    );
}

// ---- Crash detection test ----

/// Wait until `output_path` contains at least `count` lines matching any of `patterns`,
/// or until `timeout` elapses. Returns the number of matching lines found.
fn wait_for_pattern(
    output_path: &std::path::Path,
    patterns: &[&str],
    count: usize,
    timeout: std::time::Duration,
) -> usize {
    let start = std::time::Instant::now();
    let mut found = 0;
    while start.elapsed() < timeout {
        std::thread::sleep(std::time::Duration::from_secs(1));
        let content = std::fs::read_to_string(output_path).unwrap_or_default();
        found = content
            .lines()
            .filter(|l| patterns.iter().any(|p| l.contains(p)))
            .count();
        if found >= count {
            return found;
        }
    }
    found
}

/// Find the PID of a container process by matching its __node:= argument.
fn find_container_pid(node_name: &str, timeout: std::time::Duration) -> Option<u32> {
    let pattern = format!("__node:={node_name}");
    let start = std::time::Instant::now();
    let poll = std::time::Duration::from_secs(1);

    loop {
        let output = std::process::Command::new("pgrep")
            .args(["-f", &pattern])
            .output()
            .ok()?;

        if output.status.success() {
            let pid = String::from_utf8_lossy(&output.stdout)
                .lines()
                .filter_map(|line| line.trim().parse::<u32>().ok())
                .next();
            if pid.is_some() {
                return pid;
            }
        }

        if start.elapsed() >= timeout {
            return None;
        }
        std::thread::sleep(poll);
    }
}

/// Find child PIDs of a given parent PID using pgrep.
fn find_child_pids(parent_pid: u32) -> Vec<u32> {
    let output = std::process::Command::new("pgrep")
        .args(["-P", &parent_pid.to_string()])
        .output();

    match output {
        Ok(out) if out.status.success() => {
            String::from_utf8_lossy(&out.stdout)
                .lines()
                .filter_map(|line| line.trim().parse::<u32>().ok())
                .collect()
        }
        _ => Vec::new(),
    }
}

/// Wait until the given parent has at least `min_count` child processes,
/// or until `timeout` elapses.
fn wait_for_children(
    parent_pid: u32,
    min_count: usize,
    timeout: std::time::Duration,
) -> Vec<u32> {
    let start = std::time::Instant::now();
    let poll = std::time::Duration::from_secs(1);

    loop {
        let children = find_child_pids(parent_pid);
        if children.len() >= min_count {
            return children;
        }
        if start.elapsed() >= timeout {
            return children;
        }
        std::thread::sleep(poll);
    }
}

#[test]
fn test_crash_detection() {
    let env = fixtures::install_env();
    let launch = isolated_launch_file();

    // Use a temp work directory to avoid stale play_log/latest from prior tests
    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let work_dir = work_tmp.path();
    let output_path = work_dir.join("stdout.log");
    let stderr_path = work_dir.join("stderr.log");
    let output_file =
        std::fs::File::create(&output_path).expect("failed to create output file");
    let stderr_file =
        std::fs::File::create(&stderr_path).expect("failed to create stderr file");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        &launch,
    ]);
    cmd.stdout(Stdio::from(output_file));
    cmd.stderr(Stdio::from(stderr_file));
    cmd.env("RUST_LOG", "play_launch=debug");

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    // Wait for both composable nodes to be fully loaded before looking for
    // children. This ensures the container and play_launch event subscription
    // are fully initialized.
    let loaded = wait_for_pattern(
        &output_path,
        &["ComponentEvent LOADED", "LoadSucceeded"],
        2,
        std::time::Duration::from_secs(30),
    );
    assert!(
        loaded >= 2,
        "Expected 2 LOADED events before crash test, found {loaded}"
    );

    // Find container PID via pgrep (container_actor doesn't write a pid file)
    let container_pid = find_container_pid("event_container", std::time::Duration::from_secs(10))
        .expect("Failed to find container process 'event_container' via pgrep");

    eprintln!("Container PID: {container_pid}");

    // Find clone(CLONE_VM) child processes
    let children =
        wait_for_children(container_pid, 1, std::time::Duration::from_secs(10));
    assert!(
        !children.is_empty(),
        "No child processes found for container {container_pid} — \
         is --isolated mode working?"
    );
    eprintln!("Found {} child processes: {:?}", children.len(), children);

    // Kill one child to trigger crash detection
    let victim = children[0];
    eprintln!("Killing child PID {victim} with SIGKILL");
    unsafe {
        libc::kill(victim as i32, libc::SIGKILL);
    }

    // Wait for crash detection + event propagation
    let start = std::time::Instant::now();
    let timeout = std::time::Duration::from_secs(15);
    let mut found = false;
    while start.elapsed() < timeout {
        std::thread::sleep(std::time::Duration::from_secs(1));
        let stdout = std::fs::read_to_string(&output_path).unwrap_or_default();
        let stderr = std::fs::read_to_string(&stderr_path).unwrap_or_default();
        let combined = format!("{stdout}\n{stderr}");
        if combined.contains("crashed") || combined.contains("CRASHED") {
            found = true;
            break;
        }
    }

    // Read combined output for diagnostics
    let stdout_content = std::fs::read_to_string(&output_path).unwrap_or_default();
    let stderr_content = std::fs::read_to_string(&stderr_path).unwrap_or_default();
    let combined = format!("{stdout_content}\n{stderr_content}");

    eprintln!("--- Output snippet (last 2000 chars) ---");
    let snippet_start = combined.len().saturating_sub(2000);
    eprintln!("{}", &combined[snippet_start..]);

    assert!(
        found,
        "Expected crash detection message in output.\n\
         Searched for 'crashed' or 'CRASHED' in {} bytes of output",
        combined.len()
    );
}
