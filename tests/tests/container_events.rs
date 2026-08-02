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

fn isolated_launch_file() -> String {
    fixtures::test_workspace_path("container_events")
        .join("launch/isolated_crash.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

// ---- Resolve tests (Phase 47.B6: model, not record.json) ----

#[test]
fn test_resolve_container_events_rust() {
    let env = fixtures::install_env();
    let (model, _tmp) = fixtures::resolve_model(&env, &launch_file(), None, "rust");
    let (plain, containers, composables) = fixtures::model_entity_counts(&model);

    assert_eq!(plain, 0, "expected 0 standalone nodes");
    assert_eq!(containers, 1, "expected 1 container");
    assert_eq!(composables, 2, "expected 2 composable nodes");
}

#[test]
fn test_resolve_container_events_python() {
    let env = fixtures::install_env();
    let (model, _tmp) = fixtures::resolve_model(&env, &launch_file(), None, "python");
    let (plain, containers, composables) = fixtures::model_entity_counts(&model);

    assert_eq!(plain, 0, "expected 0 standalone nodes");
    assert_eq!(containers, 1, "expected 1 container");
    assert_eq!(composables, 2, "expected 2 composable nodes");
}

// ---- Parser parity ----

#[test]
fn test_parser_parity_container_events() {
    let env = fixtures::install_env();
    let launch = launch_file();
    let (rust, _r) = fixtures::resolve_model(&env, &launch, None, "rust");
    let (python, _p) = fixtures::resolve_model(&env, &launch, None, "python");

    assert_eq!(
        fixtures::model_entity_counts(&rust),
        fixtures::model_entity_counts(&python),
        "entity counts mismatch (plain, containers, composables)"
    );
}

// ---- Isolated resolve test (args attribute) ----

#[test]
fn test_resolve_isolated_has_args() {
    let env = fixtures::install_env();
    let (model, _tmp) = fixtures::resolve_model(&env, &isolated_launch_file(), None, "rust");
    let (_plain, containers, _composables) = fixtures::model_entity_counts(&model);
    assert_eq!(containers, 1, "expected 1 container");

    // Verify the container's launch-declared `args` carries --isolated
    // (Phase 47.B6: the model's `args` field is the launch-declared source
    // of truth `node_container args="--isolated"` lowers to; the fully
    // assembled argv record.json's `cmd` used to expose is a replay-time
    // derivation, not a stored model field).
    let args = fixtures::first_container_args(&model).expect("expected a container in the model");
    assert_eq!(
        args,
        vec!["--isolated".to_string()],
        "expected --isolated in container args, got: {:?}",
        args
    );
}

// ---- Launch test ----

#[test]
fn test_launch_container_events() {
    let env = fixtures::install_env();
    let launch = launch_file();

    // Resolve to get expected process count (1 container)
    let (model, _tmp) = fixtures::resolve_model(&env, &launch, None, "rust");
    let expected = fixtures::count_expected_processes_from_model(&model);
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

/// Find the PID of a container process by matching its __node:= argument,
/// restricted to the given session ID to avoid cross-contamination between
/// concurrent tests (each test's ManagedProcess calls setsid()).
fn find_container_pid(
    node_name: &str,
    session_id: u32,
    timeout: std::time::Duration,
) -> Option<u32> {
    let pattern = format!("__node:={node_name}");
    let sid = session_id.to_string();
    let start = std::time::Instant::now();
    let poll = std::time::Duration::from_secs(1);

    loop {
        let output = std::process::Command::new("pgrep")
            .args(["-s", &sid, "-f", &pattern])
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
        Ok(out) if out.status.success() => String::from_utf8_lossy(&out.stdout)
            .lines()
            .filter_map(|line| line.trim().parse::<u32>().ok())
            .collect(),
        _ => Vec::new(),
    }
}

/// Wait until the given parent has at least `min_count` child processes,
/// or until `timeout` elapses.
fn wait_for_children(parent_pid: u32, min_count: usize, timeout: std::time::Duration) -> Vec<u32> {
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
    let output_file = std::fs::File::create(&output_path).expect("failed to create output file");
    let stderr_file = std::fs::File::create(&stderr_path).expect("failed to create stderr file");

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
    let session_id = _proc.id(); // setsid() makes play_launch the session leader

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

    // Find container PID via session-scoped pgrep (avoids cross-contamination
    // between concurrent tests that use the same container name)
    let container_pid = find_container_pid(
        "event_container",
        session_id,
        std::time::Duration::from_secs(10),
    )
    .expect("Failed to find container process 'event_container' via pgrep");

    eprintln!("Container PID: {container_pid}");

    // Find fork+exec child processes
    let children = wait_for_children(container_pid, 1, std::time::Duration::from_secs(10));
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

// ---- Data delivery tests ----

/// Search all `err` files under `play_log/node/` for a pattern.
/// Returns the matching content on success.
fn wait_for_container_log(
    play_log_dir: &std::path::Path,
    pattern: &str,
    timeout: Duration,
) -> Option<String> {
    let start = std::time::Instant::now();
    while start.elapsed() < timeout {
        std::thread::sleep(Duration::from_secs(1));
        // Scan both node/ (containers) and load_node/ (per-composable-node logs in isolated mode)
        for subdir in ["node", "load_node"] {
            let dir = play_log_dir.join(subdir);
            if let Ok(entries) = std::fs::read_dir(&dir) {
                for entry in entries.flatten() {
                    for filename in ["err", "out"] {
                        let log_file = entry.path().join(filename);
                        if let Ok(content) = std::fs::read_to_string(&log_file)
                            && content.contains(pattern)
                        {
                            return Some(content);
                        }
                    }
                }
            }
        }
    }
    None
}

/// Test that composable nodes can exchange data via DDS pub/sub in isolated mode.
///
/// Talker publishes "Hello World: N" every 1s to /chatter.
/// Listener subscribes to /chatter and logs "I heard: [Hello World: N]".
/// If "I heard:" appears in the container's log, data delivery works.
#[test]
fn test_isolated_data_delivery() {
    let env = fixtures::install_env();
    let launch = isolated_launch_file();

    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let work_dir = work_tmp.path();
    let output_path = work_dir.join("stdout.log");
    let stderr_path = work_dir.join("stderr.log");
    let output_file = std::fs::File::create(&output_path).expect("failed to create output file");
    let stderr_file = std::fs::File::create(&stderr_path).expect("failed to create stderr file");

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

    // Wait for both composable nodes to be loaded
    let loaded = wait_for_pattern(
        &output_path,
        &["ComponentEvent LOADED", "LoadSucceeded"],
        2,
        Duration::from_secs(30),
    );
    assert!(loaded >= 2, "Expected 2 LOADED events, found {loaded}");

    // Wait for data delivery: Talker publishes every 1s, Listener logs "I heard:"
    let play_log = work_dir.join("play_log/latest");
    let result = wait_for_container_log(&play_log, "I heard:", Duration::from_secs(15));

    // Collect diagnostics on failure
    if result.is_none() {
        eprintln!("--- Log files ---");
        for subdir in ["node", "load_node"] {
            let dir = play_log.join(subdir);
            if let Ok(entries) = std::fs::read_dir(&dir) {
                for entry in entries.flatten() {
                    for filename in ["err", "out"] {
                        let log_file = entry.path().join(filename);
                        if let Ok(content) = std::fs::read_to_string(&log_file) {
                            eprintln!("--- {} ({} bytes) ---", log_file.display(), content.len());
                            let snippet_start = content.len().saturating_sub(2000);
                            eprintln!("{}", &content[snippet_start..]);
                        }
                    }
                }
            }
        }
        let stdout = std::fs::read_to_string(&output_path).unwrap_or_default();
        let stderr = std::fs::read_to_string(&stderr_path).unwrap_or_default();
        eprintln!("--- play_launch stdout (last 1000) ---");
        let s = stdout.len().saturating_sub(1000);
        eprintln!("{}", &stdout[s..]);
        eprintln!("--- play_launch stderr (last 1000) ---");
        let s = stderr.len().saturating_sub(1000);
        eprintln!("{}", &stderr[s..]);
    }

    assert!(
        result.is_some(),
        "Listener did not receive any messages from Talker in isolated mode.\n\
         Expected 'I heard:' in container log within 15s of nodes being loaded.\n\
         This indicates DDS pub/sub data delivery is broken between fork+exec isolated children."
    );
}

/// Test that an external process can receive data from an isolated container.
///
/// This tests cross-process DDS communication: Talker inside a fork+exec isolated
/// child publishes to /chatter, and `ros2 topic echo` running as a separate
/// process subscribes. If this fails, DDS discovery/transport across process
/// boundaries is broken for isolated containers.
#[test]
fn test_isolated_external_subscriber() {
    let env = fixtures::install_env();
    let launch = isolated_launch_file();

    // The external subscriber has to join the SAME domain as the container, so
    // this test cannot let `play_launch_cmd` pick one silently — but it does
    // not need a *constant* either. Issue 0014: this was pinned to "199", so a
    // `ros2-daemon` left on domain 199 by any earlier run made the test fail
    // and blame DDS cross-process isolation. Same allocator as every other
    // test, just captured so it can be handed to `ros2` below.
    let domain_id = fixtures::next_domain_id().to_string();
    let domain_id = domain_id.as_str();

    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let work_dir = work_tmp.path();
    let output_path = work_dir.join("stdout.log");
    let stderr_path = work_dir.join("stderr.log");
    let output_file = std::fs::File::create(&output_path).expect("failed to create output file");
    let stderr_file = std::fs::File::create(&stderr_path).expect("failed to create stderr file");

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
    cmd.env("ROS_DOMAIN_ID", domain_id); // Override auto-generated domain

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    // Wait for both composable nodes to be loaded
    let loaded = wait_for_pattern(
        &output_path,
        &["ComponentEvent LOADED", "LoadSucceeded"],
        2,
        Duration::from_secs(30),
    );
    assert!(loaded >= 2, "Expected 2 LOADED events, found {loaded}");

    // Give DDS a moment to complete discovery
    std::thread::sleep(Duration::from_secs(2));

    // Run `ros2 topic echo` as an external subscriber with the same domain ID
    // and FastDDS profile (UDP-only, no SHM)
    let fastdds_profile = fixtures::repo_root().join("tests/fixtures/fastdds_no_shm.xml");

    // `--no-daemon` keeps the ros2 CLI from consulting (or spawning) a
    // long-lived daemon for this domain. With a per-invocation domain the
    // daemon is unlikely to exist, but a daemon is exactly what turned issue
    // 0014 into a phantom DDS failure, so this removes the vector rather than
    // relying on the domain being fresh.
    //
    // stderr is CAPTURED, not discarded. It used to be `2>/dev/null` on both
    // calls, which kept the success path quiet at the cost of making the
    // failure path undiagnosable: the block below printed three empty
    // sections while the assertion named a cause nothing had checked.
    let run_ros2 = |args: &str| -> std::process::Output {
        let mut c = std::process::Command::new("bash");
        c.env_clear();
        c.envs(&env);
        c.env("ROS_DOMAIN_ID", domain_id);
        if fastdds_profile.is_file() {
            c.env("FASTRTPS_DEFAULT_PROFILES_FILE", &fastdds_profile);
        }
        c.arg("-c").arg(args);
        c.output().expect("failed to run ros2")
    };

    let echo_output =
        run_ros2("timeout 15 ros2 topic echo /chatter std_msgs/msg/String --once --no-daemon");
    let echo_stdout = String::from_utf8_lossy(&echo_output.stdout);
    let echo_stderr = String::from_utf8_lossy(&echo_output.stderr);

    let mut diagnosis = String::new();
    if !echo_stdout.contains("Hello World") {
        let list_output = run_ros2("ros2 topic list --no-daemon");
        diagnosis = format!(
            "  domain: {domain_id}\n\
               ros2 topic echo exit: {:?}\n\
               ros2 topic echo stdout: {:?}\n\
               ros2 topic echo stderr: {:?}\n\
               ros2 topic list stdout: {:?}\n\
               ros2 topic list stderr: {:?}",
            echo_output.status,
            echo_stdout.trim(),
            echo_stderr.trim(),
            String::from_utf8_lossy(&list_output.stdout).trim(),
            String::from_utf8_lossy(&list_output.stderr).trim(),
        );
        eprintln!("--- external subscriber diagnostics ---\n{diagnosis}");
    }

    // States what was observed and lets the reader conclude. The old message
    // asserted "DDS cross-process communication is broken", which was wrong
    // every time issue 0014 fired -- the ros2 CLI was failing before any DDS
    // traffic was attempted, and a confident wrong cause costs more than no
    // cause at all.
    assert!(
        echo_stdout.contains("Hello World"),
        "External subscriber saw no 'Hello World' on /chatter from the isolated \
         container.\n{diagnosis}\n\
         If `ros2 topic list` is also empty or `ros2` exited non-zero, the CLI \
         itself failed and this says nothing about DDS. If the topic is listed \
         but no data arrives, cross-process delivery for fork+exec isolated \
         children is the likely cause."
    );
}

/// Test that composable nodes can exchange data in observable (non-isolated) mode.
/// This serves as a baseline — if this passes but isolated fails, the issue is
/// specific to fork+exec isolated isolation.
#[test]
fn test_observable_data_delivery() {
    let env = fixtures::install_env();
    let launch = launch_file(); // Non-isolated (observable mode)

    let work_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let work_dir = work_tmp.path();
    let output_path = work_dir.join("stdout.log");
    let stderr_path = work_dir.join("stderr.log");
    let output_file = std::fs::File::create(&output_path).expect("failed to create output file");
    let stderr_file = std::fs::File::create(&stderr_path).expect("failed to create stderr file");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_dir);
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--container-mode",
        "observable",
        &launch,
    ]);
    cmd.stdout(Stdio::from(output_file));
    cmd.stderr(Stdio::from(stderr_file));
    cmd.env("RUST_LOG", "play_launch=debug");

    let _proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    // Wait for both composable nodes to be loaded
    let loaded = wait_for_pattern(
        &output_path,
        &["ComponentEvent LOADED", "LoadSucceeded"],
        2,
        Duration::from_secs(30),
    );
    assert!(loaded >= 2, "Expected 2 LOADED events, found {loaded}");

    // Wait for data delivery — in observable mode logs go to the container's stderr
    let play_log = work_dir.join("play_log/latest");
    let result = wait_for_container_log(&play_log, "I heard:", Duration::from_secs(15));

    assert!(
        result.is_some(),
        "Listener did not receive messages from Talker in observable mode.\n\
         Expected 'I heard:' in container log. This is the baseline test — \n\
         if this fails, the issue is not specific to fork+exec isolated isolation."
    );
}
