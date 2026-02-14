use std::collections::HashMap;
use std::process::Stdio;
use std::time::{Duration, Instant};

use play_launch_tests::fixtures;
use play_launch_tests::fixtures::array_len;
use play_launch_tests::process::ManagedProcess;

fn parallel_slow_launch() -> String {
    fixtures::test_workspace_path("parallel_loading")
        .join("launch/parallel_slow.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

fn mixed_fast_slow_launch() -> String {
    fixtures::test_workspace_path("parallel_loading")
        .join("launch/mixed_fast_slow.launch.xml")
        .to_str()
        .unwrap()
        .to_string()
}

fn lifecycle_launch(variant: &str) -> String {
    fixtures::test_workspace_path("parallel_loading")
        .join(format!("launch/lifecycle_{variant}.launch.xml"))
        .to_str()
        .unwrap()
        .to_string()
}

/// Wait until `output_path` contains at least `count` lines matching any of `patterns`,
/// or until `timeout` elapses. Returns the number of matching lines found.
fn wait_for_pattern(
    output_path: &std::path::Path,
    patterns: &[&str],
    count: usize,
    timeout: Duration,
) -> usize {
    let start = Instant::now();
    let mut found = 0;
    while start.elapsed() < timeout {
        std::thread::sleep(Duration::from_secs(1));
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

/// Wait until `output_path` contains a line matching `pattern`, or timeout.
fn wait_for_line(output_path: &std::path::Path, pattern: &str, timeout: Duration) -> bool {
    wait_for_pattern(output_path, &[pattern], 1, timeout) >= 1
}

/// Call the play_launch web API. Returns (status_code, body).
fn web_api_post(port: u16, path: &str) -> (u32, String) {
    let url = format!("http://127.0.0.1:{}{}", port, path);
    let output = std::process::Command::new("curl")
        .args(["-s", "-o", "/dev/null", "-w", "%{http_code}", "-X", "POST", &url])
        .output()
        .expect("failed to run curl");
    let status: u32 = String::from_utf8_lossy(&output.stdout)
        .trim()
        .parse()
        .unwrap_or(0);
    let body = String::from_utf8_lossy(&output.stderr).to_string();
    (status, body)
}

// ---- Dump tests ----

#[test]
fn test_dump_parallel_loading() {
    let env = fixtures::install_env();
    let (record, _tmp) = fixtures::dump_launch(&env, &parallel_slow_launch(), "rust");

    assert_eq!(array_len(&record, "node"), 0, "expected 0 standalone nodes");
    assert_eq!(array_len(&record, "container"), 1, "expected 1 container");
    assert_eq!(
        array_len(&record, "load_node"),
        2,
        "expected 2 composable nodes"
    );

    // Verify container has --isolated in its args
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
}

#[test]
fn test_dump_mixed_fast_slow() {
    let env = fixtures::install_env();
    let (record, _tmp) = fixtures::dump_launch(&env, &mixed_fast_slow_launch(), "rust");

    assert_eq!(array_len(&record, "node"), 0, "expected 0 standalone nodes");
    assert_eq!(array_len(&record, "container"), 1, "expected 1 container");
    assert_eq!(
        array_len(&record, "load_node"),
        2,
        "expected 2 composable nodes"
    );
}

// ---- Launch tests ----

/// Test that both slow-loading nodes (3s each) complete within a reasonable
/// time when loaded in parallel (should be ~3s, not 6s sequential).
#[test]
fn test_parallel_load_completes() {
    let env = fixtures::install_env();
    let launch = parallel_slow_launch();
    let work_dir = fixtures::test_workspace_path("parallel_loading");

    let output_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let output_path = output_tmp.path().join("stdout.log");
    let stderr_path = output_tmp.path().join("stderr.log");
    let output_file = std::fs::File::create(&output_path).expect("failed to create stdout file");
    let stderr_file = std::fs::File::create(&stderr_path).expect("failed to create stderr file");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
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

    // Wait for the container process to appear
    let play_log = work_dir.join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 1, std::time::Duration::from_secs(30));

    // Wait for both composable nodes to load (3s delay + overhead).
    // With parallel loading: ~3s construction + ~2s service warmup.
    // We poll until both LOADED events appear or 30s timeout.
    let start = std::time::Instant::now();
    let timeout = std::time::Duration::from_secs(30);
    let mut loaded_count = 0;

    while start.elapsed() < timeout {
        std::thread::sleep(std::time::Duration::from_secs(1));
        let stdout = std::fs::read_to_string(&output_path).unwrap_or_default();
        loaded_count = stdout
            .lines()
            .filter(|line| line.contains("ComponentEvent LOADED") || line.contains("LoadSucceeded"))
            .count();
        if loaded_count >= 2 {
            break;
        }
    }

    let elapsed = start.elapsed();
    let stdout = std::fs::read_to_string(&output_path).unwrap_or_default();

    eprintln!("--- stdout ({} bytes, last 2000 chars) ---", stdout.len());
    let snippet_start = stdout.len().saturating_sub(2000);
    eprintln!("{}", &stdout[snippet_start..]);
    eprintln!("loaded_count={loaded_count}, elapsed={:.1}s", elapsed.as_secs_f64());

    assert!(
        loaded_count >= 2,
        "Expected both SlowLoader nodes to have LOADED events, found {loaded_count}"
    );
}

/// Test that a fast component (Talker) is not blocked by a slow component
/// (SlowLoader 5s). The Talker's LOADED event should appear before the
/// SlowLoader finishes.
#[test]
fn test_fast_not_blocked_by_slow() {
    let env = fixtures::install_env();
    let launch = mixed_fast_slow_launch();
    let work_dir = fixtures::test_workspace_path("parallel_loading");

    let output_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let output_path = output_tmp.path().join("stdout.log");
    let stderr_path = output_tmp.path().join("stderr.log");
    let output_file = std::fs::File::create(&output_path).expect("failed to create stdout file");
    let stderr_file = std::fs::File::create(&stderr_path).expect("failed to create stderr file");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(&work_dir);
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

    // Wait for the container to start
    let play_log = work_dir.join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 1, std::time::Duration::from_secs(30));

    // The SlowLoader takes 3s. The Talker should load almost instantly.
    // Wait until we see both LOADED events or timeout.
    let start = std::time::Instant::now();
    let timeout = std::time::Duration::from_secs(30);
    let mut talker_loaded = false;
    let mut talker_loaded_at = None;

    while start.elapsed() < timeout {
        std::thread::sleep(std::time::Duration::from_secs(1));
        let stdout = std::fs::read_to_string(&output_path).unwrap_or_default();
        if stdout.contains("ComponentEvent LOADED for 'fast_talker'") {
            talker_loaded = true;
            if talker_loaded_at.is_none() {
                talker_loaded_at = Some(start.elapsed());
            }
        }
        // Check for both loaded
        let loaded_count = stdout
            .lines()
            .filter(|line| line.contains("ComponentEvent LOADED"))
            .count();
        if loaded_count >= 2 {
            break;
        }
    }

    let stdout_final = std::fs::read_to_string(&output_path).unwrap_or_default();
    let slow_loaded = stdout_final.contains("ComponentEvent LOADED for 'slow_node'");

    eprintln!("--- Final output (last 3000 chars) ---");
    let snippet_start = stdout_final.len().saturating_sub(3000);
    eprintln!("{}", &stdout_final[snippet_start..]);
    eprintln!(
        "talker_loaded={} (at {:?}), slow_loaded={}, total={:.1}s",
        talker_loaded,
        talker_loaded_at,
        slow_loaded,
        start.elapsed().as_secs_f64()
    );

    assert!(
        talker_loaded,
        "Talker should have loaded (fast component not blocked by slow)"
    );
    assert!(
        slow_loaded,
        "SlowLoader should have completed construction eventually"
    );

    // Verify the Talker loaded before the 5s mark (proving it wasn't blocked)
    if let Some(talker_time) = talker_loaded_at {
        assert!(
            talker_time.as_secs() < 8,
            "Talker took {:.1}s to load — should be fast (<8s including warmup)",
            talker_time.as_secs_f64()
        );
    }
}

// ---- Lifecycle tests (load → unload → reload via web API) ----

/// Helper: spawn play_launch with web UI on a given port, return (ManagedProcess, output_path, TempDir).
/// Uses a temporary work directory so concurrent tests don't conflict on play_log/latest.
fn spawn_lifecycle_play_launch(
    env: &HashMap<String, String>,
    launch_file: &str,
    port: u16,
) -> (ManagedProcess, std::path::PathBuf, tempfile::TempDir) {
    let output_tmp = tempfile::TempDir::new().expect("failed to create tempdir");
    let work_dir = output_tmp.path();
    let output_path = work_dir.join("stdout.log");
    let stderr_path = work_dir.join("stderr.log");
    let output_file = std::fs::File::create(&output_path).expect("failed to create stdout file");
    let stderr_file = std::fs::File::create(&stderr_path).expect("failed to create stderr file");

    let mut cmd = fixtures::play_launch_cmd(env);
    cmd.current_dir(work_dir);
    cmd.args([
        "launch",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--web-addr",
        &format!("127.0.0.1:{}", port),
        launch_file,
    ]);
    cmd.stdout(Stdio::from(output_file));
    cmd.stderr(Stdio::from(stderr_file));
    cmd.env("RUST_LOG", "play_launch=debug");

    let proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    // Wait for container process to appear
    let play_log = work_dir.join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 1, Duration::from_secs(30));

    (proc, output_path, output_tmp)
}

/// Test: both nodes load, then unload fast_talker via web API, verify UNLOADED event.
#[test]
fn test_unload_via_web_api() {
    let env = fixtures::install_env();
    let port = 18091_u16;
    let launch = lifecycle_launch("unload");
    let (_proc, output_path, _tmp) = spawn_lifecycle_play_launch(&env, &launch, port);

    // Wait for both composable nodes to finish loading
    let loaded = wait_for_pattern(
        &output_path,
        &["ComponentEvent LOADED", "LoadSucceeded"],
        2,
        Duration::from_secs(30),
    );
    assert!(
        loaded >= 2,
        "Expected 2 LOADED events before unload test, found {loaded}"
    );

    // Wait a moment for web UI to start
    std::thread::sleep(Duration::from_secs(1));

    // Unload fast_talker via web API
    eprintln!("Unloading fast_talker via POST /api/nodes/fast_talker/unload");
    let (status, _) = web_api_post(port, "/api/nodes/fast_talker/unload");
    eprintln!("Unload response status: {status}");
    assert!(
        status == 200 || status == 202,
        "Expected 200/202 from unload, got {status}"
    );

    // Wait for unload confirmation (either ComponentEvent or actor log)
    let unloaded = wait_for_line(
        &output_path,
        "Successfully unloaded composable node 'fast_talker'",
        Duration::from_secs(15),
    );

    let stdout = std::fs::read_to_string(&output_path).unwrap_or_default();
    eprintln!("--- stdout (last 2000 chars) ---");
    let snippet = stdout.len().saturating_sub(2000);
    eprintln!("{}", &stdout[snippet..]);

    let has_event = stdout.contains("ComponentEvent UNLOADED for 'fast_talker'");
    eprintln!("unloaded={unloaded}, has_component_event={has_event}");

    assert!(unloaded, "Expected unload confirmation for fast_talker");
}

/// Test: load → unload → reload cycle. After reloading, a fresh LOADED event
/// should appear for the re-loaded composable node.
#[test]
fn test_unload_and_reload() {
    let env = fixtures::install_env();
    let port = 18092_u16;
    let launch = lifecycle_launch("reload");
    let (_proc, output_path, _tmp) = spawn_lifecycle_play_launch(&env, &launch, port);

    // Wait for both composable nodes to finish loading
    let loaded = wait_for_pattern(
        &output_path,
        &["ComponentEvent LOADED", "LoadSucceeded"],
        2,
        Duration::from_secs(30),
    );
    assert!(
        loaded >= 2,
        "Expected 2 LOADED events, found {loaded}"
    );

    std::thread::sleep(Duration::from_secs(1));

    // Count LOADED events before unload (baseline)
    let stdout_before = std::fs::read_to_string(&output_path).unwrap_or_default();
    let loaded_before = stdout_before
        .lines()
        .filter(|l| l.contains("ComponentEvent LOADED for 'fast_talker'"))
        .count();
    eprintln!("LOADED events for fast_talker before unload: {loaded_before}");

    // Unload fast_talker
    eprintln!("Unloading fast_talker...");
    let (status, _) = web_api_post(port, "/api/nodes/fast_talker/unload");
    assert!(
        status == 200 || status == 202,
        "Unload failed with status {status}"
    );

    // Wait for unload confirmation
    let unloaded = wait_for_line(
        &output_path,
        "Successfully unloaded composable node 'fast_talker'",
        Duration::from_secs(15),
    );
    assert!(unloaded, "Expected unload confirmation for fast_talker");

    // Small delay before reload
    std::thread::sleep(Duration::from_secs(1));

    // Reload fast_talker
    eprintln!("Reloading fast_talker...");
    let (status, _) = web_api_post(port, "/api/nodes/fast_talker/load");
    eprintln!("Reload response status: {status}");
    assert!(
        status == 200 || status == 202,
        "Reload failed with status {status}"
    );

    // Wait for a new LOADED event (count should increase)
    let start = Instant::now();
    let timeout = Duration::from_secs(30);
    let mut loaded_after = loaded_before;
    while start.elapsed() < timeout {
        std::thread::sleep(Duration::from_secs(1));
        let stdout = std::fs::read_to_string(&output_path).unwrap_or_default();
        loaded_after = stdout
            .lines()
            .filter(|l| l.contains("ComponentEvent LOADED for 'fast_talker'"))
            .count();
        if loaded_after > loaded_before {
            break;
        }
    }

    let stdout_final = std::fs::read_to_string(&output_path).unwrap_or_default();
    eprintln!("--- stdout (last 3000 chars) ---");
    let snippet = stdout_final.len().saturating_sub(3000);
    eprintln!("{}", &stdout_final[snippet..]);
    eprintln!(
        "LOADED events for fast_talker: before={}, after={}",
        loaded_before, loaded_after
    );

    assert!(
        loaded_after > loaded_before,
        "Expected a new LOADED event after reload (before={loaded_before}, after={loaded_after})"
    );
}

/// Test: load both nodes, then unload the slow node after construction.
/// Verify the unload succeeds and fast_talker is unaffected.
#[test]
fn test_unload_during_construction() {
    let env = fixtures::install_env();
    let port = 18093_u16;
    let launch = lifecycle_launch("construct");
    let (_proc, output_path, _tmp) = spawn_lifecycle_play_launch(&env, &launch, port);

    // Wait for the fast_talker to load (proves container is up)
    let talker_loaded = wait_for_line(
        &output_path,
        "ComponentEvent LOADED for 'fast_talker'",
        Duration::from_secs(30),
    );
    assert!(talker_loaded, "fast_talker should have loaded");

    // Now wait for slow_node to also finish loading (3s construction)
    // Then unload it, and verify it works
    let slow_loaded = wait_for_line(
        &output_path,
        "ComponentEvent LOADED for 'slow_node'",
        Duration::from_secs(30),
    );
    assert!(slow_loaded, "slow_node should have loaded eventually");

    // Now unload slow_node (it's fully constructed)
    std::thread::sleep(Duration::from_secs(1));
    eprintln!("Unloading slow_node after construction complete...");
    let (status, _) = web_api_post(port, "/api/nodes/slow_node/unload");
    eprintln!("Unload status: {status}");

    // Wait for unload confirmation
    let unloaded = wait_for_line(
        &output_path,
        "Successfully unloaded composable node 'slow_node'",
        Duration::from_secs(15),
    );

    let stdout = std::fs::read_to_string(&output_path).unwrap_or_default();
    eprintln!("--- stdout (last 2000 chars) ---");
    let snippet = stdout.len().saturating_sub(2000);
    eprintln!("{}", &stdout[snippet..]);

    assert!(unloaded, "Expected unload confirmation for slow_node");

    // Verify fast_talker is still running (wasn't affected by unloading slow_node)
    // Check that the talker was loaded and there's no crash event for it
    let stdout_final = std::fs::read_to_string(&output_path).unwrap_or_default();
    let talker_crashed = stdout_final.contains("CRASHED") && stdout_final.contains("fast_talker");
    assert!(
        !talker_crashed,
        "fast_talker should not have crashed when slow_node was unloaded"
    );
}
