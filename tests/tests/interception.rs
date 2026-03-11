//! Integration tests for RCL interception (Phase 29).
//!
//! Tests verify that when interception is enabled via config YAML:
//! - LD_PRELOAD is injected into child processes
//! - The interception consumer task runs and processes events
//! - Summary files (frontier_summary.json, stats_summary.json) are written
//! - When disabled, no interception artifacts are created

use play_launch_tests::fixtures;
use play_launch_tests::process::ManagedProcess;
use std::path::Path;
use std::process::Stdio;
use std::time::Duration;

/// Path to the interception .so (built by `just build-interception`).
fn interception_so_path() -> std::path::PathBuf {
    let release = fixtures::repo_root()
        .join("src/play_launch_interception/target/release/libplay_launch_interception.so");
    if release.is_file() {
        return release;
    }
    let debug = fixtures::repo_root()
        .join("src/play_launch_interception/target/debug/libplay_launch_interception.so");
    if debug.is_file() {
        return debug;
    }
    panic!(
        "libplay_launch_interception.so not found. Run `just build-interception` first.\n\
         Searched:\n  {}\n  {}",
        release.display(),
        debug.display()
    );
}

/// Write a temporary config YAML file and return its path.
fn write_config(dir: &Path, content: &str) -> std::path::PathBuf {
    let config_path = dir.join("interception_test_config.yaml");
    std::fs::write(&config_path, content).expect("failed to write config YAML");
    config_path
}

/// Wait for a file to appear, polling every 500ms up to `timeout`.
fn wait_for_file(path: &Path, timeout: Duration) -> bool {
    let start = std::time::Instant::now();
    while start.elapsed() < timeout {
        if path.is_file() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(500));
    }
    false
}

/// Launch pure_nodes, let it run for `run_duration`, then drop to trigger
/// graceful shutdown. Returns the play_log directory.
fn run_with_config(config_yaml: &str, run_duration: Duration) -> tempfile::TempDir {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("simple_test")
        .join("launch/pure_nodes.launch.xml");

    let work_dir = tempfile::TempDir::new().expect("failed to create work dir");

    // Write config
    let config_path = write_config(work_dir.path(), config_yaml);

    // Set interception .so path via env var so play_launch finds it
    let so_path = interception_so_path();

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--container-mode",
        "stock",
        "--config",
        config_path.to_str().unwrap(),
        launch.to_str().unwrap(),
    ]);
    cmd.env("PLAY_LAUNCH_INTERCEPTION_SO", &so_path);
    cmd.env("RUST_LOG", "play_launch=debug");
    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());

    let proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    // Wait for processes to start
    let play_log = work_dir.path().join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 2, Duration::from_secs(15));

    // Let the talker run for a bit to generate events
    std::thread::sleep(run_duration);

    // Graceful shutdown — ManagedProcess sends SIGTERM, waits 2s
    drop(proc);

    // Give a moment for async shutdown to complete (summary file writing)
    std::thread::sleep(Duration::from_millis(500));

    work_dir
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// Test that interception enabled produces stats_summary.json with data.
#[test]
fn test_interception_stats_written() {
    let config = r#"
interception:
  enabled: true
  frontier: true
  stats: true
  ring_capacity: 4096
"#;

    let work_dir = run_with_config(config, Duration::from_secs(5));
    let play_log = work_dir.path().join("play_log/latest");

    // stats_summary.json should exist and be non-empty
    let stats_path = play_log.join("interception/stats_summary.json");
    assert!(
        wait_for_file(&stats_path, Duration::from_secs(3)),
        "stats_summary.json not found at {}",
        stats_path.display()
    );

    let stats_content = std::fs::read_to_string(&stats_path).expect("failed to read stats");
    let stats: serde_json::Value =
        serde_json::from_str(&stats_content).expect("invalid JSON in stats_summary.json");

    // Should be an object with at least one topic
    assert!(
        stats.as_object().map_or(false, |m| !m.is_empty()),
        "stats_summary.json should have at least one topic entry, got: {}",
        stats_content
    );

    // Each entry should have pub_count > 0 (talker publishes ~1Hz for 5 seconds)
    for (hash, entry) in stats.as_object().unwrap() {
        let pub_count = entry.get("pub_count").and_then(|v| v.as_u64()).unwrap_or(0);
        eprintln!("  topic_hash={hash}: pub_count={pub_count}");
        assert!(
            pub_count > 0,
            "topic {hash} should have pub_count > 0, got {pub_count}"
        );
    }
}

/// Test that interception disabled produces no interception directory.
#[test]
fn test_interception_disabled_no_artifacts() {
    let config = r#"
interception:
  enabled: false
"#;

    let work_dir = run_with_config(config, Duration::from_secs(3));
    let play_log = work_dir.path().join("play_log/latest");

    let interception_dir = play_log.join("interception");
    assert!(
        !interception_dir.exists(),
        "interception/ directory should not exist when disabled, but found at {}",
        interception_dir.display()
    );
}

/// Test that default config (no interception section) produces no interception
/// artifacts (interception.enabled defaults to false).
#[test]
fn test_interception_default_disabled() {
    let config = "# empty config\n";

    let work_dir = run_with_config(config, Duration::from_secs(3));
    let play_log = work_dir.path().join("play_log/latest");

    let interception_dir = play_log.join("interception");
    assert!(
        !interception_dir.exists(),
        "interception/ directory should not exist with default config"
    );
}

/// Test stats-only mode (frontier disabled).
#[test]
fn test_interception_stats_only() {
    let config = r#"
interception:
  enabled: true
  frontier: false
  stats: true
"#;

    let work_dir = run_with_config(config, Duration::from_secs(5));
    let play_log = work_dir.path().join("play_log/latest");

    // stats_summary.json should exist
    let stats_path = play_log.join("interception/stats_summary.json");
    assert!(
        wait_for_file(&stats_path, Duration::from_secs(3)),
        "stats_summary.json should exist in stats-only mode"
    );

    // frontier_summary.json should NOT exist (frontier disabled)
    let frontier_path = play_log.join("interception/frontier_summary.json");
    assert!(
        !frontier_path.exists(),
        "frontier_summary.json should not exist when frontier is disabled"
    );
}

/// Test frontier-only mode (stats disabled).
#[test]
fn test_interception_frontier_only() {
    let config = r#"
interception:
  enabled: true
  frontier: true
  stats: false
"#;

    let work_dir = run_with_config(config, Duration::from_secs(5));
    let play_log = work_dir.path().join("play_log/latest");

    // For std_msgs/String (no header.stamp), frontier tracking won't produce entries
    // because there's no stamp to track. So frontier_summary.json may not exist.
    // But stats_summary.json should NOT exist (stats disabled).
    let stats_path = play_log.join("interception/stats_summary.json");
    assert!(
        !stats_path.exists(),
        "stats_summary.json should not exist when stats is disabled"
    );
}
