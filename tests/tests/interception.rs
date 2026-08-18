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
    run_launch_with_config("launch/pure_nodes.launch.xml", config_yaml, run_duration)
}

/// As `run_with_config`, for a caller that needs a different launch file.
fn run_launch_with_config(
    launch_rel: &str,
    config_yaml: &str,
    run_duration: Duration,
) -> tempfile::TempDir {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("simple_test").join(launch_rel);

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
        stats.as_object().is_some_and(|m| !m.is_empty()),
        "stats_summary.json should have at least one topic entry, got: {}",
        stats_content
    );

    // Each entry should have pub_count > 0 (talker publishes ~1Hz for 5 seconds).
    // Skip metadata keys (Phase 42.0: `_events_dropped_total_best_effort` is a
    // top-level additive field, not a per-topic entry — see
    // docs/roadmap/phase-29-rcl_interception.md).
    for (hash, entry) in stats.as_object().unwrap() {
        if hash.starts_with('_') {
            continue;
        }
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

/// Phase 58 W2: the per-message record `play_launch measure` reads.
///
/// Asserts the two fields that were added for it — a per-thread CPU reading
/// and the thread it belongs to — actually arrive populated. A `cpu_ns` of 0
/// would still produce plausible-looking output downstream (every cost would
/// come out as 0 rather than as an error), so the check has to be here.
#[test]
fn test_interception_events_jsonl_carries_cpu_time() {
    let config = r#"
interception:
  enabled: true
  frontier: true
  stats: true
  ring_capacity: 4096
"#;

    let work_dir = run_with_config(config, Duration::from_secs(5));
    let play_log = work_dir.path().join("play_log/latest");

    let events_path = play_log.join("interception/events.jsonl");
    assert!(
        wait_for_file(&events_path, Duration::from_secs(3)),
        "events.jsonl not found at {}",
        events_path.display()
    );

    let body = std::fs::read_to_string(&events_path).expect("failed to read events.jsonl");
    let mut message_events = 0;
    let mut with_cpu = 0;
    let mut with_tid = 0;
    let mut topic_names = 0;
    for line in body.lines() {
        let Ok(v) = serde_json::from_str::<serde_json::Value>(line) else {
            // A truncated final line is expected: the file is written
            // streaming, so a shutdown can cut one in half.
            continue;
        };
        match v["r"].as_str() {
            Some("e") => {
                message_events += 1;
                if v["c"].as_u64().unwrap_or(0) > 0 {
                    with_cpu += 1;
                }
                if v["tid"].as_u64().unwrap_or(0) > 0 {
                    with_tid += 1;
                }
            }
            Some("t") => topic_names += 1,
            _ => {}
        }
    }

    assert!(
        message_events > 0,
        "events.jsonl has no publish/take records:\n{body}"
    );
    assert_eq!(
        with_cpu, message_events,
        "every publish/take must carry a CLOCK_THREAD_CPUTIME_ID reading; \
         {with_cpu}/{message_events} did"
    );
    assert_eq!(
        with_tid, message_events,
        "every publish/take must carry its producing thread; \
         {with_tid}/{message_events} did"
    );
    assert!(
        topic_names > 0,
        "no topic-name records — every event would be unattributable"
    );
}

/// The file is per-message data, so a run with interception off must not
/// leave one behind.
#[test]
fn test_interception_disabled_writes_no_events_jsonl() {
    let config = r#"
interception:
  enabled: false
"#;

    let work_dir = run_with_config(config, Duration::from_secs(3));
    let play_log = work_dir.path().join("play_log/latest");
    let events_path = play_log.join("interception/events.jsonl");
    assert!(
        !events_path.exists(),
        "events.jsonl written despite interception being disabled"
    );
}

/// Issue #0017 — the identity file reports the name a node ACTUALLY
/// registers, which for a node the launch file did not name is not its model
/// key.
///
/// The model keys an un-named `<node>` by its executable (plus the #0018
/// ordinal), while the process registers its compiled-in name. play_launch
/// cannot know the second from the launch file and must not impose the first
/// — forcing `-r __node:=` renames the node away from its own default, which
/// was bug `af7c524`. So the name is read from inside the process, where it
/// is simply available: the interceptor's publisher/subscription hooks
/// already hold the `rcl_node_t*`.
///
/// Asserted on the DIFFERENCE between the two nodes. A test that only checked
/// the un-named one would pass just as well if every key were reported wrong.
#[test]
fn test_interception_reports_real_node_names() {
    let work_dir = run_launch_with_config(
        "launch/unnamed_node.launch.xml",
        "interception:\n  enabled: true\n  stats: true\n",
        Duration::from_secs(5),
    );

    let identity = work_dir
        .path()
        .join("play_log/latest/interception/node_identity.tsv");
    assert!(
        identity.is_file(),
        "node_identity.tsv should be written when interception is enabled"
    );

    let text = std::fs::read_to_string(&identity).expect("read node_identity.tsv");
    let rows: Vec<(String, String)> = text
        .lines()
        .filter_map(|l| {
            let mut f = l.split('\t');
            let member = f.next()?.to_string();
            let _pid = f.next()?;
            let fqn = f.next()?.to_string();
            Some((member, fqn))
        })
        .collect();
    assert!(!rows.is_empty(), "no identity rows in {text:?}");

    // The named node: model key and ROS name are the same, and that is what
    // makes it safe to join against the graph.
    let named = rows
        .iter()
        .find(|(_, fqn)| fqn == "/identity_test/listener")
        .unwrap_or_else(|| panic!("no row for the named node in {rows:?}"));
    assert_eq!(
        named.0, named.1,
        "a node the launch file named must key by its ROS name"
    );

    // The un-named node: the key is executable-derived and is NOT the ROS
    // name. This is the whole issue.
    let unnamed = rows
        .iter()
        .find(|(_, fqn)| fqn == "/identity_test/talker")
        .unwrap_or_else(|| panic!("no row for the un-named node in {rows:?}"));
    assert_ne!(
        unnamed.0, unnamed.1,
        "an un-named node's model key is executable-derived, so it must NOT \
         equal its ROS name — if these are equal the fixture stopped \
         exercising the case"
    );
    assert!(
        unnamed.0.contains("talker"),
        "the model key should be executable-derived: {}",
        unnamed.0
    );
}
