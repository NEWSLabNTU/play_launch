//! Integration tests for RCL interception (Phase 29).
//!
//! Tests verify that when interception is enabled via config YAML:
//! - LD_PRELOAD is injected into child processes
//! - The interception consumer task runs and processes events
//! - Summary files (frontier_summary.json, stats_summary.json) are written
//! - When disabled, no interception artifacts are created

use play_launch_tests::{fixtures, process::ManagedProcess};
use std::{path::Path, process::Stdio, time::Duration};

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
        "stats_summary.json should have at least one topic entry, got: {stats_content}"
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

/// An empty config and no contract leave interception OFF: `--enforce-rules`
/// defaults to `warn`, which implies an event source only when a contract
/// resolves (issue #0031 covers that half, in `runtime_enforcement.rs`). With
/// nothing to enforce the run matches `ros2 launch` -- no LD_PRELOAD layer,
/// nothing written per message. Intercepting here anyway once logged 40 GB
/// in five minutes for an Autoware stack.
#[test]
fn test_interception_off_without_contract_under_default_enforce_mode() {
    let config = "# empty config\n";

    let work_dir = run_with_config(config, Duration::from_secs(3));
    let play_log = work_dir.path().join("play_log/latest");

    let interception_dir = play_log.join("interception");
    assert!(
        !interception_dir.exists(),
        "no contract resolved, so the default run must not intercept; found {}",
        interception_dir.display()
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
  events: true
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

/// Phase 77 — the interceptor must name a REMAPPED topic the way the running
/// system does.
///
/// `expand_topic_name` used to expand `chatter` to `/pure_test/chatter` and
/// then hand it to a remap step that could never run: the step needed
/// `rcl_get_global_arguments`, which is not a symbol rcl exports (it appears
/// once in `rcl/remap.h`, inside a doc comment), so the whole optional group
/// resolved to `None` on every installation and the expanded name was used
/// unchanged. Every consumer keyed by topic — the frontier and stats plugins,
/// the Chrome trace, `play_launch measure`, and the graph verification this
/// test was written alongside — therefore named remapped topics wrongly, and
/// silently.
///
/// It survived because no fixture in the tree had a remap. This one does, and
/// it asserts BOTH directions: the remapped name present, and the un-remapped
/// name absent. Asserting only the first would pass on a build that recorded
/// both.
#[test]
fn test_interception_reports_remapped_topic_names() {
    let config = r#"
interception:
  enabled: true
  stats: true
"#;

    let work_dir = run_launch_with_config(
        "launch/remapped_topic.launch.xml",
        config,
        Duration::from_secs(6),
    );
    let play_log = work_dir.path().join("play_log/latest");

    let endpoints = play_log.join("interception/endpoints.tsv");
    assert!(
        wait_for_file(&endpoints, Duration::from_secs(5)),
        "endpoints.tsv not found at {}",
        endpoints.display()
    );

    let body = std::fs::read_to_string(&endpoints).expect("failed to read endpoints.tsv");
    let rows: Vec<(String, String, String)> = body
        .lines()
        .filter_map(|line| {
            let mut it = line.split('\t');
            let _member = it.next()?;
            let _pid = it.next()?;
            let node = it.next()?;
            let direction = it.next()?;
            let topic = it.next()?;
            Some((node.to_string(), direction.to_string(), topic.to_string()))
        })
        .collect();
    assert!(!rows.is_empty(), "endpoints.tsv had no usable rows");

    let has = |node: &str, direction: &str, topic: &str| {
        rows.iter()
            .any(|(n, d, t)| n == node && d == direction && t == topic)
    };

    assert!(
        has("/pure_test/talker", "pub", "/renamed_chatter"),
        "the talker's publisher must be recorded under the REMAPPED name; \
         rows were {rows:?}"
    );
    assert!(
        has("/pure_test/listener", "sub", "/renamed_chatter"),
        "the listener's subscription must be recorded under the REMAPPED \
         name; rows were {rows:?}"
    );
    assert!(
        !rows.iter().any(|(_, _, t)| t == "/pure_test/chatter"),
        "the un-remapped name must not appear at all — the running nodes are \
         not wired to it; rows were {rows:?}"
    );
}

/// Issue #0047 — an endpoint's message TYPE must reach disk even when no
/// message ever crosses it.
///
/// The type is resolved at `rcl_publisher_init` / `rcl_subscription_init`, from
/// the `type_support` argument, and used to be dropped there: the only place a
/// type reached disk was `stats_summary.json` / `frontier_summary.json`, which
/// are keyed by TRAFFIC. So a publisher that was created and never used had no
/// type anywhere, and `type:` is mandatory in the manifest grammar — which
/// bounded `contract capture` to whatever the run happened to
/// exercise. Phase 77 measured the gap: 982 endpoints created, 63 carrying a
/// message, on one Autoware run.
///
/// The fixture's `silent_publisher.py` advertises `/silent_topic` and
/// subscribes `/silent_input` and sends nothing through either, so this test
/// asserts BOTH halves: the type is on the endpoint record, and the topic is
/// absent from the traffic summary. Asserting only the first would pass on a
/// build that had merely learned to copy the summary's answer.
#[test]
fn test_endpoints_record_message_type_without_traffic() {
    let config = r#"
interception:
  enabled: true
  stats: true
"#;

    let work_dir = run_launch_with_config(
        "launch/silent_endpoint.launch.xml",
        config,
        Duration::from_secs(8),
    );
    let play_log = work_dir.path().join("play_log/latest");

    let endpoints = play_log.join("interception/endpoints.tsv");
    assert!(
        wait_for_file(&endpoints, Duration::from_secs(5)),
        "endpoints.tsv not found at {}",
        endpoints.display()
    );
    let body = std::fs::read_to_string(&endpoints).expect("failed to read endpoints.tsv");

    // Every row is six fields. The type field is written even when
    // introspection cannot answer, precisely so a reader can discriminate the
    // pre-#0047 five-column format by COLUMN COUNT; a column that appeared
    // only sometimes would make the two widths ambiguous.
    for line in body.lines().filter(|l| !l.is_empty()) {
        assert_eq!(
            line.split('\t').count(),
            6,
            "every endpoint row carries a type field: {line:?}"
        );
    }

    let rows: Vec<(String, String, String, String)> = body
        .lines()
        .filter_map(|line| {
            let mut it = line.split('\t');
            let _member = it.next()?;
            let _pid = it.next()?;
            let node = it.next()?;
            let direction = it.next()?;
            let topic = it.next()?;
            let msg_type = it.next()?;
            Some((
                node.to_string(),
                direction.to_string(),
                topic.to_string(),
                msg_type.to_string(),
            ))
        })
        .collect();
    assert!(!rows.is_empty(), "endpoints.tsv had no usable rows");

    let typed = |node: &str, direction: &str, topic: &str| -> Option<String> {
        rows.iter()
            .find(|(n, d, t, _)| n == node && d == direction && t == topic)
            .map(|(_, _, _, ty)| ty.clone())
    };

    assert_eq!(
        typed("/silent_pub", "pub", "/silent_topic").as_deref(),
        Some("std_msgs/msg/String"),
        "a publisher that never published must still carry its type; rows were {rows:?}"
    );
    assert_eq!(
        typed("/silent_pub", "sub", "/silent_input").as_deref(),
        Some("std_msgs/msg/Int32"),
        "a subscription that never received must still carry its type; rows were {rows:?}"
    );
    // The exercised topic, for contrast: this one the summaries could also
    // describe, and it must not have regressed.
    assert_eq!(
        typed("/silent_test/talker", "pub", "/silent_test/chatter").as_deref(),
        Some("std_msgs/msg/String"),
        "rows were {rows:?}"
    );

    // The other half of the claim: the silent topics are NOT in the
    // traffic-keyed summary, so the endpoint record is the only source for
    // them. If this ever fails the fixture has stopped exercising the case.
    let stats = play_log.join("interception/stats_summary.json");
    assert!(
        wait_for_file(&stats, Duration::from_secs(5)),
        "stats_summary.json not found at {}",
        stats.display()
    );
    let stats_body = std::fs::read_to_string(&stats).expect("failed to read stats_summary.json");
    assert!(
        !stats_body.contains("/silent_topic"),
        "a topic no message crossed must not appear in the traffic summary — \
         the fixture is no longer silent: {stats_body}"
    );
    assert!(
        stats_body.contains("/silent_test/chatter"),
        "the exercised topic should be in the traffic summary: {stats_body}"
    );
}

/// `endpoint_stats.json` has a row for every publisher and subscription the
/// run created, counted over every path rcl moves a message on.
///
/// `message_paths.launch.xml` exercises:
/// - the SERIALIZED pair (`demo_nodes_cpp`'s `*_serialized_message`), which
///   the interceptor used not to hook at all, so `/serialized/chatter` was
///   absent from every stats file on both ends;
/// - `/rosout`, which rcl publishes through a by-value COPY of the publisher
///   handle, so a registry keyed on the handle's address lost every message;
/// - endpoints that never carry a message, which a traffic-keyed summary
///   cannot list at all.
///
/// The loaned talker's `chatter_pod` is asserted counted whichever path it
/// took: Fast DDS on Humble declines the loan and rclcpp falls back to
/// `rcl_publish`, so the loaned hooks themselves are not reached here.
#[test]
fn test_endpoint_stats_cover_every_message_path() {
    let config = r#"
interception:
  enabled: true
  stats: true
"#;

    let work_dir = run_launch_with_config(
        "launch/message_paths.launch.xml",
        config,
        Duration::from_secs(8),
    );
    let play_log = work_dir.path().join("play_log/latest");

    let path = play_log.join("interception/endpoint_stats.json");
    assert!(
        wait_for_file(&path, Duration::from_secs(5)),
        "endpoint_stats.json not found at {}",
        path.display()
    );
    let body = std::fs::read_to_string(&path).expect("failed to read endpoint_stats.json");
    let rows: Vec<serde_json::Value> = serde_json::from_str(&body).expect("invalid JSON");

    let row = |node: &str, direction: &str, topic: &str| -> &serde_json::Value {
        rows.iter()
            .find(|r| r["node"] == node && r["direction"] == direction && r["topic"] == topic)
            .unwrap_or_else(|| panic!("no {direction} row for {node} on {topic}: {body}"))
    };
    let messages = |r: &serde_json::Value| r["messages"].as_u64().unwrap();

    // Serialized: counted on both ends, and sized exactly on every message.
    let ser_pub = row("/serialized/talker", "pub", "/serialized/chatter");
    let ser_sub = row("/serialized/listener", "sub", "/serialized/chatter");
    assert!(
        messages(ser_pub) >= 3,
        "serialized publishes not counted: {ser_pub}"
    );
    assert!(
        messages(ser_sub) >= 3,
        "serialized takes not counted: {ser_sub}"
    );
    assert_eq!(
        ser_pub["size_samples"], ser_pub["messages"],
        "a serialized message's size is in hand, so every one is measured: {ser_pub}"
    );
    assert!(
        ser_pub["mean_message_bytes"].as_f64().unwrap() > 0.0,
        "{ser_pub}"
    );

    // Typed: counted, and sized by sampling.
    let typed = row("/loaned/talker", "pub", "/loaned/chatter_pod");
    assert!(messages(typed) >= 3, "{typed}");
    let sampled = typed["size_samples"].as_u64().unwrap();
    assert!(
        sampled >= 1 && sampled <= messages(typed),
        "a typed endpoint is sized by sampling: {typed}"
    );
    assert!(typed["est_bytes_per_s"].as_f64().unwrap() > 0.0, "{typed}");

    // /rosout: rcl publishes through a copy of the handle.
    let rosout = row("/loaned/talker", "pub", "/rosout");
    assert!(
        messages(rosout) >= 3,
        "the talker logs every publish, so its /rosout must carry them: {rosout}"
    );

    // Silent endpoints are listed, with nothing through them.
    for (direction, topic) in [("pub", "/silent_topic"), ("sub", "/silent_input")] {
        let r = row("/silent_pub", direction, topic);
        assert_eq!(messages(r), 0, "{r}");
        assert!(r.get("since_last_message_s").is_none(), "{r}");
    }
}

/// `events.jsonl` is opt-in: interception and `stats` on, `events` unset,
/// and the summaries are written while the per-message record is not.
#[test]
fn test_events_jsonl_is_off_by_default() {
    let config = r#"
interception:
  enabled: true
  stats: true
"#;

    let work_dir = run_with_config(config, Duration::from_secs(4));
    let play_log = work_dir.path().join("play_log/latest");
    let stats = play_log.join("interception/stats_summary.json");
    assert!(
        wait_for_file(&stats, Duration::from_secs(3)),
        "interception ran, so stats_summary.json must exist: {}",
        stats.display()
    );
    assert!(
        !play_log.join("interception/events.jsonl").exists(),
        "events.jsonl written although `events` was never asked for"
    );
}

/// One HTTP/1.1 request to the web UI; returns (status, body).
fn http(port: u16, method: &str, path: &str, body: Option<&str>) -> (u16, String) {
    use std::io::{Read, Write};
    let mut stream =
        std::net::TcpStream::connect(("127.0.0.1", port)).expect("connect to the web UI");
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let body = body.unwrap_or("");
    write!(
        stream,
        "{method} {path} HTTP/1.1\r\nHost: 127.0.0.1\r\nContent-Type: application/json\r\n\
         Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
        body.len()
    )
    .unwrap();
    let mut response = String::new();
    stream.read_to_string(&mut response).unwrap();
    let status = response
        .split_whitespace()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let body = response
        .split_once("\r\n\r\n")
        .map(|(_, b)| b.to_string())
        .unwrap_or_default();
    (status, body)
}

fn event_lines(path: &Path) -> Vec<String> {
    std::fs::read_to_string(path)
        .unwrap_or_default()
        .lines()
        .map(str::to_string)
        .collect()
}

/// Recording switches on, off and on again while the run goes, through
/// `/api/interception/events` (what the web UI's toggle calls). Off writes
/// nothing; turning it back on appends, and the new segment carries its own
/// topic-name records so `measure` can read it alone.
#[test]
fn test_event_recording_switches_at_runtime() {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("simple_test").join("launch/pure_nodes.launch.xml");
    let work_dir = tempfile::TempDir::new().expect("failed to create work dir");
    let config_path = write_config(
        work_dir.path(),
        "interception:\n  enabled: true\n  stats: true\n",
    );
    let port = std::net::TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port();

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--web-addr",
        &format!("127.0.0.1:{port}"),
        "--disable-monitoring",
        "--disable-diagnostics",
        "--container-mode",
        "stock",
        "--config",
        config_path.to_str().unwrap(),
        launch.to_str().unwrap(),
    ]);
    cmd.env("PLAY_LAUNCH_INTERCEPTION_SO", interception_so_path());
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());
    let proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let play_log = work_dir.path().join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 2, Duration::from_secs(15));
    let events = play_log.join("interception/events.jsonl");

    let (status, state) = http(port, "GET", "/api/interception/events", None);
    assert_eq!(status, 200, "{state}");
    let state: serde_json::Value = serde_json::from_str(&state).unwrap();
    assert_eq!(state["available"], true, "{state}");
    assert_eq!(state["recording"], false, "off by default: {state}");
    std::thread::sleep(Duration::from_secs(2));
    assert!(
        !events.exists(),
        "nothing may be recorded before it is asked for"
    );

    // On.
    let (status, body) = http(
        port,
        "POST",
        "/api/interception/events",
        Some(r#"{"enabled":true}"#),
    );
    assert_eq!(status, 200, "{body}");
    std::thread::sleep(Duration::from_secs(3));
    let (_, state) = http(port, "GET", "/api/interception/events", None);
    let state: serde_json::Value = serde_json::from_str(&state).unwrap();
    assert_eq!(state["recording"], true, "{state}");
    assert!(state["records"].as_u64().unwrap() > 0, "{state}");

    // Off: the file is flushed and stops growing.
    let (status, body) = http(
        port,
        "POST",
        "/api/interception/events",
        Some(r#"{"enabled":false}"#),
    );
    assert_eq!(status, 200, "{body}");
    std::thread::sleep(Duration::from_millis(500));
    let first = event_lines(&events);
    assert!(
        first.iter().any(|l| l.contains(r#""r":"e""#)),
        "the first segment recorded no message: {first:?}"
    );
    std::thread::sleep(Duration::from_secs(2));
    assert_eq!(
        event_lines(&events).len(),
        first.len(),
        "events.jsonl grew while recording was off"
    );

    // On again: appended, and readable on its own.
    http(
        port,
        "POST",
        "/api/interception/events",
        Some(r#"{"enabled":true}"#),
    );
    std::thread::sleep(Duration::from_secs(3));
    drop(proc);
    std::thread::sleep(Duration::from_millis(500));
    let all = event_lines(&events);
    assert!(
        all.len() > first.len(),
        "turning recording back on wrote nothing"
    );
    assert_eq!(
        &all[..first.len()],
        &first[..],
        "the earlier segment was rewritten"
    );
    let second = &all[first.len()..];
    assert!(
        second.iter().any(|l| l.contains(r#""r":"t""#)),
        "the second segment must name its topics again: {second:?}"
    );
    assert!(
        second.iter().any(|l| l.contains(r#""r":"e""#)),
        "the second segment recorded no message: {second:?}"
    );
}

/// The Health page's issue book, through its API: a deliberately mismatched
/// QoS pair (`qos_mismatch.launch.xml`, BEST_EFFORT publisher, RELIABLE
/// subscription) opens exactly one `qos_incompatible` issue for the topic —
/// not one per endpoint — and the open is journalled to `health.jsonl`.
#[test]
fn test_health_reports_one_issue_per_incompatible_topic() {
    let env = fixtures::install_env();
    let launch =
        fixtures::test_workspace_path("simple_test").join("launch/qos_mismatch.launch.xml");
    let work_dir = tempfile::TempDir::new().expect("failed to create work dir");
    let port = std::net::TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port();

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--interception",
        "on",
        "--web-addr",
        &format!("127.0.0.1:{port}"),
        "--disable-monitoring",
        "--disable-diagnostics",
        "--container-mode",
        "stock",
        launch.to_str().unwrap(),
    ]);
    cmd.env("PLAY_LAUNCH_INTERCEPTION_SO", interception_so_path());
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());
    let proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");
    let play_log = work_dir.path().join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 2, Duration::from_secs(15));

    // The pair is judged once both endpoints and their QoS are known.
    let deadline = std::time::Instant::now() + Duration::from_secs(20);
    let issues = loop {
        let (status, body) = http(port, "GET", "/api/health/issues", None);
        assert_eq!(status, 200, "{body}");
        let snap: serde_json::Value = serde_json::from_str(&body).unwrap();
        let qos: Vec<serde_json::Value> = snap["open"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|i| i["kind"] == "qos_incompatible")
            .cloned()
            .collect();
        if !qos.is_empty() || std::time::Instant::now() > deadline {
            break qos;
        }
        std::thread::sleep(Duration::from_millis(500));
    };
    drop(proc);
    std::thread::sleep(Duration::from_millis(500));

    assert_eq!(issues.len(), 1, "one issue per topic: {issues:?}");
    let issue = &issues[0];
    assert_eq!(issue["severity"], "error");
    assert_eq!(issue["subject"]["name"], "/qos_test");
    assert!(
        issue["detail"].as_str().unwrap().contains("reliability"),
        "{issue}"
    );

    let journal = std::fs::read_to_string(play_log.join("health.jsonl")).expect("health.jsonl");
    assert!(
        journal
            .lines()
            .any(|l| l.contains(r#""event":"open""#) && l.contains("qos_incompatible:/qos_test")),
        "{journal}"
    );
}

/// Exits reach the Health page with their signal (W3): a process that
/// segfaults and is respawned becomes a crash loop, one that aborts with
/// respawn off stays down, and `/api/nodes` carries each member's last exit.
#[test]
fn test_health_reports_crash_loops_and_members_that_stay_down() {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("simple_test").join("launch/crash_loop.launch.xml");
    let work_dir = tempfile::TempDir::new().expect("failed to create work dir");
    let port = std::net::TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port();

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--web-addr",
        &format!("127.0.0.1:{port}"),
        "--disable-monitoring",
        "--disable-diagnostics",
        launch.to_str().unwrap(),
    ]);
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());
    let proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let deadline = std::time::Instant::now() + Duration::from_secs(30);
    let (kinds, issues) = loop {
        std::thread::sleep(Duration::from_millis(500));
        let (status, body) = http(port, "GET", "/api/health/issues", None);
        if status != 200 {
            continue;
        }
        let snap: serde_json::Value = serde_json::from_str(&body).unwrap();
        let open = snap["open"].as_array().cloned().unwrap_or_default();
        let kinds: Vec<String> = open
            .iter()
            .map(|i| i["kind"].as_str().unwrap().to_string())
            .collect();
        if (kinds.contains(&"crash_loop".to_string()) && kinds.contains(&"down".to_string()))
            || std::time::Instant::now() > deadline
        {
            break (kinds, open);
        }
    };
    let (_, nodes) = http(port, "GET", "/api/nodes", None);
    drop(proc);

    assert!(kinds.contains(&"crash_loop".to_string()), "{issues:?}");
    assert!(kinds.contains(&"down".to_string()), "{issues:?}");
    let detail = |kind: &str| {
        issues
            .iter()
            .find(|i| i["kind"] == kind)
            .map(|i| i["detail"].as_str().unwrap().to_string())
            .unwrap()
    };
    assert!(
        detail("crash_loop").contains("SIGSEGV"),
        "{}",
        detail("crash_loop")
    );
    assert!(detail("down").contains("SIGABRT"), "{}", detail("down"));
    assert!(
        !issues.iter().any(|i| i["subject"]["name"] == "talker"),
        "the healthy talker raised an issue: {issues:?}"
    );

    let nodes: serde_json::Value = serde_json::from_str(&nodes).unwrap();
    let signals: Vec<i64> = nodes
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|n| n["last_exit"]["signal"].as_i64())
        .collect();
    assert!(signals.contains(&11), "SIGSEGV on /api/nodes: {signals:?}");
    assert!(signals.contains(&6), "SIGABRT on /api/nodes: {signals:?}");
}

/// Log watching (W4): a process writing ERROR lines with a changing counter
/// shows on the Health page as an error burst, and under Logs as ONE
/// signature with a count — not one row per counter value.
#[test]
fn test_health_reports_an_error_burst_with_its_signature() {
    let env = fixtures::install_env();
    let launch =
        fixtures::test_workspace_path("simple_test").join("launch/error_logger.launch.xml");
    let work_dir = tempfile::TempDir::new().expect("failed to create work dir");
    let port = std::net::TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port();

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--web-addr",
        &format!("127.0.0.1:{port}"),
        "--disable-monitoring",
        "--disable-diagnostics",
        launch.to_str().unwrap(),
    ]);
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());
    let proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let deadline = std::time::Instant::now() + Duration::from_secs(25);
    let snap = loop {
        std::thread::sleep(Duration::from_millis(500));
        let (status, body) = http(port, "GET", "/api/health/issues", None);
        if status != 200 {
            continue;
        }
        let snap: serde_json::Value = serde_json::from_str(&body).unwrap();
        let burst = snap["open"]
            .as_array()
            .unwrap()
            .iter()
            .any(|i| i["kind"] == "error_burst");
        if burst || std::time::Instant::now() > deadline {
            break snap;
        }
    };
    drop(proc);

    let burst = snap["open"]
        .as_array()
        .unwrap()
        .iter()
        .find(|i| i["kind"] == "error_burst")
        .unwrap_or_else(|| panic!("no error_burst: {snap}"));
    assert!(
        burst["detail"]
            .as_str()
            .unwrap()
            .contains("lost connection to sensor"),
        "{burst}"
    );

    let logs = snap["logs"].as_array().unwrap();
    let member = logs
        .iter()
        .find(|l| l["name"] == "error_logger")
        .unwrap_or_else(|| panic!("error_logger not under Logs: {logs:?}"));
    let errors: Vec<&serde_json::Value> = member["signatures"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|s| s["level"] == "error")
        .collect();
    assert_eq!(
        errors.len(),
        1,
        "one signature for every counter value: {errors:?}"
    );
    assert!(errors[0]["count"].as_u64().unwrap() > 10, "{errors:?}");
}

/// Delivery latency (W5): on one 20 Hz topic, a subscription that sleeps
/// 150 ms per message takes messages hundreds of milliseconds after their
/// publish and is reported, on Entities and as a Health issue; one that keeps
/// up shows a low latency and is not reported.
#[test]
fn test_delivery_latency_separates_a_slow_subscriber_from_a_fast_one() {
    let env = fixtures::install_env();
    let launch =
        fixtures::test_workspace_path("simple_test").join("launch/latency_probe.launch.xml");
    let work_dir = tempfile::TempDir::new().expect("failed to create work dir");
    let port = std::net::TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port();

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--interception",
        "on",
        "--web-addr",
        &format!("127.0.0.1:{port}"),
        "--disable-monitoring",
        "--disable-diagnostics",
        launch.to_str().unwrap(),
    ]);
    cmd.env("PLAY_LAUNCH_INTERCEPTION_SO", interception_so_path());
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());
    let proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let sub_row = |rows: &[serde_json::Value], node: &str| -> Option<serde_json::Value> {
        rows.iter()
            .find(|r| {
                r["node"] == node && r["direction"] == "sub" && r["topic"] == "/latency_probe"
            })
            .cloned()
    };
    let deadline = std::time::Instant::now() + Duration::from_secs(40);
    let (rows, issues) = loop {
        std::thread::sleep(Duration::from_millis(1000));
        let (s1, entities) = http(port, "GET", "/api/entities", None);
        let (s2, health) = http(port, "GET", "/api/health/issues", None);
        if s1 != 200 || s2 != 200 {
            continue;
        }
        let rows: Vec<serde_json::Value> = serde_json::from_str::<serde_json::Value>(&entities)
            .unwrap()["rows"]
            .as_array()
            .cloned()
            .unwrap_or_default();
        let issues: Vec<serde_json::Value> = serde_json::from_str::<serde_json::Value>(&health)
            .unwrap()["open"]
            .as_array()
            .cloned()
            .unwrap_or_default();
        let reported = issues.iter().any(|i| i["kind"] == "delivery_latency");
        if reported || std::time::Instant::now() > deadline {
            break (rows, issues);
        }
    };
    drop(proc);

    let slow = sub_row(&rows, "/latency_slow").unwrap_or_else(|| panic!("no slow row: {rows:?}"));
    let fast = sub_row(&rows, "/latency_fast").unwrap_or_else(|| panic!("no fast row: {rows:?}"));
    let p99 = |r: &serde_json::Value| r["latency"]["p99_ms"].as_f64();
    assert!(p99(&slow).is_some_and(|p| p >= 100.0), "slow: {slow}");
    assert!(p99(&fast).is_some_and(|p| p < 100.0), "fast: {fast}");
    let codes = |r: &serde_json::Value| -> Vec<String> {
        r["flags"]
            .as_array()
            .unwrap()
            .iter()
            .map(|f| f["code"].as_str().unwrap().to_string())
            .collect()
    };
    assert!(
        codes(&slow).contains(&"delivery_latency".to_string()),
        "{slow}"
    );
    assert!(
        !codes(&fast).contains(&"delivery_latency".to_string()),
        "{fast}"
    );

    let latency: Vec<&serde_json::Value> = issues
        .iter()
        .filter(|i| i["kind"] == "delivery_latency")
        .collect();
    assert_eq!(latency.len(), 1, "only the slow one: {latency:?}");
    assert!(
        latency[0]["subject"]["name"]
            .as_str()
            .unwrap()
            .contains("/latency_probe"),
        "{latency:?}"
    );
}

/// Run `graph_probe.launch.xml` with a node started outside it on the same
/// domain, and return the open Health issues once the near miss is reported
/// (or after 50 s).
fn graph_probe_issues(interception: bool) -> Vec<serde_json::Value> {
    let env = fixtures::install_env();
    let launch = fixtures::test_workspace_path("simple_test").join("launch/graph_probe.launch.xml");
    let script = fixtures::test_workspace_path("simple_test").join("scripts/graph_probe.py");
    let work_dir = tempfile::TempDir::new().expect("failed to create work dir");
    let port = std::net::TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port();

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--interception",
        if interception { "on" } else { "off" },
        "--web-addr",
        &format!("127.0.0.1:{port}"),
        "--disable-monitoring",
        "--disable-diagnostics",
        launch.to_str().unwrap(),
    ]);
    cmd.env("PLAY_LAUNCH_INTERCEPTION_SO", interception_so_path());
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());
    let domain = cmd
        .get_envs()
        .find(|(k, _)| *k == "ROS_DOMAIN_ID")
        .and_then(|(_, v)| v)
        .unwrap()
        .to_owned();
    let proc = ManagedProcess::spawn(&mut cmd).expect("failed to spawn play_launch");

    let mut stranger = std::process::Command::new("python3");
    fixtures::apply_test_env(&mut stranger, &env);
    stranger.env("ROS_DOMAIN_ID", &domain);
    stranger.args([script.to_str().unwrap(), "stranger"]);
    stranger.stdout(Stdio::null());
    stranger.stderr(Stdio::null());
    let stranger = ManagedProcess::spawn(&mut stranger).expect("failed to spawn stranger");

    let deadline = std::time::Instant::now() + Duration::from_secs(50);
    let issues = loop {
        std::thread::sleep(Duration::from_millis(1000));
        let (status, body) = http(port, "GET", "/api/health/issues", None);
        if status != 200 {
            continue;
        }
        let open: Vec<serde_json::Value> = serde_json::from_str::<serde_json::Value>(&body)
            .unwrap()["open"]
            .as_array()
            .cloned()
            .unwrap_or_default();
        let has = |k: &str| open.iter().any(|i| i["kind"] == k);
        if (has("near_miss_topic") && has("foreign_node")) || std::time::Instant::now() > deadline {
            break open;
        }
    };
    drop(stranger);
    drop(proc);
    issues
}

fn issues_of<'a>(issues: &'a [serde_json::Value], kind: &str) -> Vec<&'a serde_json::Value> {
    issues.iter().filter(|i| i["kind"] == kind).collect()
}

/// The ROS graph against the run (W6), with interception naming every node
/// of the run: the stranger is reported by name, and the subscription to a
/// typo of a published topic is a near miss naming the published one.
#[test]
fn test_health_compares_the_graph_with_the_run() {
    let issues = graph_probe_issues(true);
    let foreign = issues_of(&issues, "foreign_node");
    assert_eq!(foreign.len(), 1, "{issues:?}");
    assert_eq!(foreign[0]["subject"]["name"], "/stranger", "{foreign:?}");

    let near = issues_of(&issues, "near_miss_topic");
    assert_eq!(near.len(), 1, "{issues:?}");
    assert_eq!(near[0]["subject"]["name"], "/near_miss/pointz");
    let detail = near[0]["detail"].as_str().unwrap();
    assert!(detail.contains("`/near_miss/points`"), "{detail}");
    assert!(detail.contains("/nm_sub"), "{detail}");
    assert!(issues_of(&issues, "missing_node").is_empty(), "{issues:?}");
}

/// The same without interception: an executable's node has no name the run
/// can know, so the stranger is reported as the namespace's excess, and says
/// how to name them.
#[test]
fn test_health_counts_unnamed_nodes_by_namespace_without_interception() {
    let issues = graph_probe_issues(false);
    let foreign = issues_of(&issues, "foreign_node");
    assert_eq!(foreign.len(), 1, "{issues:?}");
    assert_eq!(
        foreign[0]["subject"]["name"], "/ (1 unaccounted)",
        "{foreign:?}"
    );
    assert!(
        foreign[0]["detail"]
            .as_str()
            .unwrap()
            .contains("--interception on"),
        "{foreign:?}"
    );
    assert_eq!(issues_of(&issues, "near_miss_topic").len(), 1, "{issues:?}");
}
