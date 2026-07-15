//! Smoke tests for Phase 36 runtime enforcement.
//!
//! Spawns a real ROS 2 launch under `play_launch` with a `--contracts`
//! overlay tree and `--enforce-rules=warn`, then verifies that the
//! RuleEngine wrote `runtime_violations.jsonl` and that the expected rule
//! fired.

use play_launch_tests::fixtures;
use play_launch_tests::process::ManagedProcess;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::Duration;

/// Prefer the cargo-built binary so Phase 36 CLI flags are present.
/// The install/ binary may be older than the workspace if `just build`
/// hasn't been re-run since Phase 36 landed.
fn play_launch_bin_with_cargo_fallback() -> PathBuf {
    let release = fixtures::repo_root().join("target/release/play_launch");
    if release.is_file() {
        return release;
    }
    let debug = fixtures::repo_root().join("target/debug/play_launch");
    if debug.is_file() {
        return debug;
    }
    fixtures::repo_root()
        .join("install/play_launch/lib/play_launch/play_launch")
}

fn play_launch_cmd_with_cargo(
    env: &std::collections::HashMap<String, String>,
) -> Command {
    let mut cmd = Command::new(play_launch_bin_with_cargo_fallback());
    cmd.env_clear();
    cmd.envs(env);
    cmd.env(
        "ROS_DOMAIN_ID",
        (std::process::id() % 200 + 30).to_string(),
    );
    let fastdds_profile =
        fixtures::repo_root().join("tests/fixtures/fastdds_no_shm.xml");
    if fastdds_profile.is_file() {
        cmd.env("FASTRTPS_DEFAULT_PROFILES_FILE", &fastdds_profile);
    }
    cmd
}

fn interception_so_path() -> PathBuf {
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

fn wait_for_file(path: &Path, timeout: Duration) -> bool {
    let start = std::time::Instant::now();
    while start.elapsed() < timeout {
        if path.is_file() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    false
}

/// Write an overlay contract tree at
/// `<work>/contracts/_/launch/pure_nodes.contract.yaml` matching
/// `tests/fixtures/simple_test/launch/pure_nodes.launch.xml` (raw path
/// launches resolve to pkg = `_`). `topics_block` is appended verbatim
/// if non-empty — lets each test shape the manifest to provoke a
/// different rule violation.
fn make_overlay_contracts(work_dir: &Path, topics_block: &str) -> PathBuf {
    let overlay_root = work_dir.join("contracts");
    let launch_dir = overlay_root.join("_/launch");
    std::fs::create_dir_all(&launch_dir).expect("create overlay launch dir");

    let nodes_block = r#"
nodes:
  talker:
    pub:
      chatter: {}
  listener:
    sub:
      chatter: {}
"#;
    let content = format!("version: 1\n{nodes_block}{topics_block}");
    std::fs::write(launch_dir.join("pure_nodes.contract.yaml"), content).expect("write contract");
    overlay_root
}

/// Spawn `play_launch launch demo_nodes_cpp talker_listener.launch.xml`
/// with an overlay contract tree + enforce-rules, run for `duration`,
/// then SIGTERM.
fn run_with_manifest(
    overlay_root: &Path,
    enforce: &str,
    extra_args: &[&str],
    duration: Duration,
) -> tempfile::TempDir {
    let env = fixtures::install_env();
    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let so_path = interception_so_path();

    // Interception isn't enabled by default; the runtime rules feed
    // off interception SPSC events so we need it on. Write a minimal
    // config YAML and pass it via `--config`.
    let config_path = work_dir.path().join("interception_on.yaml");
    std::fs::write(
        &config_path,
        "interception:\n  enabled: true\n  frontier: true\n  stats: true\n  ring_capacity: 4096\n",
    )
    .expect("write config");

    let mut cmd = play_launch_cmd_with_cargo(&env);
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
        "--contracts",
        overlay_root.to_str().unwrap(),
        "--enforce-rules",
        enforce,
    ]);
    for a in extra_args {
        cmd.arg(a);
    }
    // Use the simple_test fixture launch — pure_nodes.launch.xml runs
    // two demo_nodes_cpp nodes under namespace /pure_test.
    let launch = fixtures::test_workspace_path("simple_test")
        .join("launch/pure_nodes.launch.xml");
    cmd.arg(launch.to_str().unwrap());
    cmd.env("PLAY_LAUNCH_INTERCEPTION_SO", &so_path);
    cmd.env("RUST_LOG", "play_launch=warn");
    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());

    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    let play_log = work_dir.path().join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 2, Duration::from_secs(15));
    std::thread::sleep(duration);
    drop(proc);
    // Give the listener task time to flush jsonl + interception summaries.
    std::thread::sleep(Duration::from_millis(800));
    work_dir
}

/// Read every line of `runtime_violations.jsonl` as a `serde_json::Value`.
fn read_violations(path: &Path) -> Vec<serde_json::Value> {
    let text = std::fs::read_to_string(path).expect("read violations jsonl");
    text.lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| serde_json::from_str::<serde_json::Value>(l).expect("invalid jsonl line"))
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// Manifest declares `/pure_test/chatter` with the WRONG msg type
/// (Int32 vs the actual String). Phase 36 polish ensures the `.so`
/// expands bare `chatter` to `/pure_test/chatter` via `rcl_node_*`
/// accessors before hashing, so the consumer-side FQN matching ties
/// the runtime type-hash to the manifest decl and fires
/// `consistency-runtime`.
#[test]
fn consistency_runtime_fires_when_type_mismatch_real_launch() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }

    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let overlay_root = make_overlay_contracts(
        work_dir.path(),
        r#"topics:
  /pure_test/chatter:
    type: std_msgs/msg/Int32
    pub: [talker/chatter]
    sub: [listener/chatter]
"#,
    );

    let res = run_with_manifest(&overlay_root, "warn", &[], Duration::from_secs(3));
    let viol_path = res
        .path()
        .join("play_log/latest/runtime_violations.jsonl");
    assert!(
        wait_for_file(&viol_path, Duration::from_secs(3)),
        "runtime_violations.jsonl not written"
    );

    let violations = read_violations(&viol_path);
    assert!(
        violations.iter().any(|v| {
            v["rule_id"].as_str() == Some("consistency-runtime")
                && v["fqn"].as_str() == Some("/pure_test/chatter")
        }),
        "expected consistency-runtime on /pure_test/chatter; got: {violations:?}"
    );
}

/// Manifest declares no `topics:` block at all. Every PublisherInit /
/// SubscriptionInit fires `graph-deviation-runtime`.
#[test]
fn graph_deviation_runtime_fires_for_undeclared_topic() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }

    let work_dir = tempfile::TempDir::new().expect("tempdir");
    // Empty topics block → /chatter not in topic_hash_to_fqn map.
    let overlay_root = make_overlay_contracts(work_dir.path(), "");
    let res = run_with_manifest(&overlay_root, "warn", &[], Duration::from_secs(3));
    let viol_path = res
        .path()
        .join("play_log/latest/runtime_violations.jsonl");
    assert!(
        wait_for_file(&viol_path, Duration::from_secs(3)),
        "runtime_violations.jsonl not written"
    );
    let violations = read_violations(&viol_path);
    assert!(
        violations
            .iter()
            .any(|v| v["rule_id"].as_str() == Some("graph-deviation-runtime")),
        "expected graph-deviation-runtime; got: {violations:?}"
    );
    // Phase 36 polish: the .so ships TopicNameDeclared chunks so
    // graph-deviation messages now carry the real FQN string instead
    // of falling back to `(unknown hash ...)`.
    assert!(
        violations.iter().any(|v| v["rule_id"].as_str() == Some("graph-deviation-runtime")
            && v["fqn"].as_str().map(|s| s.starts_with("/")).unwrap_or(false)),
        "expected at least one graph-deviation FQN to be a real topic path; got: {violations:?}"
    );
}

/// `--enforce-rules=off` short-circuits the RuleEngine — no jsonl
/// should be produced even though the manifest would otherwise trip
/// rules.
#[test]
fn enforce_off_skips_rule_engine() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }

    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let overlay_root = make_overlay_contracts(work_dir.path(), "");
    let res = run_with_manifest(&overlay_root, "off", &[], Duration::from_secs(2));
    let viol_path = res
        .path()
        .join("play_log/latest/runtime_violations.jsonl");
    assert!(
        !viol_path.exists(),
        "runtime_violations.jsonl should NOT be written with --enforce-rules=off"
    );
}

/// Two rclpy nodes with mismatched reliability QoS trigger DDS
/// `OFFERED_QOS_INCOMPATIBLE` / `REQUESTED_QOS_INCOMPATIBLE` events.
/// The new RuleEngine path turns each into a `qos-match-runtime`
/// violation. The smoke test asserts at least one such violation
/// appears in `runtime_violations.jsonl`.
#[test]
fn qos_match_runtime_fires_on_dds_incompatibility() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }

    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let so_path = interception_so_path();

    // Allow both bare and absolute forms of the topic — the python
    // scripts use the relative name `qos_test`, which rcl expands to
    // `/qos_test` once node namespaces are applied.
    let overlay_root = work_dir.path().join("contracts");
    let launch_dir = overlay_root.join("_/launch");
    std::fs::create_dir_all(&launch_dir).expect("create overlay launch dir");
    std::fs::write(
        launch_dir.join("qos_mismatch.contract.yaml"),
        "version: 1\nnodes:\n  qos_mismatch_pub:\n    pub:\n      qos_test: {}\n  qos_mismatch_sub:\n    sub:\n      qos_test: {}\ntopics:\n  /qos_test:\n    type: std_msgs/msg/String\n    pub: [qos_mismatch_pub/qos_test]\n    sub: [qos_mismatch_sub/qos_test]\n",
    )
    .expect("write contract");

    let config_path = work_dir.path().join("interception_on.yaml");
    std::fs::write(
        &config_path,
        "interception:\n  enabled: true\n  frontier: true\n  stats: true\n  ring_capacity: 4096\n",
    )
    .expect("write config");

    let mut cmd = play_launch_cmd_with_cargo(&env);
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
        "--contracts",
        overlay_root.to_str().unwrap(),
        "--enforce-rules",
        "warn",
    ]);
    let launch = fixtures::test_workspace_path("simple_test")
        .join("launch/qos_mismatch.launch.xml");
    cmd.arg(launch.to_str().unwrap());
    cmd.env("PLAY_LAUNCH_INTERCEPTION_SO", &so_path);
    cmd.env("RUST_LOG", "play_launch=warn");
    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());

    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    let play_log = work_dir.path().join("play_log/latest");
    fixtures::wait_for_processes(&play_log, 2, Duration::from_secs(15));
    // DDS discovery + incompatible QoS event delivery typically takes
    // a couple of seconds even on loopback. Hold the run for 6s.
    std::thread::sleep(Duration::from_secs(6));
    drop(proc);
    std::thread::sleep(Duration::from_millis(800));

    let viol_path = work_dir
        .path()
        .join("play_log/latest/runtime_violations.jsonl");
    assert!(
        wait_for_file(&viol_path, Duration::from_secs(3)),
        "runtime_violations.jsonl not written at {}",
        viol_path.display()
    );
    let violations = read_violations(&viol_path);
    assert!(
        violations.iter().any(|v| {
            v["rule_id"].as_str() == Some("qos-match-runtime")
                && v["message"]
                    .as_str()
                    .map(|s| s.contains("DDS reported incompatible QoS"))
                    .unwrap_or(false)
        }),
        "expected DDS qos-match-runtime violation; got: {violations:?}"
    );
}

/// Blocking mode (`--block-unauthorized-endpoints`) writes
/// `expected_graph.txt`. The allowlist parser is exercised by
/// in-process unit tests; here we just confirm the file appears
/// alongside the run.
#[test]
fn block_unauthorized_endpoints_writes_allowlist_file() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }

    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let overlay_root = make_overlay_contracts(
        work_dir.path(),
        r#"topics:
  /pure_test/chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
    sub: [listener/chatter]
"#,
    );
    // Use warn + block so a violation file or block log appears.
    let res = run_with_manifest(
        &overlay_root,
        "warn",
        &["--block-unauthorized-endpoints"],
        Duration::from_secs(2),
    );
    let allow_path = res.path().join("play_log/latest/expected_graph.txt");
    assert!(
        allow_path.exists(),
        "expected_graph.txt not written at {}",
        allow_path.display()
    );
    let contents = std::fs::read_to_string(&allow_path).expect("read allowlist");
    assert!(
        contents.contains("/pure_test/chatter"),
        "allowlist missing /pure_test/chatter: {contents}"
    );
}

/// Regression (40.2): with NO contract source resolving anything, blocking
/// enforcement must DISABLE itself (with a warning) rather than write an
/// empty allowlist that would refuse every rcl endpoint in every child.
/// Before the fix, the always-built manifest index made the old
/// `is_some()` gate pass and an empty expected_graph.txt blocked all IPC.
#[test]
fn block_unauthorized_endpoints_disabled_when_no_contracts_resolve() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }

    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let mut cmd = play_launch_cmd_with_cargo(&env);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--container-mode",
        "stock",
        // no --contracts; fixture ships no provider sidecar either
        "--block-unauthorized-endpoints",
    ]);
    let launch = fixtures::test_workspace_path("simple_test")
        .join("launch/pure_nodes.launch.xml");
    cmd.arg(launch.to_str().unwrap());
    cmd.env("RUST_LOG", "play_launch=warn");
    let stdout_path = work_dir.path().join("stdout.log");
    let stderr_path = work_dir.path().join("stderr.log");
    cmd.stdout(Stdio::from(
        std::fs::File::create(&stdout_path).expect("stdout file"),
    ));
    cmd.stderr(Stdio::from(
        std::fs::File::create(&stderr_path).expect("stderr file"),
    ));

    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    let play_log = work_dir.path().join("play_log/latest");
    // Both nodes must come up — proof their rcl endpoints were NOT blocked.
    fixtures::wait_for_processes(&play_log, 2, Duration::from_secs(15));
    std::thread::sleep(Duration::from_secs(2));
    let started = std::fs::read_dir(play_log.join("node"))
        .map(|d| d.flatten().filter(|e| e.path().join("pid").exists()).count())
        .unwrap_or(0);
    drop(proc);
    std::thread::sleep(Duration::from_millis(500));

    assert!(
        started >= 2,
        "expected 2 running nodes (endpoints unblocked), found {started}"
    );
    let allow_path = play_log.join("expected_graph.txt");
    assert!(
        !allow_path.exists(),
        "empty allowlist must NOT be written when no contracts resolve"
    );
    let combined = format!(
        "{}{}",
        std::fs::read_to_string(&stdout_path).unwrap_or_default(),
        std::fs::read_to_string(&stderr_path).unwrap_or_default()
    )
    .to_lowercase();
    assert!(
        combined.contains("blocking") && combined.contains("disabled"),
        "expected the blocking-disabled warning in output, got: {combined}"
    );
}
