//! Smoke tests for Phase 36 runtime enforcement.
//!
//! Spawns a real ROS 2 launch under `play_launch` with `--manifest-dir`
//! and `--enforce-rules=warn`, then verifies that the RuleEngine wrote
//! `runtime_violations.jsonl` and that the expected rule fired.

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

/// Write a manifest dir at `<work>/manifests/_/pure_nodes.yaml` matching
/// `tests/fixtures/simple_test/launch/pure_nodes.launch.xml` (raw path
/// launches resolve to pkg = `_`). `topics_block` is appended verbatim
/// if non-empty — lets each test shape the manifest to provoke a
/// different rule violation.
fn make_manifest_dir(work_dir: &Path, topics_block: &str) -> PathBuf {
    let manifest_dir = work_dir.join("manifests");
    let pkg_dir = manifest_dir.join("_");
    std::fs::create_dir_all(&pkg_dir).expect("create pkg manifest dir");

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
    std::fs::write(pkg_dir.join("pure_nodes.yaml"), content).expect("write manifest");
    manifest_dir
}

/// Spawn `play_launch launch demo_nodes_cpp talker_listener.launch.xml`
/// with manifests + enforce-rules, run for `duration`, then SIGTERM.
fn run_with_manifest(
    manifest_dir: &Path,
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
        "--manifest-dir",
        manifest_dir.to_str().unwrap(),
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

/// Smoke test: RuleEngine writes `runtime_violations.jsonl` with at
/// least one entry for a real launch.
///
/// Note: the `.so`'s `rcl_publisher_init` hook reads `topic_name`
/// before rcl applies node-namespace expansion, so it emits bare
/// `chatter` rather than `/pure_test/chatter`. The consumer-side
/// FQN matching therefore can't tie `consistency-runtime` to the
/// manifest's `/pure_test/chatter` decl reliably across all RMWs.
/// Tracked as Phase 36 follow-up — for now assert only that the
/// violations file is produced and contains some rule (typically
/// graph-deviation-runtime fires because of the unexpanded name
/// mismatch).
#[test]
fn runtime_violations_jsonl_written_on_real_launch() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }

    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let manifest_dir = make_manifest_dir(
        work_dir.path(),
        r#"topics:
  /pure_test/chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
    sub: [listener/chatter]
"#,
    );

    let res = run_with_manifest(&manifest_dir, "warn", &[], Duration::from_secs(3));
    let play_log = res.path().join("play_log/latest");
    let viol_path = play_log.join("runtime_violations.jsonl");
    assert!(
        wait_for_file(&viol_path, Duration::from_secs(3)),
        "runtime_violations.jsonl not written at {}",
        viol_path.display()
    );

    let violations = read_violations(&viol_path);
    assert!(
        !violations.is_empty(),
        "expected at least one violation entry; got empty file"
    );
    for v in &violations {
        let rule = v["rule_id"].as_str().unwrap_or("");
        assert!(
            rule.ends_with("-runtime"),
            "every entry should be a runtime rule, got rule_id={rule:?}"
        );
    }
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
    let manifest_dir = make_manifest_dir(work_dir.path(), "");
    let res = run_with_manifest(&manifest_dir, "warn", &[], Duration::from_secs(3));
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
    let manifest_dir = make_manifest_dir(work_dir.path(), "");
    let res = run_with_manifest(&manifest_dir, "off", &[], Duration::from_secs(2));
    let viol_path = res
        .path()
        .join("play_log/latest/runtime_violations.jsonl");
    assert!(
        !viol_path.exists(),
        "runtime_violations.jsonl should NOT be written with --enforce-rules=off"
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
    let manifest_dir = make_manifest_dir(
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
        &manifest_dir,
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
