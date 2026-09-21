//! Smoke tests for Phase 36 runtime enforcement.
//!
//! Spawns a real ROS 2 launch under `play_launch` with a `--contracts`
//! overlay tree and `--enforce-rules=warn`, then verifies that the
//! RuleEngine wrote `runtime_violations.jsonl` and that the expected rule
//! fired. No test writes a `--config` file to switch interception on: the
//! enforcement mode implies it (issue #0031), which is itself under test.

use play_launch_tests::{fixtures, process::ManagedProcess};
use std::{
    path::{Path, PathBuf},
    process::{Command, Stdio},
    time::Duration,
};

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
    fixtures::repo_root().join("install/play_launch/lib/play_launch/play_launch")
}

fn play_launch_cmd_with_cargo(env: &std::collections::HashMap<String, String>) -> Command {
    let mut cmd = Command::new(play_launch_bin_with_cargo_fallback());
    cmd.env_clear();
    cmd.envs(env);
    cmd.env("ROS_DOMAIN_ID", (std::process::id() % 200 + 30).to_string());
    let fastdds_profile = fixtures::repo_root().join("tests/fixtures/fastdds_no_shm.xml");
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

/// A `play_launch launch` of the simple_test fixture with an overlay
/// contract tree and `--enforce-rules <enforce>`, still running. The
/// supervisor's stdout and stderr go to `stdout.log` / `stderr.log`
/// under the work dir (a pipe nobody drains would stall the child).
struct StrictRun {
    work_dir: tempfile::TempDir,
    proc: ManagedProcess,
}

impl StrictRun {
    fn play_log(&self) -> PathBuf {
        self.work_dir.path().join("play_log/latest")
    }
    /// stdout then stderr: the tracing lines go to stdout, a `main` error
    /// to stderr.
    fn output(&self) -> String {
        format!(
            "{}{}",
            std::fs::read_to_string(self.work_dir.path().join("stdout.log")).unwrap_or_default(),
            std::fs::read_to_string(self.work_dir.path().join("stderr.log")).unwrap_or_default()
        )
    }
}

/// Spawn `play_launch launch` on `pure_nodes.launch.xml` with an overlay
/// contract tree and `--enforce-rules <enforce>`, and hand the running
/// process back. No --config file: since issue #0031 the mode itself
/// switches interception on.
fn spawn_with_manifest(overlay_root: &Path, enforce: &str, extra_args: &[&str]) -> StrictRun {
    let mut args = vec!["--enforce-rules", enforce];
    args.extend_from_slice(extra_args);
    spawn_launch(overlay_root, None, &args)
}

/// Spawn `play_launch launch` on `pure_nodes.launch.xml` with an overlay
/// contract tree. `config_yaml`, if given, is written to a file and passed
/// as `--config`; `args` follow the fixed flags.
fn spawn_launch(overlay_root: &Path, config_yaml: Option<&str>, args: &[&str]) -> StrictRun {
    let env = fixtures::install_env();
    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let so_path = interception_so_path();

    let mut cmd = play_launch_cmd_with_cargo(&env);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--container-mode",
        "stock",
        "--contracts",
        overlay_root.to_str().unwrap(),
    ]);
    if let Some(yaml) = config_yaml {
        let config_path = work_dir.path().join("config.yaml");
        std::fs::write(
            &config_path,
            format!("{yaml}  frontier: true\n  stats: true\n  ring_capacity: 4096\n"),
        )
        .expect("write config");
        cmd.arg("--config").arg(&config_path);
    }
    for a in args {
        cmd.arg(a);
    }
    // Use the simple_test fixture launch — pure_nodes.launch.xml runs
    // two demo_nodes_cpp nodes under namespace /pure_test.
    let launch = fixtures::test_workspace_path("simple_test").join("launch/pure_nodes.launch.xml");
    cmd.arg(launch.to_str().unwrap());
    cmd.env("PLAY_LAUNCH_INTERCEPTION_SO", &so_path);
    cmd.env("RUST_LOG", "play_launch=warn");
    cmd.stdout(Stdio::from(
        std::fs::File::create(work_dir.path().join("stdout.log")).expect("stdout file"),
    ));
    cmd.stderr(Stdio::from(
        std::fs::File::create(work_dir.path().join("stderr.log")).expect("stderr file"),
    ));

    let proc = ManagedProcess::spawn(&mut cmd).expect("spawn play_launch");
    StrictRun { work_dir, proc }
}

/// Spawn as `spawn_with_manifest`, wait for both nodes, run for
/// `duration`, then SIGTERM.
fn run_with_manifest(
    overlay_root: &Path,
    enforce: &str,
    extra_args: &[&str],
    duration: Duration,
) -> tempfile::TempDir {
    let run = spawn_with_manifest(overlay_root, enforce, extra_args);
    fixtures::wait_for_processes(&run.play_log(), 2, Duration::from_secs(15));
    std::thread::sleep(duration);
    let StrictRun { work_dir, proc } = run;
    drop(proc);
    // Give the listener task time to flush jsonl + interception summaries.
    std::thread::sleep(Duration::from_millis(800));
    work_dir
}

/// The pids the run's node actors wrote under `play_log/latest/node/*/pid`.
fn node_pids(play_log: &Path) -> Vec<u32> {
    let Ok(entries) = std::fs::read_dir(play_log.join("node")) else {
        return Vec::new();
    };
    entries
        .flatten()
        .filter_map(|e| std::fs::read_to_string(e.path().join("pid")).ok())
        .filter_map(|s| s.trim().parse::<u32>().ok())
        .collect()
}

/// Is `pid` a live (non-zombie) process on this host?
fn pid_is_alive(pid: u32) -> bool {
    match std::fs::read_to_string(format!("/proc/{pid}/stat")) {
        Ok(stat) => {
            // "<pid> (<comm>) <state> ..." -- comm may contain spaces, so
            // take the state from after the closing paren.
            stat.rsplit(')')
                .next()
                .and_then(|rest| rest.split_whitespace().next())
                .map(|state| state != "Z" && state != "X")
                .unwrap_or(false)
        }
        Err(_) => false,
    }
}

/// The overlay contract that any run of the fixture violates: the
/// demo_nodes_cpp talker publishes at 1 Hz and the contract demands 1000.
fn write_rate_1000_contract(work_dir: &Path) -> PathBuf {
    let overlay_root = work_dir.join("contracts");
    let launch_dir = overlay_root.join("_/launch");
    std::fs::create_dir_all(&launch_dir).expect("create overlay launch dir");
    std::fs::write(
        launch_dir.join("pure_nodes.contract.yaml"),
        r#"version: 1
nodes:
  talker:
    pub:
      chatter:
        min_rate_hz: 1000
  listener:
    sub:
      chatter: {}
topics:
  /pure_test/chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
    sub: [listener/chatter]
"#,
    )
    .expect("write contract");
    overlay_root
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
    let viol_path = res.path().join("play_log/latest/runtime_violations.jsonl");
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
    let viol_path = res.path().join("play_log/latest/runtime_violations.jsonl");
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
        violations
            .iter()
            .any(|v| v["rule_id"].as_str() == Some("graph-deviation-runtime")
                && v["fqn"]
                    .as_str()
                    .map(|s| s.starts_with("/"))
                    .unwrap_or(false)),
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
    let viol_path = res.path().join("play_log/latest/runtime_violations.jsonl");
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

    let mut cmd = play_launch_cmd_with_cargo(&env);
    cmd.current_dir(work_dir.path());
    cmd.args([
        "launch",
        "--disable-web-ui",
        "--disable-monitoring",
        "--disable-diagnostics",
        "--container-mode",
        "stock",
        "--contracts",
        overlay_root.to_str().unwrap(),
        "--enforce-rules",
        "warn",
    ]);
    let launch =
        fixtures::test_workspace_path("simple_test").join("launch/qos_mismatch.launch.xml");
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
    let launch = fixtures::test_workspace_path("simple_test").join("launch/pure_nodes.launch.xml");
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
        .map(|d| {
            d.flatten()
                .filter(|e| e.path().join("pid").exists())
                .count()
        })
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

/// The runtime respects a declared rate contract: `demo_nodes_cpp` talker
/// publishes at ~1 Hz; a contract demanding `min_rate_hz: 1000` must make
/// `rate-hierarchy-runtime` fire. (Phase 43: `launch` routes through the
/// SystemModel, so this exercises the model-path rule engine.)
#[test]
fn rate_hierarchy_runtime_fires_when_publisher_too_slow() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let overlay_root = work_dir.path().join("contracts");
    let launch_dir = overlay_root.join("_/launch");
    std::fs::create_dir_all(&launch_dir).expect("create overlay launch dir");
    std::fs::write(
        launch_dir.join("pure_nodes.contract.yaml"),
        r#"version: 1
nodes:
  talker:
    pub:
      chatter:
        min_rate_hz: 1000
  listener:
    sub:
      chatter: {}
topics:
  /pure_test/chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
    sub: [listener/chatter]
"#,
    )
    .expect("write contract");

    let run_dir = run_with_manifest(&overlay_root, "warn", &[], Duration::from_secs(8));
    let violations_path = run_dir
        .path()
        .join("play_log/latest/runtime_violations.jsonl");
    assert!(
        wait_for_file(&violations_path, Duration::from_secs(10)),
        "runtime_violations.jsonl not written"
    );
    let violations = read_violations(&violations_path);
    assert!(
        violations
            .iter()
            .any(|v| v["rule_id"] == "rate-hierarchy-runtime"),
        "expected rate-hierarchy-runtime (1 Hz talker vs min_rate_hz 1000), got: {violations:?}"
    );
}

/// The converse: a contract the running system SATISFIES must produce no
/// rate/age/consistency violations — the engine respects compliant
/// contracts silently.
#[test]
fn compliant_contract_produces_no_rate_or_type_violations() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let overlay_root = work_dir.path().join("contracts");
    let launch_dir = overlay_root.join("_/launch");
    std::fs::create_dir_all(&launch_dir).expect("create overlay launch dir");
    std::fs::write(
        launch_dir.join("pure_nodes.contract.yaml"),
        r#"version: 1
nodes:
  talker:
    pub:
      chatter:
        min_rate_hz: 0.1
  listener:
    sub:
      chatter: {}
topics:
  /pure_test/chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
    sub: [listener/chatter]
"#,
    )
    .expect("write contract");

    let run_dir = run_with_manifest(&overlay_root, "warn", &[], Duration::from_secs(8));
    let violations_path = run_dir
        .path()
        .join("play_log/latest/runtime_violations.jsonl");
    // File may legitimately not exist (no violations at all).
    let offending: Vec<serde_json::Value> = if violations_path.is_file() {
        read_violations(&violations_path)
            .into_iter()
            .filter(|v| {
                matches!(
                    v["rule_id"].as_str(),
                    Some("rate-hierarchy-runtime")
                        | Some("consistency-runtime")
                        | Some("max-age-runtime")
                )
            })
            .collect()
    } else {
        Vec::new()
    };
    assert!(
        offending.is_empty(),
        "compliant contract must not trip rate/type/age rules, got: {offending:?}"
    );
}

/// Issue #0033: `--enforce-rules strict` must END the run on a violation.
/// The supervisor exits non-zero within seconds of the violation and its
/// nodes go with it. Before the fix the strict watcher flipped the
/// run-level watch channel and nothing else, so every actor sat in
/// `child.wait()` on a healthy child, the nodes kept running with their
/// output no longer forwarded, and play_launch lived until an outside
/// SIGINT (a CI job timeout, in practice).
#[test]
fn strict_mode_ends_the_run_non_zero_and_stops_the_nodes() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let overlay_root = write_rate_1000_contract(work_dir.path());

    let mut run = spawn_with_manifest(&overlay_root, "strict", &[]);
    // The rate rule needs a 0.5 s window and re-checks a slow topic every
    // ~5 s, so the violation lands 5-6 s after the talker's first publish;
    // the budget covers node startup on a loaded host on top of that.
    let status = run.proc.wait_with_timeout(Duration::from_secs(40));
    assert!(
        !status.success(),
        "a strict violation must end the run non-zero, got {status:?}"
    );

    let stderr = run.output();
    assert!(
        stderr.contains("Strict enforcement violated"),
        "the run ended, but not because of the strict watcher; output:\n{stderr}"
    );
    assert!(
        stderr.contains("runtime contract violated"),
        "the exit must name the violation as its cause; output:\n{stderr}"
    );

    // Issue #0032: the run got past the `/rosout` and `/parameter_events`
    // graph-deviation WARNINGS every node raises at startup, and ended on
    // the rate ERROR the contract was written for.
    let violations = read_violations(&run.play_log().join("runtime_violations.jsonl"));
    assert!(
        violations
            .iter()
            .any(|v| v["rule_id"] == "rate-hierarchy-runtime" && v["severity"] == "error"),
        "the strict run must end on the rate error, not a warning; got: {violations:?}"
    );

    // The nodes were signalled, not abandoned: none of the pids the actors
    // recorded is still alive once the supervisor has exited.
    let pids = node_pids(&run.play_log());
    assert!(!pids.is_empty(), "the run recorded no node pids");
    std::thread::sleep(Duration::from_millis(500));
    let survivors: Vec<u32> = pids.into_iter().filter(|&p| pid_is_alive(p)).collect();
    assert!(
        survivors.is_empty(),
        "nodes still running after the strict shutdown: {survivors:?}"
    );
}

/// Issue #0031: the default invocation -- no --config, no --enforce-rules --
/// with a contract beside the launch file must enforce that contract. Before
/// the fix `--enforce-rules warn` (the default) had no event source unless a
/// --config file switched interception on, so this run wrote no
/// `runtime_violations.jsonl` and exited 0 with nothing on the terminal.
#[test]
fn default_invocation_intercepts_once_a_contract_resolves() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let overlay_root = write_rate_1000_contract(work_dir.path());

    let run = spawn_launch(&overlay_root, None, &[]);
    let play_log = run.play_log();
    fixtures::wait_for_processes(&play_log, 2, Duration::from_secs(15));
    // The rate rule re-checks a slow topic every ~5 s after its first
    // 0.5 s window.
    let viol_path = play_log.join("runtime_violations.jsonl");
    assert!(
        wait_for_file(&viol_path, Duration::from_secs(15)),
        "runtime_violations.jsonl not written: the default run has no event source; stderr:\n{}",
        run.output()
    );
    let deadline = std::time::Instant::now() + Duration::from_secs(10);
    let mut violations = read_violations(&viol_path);
    while std::time::Instant::now() < deadline
        && !violations
            .iter()
            .any(|v| v["rule_id"] == "rate-hierarchy-runtime")
    {
        std::thread::sleep(Duration::from_millis(500));
        violations = read_violations(&viol_path);
    }
    // The decision is logged at info, which the terminal filter of these
    // tests (`play_launch=warn`) hides; the bundle's own log keeps it.
    let bundle_log = std::fs::read_to_string(play_log.join("play_launch.log")).unwrap_or_default();
    drop(run);
    assert!(
        violations
            .iter()
            .any(|v| v["rule_id"] == "rate-hierarchy-runtime"),
        "expected rate-hierarchy-runtime from the default invocation, got: {violations:?}"
    );
    assert!(
        bundle_log.contains("implied by --enforce-rules"),
        "the run must say what switched interception on; play_launch.log:\n{bundle_log}"
    );
}

/// Issue #0031, the other half: a strict run whose --config switches
/// interception OFF cannot measure anything, so it must refuse to start
/// rather than pass green. A warn run says so and continues.
#[test]
fn strict_refuses_to_start_without_an_event_source() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let overlay_root = write_rate_1000_contract(work_dir.path());

    let mut run = spawn_launch(
        &overlay_root,
        Some("interception:\n  enabled: false\n"),
        &["--enforce-rules", "strict"],
    );
    let status = run.proc.wait_with_timeout(Duration::from_secs(30));
    let stderr = run.output();
    assert!(
        !status.success(),
        "strict with interception off must not run; status {status:?}, stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("no event source"),
        "the refusal must name the missing event source; stderr:\n{stderr}"
    );

    let run = spawn_launch(
        &overlay_root,
        Some("interception:\n  enabled: false\n"),
        &["--enforce-rules", "warn"],
    );
    fixtures::wait_for_processes(&run.play_log(), 2, Duration::from_secs(15));
    let stderr = run.output();
    assert!(
        pid_is_alive(run.proc.id()),
        "a warn run with interception off should still run; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("no event source"),
        "a warn run with interception off must warn that no rule can fire; stderr:\n{stderr}"
    );
}

/// Issue #0032: a strict run whose only violations are warnings keeps
/// running. With no `topics:` block every endpoint is a
/// `graph-deviation-runtime` WARNING (`/rosout` and `/parameter_events`
/// included), and before the fix the first of them ended the run 57 ms in.
#[test]
fn strict_mode_keeps_running_on_warning_severity_violations() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let work_dir = tempfile::TempDir::new().expect("tempdir");
    let overlay_root = make_overlay_contracts(work_dir.path(), "");

    let run = spawn_with_manifest(&overlay_root, "strict", &[]);
    let play_log = run.play_log();
    fixtures::wait_for_processes(&play_log, 2, Duration::from_secs(15));
    let viol_path = play_log.join("runtime_violations.jsonl");
    assert!(
        wait_for_file(&viol_path, Duration::from_secs(10)),
        "runtime_violations.jsonl not written"
    );
    std::thread::sleep(Duration::from_secs(3));
    let violations = read_violations(&viol_path);
    assert!(
        violations
            .iter()
            .any(|v| v["rule_id"] == "graph-deviation-runtime" && v["severity"] == "warning"),
        "expected graph-deviation warnings; got: {violations:?}"
    );
    // The rcl-internal topics every node creates are not deviations from
    // any launch tree.
    assert!(
        !violations
            .iter()
            .any(|v| v["fqn"] == "/rosout" || v["fqn"] == "/parameter_events"),
        "/rosout and /parameter_events must not be reported; got: {violations:?}"
    );
    let stderr = run.output();
    assert!(
        pid_is_alive(run.proc.id()),
        "warnings alone must not end a strict run; stderr:\n{stderr}"
    );
    assert!(
        !stderr.contains("Strict enforcement violated"),
        "the strict watcher fired on a warning; stderr:\n{stderr}"
    );
    let pids = node_pids(&play_log);
    assert!(
        !pids.is_empty() && pids.iter().all(|&p| pid_is_alive(p)),
        "the nodes must still be running: {pids:?}"
    );
}
