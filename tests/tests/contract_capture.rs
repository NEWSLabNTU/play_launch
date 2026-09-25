//! Integration tests for `play_launch contract capture` (issue #0044).
//!
//! The verb reads a recorded run plus a SystemModel and writes to stdout, so
//! most of it is testable against a HAND-WRITTEN bundle: no ROS, no spawning,
//! milliseconds. What that buys over the unit tests in
//! `ros_launch_resolve::verbs::capture` is the wiring — CLI parsing, model
//! loading, the file layout under `play_log/<ts>/interception/` — and, in the
//! last test, the gate that matters:
//!
//!   **a capture must pass `play_launch check` as it stands.**
//!
//! A capture that produced a contract its own run violates would be lying
//! about one of the two. That test needs a real launch file (the checker
//! reconciles every contract node key against the launch dump), so it uses
//! `demo_nodes_cpp` and skips cleanly when the package is absent.

use play_launch_tests::fixtures;
use std::path::{Path, PathBuf};

/// A bundle whose SILENT endpoints are the point (issue #0047): `/cap/quiet`
/// creates a publisher and a subscription and sends nothing through either, so
/// neither topic can appear in `stats_summary.json` — which is keyed by
/// traffic. Their types are in `endpoints.tsv` and nowhere else.
const ENDPOINTS: &str = "\
/cap/talker\t101\t/cap/talker\tpub\t/cap/chatter\tstd_msgs/msg/String
/cap/listener\t102\t/cap/listener\tsub\t/cap/chatter\tstd_msgs/msg/String
/cap/talker\t101\t/cap/talker\tpub\t/rosout\trcl_interfaces/msg/Log
/cap/quiet\t103\t/cap/quiet\tpub\t/cap/silent_topic\tstd_msgs/msg/String
/cap/quiet\t103\t/cap/quiet\tsub\t/cap/silent_input\tstd_msgs/msg/Int32
";

/// Only the exercised topic is here. That asymmetry is the whole fixture.
const STATS: &str = r#"{
  "1234": {
    "avg_pub_rate_hz": 9.97,
    "duration_ms": 10000.0,
    "msg_type": "std_msgs/msg/String",
    "name": "/cap/chatter",
    "pub_count": 100,
    "take_count": 100
  },
  "_events_dropped_total_best_effort": 0
}"#;

const MODEL: &str = r#"
meta:
  version: 1
structure:
  scopes:
    "0":
      path: /
  nodes:
    /cap/talker:
      scope: "0"
      pkg: demo_nodes_cpp
      exec: talker
      node_name: talker
    /cap/listener:
      scope: "0"
      pkg: demo_nodes_cpp
      exec: listener
      node_name: listener
    /cap/quiet:
      scope: "0"
      pkg: demo_nodes_cpp
      exec: talker
      node_name: quiet
"#;

/// The same three nodes the model carries — the checker refuses a contract
/// node the launch file does not produce (issue #0048), so the two must agree.
const LAUNCH: &str = r#"<?xml version="1.0"?>
<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="talker" namespace="/cap"/>
  <node pkg="demo_nodes_cpp" exec="listener" name="listener" namespace="/cap"/>
  <node pkg="demo_nodes_cpp" exec="talker" name="quiet" namespace="/cap"/>
</launch>
"#;

fn write_bundle(dir: &Path) -> (PathBuf, PathBuf) {
    let run = dir.join("play_log/2026-09-25_00-00-00");
    let interception = run.join("interception");
    std::fs::create_dir_all(&interception).unwrap();
    std::fs::write(interception.join("endpoints.tsv"), ENDPOINTS).unwrap();
    std::fs::write(interception.join("stats_summary.json"), STATS).unwrap();
    std::fs::write(
        interception.join("node_identity.tsv"),
        "/cap/talker\t101\t/cap/talker\n",
    )
    .unwrap();
    let model = dir.join("system_model.yaml");
    std::fs::write(&model, MODEL).unwrap();
    (run, model)
}

fn capture(run: &Path, model: &Path, extra: &[&str]) -> std::process::Output {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args([
        "contract",
        "capture",
        run.to_str().unwrap(),
        "--model",
        model.to_str().unwrap(),
    ]);
    cmd.args(extra);
    cmd.output()
        .expect("failed to run play_launch contract capture")
}

fn capture_text(run: &Path, model: &Path) -> String {
    let out = capture(run, model, &[]);
    assert!(
        out.status.success(),
        "capture failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8(out.stdout).unwrap()
}

#[test]
fn capture_emits_the_wiring_a_run_created() {
    let dir = tempfile::TempDir::new().unwrap();
    let (run, model) = write_bundle(dir.path());
    let text = capture_text(&run, &model);

    // Structure, in absolute node keys.
    assert!(text.contains("\n  /cap/talker:\n"), "{text}");
    assert!(text.contains("\n  /cap/listener:\n"), "{text}");
    assert!(text.contains("pub: [/cap/talker/chatter]"), "{text}");
    assert!(text.contains("sub: [/cap/listener/chatter]"), "{text}");
    // Infra is excluded by default and kept on request.
    assert!(!text.contains("/rosout"), "{text}");
    let with_infra = capture(&run, &model, &["--include-infra"]);
    assert!(
        String::from_utf8_lossy(&with_infra.stdout).contains("/rosout"),
        "--include-infra must keep it"
    );
}

/// The capability issue #0047 unlocked: a topic that carried no message is
/// described like any other, from the type recorded at the init hook.
#[test]
fn a_silent_endpoint_is_captured_with_its_type() {
    let dir = tempfile::TempDir::new().unwrap();
    let (run, model) = write_bundle(dir.path());
    let text = capture_text(&run, &model);

    assert!(text.contains("  /cap/silent_topic:\n"), "{text}");
    assert!(text.contains("  /cap/silent_input:\n"), "{text}");
    assert!(text.contains("type: std_msgs/msg/Int32"), "{text}");
    // ...and it is NOT refused for want of a type.
    assert!(
        !text.contains("message type is nowhere on disk"),
        "the endpoint record carries it: {text}"
    );
    // The traffic-keyed summary knows only the exercised topic, so only that
    // topic may carry an observation.
    let observed: Vec<&str> = text.lines().filter(|l| l.contains("Hz mean")).collect();
    assert_eq!(observed.len(), 1, "{observed:?}");
    assert!(observed[0].contains("9.97 Hz"), "{observed:?}");
}

/// The discipline, as an absence. A run can say 9.97 Hz; it cannot say 10 Hz
/// was required, and a contract states requirements.
#[test]
fn capture_emits_no_requirement() {
    let dir = tempfile::TempDir::new().unwrap();
    let (run, model) = write_bundle(dir.path());
    let text = capture_text(&run, &model);

    for key in [
        "rate_hz:",
        "min_rate_hz:",
        "max_latency",
        "max_age",
        "max_jitter",
        "criticality",
        "paths:",
    ] {
        let offending: Vec<&str> = text
            .lines()
            .filter(|l| l.contains(key) && !l.trim_start().starts_with('#'))
            .collect();
        assert!(
            offending.is_empty(),
            "{key} emitted as a field: {offending:?}"
        );
    }
    assert!(text.contains("states no requirement"), "{text}");
}

#[test]
fn a_run_without_interception_names_the_config_that_produces_it() {
    let dir = tempfile::TempDir::new().unwrap();
    let (_, model) = write_bundle(dir.path());
    let empty = dir.path().join("play_log/empty");
    std::fs::create_dir_all(&empty).unwrap();
    let out = capture(&empty, &model, &[]);
    assert!(!out.status.success());
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(err.contains("interception"), "{err}");
    assert!(err.contains("endpoints.tsv"), "{err}");
}

/// THE GATE. A capture must pass `check` as it stands, with no errors.
///
/// Needs a real package because the checker reconciles every contract node key
/// against the launch dump; skips cleanly without one, the same convention the
/// rest of the suite uses.
#[test]
fn a_capture_passes_check_as_it_stands() {
    let env = fixtures::install_env();
    if !env.contains_key("AMENT_PREFIX_PATH") {
        eprintln!("SKIP: no ROS environment; `check` cannot parse the launch file");
        return;
    }
    let dir = tempfile::TempDir::new().unwrap();
    let (run, model) = write_bundle(dir.path());
    let text = capture_text(&run, &model);

    // Beside the launch file is the PROVIDER channel: no `--contracts` root
    // needed, and it is where an author would put the file anyway.
    let launch_dir = dir.path().join("launch");
    std::fs::create_dir_all(&launch_dir).unwrap();
    let launch = launch_dir.join("cap.launch.xml");
    std::fs::write(&launch, LAUNCH).unwrap();
    std::fs::write(launch_dir.join("cap.contract.yaml"), &text).unwrap();

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args(["check", launch.to_str().unwrap()]);
    let out = cmd.output().expect("failed to run play_launch check");
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    if combined.contains("not found in AMENT_PREFIX_PATH") {
        eprintln!("SKIP: demo_nodes_cpp not installed");
        return;
    }
    assert!(
        out.status.success(),
        "check must accept a capture unedited:\n{combined}\n--- contract ---\n{text}"
    );
    assert!(
        combined.contains("0 with errors") || combined.contains("(0 errors"),
        "check reported errors:\n{combined}\n--- contract ---\n{text}"
    );
    // The manifest was actually LOADED — an unreadable file would leave
    // "No manifests found" and the assertions above would pass vacuously
    // (phase 69: "found, but none could be read" is a different verdict).
    assert!(combined.contains("manifest(s) checked"), "{combined}");
}
