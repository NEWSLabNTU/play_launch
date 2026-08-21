//! Integration tests for `play_launch measure` (Phase 58 W2).
//!
//! The verb reads two files and writes to stdout, so it can be tested against
//! a hand-written run: no ROS, no spawning, milliseconds. What that buys is
//! coverage of the wiring the unit tests deliberately skip — CLI parsing,
//! model loading, and the endpoint→topic resolution that connects a contract's
//! endpoint names to the topic FQNs the wire actually carries.
//!
//! The measurement itself is validated against ground truth elsewhere:
//! `examples/rt_av_demo` burns a known number of milliseconds per callback,
//! and `just measure` there checks the reported cost against it.

use play_launch_tests::fixtures;
use std::path::Path;

/// A SystemModel with one node, one declared path, and the wiring that maps
/// its endpoints onto real topics.
///
/// `input`/`output` in `node_paths` are endpoint refs (`<node>/<endpoint>`),
/// not topics — remapping sits between the two, and `structure.topics` is
/// where the resolver recorded the outcome. Getting that indirection wrong is
/// exactly what this fixture is here to catch.
const MODEL: &str = r#"
meta:
  version: 1
structure:
  scopes:
    "0":
      path: /
  nodes:
    /detector:
      scope: "0"
      pkg: demo
      exec: detector
  topics:
    /scan:
      type: demo/msg/Scan
      pub: [/lidar/scan]
      sub: [/detector/scan]
    /obstacles:
      type: demo/msg/Obstacles
      pub: [/detector/obstacles]
contracts:
  node_paths:
    /detector/detect:
      input: [/detector/scan]
      output: [/detector/obstacles]
      max_latency_ms: 12.0
"#;

/// FNV-1a, the hash the interception layer keys topics by. Only the topic
/// name records in the file need to agree with these, so any stable value
/// would do — but using the real hash keeps the fixture honest.
fn hash(s: &str) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in s.as_bytes() {
        h ^= *b as u64;
        h = h.wrapping_mul(0x1000_0000_01b3);
    }
    h
}

/// One take + one publish of the same message, `cost_ns` of CPU apart and
/// `wall_ns` of wall clock apart.
fn invocation(stamp: u32, start: u64, cost_ns: u64, wall_ns: u64) -> String {
    let scan = hash("/scan");
    let obstacles = hash("/obstacles");
    format!(
        "{}\n{}\n",
        format_args!(
            r#"{{"r":"e","n":"/detector","d":"take","h":{scan},"s":{stamp},"ns":0,"t":{start},"c":{start},"tid":42}}"#
        ),
        format_args!(
            r#"{{"r":"e","n":"/detector","d":"pub","h":{obstacles},"s":{stamp},"ns":0,"t":{},"c":{},"tid":42}}"#,
            start + wall_ns,
            start + cost_ns
        ),
    )
}

fn write_run(dir: &Path, events: &str) -> std::path::PathBuf {
    let run = dir.join("play_log/2026-08-13-00-00-00");
    std::fs::create_dir_all(run.join("interception")).unwrap();
    let mut body = format!(
        "{{\"r\":\"t\",\"h\":{},\"n\":\"/scan\"}}\n{{\"r\":\"t\",\"h\":{},\"n\":\"/obstacles\"}}\n",
        hash("/scan"),
        hash("/obstacles")
    );
    body.push_str(events);
    std::fs::write(run.join("interception/events.jsonl"), body).unwrap();
    run
}

fn write_model(dir: &Path, yaml: &str) -> std::path::PathBuf {
    let path = dir.join("system_model.yaml");
    std::fs::write(&path, yaml).unwrap();
    path
}

fn measure(run: &Path, model: &Path) -> std::process::Output {
    let env = fixtures::install_env();
    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args([
        "measure",
        run.to_str().unwrap(),
        "--model",
        model.to_str().unwrap(),
    ]);
    cmd.output().expect("failed to run play_launch measure")
}

#[test]
fn measure_emits_a_budget_from_a_recorded_run() {
    let dir = tempfile::TempDir::new().unwrap();
    // Costs of 5, 7 and 9 ms; the budget must be the maximum, not the median
    // and not an average.
    let mut events = String::new();
    events.push_str(&invocation(1, 1_000_000_000, 5_000_000, 20_000_000));
    events.push_str(&invocation(2, 2_000_000_000, 7_000_000, 20_000_000));
    events.push_str(&invocation(3, 3_000_000_000, 9_000_000, 20_000_000));
    let run = write_run(dir.path(), &events);
    let model = write_model(dir.path(), MODEL);

    let out = measure(&run, &model);
    assert!(out.status.success(), "measure failed: {out:?}");
    let stdout = String::from_utf8_lossy(&out.stdout);

    assert!(
        stdout.contains("budget: 9000us"),
        "expected the maximum (9 ms) as the budget:\n{stdout}"
    );
    assert!(stdout.contains("/detector:"), "{stdout}");
    // Response is 20 ms against 9 ms of CPU — the gap is the whole point of
    // reporting both, so it has to be visible.
    assert!(stdout.contains("response"), "{stdout}");
    assert!(stdout.contains("NOT a WCET"), "{stdout}");
}

#[test]
fn a_path_with_no_traffic_is_reported_not_omitted() {
    let dir = tempfile::TempDir::new().unwrap();
    let run = write_run(dir.path(), "");
    let model = write_model(dir.path(), MODEL);

    let out = measure(&run, &model);
    assert!(out.status.success(), "measure failed: {out:?}");
    let stdout = String::from_utf8_lossy(&out.stdout);

    // Silence would read as "this path costs nothing".
    assert!(
        stdout.contains("/detector/detect: declared, but no matching traffic"),
        "{stdout}"
    );
    // The header explains what a budget means, so look for an emitted FIELD,
    // not the word. Phase 59 spelling: `budget: <n>us`.
    assert!(!stdout.contains("    budget: "), "{stdout}");
    assert!(!stdout.contains("overrides:"), "{stdout}");
}

#[test]
fn a_run_without_events_names_the_config_that_produces_them() {
    let dir = tempfile::TempDir::new().unwrap();
    let empty = dir.path().join("play_log/empty");
    std::fs::create_dir_all(&empty).unwrap();
    let model = write_model(dir.path(), MODEL);

    let out = measure(&empty, &model);
    assert!(!out.status.success(), "should have failed: {out:?}");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(stderr.contains("interception.enabled: true"), "{stderr}");
}

#[test]
fn a_model_without_contracts_says_there_is_nothing_to_attribute_to() {
    let dir = tempfile::TempDir::new().unwrap();
    let run = write_run(dir.path(), &invocation(1, 1_000_000_000, 5_000_000, 6_000_000));
    let model = write_model(
        dir.path(),
        "meta:\n  version: 1\nstructure:\n  nodes:\n    /detector:\n      scope: \"0\"\n      pkg: demo\n      exec: detector\n",
    );

    let out = measure(&run, &model);
    assert!(!out.status.success(), "should have failed: {out:?}");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(stderr.contains("declares no node paths"), "{stderr}");
}

#[test]
fn the_fragment_parses_as_yaml_and_can_be_appended_to_a_platform_file() {
    let dir = tempfile::TempDir::new().unwrap();
    let run = write_run(dir.path(), &invocation(1, 1_000_000_000, 5_000_000, 6_000_000));
    let model = write_model(dir.path(), MODEL);

    let out = measure(&run, &model);
    let stdout = String::from_utf8_lossy(&out.stdout);

    // The whole output, comments and all, must be valid YAML — its purpose is
    // to be pasted into a platform file.
    let parsed: serde_yaml_ng::Value =
        serde_yaml_ng::from_str(&stdout).expect("measure output must be valid YAML");
    assert_eq!(
        // Canonical spelling: a duration string, not a bare number — the
        // unit is no longer carried by the field name.
        parsed["overrides"]["/detector"]["budget"].as_str(),
        Some("5000us")
    );
}
