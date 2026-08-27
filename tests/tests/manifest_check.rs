//! Integration tests for `ros-launch-resolve check` CLI.
//!
//! These tests use the simple_test workspace launch files paired with
//! manifest fixtures. No ROS runtime needed — only parsing.
//!
//! `check` used to live on `play_launch`; the CLI verb reshape (2026-08-03)
//! removed it there in favor of `launch --check` (pass/fail gate) and this
//! crate's own `check` (the full diagnostic surface: `--format`, `--rule`,
//! `--explain`, `--export-graph`). Every test here exercises the diagnostic
//! surface, so all of them drive the `ros-launch-resolve` binary directly —
//! `play_launch check ...` now just errors naming both replacements (see
//! `tests/tests/migrated_verbs.rs`).

use play_launch_tests::fixtures;
use std::path::PathBuf;
use std::process::Command;

/// Locate the `ros-launch-resolve` CLI, which owns `check` (mirrors
/// `contract_eject.rs`'s `resolve_cli_bin`). The binary is not installed or
/// on `PATH`, so this looks in the submodule's own target dir and the test
/// skips cleanly (returns `None`) when it has not been built.
fn ros_launch_resolve_bin() -> Option<PathBuf> {
    let root = fixtures::repo_root().join("src/ros-launch-resolve/target");
    for profile in ["debug", "release"] {
        let candidate = root.join(profile).join("ros-launch-resolve");
        if candidate.is_file() {
            return Some(candidate);
        }
    }
    eprintln!(
        "SKIP: ros-launch-resolve CLI not built ({}/{{debug,release}}/ros-launch-resolve \
         missing) — run `cd src/ros-launch-resolve && cargo build` first",
        root.display()
    );
    None
}

fn manifest_fixture_dir() -> PathBuf {
    // Owned by the manifest repository, a git dependency since phase-55 W2 —
    // its checkout path is unpredictable, so it hands the path out itself.
    ros_launch_manifest_check::fixture_dir()
}

fn simple_launch_dir() -> PathBuf {
    fixtures::repo_root().join("tests/fixtures/simple_test/launch")
}

/// Run `ros-launch-resolve check` with given args, or `None` if the binary
/// hasn't been built (caller should skip, like every other test here).
fn run_check(args: &[&str]) -> Option<std::process::Output> {
    let bin = ros_launch_resolve_bin()?;
    Some(
        Command::new(bin)
            .args(["check"])
            .args(args)
            .output()
            .expect("failed to run ros-launch-resolve"),
    )
}

// ── Launch file + overlay contracts mode ──

#[test]
fn check_launch_with_overlay_contracts() {
    // Build a temp overlay tree from the manifest_simple fixture and run
    // `check --contracts <root>` against a direct launch file path (pkg
    // is "_" for raw-path launches — see `resolve_overlay_path`).
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }

    let overlay_root = tempfile::TempDir::new().expect("failed to create overlay root");
    let overlay_launch_dir = overlay_root.path().join("_/launch");
    std::fs::create_dir_all(&overlay_launch_dir).expect("failed to create overlay launch dir");
    let manifest_src = manifest_fixture_dir().join("manifest_simple/manifest.yaml");
    std::fs::copy(
        &manifest_src,
        overlay_launch_dir.join("pure_nodes.contract.yaml"),
    )
    .expect("failed to copy manifest fixture into overlay tree");

    let Some(output) = run_check(&[
        "--contracts",
        overlay_root.path().to_str().unwrap(),
        launch.to_str().unwrap(),
    ]) else {
        return;
    };
    let stderr = String::from_utf8_lossy(&output.stderr);
    // Should parse successfully and discover the overlay contract.
    assert!(
        stderr.contains("Parsed:") || stderr.contains("No manifests"),
        "expected parse output: {stderr}"
    );
}

// ── Single manifest validation (via launch file that matches fixture) ──
// These tests verify the CLI works end-to-end by running the binary.

#[test]
fn check_no_args_shows_help() {
    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = Command::new(bin)
        .args(["check"])
        .output()
        .expect("failed to run ros-launch-resolve");
    assert!(
        !output.status.success(),
        "expected nonzero exit with no args"
    );
}

#[test]
fn check_with_no_flags_is_valid_provider_channel_default() {
    // Phase 40.6: no manifest flags are required at all. The provider
    // sidecar channel is on by default, so `check` with no manifest
    // flags at all is valid — it just finds nothing (no <stem>.contract.yaml
    // sits next to this fixture launch file) and reports "No manifests found".
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        return;
    }
    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = Command::new(bin)
        .args(["check", launch.to_str().unwrap()])
        .output()
        .expect("failed to run ros-launch-resolve");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.success(),
        "expected success with no manifest flags (provider channel is on by default): {stderr}"
    );
    assert!(
        stderr.contains("No manifests found"),
        "expected 'No manifests found' message, got: {stderr}"
    );
}

#[test]
fn check_provider_channel_sidecar_next_to_launch_file() {
    // Copy a fixture manifest next to a launch file in a temp dir as
    // `<stem>.contract.yaml`, then run `check` with no manifest flags —
    // the provider channel should discover and load it.
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }

    let tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_copy = tmp.path().join("pure_nodes.launch.xml");
    std::fs::copy(&launch, &launch_copy).expect("failed to copy launch file");

    let manifest_src = manifest_fixture_dir().join("manifest_simple/manifest.yaml");
    let sidecar = tmp.path().join("pure_nodes.contract.yaml");
    std::fs::copy(&manifest_src, &sidecar).expect("failed to copy manifest fixture as sidecar");

    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = Command::new(bin)
        .args(["check", launch_copy.to_str().unwrap()])
        .output()
        .expect("failed to run ros-launch-resolve");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("Parsed:"),
        "expected parse output: {stderr}"
    );
    assert!(
        stderr.contains("manifest(s) checked"),
        "expected the provider sidecar to be loaded and checked, got: {stderr}"
    );
    assert!(
        !stderr.contains("No manifests found"),
        "provider sidecar should have been discovered, got: {stderr}"
    );
}

#[test]
fn check_nonexistent_launch_file_exits_nonzero() {
    let Some(output) = run_check(&["/nonexistent/launch.xml"]) else {
        return;
    };
    assert!(
        !output.status.success(),
        "expected nonzero exit for missing launch file"
    );
}

#[test]
fn check_format_json() {
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        return;
    }

    let overlay_root = tempfile::TempDir::new().expect("failed to create overlay root");
    let overlay_launch_dir = overlay_root.path().join("_/launch");
    std::fs::create_dir_all(&overlay_launch_dir).expect("failed to create overlay launch dir");
    let manifest_src = manifest_fixture_dir().join("manifest_simple/manifest.yaml");
    std::fs::copy(
        &manifest_src,
        overlay_launch_dir.join("pure_nodes.contract.yaml"),
    )
    .expect("failed to copy manifest fixture into overlay tree");

    let Some(output) = run_check(&[
        "--contracts",
        overlay_root.path().to_str().unwrap(),
        "--format",
        "json",
        launch.to_str().unwrap(),
    ]) else {
        return;
    };
    // Should complete without crash
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("Parsed:") || stderr.contains("No manifests"),
        "stderr: {stderr}"
    );
}

// ── Overlay channel (Phase 40.3) ──

#[test]
fn check_overlay_channel_contract_dir() {
    // Build a temp overlay tree `<root>/_/launch/<stem>.contract.yaml`
    // (pkg is "_" because the launch file is referenced by a raw path,
    // not a ROS package) and run `check --contracts <root>`. The overlay
    // channel should discover and load it.
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }

    let launch_tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_copy = launch_tmp.path().join("pure_nodes.launch.xml");
    std::fs::copy(&launch, &launch_copy).expect("failed to copy launch file");

    let overlay_root = tempfile::TempDir::new().expect("failed to create overlay root");
    let overlay_launch_dir = overlay_root.path().join("_/launch");
    std::fs::create_dir_all(&overlay_launch_dir).expect("failed to create overlay launch dir");
    let manifest_src = manifest_fixture_dir().join("manifest_simple/manifest.yaml");
    std::fs::copy(
        &manifest_src,
        overlay_launch_dir.join("pure_nodes.contract.yaml"),
    )
    .expect("failed to copy manifest fixture into overlay tree");

    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = Command::new(bin)
        .args(["check", "--contracts"])
        .arg(overlay_root.path())
        .arg(&launch_copy)
        .output()
        .expect("failed to run ros-launch-resolve");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("manifest(s) checked"),
        "expected the overlay contract to be loaded and checked, got: {stderr}"
    );
    assert!(
        stderr.contains("1 overlay"),
        "expected the overlay channel to supply the contract, got: {stderr}"
    );
    assert!(
        stderr.contains("0 provider"),
        "no provider sidecar exists next to the launch file, got: {stderr}"
    );
}

#[test]
fn check_overlay_beats_provider_precedence() {
    // Same temp dir holds the launch file AND a provider sidecar
    // `<stem>.contract.yaml`. A separate overlay tree supplies a
    // *different* contract for the same stem via `--contracts`. Per the
    // resolution order (overlay > provider > legacy), the overlay
    // contract must win — verified two ways: (1) the per-channel summary
    // counts attribute the contract to `overlay`, not `provider`; (2) the
    // overlay contract's distinguishing content (a topic declared with
    // only a subscriber, so it should be recognized as its own thing)
    // shows up as a cross-scope diagnostic that the provider contract
    // does not produce.
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }

    let launch_tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_copy = launch_tmp.path().join("pure_nodes.launch.xml");
    std::fs::copy(&launch, &launch_copy).expect("failed to copy launch file");

    // Provider sidecar: the clean fixture manifest (0 diagnostics).
    let manifest_src = manifest_fixture_dir().join("manifest_simple/manifest.yaml");
    std::fs::copy(
        &manifest_src,
        launch_tmp.path().join("pure_nodes.contract.yaml"),
    )
    .expect("failed to copy manifest fixture as provider sidecar");

    // Overlay contract: distinguishable content — declares a
    // subscriber-only topic with a marker name, which triggers a
    // "0 publishers" cross-scope diagnostic naming that marker.
    let overlay_root = tempfile::TempDir::new().expect("failed to create overlay root");
    let overlay_launch_dir = overlay_root.path().join("_/launch");
    std::fs::create_dir_all(&overlay_launch_dir).expect("failed to create overlay launch dir");
    let overlay_contract = r#"
version: 1

nodes:
  listener:
    sub:
      overlay_marker_topic: {}

topics:
  overlay_marker_topic:
    type: std_msgs/msg/String
    sub: [listener/overlay_marker_topic]
"#;
    std::fs::write(
        overlay_launch_dir.join("pure_nodes.contract.yaml"),
        overlay_contract,
    )
    .expect("failed to write overlay contract");

    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = Command::new(bin)
        .args(["check", "--contracts"])
        .arg(overlay_root.path())
        .arg(&launch_copy)
        .output()
        .expect("failed to run ros-launch-resolve");
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        stderr.contains("1 overlay"),
        "expected overlay channel to win over the provider sidecar, got: {stderr}"
    );
    assert!(
        stderr.contains("0 provider"),
        "provider sidecar exists but overlay should take precedence, got: {stderr}"
    );
    assert!(
        stderr.contains("overlay_marker_topic"),
        "expected the overlay contract's distinguishing content (marker topic) to be in \
         effect, got: {stderr}"
    );
}

// ── Platform-file shipping channels (Phase 41.3) ──
//
// Same channel order/discovery as contracts, but for the scheduling
// platform file: `--sched <path>` (explicit, tested elsewhere via
// `rt_workspace.rs`) > overlay `<root>/<pkg>/launch/<stem>.system.<target>.yaml`
// > provider sidecar `<launch-file-dir>/<stem>.system.<target>.yaml`.

/// Minimal valid v2 platform file: `rate_monotonic` (the `manual` mapper is
/// reachable only via the legacy `.toml` bridge, not raw v2 `.yaml` — see
/// `sched_loader::derive_sched_plan`'s "requires a legacy tiers+assign spec"
/// error) with the `rt_priority_band` its posix resources require.
fn minimal_platform_file(target: &str) -> String {
    format!(
        "target: {target}\nmapper: rate_monotonic\nresources:\n  rt_priority_band: {{ min: 10, max: 40 }}\n"
    )
}

#[test]
fn check_sched_overlay_platform_file_beats_provider_sidecar() {
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }

    let launch_tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_copy = launch_tmp.path().join("pure_nodes.launch.xml");
    std::fs::copy(&launch, &launch_copy).expect("failed to copy launch file");

    // Provider sidecar next to the launch file.
    std::fs::write(
        launch_tmp.path().join("pure_nodes.system.posix.yaml"),
        minimal_platform_file("posix"),
    )
    .expect("failed to write provider sidecar platform file");

    // Overlay platform file for the same target — must win.
    let overlay_root = tempfile::TempDir::new().expect("failed to create overlay root");
    let overlay_launch_dir = overlay_root.path().join("_/launch");
    std::fs::create_dir_all(&overlay_launch_dir).expect("failed to create overlay launch dir");
    std::fs::write(
        overlay_launch_dir.join("pure_nodes.system.posix.yaml"),
        minimal_platform_file("posix"),
    )
    .expect("failed to write overlay platform file");

    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = Command::new(bin)
        .args(["check", "--contracts"])
        .arg(overlay_root.path())
        .arg(&launch_copy)
        .output()
        .expect("failed to run ros-launch-resolve");
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "expected success (no --sched needed; resolved via channels): {stderr}"
    );
    assert!(
        stderr.contains("Scheduling platform file [overlay]:"),
        "expected the overlay platform file to win over the provider sidecar, got: {stderr}"
    );
    assert!(
        stderr.contains(
            overlay_launch_dir
                .join("pure_nodes.system.posix.yaml")
                .to_str()
                .unwrap()
        ),
        "expected the resolved path to be the overlay file, got: {stderr}"
    );
    assert!(
        stderr.contains("Scheduling (posix, mapper=rate_monotonic)"),
        "expected the resolved platform file to actually be parsed/derived, got: {stderr}"
    );
}

#[test]
fn check_sched_provider_sidecar_only_platform_file() {
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }

    let launch_tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_copy = launch_tmp.path().join("pure_nodes.launch.xml");
    std::fs::copy(&launch, &launch_copy).expect("failed to copy launch file");

    std::fs::write(
        launch_tmp.path().join("pure_nodes.system.posix.yaml"),
        minimal_platform_file("posix"),
    )
    .expect("failed to write provider sidecar platform file");

    // No --contracts at all: provider channel is the only one available.
    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = Command::new(bin)
        .args(["check"])
        .arg(&launch_copy)
        .output()
        .expect("failed to run ros-launch-resolve");
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "expected success via the provider-sidecar-only channel: {stderr}"
    );
    assert!(
        stderr.contains("Scheduling platform file [provider]:"),
        "expected the provider sidecar to be resolved, got: {stderr}"
    );
    assert!(
        stderr.contains("Scheduling (posix, mapper=rate_monotonic)"),
        "expected the resolved platform file to actually be parsed/derived, got: {stderr}"
    );
}

#[test]
fn check_sched_wrong_target_platform_file_is_ignored() {
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }

    let launch_tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_copy = launch_tmp.path().join("pure_nodes.launch.xml");
    std::fs::copy(&launch, &launch_copy).expect("failed to copy launch file");

    // Only a `posix` platform file is shipped.
    std::fs::write(
        launch_tmp.path().join("pure_nodes.system.posix.yaml"),
        minimal_platform_file("posix"),
    )
    .expect("failed to write provider sidecar platform file");

    // Requesting `zephyr` must not match the posix-named file — scheduling
    // stays disabled, and this is NOT an error (distinct from an explicit
    // `--sched` pointing at a file whose `target:` mismatches, which does
    // error — see `sched_loader::derive_target_mismatch_errors`).
    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = Command::new(bin)
        .args(["check", "--target", "zephyr"])
        .arg(&launch_copy)
        .output()
        .expect("failed to run ros-launch-resolve");
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "wrong-target platform file must not error, just leave scheduling disabled: {stderr}"
    );
    assert!(
        !stderr.contains("Scheduling platform file"),
        "expected no platform file resolved for the mismatched target, got: {stderr}"
    );
    assert!(
        !stderr.contains("Scheduling ("),
        "expected no scheduling table since nothing resolved, got: {stderr}"
    );
}

#[test]
fn check_sched_no_platform_file_leaves_scheduling_disabled() {
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }

    let launch_tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_copy = launch_tmp.path().join("pure_nodes.launch.xml");
    std::fs::copy(&launch, &launch_copy).expect("failed to copy launch file");
    // No platform file anywhere (no overlay, no provider sidecar).

    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = Command::new(bin)
        .args(["check"])
        .arg(&launch_copy)
        .output()
        .expect("failed to run ros-launch-resolve");
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "no platform file anywhere must not be an error: {stderr}"
    );
    assert!(
        !stderr.contains("Scheduling platform file"),
        "expected no platform file resolution line, got: {stderr}"
    );
    assert!(
        !stderr.contains("Scheduling ("),
        "expected no scheduling table since nothing resolved, got: {stderr}"
    );
}

// ── W2 carry-forward #3 (Phase 41.4): legacy `.toml` platform file + a live
// contract index whose rate facts contradict the manually-assigned priority
// order → warning only, `check` still succeeds. ──

#[test]
fn check_legacy_toml_with_contradicting_contract_facts_warns_but_succeeds() {
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }

    let launch_tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_copy = launch_tmp.path().join("pure_nodes.launch.xml");
    std::fs::copy(&launch, &launch_copy).expect("failed to copy launch file");

    // Contract: talker publishes at 100 Hz, listener at 10 Hz.
    let overlay_root = tempfile::TempDir::new().expect("failed to create overlay root");
    let overlay_launch_dir = overlay_root.path().join("_/launch");
    std::fs::create_dir_all(&overlay_launch_dir).expect("failed to create overlay launch dir");
    std::fs::write(
        overlay_launch_dir.join("pure_nodes.contract.yaml"),
        "\
version: 1
nodes:
  talker:
    pub:
      chatter:
        min_rate_hz: 100
  listener:
    pub:
      status:
        min_rate_hz: 10
topics:
  chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
    rate_hz: 100
  status:
    type: std_msgs/msg/String
    pub: [listener/status]
    rate_hz: 10
",
    )
    .expect("failed to write overlay contract");

    // Legacy manual-mapper platform file (explicit-path only): pins talker
    // (the FASTER node) to a LOWER priority than listener — contradicts the
    // contract's rate order.
    let system_toml = launch_tmp.path().join("system.toml");
    std::fs::write(
        &system_toml,
        "\
[tiers.low]
class = \"real_time\"
[tiers.low.posix]
priority = 10
sched_class = \"SCHED_FIFO\"

[tiers.high]
class = \"real_time\"
[tiers.high.posix]
priority = 40
sched_class = \"SCHED_FIFO\"

[[assign]]
tier = \"low\"
nodes = [\"talker\"]

[[assign]]
tier = \"high\"
nodes = [\"listener\"]
",
    )
    .expect("failed to write legacy system.toml");

    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = Command::new(bin)
        .args(["check", "--contracts"])
        .arg(overlay_root.path())
        .arg("--sched")
        .arg(&system_toml)
        .arg(&launch_copy)
        .output()
        .expect("failed to run ros-launch-resolve");
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        output.status.success(),
        "contradicting facts must be a warning, not a failure: {stderr}"
    );
    assert!(
        stderr.contains("contradicts") && stderr.contains("rate_hz"),
        "expected a rate contradiction warning citing both nodes, got: {stderr}"
    );
    // The warning cites both the contract file (fact side) and the
    // platform file (priority side) — Phase 41.4's file-citation addition.
    assert!(
        stderr.contains("contract:") && stderr.contains("platform file:"),
        "expected the warning to cite both source files, got: {stderr}"
    );
}

// ── chain_aware end-to-end: chain fixture + platform file + --explain (44.4) ──

/// A provider-sidecar contract declaring a two-segment chain across a timer
/// boundary, checked with a `chain_aware` platform file: `--explain` must
/// show chain provenance for the segment members.
#[test]
fn check_chain_aware_explain_shows_chain_provenance() {
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }

    let tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_copy = tmp.path().join("pure_nodes.launch.xml");
    std::fs::copy(&launch, &launch_copy).expect("failed to copy launch file");

    std::fs::write(
        tmp.path().join("pure_nodes.contract.yaml"),
        r#"version: 1
nodes:
  talker:
    pub:
      chatter: {}
    paths:
      tick:
        trigger: { timer: { rate_hz: 10 } }
        output: [chatter]
        max_latency_ms: 5
  listener:
    sub:
      chatter: {}
    pub:
      done: {}
    paths:
      handle:
        trigger: { input: [chatter] }
        output: [done]
        max_latency_ms: 20
chains:
  test_chain:
    semantics: reaction
    max_latency_ms: 500
    segments:
      - { scope: /, path: tick }
      - { via: /chatter }
      - { scope: /, path: handle }
topics:
  chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
    sub: [listener/chatter]
    rate_hz: 10
  done:
    type: std_msgs/msg/String
    pub: [listener/done]
"#,
    )
    .expect("failed to write chain contract");

    let platform = tmp.path().join("system.posix.yaml");
    std::fs::write(
        &platform,
        r#"target: posix
mapper: chain_aware
resources:
  rt_priority_band: { min: 10, max: 40 }
"#,
    )
    .expect("failed to write platform file");

    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = Command::new(bin)
        .args([
            "check",
            "--sched",
            platform.to_str().unwrap(),
            "--explain",
            launch_copy.to_str().unwrap(),
        ])
        .output()
        .expect("failed to run ros-launch-resolve");
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );

    assert!(
        output.status.success(),
        "check --sched chain_aware --explain failed:\n{combined}"
    );
    assert!(
        combined.contains("chain_aware"),
        "expected chain_aware provenance in explain output:\n{combined}"
    );
    assert!(
        combined.contains("test_chain"),
        "expected the chain name in explain provenance:\n{combined}"
    );
}

// ── Phase 68 W1.d: every previously write-only field now has a rule that can
//    reject it. A field nothing can fail on is indistinguishable from a
//    comment, so each of these is asserted to FIRE on a fixture written to
//    violate it.

/// Run `play_launch check` on a fixture and return its combined output.
fn check_fixture(dir: &str) -> String {
    let launch = fixtures::repo_root()
        .join("tests/fixtures")
        .join(dir)
        .join("launch/bringup.launch.xml");
    let out = Command::new(fixtures::play_launch_bin())
        .arg("check")
        .arg(&launch)
        .output()
        .expect("play_launch check runs");
    format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    )
}

#[test]
fn w1d_write_only_fields_now_have_rules_that_fail() {
    let out = check_fixture("contract_w1d");
    for rule in ["jitter-feasibility", "lifespan-age", "sync-budget"] {
        assert!(
            out.contains(rule),
            "expected {rule} to fire on contract_w1d; got:\n{out}"
        );
    }
    // The pre-existing lower bound on the same sync window fires too: the
    // window must be >= the slowest input's period (100ms) and <= the path's
    // budget (20ms), an empty interval neither rule alone can report.
    assert!(out.contains("sync-feasibility"), "got:\n{out}");
    // And its message no longer leaks a Rust expression at the reader
    // (manifest v0.1.13).
    assert!(
        !out.contains("as_millis_f64"),
        "a diagnostic printed Rust source:\n{out}"
    );
}

#[test]
fn phase67_vocabulary_checks_out_end_to_end() {
    let out = check_fixture("contract_concurrency");
    // Derived callback groups: the route through `boxes` may be blocked by
    // the sibling `masks` path they share a group with.
    assert!(out.contains("path-exclusion"), "got:\n{out}");
    assert!(out.contains("to_masks"), "the blocking sibling is named:\n{out}");
    // The per-manifest sum is SUPERSEDED where a real route exists, so the
    // two must not both report a total for one path.
    assert!(
        !out.contains("sum of node latencies"),
        "the superseded per-manifest fallback reappeared:\n{out}"
    );
}

// ── Phase 68 W2: the mapper acts on the phase 67 vocabulary. Two of these
//    are REFUSALS, which is the half that matters for safety.

fn check_with_sched(dir: &str) -> String {
    let base = fixtures::repo_root().join("tests/fixtures").join(dir).join("launch");
    let out = Command::new(fixtures::play_launch_bin())
        .arg("check")
        .arg(base.join("bringup.launch.xml"))
        .arg("--sched")
        .arg(base.join("bringup.system.posix.yaml"))
        .output()
        .expect("play_launch check --sched runs");
    format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    )
}

#[test]
fn w2_reservation_is_refused_for_a_node_that_claims_concurrency() {
    let out = check_with_sched("contract_w2");
    assert!(
        out.contains("/w/rt") && out.contains("run concurrently"),
        "the refusal must name the node and the reason:\n{out}"
    );
    // A reservation is per-thread; the leader-only reservation cannot cover
    // callbacks that may run elsewhere (phase 60 F2, now detectable).
    assert!(out.contains("per-thread"), "got:\n{out}");
}

#[test]
fn w2_an_unenforceable_miss_action_is_reported_not_downgraded() {
    let out = check_with_sched("contract_w2");
    assert!(
        out.contains("miss.action `abort`") && out.contains("Linux cannot enforce"),
        "got:\n{out}"
    );
}

#[test]
fn w2_a_jitter_bound_on_a_best_effort_node_is_reported() {
    let out = check_with_sched("contract_w2");
    assert!(
        out.contains("/w/slow") && out.contains("max_jitter"),
        "got:\n{out}"
    );
    // Reported, never promoted: moving a node into the RT band changes what
    // it preempts and what it starves.
    assert!(out.contains("not promoted automatically"), "got:\n{out}");
}

/// The derived overrun flag has to survive the MODEL boundary — `up` reads the
/// model and never the platform file, and rebuilds `overrun` from
/// `deadline_policy`. A reservation that loses its notification on round-trip
/// is phase 60's defect in a new place.
#[test]
fn w2_derived_overrun_reaches_the_model() {
    let base = fixtures::repo_root().join("tests/fixtures/contract_w2/launch");
    let out_path = std::env::temp_dir().join("play_launch_w2_model.yaml");
    let status = Command::new(fixtures::play_launch_bin())
        .arg("resolve")
        .arg(base.join("bringup.launch.xml"))
        .arg("--sched")
        .arg(base.join("bringup.system.posix.yaml"))
        .arg("-o")
        .arg(&out_path)
        .status()
        .expect("resolve runs");
    assert!(status.success(), "resolve failed");
    let model = std::fs::read_to_string(&out_path).expect("model written");
    assert!(
        model.contains("deadline_policy: fault"),
        "a `miss:` declaration must reach the model as deadline_policy:\n{model}"
    );
    assert!(model.contains("sched_class: SCHED_DEADLINE"), "{model}");
    // And the node that was refused a reservation stays on fixed priority.
    assert!(model.contains("sched_class: SCHED_FIFO"), "{model}");
    let _ = std::fs::remove_file(&out_path);
}
