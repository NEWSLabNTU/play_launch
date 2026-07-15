//! Integration tests for `play_launch check` CLI.
//!
//! These tests use the simple_test workspace launch files paired with
//! manifest fixtures. No ROS runtime needed — only parsing.

use play_launch_tests::fixtures;
use std::path::PathBuf;
use std::process::Command;

/// Find the play_launch binary, preferring the cargo debug build.
fn play_launch_bin() -> PathBuf {
    let cargo_bin = fixtures::repo_root().join("target/debug/play_launch");
    if cargo_bin.is_file() {
        return cargo_bin;
    }
    fixtures::play_launch_bin()
}

fn manifest_fixture_dir() -> PathBuf {
    fixtures::repo_root().join("src/ros-launch-manifest/tests/fixtures")
}

fn simple_launch_dir() -> PathBuf {
    fixtures::repo_root().join("tests/fixtures/simple_test/launch")
}

/// Run `play_launch check` with given args.
fn run_check(args: &[&str]) -> std::process::Output {
    Command::new(play_launch_bin())
        .args(["check"])
        .args(args)
        .output()
        .expect("failed to run play_launch")
}

// ── Launch file + manifest-dir mode ──

#[test]
fn check_launch_with_simple_manifest_dir() {
    // Use a direct launch file path with manifest_simple fixtures
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }
    let output = run_check(&[
        "--manifest-dir",
        manifest_fixture_dir().to_str().unwrap(),
        launch.to_str().unwrap(),
    ]);
    let stderr = String::from_utf8_lossy(&output.stderr);
    // Should parse successfully (may or may not find manifests)
    assert!(
        stderr.contains("Parsed:") || stderr.contains("No manifests"),
        "expected parse output: {stderr}"
    );
}

// ── Single manifest validation (via launch file that matches fixture) ──
// These tests verify the CLI works end-to-end by running the binary.

#[test]
fn check_no_args_shows_help() {
    let output = Command::new(play_launch_bin())
        .args(["check"])
        .output()
        .expect("failed to run play_launch");
    assert!(
        !output.status.success(),
        "expected nonzero exit with no args"
    );
}

#[test]
fn check_without_manifest_dir_is_valid_provider_channel_default() {
    // Phase 40.2: --manifest-dir is no longer required. The provider
    // sidecar channel is on by default, so `check` with no manifest
    // flags at all is valid — it just finds nothing (no <stem>.contract.yaml
    // sits next to this fixture launch file) and reports "No manifests found".
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        return;
    }
    let output = Command::new(play_launch_bin())
        .args(["check", launch.to_str().unwrap()])
        .output()
        .expect("failed to run play_launch");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.success(),
        "expected success without --manifest-dir (provider channel is on by default): {stderr}"
    );
    assert!(
        stderr.contains("No manifests found"),
        "expected 'No manifests found' message, got: {stderr}"
    );
}

#[test]
fn check_provider_channel_sidecar_next_to_launch_file() {
    // Copy a fixture manifest next to a launch file in a temp dir as
    // `<stem>.contract.yaml`, then run `check` WITHOUT --manifest-dir —
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

    let output = Command::new(play_launch_bin())
        .args(["check", launch_copy.to_str().unwrap()])
        .output()
        .expect("failed to run play_launch");
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
    let output = run_check(&[
        "--manifest-dir",
        manifest_fixture_dir().to_str().unwrap(),
        "/nonexistent/launch.xml",
    ]);
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
    let output = run_check(&[
        "--manifest-dir",
        manifest_fixture_dir().to_str().unwrap(),
        "--format",
        "json",
        launch.to_str().unwrap(),
    ]);
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
    // not a ROS package) and run `check --contracts <root>` with no
    // --manifest-dir. The overlay channel should discover and load it.
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

    let output = Command::new(play_launch_bin())
        .args(["check", "--contracts"])
        .arg(overlay_root.path())
        .arg(&launch_copy)
        .output()
        .expect("failed to run play_launch");
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

    let output = Command::new(play_launch_bin())
        .args(["check", "--contracts"])
        .arg(overlay_root.path())
        .arg(&launch_copy)
        .output()
        .expect("failed to run play_launch");
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
