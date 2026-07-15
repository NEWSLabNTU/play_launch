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
