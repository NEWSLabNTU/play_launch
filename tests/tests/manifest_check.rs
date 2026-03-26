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
fn check_missing_manifest_dir_shows_error() {
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        return;
    }
    // Missing --manifest-dir should error
    let output = Command::new(play_launch_bin())
        .args(["check", launch.to_str().unwrap()])
        .output()
        .expect("failed to run play_launch");
    assert!(
        !output.status.success(),
        "expected nonzero exit without --manifest-dir"
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
