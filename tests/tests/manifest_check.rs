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

    let output = run_check(&[
        "--contracts",
        overlay_root.path().to_str().unwrap(),
        launch.to_str().unwrap(),
    ]);
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
fn check_with_no_flags_is_valid_provider_channel_default() {
    // Phase 40.6: no manifest flags are required at all. The provider
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
    let output = run_check(&["/nonexistent/launch.xml"]);
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

    let output = run_check(&[
        "--contracts",
        overlay_root.path().to_str().unwrap(),
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

    let output = Command::new(play_launch_bin())
        .args(["check", "--contracts"])
        .arg(overlay_root.path())
        .arg(&launch_copy)
        .output()
        .expect("failed to run play_launch");
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
    let output = Command::new(play_launch_bin())
        .args(["check"])
        .arg(&launch_copy)
        .output()
        .expect("failed to run play_launch");
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
    let output = Command::new(play_launch_bin())
        .args(["check", "--target", "zephyr"])
        .arg(&launch_copy)
        .output()
        .expect("failed to run play_launch");
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

    let output = Command::new(play_launch_bin())
        .args(["check"])
        .arg(&launch_copy)
        .output()
        .expect("failed to run play_launch");
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
