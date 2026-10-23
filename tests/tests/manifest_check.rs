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
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    process::Command,
    sync::OnceLock,
};

/// The environment every process this file spawns runs in (issue #0051).
///
/// Sourced once: `install_env()` shells out to `bash` to source ROS and the
/// colcon tree, and this file spawns thirty-odd times.
fn test_env() -> &'static HashMap<String, String> {
    static ENV: OnceLock<HashMap<String, String>> = OnceLock::new();
    ENV.get_or_init(fixtures::install_env)
}

/// Build a `Command` for the standalone `ros-launch-resolve` binary in the
/// SAME environment `fixtures::play_launch_cmd` gives every other suite.
///
/// The path comes from `ros_launch_resolve_bin()` below rather than from
/// `fixtures` (which has no helper for layer 2's own `target/`), so only the
/// environment is shared — and the environment is the half that must not
/// differ between suites. Spawning bare here is what issue #0051 was.
fn resolve_cmd(bin: &Path) -> Command {
    let mut cmd = Command::new(bin);
    fixtures::apply_test_env(&mut cmd, test_env());
    cmd
}

/// Build a `Command` for `play_launch` — the environment-carrying helper, not
/// a bare `Command::new` (issue #0051: without the colcon library path this
/// binary dies at `libplay_launch_msgs__rosidl_typesupport_c.so`).
fn play_launch_cmd() -> Command {
    fixtures::play_launch_cmd(test_env())
}

/// Locate the `ros-launch-resolve` CLI, which owns `check`. One definition,
/// in `fixtures` — this file and `contract_eject.rs` each held a copy that
/// searched `src/ros-launch-resolve/target` only, and so tested whatever
/// leftover binary sat there on a machine whose `.cargo/config.toml`
/// redirects the target directory elsewhere. The test skips cleanly (returns
/// `None`) when the CLI has not been built.
fn ros_launch_resolve_bin() -> Option<PathBuf> {
    fixtures::resolve_cli_bin()
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
        resolve_cmd(&bin)
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
    let output = resolve_cmd(&bin)
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
    let output = resolve_cmd(&bin)
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
    let output = resolve_cmd(&bin)
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
    let output = resolve_cmd(&bin)
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
    let output = resolve_cmd(&bin)
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
    let output = resolve_cmd(&bin)
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
    let output = resolve_cmd(&bin)
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
    let output = resolve_cmd(&bin)
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
    let output = resolve_cmd(&bin)
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
    let output = resolve_cmd(&bin)
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

// ── Issue #0056: the contradiction scan reads the DECLARED facts, and phase
//    78's ranking ruling stays intact. Three tests, because the two claims
//    fail in opposite directions: a scan that reads nothing is silent (the
//    defect), and a mapper that reads a promise ranks by it (what phase 78
//    removed). ──

/// The same promise-only shape the positive test above declares: rates stated
/// as `pub.<ep>.min_rate_hz` plus a topic-level `rate_hz`, and **no `paths:`
/// at all**, so no node carries a timer trigger. This is how the format
/// reference teaches an author to state a rate, and since phase 78 it is also
/// the shape that leaves `MapperNode.rate_hz` unset on every node.
fn promise_only_rate_contract() -> &'static str {
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
"
}

/// Write `contract` as the overlay contract for `pure_nodes.launch.xml`,
/// returning the overlay root to pass to `--contracts`.
fn overlay_with_contract(contract: &str) -> tempfile::TempDir {
    let overlay_root = tempfile::TempDir::new().expect("failed to create overlay root");
    let overlay_launch_dir = overlay_root.path().join("_/launch");
    std::fs::create_dir_all(&overlay_launch_dir).expect("failed to create overlay launch dir");
    std::fs::write(
        overlay_launch_dir.join("pure_nodes.contract.yaml"),
        contract,
    )
    .expect("failed to write overlay contract");
    overlay_root
}

/// A legacy manual-mapper platform file assigning `high_node` priority 40 and
/// `low_node` priority 10.
fn legacy_toml_pinning(high_node: &str, low_node: &str) -> String {
    format!(
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
tier = \"high\"
nodes = [\"{high_node}\"]

[[assign]]
tier = \"low\"
nodes = [\"{low_node}\"]
"
    )
}

/// The negative half of the contradiction scan: the same promise-only contract,
/// with a hand-written table that AGREES with it (the 100 Hz node high, the
/// 10 Hz node low) — the scan runs over two differing rates and must report
/// nothing. Without this, a scan that read the declared facts and one that read
/// nothing at all would be indistinguishable in the positive test alone.
#[test]
fn check_promise_rates_agreeing_with_the_priority_table_stay_silent() {
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }

    let launch_tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_copy = launch_tmp.path().join("pure_nodes.launch.xml");
    std::fs::copy(&launch, &launch_copy).expect("failed to copy launch file");

    let overlay_root = overlay_with_contract(promise_only_rate_contract());
    let system_toml = launch_tmp.path().join("system.toml");
    // talker promises 100 Hz, listener 10 Hz — so talker high is the order the
    // contract implies.
    std::fs::write(&system_toml, legacy_toml_pinning("talker", "listener"))
        .expect("failed to write legacy system.toml");

    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = resolve_cmd(&bin)
        .args(["check", "--contracts"])
        .arg(overlay_root.path())
        .arg("--sched")
        .arg(&system_toml)
        .arg(&launch_copy)
        .output()
        .expect("failed to run ros-launch-resolve");
    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(output.status.success(), "check must succeed: {stderr}");
    assert!(
        !stderr.contains("contradicts"),
        "a priority table that agrees with the declared rates must report no \
         contradiction, got: {stderr}"
    );
}

/// Phase 78's actual ruling, asserted end to end: a promise is not a period, so
/// `rate_monotonic` must not rank by one. The same promise-only contract that
/// DOES produce a contradiction warning above must leave both nodes in the
/// synthesized default tier at priority 0 — if a promise ever reaches
/// `MapperNode.rate_hz` (the tempting one-line "fix" for issue #0056), talker
/// lands in the RT band above listener and this fails.
#[test]
fn check_promise_rates_are_not_ranked_by_rate_monotonic() {
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }

    let launch_tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_copy = launch_tmp.path().join("pure_nodes.launch.xml");
    std::fs::copy(&launch, &launch_copy).expect("failed to copy launch file");

    let overlay_root = overlay_with_contract(promise_only_rate_contract());
    let platform = launch_tmp.path().join("pure_nodes.system.posix.yaml");
    std::fs::write(&platform, minimal_platform_file("posix"))
        .expect("failed to write platform file");

    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = resolve_cmd(&bin)
        .args(["check", "--explain", "--contracts"])
        .arg(overlay_root.path())
        .arg("--sched")
        .arg(&platform)
        .arg(&launch_copy)
        .output()
        .expect("failed to run ros-launch-resolve");
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );

    assert!(
        output.status.success(),
        "check --explain must succeed: {combined}"
    );
    // One tier, holding both nodes: nothing was ranked.
    assert!(
        combined.contains("mapper=rate_monotonic): 1 tier(s)"),
        "expected rate_monotonic to derive a single (default) tier from \
         promise-only rates, got: {combined}"
    );
    for node in ["/pure_test/talker", "/pure_test/listener"] {
        let line = combined
            .lines()
            .find(|l| l.starts_with(node) && l.contains("SCHED_OTHER"))
            .unwrap_or_else(|| {
                panic!("no --explain row for {node} in: {combined}");
            });
        assert!(
            line.contains("default (no timing facts)"),
            "a promise must not be a timing fact the mapper ranks by, got: {line}"
        );
    }
}

/// `deadline_priority_contradictions` had no integration coverage at all
/// (issue #0056, third gap): the same shape as the rate test, with declared
/// `max_latency` instead of rates. talker's 5 ms budget is tighter than
/// listener's 50 ms, and the hand-written table inverts them.
#[test]
fn check_legacy_toml_with_contradicting_deadline_facts_warns_but_succeeds() {
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }

    let launch_tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_copy = launch_tmp.path().join("pure_nodes.launch.xml");
    std::fs::copy(&launch, &launch_copy).expect("failed to copy launch file");

    // Equal rates on both nodes, so the only order the contract implies is the
    // deadline one — a rate contradiction cannot account for the warning.
    let overlay_root = overlay_with_contract(
        "\
version: 1
nodes:
  talker:
    paths:
      tight:
        trigger: { timer: { rate_hz: 10 } }
        output: [chatter]
        max_latency: 5ms
  listener:
    paths:
      loose:
        trigger: { timer: { rate_hz: 10 } }
        output: [status]
        max_latency: 50ms
topics:
  chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
  status:
    type: std_msgs/msg/String
    pub: [listener/status]
",
    );
    let system_toml = launch_tmp.path().join("system.toml");
    // listener (the LOOSER deadline) high, talker (5 ms) low — inverted.
    std::fs::write(&system_toml, legacy_toml_pinning("listener", "talker"))
        .expect("failed to write legacy system.toml");

    let Some(bin) = ros_launch_resolve_bin() else {
        return;
    };
    let output = resolve_cmd(&bin)
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
        stderr.contains("contradicts") && stderr.contains("deadline_us"),
        "expected a deadline contradiction warning citing both nodes, got: {stderr}"
    );
    assert!(
        !stderr.contains("rate_hz` order"),
        "the rates are equal here — no rate contradiction should be reported: {stderr}"
    );
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
        max_latency: 5ms
  listener:
    sub:
      chatter: {}
    pub:
      done: {}
    paths:
      handle:
        trigger: { input: [chatter] }
        output: [done]
        max_latency: 20ms
paths:
  test_chain:
    trigger: { input: [/chatter] }
    output: [/done]
    max_latency: 500ms
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
    let output = resolve_cmd(&bin)
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
    let out = play_launch_cmd()
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
    // `jitter-range` (phase 70 W2) fires on the same declaration from the
    // other side: a 0..200ms range cannot fit a 5ms jitter bound whatever the
    // route's sampling jitter is.
    for rule in [
        "jitter-feasibility",
        "jitter-range",
        "lifespan-age",
        "sync-budget",
    ] {
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
    assert!(
        out.contains("to_masks"),
        "the blocking sibling is named:\n{out}"
    );
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
    let base = fixtures::repo_root()
        .join("tests/fixtures")
        .join(dir)
        .join("launch");
    let out = play_launch_cmd()
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
    let status = play_launch_cmd()
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

/// Phase 68 W4's gate, as a test rather than a shell probe.
///
/// `contract_derived_chain` is `rt_workspace`'s system with `chains:` and
/// `segments:` removed entirely — the route is derivable from
/// `trigger`/`output`. The mapper must reach the SAME scheduling decisions from
/// the derived route as from the authored one.
///
/// The assertion is on the PROVENANCE, not the priorities. Before the derived
/// route reached the mapper it fell back to ranking nodes by budget, which on
/// this three-node system produces the same two priorities by coincidence — so
/// a numeric check passed while the derivation was not being used at all. That
/// false pass is the reason this test reads the string.
#[test]
fn w4_the_mapper_reads_the_derived_route_not_just_authored_segments() {
    let base = fixtures::repo_root().join("tests/fixtures/contract_derived_chain/launch");
    let out = play_launch_cmd()
        .arg("check")
        .arg(base.join("bringup.launch.xml"))
        .arg("--sched")
        .arg(base.join("bringup.system.posix.yaml"))
        .arg("--explain")
        .output()
        .expect("check --explain runs");
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );

    assert!(
        !text.contains("non-chain"),
        "the mapper fell back to per-node ranking — the derived route did not \
         reach it:\n{text}"
    );
    assert!(
        text.contains("points_to_cmd segment drain"),
        "expected chain-aware drain provenance from the DERIVED route:\n{text}"
    );
    assert!(
        text.contains("points_to_cmd boundary RM"),
        "the timer boundary must be classified from the derived route:\n{text}"
    );
    // The authored form emits this too; losing it would be a behavioural
    // difference between the two spellings even with identical priorities.
    assert!(
        text.contains("override-inversion"),
        "the override-inversion warning must survive the move to a scope \
         path:\n{text}"
    );
}

/// Rate propagation and the service-response binding — the last two fields
/// that were written but never read.
///
/// `contract_rates` declares exactly ONE rate fact (a 100 Hz timer) and lets
/// every other rate be derived. Three separate claims are asserted, because
/// each fails in a different way:
///
/// - `derivable-rate` — the author's number agrees with the graph, so it is a
///   second copy and can go.
/// - `rate-mismatch` — the author's number DISAGREES. `/rates/merged` is
///   published by a fan-in path with no `sync:`, so its two 100 Hz inputs ADD
///   to 200 Hz; a contract claiming 100 is wrong and nothing else in the
///   toolchain would say so.
/// - `response-blocking` — a node promising a 5ms service response while
///   declaring a 20ms callback, which the default mutually-exclusive callback
///   group rules out.
#[test]
fn rate_propagation_and_service_response_have_rules_that_fire() {
    let out = check_fixture("contract_rates");
    for rule in [
        "derivable-rate",
        "rate-mismatch",
        "response-blocking",
        // Phase 70 W3: the endpoint side of the same derivation.
        "derivable-min-rate",
        "min-rate-mismatch",
        "derived-rate-hierarchy",
    ] {
        assert!(
            out.contains(rule),
            "expected `{rule}` to fire on contract_rates:\n{out}"
        );
    }
    // The derived value, not just the fact that something was reported: 200 Hz
    // is the sum of two 100 Hz inputs, and it is the number that distinguishes
    // a fan-in summed from one min'd or max'd.
    assert!(
        out.contains("derives 200.0000 Hz"),
        "a fan-in without `sync:` must SUM its input rates:\n{out}"
    );
}

/// Phase 71: the fault-reaction arithmetic on a chain that fits, and on
/// one that fails three different ways.
#[test]
fn fault_reaction_budget_fits_and_names_its_terms() {
    let out = check_fixture("contract_fault");
    assert!(out.contains("fault-reaction-budget"), "{out}");
    assert!(
        out.contains("detection 100.00ms") && out.contains("settle 200.00ms"),
        "the verdict must name the derived terms:\n{out}"
    );
    assert!(
        out.contains("fits the fault-tolerant time interval"),
        "{out}"
    );
    // Who watches the watcher: the reaction's sink is deliberately unguarded.
    assert!(out.contains("reaction-unguarded"), "{out}");
    // Phase 90 G5: the brake's label is gone (a hazard reaches it, so it is
    // derived); `contract_criticality_written` keeps one to refuse.
    assert!(!out.contains("criticality"), "{out}");
    assert!(!out.contains("error[fault-reaction-budget]"), "{out}");
}

#[test]
fn fault_reaction_rules_fire_on_a_broken_chain() {
    let out = check_fixture("contract_fault_late");
    for needle in [
        "error[fault-reaction-budget]",
        "607.00ms exceeds",
        "error[reaction-unreachable]",
        "not a scope path",
        "error[hazard-unguarded]",
        "nothing would ever notice",
    ] {
        assert!(
            out.contains(needle),
            "expected `{needle}` on contract_fault_late:\n{out}"
        );
    }
}

/// The same chain with the reaction handed over a SERVICE instead of a topic:
/// the detector CALLS the brake controller rather than publishing to it, which
/// is the shape of every MRM chain (Autoware's `mrm_handler` calls
/// `/system/mrm/emergency_stop/operate` on `mrm_emergency_stop_operator`).
///
/// `contract_fault_service` carries `contract_fault`'s numbers exactly, so the
/// derived verdict must be `contract_fault`'s: route 7ms, settle 200ms, total
/// 307ms. While the reaction walk enumerated `topics:` alone this fixture
/// reported `reaction-unreachable`, fell back to the scope path's declared
/// 100ms, lost the settle with it, and derived NO criticality at all for
/// brake_controller -- the hazard's ASIL_D landed on the node that notices the
/// fault and never on the node that stops the vehicle.
#[test]
fn a_reaction_crossing_a_service_is_walked_like_one_crossing_a_topic() {
    let out = check_fixture("contract_fault_service");
    for needle in [
        // Both hops of the route: the client's and the server's.
        "/safety/obstacle_detector/declare_lost",
        "/safety/brake_controller/emergency_stop = 7.00ms",
        "settle 200.00ms",
        "= 307.00ms",
    ] {
        assert!(
            out.contains(needle),
            "expected `{needle}` on contract_fault_service:\n{out}"
        );
    }
    // The severity reaches the node that ACTS: ASIL_D derives `high`
    // (phase 90 G5 removed the label that used to show it as an info).
    assert_eq!(
        derived_criticality("contract_fault_service", "/safety/brake_controller").as_deref(),
        Some("high")
    );
    // The route exists, so nothing falls back to the declared budget.
    assert!(!out.contains("reaction-unreachable"), "{out}");
}

/// The model's derived criticality of `node` (`contracts.node_criticality`),
/// from `play_launch resolve` on the fixture.
fn derived_criticality(dir: &str, node: &str) -> Option<String> {
    let launch = fixtures::repo_root()
        .join("tests/fixtures")
        .join(dir)
        .join("launch/bringup.launch.xml");
    let out = play_launch_cmd()
        .arg("resolve")
        .arg(&launch)
        .args(["-o", "-"])
        .output()
        .expect("resolve runs");
    let model = String::from_utf8_lossy(&out.stdout).to_string();
    let section = model.split("  node_criticality:\n").nth(1)?;
    section
        .lines()
        .take_while(|l| l.starts_with("    "))
        .find_map(|l| {
            l.trim()
                .strip_prefix(&format!("{node}: "))
                .map(str::to_string)
        })
}

/// Phase 82 W4: a server that LATCHES the request and acts on its own timer
/// is a sampling hop, not the end of the reaction.
///
/// `contract_fault_latch` is `contract_fault_service` with the brake
/// controller's request handler triggering nothing and its 30 Hz timer
/// publishing the command, which is the shape of Autoware's
/// `mrm_emergency_stop_operator`. The route must reach the timer path, name the
/// clock it waited for, and cost exactly one period more than the service
/// fixture's: 7ms + 33.33ms = 40.33ms, total 340.33ms. Before this the walk
/// stopped at the server, reported `reaction-unreachable`, and derived nothing
/// for the node that stops the vehicle.
#[test]
fn a_server_that_latches_the_request_is_a_sampling_hop() {
    let out = check_fixture("contract_fault_latch");
    for needle in [
        "/safety/obstacle_detector/declare_lost",
        "/safety/brake_controller/apply_brake (+33.33ms sampling) = 40.33ms",
        "settle 200.00ms",
        "= 340.33ms",
    ] {
        assert!(
            out.contains(needle),
            "expected `{needle}` on contract_fault_latch:\n{out}"
        );
    }
    assert_eq!(
        derived_criticality("contract_fault_latch", "/safety/brake_controller").as_deref(),
        Some("high")
    );
    assert!(!out.contains("reaction-unreachable"), "{out}");
    // The trigger-driven service fixture is untouched: no clock in its route.
    assert!(!check_fixture("contract_fault_service").contains("sampling)"));
}

/// Phase 82: a detector counts only for the fault classes it can see, and a
/// hazard is timed by the SLOWEST class it claims. `contract_fault_kinds` is
/// `contract_fault`'s chain with three guard topics into one detector; every
/// hazard shares the 7ms route and 200ms settle, so only detection varies.
///
/// - `scan_classes`, `on: [omission, late]` over a 100ms lease and a 40ms
///   deadline: 100ms, the max. Before phase 82 the list did not parse, and
///   the omitted-key spelling read the 40ms minimum.
/// - `scan_unnamed`, the same guard with `on:` omitted: the same 100ms, so
///   saying less about the fault buys no slack.
/// - `imu_age_under_qos`, `on: omission` with only `max_age` under the
///   default `mechanism: qos`: unguarded, and the message neither
///   recommends `max_age` nor lists `min_rate_hz`; it says the age limit is
///   declared but counts only for `on: late` (issue #0046, run 4).
/// - `odom_age_evaluated`, the same age limit under `mechanism:
///   application`: the node evaluates it, so it notices silence in 30ms.
#[test]
fn a_hazard_is_timed_by_the_slowest_fault_class_it_claims() {
    let out = check_fixture("contract_fault_kinds");
    for needle in [
        "hazard 'scan_classes': detection 100.00ms",
        "the slowest of [omission 100.00ms, late 40.00ms]",
        "hazard 'scan_unnamed': detection 100.00ms",
        "= 307.00ms fits the fault-tolerant time interval 500.00ms",
        "hazard 'odom_age_evaluated': detection 30.00ms",
        "= 237.00ms fits",
        "error[hazard-unguarded]: hazard 'imu_age_under_qos' guards '/safety/imu' `on: omission`",
        "`max_age: 30ms` is declared on '/safety/obstacle_detector/imu' but counts only for \
         `on: late`",
    ] {
        assert!(
            out.contains(needle),
            "expected `{needle}` on contract_fault_kinds:\n{out}"
        );
    }
    let unguarded = out
        .lines()
        .find(|l| l.contains("error[hazard-unguarded]"))
        .expect("the imu hazard is unguarded");
    assert!(!unguarded.contains("min_rate_hz"), "{unguarded}");
    // What it recommends is the lease alone; `max_age` appears only in the
    // declared-in-vain clause after it.
    let counts = unguarded
        .split("What counts: ")
        .nth(1)
        .and_then(|r| r.split(". ").next())
        .expect("the message names what counts");
    assert!(counts.contains("lease_duration"), "{unguarded}");
    assert!(!counts.contains("max_age"), "{unguarded}");
    assert_eq!(
        out.matches("error[hazard-unguarded]").count(),
        1,
        "only the imu hazard is unguarded:\n{out}"
    );
    assert!(
        !out.contains("manifest-parse"),
        "`on: [omission, late]` parses:\n{out}"
    );
}

/// Phase 74: the contract's liveliness lease reaches the model as a
/// `qos_overrides` parameter on the subscribing node, and the lease itself
/// is lowered for the live observer.
#[test]
fn contract_qos_becomes_qos_override_parameters_on_the_model() {
    let base = fixtures::repo_root().join("tests/fixtures/contract_fault/launch");
    let out_path = std::env::temp_dir().join("play_launch_p74_model.yaml");
    let status = play_launch_cmd()
        .arg("resolve")
        .arg(base.join("bringup.launch.xml"))
        .arg("-o")
        .arg(&out_path)
        .status()
        .expect("resolve runs");
    assert!(status.success(), "resolve failed");
    let model = std::fs::read_to_string(&out_path).expect("model written");
    assert!(
        model.contains(
            "qos_overrides./safety/scan.subscription.liveliness_lease_duration: 100000000"
        ),
        "the lease must reach the node as an rclcpp qos_overrides parameter, in ns:\n{model}"
    );
    assert!(model.contains("lease_duration_ms: 100.0"), "{model}");
    let _ = std::fs::remove_file(&out_path);
}

/// Phase 75: functions and modes on a contract that fits — the ladder
/// resolves, the terminal rung is what the budget measures, and no mode
/// rule fires on a correct declaration.
#[test]
fn modes_resolve_and_a_correct_ladder_is_quiet() {
    let out = check_fixture("contract_modes");
    assert!(out.contains("fault-reaction-budget"), "{out}");
    assert!(
        out.contains("fits the fault-tolerant time interval"),
        "{out}"
    );
    for rule in [
        "ladder-unterminated",
        "ladder-rung-budget",
        "mode-requires-unguarded",
        "override-target-missing",
    ] {
        assert!(
            !out.contains(rule),
            "`{rule}` must not fire on a correct contract:\n{out}"
        );
    }
    // W3's second half: `degraded` RELAXES the budget, so running the checks
    // in that mode introduces nothing. A mode-tagged finding here would mean
    // the per-mode pass reports what the default run already said.
    assert!(
        !out.contains("[mode:"),
        "a relaxing override must introduce nothing:\n{out}"
    );
}

/// The model a fixture resolves to, from the standalone `ros-launch-resolve
/// resolve` (no ROS needed), or `None` when that CLI has not been built.
fn resolve_fixture_model(dir: &str) -> Option<ros_launch_manifest_model::SystemModel> {
    let bin = ros_launch_resolve_bin()?;
    let launch = fixtures::repo_root()
        .join("tests/fixtures")
        .join(dir)
        .join("launch/bringup.launch.xml");
    let out_dir = tempfile::TempDir::new().expect("temp dir");
    let model_path = out_dir.path().join("system_model.yaml");
    let out = resolve_cmd(&bin)
        .arg("resolve")
        .arg(&launch)
        .arg("-o")
        .arg(&model_path)
        .output()
        .expect("ros-launch-resolve resolve runs");
    assert!(
        out.status.success(),
        "resolve failed:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let yaml = std::fs::read_to_string(&model_path).expect("model written");
    Some(ros_launch_manifest_model::SystemModel::from_yaml_str(&yaml).expect("model parses"))
}

/// rlm v0.1.63: a mode override's target reaches the model RESOLVED to the
/// model's own keys -- one per kind a target can name -- and each key names an
/// entry that exists in its table. A consumer that lowers a mode onto the
/// model (nano-ros phase-462 W4) reads these; it has no scope namespaces to
/// resolve `target` itself. (A target naming no declared requirement gets no
/// key: `override_site` answers both this and `override-target-missing`, so
/// that case is the rule's, tested on `contract_modes_bad` below.)
#[test]
fn a_mode_overrides_target_is_resolved_to_the_models_keys() {
    use ros_launch_manifest_model::OverrideTable as T;
    let Some(model) = resolve_fixture_model("contract_mode_override_keys") else {
        return;
    };
    let c = &model.contracts;
    let (_, degraded) = c
        .modes
        .iter()
        .find(|(k, _)| k.ends_with("/degraded"))
        .expect("the degraded mode");
    let got: Vec<(String, T, String, String)> = degraded
        .overrides
        .iter()
        .map(|o| {
            let r = o
                .resolved
                .as_ref()
                .unwrap_or_else(|| panic!("`{}` unresolved", o.target));
            (o.target.clone(), r.table, r.key.clone(), r.field.clone())
        })
        .collect();
    let want = [
        (
            "paths.safety.stop.max_latency",
            T::ScopePaths,
            "bringup.launch.xml/safety.stop",
            "max_latency",
        ),
        (
            "nodes.obstacle_detector.sub.scan.min_rate_hz",
            T::SubEndpoints,
            "/safety/obstacle_detector/scan",
            "min_rate_hz",
        ),
        (
            "nodes.obstacle_detector.paths.detect.max_latency",
            T::NodePaths,
            "/safety/obstacle_detector/detect",
            "max_latency",
        ),
        (
            "nodes.brake_controller.sub.obstacles.max_age",
            T::SubEndpoints,
            "/safety/brake_controller/obstacles",
            "max_age",
        ),
        (
            "topics./safety/scan.rate_hz",
            T::Topics,
            "/safety/scan",
            "rate_hz",
        ),
    ];
    for (target, table, key, field) in want {
        assert!(
            got.iter()
                .any(|g| g.0 == target && g.1 == table && g.2 == key && g.3 == field),
            "`{target}` -> {table:?} `{key}` `{field}` missing; got {got:?}"
        );
        let exists = match table {
            T::ScopePaths => c.scope_paths.contains_key(key),
            T::NodePaths => c.node_paths.contains_key(key),
            T::PubEndpoints => c.pub_endpoints.contains_key(key),
            T::SubEndpoints => c.sub_endpoints.contains_key(key),
            T::Topics => c.topics.contains_key(key),
        };
        assert!(exists, "the resolved key `{key}` names no {table:?} entry");
    }
}

/// And the four mode rules on a contract built to break each one.
#[test]
fn mode_rules_fire_on_a_broken_ladder() {
    let out = check_fixture("contract_modes_bad");
    for needle in [
        "error[ladder-unterminated]",
        "there is no floor",
        "error[ladder-rung-budget]",
        "rung 'degraded'",
        "error[mode-requires-unguarded]",
        "error[override-target-missing]",
        // W3's second half: `restricted` TIGHTENS the budget below the
        // derived route. The default contract's 100ms is fine, so only the
        // per-mode run sees this — which is the whole reason to run them.
        "warning[mode:scope-budget]",
        "in mode 'restricted'",
    ] {
        assert!(
            out.contains(needle),
            "expected `{needle}` on contract_modes_bad:\n{out}"
        );
    }
    // An override the rule REJECTS must not reach the arithmetic: `stopped`
    // pins a max_jitter the path never declares, and gets exactly one
    // answer — the rejection — not a mode-tagged jitter finding too.
    assert!(
        !out.contains("in mode 'stopped'"),
        "a rejected override must not be applied:\n{out}"
    );
}

/// Phase 83: `check` on a takeover fixture, with the fixture's own `.msg`
/// stand-ins on `AMENT_PREFIX_PATH` so `when-field-unknown` resolves fields
/// without an Autoware install. Returns (exit code, stdout + stderr).
fn check_takeover(dir: &str, extra: &[&str]) -> (i32, String) {
    let root = fixtures::repo_root().join("tests/fixtures");
    let launch = root.join(dir).join("launch/bringup.launch.xml");
    let ament = root.join("contract_takeover/ament");
    let prefix = match std::env::var("AMENT_PREFIX_PATH") {
        Ok(p) if !p.is_empty() => format!("{}:{p}", ament.display()),
        _ => ament.display().to_string(),
    };
    let out = play_launch_cmd()
        .arg("check")
        .arg(&launch)
        .args(extra)
        .env("AMENT_PREFIX_PATH", prefix)
        .output()
        .expect("play_launch check runs");
    (
        out.status.code().unwrap_or(-1),
        format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ),
    )
}

/// Phase 83: the four takeover keys on a ladder that fits. The value fault
/// waits the 10 s window out and lands on the comfortable stop; the silence
/// fault skips both rungs that need the HPC (F1) and is not charged the
/// window; the walk crosses the comfortable operator's service callback,
/// which publishes (F3); every settle is derived and printed.
#[test]
fn takeover_ladder_fits_and_explains_itself() {
    let (code, out) = check_takeover("contract_takeover", &["--explain"]);
    assert_eq!(code, 0, "{out}");
    for needle in [
        // The emergency profile at the island's assumed 3.0 m/s: the island
        // contract's own hand arithmetic, now the checker's.
        // Phase 91 E8: the bundled vehicle.braking's, named in `use:`.
        "hazard 'hpc_loss', rung 'estop': settle = 2033.33ms derived by vehicle.braking 1.0.0",
        "if v0 > v_r: 3 m/s > 2.0833 m/s, true: take `then`",
        // F1: charged, the comfortable rung would have failed a 3 s interval.
        "hazard 'hpc_loss': detection 500.00ms",
        "= 2676.67ms fits the fault-tolerant time interval 3000.00ms",
        "requires hpc_alive, which this fault removes",
        // The window, charged to the rung below it; F3's route is 110 + 300.
        "+ 'takeover' reaction 110.00ms + window 10000.00ms",
        "odd_exit  comfortable  rung     120.00  10110.00  410.00    9996.67 derived  20636.67  30000.00   9363.33",
        // Phase 84: the window is a least time, charged to its deadline; the
        // handler's late notice of it is the first hop of the route below.
        "window >=10000.00",
        "odd_exit/takeover: lasts at least 10000.00ms once on, and ends within 10110.00ms: \
         /handler reads the deadline on its 100.00ms timer ('on_timer'), charged inside \
         /handler/call_mrm 110.00ms, the first hop of the route below",
        "up to its deadline.",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    for rule in [
        "when-requires-reported",
        "when-field-unknown",
        "ladder-window-floor",
        "window-param",
        "window-unbound",
        "mode-exit-target",
        "mode-exit-unwired",
        "ladder-rung-budget",
        "settle-entry-missing",
        "settle-param-unresolved",
        "settle-conflict",
        "reaction-unbudgeted",
        // As `[rule]`, not the bare name: the table's footer names
        // `window-expiry` when it explains where the notice is charged, and
        // the stated-budget table (phase 10 B8) names the rules that check
        // each budget.
        "window-expiry",
    ] {
        assert!(
            !out.contains(&format!("[{rule}]")),
            "`{rule}` must not fire on a correct contract:\n{out}"
        );
    }
}

/// Phase 85 T1 (I1): the island's shape -- an availability publisher outside
/// the tree, behind a link the detecting subscriber states as
/// `max_transport: 57ms`, and a `call_mrm` that holds only the tick and the
/// work (149 ms). The route reads link + path on the guard edge, and the hop
/// after the window's deadline is charged no link: the request ends within
/// 10149 ms. Without the key the routes are 57 ms shorter, so `--explain`
/// changes with it (it was byte-identical before I1).
#[test]
fn takeover_link_is_charged_on_the_guard_edge() {
    let (code, out) = check_takeover("contract_takeover_link", &["--explain"]);
    assert_eq!(code, 0, "{out}");
    for needle in [
        "hpc_loss  estop        floor    500.00      0.00  239.33    4165.33 derived   4904.67  10000.00   5095.33",
        "odd_exit  takeover     window   100.00      0.00  206.00  window >=10000.00         -  30000.00         -",
        "odd_exit  comfortable  rung     100.00  10206.00  149.00    9996.67 derived  20451.67  30000.00   9548.33",
        "odd_exit  estop        floor    100.00  10206.00  182.33    4165.33 derived  14653.67  30000.00  15346.33",
        "hpc_loss/estop: route = link 57.00ms (max_transport into '/handler/availability', \
         sub-level) + path 182.33ms",
        "odd_exit/takeover: route = link 57.00ms (max_transport into '/handler/availability', \
         sub-level) + path 149.00ms",
        "odd_exit/comfortable: route = path 149.00ms; the guard edge's link (57.00ms into \
         '/handler/availability') is not charged after a window's deadline",
        "odd_exit/takeover: lasts at least 10000.00ms once on, and ends within 10149.00ms",
        "reaction route link 57.00ms into /handler/availability + /handler/call_mrm",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    // The charged declaration is no longer `declared-not-charged`; the
    // driver's 143 ms is on no guard edge, so it still is.
    assert_eq!(
        out.matches("info[declared-not-charged]").count(),
        1,
        "{out}"
    );
    assert!(
        out.contains("`nodes.handler.sub.control_mode.max_transport: 143ms`"),
        "{out}"
    );
}

/// Phase 85 T2 (D3): an on-demand publisher (rlm v0.1.49 `on_demand: true`)
/// reaches the model as a statement -- `on_demand: true` and no rate -- so
/// the image derives no rate monitor for it; a subscriber that requires a
/// rate of it is a `rate-hierarchy` error.
#[test]
fn an_on_demand_publisher_promises_no_rate() {
    let base = fixtures::repo_root().join("tests/fixtures");
    assert!(
        check_fixture("contract_on_demand").contains("0 with errors"),
        "{}",
        check_fixture("contract_on_demand")
    );
    let model_path = std::env::temp_dir().join("play_launch_p85_d3_model.yaml");
    let status = play_launch_cmd()
        .arg("resolve")
        .arg(base.join("contract_on_demand/launch/bringup.launch.xml"))
        .arg("-o")
        .arg(&model_path)
        .status()
        .expect("resolve runs");
    assert!(status.success(), "resolve failed");
    let model = std::fs::read_to_string(&model_path).expect("model written");
    assert!(
        model.contains("/operator/limit:\n      on_demand: true"),
        "{model}"
    );
    let _ = std::fs::remove_file(&model_path);

    let out = check_fixture("contract_on_demand_bad");
    assert!(
        out.contains(
            "error[rate-hierarchy]: subscriber 'planner/limit' requires min_rate_hz (10), but \
             every publisher of the topic is on demand (`on_demand: true` on 'operator/limit') \
             and promises no rate"
        ),
        "{out}"
    );
}

/// Phase 85 T2 (D1): a timer's release jitter (rlm v0.1.49). What waits for
/// a tick is charged period + jitter: the emergency operator's sampling hop
/// (33.33 + 5), and the window owner's notice of the deadline (100 + 18,
/// held by call_mrm's 149). The model carries the jitter beside the trigger.
#[test]
fn a_timers_release_jitter_is_charged_where_a_tick_is_waited_for() {
    let (code, out) = check_takeover("contract_takeover_jitter", &["--explain"]);
    assert_eq!(code, 0, "{out}");
    for needle in [
        "hpc_loss  estop        floor    500.00      0.00  244.33    4165.33 derived   4909.67  10000.00   5090.33",
        "odd_exit  estop        floor    100.00  10206.00  187.33    4165.33 derived  14658.67  30000.00  15341.33",
        "/handler/call_mrm -> /estop_op/on_timer (+33.33ms sampling + 5.00ms jitter) = 187.33ms",
        "odd_exit/takeover: lasts at least 10000.00ms once on, and ends within 10149.00ms: \
         /handler reads the deadline on its 100.00ms timer ('on_timer') released up to 18.00ms \
         late, charged inside /handler/call_mrm 149.00ms",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    assert!(!out.contains("[window-expiry]"), "{out}");

    let launch = fixtures::repo_root()
        .join("tests/fixtures/contract_takeover_jitter/launch/bringup.launch.xml");
    let model_path = std::env::temp_dir().join("play_launch_p85_d1_model.yaml");
    let status = play_launch_cmd()
        .arg("resolve")
        .arg(&launch)
        .arg("-o")
        .arg(&model_path)
        .env(
            "AMENT_PREFIX_PATH",
            fixtures::repo_root().join("tests/fixtures/contract_takeover/ament"),
        )
        .status()
        .expect("resolve runs");
    assert!(status.success(), "resolve failed");
    let model = std::fs::read_to_string(&model_path).expect("model written");
    assert!(model.contains("timer_jitter_ms: 18.0"), "{model}");
    assert!(model.contains("timer_jitter_ms: 5.0"), "{model}");
    let _ = std::fs::remove_file(&model_path);
}

/// Phase 85 D1, the two places 0.15.1 left out: a timer's release jitter
/// is charged wherever its period is. The gate that REPORTS odd_exit is on a
/// 10 Hz timer released up to 6 ms late, so the fault's detection is
/// 100 + 6 + 20 (not 120), and the scope path through the gate carries the
/// jitter in its sampling cost (332 = 226 event-segment + 106 sampling,
/// against 330 declared; without the jitter it was 326 and passed).
#[test]
fn a_timers_release_jitter_is_charged_in_detection_and_the_critical_path() {
    let (code, out) = check_takeover("contract_takeover_gate_jitter", &["--explain"]);
    assert_eq!(code, 0, "{out}");
    for needle in [
        "detection 126.00ms (/gate/availability reports every 100.00ms + 6.00ms jitter + 20.00ms)",
        "odd_exit  takeover     window   126.00",
        "warning[scope-budget]: scope path 'sys.notice' (scope 0) max_latency_ms (330) is less \
         than critical path: /gate",
        "/handler = 332.00ms (226.00ms event-segment + 106.00ms sampling_cost)",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
}

/// Phase 85 T2 (D1): the same ladder with a call_mrm of 110 ms. Against a
/// bare 100 ms period it held the late notice; against the stated 18 ms of
/// release jitter it leaves 8 ms charged nowhere, once per rung below.
#[test]
fn window_expiry_reads_the_owners_release_jitter() {
    let (code, out) = check_takeover("contract_takeover_jitter_short", &[]);
    assert_eq!(code, 1, "{out}");
    for rung in ["comfortable", "estop"] {
        let needle = format!(
            "rung '{rung}' is charged from the deadline of 'takeover' (10000.00ms), and /handler \
             reads the deadline on its 100.00ms timer ('on_timer') released up to 18.00ms late, \
             but the route charges /handler/call_mrm only 110.00ms, so up to 8.00ms of the late \
             notice is charged nowhere"
        );
        assert!(out.contains(&needle), "expected `{needle}`:\n{out}");
    }
    assert_eq!(out.matches("error[window-expiry]").count(), 2, "{out}");
}

/// Phase 84: the window is charged up to its deadline, so the route below
/// must hold the owner's late notice of it. A 5 Hz tick under a 110 ms hop
/// leaves 90 ms charged nowhere, once per rung below; the arithmetic itself
/// is unchanged (no slack is added to make it pass).
#[test]
fn window_expiry_names_a_notice_the_route_does_not_hold() {
    let (code, out) = check_takeover("contract_takeover_expiry_tick", &["--explain"]);
    assert_eq!(code, 1, "{out}");
    for rung in ["comfortable", "estop"] {
        let needle = format!(
            "hazard 'odd_exit': rung '{rung}' is charged from the deadline of 'takeover' \
             (10000.00ms), and /handler reads the deadline on its 200.00ms timer ('on_timer'), \
             but the route charges /handler/call_mrm only 110.00ms, so up to 90.00ms of the late \
             notice is charged nowhere"
        );
        assert!(out.contains(&needle), "expected `{needle}`:\n{out}");
    }
    assert_eq!(out.matches("error[window-expiry]").count(), 2, "{out}");
    // Same numbers as contract_takeover: the rule does not move the budget.
    assert!(
        out.contains(
            "odd_exit  comfortable  rung     120.00  10110.00  410.00    9996.67 derived  20636.67  30000.00   9363.33"
        ),
        "{out}"
    );
}

/// Phase 84: a window bound to a node the route below does not start at --
/// nothing charges that node noticing the deadline -- and a node with no
/// timer publishing the rung's output, so when it notices is undeclared.
#[test]
fn window_expiry_names_an_owner_off_the_route() {
    let (code, out) = check_takeover("contract_takeover_expiry_owner", &[]);
    assert_eq!(code, 1, "{out}");
    for needle in [
        "error[window-expiry]",
        "rung 'comfortable' is charged from the deadline of 'takeover' (10000.00ms), and its \
         route starts at /handler/call_mrm, not at /planner, which enforces the window -- nothing \
         charges /planner noticing the deadline",
        "warning[window-expiry]",
        "mode 'takeover' waits 10000.00ms, enforced by /planner, but no timer path of /planner \
         publishes /tor_state, so when /planner notices the deadline is not declared",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    // The declaration warning is about the window, once.
    assert_eq!(out.matches("warning[window-expiry]").count(), 1, "{out}");
}

/// Phase 83: every structural takeover rule, once, with the file and line.
#[test]
fn takeover_structure_rules_fire_on_their_fixture() {
    let (code, out) = check_takeover("contract_takeover_bad", &[]);
    assert_eq!(code, 1, "{out}");
    for needle in [
        "error[when-requires-reported]: bringup.contract.yaml:",
        "names a value fault (stop == true) with `on: [omission]`",
        "error[when-field-unknown]: bringup.contract.yaml:",
        "has no such field (it has stamp, stop, autonomous, comfortable_stop)",
        "with 'MANUALL', which demo_msgs/msg/ControlMode does not declare as a constant (it declares NO_COMMAND=0, AUTONOMOUS=1, MANUAL=4)",
        "which is `builtin_interfaces/Time` -- not a scalar",
        "warning[when-field-unknown]",
        "other_msgs/msg/Opaque, but its definition is not on the ament index",
        "error[ladder-window-floor]",
        "mode 'stopping' falls to 'parked' last, and 'parked' has a 30000ms window",
        "error[mode-exit-target]",
        "exits to 'estop', which is a rung of the `fallback:` ladder of 'engaged'",
        "error[mode-exit-unwired]",
        "mode 'hold' exits when 'driver_took_over' holds (/control_mode), but no node that implements the rung (/holder) takes it on a path trigger",
        "warning[window-unbound]",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
}

/// Phase 83: the arithmetic, one term broken per rule. The window-param
/// mismatch, the cumulative rung budget, a derived settle that contradicts a
/// measured one, one that cannot be derived, one with no entry speed, and
/// phase 7's miss: the same floor from 4.23 m/s instead of 3.0.
#[test]
fn takeover_budget_rules_fire_on_their_fixture() {
    let (code, out) = check_takeover("contract_takeover_budget", &["--explain"]);
    assert_eq!(code, 1, "{out}");
    // Every error here is cross-scope, and the one manifest is not clean
    // for that: it used to read "1 clean, 0 with errors (4 errors ...)".
    assert!(
        out.contains("1 manifest(s) checked: 0 clean, 1 with errors (4 errors"),
        "{out}"
    );
    for needle in [
        "error[window-param]",
        "bound to `handler.takeover_timeout`, which resolves to 12.0 s = 12000.00ms, not the 10000.00ms the window declares",
        "error[ladder-rung-budget]",
        "fallback rung 'comfortable' cannot make the fault-tolerant time interval",
        "detection 120.00ms + 'takeover' reaction 110.00ms + window 10000.00ms + reaction 410.00ms + settle 9996.67ms = 20636.67ms against 20000.00ms",
        "warning[settle-conflict]",
        "the literal settle 5000.00ms and vehicle.braking 1.0.0's 9996.67ms differ by 50.0%",
        "warning[settle-entry-missing]",
        "hazard 'odd_exit_unbounded' reaches rung 'comfortable'",
        "falling back to the literal settle 5000.00ms",
        "error[settle-param-unresolved]",
        "'target_decel' has no resolved launch value for /estop_op",
        "warning[reaction-unbudgeted]",
        // 4.23 m/s: 2525.33 ms, and the 3 s interval is missed.
        "hazard 'hpc_observed', rung 'estop_ok': settle = 2525.33ms derived by vehicle.braking",
        "hazard 'hpc_observed': detection 500.00ms",
        "= 3168.67ms exceeds the fault-tolerant time interval 3000.00ms",
        // Phase 84: this handler declares no tick, so when it reads the
        // deadline is unstated.
        "warning[window-expiry]",
        "no timer path of /handler publishes /tor_state, so when /handler notices the deadline \
         is not declared",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    // One finding per declaration, however many hazards walk through it.
    assert_eq!(
        out.matches("error[settle-param-unresolved]").count(),
        1,
        "{out}"
    );
}

/// Phase 83: an exit on a rung with no window is a parse error, and the
/// file is refused whole, at the line of the key.
#[test]
fn an_exit_without_a_window_refuses_the_contract() {
    let (code, out) = check_takeover("contract_takeover_exit", &[]);
    // Phase 85 I4: a refusal is its own exit status, not a rule failure.
    assert_eq!(code, 3, "{out}");
    assert!(out.contains("error[manifest-parse]"), "{out}");
    assert!(out.contains("bringup.contract.yaml:"), "{out}");
    assert!(
        out.contains("at 'modes.takeover.exit': an exit is only for a windowed rung"),
        "{out}"
    );
}

/// An action nobody serves is an Error, and the cross-scope loop is the only
/// thing that can say so.
///
/// `load_manifests` drops every per-manifest `dangling-entity` diagnostic (and
/// `service-wiring`) because the cross-scope index is authoritative for them —
/// but the replacement in `run_cross_scope_checks` iterated `index.topics` and
/// `index.services` and never `index.actions`, while rlm's own rule DOES check
/// actions. So for actions alone the suppression removed a real check and
/// nothing re-emitted it: this fixture reported "1 clean, 0 with errors" and
/// exited 0.
///
/// The fixture carries three actions so the rule is falsifiable in both
/// directions — a loop that reported every action with a client would pass a
/// test that only asserted the first row.
#[test]
fn an_action_with_no_server_anywhere_is_reported() {
    let out = check_fixture("contract_action_unserved");

    assert!(
        out.contains(
            "error[dangling-entity]: action '/nav/navigate_to_pose' has 0 servers across the \
             manifest tree (declared in 1 scope(s)) -- goals can't be processed"
        ),
        "{out}"
    );

    // Served in this tree, and served by another image: neither is a finding.
    for clean in ["/nav/dock", "/nav/remote_recovery"] {
        assert!(
            !out.contains(&format!("action '{clean}'")),
            "action '{clean}' should not be reported: {out}"
        );
    }

    // Exactly one, so the loop is not reporting every action with a client.
    assert_eq!(
        out.matches("error[dangling-entity]: action").count(),
        1,
        "{out}"
    );
    assert!(out.contains("0 clean, 1 with errors"), "{out}");
}

/// Phase 10 B1: a timer and a staleness bound are the numbers the image
/// runs. Each binding that disagrees with its launch value fails its rule
/// once, with the file and line; the two that agree say nothing.
#[test]
fn param_bindings_fail_on_disagreement() {
    let (code, out) = check_takeover("contract_param_bind", &[]);
    assert_eq!(code, 1, "{out}");
    for needle in [
        "error[timer-param]: bringup.contract.yaml:36: /handler/slow ticks at 1 Hz, bound to \
         `period_param: slow_period`, and 'slow_period' resolves to 0.5 s = 2.0000 Hz, not the \
         1 Hz the timer declares",
        "error[max-age-param]: bringup.contract.yaml:27: /handler/in declares `max_age: \
         400.00ms`, bound to `param: timeout`, and 'timeout' resolves to 0.5 s = 500.00ms, not \
         the 400.00ms `max_age` declares",
        "(2 errors, 0 warnings)",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    assert_eq!(out.matches("error[timer-param]").count(), 1, "{out}");
    assert_eq!(out.matches("error[max-age-param]").count(), 1, "{out}");
}

/// Phase 10 B2: a written response beside a braking profile is added to the
/// derived settle; `settle-derived` prints both parts and the totals and
/// `--explain` carry the sum.
#[test]
fn a_settles_written_response_is_added_to_the_profile() {
    let (code, out) = check_takeover("contract_takeover_response", &["--explain"]);
    assert_eq!(code, 0, "{out}");
    for needle in [
        // Phase 91 E8: vehicle.braking reads the response itself.
        "hazard 'hpc_loss', rung 'estop': settle = 2333.33ms derived by vehicle.braking 1.0.0",
        "response = 300 ms   (300ms)",
        "settle = 10496.67ms derived by vehicle.braking 1.0.0",
        "= 2976.67ms fits the fault-tolerant time interval 3000.00ms with 23.33ms of slack",
        "odd_exit  comfortable  rung     120.00  10110.00  410.00   10496.67 derived  21136.67",
        "hpc_loss/estop: settle = profile 2033.33ms + response 300.00ms",
        "odd_exit/comfortable: settle = profile 9996.67ms + response 500.00ms",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
}

/// Phase 10 B7: `--explain` says where each stated budget comes from, per
/// row of the fault-reaction table and per budget, and marks every total
/// that contains a measured term as measured, not a bound. Measured terms
/// that reproduce their budgets leave `budget-rule` quiet.
#[test]
fn explain_prints_each_budgets_source() {
    let (code, out) = check_takeover("contract_budget_sources", &["--explain"]);
    assert_eq!(code, 0, "{out}");
    assert!(!out.contains("budget-rule"), "{out}");
    for needle in [
        "SLACK  SOURCES",
        "hpc_loss  estop        floor    500.00      0.00  143.33    2033.33 derived   2676.67   \
         3000.00    323.33  measured",
        "9363.33  analysed+measured+requirement",
        // Phase 93: the row's LEVEL, and the unsupervised terms named.
        "hpc_loss/estop: TOTAL 2676.67 is unsupervised: it contains \
         nodes.handler.paths.call_mrm.max_latency",
        "odd_exit/comfortable: TOTAL 20636.67 is unsupervised: it contains \
         nodes.handler.paths.call_mrm.max_latency, nodes.planner.paths.follow.max_latency",
        "warning[measured-unsupervised]",
        "-- Stated budgets (--explain, ms) --",
        "measured: tick 100 + 5 x 1.2 (n=600) = 106.00 -> 106, work 3.2 x 1.2 (n=12) = 3.84 -> 4",
        "measured: 250 x 1.2 (n=5) = 300.00 -> 300",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    let row = |key: &str| {
        out.lines()
            .find(|l| l.starts_with(key))
            .unwrap_or_else(|| panic!("no row {key}:\n{out}"))
            .to_string()
    };
    assert!(row("hazards.hpc_loss.ftti").contains("requirement"));
    assert!(row("nodes.gate.paths.report.max_latency").contains("analysed"));
    assert!(row("nodes.handler.paths.on_timer.max_latency").contains("unstated"));
}

/// Phase 10 B7: a measured budget whose stated observations do not give it
/// under the rule fails `budget-rule`, with the arithmetic.
#[test]
fn a_measured_budget_its_observations_do_not_give_is_budget_rule() {
    let (code, out) = check_takeover("contract_budget_rule", &[]);
    assert_eq!(code, 1, "{out}");
    for needle in [
        "error[budget-rule]: bringup.contract.yaml:125: `nodes.handler.paths.call_mrm.max_latency: \
         110ms` is measured, but its observations give 112ms (tick 100 + 5 x 1.2 (n=600) = 106.00 \
         -> 106, work 5 x 1.2 (n=12) = 6.00 -> 6;",
        "error[budget-rule]: bringup.contract.yaml:158: `nodes.planner.paths.follow.max_latency: \
         300ms` is measured, but its observations give 288ms (240 x 1.2 (n=5) = 288.00 -> 288;",
        // Phase 93: the two measured budgets nothing supervises warn.
        "(2 errors, 4 warnings)",
        "warning[measured-unsupervised]",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
}

/// Phase 10 B8: `--explain` lists, per stated budget, what checks it, and
/// `declared-not-charged` says the same about a `max_transport` no hazard
/// charges.
#[test]
fn explain_lists_what_checks_each_stated_budget() {
    let (code, out) = check_takeover("contract_takeover_link", &["--explain"]);
    assert_eq!(code, 0, "{out}");
    let row = |key: &str| {
        out.lines()
            .find(|l| l.starts_with(key))
            .unwrap_or_else(|| panic!("no row {key}:\n{out}"))
            .to_string()
    };
    assert!(
        row("hazards.hpc_loss.ftti").ends_with("fault-reaction sum (the interval)"),
        "{out}"
    );
    assert!(
        row("modes.takeover.window").contains("fault-reaction sum (odd_exit); window-param"),
        "{out}"
    );
    for line in out
        .lines()
        .filter(|l| l.contains("info[declared-not-charged]"))
    {
        let key = line
            .split('`')
            .nth(1)
            .and_then(|k| k.split(':').next())
            .expect("the message names the key");
        let by = row(key);
        if by.ends_with("nothing") {
            assert!(line.contains("nothing else checks it either"), "{line}");
        } else {
            assert!(line.contains("; it is checked by "), "{line}\n{by}");
        }
    }
}

/// Phase 90 G5: a written criticality on a node a hazard reaches is an
/// error, agreeing (`derivable-criticality`) or not (`criticality-mismatch`),
/// and the fix is to delete it. A label on a node no hazard reaches stands.
#[test]
fn a_written_criticality_on_a_reached_node_is_an_error() {
    let out = check_fixture("contract_criticality_written");
    for needle in [
        "error[derivable-criticality]: node /safety/brake_controller declares criticality 'high'",
        "ASIL_D reacts hazard 'drive_blind'",
        "Fix: delete `criticality: high`",
        "error[criticality-mismatch]: node /safety/obstacle_detector declares criticality 'low'",
        "detects hazard 'drive_blind' (ASIL_D), which derives high",
        "Fix: delete `criticality: low`",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    assert!(
        !out.contains("trip_logger"),
        "an unreached node's label stands:\n{out}"
    );
    assert!(out.contains("(2 errors,"), "{out}");
}

/// Phase 90 G4: the pre-0.1.59 spellings (`reaction: <mode>` on a hazard,
/// `exit: { on }`) still check exactly as the current ones do, with one
/// `deprecated-spelling` warning each naming the new key and `migrate`.
#[test]
fn deprecated_spellings_warn_and_check_like_the_current_ones() {
    let (code, old) = check_takeover("contract_spelling_deprecated", &["--explain"]);
    let (code_new, new) = check_takeover("contract_takeover_jitter", &["--explain"]);
    assert_eq!((code, code_new), (0, 0), "{old}");
    let deprecated: Vec<&str> = old
        .lines()
        .filter(|l| l.contains("warning[deprecated-spelling]"))
        .collect();
    assert_eq!(deprecated.len(), 3, "{old}");
    for needle in [
        "hazard 'hpc_loss': `reaction: engaged` names a mode. A hazard enters a mode with `enters: engaged`",
        "hazard 'odd_exit': `reaction: engaged` names a mode",
        "mode 'takeover': `exit: { on: driver_took_over }` is the old spelling",
        "`exit: { holds: driver_took_over, to: manual }`",
        "`play_launch migrate <contract> --write` rewrites it",
    ] {
        assert!(old.contains(needle), "expected `{needle}`:\n{old}");
    }
    assert!(!new.contains("deprecated-spelling"), "{new}");
    // Everything else is the migrated twin's: the same findings and sums.
    let findings = |out: &str| {
        let mut v: Vec<String> = out
            .lines()
            .filter(|l| l.contains("info[") || l.contains("error[") || l.contains("warning["))
            .filter(|l| !l.contains("deprecated-spelling"))
            .map(without_lines)
            .collect();
        v.sort();
        v
    };
    assert_eq!(findings(&old), findings(&new));
}

/// A finding with its `bringup.contract.yaml:<line>` site reduced to the
/// file: the deprecated fixture carries a longer header comment.
fn without_lines(l: &str) -> String {
    let key = "bringup.contract.yaml:";
    let mut out = String::new();
    let mut rest = l;
    while let Some(i) = rest.find(key) {
        out.push_str(&rest[..i + key.len()]);
        rest = rest[i + key.len()..].trim_start_matches(|c: char| c.is_ascii_digit());
    }
    out.push_str(rest);
    out
}

/// Phase 90 G4: `enters:` names a mode of the hazard's scope, never a path.
#[test]
fn enters_naming_no_mode_is_an_error() {
    let out = check_fixture("contract_enters_unknown");
    assert!(
        out.contains(
            "error[enters-unknown-mode]: bringup.contract.yaml:40: hazard 'drive_blind' enters \
             'safety.stop', which is not a mode in its scope. A hazard that reacts by one scope \
             path writes `reaction: safety.stop`"
        ),
        "{out}"
    );
    // Not read as a path either: no reaction is walked or charged.
    assert!(!out.contains("fault-reaction-budget"), "{out}");
}

/// Phase 90 G4: a hazard enters a mode or reacts by a path, not both.
#[test]
fn enters_and_reaction_together_refuse_the_contract() {
    let launch = fixtures::repo_root()
        .join("tests/fixtures/contract_enters_and_reaction/launch/bringup.launch.xml");
    let out = play_launch_cmd()
        .arg("check")
        .arg(&launch)
        .output()
        .expect("play_launch check runs");
    let text =
        String::from_utf8_lossy(&out.stdout).to_string() + &String::from_utf8_lossy(&out.stderr);
    assert_eq!(out.status.code(), Some(3), "{text}");
    assert!(
        text.contains("error[manifest-parse]") && text.contains("not both"),
        "{text}"
    );
}

/// `play_launch check [extra] <launch>`: (exit code, stdout + stderr).
fn check_launch(launch: &std::path::Path, extra: &[&str]) -> (i32, String) {
    let out = play_launch_cmd()
        .arg("check")
        .args(extra)
        .arg(launch)
        .output()
        .expect("play_launch check runs");
    (
        out.status.code().unwrap_or(-1),
        String::from_utf8_lossy(&out.stdout).to_string() + &String::from_utf8_lossy(&out.stderr),
    )
}

/// A fixture's `launch/` directory copied into a temporary directory, so a
/// test can migrate it in place.
fn fixture_copy(dir: &str) -> (tempfile::TempDir, std::path::PathBuf) {
    let tmp = tempfile::tempdir().unwrap();
    let src = fixtures::repo_root()
        .join("tests/fixtures")
        .join(dir)
        .join("launch");
    let dst = tmp.path().join("launch");
    std::fs::create_dir(&dst).unwrap();
    for e in std::fs::read_dir(&src).unwrap() {
        let e = e.unwrap();
        std::fs::copy(e.path(), dst.join(e.file_name())).unwrap();
    }
    (tmp, dst)
}

/// `play_launch migrate <args> <file>`: (exit code, stdout).
fn migrate(file: &std::path::Path, args: &[&str]) -> (i32, String) {
    let out = play_launch_cmd()
        .arg("migrate")
        .args(args)
        .arg(file)
        .output()
        .expect("play_launch migrate runs");
    (
        out.status.code().unwrap_or(-1),
        String::from_utf8_lossy(&out.stdout).to_string(),
    )
}

/// Phase 90 G3: an `external_topics:` entry and a path's legacy `input:`
/// still check as before, with one `deprecated-spelling` warning each naming
/// the kept spelling and `migrate`; after `migrate --write` the contract
/// checks to the same findings and `--explain` sums, without them.
#[test]
fn g3_deprecated_spellings_warn_and_migrate_to_the_same_verdict() {
    let (_tmp, dir) = fixture_copy("contract_spelling_deprecated_g3");
    let launch = dir.join("bringup.launch.xml");
    let contract = dir.join("bringup.contract.yaml");
    let (code, old) = check_launch(&launch, &["--explain"]);
    assert_eq!(code, 0, "{old}");
    let deprecated: Vec<&str> = old
        .lines()
        .filter(|l| l.contains("warning[deprecated-spelling]"))
        .collect();
    assert_eq!(deprecated.len(), 5, "{old}");
    for needle in [
        "topic '/image': the `external_topics:` section is deprecated; a topic's external side \
         is stated on the topic, `topics: { /image: { external: pub, type: \
         sensor_msgs/msg/Image } }`",
        "topic '/diagnostics': the `external_topics:` section is deprecated",
        "path 'detector/main': `input: [image]` is the legacy spelling of `trigger: { input: \
         [...] }`. Fix: write `trigger: { input: [image] }`",
        "path 'tracker/main': `input: [boxes]` is the legacy spelling of `trigger: { input: \
         [...] }`. Fix: delete it (the `trigger:` beside it is what is read)",
        "path 'image_to_tracks': `input: [/image]` is the legacy spelling",
        "`play_launch migrate <contract> --write` rewrites it",
    ] {
        assert!(old.contains(needle), "expected `{needle}`:\n{old}");
    }

    let (code, out) = migrate(&contract, &["--write"]);
    assert_eq!(code, 0, "{out}");
    assert!(out.contains("1 rewritten (6 edit(s))"), "{out}");
    let migrated = std::fs::read_to_string(&contract).unwrap();
    assert!(
        !migrated.contains("\nexternal_topics:")
            && !migrated
                .lines()
                .any(|l| l.trim_start().starts_with("input:")),
        "{migrated}"
    );
    assert_eq!(migrate(&contract, &["--check"]).0, 0);

    let (code_new, new) = check_launch(&launch, &["--explain"]);
    assert_eq!(code_new, 0, "{new}");
    assert!(!new.contains("deprecated-spelling"), "{new}");
    let findings = |out: &str| {
        let mut v: Vec<String> = out
            .lines()
            .filter(|l| {
                l.contains("info[")
                    || l.contains("error[")
                    || l.contains("warning[")
                    || l.contains("note[")
            })
            .filter(|l| !l.contains("deprecated-spelling") && !l.contains("explicit-trigger"))
            .map(without_lines)
            .collect();
        v.sort();
        v
    };
    assert_eq!(findings(&old), findings(&new));
}

/// Phase 90 G3 (the bound part): a topic-level `max_transport` warns once
/// per topic, naming the subscribers that read it (or that none does), and
/// `migrate` moves it onto the one subscriber of the same file; the topic
/// with no subscriber is left for a hand edit. The migrated contract checks
/// to the same findings and `--explain` sums.
#[test]
fn g3b_a_topic_transport_warns_and_migrates_to_the_same_verdict() {
    let (_tmp, dir) = fixture_copy("contract_spelling_deprecated_g3b");
    let launch = dir.join("bringup.launch.xml");
    let contract = dir.join("bringup.contract.yaml");
    let (code, old) = check_launch(&launch, &["--explain"]);
    assert_eq!(code, 0, "{old}");
    let deprecated = |out: &str| {
        out.lines()
            .filter(|l| l.contains("warning[deprecated-spelling]"))
            .map(str::to_string)
            .collect::<Vec<_>>()
    };
    assert_eq!(deprecated(&old).len(), 2, "{old}");
    for needle in [
        "topic '/safety/obstacles': `max_transport: 3ms` on a topic is deprecated; a transport \
         is the hop into one subscriber and is stated on it, `nodes.<n>.sub.<ep>.max_transport`. \
         It is read as the transport of 1 subscriber(s) that state none: \
         '/safety/brake_controller/obstacles'. Fix: write `max_transport: 3ms` on each of them \
         and delete this key",
        "topic '/safety/brake_cmd': `max_transport: 1ms` on a topic is deprecated",
        "No subscriber reads it (each states its own, or the topic has none), so it charges \
         nothing. Fix: delete it",
    ] {
        assert!(old.contains(needle), "expected `{needle}`:\n{old}");
    }

    let (code, out) = migrate(&contract, &["--write"]);
    assert_eq!(code, 0, "{out}");
    assert!(
        out.contains(
            "topic-max-transport-subscribers: `max_transport: 3ms` -> (nothing) \
             (topics./safety/obstacles.max_transport)"
        ) && out.contains("1 rewritten (2 edit(s))"),
        "{out}"
    );
    let migrated = std::fs::read_to_string(&contract).unwrap();
    assert!(
        migrated.contains("        max_age: 60ms\n")
            && migrated.contains("        max_transport: 3ms\n")
            && migrated.contains("    max_transport: 1ms\n"),
        "the subscriber states 3ms; /safety/brake_cmd is left:\n{migrated}"
    );

    let (code_new, new) = check_launch(&launch, &["--explain"]);
    assert_eq!(code_new, 0, "{new}");
    assert_eq!(
        deprecated(&new).len(),
        1,
        "only the hand edit is left:\n{new}"
    );
    let findings = |out: &str| {
        let mut v: Vec<String> = out
            .lines()
            .filter(|l| {
                l.contains("info[")
                    || l.contains("error[")
                    || l.contains("warning[")
                    || l.contains("note[")
            })
            .filter(|l| {
                !l.contains("deprecated-spelling")
                    && !l.contains("declared-not-charged")
                    // Phase 91 E8: `migrate` also names the ASIL scale.
                    && !l.contains("deprecated-implicit-extension")
            })
            .map(without_lines)
            .collect();
        v.sort();
        v
    };
    assert_eq!(findings(&old), findings(&new));
    // `declared-not-charged` names the key, which moved from the topic to
    // the subscriber: one each, before and after.
    let ndc = |out: &str| out.matches("info[declared-not-charged]").count();
    assert_eq!(ndc(&old), ndc(&new), "{old}\n---\n{new}");
    let table = |out: &str| {
        out.lines()
            .skip_while(|l| !l.starts_with("HAZARD"))
            .take_while(|l| !l.is_empty())
            .map(str::to_string)
            .collect::<Vec<_>>()
    };
    assert!(!table(&old).is_empty(), "{old}");
    assert_eq!(table(&old), table(&new));
}

/// Phase 90 G3: a node path's `drop:` refuses the contract (exit 3) naming
/// the topic form; `migrate` moves it to the path's one output topic and
/// the contract loads.
#[test]
fn g3b_a_node_path_drop_is_refused_and_migrates_to_its_topic() {
    let (_tmp, dir) = fixture_copy("contract_node_path_drop_removed");
    let launch = dir.join("bringup.launch.xml");
    let contract = dir.join("bringup.contract.yaml");
    let (code, out) = check_launch(&launch, &[]);
    assert_eq!(code, 3, "{out}");
    for needle in [
        "error[manifest-parse]",
        "at 'nodes.obstacle_detector.paths.detect.drop': Removed in v0.1.60",
        "`topics: { <t>: { drop: ... } }`; `play_launch migrate` moves it",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    let (code, out) = migrate(&contract, &["--write"]);
    assert_eq!(code, 0, "{out}");
    assert!(out.contains("node-path-drop-topic"), "{out}");
    let migrated = std::fs::read_to_string(&contract).unwrap();
    assert!(
        migrated.contains("    sub: [brake_controller/obstacles]\n    drop: 1 / 100\n"),
        "{migrated}"
    );
    let (code, out) = check_launch(&launch, &[]);
    assert_eq!(code, 0, "{out}");
    assert!(!out.contains("manifest-parse"), "{out}");
}

/// Phase 90 G3: `drop-tightness` on the merged graph. The scope path's
/// 1 / 25 is tighter than the 1 / 20 the child contract states for a topic
/// on its route; only the checker's cross-scope route sees both.
#[test]
fn g3b_drop_tightness_compares_a_scope_path_with_a_topic_of_another_contract() {
    let out = check_fixture("contract_drop_tightness");
    let hits: Vec<&str> = out
        .lines()
        .filter(|l| l.contains("warning[drop-tightness]"))
        .collect();
    assert_eq!(hits.len(), 1, "{out}");
    assert!(
        hits[0].contains(
            "scope path 'e2e' permits a loss of 1 / 25 (4.00%) end to end, but topic \
             '/perception/objects' on its route permits 1 / 20 (5.00%) on one hop"
        ),
        "{out}"
    );
}

/// Phase 90 G6: a written settle's `response` is added to it, as to a
/// braking profile's: `--explain` charges 200 + 150 and prints both parts.
#[test]
fn g6_a_written_settle_adds_its_response() {
    let launch = fixtures::repo_root()
        .join("tests/fixtures/contract_settle_written_response/launch/bringup.launch.xml");
    let (code, out) = check_launch(&launch, &["--explain"]);
    assert_eq!(code, 0, "{out}");
    for needle in [
        "350.00 literal",
        "drive_blind/safety.stop: settle = written 200.00ms + response 150.00ms",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
}

/// Phase 91 E1: hand-written derivation records check as if derived. The
/// hazard's stated `detection` (40ms) is charged before the detectors'
/// 100ms, the derived settle as any written one: 40 + 100 + 7 + 350 = 497ms.
/// `--explain` names what derived each value, its inputs and steps, and the
/// hazard's `reaction_total`.
#[test]
fn e1_a_derivation_record_checks_as_if_derived() {
    let launch = fixtures::repo_root()
        .join("tests/fixtures/contract_derived_record/launch/bringup.launch.xml");
    let (code, out) = check_launch(&launch, &["--explain"]);
    assert_eq!(code, 0, "{out}");
    for needle in [
        "drive_blind  safety.stop  floor  140.00     0.00   7.00  350.00 literal  497.00  500.00   3.00",
        "derived by sensor.warmup 0.2.0",
        "derived by vehicle.braking 1.0.0 (sha256 4f3c9a1d2e8b)",
        "hazards.drive_blind.detection = 40.00 ms   derived by sensor.warmup 0.2.0",
        "    frame_period = 20ms",
        "    v0 / a = 1.5 / 7.5 = 200 ms",
        "drive_blind  497.00  (floor 'safety.stop')",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
}

/// Phase 91 E1, E4: a requirement that is an extension check
/// (`deadline_by`) of an extension the contract does not `use:` cannot run,
/// and is an error rather than a pass.
#[test]
fn e1_a_deadline_by_without_its_extension_is_an_error() {
    let launch =
        fixtures::repo_root().join("tests/fixtures/contract_deadline_by/launch/bringup.launch.xml");
    let (code, out) = check_launch(&launch, &[]);
    assert_eq!(code, 1, "{out}");
    assert!(
        out.contains(
            "error[deadline-unresolved]: hazard 'drive_blind': `deadline_by: \
             machine.iso13855/safety_distance` names `machine.iso13855`, which this contract \
             does not `use:`"
        ),
        "{out}"
    );
}

/// Phase 91 E1: a settle derived twice (a braking profile and a duration an
/// extension derived) is an error; a hazard with no requirement is too.
#[test]
fn e1_two_derivations_and_no_requirement_are_errors() {
    for (dir, needle) in [
        (
            "contract_derive_conflict",
            "error[derive-conflict]: bringup.contract.yaml:74: the settle is derived twice: by \
             vehicle.braking 1.0.0 and as a written duration derived by vehicle.braking 1.0.0 \
             -- keep one",
        ),
        (
            "contract_hazard_no_requirement",
            "error[hazard-no-requirement]: hazard 'drive_blind' states no requirement",
        ),
    ] {
        let launch = fixtures::repo_root()
            .join("tests/fixtures")
            .join(dir)
            .join("launch/bringup.launch.xml");
        let (code, out) = check_launch(&launch, &[]);
        assert_eq!(code, 1, "{dir}:\n{out}");
        assert!(out.contains(needle), "{dir}: expected `{needle}`:\n{out}");
    }
}

fn ext_fixture(dir: &str, args: &[&str]) -> (i32, String) {
    let launch = fixtures::repo_root()
        .join("tests/fixtures")
        .join(dir)
        .join("launch/bringup.launch.xml");
    check_launch(&launch, args)
}

/// Phase 91 E4, extension-examples.md section 2: a requirement that is a
/// distance, fed by core's reaction_total (100 + 7 + 180 = 287ms).
#[test]
fn e4_machinery_checks_its_safety_distance() {
    let (code, out) = ext_fixture("contract_ext_machinery", &[]);
    assert_eq!(code, 0, "{out}");
    for needle in [
        "info[extension-check]: hazard 'drive_blind': machine.iso13855/safety_distance PASS \
         (machine.iso13855 0.3.0, the hazard's deadline_by)",
        "if s_2000 <= 500 mm: 732 mm <= 500 mm, false: take `else`",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    assert!(!out.contains("deadline-unchecked"), "{out}");
}

/// Phase 91 E4, section 3: a derived detection, charged before the
/// detectors', that cannot fit the ftti; the report has no verdict.
#[test]
fn e4_medical_derives_a_detection_that_does_not_fit() {
    let (code, out) = ext_fixture("contract_ext_medical", &["--explain"]);
    assert_eq!(code, 1, "{out}");
    for needle in [
        "info[detection-derived]: hazard 'drive_blind': detection = 50.48 min derived by \
         medical.occlusion 0.1.0",
        "error[fault-reaction-budget]",
        "stated detection 3029000.00ms + /safety/obstacle_detector/scan detects within 100.00ms",
        "info[extension-report]: hazard 'drive_blind': medical.occlusion/bolus_on_release = \
         0.84 mL (report, no verdict)",
        "hazard.alarm_threshold = occlusion_limit_mmhg = 300",
        // Phase 91 E5: the derivation's direction, proved.
        "worst case: proved decreasing in min_rate; increasing in compliance, alarm_threshold",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
}

/// Phase 91 E4, section 4: a derived settle and a check fed by the
/// reaction it is part of.
#[test]
fn e4_uav_derives_its_settle_and_checks_its_energy() {
    let (code, out) = ext_fixture("contract_ext_uav", &["--explain"]);
    assert_eq!(code, 0, "{out}");
    for needle in [
        "hazard 'drive_blind', rung 'safety.stop': settle = 50952.38ms derived by uav.descent \
         0.2.0",
        "drive_blind  safety.stop  floor  100.00     0.00   7.00  50952.38 derived  51059.38  \
         120000.00  68940.62",
        "info[extension-check]: hazard 'drive_blind': uav.descent/energy_reserve PASS",
        // Phase 91 E5: with the domain contract_ext_uav_refuted is told to
        // add, every claim is proved.
        "worst case: proved increasing in max_altitude, slow_altitude; decreasing in \
         descent_speed, land_speed",
        "worst case: proved increasing in reaction_total",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    assert!(!out.contains("reaction-unbudgeted"), "{out}");
    assert!(!out.contains("worst-case-"), "{out}");
}

/// Phase 91 E4, section 6: the hazard's ftti is derived, and is the
/// requirement the reaction is judged against.
#[test]
fn e4_process_derives_the_ftti_it_is_judged_against() {
    let (code, out) = ext_fixture("contract_ext_process", &["--explain"]);
    assert_eq!(code, 0, "{out}");
    for needle in [
        "info[ftti-derived]: hazard 'drive_blind': ftti = 4 s derived by process.pst 0.1.0",
        "= 2107.00ms fits the fault-tolerant time interval 4000.00ms with 1893.00ms of slack",
        "hazard.ftti = (hazard_pressure - trip_setpoint) / max_rise_rate = 4 s",
        "worst case: proved decreasing in max_rise_rate; increasing in hazard_pressure",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    assert!(!out.contains("hazard-no-requirement"), "{out}");
}

/// Phase 91 E5, extension-examples.md section 4: the UAV claims as first
/// written are refuted, each with a counterexample the evaluator computed
/// and the domain that would make it hold. The suggestion admits the
/// contract's values: `max_altitude <= slow_altitude` would also prove the
/// slow_altitude claim, and is not offered, since the contract has 120 m
/// and 10 m.
#[test]
fn e5_uav_claims_as_written_are_refuted_with_the_domain_that_holds() {
    let (code, out) = ext_fixture("contract_ext_uav_refuted", &[]);
    assert_eq!(code, 1, "{out}");
    for needle in [
        "error[worst-case-refuted]: uav.descent 0.2.0: settle.duration is not increasing in \
         slow_altitude",
        "counterexample (Z3): max_altitude = 120 m, descent_speed = 3 m/s, land_speed = 5 m/s",
        "slow_altitude 1 m -> 39.87 s;  2 m -> 39.73 s",
        "help: it holds on `domain: land_speed <= descent_speed, land_speed > 0 m/s`",
        "error[worst-case-refuted]: uav.descent 0.2.0: check energy_reserve is not increasing \
         in reaction_total",
        "help: it holds on `domain: hover_power > 0 W`",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    assert_eq!(out.matches("error[worst-case-refuted]").count(), 5, "{out}");
    assert!(!out.contains("max_altitude <= slow_altitude"), "{out}");
}

/// Phase 91 E5: a claim the prover cannot decide is a warning, and an error
/// under `--strict`; it is never reported as proved.
#[test]
fn e5_an_undecided_worst_case_is_a_warning_and_strict_makes_it_an_error() {
    let (code, out) = ext_fixture("contract_ext_unproved", &[]);
    assert_eq!(code, 0, "{out}");
    let needle = "warning[worst-case-unknown]: process.pst 0.1.0: hazard.ftti: worst case \
                  decreasing in max_rise_rate not proved: a power whose exponent is not a \
                  constant integer is beyond the encoding";
    assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    assert!(!out.contains("worst case: proved"), "{out}");
    let (code, out) = ext_fixture("contract_ext_unproved", &["--strict"]);
    assert_eq!(code, 1, "{out}");
    assert_eq!(out.matches("error[worst-case-unknown]").count(), 2, "{out}");
}

/// Phase 91 E4: a `use:` that finds nothing and a key no used extension
/// declares are errors, never a silent drop.
#[test]
fn e4_an_unknown_extension_or_key_is_an_error() {
    let (code, out) = ext_fixture("contract_ext_unknown", &[]);
    assert_eq!(code, 1, "{out}");
    for needle in [
        "error[extension-not-found]: `use: { process.sis: \"^1\" }`: no such extension was found",
        "error[extension-key-unknown]: `installed_distanse` is not a key of `hazard` in any \
         extension this contract uses (machine.iso13855 0.3.0)",
        "error[rlx-missing-input]: no value for `hazard.installed_distance`",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
}

/// Phase 91 E6: a contract under the iec61508 profile draws its severity
/// from the SIL scale, and each node the hazard reaches takes the
/// criticality the profile's map names (SIL_2: `medium`).
#[test]
fn e6_a_sil_contract_derives_criticality_through_its_profile() {
    let (code, out) = ext_fixture("contract_profile_sil", &["--explain"]);
    assert_eq!(code, 0, "{out}");
    assert!(
        out.contains("iec61508 1.0.0  profile  sha256 ")
            && out.contains("bundled with the checker"),
        "{out}"
    );
    assert!(!out.contains("severity-"), "{out}");
    for node in [
        "/safety/lidar_driver",
        "/safety/obstacle_detector",
        "/safety/brake_controller",
    ] {
        assert_eq!(
            derived_criticality("contract_profile_sil", node).as_deref(),
            Some("medium"),
            "{node}"
        );
    }
}

/// Phase 91 E6: a profile states the scale, so `severity_levels:` beside
/// it is an error, and a level the profile lacks names the profile.
#[test]
fn e6_a_profile_refuses_a_second_scale_and_a_foreign_level() {
    let (code, out) = ext_fixture("contract_profile_refused", &[]);
    assert_eq!(code, 1, "{out}");
    for needle in [
        "error[severity-scale-conflict]: `severity_levels:` beside the profile iec61508 1.0.0",
        "error[severity-unknown]: hazard 'drive_blind' declares severity 'ASIL_D', which is \
         not in the scale [none, SIL_1, SIL_2, SIL_3, SIL_4] (profile iec61508 1.0.0)",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
}

fn copy_tree(from: &std::path::Path, to: &std::path::Path) {
    std::fs::create_dir_all(to).unwrap();
    for e in std::fs::read_dir(from).unwrap().flatten() {
        let dest = to.join(e.file_name());
        if e.file_type().unwrap().is_dir() {
            copy_tree(&e.path(), &dest);
        } else {
            std::fs::copy(e.path(), &dest).unwrap();
        }
    }
}

/// `contract_ext_qualified` copied whole (its contract and its ament
/// prefix), so a test may edit either.
fn qualified_copy() -> (tempfile::TempDir, std::path::PathBuf) {
    let tmp = tempfile::tempdir().unwrap();
    copy_tree(
        &fixtures::repo_root().join("tests/fixtures/contract_ext_qualified"),
        tmp.path(),
    );
    let root = tmp.path().to_path_buf();
    (tmp, root)
}

/// `check` on a `contract_ext_qualified` tree at `root`, its packages on
/// `AMENT_PREFIX_PATH`.
fn check_qualified(root: &std::path::Path, extra: &[&str]) -> (i32, String) {
    let out = play_launch_cmd()
        .env("AMENT_PREFIX_PATH", root.join("ament"))
        .arg("check")
        .args(extra)
        .arg(root.join("launch/bringup.launch.xml"))
        .output()
        .expect("play_launch check runs");
    (
        out.status.code().unwrap_or(-1),
        String::from_utf8_lossy(&out.stdout).to_string() + &String::from_utf8_lossy(&out.stderr),
    )
}

/// Phase 91 E7: two packages registered under `rlm_extensions` declare
/// `hazard.device_response`. Qualified, each extension reads its own value;
/// bare, the key is refused with both spellings. The unregistered package
/// beside them (a higher machine.stopping) is never searched.
#[test]
fn e7_two_packages_declaring_one_key_force_the_qualified_spelling() {
    let root = fixtures::repo_root().join("tests/fixtures/contract_ext_qualified");
    let (code, out) = check_qualified(&root, &["--explain"]);
    assert_eq!(code, 0, "{out}");
    for needle in [
        "info[extension-check]: hazard 'drive_blind': machine.iso13855/safety_distance PASS \
         (machine.iso13855 0.3.0, the hazard's deadline_by)",
        "info[extension-check]: hazard 'drive_blind': machine.stopping/stop_time PASS \
         (machine.stopping 0.1.0)",
        "327 ms <= 400 ms",
        "machine.stopping 0.1.0  extension  sha256 ",
        "ament/share/rlm_ext_stopping/rlm/stopping.rlx",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    assert!(!out.contains("rlm_ext_unregistered"), "{out}");

    let (_tmp, copy) = qualified_copy();
    let contract = copy.join("launch/bringup.contract.yaml");
    let text = std::fs::read_to_string(&contract).unwrap().replace(
        "    machine.iso13855/device_response: 15ms\n    machine.stopping/device_response: 40ms\n",
        "    device_response: 15ms\n",
    );
    std::fs::write(&contract, text).unwrap();
    let (code, out) = check_qualified(&copy, &[]);
    assert_eq!(code, 1, "{out}");
    let needle = "error[extension-key-ambiguous]: `device_response` is declared in `hazard` by \
                  machine.iso13855 0.3.0 and machine.stopping 0.1.0 -- a bare key names one \
                  extension's; write the qualified spelling: \
                  `machine.iso13855/device_response:` or `machine.stopping/device_response:`";
    assert!(out.contains(needle), "expected `{needle}`:\n{out}");
}

/// Phase 91 E7: `--update-lock` writes `rlm.lock` beside the contract;
/// `--locked` accepts that resolution and refuses it once the extension
/// changes under the same version (a warning without `--locked`), and
/// refuses a contract with no lock at all.
#[test]
fn e7_a_changed_extension_under_locked_is_refused() {
    let (_tmp, root) = qualified_copy();
    let (code, out) = check_qualified(&root, &["--locked"]);
    assert_eq!(code, 1, "{out}");
    assert!(out.contains("error[lock-missing]"), "{out}");

    let (code, out) = check_qualified(&root, &["--update-lock"]);
    assert_eq!(code, 0, "{out}");
    let lock_path = root.join("launch/rlm.lock");
    assert!(
        out.contains(&format!("Wrote {}", lock_path.display())),
        "{out}"
    );
    let lock = std::fs::read_to_string(&lock_path).unwrap();
    for needle in [
        "machine.iso13855:",
        "machine.stopping:",
        "version: 0.1.0",
        "kind: extension",
    ] {
        assert!(
            lock.contains(needle),
            "expected `{needle}` in the lock:\n{lock}"
        );
    }
    let (code, out) = check_qualified(&root, &["--locked"]);
    assert_eq!(code, 0, "{out}");
    assert!(!out.contains("lock-"), "{out}");

    // The same version, another formula.
    let ext = root.join("ament/share/rlm_ext_stopping/rlm/stopping.rlx");
    let text = std::fs::read_to_string(&ext).unwrap().replace(
        "device_response + reaction_total <=",
        "device_response + 1.1 * reaction_total <=",
    );
    std::fs::write(&ext, text).unwrap();
    let (code, out) = check_qualified(&root, &["--locked"]);
    assert_eq!(code, 1, "{out}");
    let needle = "machine.stopping 0.1.0 changed since it was locked";
    assert!(
        out.contains("error[lock-mismatch]") && out.contains(needle),
        "expected `{needle}`:\n{out}"
    );
    let (code, out) = check_qualified(&root, &[]);
    assert_eq!(code, 0, "{out}");
    assert!(
        out.contains("warning[lock-drift]") && out.contains(needle),
        "{out}"
    );
}

/// Phase 90 G7: the two removed keys refuse the contract (exit 3) with a
/// message naming the replacement.
#[test]
fn g7_removed_keys_refuse_the_contract_with_a_fix_it() {
    for (dir, needles) in [
        (
            "contract_include_manifest_removed",
            &[
                "at 'includes.tracking.manifest': Removed in v0.1.59",
                "Write the child contract inline as `includes.<name>: { nodes: ..., ... }`",
            ],
        ),
        (
            "contract_guard_all_of_removed",
            &[
                "at 'hazards.pose_lost.guards[0].all_of': Removed in v0.1.59",
                "Declare the function and guard it by name: `guards: [<f>]`; `play_launch \
                 migrate` does it",
            ],
        ),
    ] {
        let launch = fixtures::repo_root()
            .join("tests/fixtures")
            .join(dir)
            .join("launch/bringup.launch.xml");
        let (code, out) = check_launch(&launch, &[]);
        assert_eq!(code, 3, "{dir}:\n{out}");
        assert!(out.contains("error[manifest-parse]"), "{dir}:\n{out}");
        for needle in needles {
            assert!(out.contains(needle), "{dir}: expected `{needle}`:\n{out}");
        }
    }
}

/// Phase 90 G7: `migrate` hoists an inline `all_of` guard into `functions:`,
/// and the contract it writes loads.
#[test]
fn g7_migrate_hoists_an_inline_all_of_guard_into_a_function() {
    let (_tmp, dir) = fixture_copy("contract_guard_all_of_removed");
    let contract = dir.join("bringup.contract.yaml");
    let (code, out) = migrate(&contract, &["--write"]);
    assert_eq!(code, 0, "{out}");
    assert!(
        out.contains(
            "guard-all-of-function: `all_of: [/loc/ndt, /loc/gnss]` -> `pose_lost_all_of`"
        ),
        "{out}"
    );
    let migrated = std::fs::read_to_string(&contract).unwrap();
    assert!(
        migrated.contains("      - pose_lost_all_of\n")
            && migrated
                .contains("functions:\n  pose_lost_all_of: { all_of: [/loc/ndt, /loc/gnss] }\n"),
        "{migrated}"
    );
    let (code, out) = check_launch(&dir.join("bringup.launch.xml"), &[]);
    assert_ne!(code, 3, "{out}");
    assert!(!out.contains("manifest-parse"), "{out}");
}

/// Phase 90 G8: `migrate` rewrites the deprecated fixture into its current
/// twin's spellings, shows the diff, and `--check` gates on it.
#[test]
fn migrate_rewrites_the_deprecated_spellings_in_place() {
    let dir = tempfile::tempdir().unwrap();
    let src = fixtures::repo_root()
        .join("tests/fixtures/contract_spelling_deprecated/launch/bringup.contract.yaml");
    let c = dir.path().join("bringup.contract.yaml");
    std::fs::copy(&src, &c).unwrap();
    let run = |args: &[&str]| {
        let out = play_launch_cmd()
            .arg("migrate")
            .args(args)
            .arg(&c)
            .output()
            .expect("play_launch migrate runs");
        (
            out.status.code().unwrap_or(-1),
            String::from_utf8_lossy(&out.stdout).to_string(),
        )
    };
    let (code, out) = run(&["--check"]);
    assert_eq!(code, 1, "{out}");
    for needle in [
        "hazard-reaction-enters: `reaction` -> `enters` (hazards.hpc_loss.reaction)",
        "hazard-reaction-enters: `reaction` -> `enters` (hazards.odd_exit.reaction)",
        "exit-on-holds: `on` -> `holds` (modes.takeover.exit.on)",
        "-    reaction: engaged",
        "+    enters: engaged",
        "1 contract(s) scanned, 1 to rewrite (3 edit(s))",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    let (code, _) = run(&["--write"]);
    assert_eq!(code, 0);
    let (code, out) = run(&["--check"]);
    assert_eq!(code, 0, "{out}");
    // The migrated text is the current twin's, below the header comment.
    let migrated = std::fs::read_to_string(&c).unwrap();
    let twin = std::fs::read_to_string(
        fixtures::repo_root()
            .join("tests/fixtures/contract_takeover_jitter/launch/bringup.contract.yaml"),
    )
    .unwrap();
    assert!(migrated.ends_with(&twin), "{migrated}");
}

/// Run `check` on a takeover fixture with the `ros-launch-resolve` CLI built
/// from this tree (phase 93), or `None` when it is not built. The phase 93
/// verdicts are new, so they are not asked of an installed `play_launch`.
fn check_resolve(dir: &str, extra: &[&str]) -> Option<(i32, String)> {
    let bin = ros_launch_resolve_bin()?;
    let root = fixtures::repo_root().join("tests/fixtures");
    let launch = root.join(dir).join("launch/bringup.launch.xml");
    let ament = root.join("contract_takeover/ament");
    let out = resolve_cmd(&bin)
        .arg("check")
        .arg(&launch)
        .args(extra)
        .env("AMENT_PREFIX_PATH", ament)
        .output()
        .expect("ros-launch-resolve check runs");
    Some((
        out.status.code().unwrap_or(-1),
        format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ),
    ))
}

/// Phase 93: two supervised budgets. Each overrun case is a row with its
/// arithmetic and fits; detection is tick + switch for a timer and tick +
/// the executor's longest `work` for an executor; a reaction on the same
/// executor covers slowness, not a hang; CHECKED BY names the lowered
/// monitor; LEVEL replaces "measured, not a bound".
#[test]
fn supervised_budgets_print_their_overrun_case() {
    let Some((code, out)) = check_resolve("contract_supervised", &["--explain"]) else {
        eprintln!("Skipping: ros-launch-resolve not built");
        return;
    };
    assert_eq!(code, 0, "{out}");
    for needle in [
        "info[supervised-fit]",
        "hazard 'hpc_loss', rung 'estop': if `nodes.handler.paths.call_mrm.max_latency` overruns, \
         before it 500.00ms + budget 110.00ms + detection 1.05ms (tick 1.00ms + switch 0.05ms \
         (timer)) + reaction /handler/brake_now 38.33ms + settle 2033.33ms = 2682.72ms fits the \
         fault-tolerant time interval 3000.00ms with 317.28ms of slack",
        "detection 301.00ms (tick 1.00ms + the longest callback on /planner's executor 300.00ms \
         (follow's `work`)",
        "covers slowness, not a hang",
        "hpc_loss  estop (nodes.handler.paths.call_mrm.max_latency)        overrun  500.00      \
         0.00  149.38    2033.33 derived   2682.72   3000.00    317.28",
        "SOURCES                        LEVEL",
        "runtime monitor (route-deadline-runtime: reaction /handler/brake_now, timer)",
        "runtime monitor (route-deadline-runtime: reaction /planner/stop_now, executor)",
        "warning[measured-unsupervised]",
        "`nodes.planner.paths.stop_now.max_latency: 5.00ms` is measured and nothing supervises it",
        // The executor supervision rests on callbacks with no analysed bound.
        "warning[supervised-executor-long-callback]: bringup.contract.yaml:156: \
         `nodes.planner.paths.follow.max_latency` is supervised with `mechanism: executor`",
        "its own callback, /planner/follow, has no analysed bound (supervised only) and \
         can itself overrun: the late output is published before the overdue check runs",
        "a callback with no analysed bound can run past it (stop_now (measured only))",
        "Use `mechanism: timer`",
    ] {
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
    assert!(!out.contains("measured, not a bound"), "{out}");
    let row = |key: &str| {
        out.lines()
            .find(|l| l.starts_with(key))
            .unwrap_or_else(|| panic!("no row {key}:\n{out}"))
            .to_string()
    };
    assert!(row("nodes.handler.paths.call_mrm.max_latency").contains("supervised"));
    assert!(row("nodes.gate.paths.report.max_latency").contains("bound"));
    assert!(row("nodes.planner.paths.stop_now.max_latency").contains("unsupervised"));
}

/// Check a copy of `contract_supervised` with `edit` applied to its contract,
/// with the `ros-launch-resolve` CLI of this tree.
fn check_supervised_edited(edit: impl Fn(String) -> String) -> Option<(i32, String)> {
    let bin = ros_launch_resolve_bin()?;
    let root = fixtures::repo_root().join("tests/fixtures");
    let tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_dir = tmp.path().join("launch");
    std::fs::create_dir_all(&launch_dir).unwrap();
    let src = root.join("contract_supervised/launch");
    std::fs::copy(
        src.join("bringup.launch.xml"),
        launch_dir.join("bringup.launch.xml"),
    )
    .unwrap();
    let contract = std::fs::read_to_string(src.join("bringup.contract.yaml")).unwrap();
    std::fs::write(launch_dir.join("bringup.contract.yaml"), edit(contract)).unwrap();
    let out = resolve_cmd(&bin)
        .arg("check")
        .arg(launch_dir.join("bringup.launch.xml"))
        .env("AMENT_PREFIX_PATH", root.join("contract_takeover/ament"))
        .output()
        .expect("ros-launch-resolve check runs");
    Some((
        out.status.code().unwrap_or(-1),
        format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ),
    ))
}

/// Phase 93: `supervised-executor-long-callback` follows the policy: an error
/// under `policy: { unsupervised_measured: error }`.
#[test]
fn an_executor_supervision_over_a_long_callback_follows_the_policy() {
    let Some((code, out)) = check_supervised_edited(|c| {
        c.replacen(
            "supervision: { tick: 1ms, switch: 0.05ms }",
            "supervision: { tick: 1ms, switch: 0.05ms }\npolicy: { unsupervised_measured: error }",
            1,
        )
    }) else {
        eprintln!("Skipping: ros-launch-resolve not built");
        return;
    };
    assert_eq!(code, 1, "{out}");
    assert!(
        out.contains(
            "error[supervised-executor-long-callback]: bringup.contract.yaml:157: \
             `nodes.planner.paths.follow.max_latency` is supervised with `mechanism: executor`"
        ),
        "{out}"
    );
    // The timer supervision is not flagged.
    assert!(!out.contains("`nodes.handler.paths.call_mrm.max_latency` is supervised with"));
}

/// Phase 93: an executor supervision whose every callback on the executor is
/// analysed is bounded by the tick, and not flagged; `mechanism: timer` is
/// never flagged.
#[test]
fn an_executor_supervision_over_bounded_callbacks_is_not_flagged() {
    let analysed = |c: String| {
        c.replacen(
            "source: { measured: { margin: 0.2, terms: { work: { max: 250ms, n: 5 } } } }",
            "source: analysed",
            1,
        )
        .replacen(
            "max_latency: { duration: 5ms, source: { measured: { margin: 0.2, terms: { work: \
             { max: 4ms, n: 20 } } } } }",
            "max_latency: { duration: 5ms, source: analysed }",
            1,
        )
    };
    let Some((_, out)) = check_supervised_edited(analysed) else {
        eprintln!("Skipping: ros-launch-resolve not built");
        return;
    };
    assert!(!out.contains("supervised-executor-long-callback"), "{out}");
    let Some((_, out)) = check_supervised_edited(|c| {
        c.replacen(
            "supervise: { reaction: stop_now, mechanism: executor }",
            "supervise: { reaction: stop_now, mechanism: timer }",
            1,
        )
    }) else {
        return;
    };
    assert!(!out.contains("supervised-executor-long-callback"), "{out}");
}

/// Phase 93: an overrun case over the interval is refused.
#[test]
fn a_supervised_overrun_over_the_interval_is_refused() {
    let Some((code, out)) = check_resolve("contract_supervised_overrun", &[]) else {
        eprintln!("Skipping: ros-launch-resolve not built");
        return;
    };
    assert_eq!(code, 1, "{out}");
    assert!(
        out.contains(
            "error[supervised-fit]: bringup.contract.yaml:103: hazard 'hpc_loss', rung 'estop': \
             if `nodes.handler.paths.call_mrm.max_latency` overruns, before it 500.00ms + budget \
             110.00ms + detection 1.05ms (tick 1.00ms + switch 0.05ms (timer)) + reaction scope \
             path 'sys.comfortable' 410.00ms + settle 4666.67ms = 5687.72ms exceeds the \
             fault-tolerant time interval 3000.00ms"
        ),
        "{out}"
    );
    assert_eq!(out.matches("error[").count(), 1, "{out}");
}

/// Phase 93: a supervised budget no runtime monitor reads is refused.
#[test]
fn a_supervised_budget_with_no_monitor_is_refused() {
    let Some((code, out)) = check_resolve("contract_supervised_unlowered", &[]) else {
        eprintln!("Skipping: ros-launch-resolve not built");
        return;
    };
    assert_eq!(code, 1, "{out}");
    assert!(
        out.contains(
            "error[supervised-unlowered]: bringup.contract.yaml:76: \
             `nodes.gate.paths.report.max_latency` is supervised, but no runtime monitor checks \
             it: /gate/report is not input-triggered"
        ),
        "{out}"
    );
    assert_eq!(out.matches("error[").count(), 1, "{out}");
}

/// Phase 93: a lowered supervision reaches the model on its node path, in the
/// shape a realizer's route monitor reads: the deadline, the reaction's path
/// id (a key of the model) and the mechanism.
#[test]
fn a_lowered_supervision_reaches_the_model() {
    let Some(bin) = ros_launch_resolve_bin() else {
        eprintln!("Skipping: ros-launch-resolve not built");
        return;
    };
    let root = fixtures::repo_root().join("tests/fixtures");
    let out_path = tempfile::NamedTempFile::new().unwrap().into_temp_path();
    let status = resolve_cmd(&bin)
        .arg("resolve")
        .arg(root.join("contract_supervised/launch/bringup.launch.xml"))
        .arg("-o")
        .arg(&out_path)
        .env("AMENT_PREFIX_PATH", root.join("contract_takeover/ament"))
        .status()
        .expect("resolve runs");
    assert!(status.success(), "resolve failed");
    let model: serde_yaml_ng::Value =
        serde_yaml_ng::from_str(&std::fs::read_to_string(&out_path).unwrap()).unwrap();
    let paths = &model["contracts"]["node_paths"];
    let sv = &paths["/handler/call_mrm"]["supervise"];
    assert_eq!(sv["route_deadline_ms"].as_f64(), Some(110.0), "{sv:?}");
    assert_eq!(sv["reaction"].as_str(), Some("/handler/brake_now"));
    assert_eq!(sv["mechanism"].as_str(), Some("timer"));
    assert!(
        paths["/handler/brake_now"].is_mapping(),
        "the reaction is a model key"
    );
    let sv = &paths["/planner/follow"]["supervise"];
    assert_eq!(sv["mechanism"].as_str(), Some("executor"));
    // An unsupervised path carries none.
    assert!(paths["/handler/driver_exit"]["supervise"].is_null());
}

/// `contract_supervised` with the comfortable settle supervised to
/// standstill (rlm v0.1.62): a 943 ms measured response, `supervise: {
/// enters: estop, mechanism: timer, observes: handler/kinematic_state }`,
/// and the observed input (10 Hz, `max_transport` 56 ms). `extra` edits the
/// result further.
fn supervised_settle(c: String, extra: impl Fn(String) -> String) -> String {
    let c = c
        .replacen(
            "          settle: { decel: min_acceleration, jerk: min_jerk }",
            "          settle:\n            decel: min_acceleration\n            jerk: min_jerk\n            \
             response: { duration: 943ms, source: { measured: { max: 785.43ms, n: 3, margin: 0.2 } } }\n            \
             supervise: { enters: estop, mechanism: timer, observes: handler/kinematic_state }",
            1,
        )
        .replacen(
            "      control_mode: {}\n",
            "      control_mode: {}\n      kinematic_state:\n        min_rate_hz: 10\n        \
             max_transport: { duration: 56ms, source: { measured: { max: 46.42ms, n: 12, margin: 0.2 } } }\n",
            1,
        )
        .replacen(
            "  /tor_state:\n",
            "  /kinematic_state:\n    type: demo_msgs/msg/Odometry\n    external: pub\n    \
             sub: [handler/kinematic_state]\n    rate_hz: 10\n  /tor_state:\n",
            1,
        );
    extra(c)
}

/// rlm v0.1.62: a supervised settle lowers to a standstill deadline. Its
/// overrun row charges the rung up to the settle's end (the whole settle,
/// profile + response), how late standstill is seen (the observed input's
/// period + its transport, read from the contract), the detection and the
/// escalation's route and settle; its terms are supervised, and CHECKED BY
/// names the deadline.
#[test]
fn a_supervised_settle_charges_the_whole_settle() {
    let Some((code, out)) = check_supervised_edited(|c| supervised_settle(c, |c| c)) else {
        eprintln!("Skipping: ros-launch-resolve not built");
        return;
    };
    assert_eq!(code, 0, "{out}");
    assert!(!out.contains("supervised-unlowered"), "{out}");
    let needle = "hazard 'odd_exit', rung 'comfortable': if \
                  `nodes.comfort_op.paths.operate.safe_state.settle` overruns, before it \
                  10640.00ms + settle 10939.67ms (profile 9996.67ms + response 943.00ms) + \
                  observation 156.00ms (/handler/kinematic_state: period 100.00ms \
                  (/kinematic_state rate_hz 10) + max_transport 56.00ms) + detection 1.05ms \
                  (tick 1.00ms + switch 0.05ms (timer)) + reaction scope path 'sys.estop' \
                  143.33ms + settle 4165.33ms = 26045.38ms fits the fault-tolerant time \
                  interval 30000.00ms with 3954.62ms of slack";
    assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    assert!(out.contains("info[supervised-fit]"), "{out}");
    // The response is a term of the supervised settle: no
    // `measured-unsupervised` for it.
    assert!(
        !out.contains(
            "`nodes.comfort_op.paths.operate.safe_state.settle.response: 943.00ms` is measured"
        ),
        "{out}"
    );
    let Some((_, out)) = check_resolve_edited_explain(|c| supervised_settle(c, |c| c)) else {
        return;
    };
    let row = |key: &str| {
        out.lines()
            .find(|l| l.starts_with(key))
            .unwrap_or_else(|| panic!("no row {key}:\n{out}"))
            .to_string()
    };
    let resp = row("nodes.comfort_op.paths.operate.safe_state.settle.response");
    assert!(resp.contains(" supervised "), "{resp}");
    assert!(
        resp.contains(
            "runtime monitor (settle-deadline: reaction scope path 'sys.estop', timer, \
             standstill seen on /handler/kinematic_state)"
        ),
        "{resp}"
    );
    assert!(
        out.lines().any(|l| l.starts_with(
            "odd_exit  comfortable (nodes.comfort_op.paths.operate.safe_state.settle)"
        ) && l.contains("overrun")),
        "{out}"
    );
}

/// `check --explain` on an edited copy of `contract_supervised`.
fn check_resolve_edited_explain(edit: impl Fn(String) -> String) -> Option<(i32, String)> {
    let bin = ros_launch_resolve_bin()?;
    let root = fixtures::repo_root().join("tests/fixtures");
    let tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_dir = tmp.path().join("launch");
    std::fs::create_dir_all(&launch_dir).unwrap();
    let src = root.join("contract_supervised/launch");
    std::fs::copy(
        src.join("bringup.launch.xml"),
        launch_dir.join("bringup.launch.xml"),
    )
    .unwrap();
    let contract = std::fs::read_to_string(src.join("bringup.contract.yaml")).unwrap();
    std::fs::write(launch_dir.join("bringup.contract.yaml"), edit(contract)).unwrap();
    let out = resolve_cmd(&bin)
        .arg("check")
        .arg(launch_dir.join("bringup.launch.xml"))
        .arg("--explain")
        .env("AMENT_PREFIX_PATH", root.join("contract_takeover/ament"))
        .output()
        .expect("ros-launch-resolve check runs");
    Some((
        out.status.code().unwrap_or(-1),
        format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ),
    ))
}

/// rlm v0.1.62: a supervised settle with no observation, or an observed
/// input with no transport, does not lower (`supervised-unlowered`); a
/// supervised `response` alone never does, and says to supervise the settle.
#[test]
fn a_supervised_settle_without_its_observation_is_refused() {
    for (edit, needle) in [
        (
            (|c: String| c.replacen(", observes: handler/kinematic_state", "", 1))
                as fn(String) -> String,
            "how late standstill is seen is unknown: name the input it is seen on",
        ),
        (
            |c: String| {
                c.replacen(
                    "        max_transport: { duration: 56ms, source: { measured: { max: 46.42ms, \
                     n: 12, margin: 0.2 } } }\n",
                    "",
                    1,
                )
            },
            "`observes: handler/kinematic_state`: its transport is unknown",
        ),
        (
            |c: String| {
                c.replacen(
                    "            supervise: { enters: estop, mechanism: timer, observes: \
                     handler/kinematic_state }",
                    "",
                    1,
                )
                .replacen(
                    "response: { duration: 943ms, source: { measured: { max: 785.43ms, n: 3, \
                     margin: 0.2 } } }",
                    "response: { duration: 943ms, source: { measured: { max: 785.43ms, n: 3, \
                     margin: 0.2 } }, supervise: { enters: estop, mechanism: timer } }",
                    1,
                )
            },
            "a settle's `response` is one term of the settle",
        ),
    ] {
        let Some((code, out)) = check_supervised_edited(|c| supervised_settle(c, edit)) else {
            eprintln!("Skipping: ros-launch-resolve not built");
            return;
        };
        assert_eq!(code, 1, "{out}");
        assert!(out.contains("error[supervised-unlowered]"), "{out}");
        assert!(out.contains(needle), "expected `{needle}`:\n{out}");
    }
}

/// rlm v0.1.62: a supervised settle whose overrun case exceeds the interval
/// while the rung itself fits is refused (`supervised-fit`); when the rung
/// itself does not fit, `ladder-rung-budget` is the error and the overrun
/// row does not repeat it.
#[test]
fn a_supervised_settle_over_the_interval_is_refused_once() {
    // The rung: 10640 + 10939.67 = 21579.67; the overrun case adds 156 +
    // 1.05 + the emergency route 143.33 and settle 4165.33 = 26045.38.
    let Some((code, out)) = check_supervised_edited(|c| {
        supervised_settle(c, |c| {
            c.replacen(
                "    ftti: { duration: 30s, source: requirement }",
                "    ftti: { duration: 22s, source: requirement }",
                1,
            )
        })
    }) else {
        eprintln!("Skipping: ros-launch-resolve not built");
        return;
    };
    assert_eq!(code, 1, "{out}");
    assert!(
        out.contains("error[supervised-fit]")
            && out.contains("`nodes.comfort_op.paths.operate.safe_state.settle` overruns"),
        "{out}"
    );
    assert!(!out.contains("error[ladder-rung-budget]"), "{out}");
    let Some((code, out)) = check_supervised_edited(|c| {
        supervised_settle(c, |c| {
            c.replacen(
                "    ftti: { duration: 30s, source: requirement }",
                "    ftti: { duration: 21s, source: requirement }",
                1,
            )
        })
    }) else {
        return;
    };
    assert_eq!(code, 1, "{out}");
    assert!(out.contains("error[ladder-rung-budget]"), "{out}");
    assert!(
        !out.contains("error[supervised-fit]: bringup.contract.yaml:")
            || !out.contains("safe_state.settle` overruns"),
        "{out}"
    );
    assert!(
        out.contains("and is refused on the rung's own row"),
        "{out}"
    );
}

/// rlm v0.1.62: a lowered settle supervision reaches the model on the safe
/// state, in the shape a realizer arms a standstill deadline from.
#[test]
fn a_lowered_settle_supervision_reaches_the_model() {
    let Some(bin) = ros_launch_resolve_bin() else {
        eprintln!("Skipping: ros-launch-resolve not built");
        return;
    };
    let root = fixtures::repo_root().join("tests/fixtures");
    let tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_dir = tmp.path().join("launch");
    std::fs::create_dir_all(&launch_dir).unwrap();
    let src = root.join("contract_supervised/launch");
    std::fs::copy(
        src.join("bringup.launch.xml"),
        launch_dir.join("bringup.launch.xml"),
    )
    .unwrap();
    let contract = std::fs::read_to_string(src.join("bringup.contract.yaml")).unwrap();
    std::fs::write(
        launch_dir.join("bringup.contract.yaml"),
        supervised_settle(contract, |c| c),
    )
    .unwrap();
    let out_path = tmp.path().join("model.yaml");
    let status = resolve_cmd(&bin)
        .arg("resolve")
        .arg(launch_dir.join("bringup.launch.xml"))
        .arg("-o")
        .arg(&out_path)
        .env("AMENT_PREFIX_PATH", root.join("contract_takeover/ament"))
        .status()
        .expect("resolve runs");
    assert!(status.success(), "resolve failed");
    let model: serde_yaml_ng::Value =
        serde_yaml_ng::from_str(&std::fs::read_to_string(&out_path).unwrap()).unwrap();
    let paths = &model["contracts"]["node_paths"];
    let sv = &paths["/comfort_op/operate"]["safe_state"]["supervise"];
    assert_eq!(sv["response_ms"].as_f64(), Some(943.0), "{sv:?}");
    assert_eq!(sv["observe_ms"].as_f64(), Some(156.0), "{sv:?}");
    assert_eq!(sv["observes"].as_str(), Some("/handler/kinematic_state"));
    assert_eq!(
        sv["reaction"].as_str(),
        Some("bringup.launch.xml/sys.estop"),
        "{sv:?}"
    );
    assert_eq!(sv["mechanism"].as_str(), Some("timer"));
    // The emergency stop's settle is not supervised.
    assert!(paths["/estop_op/on_timer"]["safe_state"]["supervise"].is_null());
}

/// Phase 91 E9's gate, play_launch half: each reference extension's fixture
/// (`contract_ext_<domain>`, the examples document's section on
/// contract_fault's chain) prints what its `expected_explain.txt` records:
/// the verdicts, the derivation traces, the proved worst cases, the budget
/// row and the resolution with its sha256. rlm's own test holds the files to
/// the document and proves them; this one holds the checker's output.
#[test]
fn e9_each_reference_fixture_explains_as_recorded() {
    for dir in [
        "contract_ext_vehicle",
        "contract_ext_machinery",
        "contract_ext_medical",
        "contract_ext_uav",
        "contract_ext_rail",
        "contract_ext_process",
    ] {
        let root = fixtures::repo_root().join("tests/fixtures").join(dir);
        let expected = std::fs::read_to_string(root.join("expected_explain.txt")).unwrap();
        let code: i32 = expected
            .lines()
            .find_map(|l| l.strip_prefix("# exit "))
            .and_then(|c| c.parse().ok())
            .expect("the file records the exit status");
        let (got, out) = ext_fixture(dir, &["--explain"]);
        assert_eq!(got, code, "{dir}:\n{out}");
        for needle in expected
            .lines()
            .filter(|l| !l.is_empty() && !l.starts_with('#'))
        {
            assert!(
                out.lines().any(|l| l.contains(needle)),
                "{dir}: expected `{needle}`:\n{out}"
            );
        }
        assert!(
            !out.contains("deprecated-implicit-extension"),
            "{dir}:\n{out}"
        );
    }
}

/// Phase 91 E9: the fixtures ship the reference extensions as rlm publishes
/// them (`types::extension::REFERENCE`), byte for byte, so the files the
/// checker's output is recorded for are the ones rlm proves.
#[test]
fn e9_the_fixtures_ship_the_reference_extensions_unchanged() {
    use ros_launch_manifest_types::extension::REFERENCE;
    for (dir, file) in [
        ("contract_ext_machinery", "iso13855.rlx"),
        ("contract_ext_medical", "occlusion.rlx"),
        ("contract_ext_uav", "descent.rlx"),
        ("contract_ext_rail", "ebd.rlx"),
        ("contract_ext_process", "pst.rlx"),
    ] {
        let ours = std::fs::read_to_string(
            fixtures::repo_root()
                .join("tests/fixtures")
                .join(dir)
                .join("launch/rlm")
                .join(file),
        )
        .unwrap();
        let theirs = REFERENCE.iter().find(|(f, _)| *f == file).unwrap().1;
        assert_eq!(ours, theirs, "{dir}/{file} differs from rlm's reference");
    }
}

/// `resolve` on a launch file at any path.
fn resolve_model_at(launch: &std::path::Path) -> Option<ros_launch_manifest_model::SystemModel> {
    let bin = ros_launch_resolve_bin()?;
    let out_dir = tempfile::TempDir::new().expect("temp dir");
    let model_path = out_dir.path().join("system_model.yaml");
    let out = resolve_cmd(&bin)
        .arg("resolve")
        .arg(launch)
        .arg("-o")
        .arg(&model_path)
        .output()
        .expect("ros-launch-resolve resolve runs");
    assert!(
        out.status.success(),
        "resolve failed:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let yaml = std::fs::read_to_string(&model_path).expect("model written");
    Some(ros_launch_manifest_model::SystemModel::from_yaml_str(&yaml).expect("model parses"))
}

/// Phase 91 E10's gate, model half: a settle `vehicle.braking` derives
/// reaches the model as the plain number a hand-written settle would, so
/// the image nano-ros builds from either has the same monitors and sizing.
/// `contract_ext_vehicle` is resolved as is, and again with the settle
/// written by hand -- the derived duration, the same response, no
/// extension keys; the two models' contracts are equal once the derived
/// source (which only the first has) is set aside.
#[test]
fn e10_a_derived_settle_reaches_the_model_as_a_written_one_would() {
    let root = fixtures::repo_root().join("tests/fixtures/contract_ext_vehicle");
    let Some(derived) = resolve_model_at(&root.join("launch/bringup.launch.xml")) else {
        eprintln!("SKIP: ros-launch-resolve not built");
        return;
    };
    let key = "/safety/brake_controller/emergency_stop";
    let ss = derived.contracts.node_paths[key]
        .safe_state
        .clone()
        .unwrap();
    let src = ss.settle_derived.clone().expect("the settle's source");
    assert_eq!(
        (src.by.as_str(), src.version.as_str()),
        ("vehicle.braking", "1.0.0")
    );
    assert_eq!(src.hazard.as_deref(), Some("drive_blind"));
    assert_eq!(src.inputs["hazard.entry_speed"], "30 km/h");
    let settle = ss.settle_ms.unwrap();
    assert!((settle - 4422.6667).abs() < 1e-3, "{settle}");

    let tmp = tempfile::tempdir().unwrap();
    copy_tree(&root, tmp.path());
    let contract = tmp.path().join("launch/bringup.contract.yaml");
    let text = std::fs::read_to_string(&contract).unwrap();
    let by_hand = text
        .replace(
            "use: { vehicle.braking: \"^1\", iso26262: \"^1\" }",
            "use: { iso26262: \"^1\" }",
        )
        .replace("    entry_speed: 30 km/h\n", "")
        .replace(
            "            decel: target_acceleration\n            jerk: target_jerk\n",
            &format!("            duration: {:.6}ms\n", settle - 256.0),
        );
    assert_ne!(by_hand, text);
    std::fs::write(&contract, by_hand).unwrap();
    let written = resolve_model_at(&tmp.path().join("launch/bringup.launch.xml")).unwrap();

    let normalise = |m: &ros_launch_manifest_model::SystemModel| {
        let mut c = m.contracts.clone();
        for p in c.node_paths.values_mut() {
            if let Some(ss) = p.safe_state.as_mut() {
                ss.settle_derived = None;
                ss.settle_ms = ss.settle_ms.map(|v| (v * 1000.0).round() / 1000.0);
            }
        }
        serde_yaml_ng::to_string(&c).unwrap()
    };
    assert_eq!(normalise(&derived), normalise(&written));
    assert!(
        written.contracts.node_paths[key]
            .safe_state
            .as_ref()
            .unwrap()
            .settle_derived
            .is_none()
    );
}
