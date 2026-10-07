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
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::OnceLock;

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
    for rule in ["jitter-feasibility", "jitter-range", "lifespan-age", "sync-budget"] {
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
    assert!(out.contains("fits the fault-tolerant time interval"), "{out}");
    // Who watches the watcher: the reaction's sink is deliberately unguarded.
    assert!(out.contains("reaction-unguarded"), "{out}");
    // Phase 72: the `high` label on the brake is what ASIL_D already derives.
    assert!(out.contains("derivable-criticality"), "{out}");
    assert!(out.contains("ASIL_D reacts hazard 'drive_blind'"), "{out}");
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
        // Phase 72: a `low` label on a node that reacts for an ASIL_D hazard.
        "warning[criticality-mismatch]",
    ] {
        assert!(out.contains(needle), "expected `{needle}` on contract_fault_late:\n{out}");
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
        // The severity reaches the node that ACTS.
        "node /safety/brake_controller declares criticality 'high'",
        "ASIL_D reacts hazard 'drive_blind'",
    ] {
        assert!(
            out.contains(needle),
            "expected `{needle}` on contract_fault_service:\n{out}"
        );
    }
    // The route exists, so nothing falls back to the declared budget.
    assert!(!out.contains("reaction-unreachable"), "{out}");
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
        "node /safety/brake_controller declares criticality 'high'",
        "ASIL_D reacts hazard 'drive_blind'",
    ] {
        assert!(
            out.contains(needle),
            "expected `{needle}` on contract_fault_latch:\n{out}"
        );
    }
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
        model.contains("qos_overrides./safety/scan.subscription.liveliness_lease_duration: 100000000"),
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
    assert!(out.contains("fits the fault-tolerant time interval"), "{out}");
    for rule in ["ladder-unterminated", "ladder-rung-budget", "mode-requires-unguarded", "override-target-missing"] {
        assert!(!out.contains(rule), "`{rule}` must not fire on a correct contract:\n{out}");
    }
    // W3's second half: `degraded` RELAXES the budget, so running the checks
    // in that mode introduces nothing. A mode-tagged finding here would mean
    // the per-mode pass reports what the default run already said.
    assert!(!out.contains("[mode:"), "a relaxing override must introduce nothing:\n{out}");
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
        assert!(out.contains(needle), "expected `{needle}` on contract_modes_bad:\n{out}");
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
        "v0 = 3 > v_r, so t = a/j + (v0 - v_r)/a = 2.5/1.5 + (3 - 2.0833)/2.5 = 1666.67 + 366.67 = 2033.33ms",
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
        // `[window-expiry]`, not the bare name: the table's footer names
        // the rule when it explains where the notice is charged.
        "[window-expiry]",
    ] {
        assert!(
            !out.contains(rule),
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
        "mode 'hold' exits on 'driver_took_over' (/control_mode), but no node that implements the rung (/holder) takes it on a path trigger",
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
        "the literal settle 5000.00ms and the profile's 9996.67ms differ by 50.0%",
        "warning[settle-entry-missing]",
        "hazard 'odd_exit_unbounded' reaches rung 'comfortable'",
        "falling back to the literal settle 5000.00ms",
        "error[settle-param-unresolved]",
        "'target_decel' is not declared under `nodes.estop_op.params`",
        "warning[reaction-unbudgeted]",
        // 4.23 m/s: 2525.33 ms, and the 3 s interval is missed.
        "(4.23 - 2.0833)/2.5 = 1666.67 + 858.67 = 2525.33ms",
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
