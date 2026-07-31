//! Integration tests for `contract eject` (Phase 41.4).
//!
//! The subcommand lived on `play_launch` until `adc33a7` (RFC-0060 W3)
//! moved it into the extracted `ros-launch-resolve` CLI; these tests drive
//! that binary and skip cleanly when it has not been built.
//!
//! Two flavors:
//! - the "neither file exists" / "no overlay root" error paths need no ROS
//!   runtime at all — a direct launch-file path with no sidecar files is
//!   enough;
//! - the "actually copies the provider contract" happy path needs a real
//!   package with a provider sidecar shipped next to its launch file, so it
//!   uses the `rt_workspace` fixture (`rt_demo`'s installed
//!   `bringup.contract.yaml`) — skips cleanly if the fixture isn't built
//!   (`cd tests/fixtures/rt_workspace && just build`), same convention as
//!   `rt_workspace.rs`.

use std::{path::PathBuf, process::Command};

use play_launch_tests::fixtures;

/// Find the play_launch binary, preferring the cargo debug build (mirrors
/// `manifest_check.rs`).
#[allow(dead_code)]
fn play_launch_bin() -> PathBuf {
    let cargo_bin = fixtures::repo_root().join("target/debug/play_launch");
    if cargo_bin.is_file() {
        return cargo_bin;
    }
    fixtures::play_launch_bin()
}

/// `fixtures::play_launch_cmd`'s equivalent for the relocated CLI: same
/// cleared-and-repopulated environment, different binary.
fn resolve_cli_cmd(env: &std::collections::HashMap<String, String>) -> Option<Command> {
    let bin = resolve_cli_bin()?;
    let mut cmd = Command::new(bin);
    cmd.env_clear();
    cmd.envs(env);
    Some(cmd)
}

/// Locate the `ros-launch-resolve` CLI, which owns `contract eject`.
///
/// The subcommand used to be `play_launch contract eject`; `adc33a7`
/// ("depend on ros-launch-resolve; drop the resolve pipeline", RFC-0060 W3)
/// moved it into the extracted CLI, and `play_launch` no longer has a
/// `contract` subcommand at all (`unrecognized subcommand 'contract'`).
/// The binary is not installed or on `PATH`, so this looks in the
/// submodule's own target dir and the test skips if it has not been built.
fn resolve_cli_bin() -> Option<PathBuf> {
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

fn simple_launch_dir() -> PathBuf {
    fixtures::repo_root().join("tests/fixtures/simple_test/launch")
}

fn fixture_dir() -> PathBuf {
    fixtures::test_workspace_path("rt_workspace")
}

fn require_rt_workspace() -> bool {
    let installed_pkg = fixture_dir().join("install/rt_demo");
    if !installed_pkg.is_dir() {
        eprintln!(
            "SKIP: rt_workspace fixture not built ({} missing) — run \
             `cd tests/fixtures/rt_workspace && just build` first",
            installed_pkg.display()
        );
        return false;
    }
    true
}

// ── No-ROS error paths (direct launch-file path, no sidecar files) ──

#[test]
fn eject_errors_when_provider_ships_neither_file() {
    let launch = simple_launch_dir().join("pure_nodes.launch.xml");
    if !launch.exists() {
        eprintln!("Skipping: simple_test fixture not available");
        return;
    }
    // Copy to an isolated temp dir so no stray sidecar from the real
    // simple_test/launch/ directory (if one is ever added) leaks in.
    let launch_tmp = tempfile::TempDir::new().expect("failed to create temp dir");
    let launch_copy = launch_tmp.path().join("pure_nodes.launch.xml");
    std::fs::copy(&launch, &launch_copy).expect("failed to copy launch file");

    let Some(bin) = resolve_cli_bin() else {
        return;
    };
    let into = tempfile::TempDir::new().expect("failed to create into dir");
    let output = Command::new(bin)
        .args(["contract", "eject"])
        .arg(&launch_copy)
        .arg("--into")
        .arg(into.path())
        .output()
        .expect("failed to run ros-launch-resolve");

    assert!(
        !output.status.success(),
        "expected failure when neither provider file exists"
    );
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("neither a provider contract nor a provider platform file"),
        "got: {stderr}"
    );
}

#[test]
fn eject_errors_when_no_overlay_root_and_no_into() {
    if !require_rt_workspace() {
        return;
    }
    // Full ROS/colcon env (so the binary can actually load — the plain
    // cargo-debug build isn't self-contained; it needs LD_LIBRARY_PATH from
    // install/setup.bash), but with the overlay-discovery env vars scrubbed
    // so nothing resolves (assumes no /etc/play_launch/contracts on the
    // test host, same assumption `resolve_platform_file_none_when_nothing_resolves`
    // makes).
    let mut env = fixtures::rt_workspace_env();
    env.remove("HOME");
    env.remove("PLAY_LAUNCH_CONTRACTS");
    env.remove("XDG_CONFIG_HOME");

    let Some(mut cmd) = resolve_cli_cmd(&env) else {
        return;
    };
    let output = cmd
        .args(["contract", "eject", "rt_demo", "bringup.launch.xml"])
        .output()
        .expect("failed to run ros-launch-resolve");

    assert!(
        !output.status.success(),
        "expected failure with no --into and no discoverable overlay root"
    );
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("no overlay root found"), "got: {stderr}");
}

// ── Happy path + --force semantics (needs the built rt_workspace fixture) ──

#[test]
fn eject_provider_contract_then_check_uses_overlay_channel() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();
    let into = tempfile::TempDir::new().expect("failed to create into dir");

    let Some(mut cmd) = resolve_cli_cmd(&env) else {
        return;
    };
    let output = cmd
        .args(["contract", "eject", "rt_demo", "bringup.launch.xml"])
        .arg("--into")
        .arg(into.path())
        .output()
        .expect("failed to run ros-launch-resolve contract eject");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "eject failed: {stderr}");
    assert!(stderr.contains("Ejected contract:"), "got: {stderr}");

    let dest = into.path().join("rt_demo/launch/bringup.contract.yaml");
    assert!(
        dest.is_file(),
        "expected ejected contract at {}",
        dest.display()
    );
    // rt_demo ships a provider platform file too (Phase 41.5:
    // launch/bringup.system.posix.yaml) — both halves of the pair should
    // have been ejected together.
    assert!(
        stderr.contains("Ejected platform file:"),
        "expected the provider platform file to also be ejected: {stderr}"
    );
    let platform_dest = into.path().join("rt_demo/launch/bringup.system.posix.yaml");
    assert!(
        platform_dest.is_file(),
        "expected ejected platform file at {}",
        platform_dest.display()
    );

    // Re-check with --contracts pointing at the overlay: the ejected
    // contract must now win over the provider sidecar.
    let check_output = fixtures::play_launch_cmd(&env)
        .args(["check", "--contracts"])
        .arg(into.path())
        .args(["rt_demo", "bringup.launch.xml"])
        .output()
        .expect("failed to run play_launch check");
    let check_stderr = String::from_utf8_lossy(&check_output.stderr);
    assert!(
        check_stderr.contains("1 overlay"),
        "expected the ejected contract to resolve via the overlay channel: {check_stderr}"
    );
}

#[test]
fn eject_refuses_overwrite_without_force_then_force_succeeds() {
    if !require_rt_workspace() {
        return;
    }
    let env = fixtures::rt_workspace_env();
    let into = tempfile::TempDir::new().expect("failed to create into dir");

    let Some(mut first_cmd) = resolve_cli_cmd(&env) else {
        return;
    };
    let first = first_cmd
        .args(["contract", "eject", "rt_demo", "bringup.launch.xml"])
        .arg("--into")
        .arg(into.path())
        .output()
        .expect("failed to run ros-launch-resolve contract eject");
    assert!(
        first.status.success(),
        "first eject failed: {}",
        String::from_utf8_lossy(&first.stderr)
    );

    // Second eject without --force must refuse.
    let Some(mut second_cmd) = resolve_cli_cmd(&env) else {
        return;
    };
    let second = second_cmd
        .args(["contract", "eject", "rt_demo", "bringup.launch.xml"])
        .arg("--into")
        .arg(into.path())
        .output()
        .expect("failed to run ros-launch-resolve contract eject");
    assert!(
        !second.status.success(),
        "expected refusal to overwrite without --force"
    );
    let second_stderr = String::from_utf8_lossy(&second.stderr);
    assert!(
        second_stderr.contains("refusing to overwrite"),
        "got: {second_stderr}"
    );

    // With --force it must succeed.
    let Some(mut third_cmd) = resolve_cli_cmd(&env) else {
        return;
    };
    let third = third_cmd
        .args(["contract", "eject", "rt_demo", "bringup.launch.xml"])
        .arg("--into")
        .arg(into.path())
        .arg("--force")
        .output()
        .expect("failed to run ros-launch-resolve contract eject");
    assert!(
        third.status.success(),
        "--force should overwrite: {}",
        String::from_utf8_lossy(&third.stderr)
    );
}
