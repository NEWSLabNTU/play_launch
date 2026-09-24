//! Issue #0048 — a contract may name a node the launch tree does not have.
//!
//! `resolve_node_fqn`'s last branch qualifies a bare contract key against the
//! SCOPE's namespace when the launch dump's identity map does not have it.
//! That is not the node's namespace whenever the node carries its own
//! `namespace=` attribute, so the key resolves to a syntactically fine FQN
//! naming nothing, every requirement written on it is checked against a vertex
//! that does not run, and — before the fix — `check` reported
//! `1 clean, 0 errors, 0 warnings` and exited 0.
//!
//! The fallback is not the defect and is not removed: it is what lets a
//! contract be checked with no launch dump at all. These tests pin the
//! diagnostic for the case where the dump WAS available and the key was not
//! in it, plus the control that an absolute key passes through untouched.
//!
//! Driven through the `ros-launch-resolve` CLI, which owns `check` (same
//! pattern and same skip as `manifest_check.rs`).

use play_launch_tests::fixtures;
use std::path::{Path, PathBuf};
use std::process::Command;

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

/// The launch file #0048 was measured on: `talker` carries no `name=` and
/// takes its namespace from its own attribute, so the scope namespace ("")
/// does not predict where it lands — the exact shape the fallback gets wrong.
fn unnamed_node_launch() -> PathBuf {
    fixtures::repo_root().join("tests/fixtures/simple_test/launch/unnamed_node.launch.xml")
}

/// Write `yaml` into a throwaway overlay tree as the contract for
/// `unnamed_node.launch.xml`.
///
/// An overlay rather than a provider sidecar ON PURPOSE: a
/// `unnamed_node.contract.yaml` next to the launch file would be picked up by
/// every other test that resolves this fixture, so a test about a deliberately
/// WRONG contract would start failing launches that have nothing to do with
/// it. `_` is the package directory for a raw-path launch file (see
/// `resolve_overlay_path`).
fn overlay_with(dir: &Path, yaml: &str) {
    let launch_dir = dir.join("_/launch");
    std::fs::create_dir_all(&launch_dir).expect("create overlay launch dir");
    std::fs::write(launch_dir.join("unnamed_node.contract.yaml"), yaml).expect("write contract");
}

fn run_check(overlay: &Path) -> Option<std::process::Output> {
    let bin = ros_launch_resolve_bin()?;
    Some(
        Command::new(bin)
            .args(["check", "--contracts"])
            .arg(overlay)
            .arg(unnamed_node_launch())
            .output()
            .expect("failed to run ros-launch-resolve"),
    )
}

#[test]
fn a_contract_naming_a_node_the_launch_tree_lacks_fails_the_check() {
    if !unnamed_node_launch().exists() {
        eprintln!("SKIP: simple_test fixture not available");
        return;
    }
    let overlay = tempfile::TempDir::new().expect("overlay root");
    overlay_with(
        overlay.path(),
        r#"
version: 1
nodes:
  talker-1:
    pub: [chatter]
topics:
  chatter:
    type: std_msgs/msg/String
    pub: [talker-1/chatter]
    sub: []
    external: sub
"#,
    );

    let Some(out) = run_check(overlay.path()) else {
        return;
    };
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    let all = format!("{stdout}{stderr}");

    assert!(
        all.contains("node-identity-unknown"),
        "expected the #0048 diagnostic; got:\n{all}"
    );
    // The key, as written.
    assert!(all.contains("'talker-1'"), "{all}");
    // The phantom FQN it fell back to — before the fix, `--export-graph` was
    // the only place this number appeared at all.
    assert!(all.contains("'/talker-1'"), "{all}");
    // The candidate: the bare name the launch file really declares.
    assert!(all.contains("Did you mean 'talker'?"), "{all}");
    assert_eq!(
        out.status.code(),
        Some(1),
        "a contract describing a node that does not exist must not exit 0:\n{all}"
    );
}

#[test]
fn the_same_contract_written_with_absolute_keys_stays_clean() {
    if !unnamed_node_launch().exists() {
        eprintln!("SKIP: simple_test fixture not available");
        return;
    }
    let overlay = tempfile::TempDir::new().expect("overlay root");
    // `/identity_test/talker` is where the node really lands. An absolute key
    // never reaches the namespace fallback, which is why
    // `scripts/capture_manifest.py` emits them.
    overlay_with(
        overlay.path(),
        r#"
version: 1
nodes:
  /identity_test/talker:
    pub: [chatter]
topics:
  chatter:
    type: std_msgs/msg/String
    pub: [/identity_test/talker/chatter]
    sub: []
    external: sub
"#,
    );

    let Some(out) = run_check(overlay.path()) else {
        return;
    };
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    let all = format!("{stdout}{stderr}");

    assert!(
        !all.contains("node-identity-unknown"),
        "an absolute key passes through verbatim and must not be reported:\n{all}"
    );
    assert_eq!(out.status.code(), Some(0), "{all}");
}
