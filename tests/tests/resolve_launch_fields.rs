//! Launch spawn fields → `NodeInstance` golden test (Phase 46.1b / 46.2).
//!
//! `<remap>`, `<env>`, and `respawn`/`respawn_delay` must survive
//! `play_launch resolve`: `play_launch_parser` already captures them into
//! `LaunchDump::{NodeRecord,ComposableNodeRecord}`, and
//! `model_builder::build_system_model` now maps them into
//! `NodeInstance::{remaps,env,respawn,respawn_delay}`. Composable nodes
//! carry `remaps` but never `ros_args`/`respawn`/`respawn_delay` (no
//! independent process/lifecycle of their own).
//!
//! Container-level remaps/env aren't exercised here — see the fixture XML's
//! doc comment for why (a pre-existing `play_launch_parser` XML gap, out of
//! scope for Phase 46.2); `model_builder`'s container-side mapping has its
//! own direct unit-test coverage in `play_launch`.

use play_launch_tests::fixtures;
use std::{path::PathBuf, process::Command};

fn play_launch_bin() -> PathBuf {
    let release = fixtures::repo_root().join("target/release/play_launch");
    if release.is_file() {
        return release;
    }
    let debug = fixtures::repo_root().join("target/debug/play_launch");
    if debug.is_file() {
        return debug;
    }
    fixtures::play_launch_bin()
}

fn resolve_fixture(out: &std::path::Path) -> serde_json::Value {
    let env = fixtures::install_env();
    let launch =
        fixtures::repo_root().join("tests/fixtures/launch_fields/launch/launch_fields.launch.xml");
    let mut cmd = Command::new(play_launch_bin());
    cmd.env_clear();
    cmd.envs(&env);
    cmd.args(["resolve", launch.to_str().unwrap(), "-o", out.to_str().unwrap()]);
    let output = cmd.output().expect("run play_launch resolve");
    assert!(
        output.status.success(),
        "resolve failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let yaml = std::fs::read_to_string(out).expect("read model");
    serde_yaml_ng::from_str(&yaml).expect("parse model yaml")
}

#[test]
fn resolve_carries_remaps_env_respawn_into_node_instance() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let out = tmp.path().join("system_model.yaml");
    let model = resolve_fixture(&out);

    let nodes = model["structure"]["nodes"].as_object().expect("nodes");

    // Regular node: remaps + env + respawn + respawn_delay all present.
    let talker = &nodes["/chatter/talker"];
    assert_eq!(
        talker["remaps"],
        serde_json::json!([{"from": "chatter", "to": "/chatter/renamed"}]),
        "talker: {talker:?}"
    );
    assert_eq!(
        talker["env"],
        serde_json::json!([{"name": "TALKER_LOG_LEVEL", "value": "debug"}]),
        "talker: {talker:?}"
    );
    assert_eq!(talker["respawn"], true, "talker: {talker:?}");
    assert_eq!(talker["respawn_delay"], 2.5, "talker: {talker:?}");
    // ros_args: the Rust parser never populates NodeRecord::ros_args from
    // any XML syntax (no `ros_args=` attribute exists) — always empty,
    // omitted from the emitted YAML by `skip_serializing_if`.
    assert!(talker.get("ros_args").is_none(), "talker: {talker:?}");

    // Composable node: remaps only — no ros_args/respawn/respawn_delay keys
    // at all (the field is `None`/empty and skipped by serde).
    let composed = &nodes["/chatter/composed_talker"];
    assert_eq!(
        composed["remaps"],
        serde_json::json!([{"from": "chatter", "to": "/chatter/composed"}]),
        "composed: {composed:?}"
    );
    assert!(composed.get("ros_args").is_none(), "composed: {composed:?}");
    assert!(composed.get("respawn").is_none(), "composed: {composed:?}");
    assert!(
        composed.get("respawn_delay").is_none(),
        "composed: {composed:?}"
    );
    assert!(composed.get("env").is_none(), "composed: {composed:?}");

    // Container: present in structure, but no launch-spawn-field keys — the
    // XML parser never routes remap/env into NodeContainerRecord (see the
    // fixture's doc comment).
    let container = &nodes["/chatter/composable_container"];
    assert!(
        container.get("remaps").is_none(),
        "container: {container:?}"
    );
}
