//! SystemModel merge + full-dump tests (Phase 43 / RFC-0050).
//!
//! `play_launch resolve` on the `contract_merge` fixture must merge the
//! launch tree (root + namespaced include) with BOTH scopes' contract
//! sidecars into one model that carries the complete declared information:
//! merged topic wiring, every contract field class, externals, provenance,
//! and the arg binding.

use play_launch_tests::fixtures;
use std::{path::PathBuf, process::Command, time::Duration};

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
        fixtures::repo_root().join("tests/fixtures/contract_merge/launch/bringup.launch.xml");
    let mut cmd = Command::new(play_launch_bin());
    cmd.env_clear();
    cmd.envs(&env);
    cmd.args([
        "resolve",
        launch.to_str().unwrap(),
        "mode:=turbo",
        "-o",
        out.to_str().unwrap(),
    ]);
    let output = cmd.output().expect("run play_launch resolve");
    assert!(
        output.status.success(),
        "resolve failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let yaml = std::fs::read_to_string(out).expect("read model");
    serde_yaml_ng::from_str(&yaml).expect("parse model yaml")
}

/// Wait guard: keep clippy quiet about the unused import if the harness
/// changes; Duration is used by sibling suites' helpers.
#[allow(dead_code)]
const _POLL: Duration = Duration::from_secs(1);

#[test]
fn resolve_merges_launch_and_contracts_into_full_model() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let out = tmp.path().join("system_model.yaml");
    let model = resolve_fixture(&out);

    // --- meta: binding + provenance ------------------------------------
    assert_eq!(
        model["meta"]["args"]["mode"], "turbo",
        "CLI arg binding recorded"
    );
    let inputs = model["meta"]["inputs"].as_array().expect("inputs");
    let paths: Vec<&str> = inputs.iter().filter_map(|i| i["path"].as_str()).collect();
    for needle in [
        "bringup.launch.xml",
        "perception.launch.xml",
        "bringup.contract.yaml",
        "perception.contract.yaml",
    ] {
        assert!(
            paths.iter().any(|p| p.ends_with(needle)),
            "{needle} must be hashed into meta.inputs, got {paths:?}"
        );
    }
    assert!(
        model["meta"]["record"]["sha256"].is_string(),
        "record companion bound"
    );

    // --- structure: scope tree + nodes from BOTH launch files ----------
    let nodes = model["structure"]["nodes"].as_object().expect("nodes");
    assert_eq!(nodes["/planning/planner"]["exec"], "listener");
    assert_eq!(nodes["/planning/planner"]["criticality"], "high");
    assert_eq!(nodes["/perception/detector"]["exec"], "talker");
    assert_eq!(nodes["/perception/detector"]["lifecycle"], true);
    let scopes = model["structure"]["scopes"].as_object().expect("scopes");
    assert!(
        scopes["/perception"]["parent"].is_string(),
        "included scope has a parent link"
    );
    assert!(
        scopes["/perception"]["manifest"]
            .as_str()
            .unwrap()
            .ends_with("perception.contract.yaml"),
        "child scope bound to its own contract file"
    );

    // --- the merge itself: one topic, producer side from the child scope,
    // consumer side from the root scope, both reconciled to launch FQNs.
    let objects = &model["structure"]["topics"]["/perception/objects"];
    assert_eq!(objects["type"], "std_msgs/msg/String");
    assert_eq!(objects["pub"][0], "/perception/detector/objects");
    assert_eq!(objects["sub"][0], "/planning/planner/objects");

    // Service declared in the root scope, server ref reconciled.
    assert_eq!(
        model["structure"]["services"]["/planning/compute"]["server"][0],
        "/planning/planner/compute"
    );

    // --- contracts: every field class survives the dump -----------------
    let c = &model["contracts"];
    assert_eq!(
        c["pub_endpoints"]["/perception/detector/objects"]["min_rate_hz"],
        10.0
    );
    assert_eq!(
        c["pub_endpoints"]["/perception/detector/objects"]["jitter_ms"],
        5.0
    );
    let sub = &c["sub_endpoints"]["/planning/planner/objects"];
    assert_eq!(sub["min_rate_hz"], 5.0);
    assert_eq!(sub["max_age_ms"], 200.0);
    assert_eq!(sub["required"], true);
    assert_eq!(c["sub_endpoints"]["/planning/planner/map"]["state"], true);
    assert_eq!(
        c["srv_endpoints"]["/planning/planner/compute"]["max_response_ms"],
        100.0
    );
    // node path: periodic (no input), processing budget
    let main = &c["node_paths"]["/perception/detector/main"];
    assert!(main["input"].is_null() || main["input"].as_array().is_none_or(|a| a.is_empty()));
    assert_eq!(main["max_latency_ms"], 15.0);
    // scope path: E2E budget + correlation + drop, rooted at the root scope
    let (e2e_key, e2e) = c["scope_paths"]
        .as_object()
        .expect("scope_paths")
        .iter()
        .find(|(k, _)| k.ends_with("/e2e"))
        .expect("e2e scope path present");
    assert!(
        e2e_key.starts_with('/'),
        "scope-path key is FQN-shaped: {e2e_key}"
    );
    assert_eq!(e2e["input"][0], "/sensing/points");
    assert_eq!(e2e["output"][0], "/perception/objects");
    assert_eq!(e2e["max_latency_ms"], 80.0);
    assert_eq!(e2e["correlation"], "timestamp");
    assert_eq!(e2e["tolerance_ms"], 10.0);
    assert_eq!(e2e["drop"]["max_drop_rate"], 0.08);
    assert_eq!(e2e["drop"]["max_consecutive"], 5);
    // topic channel contract: fields merged from BOTH scopes' declarations
    // (qos from the root decl; rate/transport/drop from the child decl)
    let tc = &c["topics"]["/perception/objects"];
    assert_eq!(tc["rate_hz"], 10.0);
    assert_eq!(tc["max_transport_ms"], 5.0);
    assert_eq!(tc["drop"]["max_drop_rate"], 0.05);
    assert_eq!(tc["drop"]["max_consecutive"], 3);
    assert_eq!(tc["qos"]["reliability"], "reliable");
    assert_eq!(tc["qos"]["depth"], 5);
    // externals
    assert_eq!(c["externals"]["/sensing/points"], "pub");
}
