//! Multi-host `machine=` → `execution.deploy.host` golden test (nano-ros
//! #236, Phase 46.1).
//!
//! `<node machine="…">` must survive `play_launch resolve`: the parser
//! captures it (`play_launch_parser` `record::types::NodeRecord::machine`),
//! `launch_dump::NodeRecord` now carries it through deserialization, and
//! `model_builder::build_system_model` maps it to
//! `execution.deploy[fqn].host`. Nodes without `machine` must get no deploy
//! entry (single-host launches stay unaffected).

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
        fixtures::repo_root().join("tests/fixtures/multihost/launch/multihost.launch.xml");
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
fn resolve_carries_node_machine_into_deploy_host() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let out = tmp.path().join("system_model.yaml");
    let model = resolve_fixture(&out);

    // Both hosted nodes present in structure with their launch FQNs.
    let nodes = model["structure"]["nodes"].as_object().expect("nodes");
    assert!(
        nodes.contains_key("/robot1/talker1"),
        "structure.nodes: {nodes:?}"
    );
    assert!(
        nodes.contains_key("/robot2/talker2"),
        "structure.nodes: {nodes:?}"
    );
    assert!(nodes.contains_key("/hub/hub"), "structure.nodes: {nodes:?}");

    // machine= → execution.deploy[fqn].host, keyed on the SAME launch FQN.
    let deploy = model["execution"]["deploy"]
        .as_object()
        .expect("execution.deploy present");
    assert_eq!(
        deploy["/robot1/talker1"]["host"], "robot1",
        "deploy: {deploy:?}"
    );
    assert_eq!(
        deploy["/robot2/talker2"]["host"], "robot2",
        "deploy: {deploy:?}"
    );

    // Unhosted node: no deploy entry (backward-compatible — single-host
    // launches produce an empty execution.deploy exactly as before).
    assert!(
        deploy.get("/hub/hub").is_none(),
        "unhosted node must not get a deploy entry: {deploy:?}"
    );
}
