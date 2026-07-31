//! Multi-host launches partition at resolve time via a standard `<arg>` +
//! `if=` condition.
//!
//! ROS 2 has no multi-machine launch — `<node machine="…">` is ROS 1
//! roslaunch syntax and `launch_xml` rejects it (`ros2/design` #255, the
//! multi-machine proposal, was closed unmerged). Selecting the host with a
//! launch argument means each resolved SystemModel already holds exactly
//! one host's nodes, so no `host` field is needed anywhere in the model.

use play_launch_tests::fixtures;
use std::{collections::BTreeSet, path::PathBuf, process::Command};

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

/// Resolve the fixture for one `host:=` value with one parser.
fn resolve(parser: &str, host: &str, out: &std::path::Path) -> serde_json::Value {
    let env = fixtures::install_env();
    let launch =
        fixtures::repo_root().join("tests/fixtures/multihost/launch/multihost.launch.xml");
    let mut cmd = Command::new(play_launch_bin());
    cmd.env_clear();
    cmd.envs(&env);
    cmd.args([
        "resolve",
        "--parser",
        parser,
        launch.to_str().unwrap(),
        &format!("host:={host}"),
        "-o",
        out.to_str().unwrap(),
    ]);
    let output = cmd.output().expect("run play_launch resolve");
    assert!(
        output.status.success(),
        "resolve --parser {parser} host:={host} failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let yaml = std::fs::read_to_string(out).expect("read model");
    serde_yaml_ng::from_str(&yaml).expect("parse model yaml")
}

fn node_fqns(model: &serde_json::Value) -> BTreeSet<String> {
    model["structure"]["nodes"]
        .as_object()
        .expect("structure.nodes")
        .keys()
        .cloned()
        .collect()
}

fn expected(host: &str) -> BTreeSet<String> {
    let mut set = BTreeSet::new();
    // Unconditioned — present for every host.
    set.insert("/hub/hub".to_string());
    if host == "robot1" || host == "all" {
        set.insert("/robot1/talker1".to_string());
    }
    if host == "robot2" || host == "all" {
        set.insert("/robot2/talker2".to_string());
    }
    set
}

#[test]
fn host_arg_partitions_the_launch_in_both_parsers() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let tmp = tempfile::TempDir::new().expect("tempdir");

    for parser in ["rust", "python"] {
        for host in ["robot1", "robot2", "all"] {
            let out = tmp.path().join(format!("{parser}_{host}.yaml"));
            let model = resolve(parser, host, &out);
            assert_eq!(
                node_fqns(&model),
                expected(host),
                "--parser {parser} host:={host} selected the wrong node set"
            );
        }
    }
}

#[test]
fn a_launch_only_resolve_produces_no_deploy_entries() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let out = tmp.path().join("all.yaml");
    let model = resolve("rust", "all", &out);

    // Regression guard on the `machine=` removal: nothing in a launch file
    // creates a deploy entry any more. Placement comes from a `--system`
    // config pass, never from launch syntax.
    let deploy = &model["execution"]["deploy"];
    assert!(
        deploy.is_null() || deploy.as_object().is_some_and(|d| d.is_empty()),
        "a launch-only resolve must produce no deploy entries: {deploy:?}"
    );
}

#[test]
fn the_ros1_machine_attribute_is_rejected() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let launch = tmp.path().join("ros1_machine.launch.xml");
    std::fs::write(
        &launch,
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t" machine="robot1"/>
</launch>
"#,
    )
    .expect("write fixture");

    let env = fixtures::install_env();
    let mut cmd = Command::new(play_launch_bin());
    cmd.env_clear();
    cmd.envs(&env);
    cmd.args([
        "resolve",
        launch.to_str().unwrap(),
        "-o",
        tmp.path().join("out.yaml").to_str().unwrap(),
    ]);
    let output = cmd.output().expect("run play_launch resolve");
    assert!(
        !output.status.success(),
        "machine= is ROS 1 syntax and must fail the resolve"
    );
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("machine"),
        "the error must name the offending attribute:\n{stderr}"
    );
}
