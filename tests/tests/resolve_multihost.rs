//! Multi-host launches partition at resolve time via a standard `<arg>` +
//! `if=` condition.
//!
//! ROS 2 has no multi-machine launch — `<node machine="…">` is ROS 1
//! roslaunch syntax and `launch_xml` rejects it (`ros2/design` #255, the
//! multi-machine proposal, was closed unmerged). Selecting the host with a
//! launch argument means each resolved SystemModel already holds exactly
//! one host's nodes, so no `host` field is needed anywhere in the model.

use play_launch_tests::fixtures;
use std::collections::BTreeSet;

/// Resolve the fixture for one `host:=` value with one parser.
fn resolve(parser: &str, host: &str, out: &std::path::Path) -> serde_json::Value {
    let env = fixtures::install_env();
    let launch =
        fixtures::repo_root().join("tests/fixtures/multihost/launch/multihost.launch.xml");
    let mut cmd = fixtures::ros_launch_resolve_cmd(&env);
    cmd.args([
        "resolve",
        "--parser",
        parser,
        launch.to_str().unwrap(),
        &format!("host:={host}"),
        "-o",
        out.to_str().unwrap(),
    ]);
    let output = cmd.output().expect("run ros-launch-resolve resolve");
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

/// `check` must bind `KEY:=VALUE` given a DIRECT launch-file path.
///
/// Regression guard: `check` lacked the positional reclassification that
/// `resolve` has, so clap bound the first `host:=robot1` to the optional
/// `<launch_file>` positional and it never reached the parser. `check` then
/// validated contracts against all 3 nodes while `resolve` — same command
/// line — produced 2. A CI gate reported green on a node set nobody asked
/// for. The node count `check` prints is the observable signal, and it must
/// agree with `resolve`'s on BOTH CLIs.
#[test]
fn check_honours_launch_args_given_a_direct_path() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let launch =
        fixtures::repo_root().join("tests/fixtures/multihost/launch/multihost.launch.xml");

    // `resolve` is the reference: host:=robot1 selects talker1 + hub.
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let reference = node_fqns(&resolve("rust", "robot1", &tmp.path().join("ref.yaml"))).len();
    assert_eq!(reference, 2, "fixture changed: expected 2 nodes for robot1");

    for (label, mut cmd) in [
        ("play_launch", fixtures::play_launch_cmd(&env)),
        (
            "ros-launch-resolve",
            fixtures::ros_launch_resolve_cmd(&env),
        ),
    ] {
        cmd.args(["check", launch.to_str().unwrap(), "host:=robot1"]);
        let output = cmd.output().expect("run check");
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(
            output.status.success(),
            "{label} check failed:\n{stderr}"
        );
        assert!(
            stderr.contains(&format!("{reference} nodes")),
            "{label} check must parse the same {reference} nodes `resolve` does \
             (host:=robot1 was dropped if this says 3):\n{stderr}"
        );
    }
}

/// `launch` must bind `KEY:=VALUE` given a DIRECT launch-file path.
///
/// Same defect class as `check_honours_launch_args_given_a_direct_path`
/// above, pinned separately because the fix lives at a different call site
/// (`src/play_launch/src/commands/launch.rs:34`, not
/// `verbs/check.rs`/`verbs/resolve.rs`) and `launch` is the most-used verb
/// in the product — the worst one to leave silently unpinned. `--check`
/// validates and reports the resolved node count without spawning anything,
/// so this stays fast and needs no cleanup.
#[test]
fn launch_honours_launch_args_given_a_direct_path() {
    let env = fixtures::install_env();
    if env.is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let launch =
        fixtures::repo_root().join("tests/fixtures/multihost/launch/multihost.launch.xml");

    // `resolve` is the reference: host:=robot1 selects talker1 + hub.
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let reference = node_fqns(&resolve("rust", "robot1", &tmp.path().join("ref.yaml"))).len();
    assert_eq!(reference, 2, "fixture changed: expected 2 nodes for robot1");

    let mut cmd = fixtures::play_launch_cmd(&env);
    cmd.args([
        "launch",
        launch.to_str().unwrap(),
        "host:=robot1",
        "--check",
    ]);
    let output = cmd.output().expect("run play_launch launch --check");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.success(),
        "play_launch launch --check failed:\nstdout:\n{stdout}\nstderr:\n{stderr}"
    );
    assert!(
        stdout.contains(&format!("{reference} node(s)")),
        "play_launch launch --check must parse the same {reference} nodes `resolve` does \
         (host:=robot1 was dropped if this says 3):\nstdout:\n{stdout}\nstderr:\n{stderr}"
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
    let mut cmd = fixtures::ros_launch_resolve_cmd(&env);
    cmd.args([
        "resolve",
        launch.to_str().unwrap(),
        "-o",
        tmp.path().join("out.yaml").to_str().unwrap(),
    ]);
    let output = cmd.output().expect("run ros-launch-resolve resolve");
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
