//! A `system.toml` `[lifecycle] autostart` reaches no node (phase 86 W0).
//!
//! The key is one realizer's boot policy (rlm `docs/model-boundary.md`). The
//! resolver used to project it onto EVERY node in `structure.nodes`, which
//! overwrote the per-node answer the launch file gives and put an autostart
//! request on nodes that are not lifecycle nodes at all. A node's
//! `lifecycle_autostart` now comes from the launch file and the contract only.

use play_launch_tests::fixtures;

#[test]
fn a_system_lifecycle_default_reaches_no_node() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    let env = fixtures::install_env();
    let tmp = tempfile::TempDir::new().expect("tempdir");
    let launch = fixtures::repo_root()
        .join("tests/fixtures/lifecycle/launch/lifecycle_autostart.launch.xml");
    let system = tmp.path().join("system.toml");
    // `configure` differs from every answer the launch file gives (`active`
    // or nothing), so a projection cannot hide behind a coincidence.
    std::fs::write(&system, "[lifecycle]\nautostart = \"configure\"\n").expect("write system.toml");
    let out = tmp.path().join("model.yaml");

    let mut cmd = fixtures::ros_launch_resolve_cmd(&env);
    cmd.args([
        "resolve",
        "--parser",
        "rust",
        launch.to_str().unwrap(),
        "--system",
        system.to_str().unwrap(),
        "-o",
        out.to_str().unwrap(),
    ]);
    let output = cmd.output().expect("run ros-launch-resolve resolve");
    assert!(
        output.status.success(),
        "resolve --system failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let yaml = std::fs::read_to_string(&out).expect("read model");
    let model: serde_json::Value = serde_yaml_ng::from_str(&yaml).expect("parse model yaml");

    let nodes = model["structure"]["nodes"]
        .as_object()
        .expect("structure.nodes");
    let autostart = |fqn: &str| -> Option<String> {
        let node = nodes
            .get(fqn)
            .unwrap_or_else(|| panic!("{fqn} missing from {:?}", nodes.keys()));
        node.get("lifecycle_autostart")
            .and_then(|v| v.as_str())
            .map(str::to_string)
    };
    // The launch file's answers, untouched by `[lifecycle] autostart`.
    assert_eq!(autostart("/lc_on").as_deref(), Some("active"));
    assert_eq!(autostart("/lc_off"), None);
    assert_eq!(autostart("/lc_unsaid"), None);
}
