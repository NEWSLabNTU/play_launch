//! A file the launch tree names and that does not exist, judged as stock
//! `ros2 launch` (Humble) judges it, each case checked against it:
//!
//! - an `<include>` of a missing file fails, and the error names the file and
//!   the including file (#0088);
//! - a composable node's missing parameter file fails the launch, since
//!   `LoadComposableNodes` opens it to build the request (#0089);
//! - a plain node's missing parameter file is only a warning
//!   (`Parameter file path is not a file`), and the node starts without it.

use play_launch_parser::parse_launch_file;
use std::collections::HashMap;

fn write(dir: &std::path::Path, name: &str, text: &str) -> std::path::PathBuf {
    let p = dir.join(name);
    std::fs::write(&p, text).unwrap();
    p
}

#[test]
fn a_missing_include_names_the_file_and_the_includer() {
    let dir = tempfile::tempdir().unwrap();
    for (name, text, missing) in [
        (
            "a.launch.xml",
            "<launch>\n  <include file=\"/nonexistent/x.launch.xml\"/>\n</launch>\n",
            "/nonexistent/x.launch.xml",
        ),
        (
            "b.launch.yaml",
            "launch:\n- include:\n    file: /nonexistent/y.launch.py\n",
            "/nonexistent/y.launch.py",
        ),
    ] {
        let root = write(dir.path(), name, text);
        let err = parse_launch_file(&root, HashMap::new()).unwrap_err();
        let msg = err.to_string();
        assert!(err.is_missing_file(), "{msg}");
        assert!(
            msg.contains(&format!("Included launch file not found: {missing}"))
                && msg.contains(&format!("included from {}", root.display())),
            "{msg}"
        );
    }
}

#[test]
fn a_composable_nodes_missing_param_file_is_an_error() {
    let dir = tempfile::tempdir().unwrap();
    let root = write(
        dir.path(),
        "c.launch.xml",
        r#"<launch>
  <node_container pkg="rclcpp_components" exec="component_container" name="c" namespace="">
    <composable_node pkg="demo_nodes_cpp" plugin="demo_nodes_cpp::Talker" name="t">
      <param from="/nonexistent/params.yaml"/>
    </composable_node>
  </node_container>
</launch>
"#,
    );
    let err = parse_launch_file(&root, HashMap::new()).unwrap_err();
    let msg = err.to_string();
    assert!(err.is_missing_file(), "{msg}");
    assert!(
        msg.contains("Parameter file not found: /nonexistent/params.yaml (composable node 't')"),
        "{msg}"
    );
}

#[test]
fn a_plain_nodes_missing_param_file_is_left_out() {
    let dir = tempfile::tempdir().unwrap();
    let root = write(
        dir.path(),
        "p.launch.xml",
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t">
    <param from="/nonexistent/params.yaml"/>
    <param name="x" value="1"/>
  </node>
</launch>
"#,
    );
    let record = parse_launch_file(&root, HashMap::new()).unwrap();
    assert_eq!(record.node.len(), 1);
    let node = &record.node[0];
    assert!(node.params_files.is_empty(), "{:?}", node.params_files);
    assert!(
        !node.cmd.iter().any(|a| a.contains("/nonexistent/")),
        "{:?}",
        node.cmd
    );
}

/// With `allow_substs="true"` launch opens the file to substitute in it, so a
/// missing one fails there too.
#[test]
fn a_missing_param_file_with_allow_substs_is_an_error() {
    let dir = tempfile::tempdir().unwrap();
    let root = write(
        dir.path(),
        "s.launch.xml",
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="t">
    <param from="/nonexistent/params.yaml" allow_substs="true"/>
  </node>
</launch>
"#,
    );
    let msg = parse_launch_file(&root, HashMap::new())
        .unwrap_err()
        .to_string();
    assert!(msg.contains("/nonexistent/params.yaml"), "{msg}");
}
