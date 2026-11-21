//! `<param from=…>` reads a parameter file as written unless it says
//! `allow_substs="true"` (#0106). launch_ros's `ParameterFile` defaults to
//! `allow_substs=False`; stock `ros2 launch` on Humble starts a node whose
//! file holds `note: $(var not_declared)` and the node's `note` is that
//! string. The parser substituted every file and refused the launch with
//! `Undefined variable: 'not_declared'`.
//!
//! Checked for the three places a file reaches: a plain node (its file's
//! content), a container (a node too) and a composable (the values in its
//! LoadNode request).

use play_launch_parser::parse_launch_file;
use std::collections::HashMap;

const RAW: &str = "/**:\n  ros__parameters:\n    note: $(var not_declared)\n";
const OPTED_IN: &str = "/**:\n  ros__parameters:\n    note: $(var map)\n";

fn launch(dir: &std::path::Path, allow: &str) -> serde_json::Value {
    let root = dir.join(format!("l{}.launch.xml", allow.len()));
    let text = format!(
        r#"<launch>
  <arg name="map" default="/maps/town"/>
  <node pkg="demo_nodes_cpp" exec="parameter_blackboard" name="talker">
    <param from="$(dirname)/p.yaml"{allow}/>
  </node>
  <node_container pkg="rclcpp_components" exec="component_container" name="c" namespace="">
    <param from="$(dirname)/p.yaml"{allow}/>
    <composable_node pkg="demo_nodes_cpp" plugin="demo_nodes_cpp::Talker" name="t">
      <param from="$(dirname)/p.yaml"{allow}/>
    </composable_node>
  </node_container>
</launch>
"#
    );
    std::fs::write(&root, text).unwrap();
    serde_json::to_value(parse_launch_file(&root, HashMap::new()).unwrap()).unwrap()
}

fn composable_note(json: &serde_json::Value) -> String {
    json["load_node"][0]["params"]
        .as_array()
        .unwrap()
        .iter()
        .find(|p| p[0] == "note")
        .unwrap_or_else(|| panic!("{json:#}"))[1]
        .as_str()
        .unwrap()
        .to_string()
}

fn file_contents(json: &serde_json::Value, kind: &str) -> String {
    json[kind][0]["params_files"]
        .as_array()
        .unwrap()
        .iter()
        .map(|f| f.as_str().unwrap().to_string())
        .collect::<Vec<_>>()
        .join("\n")
}

#[test]
fn a_param_file_without_the_opt_in_is_read_as_written() {
    let dir = tempfile::tempdir().unwrap();
    std::fs::write(dir.path().join("p.yaml"), RAW).unwrap();
    for allow in ["", r#" allow_substs="false""#] {
        let json = launch(dir.path(), allow);
        assert!(
            file_contents(&json, "node").contains("note: $(var not_declared)"),
            "node, {allow:?}: {json:#}"
        );
        assert!(
            file_contents(&json, "container").contains("note: $(var not_declared)"),
            "container, {allow:?}: {json:#}"
        );
        assert_eq!(composable_note(&json), "$(var not_declared)", "{allow:?}");
    }
}

#[test]
fn a_param_file_with_the_opt_in_is_substituted() {
    let dir = tempfile::tempdir().unwrap();
    std::fs::write(dir.path().join("p.yaml"), OPTED_IN).unwrap();
    let json = launch(dir.path(), r#" allow_substs="true""#);
    assert!(
        file_contents(&json, "node").contains("note: /maps/town"),
        "{json:#}"
    );
    assert!(
        file_contents(&json, "container").contains("note: /maps/town"),
        "{json:#}"
    );
    assert_eq!(composable_note(&json), "/maps/town");
}
