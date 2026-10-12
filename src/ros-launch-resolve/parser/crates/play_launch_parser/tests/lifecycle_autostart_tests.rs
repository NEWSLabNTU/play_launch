//! A lifecycle node and its `autostart`, in every frontend (play_launch
//! phase 86).
//!
//! Jazzy `launch_ros` exposes `<lifecycle_node>` as `<node>` plus
//! `autostart` (a bool substitution, default false), and
//! `LifecycleNode(autostart=True)` requests configure then activate. Humble
//! has neither the element nor the argument. The record carries the marker
//! (`lifecycle: true`) and the RESOLVED `autostart`; the model lowering
//! (`ros-launch-resolve`'s `model_builder`) turns `true` into `Active`.
//!
//! Before this, `<lifecycle_node>` was a dropped action and `LifecycleNode`
//! a plain node, so neither reached the model as a lifecycle node.

use play_launch_parser::parse_launch_file;
use play_launch_parser_pyexec::python_test_guard;
use std::{collections::HashMap, io::Write};
use tempfile::NamedTempFile;

fn write(suffix: &str, body: &str) -> NamedTempFile {
    let mut file = NamedTempFile::with_suffix(suffix).unwrap();
    file.write_all(body.as_bytes()).unwrap();
    file.flush().unwrap();
    file
}

fn parse_with(suffix: &str, body: &str, args: &[(&str, &str)]) -> serde_json::Value {
    let file = write(suffix, body);
    let args = args
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect::<HashMap<_, _>>();
    let dump = parse_launch_file(file.path(), args)
        .unwrap_or_else(|e| panic!("{suffix} should parse: {e}"));
    serde_json::to_value(dump).unwrap()
}

fn parse(suffix: &str, body: &str) -> serde_json::Value {
    parse_with(suffix, body, &[])
}

/// `(lifecycle, lifecycle_autostart)` of the node named `name`.
fn facts(json: &serde_json::Value, name: &str) -> (Option<bool>, Option<bool>) {
    let nodes = json["node"].as_array().expect("node array");
    let n = nodes
        .iter()
        .find(|e| e["name"] == name)
        .unwrap_or_else(|| panic!("{name} missing: {nodes:?}"));
    (
        n.get("lifecycle").and_then(|v| v.as_bool()),
        n.get("lifecycle_autostart").and_then(|v| v.as_bool()),
    )
}

#[test]
fn xml_lifecycle_node_is_a_node_with_the_marker_and_its_autostart() {
    let json = parse_with(
        ".launch.xml",
        r#"<launch>
  <arg name="go" default="false"/>
  <lifecycle_node pkg="lifecycle" exec="lifecycle_talker" name="on" namespace="" autostart="true"/>
  <lifecycle_node pkg="lifecycle" exec="lifecycle_talker" name="off" namespace="" autostart="false"/>
  <lifecycle_node pkg="lifecycle" exec="lifecycle_talker" name="unsaid" namespace=""/>
  <lifecycle_node pkg="lifecycle" exec="lifecycle_talker" name="by_arg" namespace="" autostart="$(var go)"/>
  <node pkg="demo_nodes_cpp" exec="talker" name="plain"/>
</launch>"#,
        &[("go", "True")],
    );
    assert_eq!(facts(&json, "on"), (Some(true), Some(true)));
    assert_eq!(facts(&json, "off"), (Some(true), Some(false)));
    assert_eq!(facts(&json, "unsaid"), (Some(true), None));
    // A substitution resolves, then coerces the way `launch` does (`True`).
    assert_eq!(facts(&json, "by_arg"), (Some(true), Some(true)));
    // A plain node's record is unchanged: neither key is written.
    assert_eq!(facts(&json, "plain"), (None, None));
    let plain = json["node"]
        .as_array()
        .unwrap()
        .iter()
        .find(|e| e["name"] == "plain")
        .unwrap();
    assert!(plain.get("lifecycle").is_none(), "{plain}");
    assert!(plain.get("lifecycle_autostart").is_none(), "{plain}");
    // Dispatched, not dropped: nothing was recorded as unsupported.
    assert!(
        json.get("dropped_actions")
            .and_then(|u| u.as_array())
            .is_none_or(|u| u.is_empty()),
        "{json}"
    );
}

#[test]
fn xml_autostart_is_refused_on_a_plain_node_and_on_a_non_bool() {
    let file = write(
        ".launch.xml",
        r#"<launch><node pkg="p" exec="e" name="n" autostart="true"/></launch>"#,
    );
    let err =
        parse_launch_file(file.path(), HashMap::new()).expect_err("<node autostart> is not ROS 2");
    assert!(err.to_string().contains("autostart"), "{err}");

    let file = write(
        ".launch.xml",
        r#"<launch><lifecycle_node pkg="p" exec="e" name="n" namespace="" autostart="maybe"/></launch>"#,
    );
    let err = parse_launch_file(file.path(), HashMap::new())
        .expect_err("launch refuses a non-bool autostart");
    assert!(err.to_string().contains("maybe"), "{err}");
}

#[test]
fn yaml_lifecycle_node_reads_a_native_bool_and_a_string() {
    let json = parse(
        ".launch.yaml",
        r#"launch:
- lifecycle_node:
    pkg: lifecycle
    exec: lifecycle_talker
    name: native
    namespace: ""
    autostart: true
- lifecycle_node:
    pkg: lifecycle
    exec: lifecycle_talker
    name: text
    namespace: ""
    autostart: "false"
- lifecycle_node:
    pkg: lifecycle
    exec: lifecycle_talker
    name: unsaid
    namespace: ""
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: plain
"#,
    );
    assert_eq!(facts(&json, "native"), (Some(true), Some(true)));
    assert_eq!(facts(&json, "text"), (Some(true), Some(false)));
    assert_eq!(facts(&json, "unsaid"), (Some(true), None));
    assert_eq!(facts(&json, "plain"), (None, None));
}

#[test]
fn python_lifecycle_node_captures_the_marker_and_autostart() {
    let _guard = python_test_guard();
    let json = parse(
        ".launch.py",
        r#"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('go', default_value='true'),
        LifecycleNode(package='lifecycle', executable='lifecycle_talker',
                      name='on', namespace='', autostart=True),
        LifecycleNode(package='lifecycle', executable='lifecycle_talker',
                      name='off', namespace='', autostart=False),
        LifecycleNode(package='lifecycle', executable='lifecycle_talker',
                      name='unsaid', namespace=''),
        LifecycleNode(package='lifecycle', executable='lifecycle_talker',
                      name='by_arg', namespace='',
                      autostart=LaunchConfiguration('go')),
        Node(package='demo_nodes_cpp', executable='talker', name='plain'),
    ])
"#,
    );
    assert_eq!(facts(&json, "on"), (Some(true), Some(true)));
    assert_eq!(facts(&json, "off"), (Some(true), Some(false)));
    assert_eq!(facts(&json, "unsaid"), (Some(true), None));
    assert_eq!(facts(&json, "by_arg"), (Some(true), Some(true)));
    assert_eq!(facts(&json, "plain"), (None, None));
}
