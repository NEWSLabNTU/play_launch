//! `respawn` / `respawn_delay` on `<executable>`, XML and YAML.
//!
//! Stock `ExecuteProcess` takes both, and `launch_xml`/`launch_yaml` read them
//! for `<executable>` as for `<node>`. The Rust parser had them on the
//! attribute allowlist and dropped them, so an `<executable respawn="true">`
//! that died stayed dead (found by play_launch's crash-loop test, W3 of
//! `docs/design/health-page.md`).
//!
//! The Python frontend dropped both for `Node`, `ExecuteProcess` and
//! `ComposableNodeContainer` (#0070): the mocks swallowed them into
//! `**kwargs` and the bridge wrote `respawn: None` for every capture.

use play_launch_parser::parse_launch_file;
use play_launch_parser_pyexec::python_test_guard;
use std::{collections::HashMap, io::Write};
use tempfile::NamedTempFile;

fn parse(suffix: &str, body: &str) -> serde_json::Value {
    let mut file = NamedTempFile::with_suffix(suffix).unwrap();
    file.write_all(body.as_bytes()).unwrap();
    file.flush().unwrap();
    let dump = parse_launch_file(file.path(), HashMap::new())
        .unwrap_or_else(|e| panic!("{suffix} should parse: {e}"));
    serde_json::to_value(dump).unwrap()
}

fn by_name<'a>(json: &'a serde_json::Value, name: &str) -> &'a serde_json::Value {
    json["node"]
        .as_array()
        .unwrap()
        .iter()
        .find(|e| e["name"] == name)
        .unwrap_or_else(|| panic!("{name} missing: {json}"))
}

#[test]
fn xml_executable_carries_respawn() {
    let json = parse(
        ".launch.xml",
        r#"<launch>
  <arg name="again" default="true"/>
  <executable cmd="sleep 1" name="respawned" respawn="$(var again)" respawn_delay="0.5"/>
  <executable cmd="sleep 1" name="plain"/>
</launch>"#,
    );
    let r = by_name(&json, "respawned");
    assert_eq!(r["respawn"], true, "{r}");
    assert_eq!(r["respawn_delay"], 0.5, "{r}");
    let p = by_name(&json, "plain");
    assert!(p["respawn"].is_null(), "absent stays absent: {p}");
}

#[test]
fn yaml_executable_carries_respawn() {
    let json = parse(
        ".launch.yaml",
        r#"launch:
- executable:
    cmd: sleep 1
    name: respawned
    respawn: "true"
    respawn_delay: "2.0"
"#,
    );
    let r = by_name(&json, "respawned");
    assert_eq!(r["respawn"], true, "{r}");
    assert_eq!(r["respawn_delay"], 2.0, "{r}");
}

#[test]
fn python_node_execute_process_and_container_carry_respawn() {
    let _guard = python_test_guard();
    let json = parse(
        ".launch.py",
        r#"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('again', default_value='true'),
        Node(package='demo_nodes_cpp', executable='talker', name='node_respawned',
             respawn=True, respawn_delay=0.5),
        Node(package='demo_nodes_cpp', executable='talker', name='node_off',
             respawn=False),
        Node(package='demo_nodes_cpp', executable='talker', name='node_plain'),
        ExecuteProcess(cmd=['sleep', '1'], name='proc_respawned',
                       respawn=LaunchConfiguration('again'), respawn_delay=2.0),
        ExecuteProcess(cmd=['sleep', '1'], name='proc_plain'),
        ComposableNodeContainer(package='rclcpp_components',
                                executable='component_container',
                                name='container_respawned', namespace='',
                                respawn=True, respawn_delay=1.5),
    ])
"#,
    );
    let r = by_name(&json, "node_respawned");
    assert_eq!(r["respawn"], true, "{r}");
    assert_eq!(r["respawn_delay"], 0.5, "{r}");
    // False and unset are both absent, as the Python dumper records them.
    for name in ["node_off", "node_plain", "proc_plain"] {
        let p = by_name(&json, name);
        assert!(p["respawn"].is_null(), "absent stays absent: {p}");
        assert!(p["respawn_delay"].is_null(), "{p}");
    }
    let r = by_name(&json, "proc_respawned");
    assert_eq!(r["respawn"], true, "a substitution is performed: {r}");
    assert_eq!(r["respawn_delay"], 2.0, "{r}");

    let containers = json["container"].as_array().unwrap();
    let c = containers
        .iter()
        .find(|c| c["name"] == "container_respawned")
        .unwrap_or_else(|| panic!("container missing: {json}"));
    assert_eq!(c["respawn"], true, "{c}");
    assert_eq!(c["respawn_delay"], 1.5, "{c}");
}

#[test]
fn python_respawn_launch_would_refuse_is_refused() {
    let _guard = python_test_guard();
    let mut file = NamedTempFile::with_suffix(".launch.py").unwrap();
    file.write_all(
        br#"
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['sleep', '1'], respawn='sometimes'),
    ])
"#,
    )
    .unwrap();
    file.flush().unwrap();
    let err = parse_launch_file(file.path(), HashMap::new())
        .expect_err("a respawn launch cannot coerce to bool must not parse");
    assert!(err.to_string().contains("respawn"), "{err}");
}
