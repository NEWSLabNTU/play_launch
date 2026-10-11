//! `on_exit="shutdown"` on the XML and YAML frontends (play_launch #0064).
//!
//! Stock `ExecuteProcess.parse` reads `on_exit` for `<node>`, `<executable>`
//! and `<node_container>` (all three reach it), accepts the literal
//! `shutdown` as `on_exit=[Shutdown()]`, and raises for any other value. The
//! Rust parser used to warn and drop the attribute, so a launch whose
//! required process exited kept running.

use play_launch_parser::parse_launch_file;
use std::{collections::HashMap, io::Write};
use tempfile::NamedTempFile;

fn write(suffix: &str, body: &str) -> NamedTempFile {
    let mut file = NamedTempFile::with_suffix(suffix).unwrap();
    file.write_all(body.as_bytes()).unwrap();
    file.flush().unwrap();
    file
}

fn parse(suffix: &str, body: &str) -> serde_json::Value {
    let file = write(suffix, body);
    let dump = parse_launch_file(file.path(), HashMap::new())
        .unwrap_or_else(|e| panic!("{suffix} should parse: {e}"));
    serde_json::to_value(dump).unwrap()
}

fn on_exit(entry: &serde_json::Value) -> Option<bool> {
    entry.get("on_exit_shutdown").and_then(|v| v.as_bool())
}

#[test]
fn xml_node_executable_and_container_carry_on_exit_shutdown() {
    let json = parse(
        ".launch.xml",
        r#"<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="required_node" on_exit="shutdown"/>
  <node pkg="demo_nodes_cpp" exec="listener" name="plain_node"/>
  <executable cmd="sleep 2" name="required_exec" on_exit="shutdown"/>
  <node_container pkg="rclcpp_components" exec="component_container"
                  name="required_container" namespace="" on_exit="shutdown"/>
</launch>"#,
    );
    let nodes = json["node"].as_array().unwrap();
    let by_name = |n: &str| {
        nodes
            .iter()
            .find(|e| e["name"] == n)
            .unwrap_or_else(|| panic!("{n} missing: {nodes:?}"))
    };
    assert_eq!(on_exit(by_name("required_node")), Some(true));
    assert_eq!(on_exit(by_name("required_exec")), Some(true));
    // Absent stays absent: a node without the attribute is not required.
    assert_eq!(on_exit(by_name("plain_node")), None);

    let containers = json["container"].as_array().unwrap();
    assert_eq!(containers.len(), 1, "{containers:?}");
    assert_eq!(on_exit(&containers[0]), Some(true));
}

#[test]
fn yaml_node_executable_and_container_carry_on_exit_shutdown() {
    let json = parse(
        ".launch.yaml",
        r#"launch:
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: required_node
    on_exit: shutdown
- executable:
    cmd: sleep 2
    name: required_exec
    on_exit: shutdown
- node_container:
    pkg: rclcpp_components
    exec: component_container
    name: required_container
    namespace: ""
    on_exit: shutdown
"#,
    );
    let nodes = json["node"].as_array().unwrap();
    assert!(
        nodes.iter().all(|n| on_exit(n) == Some(true)) && nodes.len() == 2,
        "{nodes:?}"
    );
    let containers = json["container"].as_array().unwrap();
    assert_eq!(containers.len(), 1, "{containers:?}");
    assert_eq!(on_exit(&containers[0]), Some(true));
}

/// Stock launch raises for any value but `shutdown`; so do we, with its words.
#[test]
fn any_other_on_exit_value_is_refused_as_stock_launch_does() {
    for (suffix, body) in [
        (
            ".launch.xml",
            r#"<launch><node pkg="demo_nodes_cpp" exec="talker" on_exit="restart"/></launch>"#,
        ),
        (
            ".launch.yaml",
            "launch:\n- executable:\n    cmd: sleep 2\n    on_exit: restart\n",
        ),
    ] {
        let file = write(suffix, body);
        let err = parse_launch_file(file.path(), HashMap::new())
            .expect_err("on_exit=restart must be refused")
            .to_string();
        assert!(
            err.contains("expected to be shutdown but got `restart`"),
            "{suffix}: {err}"
        );
    }
}
