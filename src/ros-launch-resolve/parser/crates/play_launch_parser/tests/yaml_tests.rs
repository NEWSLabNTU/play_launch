use play_launch_parser::parse_launch_file;
use std::{collections::HashMap, io::Write, path::PathBuf};
use tempfile::NamedTempFile;

/// Helper to get fixture path from crate tests directory
fn get_fixture_path(filename: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/launch")
        .join(filename)
}

/// Helper: write YAML to a temp .launch.yaml file
fn write_yaml(yaml: &str) -> NamedTempFile {
    let mut file = NamedTempFile::with_suffix(".launch.yaml").unwrap();
    file.write_all(yaml.as_bytes()).unwrap();
    file.flush().unwrap();
    file
}

#[test]
fn test_yaml_standalone_node() {
    let file = write_yaml(
        r#"launch:
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: my_talker
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "YAML node should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "my_talker");
    assert_eq!(nodes[0]["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(nodes[0]["executable"].as_str().unwrap(), "talker");
}

#[test]
fn test_yaml_node_with_namespace() {
    let file = write_yaml(
        r#"launch:
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: my_talker
    namespace: /robot1
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["namespace"].as_str().unwrap(), "/robot1");
}

#[test]
fn test_yaml_node_with_params() {
    let file = write_yaml(
        r#"launch:
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: my_talker
    param:
    - name: frequency
      value: "10.0"
    - name: topic_name
      value: /chatter
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);

    let params = nodes[0]["params"].as_array().unwrap();
    assert_eq!(params.len(), 2, "Should have 2 parameters");

    let freq = params
        .iter()
        .find(|p| p[0].as_str() == Some("frequency"))
        .unwrap();
    assert_eq!(freq[1].as_str().unwrap(), "10.0");

    let topic = params
        .iter()
        .find(|p| p[0].as_str() == Some("topic_name"))
        .unwrap();
    assert_eq!(topic[1].as_str().unwrap(), "/chatter");
}

#[test]
fn test_yaml_node_with_non_string_param_values() {
    // YAML booleans and numbers need conversion via value_to_string
    let file = write_yaml(
        r#"launch:
- node:
    pkg: my_pkg
    exec: my_node
    name: test_node
    param:
    - name: enabled
      value: true
    - name: count
      value: 42
    - name: rate
      value: 5.0
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);

    let params = nodes[0]["params"].as_array().unwrap();
    assert_eq!(params.len(), 3);

    // Bool "true" may be capitalized to "True" by the record generator (ROS convention)
    let enabled = params
        .iter()
        .find(|p| p[0].as_str() == Some("enabled"))
        .unwrap();
    let enabled_val = enabled[1].as_str().unwrap().to_lowercase();
    assert_eq!(enabled_val, "true");

    let count = params
        .iter()
        .find(|p| p[0].as_str() == Some("count"))
        .unwrap();
    assert_eq!(count[1].as_str().unwrap(), "42");

    let rate = params
        .iter()
        .find(|p| p[0].as_str() == Some("rate"))
        .unwrap();
    assert_eq!(rate[1].as_str().unwrap(), "5.0");
}

#[test]
fn test_yaml_node_with_remaps() {
    let file = write_yaml(
        r#"launch:
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: my_talker
    remap:
    - from: chatter
      to: /remapped_chatter
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);

    let remaps = nodes[0]["remaps"].as_array().unwrap();
    assert_eq!(remaps.len(), 1);
    assert_eq!(remaps[0][0].as_str().unwrap(), "chatter");
    assert_eq!(remaps[0][1].as_str().unwrap(), "/remapped_chatter");
}

#[test]
fn test_yaml_arg_declaration() {
    let file = write_yaml(
        r#"launch:
- arg:
    name: my_param
    default: hello_world
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: "$(var my_param)"
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "hello_world");
}

#[test]
fn test_yaml_arg_override() {
    let file = write_yaml(
        r#"launch:
- arg:
    name: mode
    default: default_mode
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: "$(var mode)"
"#,
    );

    let mut args = HashMap::new();
    args.insert("mode".to_string(), "override_mode".to_string());

    let json = serde_json::to_value(parse_launch_file(file.path(), args).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "override_mode");
}

#[test]
fn test_yaml_let_action() {
    let file = write_yaml(
        r#"launch:
- arg:
    name: base
    default: hello
- let:
    name: derived
    value: "$(var base)_world"
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: "$(var derived)"
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "hello_world");
}

#[test]
fn test_yaml_let_sequential_resolution() {
    let file = write_yaml(
        r#"launch:
- let:
    name: var1
    value: first
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: "$(var var1)"
- let:
    name: var1
    value: second
- node:
    pkg: demo_nodes_cpp
    exec: listener
    name: "$(var var1)"
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 2);

    let talker = nodes
        .iter()
        .find(|n| n["executable"].as_str() == Some("talker"))
        .unwrap();
    assert_eq!(talker["name"].as_str().unwrap(), "first");

    let listener = nodes
        .iter()
        .find(|n| n["executable"].as_str() == Some("listener"))
        .unwrap();
    assert_eq!(listener["name"].as_str().unwrap(), "second");
}

#[test]
fn test_yaml_group_with_namespace() {
    let file = write_yaml(
        r#"launch:
- group:
    ns: /robot1
    children:
    - node:
        pkg: demo_nodes_cpp
        exec: talker
        name: talker1
- node:
    pkg: demo_nodes_cpp
    exec: listener
    name: listener1
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 2);

    let talker = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("talker1"))
        .unwrap();
    assert_eq!(talker["namespace"].as_str().unwrap(), "/robot1");

    let listener = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("listener1"))
        .unwrap();
    assert!(
        listener["namespace"].is_null(),
        "listener1 outside group should have null namespace"
    );
}

#[test]
fn test_yaml_group_if_condition_true() {
    let file = write_yaml(
        r#"launch:
- arg:
    name: use_sim
    default: "true"
- group:
    if: "$(var use_sim)"
    children:
    - node:
        pkg: sim_pkg
        exec: sim_node
        name: sim_node
- group:
    unless: "$(var use_sim)"
    children:
    - node:
        pkg: real_pkg
        exec: real_node
        name: real_node
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "sim_node");
}

#[test]
fn test_yaml_group_if_condition_false() {
    let file = write_yaml(
        r#"launch:
- arg:
    name: use_sim
    default: "true"
- group:
    if: "$(var use_sim)"
    children:
    - node:
        pkg: sim_pkg
        exec: sim_node
        name: sim_node
- group:
    unless: "$(var use_sim)"
    children:
    - node:
        pkg: real_pkg
        exec: real_node
        name: real_node
"#,
    );

    let mut args = HashMap::new();
    args.insert("use_sim".to_string(), "false".to_string());

    let json = serde_json::to_value(parse_launch_file(file.path(), args).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "real_node");
}

#[test]
fn test_yaml_node_if_condition() {
    // if/unless on individual nodes, not just groups
    let file = write_yaml(
        r#"launch:
- arg:
    name: debug
    default: "false"
- node:
    if: "$(var debug)"
    pkg: debug_pkg
    exec: debug_node
    name: debug_node
- node:
    pkg: main_pkg
    exec: main_node
    name: main_node
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(
        nodes.len(),
        1,
        "Only main_node should be present with debug=false"
    );
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "main_node");
}

#[test]
fn test_yaml_set_env() {
    let file = write_yaml(
        r#"launch:
- set_env:
    name: MY_YAML_VAR
    value: yaml_value
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: test_node
"#,
    );

    // set_env modifies context but doesn't directly appear in node output
    // Just verify parsing succeeds
    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "YAML set_env should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
}

#[test]
fn test_yaml_unset_env() {
    let file = write_yaml(
        r#"launch:
- unset_env:
    name: NONEXISTENT_VAR
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: test_node
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "YAML unset_env should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
}

#[test]
fn test_yaml_push_ros_namespace() {
    let file = write_yaml(
        r#"launch:
- push-ros-namespace:
    namespace: /sensors
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: camera
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["namespace"].as_str().unwrap(), "/sensors");
}

#[test]
fn test_yaml_push_ros_namespace_ns_key() {
    // Test with the "ns" key instead of "namespace"
    let file = write_yaml(
        r#"launch:
- push-ros-namespace:
    ns: /alt_namespace
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: test_node
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["namespace"].as_str().unwrap(), "/alt_namespace");
}

#[test]
fn test_yaml_include_xml() {
    // Create an XML file to include
    let mut xml_file = NamedTempFile::with_suffix(".launch.xml").unwrap();
    xml_file
        .write_all(
            br#"<launch>
    <node pkg="demo_nodes_cpp" exec="talker" name="included_talker" />
</launch>"#,
        )
        .unwrap();
    xml_file.flush().unwrap();
    let xml_path = xml_file.path().to_str().unwrap().to_string();

    let file = write_yaml(&format!(
        r#"launch:
- include:
    file: "{}"
"#,
        xml_path
    ));

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "included_talker");
}

#[test]
fn test_yaml_include_with_args() {
    // Create an XML file that uses an arg
    let mut xml_file = NamedTempFile::with_suffix(".launch.xml").unwrap();
    xml_file
        .write_all(
            br#"<launch>
    <arg name="node_name" default="default_name" />
    <node pkg="demo_nodes_cpp" exec="talker" name="$(var node_name)" />
</launch>"#,
        )
        .unwrap();
    xml_file.flush().unwrap();
    let xml_path = xml_file.path().to_str().unwrap().to_string();

    let file = write_yaml(&format!(
        r#"launch:
- include:
    file: "{}"
    arg:
    - name: node_name
      value: custom_name
"#,
        xml_path
    ));

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "custom_name");
}

#[test]
fn test_yaml_include_yaml() {
    // YAML including another YAML
    let inner = write_yaml(
        r#"launch:
- arg:
    name: inner_var
    default: inner_default
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: "$(var inner_var)"
"#,
    );
    let inner_path = inner.path().to_str().unwrap().to_string();

    let outer = write_yaml(&format!(
        r#"launch:
- include:
    file: "{}"
"#,
        inner_path
    ));

    let json =
        serde_json::to_value(parse_launch_file(outer.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "inner_default");
}

#[test]
fn test_yaml_parent_scope_modification() {
    // YAML modifies parent scope — variables declared in YAML are visible after include
    let yaml_preset = write_yaml(
        r#"launch:
- arg:
    name: preset_var
    default: preset_value
"#,
    );
    let yaml_path = yaml_preset.path().to_str().unwrap().to_string();

    // XML includes YAML preset, then uses the variable
    let mut xml_file = NamedTempFile::with_suffix(".launch.xml").unwrap();
    xml_file
        .write_all(
            format!(
                r#"<launch>
    <include file="{}" />
    <node pkg="demo_nodes_cpp" exec="talker" name="$(var preset_var)" />
</launch>"#,
                yaml_path
            )
            .as_bytes(),
        )
        .unwrap();
    xml_file.flush().unwrap();

    let json =
        serde_json::to_value(parse_launch_file(xml_file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(
        nodes[0]["name"].as_str().unwrap(),
        "preset_value",
        "Variable from YAML preset should be visible in parent XML scope"
    );
}

#[test]
fn test_yaml_group_namespace_scope_isolation() {
    // Group namespace scoping: namespace inside group should not leak outside
    let file = write_yaml(
        r#"launch:
- group:
    ns: /scoped_ns
    children:
    - node:
        pkg: demo_nodes_cpp
        exec: talker
        name: inside_group
- node:
    pkg: demo_nodes_cpp
    exec: listener
    name: outside_group
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 2);

    let inside = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("inside_group"))
        .unwrap();
    assert_eq!(
        inside["namespace"].as_str().unwrap(),
        "/scoped_ns",
        "Inside group should have group namespace"
    );

    let outside = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("outside_group"))
        .unwrap();
    assert!(
        outside["namespace"].is_null(),
        "Outside group should have null namespace (scope restored)"
    );
}

#[test]
fn test_yaml_multiple_nodes() {
    let file = write_yaml(
        r#"launch:
- node:
    pkg: pkg_a
    exec: node_a
    name: node_a
- node:
    pkg: pkg_b
    exec: node_b
    name: node_b
- node:
    pkg: pkg_c
    exec: node_c
    name: node_c
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 3);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "node_a");
    assert_eq!(nodes[1]["name"].as_str().unwrap(), "node_b");
    assert_eq!(nodes[2]["name"].as_str().unwrap(), "node_c");
}

#[test]
fn test_yaml_combined_actions() {
    // Test a YAML file with multiple action types interleaved
    let file = write_yaml(
        r#"launch:
- arg:
    name: ns
    default: my_ns
- let:
    name: exec_name
    value: talker
- push-ros-namespace:
    namespace: "/$(var ns)"
- node:
    pkg: demo_nodes_cpp
    exec: "$(var exec_name)"
    name: combined_node
    param:
    - name: rate
      value: 10
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "combined_node");
    assert_eq!(nodes[0]["namespace"].as_str().unwrap(), "/my_ns");
    assert_eq!(nodes[0]["executable"].as_str().unwrap(), "talker");

    let params = nodes[0]["params"].as_array().unwrap();
    let rate = params
        .iter()
        .find(|p| p[0].as_str() == Some("rate"))
        .unwrap();
    assert_eq!(rate[1].as_str().unwrap(), "10");
}

#[test]
fn test_yaml_nested_groups() {
    // Use relative namespaces so they stack (absolute namespaces replace the stack)
    let file = write_yaml(
        r#"launch:
- group:
    ns: level1
    children:
    - group:
        ns: level2
        children:
        - node:
            pkg: demo_nodes_cpp
            exec: talker
            name: deep_node
    - node:
        pkg: demo_nodes_cpp
        exec: listener
        name: mid_node
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: root_node
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 3);

    let deep = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("deep_node"))
        .unwrap();
    assert_eq!(deep["namespace"].as_str().unwrap(), "/level1/level2");

    let mid = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("mid_node"))
        .unwrap();
    assert_eq!(mid["namespace"].as_str().unwrap(), "/level1");

    let root = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("root_node"))
        .unwrap();
    assert!(root["namespace"].is_null());
}

#[test]
fn test_yaml_no_launch_key() {
    // YAML file without "launch:" key should produce empty result
    let file = write_yaml(
        r#"some_other_key:
  - value: test
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 0);
}

#[test]
fn test_yaml_node_output_and_respawn() {
    let file = write_yaml(
        r#"launch:
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: respawning_node
    output: screen
    respawn: "true"
    respawn_delay: "2.0"
"#,
    );

    let json =
        serde_json::to_value(parse_launch_file(file.path(), HashMap::new()).unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "respawning_node");
}

#[test]
fn test_yaml_hyphenated_action_variants() {
    // Test set-env and unset-env hyphenated variants
    let file = write_yaml(
        r#"launch:
- set-env:
    name: HYPHEN_VAR
    value: hyphen_value
- unset-env:
    name: HYPHEN_VAR
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: test_node
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "Hyphenated env actions should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
}

// =============================================================================
// YAML set_parameter tests
// =============================================================================

#[test]
fn test_yaml_set_parameter() {
    let file = write_yaml(
        r#"launch:
- set_parameter:
    name: use_sim_time
    value: "true"
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: my_node
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "set_parameter should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);

    // Global parameters should be applied to the node
    let node = &nodes[0];
    let global_params = node["global_params"].as_array();
    assert!(
        global_params.is_some(),
        "Node should have global_params from set_parameter"
    );
    let params = global_params.unwrap();
    // Note: YAML "true" string is capitalized to "True" by the parameter system
    let has_sim_time = params.iter().any(|p| {
        p[0].as_str() == Some("use_sim_time")
            && p[1]
                .as_str()
                .is_some_and(|v| v.eq_ignore_ascii_case("true"))
    });
    assert!(has_sim_time, "Should have use_sim_time=true global param");
}

#[test]
fn test_yaml_set_parameter_with_condition() {
    let file = write_yaml(
        r#"launch:
- arg:
    name: use_sim
    default: "false"
- set_parameter:
    if: "$(var use_sim)"
    name: use_sim_time
    value: "true"
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: my_node
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "Conditional set_parameter should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    let node = &nodes[0];

    // use_sim is false, so set_parameter should be skipped
    let global_params = node.get("global_params");
    let has_sim_time = global_params
        .and_then(|p| p.as_array())
        .map(|p| p.iter().any(|x| x[0].as_str() == Some("use_sim_time")))
        .unwrap_or(false);
    assert!(
        !has_sim_time,
        "Should not have use_sim_time when condition is false"
    );
}

#[test]
fn test_yaml_set_parameter_hyphenated() {
    let file = write_yaml(
        r#"launch:
- set-parameter:
    name: my_param
    value: "42"
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: test_node
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "Hyphenated set-parameter should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    let global_params = nodes[0]["global_params"].as_array().unwrap();
    let has_param = global_params
        .iter()
        .any(|p| p[0].as_str() == Some("my_param") && p[1].as_str() == Some("42"));
    assert!(has_param, "Should have my_param=42");
}

// =============================================================================
// YAML set_remap tests
// =============================================================================

#[test]
fn test_yaml_set_remap() {
    let file = write_yaml(
        r#"launch:
- set_remap:
    from: /input
    to: /remapped_input
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: my_node
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "set_remap should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);

    // Check the node's cmd contains the remap
    let cmd = nodes[0]["cmd"].as_array().unwrap();
    let cmd_strs: Vec<&str> = cmd.iter().filter_map(|v| v.as_str()).collect();
    let cmd_str = cmd_strs.join(" ");
    assert!(
        cmd_str.contains("/input:=/remapped_input"),
        "CMD should contain remap: {}",
        cmd_str
    );
}

#[test]
fn test_yaml_set_remap_hyphenated() {
    let file = write_yaml(
        r#"launch:
- set-remap:
    from: /old_topic
    to: /new_topic
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: test_node
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "Hyphenated set-remap should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);

    let cmd = nodes[0]["cmd"].as_array().unwrap();
    let cmd_str: String = cmd
        .iter()
        .filter_map(|v| v.as_str())
        .collect::<Vec<_>>()
        .join(" ");
    assert!(
        cmd_str.contains("/old_topic:=/new_topic"),
        "CMD should contain remap: {}",
        cmd_str
    );
}

// =============================================================================
// YAML executable tests
// =============================================================================

#[test]
fn test_yaml_executable() {
    let file = write_yaml(
        r#"launch:
- executable:
    cmd: "ros2 bag record -a"
    name: rosbag_recorder
    output: screen
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "executable should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Executable should produce a node record");

    let node = &nodes[0];
    // Executable records go to node array
    let cmd = node["cmd"].as_array().unwrap();
    let cmd_strs: Vec<&str> = cmd.iter().filter_map(|v| v.as_str()).collect();
    // cmd should contain the executable command
    assert!(!cmd_strs.is_empty(), "CMD should not be empty");
}

#[test]
fn test_yaml_executable_with_arg_children() {
    // Rust-only extension (also live on the XML path via the `executable-arg`
    // spec in `xml/attr_spec.rs`): `arg:` under `executable:` supplies
    // positional arguments appended to `cmd`. `validate_yaml_keys` used to
    // reject this — `executable`'s spec `children` list only had `env`, not
    // `arg`, even though `process_yaml_executable` (traverser/yaml.rs) has
    // always read `arg:` and turned it into `ExecutableAction::arguments`.
    // No test covered this combination, which is how the mismatch between
    // the reader and the allowlist survived.
    let file = write_yaml(
        r#"launch:
- executable:
    cmd: "ros2 bag record"
    arg:
    - value: "-a"
    - value: "-o"
    - value: "/tmp/bag"
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "executable with arg: children should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Executable should produce a node record");

    let cmd = nodes[0]["cmd"].as_array().unwrap();
    let cmd_strs: Vec<&str> = cmd.iter().filter_map(|v| v.as_str()).collect();
    assert_eq!(
        cmd_strs,
        vec!["ros2 bag record", "-a", "-o", "/tmp/bag"],
        "arg: children should be appended to cmd in order"
    );
}

#[test]
fn test_yaml_executable_with_condition() {
    let file = write_yaml(
        r#"launch:
- arg:
    name: record_bag
    default: "false"
- executable:
    if: "$(var record_bag)"
    cmd: "ros2 bag record -a"
    name: rosbag_recorder
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "Conditional executable should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(
        nodes.len(),
        0,
        "Executable should be skipped when condition is false"
    );
}

// =============================================================================
// YAML node_container tests
// =============================================================================

#[test]
fn test_yaml_node_container_basic() {
    let file = write_yaml(
        r#"launch:
- node_container:
    name: my_container
    namespace: /test
    composable_node:
    - pkg: my_pkg
      plugin: "my_pkg::MyComponent"
      name: my_component
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "node_container should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();

    // Check container record
    let containers = json["container"].as_array().unwrap();
    assert_eq!(containers.len(), 1, "Should have one container");
    let container = &containers[0];
    assert_eq!(container["name"].as_str(), Some("my_container"));
    assert_eq!(container["namespace"].as_str(), Some("/test"));
    assert_eq!(container["package"].as_str(), Some("rclcpp_components"));
    assert_eq!(
        container["executable"].as_str(),
        Some("component_container")
    );

    // Check load_node record
    let load_nodes = json["load_node"].as_array().unwrap();
    assert_eq!(load_nodes.len(), 1, "Should have one load_node");
    let load = &load_nodes[0];
    assert_eq!(load["node_name"].as_str(), Some("my_component"));
    assert_eq!(load["package"].as_str(), Some("my_pkg"));
    assert_eq!(load["plugin"].as_str(), Some("my_pkg::MyComponent"));
}

#[test]
fn test_yaml_node_container_with_params_and_remaps() {
    let file = write_yaml(
        r#"launch:
- node_container:
    name: sensor_container
    namespace: /sensing
    pkg: rclcpp_components
    exec: component_container_mt
    composable_node:
    - pkg: sensor_pkg
      plugin: "sensor_pkg::PointcloudFilter"
      name: filter_node
      namespace: /sensing
      param:
      - name: threshold
        value: "0.5"
      - name: enabled
        value: true
      remap:
      - from: input_cloud
        to: /lidar/points
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "node_container with params should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();

    let containers = json["container"].as_array().unwrap();
    assert_eq!(containers.len(), 1);
    assert_eq!(
        containers[0]["executable"].as_str(),
        Some("component_container_mt")
    );

    let load_nodes = json["load_node"].as_array().unwrap();
    assert_eq!(load_nodes.len(), 1);
    let load = &load_nodes[0];
    assert_eq!(load["node_name"].as_str(), Some("filter_node"));

    // Check params
    let params = load["params"].as_array().unwrap();
    let has_threshold = params
        .iter()
        .any(|p| p[0].as_str() == Some("threshold") && p[1].as_str() == Some("0.5"));
    assert!(has_threshold, "Should have threshold=0.5 param");

    // Check remaps
    let remaps = load["remaps"].as_array().unwrap();
    let has_remap = remaps
        .iter()
        .any(|r| r[0].as_str() == Some("input_cloud") && r[1].as_str() == Some("/lidar/points"));
    assert!(has_remap, "Should have input_cloud -> /lidar/points remap");
}

#[test]
fn test_yaml_node_container_hyphenated() {
    let file = write_yaml(
        r#"launch:
- node-container:
    name: my_container
    namespace: /ns
    composable_node:
    - pkg: my_pkg
      plugin: "my_pkg::MyNode"
      name: my_node
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "Hyphenated node-container should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let containers = json["container"].as_array().unwrap();
    assert_eq!(containers.len(), 1);
}

#[test]
fn test_yaml_node_container_multiple_composable_nodes() {
    let file = write_yaml(
        r#"launch:
- node_container:
    name: processing_container
    namespace: /processing
    composable_node:
    - pkg: pkg_a
      plugin: "pkg_a::NodeA"
      name: node_a
    - pkg: pkg_b
      plugin: "pkg_b::NodeB"
      name: node_b
    - pkg: pkg_c
      plugin: "pkg_c::NodeC"
      name: node_c
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "Container with multiple composable nodes should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();

    let containers = json["container"].as_array().unwrap();
    assert_eq!(containers.len(), 1);

    let load_nodes = json["load_node"].as_array().unwrap();
    assert_eq!(load_nodes.len(), 3, "Should have 3 load_node records");

    let names: Vec<&str> = load_nodes
        .iter()
        .filter_map(|l| l["node_name"].as_str())
        .collect();
    assert!(names.contains(&"node_a"));
    assert!(names.contains(&"node_b"));
    assert!(names.contains(&"node_c"));
}

// =============================================================================
// YAML load_composable_node tests
// =============================================================================

#[test]
fn test_yaml_load_composable_node() {
    // load_composable_node loads into an existing container
    // First create a container, then load a node into it
    let file = write_yaml(
        r#"launch:
- node_container:
    name: my_container
    namespace: /test
- load_composable_node:
    target: /test/my_container
    composable_node:
    - pkg: my_pkg
      plugin: "my_pkg::LazyLoader"
      name: lazy_node
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "load_composable_node should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();

    let containers = json["container"].as_array().unwrap();
    assert_eq!(containers.len(), 1, "Should have one container");

    let load_nodes = json["load_node"].as_array().unwrap();
    assert!(
        !load_nodes.is_empty(),
        "Should have at least one load_node from load_composable_node"
    );

    let has_lazy = load_nodes.iter().any(|l| {
        l["node_name"].as_str() == Some("lazy_node")
            && l["plugin"].as_str() == Some("my_pkg::LazyLoader")
    });
    assert!(has_lazy, "Should have lazy_node load_node record");
}

#[test]
fn test_yaml_load_composable_node_hyphenated() {
    let file = write_yaml(
        r#"launch:
- node_container:
    name: container
    namespace: /ns
- load-composable-node:
    target: /ns/container
    composable_node:
    - pkg: my_pkg
      plugin: "my_pkg::Node"
      name: my_node
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "Hyphenated load-composable-node should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let load_nodes = json["load_node"].as_array().unwrap();
    let has_node = load_nodes
        .iter()
        .any(|l| l["node_name"].as_str() == Some("my_node"));
    assert!(has_node, "Should have load_node from hyphenated action");
}

#[test]
fn test_yaml_load_composable_node_with_params() {
    let file = write_yaml(
        r#"launch:
- node_container:
    name: my_container
    namespace: /test
- load_composable_node:
    target: /test/my_container
    composable_node:
    - pkg: perception_pkg
      plugin: "perception_pkg::Detector"
      name: detector
      param:
      - name: confidence
        value: "0.8"
      - name: max_objects
        value: "100"
      remap:
      - from: image_raw
        to: /camera/image
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "load_composable_node with params should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let load_nodes = json["load_node"].as_array().unwrap();

    let detector = load_nodes
        .iter()
        .find(|l| l["node_name"].as_str() == Some("detector"));
    assert!(detector.is_some(), "Should have detector load_node");
    let det = detector.unwrap();

    // Check params
    let params = det["params"].as_array().unwrap();
    let has_confidence = params
        .iter()
        .any(|p| p[0].as_str() == Some("confidence") && p[1].as_str() == Some("0.8"));
    assert!(has_confidence, "Should have confidence=0.8 param");

    // Check remaps
    let remaps = det["remaps"].as_array().unwrap();
    let has_remap = remaps
        .iter()
        .any(|r| r[0].as_str() == Some("image_raw") && r[1].as_str() == Some("/camera/image"));
    assert!(has_remap, "Should have image_raw -> /camera/image remap");
}

// =============================================================================
// YAML fixture-based tests (28.1 + 28.2)
// =============================================================================

#[test]
fn test_yaml_fixture_all_actions() {
    let fixture = get_fixture_path("test_yaml_all_actions.launch.yaml");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(result.is_ok(), "Should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(
        nodes.len(),
        2,
        "Should have 2 nodes (sensor_node + main_node)"
    );

    let sensor = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("sensor_node"))
        .expect("Should have sensor_node");
    assert_eq!(
        sensor["namespace"].as_str().unwrap(),
        "/my_robot/sensors",
        "sensor_node should be in /my_robot/sensors namespace"
    );
    assert_eq!(sensor["executable"].as_str().unwrap(), "talker");

    // Check sensor_node params
    let params = sensor["params"].as_array().unwrap();
    let has_rate = params
        .iter()
        .any(|p| p[0].as_str() == Some("rate") && p[1].as_str() == Some("10.0"));
    assert!(has_rate, "sensor_node should have rate=10.0");

    // Check global params (from set_parameter)
    let global_params = sensor["global_params"].as_array();
    assert!(global_params.is_some(), "Should have global_params");
    let has_sim_time = global_params.unwrap().iter().any(|p| {
        p[0].as_str() == Some("use_sim_time")
            && p[1]
                .as_str()
                .is_some_and(|v| v.eq_ignore_ascii_case("true"))
    });
    assert!(has_sim_time, "Should have use_sim_time global param");

    let main = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("main_node"))
        .expect("Should have main_node");
    assert_eq!(
        main["namespace"].as_str().unwrap(),
        "/my_robot",
        "main_node should be in /my_robot namespace"
    );

    // Check set_remap is in cmd
    let cmd = main["cmd"].as_array().unwrap();
    let cmd_str: String = cmd
        .iter()
        .filter_map(|v| v.as_str())
        .collect::<Vec<_>>()
        .join(" ");
    assert!(
        cmd_str.contains("/input_topic:=/remapped_topic"),
        "CMD should contain remap: {}",
        cmd_str
    );
}

#[test]
fn test_yaml_fixture_container() {
    let fixture = get_fixture_path("test_yaml_container.launch.yaml");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(result.is_ok(), "Should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();

    let containers = json["container"].as_array().unwrap();
    assert_eq!(containers.len(), 1);
    assert_eq!(containers[0]["name"].as_str(), Some("test_container"));
    assert_eq!(containers[0]["namespace"].as_str(), Some("/container_ns"));
    assert_eq!(
        containers[0]["executable"].as_str(),
        Some("component_container")
    );

    let load_nodes = json["load_node"].as_array().unwrap();
    assert_eq!(load_nodes.len(), 2, "Should have 2 composable nodes");

    let talker = load_nodes
        .iter()
        .find(|l| l["node_name"].as_str() == Some("comp_talker"))
        .expect("Should have comp_talker");
    assert_eq!(talker["plugin"].as_str(), Some("demo_nodes_cpp::Talker"));

    let params = talker["params"].as_array().unwrap();
    let has_freq = params
        .iter()
        .any(|p| p[0].as_str() == Some("frequency") && p[1].as_str() == Some("5.0"));
    assert!(has_freq, "comp_talker should have frequency=5.0");

    let remaps = talker["remaps"].as_array().unwrap();
    let has_remap = remaps
        .iter()
        .any(|r| r[0].as_str() == Some("chatter") && r[1].as_str() == Some("/remapped_chatter"));
    assert!(has_remap, "comp_talker should have chatter remap");

    let listener = load_nodes
        .iter()
        .find(|l| l["node_name"].as_str() == Some("comp_listener"));
    assert!(listener.is_some(), "Should have comp_listener");
}

#[test]
fn test_yaml_fixture_load_composable_node() {
    let fixture = get_fixture_path("test_yaml_load_composable_node.launch.yaml");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(result.is_ok(), "Should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();

    let containers = json["container"].as_array().unwrap();
    assert_eq!(containers.len(), 1);

    let load_nodes = json["load_node"].as_array().unwrap();
    let lazy = load_nodes
        .iter()
        .find(|l| l["node_name"].as_str() == Some("lazy_talker"));
    assert!(lazy.is_some(), "Should have lazy_talker");
    let lazy = lazy.unwrap();
    assert_eq!(lazy["plugin"].as_str(), Some("demo_nodes_cpp::Talker"));
    assert_eq!(
        lazy["target_container_name"].as_str(),
        Some("/test/my_container")
    );

    let params = lazy["params"].as_array().unwrap();
    let has_freq = params
        .iter()
        .any(|p| p[0].as_str() == Some("frequency") && p[1].as_str() == Some("2.0"));
    assert!(has_freq, "lazy_talker should have frequency=2.0");
}

#[test]
fn test_yaml_fixture_executable() {
    let fixture = get_fixture_path("test_yaml_executable.launch.yaml");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(result.is_ok(), "Should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Executable should produce a node record");

    let node = &nodes[0];
    let cmd = node["cmd"].as_array().unwrap();
    let cmd_strs: Vec<&str> = cmd.iter().filter_map(|v| v.as_str()).collect();
    assert!(!cmd_strs.is_empty(), "CMD should not be empty");
}

#[test]
fn test_yaml_fixture_preset_pattern() {
    // Autoware-style preset: XML includes YAML preset, then uses variables from it
    let fixture_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/launch");
    let preset_path = fixture_dir
        .join("test_yaml_preset.launch.yaml")
        .to_str()
        .unwrap()
        .to_string();

    // Create inline XML that includes the YAML preset
    let mut xml_file = NamedTempFile::with_suffix(".launch.xml").unwrap();
    xml_file
        .write_all(
            format!(
                r#"<launch>
    <include file="{}" />
    <node pkg="demo_nodes_cpp" exec="talker" name="$(var velocity_smoother_type)" />
</launch>"#,
                preset_path
            )
            .as_bytes(),
        )
        .unwrap();
    xml_file.flush().unwrap();

    let result = parse_launch_file(xml_file.path(), HashMap::new());
    assert!(result.is_ok(), "Should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(
        nodes[0]["name"].as_str().unwrap(),
        "JointTrajectory",
        "Variable from YAML preset should resolve in parent XML scope"
    );
}

#[test]
fn test_yaml_fixture_conditions() {
    let fixture = get_fixture_path("test_yaml_conditions.launch.yaml");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    // Default: use_sim=true, enable_debug=false
    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(result.is_ok(), "Should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 2, "Should have sim_talker + main_listener");

    let has_sim = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("sim_talker"));
    assert!(has_sim, "Should have sim_talker when use_sim=true");

    let has_real = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("real_talker"));
    assert!(!has_real, "Should not have real_talker when use_sim=true");

    let has_debug = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("debug_listener"));
    assert!(
        !has_debug,
        "Should not have debug_listener when enable_debug=false"
    );

    // Override: use_sim=false, enable_debug=true
    let mut args = HashMap::new();
    args.insert("use_sim".to_string(), "false".to_string());
    args.insert("enable_debug".to_string(), "true".to_string());

    let result2 = parse_launch_file(&fixture, args);
    assert!(
        result2.is_ok(),
        "Should parse with overrides: {:?}",
        result2.err()
    );

    let json2 = serde_json::to_value(result2.unwrap()).unwrap();
    let nodes2 = json2["node"].as_array().unwrap();
    assert_eq!(
        nodes2.len(),
        3,
        "Should have real_talker + debug_listener + main_listener"
    );

    let has_real2 = nodes2
        .iter()
        .any(|n| n["name"].as_str() == Some("real_talker"));
    assert!(has_real2, "Should have real_talker when use_sim=false");

    let has_debug2 = nodes2
        .iter()
        .any(|n| n["name"].as_str() == Some("debug_listener"));
    assert!(
        has_debug2,
        "Should have debug_listener when enable_debug=true"
    );
}

#[test]
fn test_yaml_fixture_dirname() {
    let fixture = get_fixture_path("test_yaml_nested_groups.launch.yaml");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(result.is_ok(), "Should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 3);

    let deep = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("deep_node"))
        .expect("Should have deep_node");
    assert_eq!(deep["namespace"].as_str().unwrap(), "/level1/level2");

    // Check dirname resolves to fixtures dir, not /tmp
    let params = deep["params"].as_array().unwrap();
    let config_path = params
        .iter()
        .find(|p| p[0].as_str() == Some("config_path"))
        .expect("Should have config_path param");
    let value = config_path[1].as_str().unwrap();
    assert!(
        value.contains("tests/fixtures/launch"),
        "$(dirname) should resolve to fixtures dir, got: {}",
        value
    );
    assert!(
        !value.contains("/tmp"),
        "$(dirname) should not resolve to /tmp, got: {}",
        value
    );

    let mid = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("mid_node"))
        .expect("Should have mid_node");
    assert_eq!(mid["namespace"].as_str().unwrap(), "/level1");

    let root = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("root_node"))
        .expect("Should have root_node");
    assert!(root["namespace"].is_null());
}

// =============================================================================
// Cross-format test: YAML includes Python (28.5)
// =============================================================================

#[test]
fn test_yaml_include_python() {
    // A YAML launch file INCLUDING a `.launch.py` needs an
    // interpreter just as much as a Python file does, and this is
    // the only test in this file that does. Registration is the
    // caller's job since 0897 W2b.
    play_launch_parser_pyexec::register();
    let fixture = get_fixture_path("test_yaml_include_python.launch.yaml");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(
        result.is_ok(),
        "YAML including Python should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 2, "Should have 2 nodes from Python include");

    let has_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("python_talker"));
    assert!(
        has_talker,
        "Should have python_talker from included Python file"
    );

    let has_listener = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("python_listener"));
    assert!(
        has_listener,
        "Should have python_listener from included Python file"
    );

    // Check namespace from push-ros-namespace in YAML
    for node in nodes {
        assert_eq!(
            node["namespace"].as_str().unwrap(),
            "/cross_format",
            "Nodes should inherit /cross_format namespace"
        );
    }
}

/// Test $(command ...) substitution in YAML launch files
#[test]
fn test_yaml_command_substitution() {
    let file = write_yaml(
        r#"launch:
- let:
    name: cmd_val
    value: "$(command echo yaml_cmd_result)"
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: "$(var cmd_val)"
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "Should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "yaml_cmd_result");
}

/// Test $(command ...) with multiple args in YAML
#[test]
fn test_yaml_command_substitution_multi_args() {
    let file = write_yaml(
        r#"launch:
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: "$(command printf '%s_%s' foo bar)"
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "Should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "foo_bar");
}

/// Test $(command ...) concatenated with other text in YAML
#[test]
fn test_yaml_command_substitution_concat() {
    let file = write_yaml(
        r#"launch:
- node:
    pkg: demo_nodes_cpp
    exec: talker
    name: "pre_$(command echo mid)_post"
"#,
    );

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "Should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "pre_mid_post");
}

/// Issue #0022, YAML half — `extra_arg` was dropped here exactly as in XML.
///
/// The Python parser has always carried `extra_arguments`
/// (`visitor/load_composable_nodes.py`), so a YAML launch file lost them under
/// the Rust parser and kept them under the Python one. That makes this a
/// cross-parser divergence, which CLAUDE.md resolves one way: Python is right.
#[test]
fn yaml_extra_args_reach_the_record() {
    let yaml = r#"
launch:
  - node_container:
      pkg: rclcpp_components
      exec: component_container
      name: c
      namespace: ""
      composable_node:
        - pkg: demo_nodes_cpp
          plugin: demo_nodes_cpp::Talker
          name: inner
          extra_arg:
            - name: use_intra_process_comms
              value: true
            - name: executor_threads
              value: 4
        - pkg: demo_nodes_cpp
          plugin: demo_nodes_cpp::Listener
          name: plain
"#;
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("extra.launch.yaml");
    std::fs::write(&path, yaml).unwrap();

    let record = parse_launch_file(&path, HashMap::new()).expect("fixture should parse");
    let json = serde_json::to_value(&record).unwrap();
    let node = |name: &str| -> serde_json::Value {
        json["load_node"]
            .as_array()
            .unwrap()
            .iter()
            .find(|n| n["node_name"] == name)
            .unwrap_or_else(|| panic!("{name} present"))
            .clone()
    };

    // YAML types its scalars where XML leaves text, so `true` and `4` arrive
    // as Bool and Number — both must survive as the strings the LoadNode
    // conversion expects.
    assert_eq!(
        node("inner")["extra_args"]["use_intra_process_comms"],
        "true"
    );
    assert_eq!(node("inner")["extra_args"]["executor_threads"], "4");

    let plain = node("plain");
    assert!(
        plain["extra_args"].as_object().is_none_or(|m| m.is_empty()),
        "unset must stay unset: {:?}",
        plain["extra_args"]
    );
}
