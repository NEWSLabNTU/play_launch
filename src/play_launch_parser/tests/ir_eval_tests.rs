use play_launch_parser::{evaluate_launch_file, parse_launch_file};
use std::{collections::HashMap, io::Write};
use tempfile::NamedTempFile;

/// Helper: write XML to a temp file.
fn write_xml(xml: &str) -> NamedTempFile {
    let mut file = NamedTempFile::with_suffix(".launch.xml").unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();
    file
}

#[test]
fn test_evaluate_simple_node() {
    let file = write_xml(
        r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#,
    );

    let record = evaluate_launch_file(file.path(), HashMap::new()).unwrap();
    assert_eq!(record.node.len(), 1);
    assert_eq!(record.node[0].executable, "talker");
    assert_eq!(record.node[0].package, Some("demo_nodes_cpp".to_string()));
}

#[test]
fn test_evaluate_conditional() {
    let file = write_xml(
        r#"<launch>
            <arg name="use_sim" default="true" />
            <node pkg="sim_pkg" exec="sim_node" name="sim" if="$(var use_sim)" />
            <node pkg="real_pkg" exec="real_node" name="real" unless="$(var use_sim)" />
        </launch>"#,
    );

    let record = evaluate_launch_file(file.path(), HashMap::new()).unwrap();
    // With default use_sim=true: sim_node present, real_node skipped
    assert_eq!(record.node.len(), 1);
    assert_eq!(record.node[0].name, Some("sim".to_string()));
}

#[test]
fn test_evaluate_conditional_override() {
    let file = write_xml(
        r#"<launch>
            <arg name="use_sim" default="true" />
            <node pkg="sim_pkg" exec="sim_node" name="sim" if="$(var use_sim)" />
            <node pkg="real_pkg" exec="real_node" name="real" unless="$(var use_sim)" />
        </launch>"#,
    );

    let mut args = HashMap::new();
    args.insert("use_sim".to_string(), "false".to_string());

    let record = evaluate_launch_file(file.path(), args).unwrap();
    // With use_sim=false: sim_node skipped, real_node present
    assert_eq!(record.node.len(), 1);
    assert_eq!(record.node[0].name, Some("real".to_string()));
}

#[test]
fn test_evaluate_group_namespace() {
    let file = write_xml(
        r#"<launch>
            <group ns="robot1">
                <node pkg="demo_nodes_cpp" exec="talker" />
            </group>
        </launch>"#,
    );

    let record = evaluate_launch_file(file.path(), HashMap::new()).unwrap();
    assert_eq!(record.node.len(), 1);
    assert_eq!(record.node[0].namespace, Some("/robot1".to_string()));
}

#[test]
fn test_evaluate_include() {
    // Create inner file
    let mut inner = NamedTempFile::with_suffix(".launch.xml").unwrap();
    inner
        .write_all(
            br#"<launch>
                <arg name="node_name" default="default_inner" />
                <node pkg="inner_pkg" exec="inner_exec" name="$(var node_name)" />
            </launch>"#,
        )
        .unwrap();
    inner.flush().unwrap();
    let inner_path = inner.path().to_str().unwrap().to_string();

    // Create outer file
    let xml = format!(
        r#"<launch>
            <include file="{}">
                <arg name="node_name" value="custom_name" />
            </include>
        </launch>"#,
        inner_path
    );
    let outer = write_xml(&xml);

    let record = evaluate_launch_file(outer.path(), HashMap::new()).unwrap();
    assert_eq!(record.node.len(), 1);
    assert_eq!(record.node[0].name, Some("custom_name".to_string()));
}

#[test]
fn test_evaluate_container() {
    let file = write_xml(
        r#"<launch>
            <node_container name="my_container" pkg="rclcpp_components" exec="component_container">
                <composable_node pkg="composition" plugin="composition::Talker" name="talker" />
                <composable_node pkg="composition" plugin="composition::Listener" name="listener" />
            </node_container>
        </launch>"#,
    );

    let record = evaluate_launch_file(file.path(), HashMap::new()).unwrap();
    assert_eq!(record.container.len(), 1);
    assert_eq!(record.container[0].name, "my_container");
    assert_eq!(record.load_node.len(), 2);
    assert_eq!(record.load_node[0].node_name, "talker");
    assert_eq!(record.load_node[1].node_name, "listener");
}

#[test]
fn test_evaluate_set_env() {
    let file = write_xml(
        r#"<launch>
            <set_env name="MY_VAR" value="my_value" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#,
    );

    let record = evaluate_launch_file(file.path(), HashMap::new()).unwrap();
    assert_eq!(record.node.len(), 1);
    let env = record.node[0].env.as_ref().unwrap();
    assert!(env.iter().any(|(k, v)| k == "MY_VAR" && v == "my_value"));
}

#[test]
fn test_evaluate_set_parameter() {
    let file = write_xml(
        r#"<launch>
            <set_parameter name="use_sim_time" value="true" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#,
    );

    let record = evaluate_launch_file(file.path(), HashMap::new()).unwrap();
    assert_eq!(record.node.len(), 1);
    let global_params = record.node[0].global_params.as_ref().unwrap();
    assert!(
        global_params
            .iter()
            .any(|(k, v)| k == "use_sim_time" && v == "True")
    );
}

#[test]
fn test_evaluate_round_trip() {
    // A multi-feature XML launch file — verify evaluate produces same output as parse
    let file = write_xml(
        r#"<launch>
            <arg name="my_arg" default="hello" />
            <let name="my_var" value="$(var my_arg)_world" />
            <set_env name="TEST_ENV" value="42" />
            <set_parameter name="use_sim_time" value="true" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var my_var)" />
            <node pkg="demo_nodes_cpp" exec="listener" />
        </launch>"#,
    );

    let parsed = parse_launch_file(file.path(), HashMap::new()).unwrap();
    let evaluated = evaluate_launch_file(file.path(), HashMap::new()).unwrap();

    // Same number of nodes
    assert_eq!(parsed.node.len(), evaluated.node.len());

    // Compare node-by-node
    for (p, e) in parsed.node.iter().zip(evaluated.node.iter()) {
        assert_eq!(p.executable, e.executable, "executable mismatch");
        assert_eq!(p.name, e.name, "name mismatch");
        assert_eq!(p.package, e.package, "package mismatch");
        assert_eq!(p.namespace, e.namespace, "namespace mismatch");
        assert_eq!(p.cmd, e.cmd, "cmd mismatch for {}", p.executable);
        assert_eq!(p.params, e.params, "params mismatch");
        assert_eq!(p.global_params, e.global_params, "global_params mismatch");
    }
}

#[test]
fn test_evaluate_round_trip_with_args() {
    let file = write_xml(
        r#"<launch>
            <arg name="node_name" default="default_name" />
            <node pkg="demo_nodes_cpp" exec="talker" name="$(var node_name)" />
        </launch>"#,
    );

    let mut args = HashMap::new();
    args.insert("node_name".to_string(), "custom_name".to_string());

    let parsed = parse_launch_file(file.path(), args.clone()).unwrap();
    let evaluated = evaluate_launch_file(file.path(), args).unwrap();

    assert_eq!(parsed.node.len(), evaluated.node.len());
    assert_eq!(parsed.node[0].name, evaluated.node[0].name);
    assert_eq!(parsed.node[0].cmd, evaluated.node[0].cmd);
}

#[test]
fn test_evaluate_round_trip_with_group_namespace() {
    let file = write_xml(
        r#"<launch>
            <group ns="robot1">
                <node pkg="demo_nodes_cpp" exec="talker" />
            </group>
            <group ns="robot2">
                <node pkg="demo_nodes_cpp" exec="listener" />
            </group>
        </launch>"#,
    );

    let parsed = parse_launch_file(file.path(), HashMap::new()).unwrap();
    let evaluated = evaluate_launch_file(file.path(), HashMap::new()).unwrap();

    assert_eq!(parsed.node.len(), evaluated.node.len());
    for (p, e) in parsed.node.iter().zip(evaluated.node.iter()) {
        assert_eq!(p.namespace, e.namespace, "namespace mismatch");
        assert_eq!(p.cmd, e.cmd, "cmd mismatch");
    }
}

#[test]
fn test_evaluate_round_trip_include() {
    // Create inner file
    let mut inner = NamedTempFile::with_suffix(".launch.xml").unwrap();
    inner
        .write_all(
            br#"<launch>
                <arg name="inner_name" default="inner_default" />
                <node pkg="inner_pkg" exec="inner_exec" name="$(var inner_name)" />
            </launch>"#,
        )
        .unwrap();
    inner.flush().unwrap();
    let inner_path = inner.path().to_str().unwrap().to_string();

    let xml = format!(
        r#"<launch>
            <node pkg="outer_pkg" exec="outer_exec" name="outer" />
            <include file="{}">
                <arg name="inner_name" value="overridden" />
            </include>
        </launch>"#,
        inner_path
    );
    let outer = write_xml(&xml);

    let parsed = parse_launch_file(outer.path(), HashMap::new()).unwrap();
    let evaluated = evaluate_launch_file(outer.path(), HashMap::new()).unwrap();

    assert_eq!(parsed.node.len(), evaluated.node.len());
    for (p, e) in parsed.node.iter().zip(evaluated.node.iter()) {
        assert_eq!(p.executable, e.executable, "executable mismatch");
        assert_eq!(p.name, e.name, "name mismatch");
        assert_eq!(p.cmd, e.cmd, "cmd mismatch for {}", p.executable);
    }
}

#[test]
fn test_evaluate_executable() {
    let file = write_xml(
        r#"<launch>
            <executable cmd="rviz2">
                <arg value="--display-config" />
                <arg value="default.rviz" />
            </executable>
        </launch>"#,
    );

    let record = evaluate_launch_file(file.path(), HashMap::new()).unwrap();
    assert_eq!(record.node.len(), 1);
    assert_eq!(record.node[0].executable, "rviz2");
    assert_eq!(
        record.node[0].cmd,
        vec!["rviz2", "--display-config", "default.rviz"]
    );
}

#[test]
fn test_evaluate_launch_program_arguments() {
    use play_launch_parser::analyze_launch_file;

    let file = write_xml(
        r#"<launch>
            <arg name="robot_name" default="turtlebot" />
            <arg name="use_sim" default="true" />
            <node pkg="pkg" exec="node" />
        </launch>"#,
    );

    let program = analyze_launch_file(file.path()).unwrap();
    let args = program.arguments();
    assert_eq!(args.len(), 2);
    assert!(args.contains(&"robot_name"));
    assert!(args.contains(&"use_sim"));
}

#[test]
fn test_evaluate_launch_program_all_nodes() {
    use play_launch_parser::{analyze_launch_file, ir::ActionKind};

    let file = write_xml(
        r#"<launch>
            <node pkg="pkg1" exec="node1" />
            <group ns="ns1">
                <executable cmd="exec1" />
            </group>
            <node_container name="container1" pkg="rclcpp_components" exec="component_container">
                <composable_node pkg="comp" plugin="comp::Node" name="comp_node" />
            </node_container>
        </launch>"#,
    );

    let program = analyze_launch_file(file.path()).unwrap();
    let nodes = program.all_nodes();
    assert_eq!(nodes.len(), 3); // node + executable + container
    assert!(matches!(&nodes[0].kind, ActionKind::SpawnNode { .. }));
    assert!(matches!(&nodes[1].kind, ActionKind::SpawnExecutable { .. }));
    assert!(matches!(&nodes[2].kind, ActionKind::SpawnContainer { .. }));
}

#[test]
fn test_evaluate_unset_env() {
    let file = write_xml(
        r#"<launch>
            <set_env name="VAR1" value="value1" />
            <set_env name="VAR2" value="value2" />
            <unset_env name="VAR1" />
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#,
    );

    let record = evaluate_launch_file(file.path(), HashMap::new()).unwrap();
    assert_eq!(record.node.len(), 1);
    let env = record.node[0].env.as_ref().unwrap();
    assert!(!env.iter().any(|(k, _)| k == "VAR1"));
    assert!(env.iter().any(|(k, v)| k == "VAR2" && v == "value2"));
}

#[test]
fn test_evaluate_push_namespace() {
    let file = write_xml(
        r#"<launch>
            <group>
                <push-ros-namespace namespace="robot1" />
                <node pkg="demo_nodes_cpp" exec="talker" />
            </group>
        </launch>"#,
    );

    let parsed = parse_launch_file(file.path(), HashMap::new()).unwrap();
    let evaluated = evaluate_launch_file(file.path(), HashMap::new()).unwrap();

    assert_eq!(parsed.node.len(), 1);
    assert_eq!(evaluated.node.len(), 1);
    assert_eq!(parsed.node[0].namespace, evaluated.node[0].namespace);
    assert_eq!(evaluated.node[0].namespace, Some("/robot1".to_string()));
}

#[test]
fn test_evaluate_nested_groups() {
    let file = write_xml(
        r#"<launch>
            <group ns="robot1">
                <group ns="sensors">
                    <node pkg="demo_nodes_cpp" exec="talker" />
                </group>
            </group>
        </launch>"#,
    );

    let parsed = parse_launch_file(file.path(), HashMap::new()).unwrap();
    let evaluated = evaluate_launch_file(file.path(), HashMap::new()).unwrap();

    assert_eq!(parsed.node[0].namespace, evaluated.node[0].namespace);
    assert_eq!(
        evaluated.node[0].namespace,
        Some("/robot1/sensors".to_string())
    );
}

// --- YAML evaluator tests ---

#[test]
fn test_evaluate_yaml_include_parent_scope() {
    // YAML preset file declares a variable
    let mut yaml = NamedTempFile::with_suffix(".launch.yaml").unwrap();
    yaml.write_all(b"launch:\n  - arg:\n      name: my_var\n      default: preset_value\n")
        .unwrap();
    yaml.flush().unwrap();
    let yaml_path = yaml.path().to_str().unwrap().to_string();

    // XML includes YAML, then uses the variable in a node name
    let xml = format!(
        r#"<launch>
            <include file="{}" />
            <node pkg="my_pkg" exec="my_exec" name="$(var my_var)" />
        </launch>"#,
        yaml_path
    );
    let outer = write_xml(&xml);

    let record = evaluate_launch_file(outer.path(), HashMap::new()).unwrap();
    assert_eq!(record.node.len(), 1);
    assert_eq!(record.node[0].name, Some("preset_value".to_string()));
}

#[test]
fn test_evaluate_yaml_multiple_args() {
    let mut yaml = NamedTempFile::with_suffix(".launch.yaml").unwrap();
    yaml.write_all(
        b"launch:\n  - arg:\n      name: arg_a\n      default: alpha\n  - arg:\n      name: arg_b\n      default: beta\n",
    )
    .unwrap();
    yaml.flush().unwrap();
    let yaml_path = yaml.path().to_str().unwrap().to_string();

    let xml = format!(
        r#"<launch>
            <include file="{}" />
            <node pkg="pkg" exec="exec" name="$(var arg_a)_$(var arg_b)" />
        </launch>"#,
        yaml_path
    );
    let outer = write_xml(&xml);

    let record = evaluate_launch_file(outer.path(), HashMap::new()).unwrap();
    assert_eq!(record.node.len(), 1);
    assert_eq!(record.node[0].name, Some("alpha_beta".to_string()));
}

#[test]
fn test_evaluate_yaml_round_trip() {
    // YAML preset
    let mut yaml = NamedTempFile::with_suffix(".launch.yaml").unwrap();
    yaml.write_all(b"launch:\n  - arg:\n      name: my_var\n      default: yaml_default\n")
        .unwrap();
    yaml.flush().unwrap();
    let yaml_path = yaml.path().to_str().unwrap().to_string();

    // XML includes YAML
    let xml = format!(
        r#"<launch>
            <include file="{}" />
            <node pkg="test_pkg" exec="test_exec" name="$(var my_var)_node" />
        </launch>"#,
        yaml_path
    );
    let outer = write_xml(&xml);

    let parsed = parse_launch_file(outer.path(), HashMap::new()).unwrap();
    let evaluated = evaluate_launch_file(outer.path(), HashMap::new()).unwrap();

    assert_eq!(parsed.node.len(), evaluated.node.len());
    assert_eq!(parsed.node[0].name, evaluated.node[0].name);
    assert_eq!(parsed.node[0].cmd, evaluated.node[0].cmd);
}

#[test]
fn test_evaluate_yaml_cli_arg_override() {
    // YAML preset declares a variable with default
    let mut yaml = NamedTempFile::with_suffix(".launch.yaml").unwrap();
    yaml.write_all(b"launch:\n  - arg:\n      name: mode\n      default: simulation\n")
        .unwrap();
    yaml.flush().unwrap();
    let yaml_path = yaml.path().to_str().unwrap().to_string();

    let xml = format!(
        r#"<launch>
            <include file="{}" />
            <node pkg="pkg" exec="exec" name="$(var mode)_node" />
        </launch>"#,
        yaml_path
    );
    let outer = write_xml(&xml);

    // Override the YAML default via CLI args
    let mut args = HashMap::new();
    args.insert("mode".to_string(), "real".to_string());

    let record = evaluate_launch_file(outer.path(), args).unwrap();
    assert_eq!(record.node.len(), 1);
    assert_eq!(record.node[0].name, Some("real_node".to_string()));
}
