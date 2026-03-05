use play_launch_parser::parse_launch_file;
use std::{collections::HashMap, io::Write, path::PathBuf};
use tempfile::NamedTempFile;

/// Helper to get fixture path from crate tests directory
fn get_fixture_path(filename: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/launch")
        .join(filename)
}

/// Helper to get includes fixture path from crate tests directory
fn get_includes_fixture_path(filename: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/includes")
        .join(filename)
}

#[test]
fn test_parse_conditions_with_args() {
    let fixture = get_fixture_path("test_conditions.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test with use_sim=true (default)
    let mut args = HashMap::new();
    args.insert("use_sim".to_string(), "true".to_string());

    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing with use_sim=true should succeed");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // With use_sim=true, should have sim_talker (if condition)
    let has_sim_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("sim_talker"));
    assert!(has_sim_talker, "Should have sim_talker when use_sim=true");

    // Should not have real_talker (unless condition)
    let has_real_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("real_talker"));
    assert!(
        !has_real_talker,
        "Should not have real_talker when use_sim=true"
    );

    // Test with use_sim=false
    let mut args = HashMap::new();
    args.insert("use_sim".to_string(), "false".to_string());

    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing with use_sim=false should succeed");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // With use_sim=false, should have real_talker (unless condition)
    let has_real_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("real_talker"));
    assert!(
        has_real_talker,
        "Should have real_talker when use_sim=false"
    );

    // Should not have sim_talker (if condition)
    let has_sim_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("sim_talker"));
    assert!(
        !has_sim_talker,
        "Should not have sim_talker when use_sim=false"
    );
}

#[test]
fn test_parse_include_fixture() {
    let fixture = get_fixture_path("test_include.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Also verify included file exists
    let included = get_includes_fixture_path("included.launch.xml");
    assert!(
        included.exists(),
        "Included fixture should exist: {:?}",
        included
    );

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing with includes should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // Should have 3 nodes:
    // 1. From first include (default node name)
    // 2. From second include (custom_talker)
    // 3. Local listener node
    assert_eq!(nodes.len(), 3, "Should have 3 nodes total");

    // Check we have the local listener
    let has_listener = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("local_listener"));
    assert!(has_listener, "Should have local_listener node");

    // Check we have custom_talker from second include
    let has_custom = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("custom_talker"));
    assert!(has_custom, "Should have custom_talker from second include");
}

#[test]
fn test_parse_all_features() {
    let fixture = get_fixture_path("test_all_features.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test with default args (use_sim=true)
    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing all_features should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify structure
    assert!(json["node"].is_array(), "Should have node array");
    assert!(json["container"].is_array(), "Should have container array");
    assert!(json["load_node"].is_array(), "Should have load_node array");
    assert!(
        json["lifecycle_node"].is_array(),
        "Should have lifecycle_node array"
    );

    let nodes = json["node"].as_array().unwrap();

    // With use_sim=true (default), should have 2 nodes: talker and included_node
    assert_eq!(nodes.len(), 2, "Should have 2 nodes with use_sim=true");

    // Should have talker (if condition)
    let talker = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("talker"))
        .expect("Should have talker when use_sim=true");
    assert_eq!(
        talker["namespace"].as_str().unwrap(),
        "/robots",
        "Talker should be in /robots namespace from group"
    );

    // Should not have listener (unless condition)
    let has_listener = nodes.iter().any(|n| n["name"].as_str() == Some("listener"));
    assert!(!has_listener, "Should not have listener when use_sim=true");

    // Should have included_node from the include
    let included_node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("included_node"))
        .expect("Should have included_node from include");
    assert!(
        included_node["namespace"].is_null(),
        "Included node should have null namespace (root/unspecified)"
    );
}

#[test]
fn test_find_pkg_share_substitution() {
    let fixture = get_fixture_path("test_find_pkg.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing with find-pkg-share should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();
    let nodes = json["node"].as_array().unwrap();

    assert!(!nodes.is_empty(), "Should have at least one node");

    // Check that find-pkg-share was resolved
    // The params should have resolved paths (not containing $(find-pkg-share...))
    for node in nodes {
        if let Some(params) = node["params"].as_array() {
            for param in params {
                if let Some(param_arr) = param.as_array()
                    && param_arr.len() >= 2
                {
                    let value = param_arr[1].as_str().unwrap_or("");
                    assert!(
                        !value.contains("$(find-pkg-share"),
                        "find-pkg-share should be resolved, got: {}",
                        value
                    );
                }
            }
        }
    }
}

#[test]
fn test_argument_override() {
    let fixture = get_fixture_path("test_args.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test with custom argument
    let mut args = HashMap::new();
    args.insert("topic_name".to_string(), "/custom_topic".to_string());

    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing with custom args should succeed");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // Check that the remap uses the custom topic
    for node in nodes {
        if let Some(remaps) = node["remaps"].as_array() {
            for remap in remaps {
                if let Some(remap_arr) = remap.as_array()
                    && remap_arr.len() >= 2
                {
                    let from = remap_arr[0].as_str().unwrap_or("");
                    let to = remap_arr[1].as_str().unwrap_or("");
                    if from == "chatter" {
                        // The fixture might use the arg for remapping
                        // This depends on the fixture structure
                        // Just verify remaps exist and are strings
                        assert!(!to.is_empty(), "Remap target should not be empty");
                    }
                }
            }
        }
    }
}

#[test]
fn test_json_output_structure() {
    let fixture = get_fixture_path("test_args.launch.xml");

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok());

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Verify top-level structure matches record.json format
    assert!(json.is_object(), "Output should be JSON object");
    assert!(json.get("node").is_some(), "Should have 'node' field");
    assert!(
        json.get("container").is_some(),
        "Should have 'container' field"
    );
    assert!(
        json.get("load_node").is_some(),
        "Should have 'load_node' field"
    );
    assert!(
        json.get("lifecycle_node").is_some(),
        "Should have 'lifecycle_node' field"
    );
    assert!(
        json.get("file_data").is_some(),
        "Should have 'file_data' field"
    );

    // All should be arrays or objects
    assert!(json["node"].is_array());
    assert!(json["container"].is_array());
    assert!(json["load_node"].is_array());
    assert!(json["lifecycle_node"].is_array());
    assert!(json["file_data"].is_object());
}

#[test]
fn test_node_command_generation() {
    let fixture = get_fixture_path("test_args.launch.xml");

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    assert!(!nodes.is_empty(), "Should have at least one node");

    // Check that cmd array is generated
    for node in nodes {
        assert!(node.get("cmd").is_some(), "Node should have cmd field");
        let cmd = node["cmd"].as_array().unwrap();
        assert!(!cmd.is_empty(), "cmd array should not be empty");

        // First element should be the executable path
        let exe_path = cmd[0].as_str().unwrap();
        assert!(!exe_path.is_empty(), "Executable path should not be empty");

        // Should contain --ros-args if there are params or remaps
        if node
            .get("params")
            .and_then(|p| p.as_array())
            .map(|a| !a.is_empty())
            .unwrap_or(false)
            || node
                .get("remaps")
                .and_then(|r| r.as_array())
                .map(|a| !a.is_empty())
                .unwrap_or(false)
        {
            let has_ros_args = cmd.iter().any(|arg| arg.as_str() == Some("--ros-args"));
            assert!(
                has_ros_args,
                "cmd should contain --ros-args when there are params or remaps"
            );
        }
    }
}

#[test]
fn test_push_pop_ros_namespace_actions() {
    // Test push-ros-namespace and pop-ros-namespace XML actions
    let xml = r#"<launch>
        <push-ros-namespace ns="robot1" />
        <node pkg="demo_nodes_cpp" exec="talker" name="talker1" />

        <push-ros-namespace ns="sensors" />
        <node pkg="demo_nodes_cpp" exec="listener" name="listener1" />
        <pop-ros-namespace />

        <node pkg="demo_nodes_cpp" exec="talker" name="talker2" />
        <pop-ros-namespace />

        <node pkg="demo_nodes_cpp" exec="listener" name="listener2" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "Should parse successfully");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 4, "Should have 4 nodes");

    // Check namespaces
    // talker1 should be in /robot1
    let talker1 = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("talker1"))
        .unwrap();
    assert_eq!(talker1["namespace"].as_str().unwrap(), "/robot1");

    // listener1 should be in /robot1/sensors
    let listener1 = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("listener1"))
        .unwrap();
    assert_eq!(listener1["namespace"].as_str().unwrap(), "/robot1/sensors");

    // talker2 should be in /robot1 (after popping sensors)
    let talker2 = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("talker2"))
        .unwrap();
    assert_eq!(talker2["namespace"].as_str().unwrap(), "/robot1");

    // listener2 should have null namespace (root/unspecified, after popping robot1)
    let listener2 = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("listener2"))
        .unwrap();
    assert!(
        listener2["namespace"].is_null(),
        "listener2 should have null namespace (root/unspecified)"
    );
}

#[test]
fn test_push_ros_namespace_with_substitution() {
    // Test that push-ros-namespace supports substitutions
    let xml = r#"<launch>
        <arg name="robot_ns" default="my_robot" />
        <push-ros-namespace ns="$(var robot_ns)" />
        <node pkg="demo_nodes_cpp" exec="talker" name="talker" />
        <pop-ros-namespace />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "Should parse successfully");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);

    // Should use the substituted namespace
    assert_eq!(nodes[0]["namespace"].as_str().unwrap(), "/my_robot");
}

#[test]
fn test_complex_nested_launch() {
    // Test complex nested launch file with multiple levels of namespaces and includes
    let fixture = get_fixture_path("test_complex_nested.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test with default args (use_sim_time=true, use_rviz=false)
    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);
    assert!(
        result.is_ok(),
        "Should parse complex file: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // Should have multiple nodes from different namespaces
    assert!(nodes.len() >= 5, "Should have at least 5 nodes");

    // Check camera node namespace
    let camera = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("front_camera"))
        .expect("Should have front_camera node");
    assert_eq!(
        camera["namespace"].as_str().unwrap(),
        "/robot1/sensors/camera",
        "Camera should be in /robot1/sensors/camera namespace"
    );

    // Check lidar node (should be present with use_sim_time=true)
    let lidar = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("front_lidar"));
    assert!(
        lidar.is_some(),
        "Lidar should be present when use_sim_time=true"
    );
    assert_eq!(
        lidar.unwrap()["namespace"].as_str().unwrap(),
        "/robot1/sensors/lidar",
        "Lidar should be in /robot1/sensors/lidar namespace"
    );

    // Check AMCL node namespace
    let amcl = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("amcl"))
        .expect("Should have amcl node");
    assert_eq!(
        amcl["namespace"].as_str().unwrap(),
        "/robot1/navigation",
        "AMCL should be in /robot1/navigation namespace"
    );

    // Check that map_server is NOT present (unless condition with use_sim_time=true)
    let map_server = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("map_server"));
    assert!(
        map_server.is_none(),
        "Map server should not be present when use_sim_time=true"
    );

    // Check that RViz is NOT present (use_rviz=false by default)
    let rviz = nodes.iter().find(|n| n["name"].as_str() == Some("rviz2"));
    assert!(
        rviz.is_none(),
        "RViz should not be present when use_rviz=false"
    );

    // Check nodes from included file
    let robot_state_pub = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("robot_state_publisher"))
        .expect("Should have robot_state_publisher from include");
    assert_eq!(
        robot_state_pub["namespace"].as_str().unwrap(),
        "/robot1",
        "Robot state publisher should be in /robot1 namespace"
    );

    // Joint state publisher should be present (use_sim_time=true in include)
    let joint_state_pub = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("joint_state_publisher"));
    assert!(
        joint_state_pub.is_some(),
        "Joint state publisher should be present in sim mode"
    );
}

#[test]
fn test_complex_nested_with_args_override() {
    // Test same complex file but with overridden arguments
    let fixture = get_fixture_path("test_complex_nested.launch.xml");

    let mut args = HashMap::new();
    args.insert("robot_name".to_string(), "test_bot".to_string());
    args.insert("use_sim_time".to_string(), "false".to_string());
    args.insert("use_rviz".to_string(), "true".to_string());

    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Should parse with overridden args");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // Check camera is in test_bot namespace
    let camera = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("front_camera"))
        .expect("Should have front_camera");
    assert!(
        camera["namespace"]
            .as_str()
            .unwrap()
            .starts_with("/test_bot"),
        "Camera should be in test_bot namespace"
    );

    // Lidar should NOT be present (use_sim_time=false)
    let lidar = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("front_lidar"));
    assert!(
        lidar.is_none(),
        "Lidar should not be present when use_sim_time=false"
    );

    // Map server SHOULD be present (unless condition with use_sim_time=false)
    let map_server = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("map_server"));
    assert!(
        map_server.is_some(),
        "Map server should be present when use_sim_time=false"
    );

    // RViz SHOULD be present (use_rviz=true)
    let rviz = nodes.iter().find(|n| n["name"].as_str() == Some("rviz2"));
    assert!(rviz.is_some(), "RViz should be present when use_rviz=true");
}

#[test]
fn test_deeply_nested_namespaces() {
    // Test that deeply nested namespace stacking works correctly
    let xml = r#"<launch>
        <group ns="level1">
            <push-ros-namespace ns="level2" />
            <group>
                <push-ros-namespace ns="level3" />
                <group>
                    <push-ros-namespace ns="level4" />
                    <node pkg="test_pkg" exec="test_node" name="deep_node" />
                    <pop-ros-namespace />
                </group>
                <node pkg="test_pkg" exec="test_node" name="mid_node" />
                <pop-ros-namespace />
            </group>
            <node pkg="test_pkg" exec="test_node" name="shallow_node" />
            <pop-ros-namespace />
        </group>
        <node pkg="test_pkg" exec="test_node" name="root_node" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "Should parse deeply nested namespaces");

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 4, "Should have 4 nodes");

    // Check each node's namespace
    let deep_node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("deep_node"))
        .unwrap();
    assert_eq!(
        deep_node["namespace"].as_str().unwrap(),
        "/level1/level2/level3/level4"
    );

    let mid_node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("mid_node"))
        .unwrap();
    assert_eq!(
        mid_node["namespace"].as_str().unwrap(),
        "/level1/level2/level3"
    );

    let shallow_node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("shallow_node"))
        .unwrap();
    assert_eq!(
        shallow_node["namespace"].as_str().unwrap(),
        "/level1/level2"
    );

    let root_node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("root_node"))
        .unwrap();
    assert!(
        root_node["namespace"].is_null(),
        "root_node should have null namespace (root/unspecified)"
    );
}

#[test]
fn test_nested_variable_substitutions() {
    // Test that variables containing substitutions are resolved correctly
    let fixture = get_fixture_path("test_nested_var_substitutions.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    // Should parse successfully (packages may not exist, but lenient mode should handle it)
    assert!(
        result.is_ok(),
        "Should parse with nested variable substitutions: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    assert_eq!(nodes.len(), 1, "Should have 1 node");

    // Check node uses the variable
    let node = &nodes[0];
    assert_eq!(node["name"].as_str().unwrap(), "test_node");

    // Package should be resolved from pkg_name variable
    assert_eq!(node["package"].as_str().unwrap(), "demo_nodes_cpp");

    // The exec_path parameter should contain either:
    // 1. The resolved path if demo_nodes_cpp is installed
    // 2. The literal $(find-pkg-share demo_nodes_cpp) if not found (lenient mode)
    let params = node["params"].as_array().unwrap();
    let exec_path_param = params
        .iter()
        .find(|p| p[0].as_str() == Some("exec_path"))
        .expect("Should have exec_path parameter");

    // The value should be present (either resolved or literal)
    assert!(
        !exec_path_param[1].as_str().unwrap().is_empty(),
        "exec_path parameter should have a value"
    );

    println!("Nested variable substitution test passed!");
    println!("exec_path value: {}", exec_path_param[1].as_str().unwrap());
}

#[test]
fn test_yaml_file_skip() {
    // Create a temporary YAML file
    let mut temp_file = NamedTempFile::new().unwrap();
    let yaml_content = r#"
some_param: value
another_param: 123
"#;
    temp_file.write_all(yaml_content.as_bytes()).unwrap();

    // Rename to .yaml extension
    let yaml_path = temp_file.path().with_extension("yaml");
    std::fs::copy(temp_file.path(), &yaml_path).unwrap();

    // Parse should succeed and skip the file gracefully
    let args = HashMap::new();
    let result = parse_launch_file(&yaml_path, args);

    // Should succeed with empty records
    assert!(result.is_ok(), "YAML file should be skipped without error");
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Should have no nodes (file was skipped)
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 0, "YAML file should produce no nodes");

    // Cleanup
    std::fs::remove_file(&yaml_path).ok();
}

#[test]
fn test_set_env_hyphenated() {
    // Create temporary launch file with set-env (hyphenated)
    let mut temp_file = NamedTempFile::new().unwrap();
    let launch_content = r#"<?xml version="1.0"?>
<launch>
    <set-env name="MY_TEST_VAR" value="test_value" />
    <node pkg="demo_nodes_cpp" exec="talker" name="test_node">
        <env name="MY_TEST_VAR" value="$(env MY_TEST_VAR)" />
    </node>
</launch>
"#;
    temp_file.write_all(launch_content.as_bytes()).unwrap();

    let args = HashMap::new();
    let result = parse_launch_file(temp_file.path(), args);

    assert!(
        result.is_ok(),
        "set-env should work with hyphens: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();

    assert_eq!(nodes.len(), 1, "Should have 1 node");

    // Check node has the environment variable
    let node = &nodes[0];
    let env = node["env"].as_array().unwrap();
    assert!(!env.is_empty(), "Node should have env vars");

    // Find MY_TEST_VAR
    let test_var = env
        .iter()
        .find(|e| e[0].as_str() == Some("MY_TEST_VAR"))
        .expect("Should have MY_TEST_VAR");

    assert_eq!(
        test_var[1].as_str().unwrap(),
        "test_value",
        "Environment variable should be resolved"
    );
}

#[test]
fn test_unset_env_hyphenated() {
    // Set an environment variable first
    unsafe { std::env::set_var("TEST_UNSET_VAR", "initial_value") };

    // Create temporary launch file with unset-env (hyphenated)
    let mut temp_file = NamedTempFile::new().unwrap();
    let launch_content = r#"<?xml version="1.0"?>
<launch>
    <unset-env name="TEST_UNSET_VAR" />
    <node pkg="demo_nodes_cpp" exec="talker" name="test_node" />
</launch>
"#;
    temp_file.write_all(launch_content.as_bytes()).unwrap();

    let args = HashMap::new();
    let result = parse_launch_file(temp_file.path(), args);

    assert!(
        result.is_ok(),
        "unset-env should work with hyphens: {:?}",
        result.err()
    );

    // Note: unset-env affects the context during parsing
    // It doesn't directly show in the output, but should not cause errors
    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node");
}

#[test]
fn test_node_container() {
    // Create temporary launch file with node_container (underscore)
    let mut temp_file = NamedTempFile::new().unwrap();
    let launch_content = r#"<?xml version="1.0"?>
<launch>
    <node_container pkg="rclcpp_components" exec="component_container" name="my_container" namespace="/test" />
</launch>
"#;
    temp_file.write_all(launch_content.as_bytes()).unwrap();

    let args = HashMap::new();
    let result = parse_launch_file(temp_file.path(), args);

    assert!(
        result.is_ok(),
        "node_container should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let containers = json["container"].as_array().unwrap();

    assert_eq!(containers.len(), 1, "Should have 1 container");

    // Check container details
    let container = &containers[0];
    assert_eq!(container["name"].as_str().unwrap(), "my_container");
    assert_eq!(container["namespace"].as_str().unwrap(), "/test");
}

#[test]
fn test_node_container_hyphenated() {
    // Create temporary launch file with node-container (hyphenated)
    let mut temp_file = NamedTempFile::new().unwrap();
    let launch_content = r#"<?xml version="1.0"?>
<launch>
    <node-container pkg="rclcpp_components" exec="component_container" name="my_container" namespace="/test" />
</launch>
"#;
    temp_file.write_all(launch_content.as_bytes()).unwrap();

    let args = HashMap::new();
    let result = parse_launch_file(temp_file.path(), args);

    assert!(
        result.is_ok(),
        "node-container should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let containers = json["container"].as_array().unwrap();

    assert_eq!(containers.len(), 1, "Should have 1 container");
}

#[test]
fn test_composable_node_in_container() {
    // Create temporary launch file with composable_node inside node_container
    let mut temp_file = NamedTempFile::new().unwrap();
    let launch_content = r#"<?xml version="1.0"?>
<launch>
    <node_container pkg="rclcpp_components" exec="component_container" name="my_container">
        <composable_node pkg="demo_nodes_cpp" plugin="demo_nodes_cpp::Talker" name="talker" />
        <composable_node pkg="demo_nodes_cpp" plugin="demo_nodes_cpp::Listener" name="listener" />
    </node_container>
</launch>
"#;
    temp_file.write_all(launch_content.as_bytes()).unwrap();

    let args = HashMap::new();
    let result = parse_launch_file(temp_file.path(), args);

    assert!(
        result.is_ok(),
        "node_container with composable_node children should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();

    // Check container
    let containers = json["container"].as_array().unwrap();
    assert_eq!(containers.len(), 1, "Should have 1 container");
    assert_eq!(containers[0]["name"].as_str().unwrap(), "my_container");

    // Check composable nodes are captured as load_node entries
    let load_nodes = json["load_node"].as_array().unwrap();
    assert_eq!(load_nodes.len(), 2, "Should have 2 composable nodes");

    // Check first composable node
    assert_eq!(load_nodes[0]["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(
        load_nodes[0]["plugin"].as_str().unwrap(),
        "demo_nodes_cpp::Talker"
    );
    assert_eq!(load_nodes[0]["node_name"].as_str().unwrap(), "talker");
    assert_eq!(
        load_nodes[0]["target_container_name"].as_str().unwrap(),
        "/my_container"
    );

    // Check second composable node
    assert_eq!(load_nodes[1]["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(
        load_nodes[1]["plugin"].as_str().unwrap(),
        "demo_nodes_cpp::Listener"
    );
    assert_eq!(load_nodes[1]["node_name"].as_str().unwrap(), "listener");
    assert_eq!(
        load_nodes[1]["target_container_name"].as_str().unwrap(),
        "/my_container"
    );
}

#[test]
fn test_load_composable_node() {
    let fixture = get_fixture_path("test_load_composable_node.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing launch file with load_composable_node should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Check containers
    let containers = json["container"].as_array().unwrap();
    eprintln!("Found {} containers:", containers.len());
    for container in containers {
        eprintln!(
            "  - {} (namespace: {})",
            container["name"].as_str().unwrap_or("no name"),
            container["namespace"].as_str().unwrap_or("no namespace")
        );
    }
    assert_eq!(containers.len(), 2, "Should have 2 containers");

    // Check first container (with substitution)
    let test_container = containers
        .iter()
        .find(|c| c["namespace"].as_str() == Some("/test"));
    assert!(test_container.is_some(), "Should have /test container");
    let test_container = test_container.unwrap();
    assert_eq!(test_container["name"].as_str().unwrap(), "my_container");

    // Check second container (planning)
    let planning_container = containers
        .iter()
        .find(|c| c["namespace"].as_str() == Some("/planning"));
    assert!(
        planning_container.is_some(),
        "Should have /planning container"
    );
    let planning_container = planning_container.unwrap();
    assert_eq!(
        planning_container["name"].as_str().unwrap(),
        "planning_container"
    );

    // Check load_nodes
    let load_nodes = json["load_node"].as_array().unwrap();
    eprintln!("Found {} load_nodes:", load_nodes.len());
    for node in load_nodes {
        eprintln!(
            "  - {} (container: {}, namespace: {})",
            node["node_name"].as_str().unwrap_or("no name"),
            node["target_container_name"]
                .as_str()
                .unwrap_or("no container"),
            node["namespace"].as_str().unwrap_or("no namespace")
        );
    }

    // Should have 4 load_nodes:
    // 1. initial_node (in test container, from node_container)
    // 2. dynamic_node_1 (loaded via load_composable_node)
    // 3. dynamic_node_2 (loaded via load_composable_node, custom namespace)
    // 4. planner (in planning container, from node_container)
    // 5. validator (loaded via load_composable_node into planning container)
    assert_eq!(load_nodes.len(), 5, "Should have 5 load_nodes total");

    // Check initial_node (from node_container)
    let initial_node = load_nodes
        .iter()
        .find(|n| n["node_name"].as_str() == Some("initial_node"));
    assert!(initial_node.is_some(), "Should have initial_node");
    let initial_node = initial_node.unwrap();
    assert_eq!(
        initial_node["target_container_name"].as_str().unwrap(),
        "/test/my_container"
    );
    assert_eq!(initial_node["namespace"].as_str().unwrap(), "/test");
    assert_eq!(initial_node["package"].as_str().unwrap(), "initial_pkg");

    // Check dynamic_node_1 (loaded via load_composable_node)
    let dynamic_node_1 = load_nodes
        .iter()
        .find(|n| n["node_name"].as_str() == Some("dynamic_node_1"));
    assert!(dynamic_node_1.is_some(), "Should have dynamic_node_1");
    let dynamic_node_1 = dynamic_node_1.unwrap();
    assert_eq!(
        dynamic_node_1["target_container_name"].as_str().unwrap(),
        "/test/my_container"
    );
    assert_eq!(dynamic_node_1["namespace"].as_str().unwrap(), "/test");
    assert_eq!(dynamic_node_1["package"].as_str().unwrap(), "dynamic_pkg1");
    assert_eq!(dynamic_node_1["plugin"].as_str().unwrap(), "DynamicPlugin1");

    // Check params
    let params = dynamic_node_1["params"].as_array().unwrap();
    assert_eq!(params.len(), 1, "dynamic_node_1 should have 1 param");
    let param = params[0].as_array().unwrap();
    assert_eq!(param[0].as_str().unwrap(), "param1");
    assert_eq!(param[1].as_str().unwrap(), "value1");

    // Check remaps
    let remaps = dynamic_node_1["remaps"].as_array().unwrap();
    assert_eq!(remaps.len(), 1, "dynamic_node_1 should have 1 remap");
    let remap = remaps[0].as_array().unwrap();
    assert_eq!(remap[0].as_str().unwrap(), "input");
    assert_eq!(remap[1].as_str().unwrap(), "/test/input");

    // Check dynamic_node_2 (with custom namespace)
    let dynamic_node_2 = load_nodes
        .iter()
        .find(|n| n["node_name"].as_str() == Some("dynamic_node_2"));
    assert!(dynamic_node_2.is_some(), "Should have dynamic_node_2");
    let dynamic_node_2 = dynamic_node_2.unwrap();
    assert_eq!(
        dynamic_node_2["target_container_name"].as_str().unwrap(),
        "/test/my_container"
    );
    // This node has custom namespace, not inherited from container
    assert_eq!(dynamic_node_2["namespace"].as_str().unwrap(), "/custom_ns");
    assert_eq!(dynamic_node_2["package"].as_str().unwrap(), "dynamic_pkg2");

    // Check validator (loaded into planning container)
    let validator = load_nodes
        .iter()
        .find(|n| n["node_name"].as_str() == Some("validator"));
    assert!(validator.is_some(), "Should have validator node");
    let validator = validator.unwrap();
    assert_eq!(
        validator["target_container_name"].as_str().unwrap(),
        "/planning/planning_container"
    );
    assert_eq!(validator["namespace"].as_str().unwrap(), "/planning");
    assert_eq!(validator["package"].as_str().unwrap(), "validator_pkg");
}

#[test]
fn test_let_statement_ordering() {
    let fixture = get_fixture_path("test_let_ordering.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing launch file with let statements should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have 3 nodes
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 3, "Should have 3 nodes");

    // Node 1: should have parameter resolved to "initial_value" (before first let)
    let node1 = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("node1"))
        .expect("Should have node1");
    let params1 = node1["params"].as_array().unwrap();
    assert_eq!(params1.len(), 1, "Node1 should have 1 parameter");
    let param1 = params1[0].as_array().unwrap();
    assert_eq!(param1[0].as_str().unwrap(), "before_let");
    assert_eq!(
        param1[1].as_str().unwrap(),
        "initial_value",
        "Parameter should be resolved to initial_value (before let)"
    );

    // Node 2: should have parameter resolved to "changed_value" (after first let)
    let node2 = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("node2"))
        .expect("Should have node2");
    let params2 = node2["params"].as_array().unwrap();
    assert_eq!(params2.len(), 1, "Node2 should have 1 parameter");
    let param2 = params2[0].as_array().unwrap();
    assert_eq!(param2[0].as_str().unwrap(), "after_let");
    assert_eq!(
        param2[1].as_str().unwrap(),
        "changed_value",
        "Parameter should be resolved to changed_value (after first let)"
    );

    // Node 3: should have parameter resolved to "final_value" (after second let)
    let node3 = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("node3"))
        .expect("Should have node3");
    let params3 = node3["params"].as_array().unwrap();
    assert_eq!(params3.len(), 1, "Node3 should have 1 parameter");
    let param3 = params3[0].as_array().unwrap();
    assert_eq!(param3[0].as_str().unwrap(), "after_second_let");
    assert_eq!(
        param3[1].as_str().unwrap(),
        "final_value",
        "Parameter should be resolved to final_value (after second let)"
    );

    // Verify that the final value in variables is "final_value"
    let variables = json["variables"].as_object().unwrap();
    assert_eq!(
        variables.get("test_var").and_then(|v| v.as_str()),
        Some("final_value"),
        "Variables should contain final value of test_var"
    );
}

// =============================================================================
// XML fixture-based tests (28.3)
// =============================================================================

#[test]
fn test_xml_set_remap() {
    let fixture = get_fixture_path("test_xml_set_remap.launch.xml");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(result.is_ok(), "Should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "remap_node");

    let cmd = nodes[0]["cmd"].as_array().unwrap();
    let cmd_str: String = cmd
        .iter()
        .filter_map(|v| v.as_str())
        .collect::<Vec<_>>()
        .join(" ");
    assert!(
        cmd_str.contains("/input:=/remapped_input"),
        "CMD should contain remap: {}",
        cmd_str
    );
}

#[test]
fn test_xml_executable() {
    let fixture = get_fixture_path("test_xml_executable.launch.xml");
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
fn test_xml_set_parameter() {
    let fixture = get_fixture_path("test_xml_set_parameter.launch.xml");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(result.is_ok(), "Should parse: {:?}", result.err());

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "param_node");

    let global_params = nodes[0]["global_params"].as_array();
    assert!(
        global_params.is_some(),
        "Node should have global_params from set_parameter"
    );
    let has_sim_time = global_params.unwrap().iter().any(|p| {
        p[0].as_str() == Some("use_sim_time")
            && p[1]
                .as_str()
                .is_some_and(|v| v.eq_ignore_ascii_case("true"))
    });
    assert!(has_sim_time, "Should have use_sim_time=true global param");
}

// =============================================================================
// Cross-format test: XML → YAML preset chain (28.5)
// =============================================================================

#[test]
fn test_xml_include_yaml_preset_then_use() {
    let fixture = get_fixture_path("test_xml_yaml_preset_chain.launch.xml");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(
        result.is_ok(),
        "XML including YAML preset should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(
        nodes[0]["name"].as_str().unwrap(),
        "JointTrajectory",
        "Variable from YAML preset should resolve in XML scope"
    );
}
