use play_launch_parser::parse_launch_file;
use std::{collections::HashMap, io::Write, path::PathBuf};
use tempfile::NamedTempFile;

use std::sync::{Mutex, MutexGuard};

/// Global mutex to serialize Python tests
/// This prevents race conditions in the Python interpreter's global state
static PYTHON_TEST_LOCK: Mutex<()> = Mutex::new(());

/// Helper to get a lock for Python tests to ensure they run serially
/// Recovers from poisoned mutex if a previous test panicked
fn python_test_guard() -> MutexGuard<'static, ()> {
    PYTHON_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

/// Create a temporary launch file with a .launch.py suffix for inline test content.
/// Returns a NamedTempFile that stays alive (and on disk) while the returned value exists.
fn write_temp_launch_file(content: &str) -> NamedTempFile {
    let mut f = tempfile::Builder::new()
        .suffix(".launch.py")
        .tempfile()
        .unwrap();
    f.write_all(content.as_bytes()).unwrap();
    f
}

/// Helper to get fixture path from crate tests directory
fn get_fixture_path(filename: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/launch")
        .join(filename)
}

#[test]
fn test_parse_args_fixture() {
    let fixture = get_fixture_path("test_args.launch.xml");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Parse with default args
    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(result.is_ok(), "Parsing should succeed: {:?}", result.err());
    let record = result.unwrap();

    // Convert to JSON for easier inspection
    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    assert!(!nodes.is_empty(), "Should have at least one node");

    // Check first node
    let node = &nodes[0];
    assert_eq!(node["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node["executable"].as_str().unwrap(), "talker");
    assert_eq!(node["name"].as_str().unwrap(), "my_talker");
    assert!(
        node["namespace"].is_null(),
        "Namespace should be null (root/unspecified)"
    );

    // Check params
    assert!(node["params"].is_array(), "Should have params");
    let params = node["params"].as_array().unwrap();
    assert!(!params.is_empty(), "Should have at least one param");

    // Check remaps
    assert!(node["remaps"].is_array(), "Should have remaps");
    let remaps = node["remaps"].as_array().unwrap();
    assert!(!remaps.is_empty(), "Should have at least one remap");

    // Check env vars
    assert!(node["env"].is_array(), "Should have env vars");
    let env = node["env"].as_array().unwrap();
    assert!(!env.is_empty(), "Should have at least one env var");
}

#[test]
fn test_parse_python_no_import() {
    let _guard = python_test_guard();
    // Test that sys.modules registration works
    let fixture = get_fixture_path("test_no_import.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file (sys.modules access) should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node");

    // Check the node
    let node = &nodes[0];
    assert_eq!(
        node["package"].as_str().unwrap(),
        "demo_nodes_cpp",
        "Should have correct package"
    );
    assert_eq!(
        node["name"].as_str().unwrap(),
        "sys_modules_test",
        "Should have correct name"
    );
}

#[test]
fn test_parse_python_container() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_python_container.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python container launch file should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Check container
    let containers = json["container"].as_array().unwrap();
    assert_eq!(containers.len(), 1, "Should have 1 container");
    assert_eq!(
        containers[0]["name"].as_str().unwrap(),
        "my_component_container"
    );
    assert_eq!(containers[0]["namespace"].as_str().unwrap(), "/test_ns");

    // Check composable nodes are captured as load_node entries
    let load_nodes = json["load_node"].as_array().unwrap();
    assert_eq!(load_nodes.len(), 2, "Should have 2 composable nodes");

    // Check first composable node
    assert_eq!(load_nodes[0]["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(
        load_nodes[0]["plugin"].as_str().unwrap(),
        "demo_nodes_cpp::Talker"
    );
    assert_eq!(load_nodes[0]["node_name"].as_str().unwrap(), "talker_node");
    assert_eq!(
        load_nodes[0]["target_container_name"].as_str().unwrap(),
        "/test_ns/my_component_container"
    );
    assert_eq!(
        load_nodes[0]["namespace"].as_str().unwrap(),
        "/test_ns", // Inherits from container
        "Should inherit namespace from container"
    );

    // Check second composable node
    assert_eq!(load_nodes[1]["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(
        load_nodes[1]["plugin"].as_str().unwrap(),
        "demo_nodes_cpp::Listener"
    );
    assert_eq!(
        load_nodes[1]["node_name"].as_str().unwrap(),
        "listener_node"
    );
    assert_eq!(
        load_nodes[1]["target_container_name"].as_str().unwrap(),
        "/test_ns/my_component_container"
    );
    assert_eq!(
        load_nodes[1]["namespace"].as_str().unwrap(),
        "/custom_ns", // Has its own namespace
        "Should use custom namespace"
    );
}

#[test]
fn test_parse_simple_python_launch() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_simple_python.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node");

    // Check the node
    let node = &nodes[0];
    assert_eq!(
        node["package"].as_str().unwrap(),
        "demo_nodes_cpp",
        "Should have correct package"
    );
    assert_eq!(
        node["executable"].as_str().unwrap(),
        "talker",
        "Should have correct executable"
    );
    assert_eq!(
        node["name"].as_str().unwrap(),
        "my_talker",
        "Should have correct name"
    );
    assert_eq!(
        node["namespace"].as_str().unwrap(),
        "/test",
        "Should have correct namespace"
    );
}

#[test]
fn test_parse_python_substitutions() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_python_substitutions.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let mut args = HashMap::new();
    args.insert("package_name".to_string(), "my_package".to_string());
    args.insert("use_sim_time".to_string(), "true".to_string());

    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with substitutions should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 4, "Should have 4 nodes");

    // Check first node with PathJoinSubstitution
    let node = &nodes[0];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "talker_with_config",
        "Should have correct name"
    );

    // Verify parameter with substitutions
    let params = node["params"].as_array().expect("Should have params array");
    assert!(!params.is_empty(), "Should have parameters");

    // Check that config_path parameter contains PathJoinSubstitution result
    let config_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("config_path")
    });
    assert!(config_param.is_some(), "Should have config_path parameter");

    // Check second node with EnvironmentVariable
    let node = &nodes[1];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "listener_with_env",
        "Should have correct name"
    );

    // Check third node with ThisLaunchFileDir
    let node = &nodes[2];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "talker_with_dir",
        "Should have correct name"
    );
}

#[test]
fn test_parse_python_parameters() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_python_parameters.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with parameters should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 6, "Should have 6 nodes");

    // Check first node with simple parameters
    let node = &nodes[0];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "talker_simple_params",
        "Should have correct name"
    );

    let params = node["params"].as_array().unwrap();
    assert!(!params.is_empty(), "Should have parameters");

    // Verify string parameter
    let string_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("string_param")
    });
    assert!(string_param.is_some(), "Should have string_param");
    let string_value = string_param
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert_eq!(string_value, "hello_world", "String param should match");

    // Verify int parameter
    let int_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("int_param")
    });
    assert!(int_param.is_some(), "Should have int_param");
    let int_value = int_param
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert_eq!(int_value, "42", "Int param should match");

    // Verify bool parameter
    let bool_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("bool_param")
    });
    assert!(bool_param.is_some(), "Should have bool_param");
    let bool_value = bool_param
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert_eq!(
        bool_value, "True",
        "Bool param should match (Python convention)"
    );

    // Check second node with nested parameters (uses dot notation)
    let node = &nodes[1];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "listener_nested_params",
        "Should have correct name"
    );

    let params = node["params"].as_array().unwrap();
    assert!(!params.is_empty(), "Should have parameters");

    // Verify nested parameter with dot notation
    let nested_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("robot.name")
    });
    assert!(
        nested_param.is_some(),
        "Should have nested parameter with dot notation"
    );
    let nested_value = nested_param
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert_eq!(nested_value, "my_robot", "Nested param should match");

    // Verify deeply nested parameter
    let deep_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("config.advanced.timeout")
    });
    assert!(deep_param.is_some(), "Should have deeply nested parameter");

    // Check third node with list of parameter dicts
    let node = &nodes[2];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "talker_list_params",
        "Should have correct name"
    );

    let params = node["params"].as_array().unwrap();
    assert_eq!(params.len(), 3, "Should have 3 parameters from list");

    // Check fourth node with mixed parameters (including param file)
    let node = &nodes[3];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "listener_mixed_params",
        "Should have correct name"
    );

    let params = node["params"].as_array().unwrap();
    assert!(!params.is_empty(), "Should have parameters");

    // Verify parameter files are in params_files field (not in params)
    let params_files = node["params_files"].as_array().unwrap();
    assert!(!params_files.is_empty(), "Should have parameter files");

    // Verify parameter file path
    let param_file_path = params_files.first().and_then(|v| v.as_str());
    assert!(param_file_path.is_some(), "Should have parameter file path");
    assert!(
        param_file_path.unwrap().ends_with(".yaml"),
        "Parameter file should be a YAML file"
    );

    // Check fifth node with array parameters
    let node = &nodes[4];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "talker_array_params",
        "Should have correct name"
    );

    let params = node["params"].as_array().unwrap();
    assert!(!params.is_empty(), "Should have array parameters");

    // Verify array parameter is formatted as string
    let joints_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("joints")
    });
    assert!(joints_param.is_some(), "Should have joints array parameter");
    let joints_value = joints_param
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert!(
        joints_value.starts_with('['),
        "Array param should be formatted as string array"
    );

    // Check sixth node with no parameters
    let node = &nodes[5];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "listener_no_params",
        "Should have correct name"
    );
}

#[test]
fn test_parse_python_conditions() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_python_conditions.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test 1: use_sim=false, enable_debug=true (default)
    let mut args = HashMap::new();
    args.insert("use_sim".to_string(), "false".to_string());
    args.insert("enable_debug".to_string(), "true".to_string());

    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with conditions should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();

    // With use_sim=false and enable_debug=true, we should have:
    // - real_talker (UnlessCondition(use_sim) = UnlessCondition(false) = true)
    // - debug_listener (IfCondition(enable_debug) = IfCondition(true) = true)
    // - main_listener (always)
    // NOT sim_talker (IfCondition(use_sim) = IfCondition(false) = false)
    assert_eq!(
        nodes.len(),
        3,
        "Should have 3 nodes with use_sim=false, enable_debug=true"
    );

    // Check we have real_talker
    let has_real_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("real_talker"));
    assert!(
        has_real_talker,
        "Should have real_talker when use_sim=false"
    );

    // Check we have debug_listener
    let has_debug_listener = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("debug_listener"));
    assert!(
        has_debug_listener,
        "Should have debug_listener when enable_debug=true"
    );

    // Check we have main_listener
    let has_main_listener = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("main_listener"));
    assert!(has_main_listener, "Should always have main_listener");

    // Check we DON'T have sim_talker
    let has_sim_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("sim_talker"));
    assert!(
        !has_sim_talker,
        "Should NOT have sim_talker when use_sim=false"
    );

    // Test 2: use_sim=true, enable_debug=false
    let mut args = HashMap::new();
    args.insert("use_sim".to_string(), "true".to_string());
    args.insert("enable_debug".to_string(), "false".to_string());

    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing should succeed with different args");
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // With use_sim=true and enable_debug=false, we should have:
    // - sim_talker (IfCondition(use_sim) = IfCondition(true) = true)
    // - main_listener (always)
    // NOT real_talker (UnlessCondition(use_sim) = UnlessCondition(true) = false)
    // NOT debug_listener (IfCondition(enable_debug) = IfCondition(false) = false)
    assert_eq!(
        nodes.len(),
        2,
        "Should have 2 nodes with use_sim=true, enable_debug=false"
    );

    // Check we have sim_talker
    let has_sim_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("sim_talker"));
    assert!(has_sim_talker, "Should have sim_talker when use_sim=true");

    // Check we DON'T have real_talker
    let has_real_talker = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("real_talker"));
    assert!(
        !has_real_talker,
        "Should NOT have real_talker when use_sim=true"
    );

    // Check we DON'T have debug_listener
    let has_debug_listener = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("debug_listener"));
    assert!(
        !has_debug_listener,
        "Should NOT have debug_listener when enable_debug=false"
    );
}

#[test]
fn test_parse_python_include() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_python_include.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test with custom launch arguments
    let mut args = HashMap::new();
    args.insert("node_name".to_string(), "my_included_node".to_string());
    args.insert("param_value".to_string(), "my_custom_value".to_string());

    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with includes should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();
    let nodes = json["node"].as_array().unwrap();

    // Debug: print node names
    eprintln!("Found {} nodes:", nodes.len());
    for node in nodes {
        eprintln!("  - {}", node["name"].as_str().unwrap_or("no name"));
    }

    // Should have 2 nodes: 1 from main file + 1 from included file
    assert_eq!(nodes.len(), 2, "Should have 2 nodes (1 main + 1 included)");

    // Check for the main node
    let main_node = nodes
        .iter()
        .find(|n| n["name"].as_str() == Some("main_node"));
    assert!(main_node.is_some(), "Should have main_node from main file");
    let main_node = main_node.unwrap();
    assert_eq!(main_node["package"].as_str().unwrap(), "main_pkg");
    assert_eq!(main_node["executable"].as_str().unwrap(), "main_node");
    assert_eq!(main_node["namespace"].as_str().unwrap(), "/main");

    // Check for the included node (name will be resolved)
    let included_node = nodes
        .iter()
        .find(|n| n["package"].as_str() == Some("included_pkg"));
    assert!(
        included_node.is_some(),
        "Should have included_pkg node from included file"
    );
    let included_node = included_node.unwrap();
    assert_eq!(included_node["package"].as_str().unwrap(), "included_pkg");
    assert_eq!(
        included_node["executable"].as_str().unwrap(),
        "included_node"
    );
    assert_eq!(included_node["namespace"].as_str().unwrap(), "/included");
    // The name should be resolved to the passed argument value
    assert_eq!(included_node["name"].as_str().unwrap(), "my_included_node");

    // Check that launch arguments were passed to included file and resolved
    // The parameter value should be resolved to the passed argument value
    let params = included_node["params"].as_array().unwrap();
    let has_param = params.iter().any(|p| {
        let tuple = p.as_array().unwrap();
        tuple[0].as_str() == Some("included_param") && tuple[1].as_str() == Some("my_custom_value")
    });
    assert!(
        has_param,
        "Included node should have included_param with resolved value"
    );
}

#[test]
fn test_python_load_composable_nodes() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_python_load_composable_nodes.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let mut args = HashMap::new();
    args.insert("container_name".to_string(), "test_container".to_string());
    args.insert(
        "full_container_path".to_string(),
        "/test/test_container".to_string(),
    );

    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with LoadComposableNodes should succeed: {:?}",
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
    assert_eq!(containers.len(), 1, "Should have 1 container");

    let container = &containers[0];
    // Container name uses LaunchConfiguration, which Python resolves to the actual value
    assert_eq!(container["name"].as_str().unwrap(), "test_container");
    assert_eq!(container["namespace"].as_str().unwrap(), "/test");

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
    // 1. initial_node (from container)
    // 2. dynamic_node_1 (from LoadComposableNodes with LaunchConfiguration)
    // 3. dynamic_node_2 (from LoadComposableNodes with LaunchConfiguration, custom namespace)
    // 4. string_node (from LoadComposableNodes with string reference)
    assert_eq!(load_nodes.len(), 4, "Should have 4 load_nodes total");

    // Check initial_node
    let initial_node = load_nodes
        .iter()
        .find(|n| n["node_name"].as_str() == Some("initial_node"));
    assert!(initial_node.is_some(), "Should have initial_node");
    let initial_node = initial_node.unwrap();
    // target_container_name is constructed from container namespace + name
    // Python resolves LaunchConfiguration to actual value
    assert_eq!(
        initial_node["target_container_name"].as_str().unwrap(),
        "/test/test_container"
    );
    assert_eq!(initial_node["namespace"].as_str().unwrap(), "/test");

    // Check dynamic_node_1 (LaunchConfiguration target with full path)
    let dynamic_node_1 = load_nodes
        .iter()
        .find(|n| n["node_name"].as_str() == Some("dynamic_node_1"));
    assert!(dynamic_node_1.is_some(), "Should have dynamic_node_1");
    let dynamic_node_1 = dynamic_node_1.unwrap();
    // LoadComposableNodes.target_container resolves LaunchConfiguration to actual value (Python behavior)
    assert_eq!(
        dynamic_node_1["target_container_name"].as_str().unwrap(),
        "/test/test_container",
        "target_container with LaunchConfiguration should be resolved to actual value"
    );
    assert_eq!(dynamic_node_1["namespace"].as_str().unwrap(), "/test");
    assert_eq!(dynamic_node_1["package"].as_str().unwrap(), "dynamic_pkg1");

    // Check dynamic_node_2 (custom namespace)
    let dynamic_node_2 = load_nodes
        .iter()
        .find(|n| n["node_name"].as_str() == Some("dynamic_node_2"));
    assert!(dynamic_node_2.is_some(), "Should have dynamic_node_2");
    let dynamic_node_2 = dynamic_node_2.unwrap();
    assert_eq!(
        dynamic_node_2["target_container_name"].as_str().unwrap(),
        "/test/test_container",
        "target_container with LaunchConfiguration should be resolved to actual value"
    );
    assert_eq!(dynamic_node_2["namespace"].as_str().unwrap(), "/custom_ns");

    // Check string_node (string target with absolute path)
    let string_node = load_nodes
        .iter()
        .find(|n| n["node_name"].as_str() == Some("string_node"));
    assert!(string_node.is_some(), "Should have string_node");
    let string_node = string_node.unwrap();
    assert_eq!(
        string_node["target_container_name"].as_str().unwrap(),
        "/test/my_container"
    );
    // Namespace should be inherited from container ("/test")
    assert_eq!(string_node["namespace"].as_str().unwrap(), "/test");
    assert_eq!(string_node["package"].as_str().unwrap(), "string_pkg");
}

#[test]
fn test_opaque_function() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_opaque_function.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let mut args = HashMap::new();
    args.insert("node_count".to_string(), "5".to_string());

    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with OpaqueFunction should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Check nodes - should have 5 dynamically generated nodes
    let nodes = json["node"].as_array().unwrap();
    eprintln!("Found {} nodes:", nodes.len());
    for node in nodes {
        eprintln!(
            "  - {} (package: {}, namespace: {})",
            node["name"].as_str().unwrap_or("no name"),
            node["package"].as_str().unwrap_or("no package"),
            node["namespace"].as_str().unwrap_or("no namespace")
        );
    }

    assert_eq!(nodes.len(), 5, "Should have 5 dynamically generated nodes");

    // Check that all nodes were created correctly
    for i in 0..5 {
        let node_name = format!("dynamic_node_{}", i);
        let pkg_name = format!("pkg_{}", i);

        let node = nodes
            .iter()
            .find(|n| n["name"].as_str() == Some(&node_name));
        assert!(node.is_some(), "Should have node {}", node_name);

        let node = node.unwrap();
        assert_eq!(node["package"].as_str().unwrap(), pkg_name);
        assert_eq!(node["executable"].as_str().unwrap(), "exec");
        assert_eq!(node["namespace"].as_str().unwrap(), "/dynamic");
    }
}

#[test]
fn test_opaque_function_file_io() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_opaque_file_io.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing Python launch file with OpaqueFunction file I/O should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Check nodes - should have 1 node with parameters read from YAML file
    let nodes = json["node"].as_array().unwrap();
    eprintln!("Found {} nodes:", nodes.len());
    for node in nodes {
        eprintln!(
            "  - {} (package: {}, namespace: {})",
            node["name"].as_str().unwrap_or("no name"),
            node["package"].as_str().unwrap_or("no package"),
            node["namespace"].as_str().unwrap_or("no namespace")
        );
    }

    assert_eq!(nodes.len(), 1, "Should have 1 node");

    let node = &nodes[0];
    assert_eq!(node["name"].as_str().unwrap(), "test_node_with_file_params");
    assert_eq!(node["package"].as_str().unwrap(), "test_pkg");
    assert_eq!(node["executable"].as_str().unwrap(), "test_exec");
    assert_eq!(node["namespace"].as_str().unwrap(), "/test");

    // Check that parameters were read from the YAML file
    let params = node["params"].as_array().unwrap();
    eprintln!("Found {} params:", params.len());
    for param in params {
        if let Some(arr) = param.as_array() {
            eprintln!(
                "  - {}: {}",
                arr[0].as_str().unwrap_or("no key"),
                arr[1].as_str().unwrap_or("no value")
            );
        }
    }

    assert!(!params.is_empty(), "Should have parameters from YAML file");

    // Check test_param (should be 42)
    let test_param = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("test_param")
    });
    assert!(test_param.is_some(), "Should have test_param from YAML");
    let test_param_value = test_param
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert_eq!(test_param_value, "42", "test_param should be 42");

    // Check test_string (should be "hello")
    let test_string = params.iter().find(|p| {
        p.as_array()
            .and_then(|arr| arr.first())
            .and_then(|v| v.as_str())
            == Some("test_string")
    });
    assert!(test_string.is_some(), "Should have test_string from YAML");
    let test_string_value = test_string
        .unwrap()
        .as_array()
        .unwrap()
        .get(1)
        .unwrap()
        .as_str()
        .unwrap();
    assert_eq!(test_string_value, "hello", "test_string should be 'hello'");
}

#[test]
fn test_opaque_function_conditional_nodes() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_opaque_conditional.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    // Test with mode=minimal (should create only base_node)
    let mut args = HashMap::new();
    args.insert("mode".to_string(), "minimal".to_string());
    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing should succeed: {:?}", result.err());
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node with mode=minimal");
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "conditional_base");

    // Test with mode=full (should create base_node + extra_node)
    let mut args = HashMap::new();
    args.insert("mode".to_string(), "full".to_string());
    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing should succeed: {:?}", result.err());
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 2, "Should have 2 nodes with mode=full");

    let node_names: Vec<&str> = nodes.iter().map(|n| n["name"].as_str().unwrap()).collect();
    assert!(node_names.contains(&"conditional_base"));
    assert!(node_names.contains(&"conditional_extra"));
}

#[test]
fn test_list_concatenation_in_substitutions() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_list_concatenation.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);
    assert!(result.is_ok(), "Parsing should succeed: {:?}", result.err());
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node");

    // Note: We can't fully test the concatenated paths without ROS env,
    // but we verify the file parses and captures the node structure
    let node = &nodes[0];
    assert_eq!(node["name"].as_str().unwrap(), "list_concat_node");
    assert!(node["params"].is_array(), "Should have params");
}

#[test]
fn test_parameter_file_usage() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_parameter_file.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    // This test verifies that ParameterFile is properly mocked and doesn't crash
    // In a real ROS environment, ParameterFile would load params from the file
    assert!(result.is_ok(), "Parsing should succeed: {:?}", result.err());
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node");
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "param_file_node");
}

#[test]
fn test_include_with_list_arguments() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("test_include_with_list_args.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    // The include target doesn't exist, so we expect an error
    // But we verify the file parses and the IncludeLaunchDescription is captured
    // (it will fail during include processing, which is expected)
    assert!(result.is_err(), "Should fail to find included file");

    // The error message should indicate it's trying to include a file
    let err_msg = format!("{:?}", result.err().unwrap());
    assert!(
        err_msg.contains("params.launch.py") || err_msg.contains("No such file"),
        "Error should mention the included file"
    );
}

// ============================================================================
// Autoware-inspired edge case tests
// ============================================================================

#[test]
fn test_namespace_sync_xml_python() {
    let _guard = python_test_guard();

    // Test: XML <push-ros-namespace> is visible to Python OpaqueFunction
    // This replicates Autoware's system.launch.xml including component_state_monitor.launch.py
    let fixture = get_fixture_path("test_namespace_sync_xml_python.launch.xml");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Should parse successfully: {:?}",
        result.err()
    );
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Verify the node was created with the namespace from XML context
    let nodes = json["node"].as_array().unwrap();
    assert!(!nodes.is_empty(), "Should have at least one node");

    let node = &nodes[0];
    assert_eq!(
        node["name"].as_str().unwrap(),
        "context_aware_node",
        "Node should have the expected name"
    );
    assert_eq!(
        node["namespace"].as_str().unwrap(),
        "/system",
        "Node should have namespace from XML context"
    );
}

#[test]
fn test_list_namespace_concatenation() {
    let _guard = python_test_guard();

    // Test: List concatenation in namespace field (Autoware pattern)
    // namespace=["/", "my_container"]
    let fixture = get_fixture_path("python/list_namespace_concatenation.launch.py");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Should parse successfully: {:?}",
        result.err()
    );
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Verify container namespace was concatenated correctly
    let containers = json["container"].as_array().unwrap();
    assert_eq!(containers.len(), 1, "Should have one container");

    let container = &containers[0];
    assert_eq!(
        container["namespace"].as_str().unwrap(),
        "/my_container",
        "Container namespace should be concatenated from list ['/\", \"my_container\"]"
    );

    // Verify composable node also has concatenated namespace
    let load_nodes = json["load_node"].as_array().unwrap();
    assert_eq!(load_nodes.len(), 1, "Should have one composable node");

    let load_node = &load_nodes[0];
    assert_eq!(
        load_node["namespace"].as_str().unwrap(),
        "/my_container",
        "Composable node namespace should be concatenated from list [\"my\", \"_\", \"container\"] with leading slash"
    );
}

#[test]
fn test_opaque_xml_namespace_preservation() {
    let _guard = python_test_guard();

    // Test: XML includes returned from OpaqueFunction preserve namespace
    // This is the core Autoware component_state_monitor pattern:
    // 1. XML pushes /system namespace
    // 2. Python OpaqueFunction returns XML includes
    // 3. Those XML includes should get /system namespace applied
    let fixture = get_fixture_path("test_opaque_xml_namespace.launch.xml");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let fixture_dir = get_fixture_path("");
    let mut args = HashMap::new();
    args.insert(
        "fixture_dir".to_string(),
        fixture_dir.to_str().unwrap().to_string(),
    );

    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Should parse successfully: {:?}",
        result.err()
    );
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Verify container was created in /system namespace
    let containers = json["container"].as_array().unwrap();
    assert!(!containers.is_empty(), "Should have at least one container");

    let container = &containers[0];
    assert_eq!(
        container["namespace"].as_str().unwrap(),
        "/system/monitor",
        "Container should be in /system/monitor namespace"
    );

    // Verify composable nodes from XML includes target the correctly-namespaced container
    let load_nodes = json["load_node"].as_array().unwrap();
    assert!(
        load_nodes.len() >= 2,
        "Should have at least 2 composable nodes from XML includes"
    );

    for load_node in load_nodes.iter() {
        let target = load_node["target_container_name"].as_str().unwrap();
        assert!(
            target.starts_with("/system/"),
            "Composable nodes should target container in /system namespace, got: {}",
            target
        );
    }
}

#[test]
fn test_debug_namespace_context() {
    let _guard = python_test_guard();

    // Simple test to debug namespace context passing
    // Flow: XML pushes /test_ns → Python OpaqueFunction → XML include with load_node
    let fixture = get_fixture_path("debug_ns_main.launch.xml");
    let fixture_dir = get_fixture_path("");

    let mut args = HashMap::new();
    args.insert(
        "fixture_dir".to_string(),
        fixture_dir.to_str().unwrap().to_string(),
    );

    let result = parse_launch_file(&fixture, args);
    if let Err(ref e) = result {
        eprintln!("Parse error: {}", e);
        eprintln!("Error debug: {:?}", e);
    }
    assert!(
        result.is_ok(),
        "Should parse successfully: {:?}",
        result.err()
    );

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Debug: print what we got
    eprintln!("\n=== DEBUG OUTPUT ===");
    eprintln!(
        "Containers: {}",
        serde_json::to_string_pretty(&json["container"]).unwrap()
    );
    eprintln!(
        "Load nodes: {}",
        serde_json::to_string_pretty(&json["load_node"]).unwrap()
    );

    let containers = json["container"].as_array().unwrap();
    eprintln!("Container count: {}", containers.len());

    let load_nodes = json["load_node"].as_array().unwrap();
    eprintln!("Load node count: {}", load_nodes.len());

    // Check what namespace we got
    if !load_nodes.is_empty() {
        let target = load_nodes[0]["target_container_name"].as_str().unwrap();
        eprintln!("First load_node target: {}", target);
        eprintln!("Expected: should start with /test_ns/");
        eprintln!(
            "Actual starts with /test_ns/: {}",
            target.starts_with("/test_ns/")
        );
    }
}

#[test]
fn test_utilities_functions() {
    let _guard = python_test_guard();

    // Test: launch_ros.utilities functions (make_namespace_absolute, prefix_namespace)
    let fixture = get_fixture_path("python/test_utilities.launch.py");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Should parse successfully: {:?}",
        result.err()
    );
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Verify nodes were created with computed namespaces
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 2, "Should have 2 nodes");

    // First node uses make_namespace_absolute("robot1") -> "/robot1"
    let node1 = &nodes[0];
    assert_eq!(node1["name"].as_str().unwrap(), "test_abs");
    assert_eq!(
        node1["namespace"].as_str().unwrap(),
        "/robot1",
        "make_namespace_absolute should add leading slash"
    );

    // Second node uses prefix_namespace("/system", "monitor") -> "/system/monitor"
    let node2 = &nodes[1];
    assert_eq!(node2["name"].as_str().unwrap(), "test_prefixed");
    assert_eq!(
        node2["namespace"].as_str().unwrap(),
        "/system/monitor",
        "prefix_namespace should combine namespaces correctly"
    );
}

#[test]
fn test_autoware_patterns_combined() {
    let _guard = python_test_guard();

    // Test: Multiple Autoware patterns in one launch file
    // This combines: nested namespaces, OpaqueFunction, XML includes, list concatenation
    let fixture = get_fixture_path("test_autoware_patterns.launch.xml");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let fixture_dir = get_fixture_path("");
    let mut args = HashMap::new();
    args.insert(
        "fixture_dir".to_string(),
        fixture_dir.to_str().unwrap().to_string(),
    );

    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Should parse successfully: {:?}",
        result.err()
    );
    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes from multiple sources
    let nodes = json["node"].as_array().unwrap();
    assert!(
        nodes.len() >= 3,
        "Should have nodes from nested namespaces and utilities test"
    );

    // Verify we have containers from different tests
    let containers = json["container"].as_array().unwrap();
    assert!(
        containers.len() >= 2,
        "Should have containers from list concat and opaque tests"
    );

    // Verify namespace scoping worked correctly
    let has_vehicle_sensors_ns = nodes
        .iter()
        .any(|n| n["namespace"].as_str() == Some("/vehicle/sensors"));
    assert!(
        has_vehicle_sensors_ns,
        "Should have node in /vehicle/sensors namespace from nested groups"
    );

    let has_system_ns_container = containers.iter().any(|c| {
        c["namespace"]
            .as_str()
            .is_some_and(|ns| ns.starts_with("/system"))
    });
    assert!(
        has_system_ns_container,
        "Should have container in /system namespace from OpaqueFunction test"
    );
}

#[test]
fn test_conditional_substitutions() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("python/test_conditional_substitutions.launch.py");

    // Get the fixtures directory path for FileContent test data files
    let fixture_dir =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/launch/python");
    let fixture_dir_str = fixture_dir.to_str().unwrap();

    let mut args = HashMap::new();
    args.insert("config_dir".to_string(), fixture_dir_str.to_string());
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Should parse conditional substitutions test: {:?}",
        result.err()
    );

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Get nodes
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 9, "Should have 9 nodes");

    // Test EqualsSubstitution - mode == 'debug' -> true
    // Conditional substitutions now evaluate correctly by calling perform() with real context
    let node_equals_true = nodes
        .iter()
        .find(|n| n["name"] == "node_equals_true")
        .unwrap();
    assert_eq!(
        node_equals_true["namespace"].as_str().unwrap(),
        "/true",
        "EqualsSubstitution should return 'true' when values match"
    );

    // Test EqualsSubstitution - mode == 'release' -> false
    let node_equals_false = nodes
        .iter()
        .find(|n| n["name"] == "node_equals_false")
        .unwrap();
    assert_eq!(
        node_equals_false["namespace"].as_str().unwrap(),
        "/false",
        "EqualsSubstitution should return 'false' for non-matching values"
    );

    // Test NotEqualsSubstitution - mode != 'release' -> true
    // NOTE: Currently evaluates incorrectly due to substitution preservation
    let node_not_equals_true = nodes
        .iter()
        .find(|n| n["name"] == "node_not_equals_true")
        .unwrap();
    assert_eq!(
        node_not_equals_true["namespace"].as_str().unwrap(),
        "/true", // This one happens to work because "$(var mode)" != "release"
        "NotEqualsSubstitution"
    );

    // Test NotEqualsSubstitution - mode != 'debug' -> false
    // Conditional substitutions now evaluate correctly
    let node_not_equals_false = nodes
        .iter()
        .find(|n| n["name"] == "node_not_equals_false")
        .unwrap();
    assert_eq!(
        node_not_equals_false["namespace"].as_str().unwrap(),
        "/false",
        "NotEqualsSubstitution should return 'false' when values are equal"
    );

    // Test IfElseSubstitution - if mode == 'debug' then '/debug_ns' else '/release_ns'
    // Conditional substitutions now evaluate correctly
    let node_ifelse_true = nodes
        .iter()
        .find(|n| n["name"] == "node_ifelse_true")
        .unwrap();
    assert_eq!(
        node_ifelse_true["namespace"].as_str().unwrap(),
        "/debug_ns",
        "IfElseSubstitution should return if_value when condition is true"
    );

    // Test IfElseSubstitution with enable_feature
    let node_ifelse_with_config = nodes
        .iter()
        .find(|n| n["name"] == "node_ifelse_with_config")
        .unwrap();
    assert_eq!(
        node_ifelse_with_config["namespace"].as_str().unwrap(),
        "/enabled",
        "IfElseSubstitution with enable_feature should evaluate correctly"
    );

    // Test nested IfElseSubstitution
    let node_nested_ifelse = nodes
        .iter()
        .find(|n| n["name"] == "node_nested_ifelse")
        .unwrap();
    assert_eq!(
        node_nested_ifelse["namespace"].as_str().unwrap(),
        "/debug_enabled",
        "Nested IfElseSubstitution should evaluate correctly"
    );

    // Test FileContent with PathJoinSubstitution (config_dir + test_content.txt)
    let node_file_content_simple = nodes
        .iter()
        .find(|n| n["name"] == "node_file_content_simple")
        .expect("node_file_content_simple should exist");

    // FileContent reads from <fixtures_dir>/test_content.txt which contains "/file_content_namespace"
    assert_eq!(
        node_file_content_simple["namespace"].as_str().unwrap(),
        "/file_content_namespace",
        "FileContent should read file content correctly"
    );

    // Test FileContent with PathJoinSubstitution
    let node_file_content_path_join = nodes
        .iter()
        .find(|n| n["name"] == "node_file_content_path_join")
        .expect("node_file_content_path_join should exist");

    // FileContent(PathJoinSubstitution([LaunchConfiguration('config_dir'), 'namespace.txt']))
    // Should resolve to: FileContent('<fixtures_dir>/namespace.txt') which contains "/config_namespace"
    assert_eq!(
        node_file_content_path_join["namespace"].as_str().unwrap(),
        "/config_namespace",
        "FileContent with PathJoinSubstitution should resolve nested substitutions and read file"
    );
}

#[test]
fn test_set_parameters_from_file() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("python/test_set_parameters_from_file.launch.py");

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Should parse SetParametersFromFile test: {:?}",
        result.err()
    );

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Get nodes - should have 3 nodes
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 3, "Should have 3 nodes");

    // Verify all nodes are created correctly
    let node1 = nodes.iter().find(|n| n["name"] == "node1").unwrap();
    assert_eq!(node1["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node1["executable"].as_str().unwrap(), "talker");
    assert_eq!(node1["namespace"].as_str().unwrap(), "/test");

    let node2 = nodes.iter().find(|n| n["name"] == "node2").unwrap();
    assert_eq!(node2["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node2["executable"].as_str().unwrap(), "listener");
    assert_eq!(node2["namespace"].as_str().unwrap(), "/test");

    let node3 = nodes.iter().find(|n| n["name"] == "node3").unwrap();
    assert_eq!(node3["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node3["executable"].as_str().unwrap(), "talker");
    assert_eq!(node3["namespace"].as_str().unwrap(), "/test");

    // Note: SetParametersFromFile is a launch action that doesn't directly affect
    // the node records in our static analysis. It would apply parameters at runtime.
    // The test verifies that the action doesn't break parsing and nodes are captured correctly.
}

#[test]
fn test_lifecycle_nodes() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("python/test_lifecycle_nodes.launch.py");

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Should parse lifecycle nodes test: {:?}",
        result.err()
    );

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Get nodes - should have 4 lifecycle nodes
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 4, "Should have 4 lifecycle nodes");

    // Test 1: Basic lifecycle node
    let lifecycle_node_1 = nodes
        .iter()
        .find(|n| n["name"] == "lifecycle_node_1")
        .unwrap();
    assert_eq!(
        lifecycle_node_1["package"].as_str().unwrap(),
        "lifecycle_pkg"
    );
    assert_eq!(
        lifecycle_node_1["executable"].as_str().unwrap(),
        "lifecycle_node"
    );
    assert_eq!(lifecycle_node_1["namespace"].as_str().unwrap(), "/test");

    // Test 2: Lifecycle node with parameters and remappings
    let lifecycle_node_2 = nodes
        .iter()
        .find(|n| n["name"] == "lifecycle_node_2")
        .unwrap();
    assert_eq!(
        lifecycle_node_2["package"].as_str().unwrap(),
        "lifecycle_pkg"
    );

    // Check parameters
    let params = lifecycle_node_2["params"].as_array().unwrap();
    assert!(
        params.len() >= 2,
        "Should have at least 2 parameters (use_sim_time, param1, param2)"
    );

    // Check for specific parameters
    let param_names: Vec<String> = params
        .iter()
        .map(|p| p[0].as_str().unwrap().to_string())
        .collect();
    assert!(
        param_names.contains(&"param1".to_string()),
        "Should have param1"
    );
    assert!(
        param_names.contains(&"param2".to_string()),
        "Should have param2"
    );

    // Check remappings
    let remaps = lifecycle_node_2["remaps"].as_array().unwrap();
    assert_eq!(remaps.len(), 2, "Should have 2 remappings");
    assert_eq!(remaps[0][0].as_str().unwrap(), "/input");
    assert_eq!(remaps[0][1].as_str().unwrap(), "/remapped_input");
    assert_eq!(remaps[1][0].as_str().unwrap(), "/output");
    assert_eq!(remaps[1][1].as_str().unwrap(), "/remapped_output");

    // Test 3: Lifecycle node with LaunchConfiguration for name
    // After process_launch_arguments(), the name should be resolved to the default value
    let lifecycle_node_dynamic = nodes
        .iter()
        .find(|n| n["name"] == "my_lifecycle_node")
        .unwrap();
    assert_eq!(
        lifecycle_node_dynamic["package"].as_str().unwrap(),
        "lifecycle_pkg"
    );

    // Check arguments
    let args_array = lifecycle_node_dynamic["args"].as_array().unwrap();
    assert!(args_array.len() >= 3, "Should have at least 3 arguments");

    // Test 4: Lifecycle node with output specification
    let lifecycle_node_3 = nodes
        .iter()
        .find(|n| n["name"] == "lifecycle_node_3")
        .unwrap();
    assert_eq!(
        lifecycle_node_3["package"].as_str().unwrap(),
        "lifecycle_pkg"
    );

    // Note: LifecycleTransition actions don't create node records in static analysis.
    // They're just captured as actions that would trigger state transitions at runtime.
    // The test verifies that the actions don't break parsing and lifecycle nodes are captured correctly.
}

#[test]
fn test_environment_management() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("python/test_environment_management.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing environment management launch file should succeed: {:?}",
        result.err()
    );

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Get nodes - should have 11 nodes from the environment management test
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 11, "Should have 11 nodes");

    // Test 1: Node with modified environment (after first SetEnvironmentVariable)
    let node_modified = nodes
        .iter()
        .find(|n| n["name"] == "node_with_modified_env")
        .unwrap();
    assert_eq!(node_modified["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_modified["executable"].as_str().unwrap(), "talker");

    // Test 2: Node with restored environment (after PopEnvironment)
    let node_restored = nodes
        .iter()
        .find(|n| n["name"] == "node_with_restored_env")
        .unwrap();
    assert_eq!(node_restored["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_restored["executable"].as_str().unwrap(), "listener");

    // Test 3: Node with appended PATH
    let node_appended = nodes
        .iter()
        .find(|n| n["name"] == "node_with_appended_path")
        .unwrap();
    assert_eq!(node_appended["package"].as_str().unwrap(), "demo_nodes_cpp");

    // Test 4: Node with prepended LD_LIBRARY_PATH
    let node_prepended = nodes
        .iter()
        .find(|n| n["name"] == "node_with_prepended_lib")
        .unwrap();
    assert_eq!(
        node_prepended["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );

    // Test 5: Nested environment test - level 2
    let node_level_2 = nodes
        .iter()
        .find(|n| n["name"] == "node_at_level_2")
        .unwrap();
    assert_eq!(node_level_2["package"].as_str().unwrap(), "demo_nodes_cpp");

    // Test 6: Nested environment test - level 1 (after first pop)
    let node_level_1 = nodes
        .iter()
        .find(|n| n["name"] == "node_at_level_1")
        .unwrap();
    assert_eq!(node_level_1["package"].as_str().unwrap(), "demo_nodes_cpp");

    // Test 7: Nested environment test - level 0 (after second pop)
    let node_level_0 = nodes
        .iter()
        .find(|n| n["name"] == "node_at_level_0")
        .unwrap();
    assert_eq!(node_level_0["package"].as_str().unwrap(), "demo_nodes_cpp");

    // Test 8: Node after ResetEnvironment
    let node_reset = nodes
        .iter()
        .find(|n| n["name"] == "node_after_reset")
        .unwrap();
    assert_eq!(node_reset["package"].as_str().unwrap(), "demo_nodes_cpp");

    // Test 9: Node with custom list separator
    let node_custom = nodes
        .iter()
        .find(|n| n["name"] == "node_with_custom_list")
        .unwrap();
    assert_eq!(node_custom["package"].as_str().unwrap(), "demo_nodes_cpp");

    // Test 10: Node with unset variable (inside push/pop)
    let node_unset = nodes
        .iter()
        .find(|n| n["name"] == "node_with_unset_var")
        .unwrap();
    assert_eq!(node_unset["package"].as_str().unwrap(), "demo_nodes_cpp");

    // Test 11: Node with restored variable (after pop)
    let node_var_restored = nodes
        .iter()
        .find(|n| n["name"] == "node_with_restored_var")
        .unwrap();
    assert_eq!(
        node_var_restored["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );

    // Note: Environment management actions (PushEnvironment, PopEnvironment, etc.)
    // don't produce output records in static analysis. They would affect the runtime
    // environment but we're only doing static parsing. The test verifies that:
    // 1. The actions parse correctly without errors
    // 2. All nodes are captured properly
    // 3. The parser doesn't crash on these actions
}

#[test]
fn test_launch_config_management() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("python/test_launch_config_management.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing launch config management file should succeed: {:?}",
        result.err()
    );

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Get nodes - should have 11 nodes from the launch config management test
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 11, "Should have 11 nodes");

    // Test 1: Node with temporary config (inside push/pop)
    let node_temp = nodes
        .iter()
        .find(|n| n["name"] == "node_with_temp_config")
        .unwrap();
    assert_eq!(node_temp["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_temp["executable"].as_str().unwrap(), "talker");

    // Test 2: Node with restored config (after pop)
    let node_restored = nodes
        .iter()
        .find(|n| n["name"] == "node_with_restored_config")
        .unwrap();
    assert_eq!(node_restored["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_restored["executable"].as_str().unwrap(), "listener");

    // Test 3: Nested config test - level 2
    let node_level_2 = nodes
        .iter()
        .find(|n| n["name"] == "node_at_config_level_2")
        .unwrap();
    assert_eq!(node_level_2["package"].as_str().unwrap(), "demo_nodes_cpp");

    // Test 4: Nested config test - level 1 (after first pop)
    let node_level_1 = nodes
        .iter()
        .find(|n| n["name"] == "node_at_config_level_1")
        .unwrap();
    assert_eq!(node_level_1["package"].as_str().unwrap(), "demo_nodes_cpp");

    // Test 5: Nested config test - level 0 (after second pop)
    let node_level_0 = nodes
        .iter()
        .find(|n| n["name"] == "node_at_config_level_0")
        .unwrap();
    assert_eq!(node_level_0["package"].as_str().unwrap(), "demo_nodes_cpp");

    // Test 6: Node with unset config (inside push/pop)
    let node_unset = nodes
        .iter()
        .find(|n| n["name"] == "node_with_unset_config")
        .unwrap();
    assert_eq!(node_unset["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_unset["namespace"].as_str().unwrap(), "/unset_test");

    // Test 7: Node with restored config3 (after pop)
    let node_restored3 = nodes
        .iter()
        .find(|n| n["name"] == "node_with_restored_config3")
        .unwrap();
    assert_eq!(
        node_restored3["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );

    // Test 8: Node after ResetLaunchConfigurations
    let node_reset = nodes
        .iter()
        .find(|n| n["name"] == "node_after_reset")
        .unwrap();
    assert_eq!(node_reset["package"].as_str().unwrap(), "demo_nodes_cpp");

    // Test 9: Node after multiple unsets
    let node_unsets = nodes
        .iter()
        .find(|n| n["name"] == "node_after_unsets")
        .unwrap();
    assert_eq!(node_unsets["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_unsets["namespace"].as_str().unwrap(), "/after_unsets");

    // Test 10: Node in inner scope
    let node_inner = nodes
        .iter()
        .find(|n| n["name"] == "node_in_inner_scope")
        .unwrap();
    assert_eq!(node_inner["package"].as_str().unwrap(), "demo_nodes_cpp");

    // Test 11: Node in outer scope (after pop)
    let node_outer = nodes
        .iter()
        .find(|n| n["name"] == "node_in_outer_scope")
        .unwrap();
    assert_eq!(node_outer["package"].as_str().unwrap(), "demo_nodes_cpp");

    // Note: Launch configuration management actions (PushLaunchConfigurations, etc.)
    // don't produce output records in static analysis. They would affect the runtime
    // launch configuration state but we're only doing static parsing. The test verifies that:
    // 1. The actions parse correctly without errors
    // 2. All nodes are captured properly
    // 3. The parser doesn't crash on these actions
}

#[test]
fn test_additional_substitutions() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("python/test_additional_substitutions.launch.py");
    assert!(fixture.exists(), "Fixture file should exist: {:?}", fixture);

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing additional substitutions file should succeed: {:?}",
        result.err()
    );

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Get nodes - should have 8 nodes from the additional substitutions test
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 8, "Should have 8 nodes");

    // Test 1: Node with ExecutableInPackage (static)
    let node_exec_static = nodes
        .iter()
        .find(|n| n["name"] == "node_with_exec_in_pkg_static")
        .unwrap();
    assert_eq!(
        node_exec_static["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );

    // Test 2: Node with ExecutableInPackage (dynamic)
    let node_exec_dynamic = nodes
        .iter()
        .find(|n| n["name"] == "node_with_exec_in_pkg_dynamic")
        .unwrap();
    assert_eq!(
        node_exec_dynamic["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );

    // Test 3: Node with FindPackage (static)
    let node_find_static = nodes
        .iter()
        .find(|n| n["name"] == "node_with_find_pkg_static")
        .unwrap();
    assert_eq!(
        node_find_static["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );
    assert_eq!(node_find_static["executable"].as_str().unwrap(), "listener");

    // Test 4: Node with FindPackage (dynamic)
    let node_find_dynamic = nodes
        .iter()
        .find(|n| n["name"] == "node_with_find_pkg_dynamic")
        .unwrap();
    assert_eq!(
        node_find_dynamic["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );
    assert_eq!(node_find_dynamic["executable"].as_str().unwrap(), "talker");

    // Test 5: Node with Parameter (static)
    let node_param_static = nodes
        .iter()
        .find(|n| n["name"] == "node_with_param_static")
        .unwrap();
    assert_eq!(
        node_param_static["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );
    assert_eq!(
        node_param_static["executable"].as_str().unwrap(),
        "listener"
    );

    // Test 6: Node with Parameter (dynamic)
    let node_param_dynamic = nodes
        .iter()
        .find(|n| n["name"] == "node_with_param_dynamic")
        .unwrap();
    assert_eq!(
        node_param_dynamic["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );
    assert_eq!(node_param_dynamic["executable"].as_str().unwrap(), "talker");

    // Test 7: Node with combined ExecutableInPackage
    let node_combined_exec = nodes
        .iter()
        .find(|n| n["name"] == "node_combined_exec")
        .unwrap();
    // Package uses LaunchConfiguration — resolved from context when available
    assert_eq!(
        node_combined_exec["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );

    // Test 8: Node with combined FindPackage and Parameter
    let node_combined = nodes
        .iter()
        .find(|n| n["name"] == "node_combined_pkg_param")
        .unwrap();
    assert_eq!(node_combined["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_combined["executable"].as_str().unwrap(), "listener");

    // Note: ExecutableInPackage, FindPackage, and Parameter substitutions
    // return placeholder values in static analysis since we can't resolve
    // actual package paths or parameter values without runtime context.
    // The test verifies that:
    // 1. The substitutions parse correctly without errors
    // 2. All nodes are captured properly
    // 3. The parser handles these substitutions gracefully
}

#[test]
fn test_utility_substitutions() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("python/test_utility_substitutions.launch.py");

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing utility substitutions file should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 10, "Should have 10 nodes");

    // Test 1: Node with BooleanSubstitution (true)
    let node_bool_true = nodes
        .iter()
        .find(|n| n["name"] == "node_with_bool_true")
        .unwrap();
    assert_eq!(
        node_bool_true["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );
    assert_eq!(node_bool_true["executable"].as_str().unwrap(), "talker");

    // Test 2: Node with BooleanSubstitution (false)
    let node_bool_false = nodes
        .iter()
        .find(|n| n["name"] == "node_with_bool_false")
        .unwrap();
    assert_eq!(
        node_bool_false["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );
    assert_eq!(node_bool_false["executable"].as_str().unwrap(), "listener");

    // Test 3: Node with BooleanSubstitution (1 -> true)
    let node_bool_1 = nodes
        .iter()
        .find(|n| n["name"] == "node_with_bool_1")
        .unwrap();
    assert_eq!(node_bool_1["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_bool_1["executable"].as_str().unwrap(), "talker");

    // Test 4: Node with BooleanSubstitution (0 -> false)
    let node_bool_0 = nodes
        .iter()
        .find(|n| n["name"] == "node_with_bool_0")
        .unwrap();
    assert_eq!(node_bool_0["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_bool_0["executable"].as_str().unwrap(), "listener");

    // Test 5: Node with BooleanSubstitution (dynamic)
    let node_bool_dynamic = nodes
        .iter()
        .find(|n| n["name"] == "node_with_bool_dynamic")
        .unwrap();
    assert_eq!(
        node_bool_dynamic["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );
    assert_eq!(node_bool_dynamic["executable"].as_str().unwrap(), "talker");

    // Test 6: Node with FindExecutable (static)
    let node_find_exec_static = nodes
        .iter()
        .find(|n| n["name"] == "node_with_find_exec_static")
        .unwrap();
    assert_eq!(
        node_find_exec_static["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );
    assert_eq!(
        node_find_exec_static["executable"].as_str().unwrap(),
        "listener"
    );

    // Test 7: Node with FindExecutable (dynamic)
    let node_find_exec_dynamic = nodes
        .iter()
        .find(|n| n["name"] == "node_with_find_exec_dynamic")
        .unwrap();
    assert_eq!(
        node_find_exec_dynamic["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );
    assert_eq!(
        node_find_exec_dynamic["executable"].as_str().unwrap(),
        "talker"
    );

    // Test 8: Node with LaunchLogDir
    let node_log_dir = nodes
        .iter()
        .find(|n| n["name"] == "node_with_log_dir")
        .unwrap();
    assert_eq!(node_log_dir["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_log_dir["executable"].as_str().unwrap(), "listener");

    // Test 9: Node with ThisLaunchFile
    let node_this_file = nodes
        .iter()
        .find(|n| n["name"] == "node_with_this_file")
        .unwrap();
    assert_eq!(
        node_this_file["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );
    assert_eq!(node_this_file["executable"].as_str().unwrap(), "talker");

    // Test 10: Node with BooleanSubstitution ("yes" -> "true")
    let node_bool_yes = nodes
        .iter()
        .find(|n| n["name"] == "node_combined_bool_yes")
        .unwrap();
    assert_eq!(node_bool_yes["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_bool_yes["executable"].as_str().unwrap(), "listener");

    // Note: BooleanSubstitution, FindExecutable, LaunchLogDir, and ThisLaunchFile
    // return placeholder values in static analysis:
    // - BooleanSubstitution converts values to "true"/"false" strings
    // - FindExecutable returns $(find-exec <name>) placeholder
    // - LaunchLogDir returns $(launch-log-dir) placeholder
    // - ThisLaunchFile returns $(this-launch-file) placeholder
    // The test verifies that:
    // 1. The substitutions parse correctly without errors
    // 2. All nodes are captured properly
    // 3. The parser handles these substitutions gracefully
}

#[test]
fn test_push_pop_ros_namespace() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("python/test_push_pop_namespace.launch.py");

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing push/pop namespace file should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 5, "Should have 5 nodes");

    // Test 1: Node with default namespace (null)
    let node_default = nodes
        .iter()
        .find(|n| n["name"] == "node_default_ns")
        .unwrap();
    assert_eq!(node_default["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_default["executable"].as_str().unwrap(), "talker");
    assert!(
        node_default["namespace"].is_null(),
        "Default namespace should be null"
    );

    // Test 2: Node in pushed namespace
    let node_pushed = nodes
        .iter()
        .find(|n| n["name"] == "node_in_pushed_ns")
        .unwrap();
    assert_eq!(node_pushed["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_pushed["executable"].as_str().unwrap(), "listener");
    assert_eq!(
        node_pushed["namespace"].as_str().unwrap(),
        "/pushed_ns",
        "Namespace should be /pushed_ns after push"
    );

    // Test 3: Node in nested namespace
    let node_nested = nodes
        .iter()
        .find(|n| n["name"] == "node_in_nested_ns")
        .unwrap();
    assert_eq!(node_nested["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_nested["executable"].as_str().unwrap(), "talker");
    assert_eq!(
        node_nested["namespace"].as_str().unwrap(),
        "/pushed_ns/nested",
        "Namespace should be /pushed_ns/nested after nested push"
    );

    // Test 4: Node after first pop (back to /pushed_ns)
    let node_first_pop = nodes
        .iter()
        .find(|n| n["name"] == "node_after_first_pop")
        .unwrap();
    assert_eq!(
        node_first_pop["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );
    assert_eq!(node_first_pop["executable"].as_str().unwrap(), "listener");
    assert_eq!(
        node_first_pop["namespace"].as_str().unwrap(),
        "/pushed_ns",
        "Namespace should be /pushed_ns after first pop"
    );

    // Test 5: Node after second pop (back to default)
    let node_second_pop = nodes
        .iter()
        .find(|n| n["name"] == "node_after_second_pop")
        .unwrap();
    assert_eq!(
        node_second_pop["package"].as_str().unwrap(),
        "demo_nodes_cpp"
    );
    assert_eq!(node_second_pop["executable"].as_str().unwrap(), "talker");
    assert!(
        node_second_pop["namespace"].is_null(),
        "Namespace should be null after second pop (back to default)"
    );
}

#[test]
fn test_advanced_actions() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("python/test_advanced_actions.launch.py");

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing advanced actions file should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    // We have 3 nodes: node_with_sim_time, delayed_node (from RosTimer), regular_node
    assert_eq!(nodes.len(), 3, "Should have 3 nodes");

    // Test 1: Node with sim time
    let node_sim = nodes
        .iter()
        .find(|n| n["name"] == "node_with_sim_time")
        .unwrap();
    assert_eq!(node_sim["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_sim["executable"].as_str().unwrap(), "talker");

    // Test 2: Delayed node (from RosTimer)
    let node_delayed = nodes.iter().find(|n| n["name"] == "delayed_node").unwrap();
    assert_eq!(node_delayed["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_delayed["executable"].as_str().unwrap(), "listener");

    // Test 3: Regular node
    let node_regular = nodes.iter().find(|n| n["name"] == "regular_node").unwrap();
    assert_eq!(node_regular["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_regular["executable"].as_str().unwrap(), "talker");

    // Note: ExecuteLocal, Shutdown, RosTimer, and SetUseSimTime are captured
    // but don't produce node records. They are informational actions.
    // The test verifies that:
    // 1. These actions parse correctly without errors
    // 2. Nodes within RosTimer actions are captured
    // 3. The parser handles these actions gracefully
}

#[test]
fn test_remap_logging() {
    let _guard = python_test_guard();
    let fixture = get_fixture_path("python/test_remap_logging.launch.py");

    let args = HashMap::new();
    let result = parse_launch_file(&fixture, args);

    assert!(
        result.is_ok(),
        "Parsing remaining actions file should succeed: {:?}",
        result.err()
    );
    let record = result.unwrap();

    let json = serde_json::to_value(&record).unwrap();

    // Verify we have nodes
    assert!(json["node"].is_array(), "Should have node array");
    let nodes = json["node"].as_array().unwrap();
    // We have 3 nodes: node_with_remap, listener_node, talker_with_remap
    assert_eq!(nodes.len(), 3, "Should have 3 nodes");

    // Test 1: Node with global remap in effect
    let node_remap = nodes
        .iter()
        .find(|n| n["name"] == "node_with_remap")
        .unwrap();
    assert_eq!(node_remap["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(node_remap["executable"].as_str().unwrap(), "talker");

    // Test 2: Listener node
    let listener = nodes.iter().find(|n| n["name"] == "listener_node").unwrap();
    assert_eq!(listener["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(listener["executable"].as_str().unwrap(), "listener");

    // Test 3: Talker with explicit remapping
    let talker_remap = nodes
        .iter()
        .find(|n| n["name"] == "talker_with_remap")
        .unwrap();
    assert_eq!(talker_remap["package"].as_str().unwrap(), "demo_nodes_cpp");
    assert_eq!(talker_remap["executable"].as_str().unwrap(), "talker");

    // Verify remappings field exists (may be null or array)
    // In static analysis, explicit node remappings in Python are captured
    // Note: the remappings list format in Python is a list of tuples like [('/old', '/new')]
    // which gets properly parsed and captured

    // Note: OpaqueCoroutine, SetRemap, and SetROSLogDir are captured
    // but don't produce node records. They are informational actions.
    // The test verifies that:
    // 1. These actions parse correctly without errors
    // 2. Nodes are captured properly
    // 3. The parser handles these actions gracefully
}

// ============================================================================
// Phase 15: Python API Type Safety Tests
// ============================================================================

#[test]
fn test_set_environment_variable_with_substitutions() {
    let _guard = python_test_guard();

    // Create temporary launch file with SetEnvironmentVariable using LaunchConfiguration
    let launch_content = r#"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('domain_id', default_value='42'),
        DeclareLaunchArgument('log_level', default_value='INFO'),
        
        # Test 1: Plain strings (backward compatibility)
        SetEnvironmentVariable('PLAIN_VAR', 'plain_value'),
        
        # Test 2: Single substitution
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        
        # Test 3: List with mixed types
        SetEnvironmentVariable('ROS_LOG_LEVEL', [LaunchConfiguration('log_level')]),
        
        # Test Node to verify parsing completes
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='test_node'
        ),
    ])
"#;

    let temp_file = write_temp_launch_file(launch_content);

    let args = HashMap::new();
    let result = parse_launch_file(temp_file.path(), args);

    assert!(
        result.is_ok(),
        "Parsing should succeed with LaunchConfiguration in SetEnvironmentVariable: {:?}",
        result.err()
    );

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Verify node was captured
    assert!(json["node"].is_array());
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node");
}

#[test]
fn test_append_environment_variable_with_substitutions() {
    let _guard = python_test_guard();

    let launch_content = r#"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('custom_path', default_value='/custom/path'),
        DeclareLaunchArgument('path_sep', default_value=':'),
        DeclareLaunchArgument('should_prepend', default_value='false'),
        
        # Test 1: Plain strings
        AppendEnvironmentVariable('PATH', '/usr/local/bin'),
        
        # Test 2: Substitution for value
        AppendEnvironmentVariable('PYTHONPATH', LaunchConfiguration('custom_path')),
        
        # Test 3: Substitution for separator
        AppendEnvironmentVariable('LD_LIBRARY_PATH', '/lib', separator=LaunchConfiguration('path_sep')),
        
        # Test 4: Substitution for prepend (bool from string)
        AppendEnvironmentVariable('PATH', '/prepend/path', prepend=LaunchConfiguration('should_prepend')),
        
        # Test Node to verify parsing completes
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='test_listener'
        ),
    ])
"#;

    let temp_file = write_temp_launch_file(launch_content);

    let args = HashMap::new();
    let result = parse_launch_file(temp_file.path(), args);

    assert!(
        result.is_ok(),
        "Parsing should succeed with substitutions in AppendEnvironmentVariable: {:?}",
        result.err()
    );

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Verify node was captured
    assert!(json["node"].is_array());
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node");
}

#[test]
fn test_execute_process_with_substitutions() {
    let _guard = python_test_guard();

    let launch_content = r#"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('package_name', default_value='demo_nodes_cpp'),
        DeclareLaunchArgument('node_name', default_value='talker'),
        
        # Test 1: Plain strings
        ExecuteProcess(cmd=['echo', 'hello']),
        
        # Test 2: Mixed strings and substitutions in cmd
        ExecuteProcess(
            cmd=['ros2', 'run', LaunchConfiguration('package_name'), LaunchConfiguration('node_name')]
        ),
        
        # Test Node to verify parsing completes
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='test_process_node'
        ),
    ])
"#;

    let temp_file = write_temp_launch_file(launch_content);

    let args = HashMap::new();
    let result = parse_launch_file(temp_file.path(), args);

    assert!(
        result.is_ok(),
        "Parsing should succeed with substitutions in ExecuteProcess: {:?}",
        result.err()
    );

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Verify node was captured
    assert!(json["node"].is_array());
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node");
}

#[test]
fn test_node_arguments_with_substitutions() {
    let _guard = python_test_guard();

    let launch_content = r#"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='/test_config/config.yaml'),
        DeclareLaunchArgument('log_level', default_value='info'),
        
        # Test 1: Plain string arguments
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='plain_args_node',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        
        # Test 2: Arguments with substitutions
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='sub_args_node',
            arguments=['--config', LaunchConfiguration('config_file'), '--verbose']
        ),
        
        # Test 3: Mixed arguments
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='mixed_args_node',
            arguments=[
                '--log-level',
                LaunchConfiguration('log_level'),
                '--extra-flag'
            ]
        ),
    ])
"#;

    let temp_file = write_temp_launch_file(launch_content);

    let args = HashMap::new();
    let result = parse_launch_file(temp_file.path(), args);

    assert!(
        result.is_ok(),
        "Parsing should succeed with substitutions in Node arguments: {:?}",
        result.err()
    );

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Verify all nodes were captured
    assert!(json["node"].is_array());
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 3, "Should have 3 nodes");

    // Find each node and verify they have arguments
    let plain_node = nodes
        .iter()
        .find(|n| n["name"] == "plain_args_node")
        .unwrap();
    assert!(
        plain_node["args"].is_array(),
        "plain_args_node should have args"
    );

    let sub_node = nodes.iter().find(|n| n["name"] == "sub_args_node").unwrap();
    assert!(
        sub_node["args"].is_array(),
        "sub_args_node should have args"
    );

    let mixed_node = nodes
        .iter()
        .find(|n| n["name"] == "mixed_args_node")
        .unwrap();
    assert!(
        mixed_node["args"].is_array(),
        "mixed_args_node should have args"
    );
}

#[test]
fn test_phase_15_combined() {
    let _guard = python_test_guard();

    // Comprehensive test combining all Phase 15 fixes
    let launch_content = r#"
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    AppendEnvironmentVariable,
    ExecuteProcess
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('domain_id', default_value='42'),
        DeclareLaunchArgument('workspace', default_value='/opt/ros/humble'),
        DeclareLaunchArgument('config_path', default_value='/etc/config.yaml'),
        
        # SetEnvironmentVariable with substitutions
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        
        # AppendEnvironmentVariable with substitutions
        AppendEnvironmentVariable(
            'CMAKE_PREFIX_PATH',
            LaunchConfiguration('workspace'),
            separator=':'
        ),
        
        # ExecuteProcess with substitutions
        ExecuteProcess(
            cmd=['echo', 'Using config:', LaunchConfiguration('config_path')]
        ),
        
        # Node with argument substitutions
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='comprehensive_test_node',
            arguments=['--config', LaunchConfiguration('config_path')]
        ),
    ])
"#;

    let temp_file = write_temp_launch_file(launch_content);

    let args = HashMap::new();
    let result = parse_launch_file(temp_file.path(), args);

    assert!(
        result.is_ok(),
        "Combined Phase 15 test should succeed: {:?}",
        result.err()
    );

    let record = result.unwrap();
    let json = serde_json::to_value(&record).unwrap();

    // Verify node was captured
    assert!(json["node"].is_array());
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1, "Should have 1 node");

    let node = &nodes[0];
    assert_eq!(node["name"].as_str().unwrap(), "comprehensive_test_node");
    assert!(node["args"].is_array(), "Node should have args");
}

// =============================================================================
// Python fixture-based test: SetRemap global (28.4)
// =============================================================================

#[test]
fn test_python_set_remap_global() {
    let _guard = python_test_guard();

    let fixture = get_fixture_path("test_python_set_remap_global.launch.py");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(
        result.is_ok(),
        "Python remappings should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert_eq!(nodes.len(), 1);
    assert_eq!(nodes[0]["name"].as_str().unwrap(), "remap_node");

    // Check remappings are captured
    let remaps = nodes[0]["remaps"].as_array().unwrap();
    let has_remap = remaps
        .iter()
        .any(|r| r[0].as_str() == Some("/input") && r[1].as_str() == Some("/remapped_input"));
    assert!(
        has_remap,
        "Should have /input -> /remapped_input remap, got: {:?}",
        remaps
    );
}

// =============================================================================
// Cross-format test: Python includes YAML (28.5)
// =============================================================================

#[test]
fn test_python_include_yaml() {
    let _guard = python_test_guard();

    let fixture = get_fixture_path("test_python_include_yaml.launch.py");
    assert!(fixture.exists(), "Fixture should exist: {:?}", fixture);

    let result = parse_launch_file(&fixture, HashMap::new());
    assert!(
        result.is_ok(),
        "Python including YAML should parse: {:?}",
        result.err()
    );

    let json = serde_json::to_value(result.unwrap()).unwrap();
    let nodes = json["node"].as_array().unwrap();
    assert!(
        !nodes.is_empty(),
        "Should have at least one node from Python file"
    );

    let has_after = nodes
        .iter()
        .any(|n| n["name"].as_str() == Some("after_yaml_include"));
    assert!(
        has_after,
        "Should have after_yaml_include node from Python file"
    );

    // Check that YAML preset variables were processed
    let variables = json["variables"].as_object();
    if let Some(vars) = variables {
        let has_yaml_var = vars.contains_key("yaml_var");
        assert!(
            has_yaml_var,
            "YAML preset variable should be visible after include"
        );
    }
}
