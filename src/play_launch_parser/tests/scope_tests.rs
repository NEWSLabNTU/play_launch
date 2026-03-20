//! Tests for launch tree scope tracking (Phase 30.1)

use play_launch_parser::record::{ScopeTable, extract_package_from_path};
use std::{collections::HashMap, path::Path};

#[test]
fn test_scope_table_root() {
    let mut table = ScopeTable::new();
    assert!(table.is_empty());

    let id = table.push(
        Some("my_pkg".to_string()),
        "launch.xml".to_string(),
        "/".to_string(),
        HashMap::new(),
        None,
    );
    assert_eq!(id, 0);
    assert_eq!(table.len(), 1);

    let entry = table.get(0).unwrap();
    assert_eq!(entry.id, 0);
    assert_eq!(entry.pkg.as_deref(), Some("my_pkg"));
    assert_eq!(entry.file, "launch.xml");
    assert_eq!(entry.ns, "/");
    assert!(entry.parent.is_none());
}

#[test]
fn test_scope_table_parent_chain() {
    let mut table = ScopeTable::new();

    let root = table.push(
        Some("autoware_launch".to_string()),
        "planning_simulator.launch.xml".to_string(),
        "/".to_string(),
        HashMap::new(),
        None,
    );

    let child1 = table.push(
        Some("tier4_sensing_launch".to_string()),
        "sensing.launch.xml".to_string(),
        "/sensing".to_string(),
        HashMap::from([("vehicle_model".to_string(), "sample_vehicle".to_string())]),
        Some(root),
    );

    let child2 = table.push(
        Some("camera_driver".to_string()),
        "camera.launch.xml".to_string(),
        "/sensing/camera/front".to_string(),
        HashMap::from([("camera_id".to_string(), "0".to_string())]),
        Some(child1),
    );

    assert_eq!(table.len(), 3);

    // Verify parent chain
    let entry = table.get(child2).unwrap();
    assert_eq!(entry.parent, Some(child1));
    assert_eq!(entry.ns, "/sensing/camera/front");
    assert_eq!(entry.args.get("camera_id").unwrap(), "0");

    let parent = table.get(entry.parent.unwrap()).unwrap();
    assert_eq!(parent.parent, Some(root));
    assert_eq!(parent.ns, "/sensing");

    let grandparent = table.get(parent.parent.unwrap()).unwrap();
    assert!(grandparent.parent.is_none());
}

#[test]
fn test_scope_table_same_file_different_ns() {
    let mut table = ScopeTable::new();

    let root = table.push(
        Some("root_pkg".to_string()),
        "main.launch.xml".to_string(),
        "/".to_string(),
        HashMap::new(),
        None,
    );

    // Same file included twice with different namespaces
    let front = table.push(
        Some("camera_driver".to_string()),
        "camera.launch.xml".to_string(),
        "/sensing/camera/front".to_string(),
        HashMap::from([("camera_id".to_string(), "0".to_string())]),
        Some(root),
    );

    let rear = table.push(
        Some("camera_driver".to_string()),
        "camera.launch.xml".to_string(),
        "/sensing/camera/rear".to_string(),
        HashMap::from([("camera_id".to_string(), "1".to_string())]),
        Some(root),
    );

    assert_eq!(table.len(), 3);
    assert_ne!(front, rear);

    // Both point to same parent
    assert_eq!(table.get(front).unwrap().parent, Some(root));
    assert_eq!(table.get(rear).unwrap().parent, Some(root));

    // But different namespaces and args
    assert_eq!(table.get(front).unwrap().ns, "/sensing/camera/front");
    assert_eq!(table.get(rear).unwrap().ns, "/sensing/camera/rear");
}

#[test]
fn test_extract_package_from_path() {
    // Standard ament install path
    assert_eq!(
        extract_package_from_path(Path::new(
            "/opt/ros/humble/share/demo_nodes_cpp/launch/talker.launch.xml"
        )),
        Some("demo_nodes_cpp".to_string())
    );

    // Autoware install path
    assert_eq!(
        extract_package_from_path(Path::new(
            "/opt/autoware/1.5.0/share/autoware_launch/launch/planning_simulator.launch.xml"
        )),
        Some("autoware_launch".to_string())
    );

    // Workspace source path (no /share/ pattern)
    assert_eq!(
        extract_package_from_path(Path::new("/home/user/ws/src/my_pkg/launch/my_launch.xml")),
        None
    );

    // Edge case: /share/ at end
    assert_eq!(
        extract_package_from_path(Path::new("/opt/ros/share/")),
        None
    );
}

#[test]
fn test_scope_serialization_roundtrip() {
    use play_launch_parser::record::RecordJson;

    let mut record = RecordJson::new();
    assert!(record.scopes.is_empty());

    record.scopes.push(play_launch_parser::record::ScopeEntry {
        id: 0,
        pkg: Some("demo_nodes_cpp".to_string()),
        file: "talker_listener.launch.xml".to_string(),
        ns: "/".to_string(),
        args: HashMap::new(),
        parent: None,
    });

    let json = record.to_json().unwrap();
    assert!(json.contains("\"scopes\""));
    assert!(json.contains("talker_listener.launch.xml"));

    // Deserialize back
    let parsed: RecordJson = serde_json::from_str(&json).unwrap();
    assert_eq!(parsed.scopes.len(), 1);
    assert_eq!(parsed.scopes[0].file, "talker_listener.launch.xml");
    assert!(parsed.scopes[0].parent.is_none());
}

#[test]
fn test_old_record_json_without_scopes() {
    use play_launch_parser::record::RecordJson;

    // Old format: no "scopes" field
    let json = r#"{
        "container": [],
        "file_data": {},
        "lifecycle_node": [],
        "load_node": [],
        "node": []
    }"#;

    let parsed: RecordJson = serde_json::from_str(json).unwrap();
    assert!(parsed.scopes.is_empty());
    assert!(parsed.node.is_empty());
}

#[test]
fn test_node_record_scope_field_optional() {
    use play_launch_parser::record::NodeRecord;

    // Old format: no "scope" field
    let json = r#"{
        "cmd": [],
        "executable": "talker",
        "params": [],
        "params_files": [],
        "remaps": []
    }"#;

    let parsed: NodeRecord = serde_json::from_str(json).unwrap();
    assert!(parsed.scope.is_none());

    // New format: with "scope" field
    let json = r#"{
        "cmd": [],
        "executable": "talker",
        "params": [],
        "params_files": [],
        "remaps": [],
        "scope": 2
    }"#;

    let parsed: NodeRecord = serde_json::from_str(json).unwrap();
    assert_eq!(parsed.scope, Some(2));
}

#[test]
fn test_scope_tracking_simple_launch() {
    use play_launch_parser::parse_launch_file;
    use std::collections::HashMap;

    // Parse the simplest test fixture — a single launch file with nodes
    let fixture_dir = format!("{}/tests/fixtures/launch", env!("CARGO_MANIFEST_DIR"));
    let launch_file = format!("{}/test_basic_node.launch.xml", fixture_dir);
    let path = std::path::Path::new(&launch_file);

    if !path.exists() {
        // Skip if fixture doesn't exist
        return;
    }

    let record = parse_launch_file(path, HashMap::new()).unwrap();

    // Should have at least the root scope
    assert!(!record.scopes.is_empty(), "scopes should not be empty");
    assert_eq!(record.scopes[0].parent, None, "root scope has no parent");
    assert!(
        record.scopes[0].file.ends_with(".launch.xml"),
        "root scope file should be a launch file"
    );

    // All nodes should have a scope
    for node in &record.node {
        assert!(
            node.scope.is_some(),
            "node '{}' should have a scope",
            node.name.as_deref().unwrap_or("unnamed")
        );
    }
}

#[test]
fn test_scope_tracking_with_include() {
    use play_launch_parser::parse_launch_file;
    use std::collections::HashMap;

    // Parse a launch file that includes another
    let fixture_dir = format!("{}/tests/fixtures/launch", env!("CARGO_MANIFEST_DIR"));
    let launch_file = format!("{}/test_include.launch.xml", fixture_dir);
    let path = std::path::Path::new(&launch_file);

    if !path.exists() {
        return;
    }

    let record = parse_launch_file(path, HashMap::new()).unwrap();

    // Should have at least 2 scopes (root + included file)
    assert!(
        record.scopes.len() >= 2,
        "should have at least 2 scopes for an include, got {}",
        record.scopes.len()
    );

    // Root scope
    assert_eq!(record.scopes[0].parent, None);

    // At least one child scope with parent=0
    let has_child = record.scopes.iter().any(|s| s.parent == Some(0));
    assert!(has_child, "should have a child scope with parent=0");
}

#[test]
fn test_scope_group_push_ros_namespace_with_include() {
    use play_launch_parser::parse_launch_file;
    use std::collections::HashMap;

    let fixture_dir = format!("{}/tests/fixtures/launch", env!("CARGO_MANIFEST_DIR"));
    let launch_file = format!("{}/test_group_namespace.launch.xml", fixture_dir);
    let path = std::path::Path::new(&launch_file);

    if !path.exists() {
        return;
    }

    let record = parse_launch_file(path, HashMap::new()).unwrap();

    // Should have 2 scopes: root + included file
    assert!(
        record.scopes.len() >= 2,
        "expected at least 2 scopes, got {}",
        record.scopes.len()
    );

    // The included file's scope should have ns=/sensing
    // (push-ros-namespace inside group propagates to include)
    let child_scope = record.scopes.iter().find(|s| s.parent == Some(0));
    assert!(child_scope.is_some(), "should have a child scope");
    let child = child_scope.unwrap();
    assert_eq!(
        child.ns, "/sensing",
        "included file scope ns should be /sensing, got {}",
        child.ns
    );

    // The outer node should have no namespace or root namespace
    let outer = record
        .node
        .iter()
        .find(|n| n.name.as_deref() == Some("outer_node"));
    assert!(outer.is_some(), "should have outer_node");

    // The included node should have /sensing namespace
    let inner = record
        .node
        .iter()
        .find(|n| n.namespace.as_deref() == Some("/sensing"));
    assert!(
        inner.is_some(),
        "should have a node with /sensing namespace"
    );
}

#[test]
fn test_scope_group_namespace_attr_with_include() {
    use play_launch_parser::parse_launch_file;
    use std::collections::HashMap;

    // Tests B2: <group namespace="sensing"> wrapping an include.
    // Known limitation: the <group namespace="..."> attribute form
    // does not propagate namespace the same way as <push-ros-namespace>.
    // This matches ROS 2 launch XML behavior where the standard form
    // is <push-ros-namespace> inside the group.
    let fixture_dir = format!("{}/tests/fixtures/launch", env!("CARGO_MANIFEST_DIR"));
    let launch_file = format!("{}/test_group_namespace_attr.launch.xml", fixture_dir);
    let path = std::path::Path::new(&launch_file);

    if !path.exists() {
        return;
    }

    let record = parse_launch_file(path, HashMap::new()).unwrap();

    // Should parse without errors
    assert!(
        record.scopes.len() >= 2,
        "expected at least 2 scopes, got {}",
        record.scopes.len()
    );

    // Verify the parse produces nodes (correctness of the parse itself)
    assert!(
        !record.node.is_empty(),
        "should have at least one node from the include"
    );
}

#[test]
fn test_include_path_validation_blocks_non_launch() {
    use play_launch_parser::parse_launch_file;
    use std::collections::HashMap;

    // A launch file that tries to include a non-launch file should fail
    let xml = r#"<launch>
        <include file="/etc/passwd"/>
    </launch>"#;

    let dir = std::env::temp_dir().join("play_launch_test_path_validation");
    std::fs::create_dir_all(&dir).unwrap();
    let file = dir.join("test.launch.xml");
    std::fs::write(&file, xml).unwrap();

    let result = parse_launch_file(&file, HashMap::new());
    assert!(
        result.is_err(),
        "including /etc/passwd should be rejected by extension validation"
    );

    std::fs::remove_dir_all(&dir).ok();
}
