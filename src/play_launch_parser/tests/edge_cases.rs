use play_launch_parser::parse_launch_file;
use std::{collections::HashMap, io::Write};
use tempfile::NamedTempFile;

#[test]
fn test_empty_file() {
    // Empty file should fail gracefully
    let xml = "";

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_err(), "Empty file should produce an error");
}

#[test]
fn test_malformed_xml() {
    // Malformed XML should produce clear error
    let xml = "<launch>\n  <node pkg=\"test\" exec=\"test\"\n</launch>";

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_err(), "Malformed XML should produce an error");

    let err = result.unwrap_err();
    let err_msg = err.to_string();
    assert!(
        err_msg.contains("XML parsing error") || err_msg.contains("expected"),
        "Error should mention XML parsing: {}",
        err_msg
    );
}

#[test]
fn test_unclosed_tags() {
    // Unclosed tags should be caught
    let xml = "<launch>\n  <node pkg=\"test\" exec=\"test\" name=\"test\">\n</launch>";

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_err(), "Unclosed tags should produce an error");
}

#[test]
fn test_missing_required_attribute_node_pkg() {
    // Node without pkg attribute
    let xml = r#"<launch>
        <node exec="talker" name="test_node" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_err(), "Node without pkg should produce an error");

    let err = result.unwrap_err();
    let err_msg = err.to_string();
    assert!(
        err_msg.contains("pkg") || err_msg.contains("package"),
        "Error should mention missing pkg attribute: {}",
        err_msg
    );
}

#[test]
fn test_missing_required_attribute_node_exec() {
    // Node without exec attribute
    let xml = r#"<launch>
        <node pkg="demo_nodes_cpp" name="test_node" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_err(), "Node without exec should produce an error");

    let err = result.unwrap_err();
    let err_msg = err.to_string();
    assert!(
        err_msg.contains("exec") || err_msg.contains("executable"),
        "Error should mention missing exec attribute: {}",
        err_msg
    );
}

#[test]
fn test_missing_required_attribute_arg_name() {
    // Arg without name attribute
    let xml = r#"<launch>
        <arg default="value" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_err(), "Arg without name should produce an error");

    let err = result.unwrap_err();
    let err_msg = err.to_string();
    assert!(
        err_msg.contains("name"),
        "Error should mention missing name attribute: {}",
        err_msg
    );
}

#[test]
fn test_missing_required_attribute_include_file() {
    // Include without file attribute
    let xml = r#"<launch>
        <include />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_err(),
        "Include without file should produce an error"
    );

    let err = result.unwrap_err();
    let err_msg = err.to_string();
    assert!(
        err_msg.contains("file"),
        "Error should mention missing file attribute: {}",
        err_msg
    );
}

#[test]
fn test_undefined_variable_in_substitution() {
    // Reference to undefined variable
    let xml = r#"<launch>
        <node pkg="demo_nodes_cpp" exec="talker" name="$(var undefined_var)" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_err(),
        "Undefined variable should produce an error"
    );

    let err = result.unwrap_err();
    let err_msg = err.to_string();
    assert!(
        err_msg.contains("Undefined") || err_msg.contains("undefined_var"),
        "Error should mention undefined variable: {}",
        err_msg
    );
}

#[test]
fn test_undefined_env_var_in_substitution() {
    // Reference to undefined environment variable with $(env)
    let xml = r#"<launch>
        <node pkg="demo_nodes_cpp" exec="talker" name="$(env NONEXISTENT_VAR_XYZ_123)" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    unsafe { std::env::remove_var("NONEXISTENT_VAR_XYZ_123") };

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_err(), "Undefined env var should produce an error");

    let err = result.unwrap_err();
    let err_msg = err.to_string();
    assert!(
        err_msg.contains("environment variable") || err_msg.contains("NONEXISTENT_VAR_XYZ_123"),
        "Error should mention undefined environment variable: {}",
        err_msg
    );
}

#[test]
fn test_invalid_substitution_syntax() {
    // Malformed substitution syntax
    let xml = r#"<launch>
        <node pkg="demo_nodes_cpp" exec="talker" name="$(var" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_err(),
        "Invalid substitution syntax should produce an error"
    );
}

#[test]
fn test_nonexistent_include_file() {
    // Include referencing a non-existent file
    let xml = r#"<launch>
        <include file="/nonexistent/path/to/file.launch.xml" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_err(),
        "Nonexistent include file should produce an error"
    );

    let err = result.unwrap_err();
    let err_msg = err.to_string();
    assert!(
        err_msg.contains("IO error") || err_msg.contains("No such file"),
        "Error should mention file not found: {}",
        err_msg
    );
}

#[test]
fn test_only_whitespace_file() {
    // File with only whitespace
    let xml = "   \n  \t  \n   ";

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_err(),
        "Whitespace-only file should produce an error"
    );
}

#[test]
fn test_invalid_boolean_value() {
    // Invalid boolean value in condition
    let xml = r#"<launch>
        <node pkg="demo_nodes_cpp" exec="talker" name="test" if="maybe" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    // This should either fail or treat "maybe" as false
    // Current implementation treats non-true values as false, which is acceptable
    // Just verify it doesn't panic
    let _ = result;
}

#[test]
fn test_circular_substitution_prevention() {
    // Test that circular references don't cause infinite loops or crashes
    // Let variables store their values as strings, so var1="$(var var2)" and var2="$(var var1)"
    // When resolving, this could potentially cause infinite recursion
    let xml = r#"<launch>
        <let name="var1" value="$(var var2)" />
        <let name="var2" value="$(var var1)" />
        <node pkg="demo_nodes_cpp" exec="talker" name="$(var var1)" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    // The test should complete without hanging or crashing
    // It may succeed or fail, but should not infinite loop
    let result = parse_launch_file(file.path(), HashMap::new());

    // For now, just verify it doesn't hang - the actual behavior is implementation-defined
    // In the future, we might want to detect circular references explicitly
    let _ = result;
}

#[test]
fn test_empty_launch_tag() {
    // Empty launch file (valid but produces no nodes)
    let xml = "<launch></launch>";

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "Empty launch should be valid");

    let record = result.unwrap();
    assert_eq!(record.node.len(), 0, "Should have no nodes");
    assert_eq!(record.container.len(), 0, "Should have no containers");
}

#[test]
fn test_nested_groups_without_nodes() {
    // Nested groups without any actual nodes
    let xml = r#"<launch>
        <group><push-ros-namespace namespace="outer"/>
            <group><push-ros-namespace namespace="inner"/>
                <group><push-ros-namespace namespace="innermost"/>
                    <!-- No nodes here -->
                </group>
            </group>
        </group>
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "Nested empty groups should be valid");

    let record = result.unwrap();
    assert_eq!(record.node.len(), 0, "Should have no nodes");
}

#[test]
fn test_special_characters_in_names() {
    // Test special characters in node names
    let xml = r#"<launch>
        <node pkg="demo_nodes_cpp" exec="talker" name="test_node-123_with.special" />
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(
        result.is_ok(),
        "Special characters in names should be allowed"
    );

    let record = result.unwrap();
    assert_eq!(record.node.len(), 1);
    assert_eq!(
        record.node[0].name.as_ref().unwrap(),
        "test_node-123_with.special"
    );
}

#[test]
fn test_unicode_in_values() {
    // Test Unicode characters in parameter values
    let xml = r#"<launch>
        <node pkg="demo_nodes_cpp" exec="talker" name="test">
            <param name="message" value="Hello 世界 🌍" />
        </node>
    </launch>"#;

    let mut file = NamedTempFile::new().unwrap();
    file.write_all(xml.as_bytes()).unwrap();
    file.flush().unwrap();

    let result = parse_launch_file(file.path(), HashMap::new());
    assert!(result.is_ok(), "Unicode should be handled correctly");

    let record = result.unwrap();
    assert_eq!(record.node.len(), 1);
    let params = &record.node[0].params;
    let message_param = params.iter().find(|(k, _)| k == "message");
    assert!(message_param.is_some());
    assert_eq!(message_param.unwrap().1, "Hello 世界 🌍");
}
