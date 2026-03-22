//! Parameter file loading

use crate::{
    error::ParseError,
    substitution::{LaunchContext, resolve_substitutions},
};
use serde_yaml_ng::Value;
use std::{fs, path::Path};

/// Load a parameter file and resolve all substitutions in its contents.
/// Returns the resolved YAML content as a string, preserving comments and formatting.
pub fn load_and_resolve_param_file(
    path: &Path,
    context: &LaunchContext,
) -> Result<String, ParseError> {
    let content = fs::read_to_string(path)?;

    // If the file has no substitutions, return raw content (preserves comments/formatting)
    if !content.contains("$(") {
        return Ok(content);
    }

    // Resolve substitutions line by line to preserve comments and formatting
    let mut resolved_lines = Vec::new();
    for line in content.lines() {
        if line.contains("$(") {
            // Resolve substitutions in this line while preserving surrounding text
            let resolved = resolve_line_substitutions(line, context)?;
            resolved_lines.push(resolved);
        } else {
            resolved_lines.push(line.to_string());
        }
    }
    // Preserve trailing newline if original had one
    let mut result = resolved_lines.join("\n");
    if content.ends_with('\n') {
        result.push('\n');
    }
    Ok(result)
}

/// Resolve substitutions in a single line of text.
/// Handles patterns like `$(var name)` while preserving surrounding text.
fn resolve_line_substitutions(line: &str, context: &LaunchContext) -> Result<String, ParseError> {
    let mut result = String::new();
    let mut remaining = line;

    while let Some(start) = remaining.find("$(") {
        // Add text before the substitution
        result.push_str(&remaining[..start]);

        // Find the matching closing paren
        let after_start = &remaining[start..];
        if let Some(end) = find_matching_paren(after_start) {
            let sub_str = &after_start[..end + 1];
            // Parse and resolve the substitution
            let subs = crate::substitution::parse_substitutions(sub_str)?;
            let resolved = resolve_substitutions(&subs, context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
            result.push_str(&resolved);
            remaining = &after_start[end + 1..];
        } else {
            // No matching paren, keep as-is
            result.push_str(after_start);
            remaining = "";
        }
    }
    result.push_str(remaining);
    Ok(result)
}

/// Find the matching closing paren for a $(...) substitution.
/// Returns the index of the closing ')'.
fn find_matching_paren(s: &str) -> Option<usize> {
    let mut depth = 0;
    for (i, c) in s.char_indices() {
        match c {
            '(' => depth += 1,
            ')' => {
                depth -= 1;
                if depth == 0 {
                    return Some(i);
                }
            }
            _ => {}
        }
    }
    None
}

/// Extract parameters from a YAML file and resolve substitutions.
/// Returns a vector of (param_name, param_value) tuples.
/// Used for temp parameter files that should be expanded into inline params.
pub fn extract_params_from_yaml(
    path: &Path,
    context: &LaunchContext,
) -> Result<Vec<(String, String)>, ParseError> {
    let content = fs::read_to_string(path)?;
    let mut yaml: Value = serde_yaml_ng::from_str(&content)
        .map_err(|e| ParseError::InvalidSubstitution(format!("YAML parse error: {}", e)))?;

    // Recursively resolve all substitutions in the YAML structure
    resolve_yaml_substitutions(&mut yaml, context)?;

    // Extract parameters from resolved YAML
    let mut params = Vec::new();
    if let Value::Mapping(root_map) = yaml {
        // Iterate through each node in the YAML
        for (_node_name, node_value) in root_map.iter() {
            if let Value::Mapping(node_map) = node_value {
                // Look for ros__parameters key
                if let Some(Value::Mapping(params_map)) = node_map.get("ros__parameters") {
                    flatten_params("", params_map, &mut params);
                }
            }
        }
    }

    Ok(params)
}

/// Recursively resolve substitutions in a YAML value
fn resolve_yaml_substitutions(
    value: &mut Value,
    context: &LaunchContext,
) -> Result<(), ParseError> {
    match value {
        Value::String(s) => {
            // Check if the string contains substitutions (starts with $)
            if s.contains('$') {
                // Parse and resolve the substitution
                let subs = crate::substitution::parse_substitutions(s)?;
                let resolved = resolve_substitutions(&subs, context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

                // Try to convert the resolved string to the appropriate YAML type
                *value = string_to_yaml_value(&resolved);
            }
        }
        Value::Mapping(map) => {
            // Recursively process all values in the mapping
            for (_, v) in map.iter_mut() {
                resolve_yaml_substitutions(v, context)?;
            }
        }
        Value::Sequence(seq) => {
            // Recursively process all items in the sequence
            for item in seq.iter_mut() {
                resolve_yaml_substitutions(item, context)?;
            }
        }
        _ => {
            // Other types (numbers, bools, null) don't need resolution
        }
    }
    Ok(())
}

/// Convert a string to the appropriate YAML value type
fn string_to_yaml_value(s: &str) -> Value {
    // Try to parse as boolean
    match s {
        "true" => return Value::Bool(true),
        "false" => return Value::Bool(false),
        "null" | "~" => return Value::Null,
        _ => {}
    }

    // Try to parse as integer
    if let Ok(n) = s.parse::<i64>() {
        return Value::Number(n.into());
    }

    // Try to parse as float
    if let Ok(n) = s.parse::<f64>() {
        // serde_yaml_ng doesn't have a direct from_f64, so serialize and deserialize
        if let Ok(value) = serde_yaml_ng::to_value(n) {
            return value;
        }
    }

    // Default to string
    Value::String(s.to_string())
}

/// Load parameters from a YAML file
pub fn load_param_file(path: &Path) -> Result<Vec<(String, String)>, ParseError> {
    let content = fs::read_to_string(path)?;
    let yaml: Value = serde_yaml_ng::from_str(&content)
        .map_err(|e| ParseError::InvalidSubstitution(format!("YAML parse error: {}", e)))?;

    let mut params = Vec::new();

    // ROS 2 parameter files have the structure:
    // node_name:
    //   ros__parameters:
    //     param1: value1
    //     param2: value2

    if let Value::Mapping(root_map) = yaml {
        // Iterate through each node in the YAML
        for (_node_name, node_value) in root_map.iter() {
            if let Value::Mapping(node_map) = node_value {
                // Look for ros__parameters key
                if let Some(Value::Mapping(params_map)) = node_map.get("ros__parameters") {
                    flatten_params("", params_map, &mut params);
                }
            }
        }
    }

    Ok(params)
}

/// Recursively flatten nested parameter maps
fn flatten_params(prefix: &str, map: &serde_yaml_ng::Mapping, output: &mut Vec<(String, String)>) {
    for (key, value) in map.iter() {
        if let Value::String(key_str) = key {
            let full_key = if prefix.is_empty() {
                key_str.clone()
            } else {
                format!("{}.{}", prefix, key_str)
            };

            match value {
                Value::Mapping(nested_map) => {
                    // Nested parameters
                    flatten_params(&full_key, nested_map, output);
                }
                Value::String(s) => {
                    output.push((full_key, s.clone()));
                }
                Value::Number(n) => {
                    output.push((full_key, n.to_string()));
                }
                Value::Bool(b) => {
                    // Use Python convention: True/False
                    output.push((
                        full_key,
                        if *b {
                            "True".to_string()
                        } else {
                            "False".to_string()
                        },
                    ));
                }
                Value::Sequence(seq) => {
                    // Arrays get serialized as JSON
                    let json_str =
                        serde_json::to_string(seq).unwrap_or_else(|_| format!("{:?}", seq));
                    output.push((full_key, json_str));
                }
                Value::Null => {
                    output.push((full_key, "null".to_string()));
                }
                _ => {
                    // For other types, convert to string
                    output.push((full_key, format!("{:?}", value)));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    #[test]
    fn test_load_simple_params() {
        let yaml = r#"
my_node:
  ros__parameters:
    param1: "value1"
    param2: 42
    param3: true
"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(yaml.as_bytes()).unwrap();
        file.flush().unwrap();

        let params = load_param_file(file.path()).unwrap();
        assert_eq!(params.len(), 3);

        // Find specific parameters
        let param1 = params.iter().find(|(k, _)| k == "param1");
        assert_eq!(param1, Some(&("param1".to_string(), "value1".to_string())));

        let param2 = params.iter().find(|(k, _)| k == "param2");
        assert_eq!(param2, Some(&("param2".to_string(), "42".to_string())));

        let param3 = params.iter().find(|(k, _)| k == "param3");
        assert_eq!(param3, Some(&("param3".to_string(), "True".to_string())));
    }

    #[test]
    fn test_load_nested_params() {
        let yaml = r#"
my_node:
  ros__parameters:
    simple: "value"
    nested:
      param1: "nested_value"
      param2: 100
"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(yaml.as_bytes()).unwrap();
        file.flush().unwrap();

        let params = load_param_file(file.path()).unwrap();
        assert_eq!(params.len(), 3);

        let nested_param1 = params.iter().find(|(k, _)| k == "nested.param1");
        assert_eq!(
            nested_param1,
            Some(&("nested.param1".to_string(), "nested_value".to_string()))
        );

        let nested_param2 = params.iter().find(|(k, _)| k == "nested.param2");
        assert_eq!(
            nested_param2,
            Some(&("nested.param2".to_string(), "100".to_string()))
        );
    }

    #[test]
    fn test_load_array_params() {
        let yaml = r#"
my_node:
  ros__parameters:
    my_array: [1, 2, 3, 4]
"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(yaml.as_bytes()).unwrap();
        file.flush().unwrap();

        let params = load_param_file(file.path()).unwrap();
        assert_eq!(params.len(), 1);

        let array_param = params.iter().find(|(k, _)| k == "my_array");
        assert!(array_param.is_some());
        let (_, value) = array_param.unwrap();
        // Should be JSON array
        assert!(value.contains("[1,2,3,4]") || value.contains("[1, 2, 3, 4]"));
    }

    #[test]
    fn test_multiple_nodes() {
        let yaml = r#"
node1:
  ros__parameters:
    param1: "value1"

node2:
  ros__parameters:
    param2: "value2"
"#;

        let mut file = NamedTempFile::new().unwrap();
        file.write_all(yaml.as_bytes()).unwrap();
        file.flush().unwrap();

        let params = load_param_file(file.path()).unwrap();
        // Should load parameters from all nodes
        assert_eq!(params.len(), 2);
    }
}
