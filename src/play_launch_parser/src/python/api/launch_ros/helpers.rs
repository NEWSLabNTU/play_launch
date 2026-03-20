//! YAML parameter file loading helpers

use pyo3::PyResult;

/// Load and expand YAML parameter file into key-value pairs
///
/// This matches the Python parser's behavior of loading parameter files
/// and extracting all parameters as individual key-value pairs.
pub(crate) fn load_yaml_params(path: &str) -> PyResult<Vec<(String, String)>> {
    use serde_yml::Value;

    // Read YAML file
    let contents = std::fs::read_to_string(path).map_err(|e| {
        pyo3::exceptions::PyIOError::new_err(format!(
            "Failed to read parameter file {}: {}",
            path, e
        ))
    })?;

    // Parse YAML
    let yaml: Value = serde_yml::from_str(&contents).map_err(|e| {
        pyo3::exceptions::PyValueError::new_err(format!(
            "Failed to parse YAML file {}: {}",
            path, e
        ))
    })?;

    // Strip ROS 2 parameter file wrappers (/**:  and ros__parameters:)
    // Iterate ALL matching top-level keys (YAML files like default_adapi.param.yaml
    // have multiple node-specific sections: /adapi/node/autoware_state, /adapi/node/motion)
    let mut params = Vec::new();

    if let Value::Mapping(top_map) = &yaml {
        let mut found_node_keys = false;
        for (k, v) in top_map {
            if let Value::String(key) = k
                && (key == "/**" || key.starts_with('/'))
            {
                found_node_keys = true;
                // Found a node matcher, look for ros__parameters
                if let Value::Mapping(node_map) = v {
                    let params_value = node_map
                        .get(Value::String("ros__parameters".to_string()))
                        .unwrap_or(v);
                    flatten_yaml(params_value, "", &mut params);
                } else {
                    flatten_yaml(v, "", &mut params);
                }
            }
        }

        if !found_node_keys {
            // No node-specific keys found, flatten the whole YAML
            flatten_yaml(&yaml, "", &mut params);
        }
    } else {
        flatten_yaml(&yaml, "", &mut params);
    }

    Ok(params)
}

/// Flatten YAML structure into key-value pairs with dot notation for nested objects
///
/// Example:
/// ```yaml
/// namespace:
///   param1: value1
///   nested:
///     param2: value2
/// ```
/// Becomes: `[("namespace.param1", "value1"), ("namespace.nested.param2", "value2")]`
pub(crate) fn flatten_yaml(
    value: &serde_yml::Value,
    prefix: &str,
    params: &mut Vec<(String, String)>,
) {
    use serde_yml::Value;

    match value {
        Value::Mapping(map) => {
            for (k, v) in map {
                if let Some(key) = k.as_str() {
                    let full_key = if prefix.is_empty() {
                        key.to_string()
                    } else {
                        format!("{}.{}", prefix, key)
                    };

                    if v.is_mapping() {
                        // Recurse for nested objects
                        flatten_yaml(v, &full_key, params);
                    } else {
                        // Convert value to string
                        let value_str = yaml_value_to_string(v);
                        params.push((full_key, value_str));
                    }
                }
            }
        }
        _ => {
            // Top-level is not a mapping, treat as single value
            if !prefix.is_empty() {
                params.push((prefix.to_string(), yaml_value_to_string(value)));
            }
        }
    }
}

/// Convert YAML value to string representation
///
/// Handles booleans, numbers, strings, arrays, and null values
pub(crate) fn yaml_value_to_string(value: &serde_yml::Value) -> String {
    use serde_yml::Value;

    match value {
        Value::Bool(b) => b.to_string(),
        Value::Number(n) => {
            // Preserve original YAML type: integers stay integers, floats stay floats
            if n.is_i64() {
                format!("{}", n.as_i64().unwrap())
            } else if n.is_u64() {
                format!("{}", n.as_u64().unwrap())
            } else if let Some(f) = n.as_f64() {
                // Float value - ensure decimal point for ROS type checking
                let s = f.to_string();
                if s.contains('.') {
                    s
                } else {
                    format!("{}.0", s)
                }
            } else {
                n.to_string()
            }
        }
        Value::String(s) => s.clone(),
        Value::Sequence(seq) => {
            // Convert array to string representation
            let elements: Vec<String> = seq.iter().map(yaml_value_to_string).collect();
            format!("[{}]", elements.join(", "))
        }
        Value::Null => "null".to_string(),
        _ => format!("{:?}", value),
    }
}

/// Check if a string looks like a YAML file path
pub(crate) fn is_yaml_file(path: &str) -> bool {
    path.ends_with(".yaml") || path.ends_with(".yml")
}
