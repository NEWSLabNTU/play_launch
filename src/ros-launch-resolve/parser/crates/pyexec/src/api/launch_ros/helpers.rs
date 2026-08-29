//! YAML parameter file loading helpers

use pyo3::PyResult;

/// Load and expand YAML parameter file into key-value pairs
///
/// This matches the Python parser's behavior of loading parameter files
/// and extracting all parameters as individual key-value pairs.
pub(crate) fn load_yaml_params(path: &str) -> PyResult<Vec<(String, String)>> {
    use serde_yaml_ng::Value;

    // Read YAML file
    let contents = std::fs::read_to_string(path).map_err(|e| {
        pyo3::exceptions::PyIOError::new_err(format!(
            "Failed to read parameter file {}: {}",
            path, e
        ))
    })?;

    // Parse YAML
    let yaml: Value = serde_yaml_ng::from_str(&contents).map_err(|e| {
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

/// Load a YAML parameter file, keeping only the sections that apply to the
/// node identified by `node_fqn` (the node's fully-qualified path WITHOUT a
/// leading slash, e.g. `"adapi/node/autoware_state"`).
///
/// Mirrors launch_ros `to_parameters_list` (`utilities/to_parameters_list.py`):
/// a params file keyed by node name (`/**`, `/*`, or an exact FQN) applies a
/// section to a node only when the section key matches the node's FQN. Wildcard
/// sections apply in file order; the exact-FQN section is applied LAST so it
/// wins on key collisions. This is the per-node filtering the real ROS node
/// does at runtime — without it, EVERY composable in a container that shares a
/// multi-section params file wrongly receives every other node's parameters
/// (Autoware `default_adapi.param.yaml`: `update_rate` etc. leaking across all
/// `/adapi/node/*` composables).
///
/// A file with NO node-key sections (a plain flat param mapping) is flattened
/// whole, exactly like [`load_yaml_params`].
pub(crate) fn load_yaml_params_for_node(
    path: &str,
    node_fqn: &str,
) -> PyResult<Vec<(String, String)>> {
    use serde_yaml_ng::Value;

    let contents = std::fs::read_to_string(path).map_err(|e| {
        pyo3::exceptions::PyIOError::new_err(format!(
            "Failed to read parameter file {}: {}",
            path, e
        ))
    })?;
    let yaml: Value = serde_yaml_ng::from_str(&contents).map_err(|e| {
        pyo3::exceptions::PyValueError::new_err(format!(
            "Failed to parse YAML file {}: {}",
            path, e
        ))
    })?;

    match collect_node_sections(&yaml) {
        Some(sections) => {
            let fqn_key = node_fqn.trim_start_matches('/');
            let anchored = format!("/{}", fqn_key);
            let mut merged: Vec<(String, String)> = Vec::new();

            // Pass 1: every section (wildcard or exact) whose key matches the
            // node FQN, in file order — mirrors the first `for key in ...` loop.
            for (key, value) in &sections {
                if is_node_name_matched(&format!("/{}", key), &anchored) {
                    apply_section(value, &mut merged);
                }
            }
            // Pass 2: the exact-FQN section again, last — mirrors the trailing
            // `if node_name in normalized: update`, so it wins on collisions.
            for (key, value) in &sections {
                if key.as_str() == fqn_key {
                    apply_section(value, &mut merged);
                }
            }
            Ok(merged)
        }
        // No node-keyed sections — behave like the unfiltered loader.
        None => {
            let mut params = Vec::new();
            flatten_yaml(&yaml, "", &mut params);
            Ok(params)
        }
    }
}

/// Collect the top-level node-name sections (`/**`, `/*`, `/foo/bar`) of a ROS
/// parameter file as `(key_without_leading_slash, params_value)` in file order,
/// unwrapping the inner `ros__parameters` mapping. Returns `None` when the file
/// has no `/`-prefixed section keys (a plain flat param mapping).
fn collect_node_sections(
    yaml: &serde_yaml_ng::Value,
) -> Option<Vec<(String, &serde_yaml_ng::Value)>> {
    use serde_yaml_ng::Value;

    let Value::Mapping(top_map) = yaml else {
        return None;
    };
    let mut sections = Vec::new();
    for (k, v) in top_map {
        if let Value::String(key) = k
            && (key == "/**" || key.starts_with('/'))
        {
            let params_value = if let Value::Mapping(node_map) = v {
                node_map
                    .get(Value::String("ros__parameters".to_string()))
                    .unwrap_or(v)
            } else {
                v
            };
            sections.push((key.trim_start_matches('/').to_string(), params_value));
        }
    }
    if sections.is_empty() {
        None
    } else {
        Some(sections)
    }
}

/// Flatten a params section into `merged`, upserting so a later section
/// overrides an earlier key while preserving first-seen order (matches the
/// dict `.update()` last-wins semantics of `to_parameters_list`).
fn apply_section(params_value: &serde_yaml_ng::Value, merged: &mut Vec<(String, String)>) {
    let mut flat = Vec::new();
    flatten_yaml(params_value, "", &mut flat);
    for (key, value) in flat {
        if let Some(slot) = merged.iter_mut().find(|(k, _)| *k == key) {
            slot.1 = value;
        } else {
            merged.push((key, value));
        }
    }
}

/// ROS node-name matching used by parameter files, mirroring launch_ros
/// `is_node_name_matched`: `/*` matches exactly one path token, `/**` matches
/// zero or more. Both `pattern` and `node_fqn` carry a leading slash.
pub(crate) fn is_node_name_matched(pattern: &str, node_fqn: &str) -> bool {
    // Same transform as launch_ros: "/*" -> "(/\w+)"; so "/**" -> "(/\w+)*".
    let regex_src = format!("^{}$", pattern.replace("/*", "(/\\w+)"));
    match regex::Regex::new(&regex_src) {
        Ok(re) => re.is_match(node_fqn),
        // A pattern with no wildcards is a plain literal; fall back to equality.
        Err(_) => pattern == node_fqn,
    }
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
    value: &serde_yaml_ng::Value,
    prefix: &str,
    params: &mut Vec<(String, String)>,
) {
    use serde_yaml_ng::Value;

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
pub(crate) fn yaml_value_to_string(value: &serde_yaml_ng::Value) -> String {
    use serde_yaml_ng::Value;

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

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn node_name_matching_rules() {
        // `/**` matches any FQN (zero or more tokens).
        assert!(is_node_name_matched("/**", "/adapi/node/autoware_state"));
        assert!(is_node_name_matched("/**", "/foo"));
        // Exact match.
        assert!(is_node_name_matched(
            "/adapi/node/autoware_state",
            "/adapi/node/autoware_state"
        ));
        // Non-matching exact section is excluded.
        assert!(!is_node_name_matched(
            "/adapi/node/motion",
            "/adapi/node/autoware_state"
        ));
        // Single-level wildcard matches one token, not several.
        assert!(is_node_name_matched("/adapi/*", "/adapi/node"));
        assert!(!is_node_name_matched(
            "/adapi/*",
            "/adapi/node/autoware_state"
        ));
    }

    fn write_tmp(contents: &str) -> tempfile::NamedTempFile {
        let mut f = tempfile::Builder::new().suffix(".yaml").tempfile().unwrap();
        f.write_all(contents.as_bytes()).unwrap();
        f
    }

    #[test]
    fn per_node_filtering_selects_wildcard_plus_own_section() {
        // Mirrors Autoware default_adapi.param.yaml: a shared file with a `/**`
        // section plus per-node FQN sections. A composable must receive only
        // `/**` + its own section, with its own section winning on collisions.
        let f = write_tmp(
            "/**:\n  ros__parameters:\n    diagnostic_updater.period: 1.0\n    update_rate: 0.1\n\
             /adapi/node/autoware_state:\n  ros__parameters:\n    update_rate: 10.0\n\
             /adapi/node/motion:\n  ros__parameters:\n    require_accept_start: false\n",
        );
        let path = f.path().to_str().unwrap();

        let params = load_yaml_params_for_node(path, "adapi/node/autoware_state").unwrap();
        let map: std::collections::HashMap<_, _> = params.into_iter().collect();

        // `/**` param present.
        assert_eq!(
            map.get("diagnostic_updater.period").map(String::as_str),
            Some("1.0")
        );
        // Own section wins over `/**` (10.0, not 0.1).
        assert_eq!(map.get("update_rate").map(String::as_str), Some("10.0"));
        // Another node's section MUST NOT leak in.
        assert!(!map.contains_key("require_accept_start"));
    }

    #[test]
    fn flat_param_file_without_node_keys_is_unfiltered() {
        let f = write_tmp("alpha: 1\nbeta: two\n");
        let path = f.path().to_str().unwrap();
        let params = load_yaml_params_for_node(path, "any/node").unwrap();
        let map: std::collections::HashMap<_, _> = params.into_iter().collect();
        assert_eq!(map.get("alpha").map(String::as_str), Some("1"));
        assert_eq!(map.get("beta").map(String::as_str), Some("two"));
    }
}
