//! YAML parameter file loading helpers

use pyo3::PyResult;

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
/// A section is any mapping holding `ros__parameters`, at any depth, with or
/// without a leading `/` on its keys: Nav2's `nav2_params.yaml` writes
/// `amcl:` and `global_costmap: global_costmap:`. Reading only `/`-prefixed
/// keys flattened that whole file onto every Nav2 composable.
///
/// A file with NO `ros__parameters` anywhere (a plain flat param mapping) is
/// flattened whole.
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

    match play_launch_parser::params::sections_for_node(&yaml, node_fqn) {
        Some(sections) => {
            let mut merged: Vec<(String, String)> = Vec::new();
            for value in sections {
                apply_section(value, &mut merged);
            }
            Ok(merged)
        }
        // No `ros__parameters` anywhere — the file applies whole.
        None => {
            let mut params = Vec::new();
            flatten_yaml(&yaml, "", &mut params);
            Ok(params)
        }
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

#[cfg(test)]
use play_launch_parser::params::is_node_name_matched;

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

    /// Nav2's `nav2_params.yaml` names its sections without a leading `/`,
    /// and nests a costmap's under the costmap's own namespace. launch_ros
    /// finds `ros__parameters` at any depth, so map_server gets its own two
    /// values and nothing of amcl's; before, every composable received the
    /// whole file as `amcl.ros__parameters.alpha1` and the like.
    #[test]
    fn sections_without_a_slash_and_nested_sections_are_selected() {
        let f = write_tmp(
            "amcl:\n  ros__parameters:\n    alpha1: 0.2\n\
             map_server:\n  ros__parameters:\n    use_sim_time: true\n    yaml_filename: m.yaml\n\
             global_costmap:\n  global_costmap:\n    ros__parameters:\n      resolution: 0.05\n",
        );
        let path = f.path().to_str().unwrap();

        let map: std::collections::HashMap<_, _> = load_yaml_params_for_node(path, "map_server")
            .unwrap()
            .into_iter()
            .collect();
        assert_eq!(map.len(), 2, "{map:?}");
        assert_eq!(map.get("yaml_filename").map(String::as_str), Some("m.yaml"));
        assert_eq!(map.get("use_sim_time").map(String::as_str), Some("true"));

        let nested: std::collections::HashMap<_, _> =
            load_yaml_params_for_node(path, "global_costmap/global_costmap")
                .unwrap()
                .into_iter()
                .collect();
        assert_eq!(nested.get("resolution").map(String::as_str), Some("0.05"));
        assert_eq!(nested.len(), 1, "{nested:?}");

        // A node no section names gets nothing from the file.
        assert!(
            load_yaml_params_for_node(path, "amcl_other")
                .unwrap()
                .is_empty()
        );
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
