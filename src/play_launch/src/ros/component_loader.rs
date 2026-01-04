//! Helper functions for ROS parameter conversion
//!
//! This module provides utilities for converting string parameters to ROS Parameter messages.
//! Originally part of the centralized ComponentLoader (removed in Phase 3), these helper
//! functions are now used by Container actors for LoadNode service calls.

use eyre::Result;

/// Convert string parameters to ROS Parameter messages
pub(crate) fn convert_parameters_to_ros(
    params: &[(String, String)],
) -> Result<Vec<rcl_interfaces::msg::Parameter>> {
    params
        .iter()
        .map(|(name, value)| {
            Ok(rcl_interfaces::msg::Parameter {
                name: name.clone(),
                value: parse_parameter_value(value)?,
            })
        })
        .collect()
}

/// Parse a parameter value string into a ParameterValue
///
/// This function mimics the behavior of ROS2 CLI's get_parameter_value() which uses
/// yaml.safe_load() to determine parameter types. This allows proper handling of
/// array parameters like [1, 2, 3] (integer_array) or [0.1, 0.2] (double_array).
fn parse_parameter_value(value: &str) -> Result<rcl_interfaces::msg::ParameterValue> {
    // Parse value as YAML to determine type
    let yaml_value: serde_yaml::Value = match serde_yaml::from_str(value) {
        Ok(v) => v,
        Err(_) => {
            // If YAML parsing fails, treat as string
            return Ok(create_string_parameter(value));
        }
    };

    match yaml_value {
        // Boolean
        serde_yaml::Value::Bool(b) => Ok(create_bool_parameter(b)),

        // Integer
        serde_yaml::Value::Number(n) if n.is_i64() => {
            Ok(create_integer_parameter(n.as_i64().unwrap()))
        }

        // Double (includes f64 and integers that need to be floats)
        serde_yaml::Value::Number(n) if n.is_f64() => {
            Ok(create_double_parameter(n.as_f64().unwrap()))
        }

        // Arrays (Sequence)
        serde_yaml::Value::Sequence(seq) => {
            if seq.is_empty() {
                // Empty array defaults to string array
                return Ok(create_string_array_parameter(Vec::new()));
            }

            // Check if all elements are the same type
            if seq.iter().all(|v| matches!(v, serde_yaml::Value::Bool(_))) {
                let values: Vec<bool> = seq.iter().filter_map(|v| v.as_bool()).collect();
                Ok(create_bool_array_parameter(values))
            } else if seq
                .iter()
                .all(|v| matches!(v, serde_yaml::Value::Number(n) if n.is_i64()))
            {
                let values: Vec<i64> = seq.iter().filter_map(|v| v.as_i64()).collect();
                Ok(create_integer_array_parameter(values))
            } else if seq
                .iter()
                .all(|v| matches!(v, serde_yaml::Value::Number(_)))
            {
                // All numbers, treat as doubles (handles mixed int/float)
                let values: Vec<f64> = seq.iter().filter_map(|v| v.as_f64()).collect();
                Ok(create_double_array_parameter(values))
            } else if seq
                .iter()
                .all(|v| matches!(v, serde_yaml::Value::String(_)))
            {
                let values: Vec<String> = seq
                    .iter()
                    .filter_map(|v| v.as_str().map(String::from))
                    .collect();
                Ok(create_string_array_parameter(values))
            } else {
                // Mixed types or unsupported, fall back to string
                Ok(create_string_parameter(value))
            }
        }

        // String or other types
        serde_yaml::Value::String(s) => Ok(create_string_parameter(&s)),
        _ => Ok(create_string_parameter(value)),
    }
}

// Helper functions to create ParameterValue structs
// These eliminate repetitive code and match the pattern used by ROS2 CLI

fn create_bool_parameter(value: bool) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
        bool_value: value,
        integer_value: 0,
        double_value: 0.0,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: Vec::new(),
        double_array_value: Vec::new(),
        string_array_value: Vec::new(),
    }
}

fn create_integer_parameter(value: i64) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
        bool_value: false,
        integer_value: value,
        double_value: 0.0,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: Vec::new(),
        double_array_value: Vec::new(),
        string_array_value: Vec::new(),
    }
}

fn create_double_parameter(value: f64) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
        bool_value: false,
        integer_value: 0,
        double_value: value,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: Vec::new(),
        double_array_value: Vec::new(),
        string_array_value: Vec::new(),
    }
}

fn create_string_parameter(value: &str) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
        bool_value: false,
        integer_value: 0,
        double_value: 0.0,
        string_value: value.to_string(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: Vec::new(),
        double_array_value: Vec::new(),
        string_array_value: Vec::new(),
    }
}

fn create_bool_array_parameter(values: Vec<bool>) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY,
        bool_value: false,
        integer_value: 0,
        double_value: 0.0,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: values,
        integer_array_value: Vec::new(),
        double_array_value: Vec::new(),
        string_array_value: Vec::new(),
    }
}

fn create_integer_array_parameter(values: Vec<i64>) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY,
        bool_value: false,
        integer_value: 0,
        double_value: 0.0,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: values,
        double_array_value: Vec::new(),
        string_array_value: Vec::new(),
    }
}

fn create_double_array_parameter(values: Vec<f64>) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY,
        bool_value: false,
        integer_value: 0,
        double_value: 0.0,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: Vec::new(),
        double_array_value: values,
        string_array_value: Vec::new(),
    }
}

fn create_string_array_parameter(values: Vec<String>) -> rcl_interfaces::msg::ParameterValue {
    rcl_interfaces::msg::ParameterValue {
        type_: rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
        bool_value: false,
        integer_value: 0,
        double_value: 0.0,
        string_value: String::new(),
        byte_array_value: Vec::new(),
        bool_array_value: Vec::new(),
        integer_array_value: Vec::new(),
        double_array_value: Vec::new(),
        string_array_value: values,
    }
}
