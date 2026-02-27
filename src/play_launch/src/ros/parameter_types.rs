//! Parameter types for the web UI parameter control feature (Phase 24).
//!
//! Wire-friendly types that bridge between `rcl_interfaces::msg` ROS messages
//! and JSON for the web API.

use serde::{Deserialize, Serialize};
#[cfg(test)]
use ts_rs::TS;

/// Integer range constraint from ParameterDescriptor.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
pub struct IntegerRange {
    pub from: i64,
    pub to: i64,
    pub step: u64,
}

/// Floating-point range constraint from ParameterDescriptor.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
pub struct FloatRange {
    pub from: f64,
    pub to: f64,
    pub step: f64,
}

/// Wire-friendly parameter value enum (maps to ParameterType constants).
///
/// Serde tagged representation allows the web UI to send/receive values like:
///   `{ "type": "Double", "value": 3.14 }`
///   `{ "type": "Bool", "value": true }`
///   `{ "type": "NotSet" }`
#[derive(Debug, Clone, Serialize, Deserialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
#[serde(tag = "type", content = "value")]
pub enum ParamValue {
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    BoolArray(Vec<bool>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<String>),
    ByteArray(Vec<u8>),
    NotSet,
}

/// Full parameter entry for the web UI.
#[derive(Debug, Clone, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
pub struct ParamEntry {
    pub name: String,
    pub value: ParamValue,
    /// Human-readable type name: "bool", "integer", "double", "string", etc.
    pub type_name: String,
    pub description: String,
    pub read_only: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[cfg_attr(test, ts(optional = nullable))]
    pub integer_range: Option<IntegerRange>,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[cfg_attr(test, ts(optional = nullable))]
    pub floating_point_range: Option<FloatRange>,
}

/// Result of a set operation.
#[derive(Debug, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
pub struct SetParamResult {
    pub successful: bool,
    pub reason: String,
}

/// A single parameter update for SSE events.
#[derive(Debug, Clone, Serialize)]
#[cfg_attr(test, derive(TS))]
#[cfg_attr(test, ts(export))]
pub struct ParamUpdate {
    pub param_name: String,
    pub value: ParamValue,
}

// --- Conversions between ParamValue and rcl_interfaces::msg types ---

impl ParamValue {
    /// Convert from an `rcl_interfaces::msg::ParameterValue` to `ParamValue`.
    pub fn from_ros(pv: &rcl_interfaces::msg::ParameterValue) -> Self {
        use rcl_interfaces::msg::ParameterType;
        match pv.type_ {
            ParameterType::PARAMETER_BOOL => ParamValue::Bool(pv.bool_value),
            ParameterType::PARAMETER_INTEGER => ParamValue::Integer(pv.integer_value),
            ParameterType::PARAMETER_DOUBLE => ParamValue::Double(pv.double_value),
            ParameterType::PARAMETER_STRING => ParamValue::String(pv.string_value.clone()),
            ParameterType::PARAMETER_BYTE_ARRAY => {
                ParamValue::ByteArray(pv.byte_array_value.clone())
            }
            ParameterType::PARAMETER_BOOL_ARRAY => {
                ParamValue::BoolArray(pv.bool_array_value.clone())
            }
            ParameterType::PARAMETER_INTEGER_ARRAY => {
                ParamValue::IntegerArray(pv.integer_array_value.clone())
            }
            ParameterType::PARAMETER_DOUBLE_ARRAY => {
                ParamValue::DoubleArray(pv.double_array_value.clone())
            }
            ParameterType::PARAMETER_STRING_ARRAY => {
                ParamValue::StringArray(pv.string_array_value.clone())
            }
            _ => ParamValue::NotSet,
        }
    }

    /// Convert to an `rcl_interfaces::msg::ParameterValue`.
    pub fn to_ros(&self) -> rcl_interfaces::msg::ParameterValue {
        use rcl_interfaces::msg::ParameterType;

        let mut pv = rcl_interfaces::msg::ParameterValue {
            type_: ParameterType::PARAMETER_NOT_SET,
            bool_value: false,
            integer_value: 0,
            double_value: 0.0,
            string_value: String::new(),
            byte_array_value: Vec::new(),
            bool_array_value: Vec::new(),
            integer_array_value: Vec::new(),
            double_array_value: Vec::new(),
            string_array_value: Vec::new(),
        };

        match self {
            ParamValue::Bool(v) => {
                pv.type_ = ParameterType::PARAMETER_BOOL;
                pv.bool_value = *v;
            }
            ParamValue::Integer(v) => {
                pv.type_ = ParameterType::PARAMETER_INTEGER;
                pv.integer_value = *v;
            }
            ParamValue::Double(v) => {
                pv.type_ = ParameterType::PARAMETER_DOUBLE;
                pv.double_value = *v;
            }
            ParamValue::String(v) => {
                pv.type_ = ParameterType::PARAMETER_STRING;
                pv.string_value = v.clone();
            }
            ParamValue::ByteArray(v) => {
                pv.type_ = ParameterType::PARAMETER_BYTE_ARRAY;
                pv.byte_array_value = v.clone();
            }
            ParamValue::BoolArray(v) => {
                pv.type_ = ParameterType::PARAMETER_BOOL_ARRAY;
                pv.bool_array_value = v.clone();
            }
            ParamValue::IntegerArray(v) => {
                pv.type_ = ParameterType::PARAMETER_INTEGER_ARRAY;
                pv.integer_array_value = v.clone();
            }
            ParamValue::DoubleArray(v) => {
                pv.type_ = ParameterType::PARAMETER_DOUBLE_ARRAY;
                pv.double_array_value = v.clone();
            }
            ParamValue::StringArray(v) => {
                pv.type_ = ParameterType::PARAMETER_STRING_ARRAY;
                pv.string_array_value = v.clone();
            }
            ParamValue::NotSet => {}
        }

        pv
    }
}

/// Get a human-readable type name from a ParameterType constant.
pub fn type_name_from_ros(type_: u8) -> &'static str {
    use rcl_interfaces::msg::ParameterType;
    match type_ {
        ParameterType::PARAMETER_BOOL => "bool",
        ParameterType::PARAMETER_INTEGER => "integer",
        ParameterType::PARAMETER_DOUBLE => "double",
        ParameterType::PARAMETER_STRING => "string",
        ParameterType::PARAMETER_BYTE_ARRAY => "byte_array",
        ParameterType::PARAMETER_BOOL_ARRAY => "bool_array",
        ParameterType::PARAMETER_INTEGER_ARRAY => "integer_array",
        ParameterType::PARAMETER_DOUBLE_ARRAY => "double_array",
        ParameterType::PARAMETER_STRING_ARRAY => "string_array",
        _ => "not_set",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_param_value_roundtrip_bool() {
        let pv = ParamValue::Bool(true);
        let ros = pv.to_ros();
        assert_eq!(
            ros.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_BOOL
        );
        assert!(ros.bool_value);
        let back = ParamValue::from_ros(&ros);
        assert!(matches!(back, ParamValue::Bool(true)));
    }

    #[test]
    fn test_param_value_roundtrip_double() {
        let pv = ParamValue::Double(3.14);
        let ros = pv.to_ros();
        assert_eq!(
            ros.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE
        );
        assert!((ros.double_value - 3.14).abs() < f64::EPSILON);
        let back = ParamValue::from_ros(&ros);
        assert!(matches!(back, ParamValue::Double(v) if (v - 3.14).abs() < f64::EPSILON));
    }

    #[test]
    fn test_param_value_roundtrip_string_array() {
        let pv = ParamValue::StringArray(vec!["a".into(), "b".into()]);
        let ros = pv.to_ros();
        assert_eq!(
            ros.type_,
            rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY
        );
        assert_eq!(ros.string_array_value, vec!["a", "b"]);
    }

    #[test]
    fn test_param_value_serde_roundtrip() {
        let pv = ParamValue::Integer(42);
        let json = serde_json::to_string(&pv).unwrap();
        assert!(json.contains("\"type\":\"Integer\""));
        assert!(json.contains("\"value\":42"));
        let back: ParamValue = serde_json::from_str(&json).unwrap();
        assert!(matches!(back, ParamValue::Integer(42)));
    }

    #[test]
    fn test_type_name_from_ros() {
        assert_eq!(
            type_name_from_ros(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL),
            "bool"
        );
        assert_eq!(
            type_name_from_ros(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY),
            "double_array"
        );
        assert_eq!(type_name_from_ros(255), "not_set");
    }
}
