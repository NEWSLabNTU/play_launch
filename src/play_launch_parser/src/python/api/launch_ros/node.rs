//! Mock Node class for launch_ros.actions

use super::helpers::{is_yaml_file, load_yaml_params};
use crate::{
    captures::NodeCapture,
    python::bridge::{capture_node, get_current_ros_namespace},
};
use pyo3::{
    prelude::*,
    types::{PyAny, PyDict, PyList},
};

/// Mock Node class
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import Node
/// node = Node(
///     package='my_package',
///     executable='my_executable',
///     name='my_node',
///     namespace='/my_namespace',
///     parameters=[...],
///     remappings=[...],
///     arguments=[...],
/// )
/// ```
///
/// When constructed, automatically captures the node definition.
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct Node {
    package: String,
    executable: String,
    name: Option<String>,
    namespace: Option<String>,
    parameters: Vec<PyObject>,
    remappings: Vec<PyObject>,
    arguments: Vec<String>,
    env_vars: Vec<(String, String)>,
    #[allow(dead_code)] // Used for condition evaluation, not stored in captures
    condition: Option<PyObject>,
}

#[pymethods]
impl Node {
    #[new]
    #[pyo3(signature = (
        *,
        package,
        executable,
        name=None,
        namespace=None,
        parameters=None,
        remappings=None,
        arguments=None,
        env=None,
        condition=None,
        **_kwargs
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        py: Python,
        package: PyObject,
        executable: PyObject,
        name: Option<PyObject>,
        namespace: Option<PyObject>,
        parameters: Option<Vec<PyObject>>,
        remappings: Option<Vec<PyObject>>,
        arguments: Option<Vec<PyObject>>,
        env: Option<Vec<(String, String)>>,
        condition: Option<PyObject>,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<Self> {
        // Convert PyObjects to strings (handles both strings and substitutions)
        log::debug!("Node::new: creating node with package and executable");
        let package = Self::pyobject_to_string(py, &package)?;
        let executable = Self::pyobject_to_string(py, &executable)?;

        log::debug!(
            "Node::new: package='{}', executable='{}'",
            package,
            executable
        );

        let name = name
            .map(|obj| {
                log::debug!("Node::new: processing name parameter");
                Self::pyobject_to_string(py, &obj)
            })
            .transpose()?;

        let namespace = namespace
            .map(|obj| {
                log::debug!("Node::new: processing namespace parameter");
                Self::pyobject_to_string(py, &obj)
            })
            .transpose()?;

        // Convert arguments from Vec<PyObject> to Vec<String>
        let arguments_vec = if let Some(args) = arguments {
            args.iter()
                .map(|obj| Self::pyobject_to_string(py, obj))
                .collect::<Result<Vec<String>, _>>()?
        } else {
            Vec::new()
        };

        log::debug!("Node::new: name={:?}, namespace={:?}", name, namespace);

        let node = Self {
            package: package.clone(),
            executable: executable.clone(),
            name: name.clone(),
            namespace: namespace.clone(),
            parameters: parameters.unwrap_or_default(),
            remappings: remappings.unwrap_or_default(),
            arguments: arguments_vec,
            env_vars: env.unwrap_or_default(),
            condition: condition.clone(),
        };

        // Evaluate condition (if present) and only capture if true
        let should_capture = if let Some(cond_obj) = &condition {
            Self::evaluate_condition(py, cond_obj).unwrap_or(true)
        } else {
            true // No condition means always capture
        };

        if should_capture {
            // Capture this node immediately
            Self::capture_node(&node);
        } else {
            log::debug!(
                "Skipping node capture due to condition: {} / {}",
                package,
                executable
            );
        }

        Ok(node)
    }

    fn __repr__(&self) -> String {
        format!(
            "Node(package='{}', executable='{}', name='{}')",
            self.package,
            self.executable,
            self.name.as_deref().unwrap_or(&self.executable)
        )
    }
}

impl Node {
    /// Evaluate a condition object
    ///
    /// Calls the evaluate() method on the condition if it exists
    pub(super) fn evaluate_condition(py: Python, condition: &PyObject) -> PyResult<bool> {
        let cond_ref = condition.as_ref(py);

        // Try calling evaluate() method on the condition object
        // Note: The py: Python parameter is automatically injected by pyo3,
        // so we call with no arguments from Python's perspective
        if let Ok(result) = cond_ref.call_method0("evaluate")
            && let Ok(bool_val) = result.extract::<bool>()
        {
            log::debug!("Condition evaluated to: {}", bool_val);
            return Ok(bool_val);
        }

        // Fallback: treat as truthy if we can't evaluate
        log::warn!("Failed to evaluate condition, defaulting to true");
        Ok(true)
    }

    /// Convert a PyObject to a string (handles both strings and substitutions)
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        crate::python::api::utils::pyobject_to_string(py, obj)
    }

    /// Capture node to global storage
    fn capture_node(node: &Node) {
        // Parse parameters and remappings from Python objects
        let all_params = Python::with_gil(|py| node.parse_parameters(py).unwrap_or_default());
        let remappings = Python::with_gil(|py| node.parse_remappings(py).unwrap_or_default());

        // Separate regular parameters from parameter files
        let mut parameters = Vec::new();
        let mut params_files = Vec::new();
        for (key, value) in all_params {
            if key == "__param_file" {
                params_files.push(value);
            } else {
                parameters.push((key, value));
            }
        }

        // Get current ROS namespace and combine with node's namespace
        let ros_namespace = get_current_ros_namespace();
        log::debug!(
            "Node::capture_node: package={}, exec={}, node_ns={:?}, ros_namespace={}",
            node.package,
            node.executable,
            node.namespace,
            ros_namespace
        );
        let node_ns = node.namespace.as_ref().map(|ns| {
            if ns.is_empty() {
                String::new()
            } else if ns.starts_with('/') {
                ns.clone()
            } else {
                format!("/{}", ns)
            }
        });

        let full_namespace = match node_ns {
            Some(ns) => {
                if ros_namespace == "/" {
                    if ns.is_empty() || ns == "/" {
                        // Empty namespace or root namespace = no namespace (matches Python behavior)
                        None
                    } else {
                        Some(ns)
                    }
                } else if ns.is_empty() {
                    Some(ros_namespace.clone())
                } else {
                    Some(format!("{}{}", ros_namespace, ns))
                }
            }
            None => {
                if ros_namespace == "/" {
                    None
                } else {
                    Some(ros_namespace.clone())
                }
            }
        };

        let params_files_count = params_files.len();
        let capture = NodeCapture {
            package: node.package.clone(),
            executable: node.executable.clone(),
            name: node.name.clone(),
            namespace: full_namespace.clone(),
            parameters,
            params_files,
            remappings,
            arguments: node.arguments.clone(),
            env_vars: node.env_vars.clone(),
        };

        log::debug!(
            "Captured Python node: {} / {} (ros_namespace: {}, full_namespace: {:?}, params_files: {})",
            capture.package,
            capture.executable,
            ros_namespace,
            full_namespace,
            params_files_count
        );

        capture_node(capture);
    }

    /// Parse Python parameters to string tuples
    ///
    /// Handles:
    /// - Dict parameters: `{'param': 'value'}` -> `[("param", "value")]`
    /// - Nested dicts: `{'ns': {'param': 'value'}}` -> `[("ns.param", "value")]`
    /// - List of dicts: `[{'p1': 'v1'}, {'p2': 'v2'}]` -> `[("p1", "v1"), ("p2", "v2")]`
    /// - Parameter files: `/path/to/params.yaml` -> special marker
    fn parse_parameters(&self, py: Python) -> PyResult<Vec<(String, String)>> {
        let mut parsed_params = Vec::new();

        for param_obj in &self.parameters {
            // Try to extract as different types
            let param_any = param_obj.as_ref(py);

            // Case 1: String (parameter file path or literal value)
            if let Ok(path) = param_any.extract::<String>() {
                // Check if it's a YAML parameter file
                if is_yaml_file(&path) {
                    // Load and expand YAML parameter file
                    match load_yaml_params(&path) {
                        Ok(yaml_params) => {
                            log::debug!("Loaded {} parameters from {}", yaml_params.len(), path);
                            parsed_params.extend(yaml_params);
                        }
                        Err(e) => {
                            log::warn!("Failed to load parameter file {}: {}", path, e);
                            // Fallback: store as __param_file for backward compatibility
                            parsed_params.push(("__param_file".to_string(), path));
                        }
                    }
                } else if path.contains('/') {
                    // Looks like a file path but not YAML - store as reference
                    parsed_params.push(("__param_file".to_string(), path));
                } else {
                    // Treat as a literal parameter value
                    parsed_params.push(("value".to_string(), path));
                }
                continue;
            }

            // Case 2: Dict (single parameter dict or nested dict)
            if let Ok(dict) = param_any.downcast::<PyDict>() {
                Self::parse_dict_params(dict, "", &mut parsed_params)?;
                continue;
            }

            // Case 3: List (list of parameter dicts)
            if let Ok(list) = param_any.downcast::<PyList>() {
                for item in list.iter() {
                    if let Ok(dict) = item.downcast::<PyDict>() {
                        Self::parse_dict_params(dict, "", &mut parsed_params)?;
                    }
                }
                continue;
            }

            // Case 4: ParameterFile object -- load YAML and expand inline
            let type_name = param_any.get_type().name().unwrap_or("");
            if type_name.contains("ParameterFile") {
                if let Ok(str_val) = param_any.call_method0("__str__")
                    && let Ok(path) = str_val.extract::<String>()
                {
                    if is_yaml_file(&path) {
                        match load_yaml_params(&path) {
                            Ok(yaml_params) => {
                                log::debug!(
                                    "Loaded {} parameters from ParameterFile {}",
                                    yaml_params.len(),
                                    path
                                );
                                parsed_params.extend(yaml_params);
                            }
                            Err(e) => {
                                log::warn!("Failed to load ParameterFile {}: {}", path, e);
                                parsed_params.push(("__param_file".to_string(), path));
                            }
                        }
                    } else {
                        parsed_params.push(("__param_file".to_string(), path));
                    }
                }
                continue;
            }

            // Case 5: Try calling __str__ on the object (for substitutions)
            if let Ok(str_val) = param_any.call_method0("__str__")
                && let Ok(s) = str_val.extract::<String>()
            {
                if is_yaml_file(&s) {
                    match load_yaml_params(&s) {
                        Ok(yaml_params) => parsed_params.extend(yaml_params),
                        Err(_) => parsed_params.push(("substitution".to_string(), s)),
                    }
                } else {
                    parsed_params.push(("substitution".to_string(), s));
                }
            }
        }

        Ok(parsed_params)
    }

    /// Parse a Python dict into parameter tuples
    ///
    /// Handles nested dicts by using dot notation: `{"ns": {"param": "value"}}` -> `("ns.param", "value")`
    pub(super) fn parse_dict_params(
        dict: &PyDict,
        prefix: &str,
        params: &mut Vec<(String, String)>,
    ) -> PyResult<()> {
        for (key, value) in dict.iter() {
            let key_str = key.extract::<String>()?;
            let full_key = if prefix.is_empty() {
                key_str.clone()
            } else {
                format!("{}.{}", prefix, key_str)
            };

            // Check if value is a nested dict
            if let Ok(nested_dict) = value.downcast::<PyDict>() {
                // Recursively parse nested dict
                Self::parse_dict_params(nested_dict, &full_key, params)?;
            } else {
                let type_name = value.get_type().name().unwrap_or("?");
                if type_name == "dict" {
                    // Value is a dict but downcast failed (e.g., from CPython yaml.safe_load)
                    // Use PyAny dict API instead
                    log::debug!(
                        "parse_dict_params: '{}' has dict type but downcast failed, using items()",
                        full_key
                    );
                    if let Ok(items) = value.call_method0("items")
                        && let Ok(iter) = items.iter()
                    {
                        for item in iter.flatten() {
                            if let Ok((k, v)) = item.extract::<(String, &PyAny)>() {
                                let sub_key = if full_key.is_empty() {
                                    k.clone()
                                } else {
                                    format!("{}.{}", full_key, k)
                                };
                                // Recurse if the sub-value is also a dict
                                if v.get_type().name().unwrap_or("") == "dict" {
                                    if let Ok(sub_dict) = v.downcast::<PyDict>() {
                                        Self::parse_dict_params(sub_dict, &sub_key, params)?;
                                    } else {
                                        let val = Self::extract_param_value(v)?;
                                        params.push((sub_key, val));
                                    }
                                } else {
                                    let val = Self::extract_param_value(v)?;
                                    params.push((sub_key, val));
                                }
                            }
                        }
                        continue;
                    }
                }
                // Extract value as string (handles various Python types)
                let value_str = Self::extract_param_value(value)?;
                params.push((full_key, value_str));
            }
        }

        Ok(())
    }

    /// Extract a parameter value from a Python object
    ///
    /// Handles: str, int, float, bool, substitutions (including PythonExpression), and objects with __str__
    pub(super) fn extract_param_value(value: &PyAny) -> PyResult<String> {
        use pyo3::types::{PyBool, PyList};

        // Try direct string extraction
        if let Ok(s) = value.extract::<String>() {
            return Ok(s);
        }

        // Try boolean BEFORE integer (Python bools are subclass of int)
        if value.is_instance_of::<PyBool>()
            && let Ok(b) = value.extract::<bool>()
        {
            return Ok(if b { "true" } else { "false" }.to_string());
        }

        // Try integer
        if let Ok(i) = value.extract::<i64>() {
            return Ok(i.to_string());
        }

        // Try float
        if let Ok(f) = value.extract::<f64>() {
            // Always format floats with decimal point to preserve type information
            if f.fract() == 0.0 && f.is_finite() {
                return Ok(format!("{:.1}", f));
            } else {
                return Ok(f.to_string());
            }
        }

        // Try list (convert to Python-style string with quoted string elements)
        if let Ok(list) = value.downcast::<PyList>() {
            let mut formatted_items = Vec::new();
            for item in list.iter() {
                let val = Self::extract_param_value(item)?;
                // Quote string elements (non-numeric, non-boolean) with single quotes
                // to match Python parser output format: ['a', 'b'] not [a, b]
                let is_numeric_or_bool = val.parse::<f64>().is_ok()
                    || val.parse::<i64>().is_ok()
                    || val == "true"
                    || val == "false";
                if is_numeric_or_bool {
                    formatted_items.push(val);
                } else {
                    formatted_items.push(format!("'{}'", val));
                }
            }
            return Ok(format!("[{}]", formatted_items.join(", ")));
        }

        // For substitutions (including PythonExpression), use the centralized utility
        // This handles evaluation of PythonExpression and other evaluating substitutions
        let py = value.py();
        let obj_py = value.to_object(py);
        crate::python::api::utils::pyobject_to_string(py, &obj_py)
    }

    /// Parse Python remappings to string tuples
    ///
    /// Handles:
    /// - Tuple pairs: `[('old_topic', 'new_topic')]` -> `[("old_topic", "new_topic")]`
    /// - LaunchConfiguration objects in tuples: `[('topic', LaunchConfiguration('name'))]`
    /// - Substitutions that need string conversion
    fn parse_remappings(&self, py: Python) -> PyResult<Vec<(String, String)>> {
        let mut parsed_remaps = Vec::new();

        for remap_obj in &self.remappings {
            let remap_any = remap_obj.as_ref(py);

            // Remappings should be tuples of (from, to)
            if let Ok(remap_tuple) = remap_any.downcast::<pyo3::types::PyTuple>()
                && remap_tuple.len() == 2
            {
                // Extract both elements and convert to strings
                // This handles both plain strings and LaunchConfiguration objects
                let from_obj = remap_tuple.get_item(0)?;
                let to_obj = remap_tuple.get_item(1)?;

                // Convert to strings (handles LaunchConfiguration via __str__)
                let from = if let Ok(s) = from_obj.extract::<String>() {
                    s
                } else if let Ok(str_result) = from_obj.call_method0("__str__") {
                    str_result.extract::<String>()?
                } else {
                    from_obj.to_string()
                };

                let to = if let Ok(s) = to_obj.extract::<String>() {
                    s
                } else if let Ok(str_result) = to_obj.call_method0("__str__") {
                    str_result.extract::<String>()?
                } else {
                    to_obj.to_string()
                };

                parsed_remaps.push((from, to));
            }
        }

        Ok(parsed_remaps)
    }
}
