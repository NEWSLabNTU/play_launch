//! Mock LifecycleNode and LifecycleTransition classes for launch_ros.actions

use super::helpers::is_yaml_file;
use pyo3::{prelude::*, types::PyDict};

/// Mock LifecycleNode class
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import LifecycleNode
///
/// lifecycle_node = LifecycleNode(
///     package='my_package',
///     executable='my_lifecycle_node',
///     name='my_node',
///     namespace='/my_namespace',
///     output='screen'
/// )
/// ```
///
/// LifecycleNode is similar to Node but adds lifecycle management support
/// For now, we treat it the same as a regular Node
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct LifecycleNode {
    package: String,
    executable: String,
    name: Option<String>,
    namespace: Option<String>,
    parameters: Vec<Py<PyAny>>,
    remappings: Vec<Py<PyAny>>,
    arguments: Vec<Py<PyAny>>,
    ros_arguments: Vec<Py<PyAny>>,
    #[allow(dead_code)] // Keep for API compatibility
    output: String,
}

#[pymethods]
impl LifecycleNode {
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
        ros_arguments=None,
        output=None,
        **_kwargs
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        py: Python,
        package: Py<PyAny>,
        executable: Py<PyAny>,
        name: Option<Py<PyAny>>,
        namespace: Option<Py<PyAny>>,
        parameters: Option<Vec<Py<PyAny>>>,
        remappings: Option<Vec<Py<PyAny>>>,
        arguments: Option<Vec<Py<PyAny>>>,
        ros_arguments: Option<Vec<Py<PyAny>>>,
        output: Option<String>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<Self> {
        // Convert PyObjects to strings (handles both strings and substitutions)
        let package_str = Self::pyobject_to_string_static(py, &package)?;
        let executable_str = Self::pyobject_to_string_static(py, &executable)?;

        let name_str = name
            .map(|obj| Self::pyobject_to_string_static(py, &obj))
            .transpose()?;

        let namespace_str = namespace
            .map(|obj| Self::pyobject_to_string_static(py, &obj))
            .transpose()?;

        let node = Self {
            package: package_str,
            executable: executable_str,
            name: name_str,
            namespace: namespace_str,
            parameters: parameters.unwrap_or_default(),
            remappings: remappings.unwrap_or_default(),
            arguments: arguments.unwrap_or_default(),
            ros_arguments: ros_arguments.unwrap_or_default(),
            output: output.unwrap_or_else(|| "screen".to_string()),
        };

        // Capture as a regular node (lifecycle management not supported in static parsing)
        Self::capture_node(&node, py)?;

        Ok(node)
    }

    fn __repr__(&self) -> String {
        format!(
            "LifecycleNode(package='{}', executable='{}', name='{}')",
            self.package,
            self.executable,
            self.name.as_deref().unwrap_or(&self.executable)
        )
    }
}

impl LifecycleNode {
    /// Convert Py<PyAny> to string (static version for use in constructor)
    fn pyobject_to_string_static(py: Python, obj: &Py<PyAny>) -> PyResult<String> {
        // Try direct string extraction
        if let Ok(s) = obj.extract::<String>(py) {
            return Ok(s);
        }

        // Try calling __str__ method (for substitutions)
        if let Ok(str_result) = obj.call_method0(py, "__str__")
            && let Ok(s) = str_result.extract::<String>(py)
        {
            return Ok(s);
        }

        // Fallback to repr
        Ok(obj.bind(py).str()?.to_string())
    }

    /// Capture the lifecycle node as a regular NodeCapture
    fn capture_node(&self, py: Python) -> PyResult<()> {
        use crate::{
            bridge::{capture_node, get_current_ros_namespace},
            captures::NodeCapture,
        };

        // Parse parameters (same logic as regular Node)
        let all_params = self.parse_parameters(py)?;

        // Separate regular parameters from parameter files
        let mut parsed_params = Vec::new();
        let mut params_files = Vec::new();
        for (key, value) in all_params {
            if key == "__param_file" {
                params_files.push(value);
            } else {
                parsed_params.push((key, value));
            }
        }

        // Parse remappings
        let parsed_remaps = self.parse_remappings(py)?;

        // Parse arguments
        let parsed_args = self.parse_arguments(py, &self.arguments)?;
        // Issue #9 — its own `--ros-args` block, not part of `arguments`.
        let parsed_ros_args = self.parse_arguments(py, &self.ros_arguments)?;

        // Get current ROS namespace and combine with node's namespace
        let ros_namespace = get_current_ros_namespace();
        let node_ns = self.namespace.as_ref().map(|ns| {
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
            package: self.package.clone(),
            executable: self.executable.clone(),
            name: self.name.clone(),
            namespace: full_namespace,
            parameters: parsed_params,
            params_files,
            param_sources: Vec::new(),
            remappings: parsed_remaps,
            arguments: parsed_args,
            ros_arguments: parsed_ros_args,
            env_vars: Vec::new(),
            scope_id: None,
        };

        log::debug!(
            "Captured Python lifecycle node: {} / {} (ros_namespace: {}, params_files: {})",
            capture.package,
            capture.executable,
            ros_namespace,
            params_files_count
        );

        capture_node(capture);
        Ok(())
    }

    /// Parse Python parameters (similar to Node)
    fn parse_parameters(&self, py: Python) -> PyResult<Vec<(String, String)>> {
        let mut parsed_params = Vec::new();

        log::debug!(
            "LifecycleNode parse_parameters: Processing {} parameter objects",
            self.parameters.len()
        );

        for param_obj in self.parameters.iter() {
            let param_any = param_obj.bind(py);

            // Try to extract as dict (most common case for parameters)
            if let Ok(param_dict) = param_any.cast::<pyo3::types::PyDict>() {
                for (key, value) in param_dict.iter() {
                    let key_str = key.extract::<String>()?;
                    // Convert value to string
                    let value_str = Self::pyobject_to_string(&value)?;
                    parsed_params.push((key_str, value_str));
                }
                continue;
            }

            // Try to extract as string (path to YAML parameter file). Kept as a
            // params_files REFERENCE (runtime `--params-file` filtering), not
            // flattened inline — see the Node Case 1 comment for why.
            if let Ok(path_str) = param_any.extract::<String>() {
                parsed_params.push(("__param_file".to_string(), path_str));
                continue;
            }

            // ParameterFile object — kept as a params_files reference.
            let type_name = param_any
                .get_type()
                .name()
                .map(|n| n.to_string())
                .unwrap_or_default();
            if type_name.contains("ParameterFile") {
                if let Ok(str_val) = param_any.call_method0("__str__")
                    && let Ok(path) = str_val.extract::<String>()
                {
                    parsed_params.push(("__param_file".to_string(), path));
                }
                continue;
            }

            // Try calling __str__ on the object (for substitutions)
            if let Ok(str_val) = param_any.call_method0("__str__")
                && let Ok(s) = str_val.extract::<String>()
            {
                if is_yaml_file(&s) {
                    parsed_params.push(("__param_file".to_string(), s));
                } else {
                    parsed_params.push(("substitution".to_string(), s));
                }
            }
        }

        Ok(parsed_params)
    }

    /// Parse Python remappings (similar to Node)
    fn parse_remappings(&self, py: Python) -> PyResult<Vec<(String, String)>> {
        let mut parsed_remaps = Vec::new();

        for remap_obj in &self.remappings {
            let remap_any = remap_obj.bind(py);
            if let Ok(remap_tuple) = remap_any.cast::<pyo3::types::PyTuple>()
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
                    from_obj.str()?.to_string()
                };

                let to = if let Ok(s) = to_obj.extract::<String>() {
                    s
                } else if let Ok(str_result) = to_obj.call_method0("__str__") {
                    str_result.extract::<String>()?
                } else {
                    to_obj.str()?.to_string()
                };

                parsed_remaps.push((from, to));
            }
        }

        Ok(parsed_remaps)
    }

    /// Parse Python arguments (similar to Node)
    fn parse_arguments(&self, py: Python, source: &[Py<PyAny>]) -> PyResult<Vec<String>> {
        let mut parsed_args = Vec::new();

        for arg_obj in source {
            if let Ok(arg_str) = arg_obj.extract::<String>(py) {
                parsed_args.push(arg_str);
            }
        }

        Ok(parsed_args)
    }

    /// Convert Py<PyAny> to string (handles substitutions)
    fn pyobject_to_string(obj: &Bound<'_, PyAny>) -> PyResult<String> {
        let py = obj.py();
        let py_obj: Py<PyAny> = obj.clone().unbind();
        crate::python::api::utils::pyobject_to_string(py, &py_obj)
    }
}

/// Mock LifecycleTransition action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import LifecycleTransition
///
/// LifecycleTransition(
///     lifecycle_node_names=['my_lifecycle_node'],
///     transition_id=3,  # e.g., configure, activate, etc.
///     transition_label='activate'
/// )
/// ```
///
/// Triggers a lifecycle state transition for managed nodes.
/// For static analysis, we just capture the intent without executing transitions.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct LifecycleTransition {
    #[allow(dead_code)] // Stored for API compatibility
    lifecycle_node_names: Vec<String>,
    #[allow(dead_code)] // Stored for API compatibility
    transition_id: Option<i32>,
    #[allow(dead_code)] // Stored for API compatibility
    transition_label: Option<String>,
}

#[pymethods]
impl LifecycleTransition {
    #[new]
    #[pyo3(signature = (*, lifecycle_node_names, transition_id=None, transition_label=None, **_kwargs))]
    fn new(
        lifecycle_node_names: Vec<String>,
        transition_id: Option<i32>,
        transition_label: Option<String>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        log::debug!(
            "Python Launch LifecycleTransition: nodes={:?}, transition={:?}",
            lifecycle_node_names,
            transition_label
        );
        // For static analysis, we just capture the action
        // We don't actually trigger lifecycle transitions
        Self {
            lifecycle_node_names,
            transition_id,
            transition_label,
        }
    }

    fn __repr__(&self) -> String {
        if let Some(ref label) = self.transition_label {
            format!(
                "LifecycleTransition(nodes={:?}, transition='{}')",
                self.lifecycle_node_names, label
            )
        } else if let Some(id) = self.transition_id {
            format!(
                "LifecycleTransition(nodes={:?}, transition_id={})",
                self.lifecycle_node_names, id
            )
        } else {
            format!("LifecycleTransition(nodes={:?})", self.lifecycle_node_names)
        }
    }
}
