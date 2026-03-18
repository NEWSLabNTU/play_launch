//! Mock ComposableNode class for launch_ros.descriptions

use super::{
    helpers::{is_yaml_file, load_yaml_params},
    node::Node,
};
use crate::{captures::LoadNodeCapture, python::bridge::capture_load_node};
use pyo3::{
    prelude::*,
    types::{PyDict, PyList},
};

/// Mock ComposableNode class (for descriptions module)
///
/// Python equivalent:
/// ```python
/// from launch_ros.descriptions import ComposableNode
///
/// node = ComposableNode(
///     package='my_package',
///     plugin='my_package::MyPlugin',
///     name='my_node',
///     namespace='/my_namespace',
///     parameters=[...],
///     remappings=[...],
/// )
/// ```
#[pyclass(module = "launch_ros.descriptions")]
#[derive(Clone)]
pub struct ComposableNode {
    package: String,
    plugin: String,
    name: String,
    namespace: Option<String>,
    parameters: Vec<PyObject>,
    remappings: Vec<PyObject>,
}

#[pymethods]
impl ComposableNode {
    #[new]
    #[pyo3(signature = (
        *,
        package,
        plugin,
        name,
        namespace=None,
        parameters=None,
        remappings=None,
        **_kwargs
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        py: Python,
        package: PyObject,
        plugin: PyObject,
        name: PyObject,
        namespace: Option<PyObject>,
        parameters: Option<Vec<PyObject>>,
        remappings: Option<Vec<PyObject>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<Self> {
        // Convert PyObjects to strings (handles both strings and substitutions/lists)
        log::debug!("ComposableNode::new: creating composable node");
        let package_str = Self::pyobject_to_string(py, &package)?;
        let plugin_str = Self::pyobject_to_string(py, &plugin)?;
        let name_str = Self::pyobject_to_string(py, &name)?;

        let namespace_str = namespace
            .map(|ns| {
                log::debug!("ComposableNode::new: processing namespace parameter");
                Self::pyobject_to_string(py, &ns)
            })
            .transpose()?;

        log::debug!(
            "ComposableNode::new: package='{}', plugin='{}', name='{}', namespace={:?}",
            package_str,
            plugin_str,
            name_str,
            namespace_str
        );
        Ok(Self {
            package: package_str,
            plugin: plugin_str,
            name: name_str,
            namespace: namespace_str,
            parameters: parameters.unwrap_or_default(),
            remappings: remappings.unwrap_or_default(),
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "ComposableNode(package='{}', plugin='{}', name='{}')",
            self.package, self.plugin, self.name
        )
    }

    // Getter methods for LoadComposableNodes to access attributes
    #[getter]
    fn package(&self) -> &str {
        &self.package
    }

    #[getter]
    fn plugin(&self) -> &str {
        &self.plugin
    }

    #[getter]
    fn name(&self) -> &str {
        &self.name
    }

    #[getter]
    fn namespace(&self) -> Option<&str> {
        self.namespace.as_deref()
    }

    #[getter]
    fn parameters(&self) -> Vec<PyObject> {
        self.parameters.clone()
    }

    #[getter]
    fn remappings(&self) -> Vec<PyObject> {
        self.remappings.clone()
    }
}

impl ComposableNode {
    /// Convert a PyObject to a string (handles both strings and substitutions)
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        crate::python::api::utils::pyobject_to_string(py, obj)
    }

    pub(super) fn capture_as_load_node(
        &self,
        container_name: &str,
        container_namespace: &Option<String>,
        ros_namespace: &str,
    ) {
        // Parse parameters and remappings from Python objects
        let parameters = Python::with_gil(|py| self.parse_parameters(py).unwrap_or_default());
        let remappings = Python::with_gil(|py| self.parse_remappings(py).unwrap_or_default());

        // Build full target container name: namespace + name
        // Normalize to ensure consistent path format with leading slash
        let target_container_name = if let Some(ns) = container_namespace {
            Self::normalize_namespace_path(ns, container_name)
        } else {
            // If no namespace, assume root and add leading slash
            if container_name.starts_with('/') {
                container_name.to_string()
            } else {
                format!("/{}", container_name)
            }
        };

        // Resolve node namespace using ROS context namespace (from push-ros-namespace).
        // This matches ROS2's get_composable_node_load_request() behavior:
        // - If the node has an explicit namespace: resolve relative to ROS context
        // - If no namespace: inherit the ROS context namespace
        let normalized_namespace = if let Some(ns) = &self.namespace {
            if ns.is_empty() {
                ros_namespace.to_string()
            } else if ns.starts_with('/') {
                // Absolute namespace — use as-is
                ns.clone()
            } else {
                // Relative namespace — combine with ROS context namespace
                if ros_namespace == "/" {
                    format!("/{}", ns)
                } else {
                    format!("{}/{}", ros_namespace, ns)
                }
            }
        } else {
            // No explicit namespace — use the ROS context namespace
            ros_namespace.to_string()
        };

        let capture = LoadNodeCapture {
            package: self.package.clone(),
            plugin: self.plugin.clone(),
            target_container_name,
            node_name: self.name.clone(),
            namespace: normalized_namespace,
            parameters,
            remappings,
            scope_id: None,
        };

        log::debug!(
            "Captured Python composable node: {} / {} (container: {})",
            capture.package,
            capture.plugin,
            capture.target_container_name
        );

        capture_load_node(capture);
    }

    /// Normalize namespace + name into a proper path
    fn normalize_namespace_path(namespace: &str, name: &str) -> String {
        // Handle empty name - just return namespace
        if name.is_empty() {
            return if namespace.is_empty() || namespace == "/" {
                "/".to_string()
            } else if namespace.starts_with('/') {
                namespace.to_string()
            } else {
                format!("/{}", namespace)
            };
        }

        // Handle empty namespace
        if namespace.is_empty() {
            return if name.starts_with('/') {
                name.to_string()
            } else {
                format!("/{}", name)
            };
        }

        let ns = if namespace == "/" {
            ""
        } else if namespace.starts_with('/') {
            namespace
        } else {
            return format!("/{}/{}", namespace, name);
        };

        format!("{}/{}", ns, name)
    }

    /// Parse Python parameters to string tuples (same logic as Node)
    fn parse_parameters(&self, py: Python) -> PyResult<Vec<(String, String)>> {
        let mut parsed_params = Vec::new();

        for param_obj in &self.parameters {
            let param_any = param_obj.bind(py);

            // String (parameter file path or literal value)
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

            // Dict (single parameter dict or nested dict)
            if let Ok(dict) = param_any.downcast::<PyDict>() {
                Node::parse_dict_params(dict, "", &mut parsed_params)?;
                continue;
            }

            // List (list of parameter dicts)
            if let Ok(list) = param_any.downcast::<PyList>() {
                for item in list.iter() {
                    if let Ok(dict) = item.downcast::<PyDict>() {
                        Node::parse_dict_params(dict, "", &mut parsed_params)?;
                    }
                }
                continue;
            }

            // ParameterFile object — load YAML and expand inline
            let type_name = param_any
                .get_type()
                .name()
                .map(|n| n.to_string())
                .unwrap_or_default();
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

            // Try calling __str__ on the object (for substitutions)
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

    /// Parse Python remappings to string tuples (same logic as Node)
    ///
    /// Handles:
    /// - Tuple pairs: `[('old_topic', 'new_topic')]` -> `[("old_topic", "new_topic")]`
    /// - LaunchConfiguration objects in tuples: `[('topic', LaunchConfiguration('name'))]`
    /// - Substitutions that need string conversion
    fn parse_remappings(&self, py: Python) -> PyResult<Vec<(String, String)>> {
        let mut parsed_remaps = Vec::new();

        for remap_obj in &self.remappings {
            let remap_any = remap_obj.bind(py);

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
}
