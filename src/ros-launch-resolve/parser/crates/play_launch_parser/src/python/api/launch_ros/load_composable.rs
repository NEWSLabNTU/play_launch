//! Mock LoadComposableNodes action for launch_ros.actions

use super::{
    helpers::{is_yaml_file, load_yaml_params},
    node::Node,
};
use pyo3::prelude::*;

/// Mock LoadComposableNodes action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import LoadComposableNodes
/// from launch_ros.descriptions import ComposableNode
///
/// LoadComposableNodes(
///     target_container='container_name',
///     composable_node_descriptions=[
///         ComposableNode(
///             package='my_package',
///             plugin='my_package::MyPlugin',
///             name='my_node',
///         ),
///     ],
/// )
/// ```
///
/// Loads composable nodes into an existing container
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct LoadComposableNodes {
    #[allow(dead_code)] // Keep for API compatibility
    target_container: Py<PyAny>,
    composable_node_descriptions: Vec<Py<PyAny>>,
}

#[pymethods]
impl LoadComposableNodes {
    #[new]
    #[pyo3(signature = (*, target_container, composable_node_descriptions, condition=None, **_kwargs))]
    fn new(
        py: Python,
        target_container: Py<PyAny>,
        composable_node_descriptions: Vec<Py<PyAny>>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> PyResult<Self> {
        // Extract target container name for logging
        let target_str = Self::pyobject_to_string(target_container.bind(py))
            .unwrap_or_else(|_| "<unknown>".to_string());

        // Evaluate condition (if present) and only capture if true
        let should_capture = if let Some(cond_obj) = &condition {
            let result = Self::evaluate_condition(py, cond_obj).unwrap_or(true);
            log::debug!(
                "LoadComposableNodes target='{}': condition evaluated to {}",
                target_str,
                result
            );
            result
        } else {
            log::debug!(
                "LoadComposableNodes target='{}': no condition, capturing",
                target_str
            );
            true // No condition means always capture
        };

        if should_capture {
            // Log node names for debugging
            let node_names: Vec<String> = composable_node_descriptions
                .iter()
                .filter_map(|obj| {
                    obj.bind(py)
                        .getattr("_ComposableNode__node_name")
                        .ok()
                        .and_then(|attr| attr.extract::<String>().ok())
                })
                .collect();
            log::debug!(
                "LoadComposableNodes: target='{}', {} nodes: {:?}",
                target_str,
                composable_node_descriptions.len(),
                node_names
            );

            // Capture the composable nodes
            Self::capture_composable_nodes(py, &target_container, &composable_node_descriptions)?;

            log::debug!(
                "Python Launch LoadComposableNodes created with {} nodes",
                composable_node_descriptions.len()
            );
        } else {
            log::debug!(
                "Skipping LoadComposableNodes capture due to condition ({} nodes)",
                composable_node_descriptions.len()
            );
        }

        Ok(Self {
            target_container,
            composable_node_descriptions,
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "LoadComposableNodes({} nodes)",
            self.composable_node_descriptions.len()
        )
    }
}

impl LoadComposableNodes {
    /// Capture composable nodes from the descriptions list
    fn capture_composable_nodes(
        py: Python,
        target_container: &Py<PyAny>,
        descriptions: &[Py<PyAny>],
    ) -> PyResult<()> {
        use crate::{
            captures::LoadNodeCapture,
            python::bridge::{capture_load_node, get_current_ros_namespace},
        };

        // Extract target container name and namespace
        let (container_name, _container_namespace) =
            Self::extract_target_container(py, target_container)?;

        // Get the ROS context namespace (from push-ros-namespace stack).
        // ROS2's get_composable_node_load_request() uses the launch context namespace,
        // not the container's namespace, for composable nodes.
        let ros_namespace = get_current_ros_namespace();

        for desc_obj in descriptions {
            // Try to get the ComposableNode attributes
            let desc = desc_obj.bind(py);

            // Extract package (required)
            let package = if let Ok(pkg) = desc.getattr("package") {
                Self::pyobject_to_string(&pkg)?
            } else {
                continue; // Skip if no package
            };

            // Extract plugin (required)
            let plugin = if let Ok(plg) = desc.getattr("plugin") {
                Self::pyobject_to_string(&plg)?
            } else {
                continue; // Skip if no plugin
            };

            // Extract name (required)
            let node_name = if let Ok(n) = desc.getattr("name") {
                Self::pyobject_to_string(&n)?
            } else {
                continue; // Skip if no name
            };

            // Resolve namespace using ROS context namespace (matching ROS2 behavior).
            // If the node has an explicit namespace, resolve relative to ROS context.
            // If no namespace, use the ROS context namespace.
            let explicit_ns = desc.getattr("namespace").ok().and_then(|ns| {
                if ns.is_none() {
                    None
                } else {
                    Self::pyobject_to_string(&ns).ok()
                }
            });

            let namespace = if let Some(ns) = explicit_ns {
                if ns.is_empty() {
                    ros_namespace.clone()
                } else if ns.starts_with('/') {
                    ns // Absolute — use as-is
                } else {
                    // Relative — combine with ROS context namespace
                    if ros_namespace == "/" {
                        format!("/{}", ns)
                    } else {
                        format!("{}/{}", ros_namespace, ns)
                    }
                }
            } else {
                // No explicit namespace — use ROS context namespace
                ros_namespace.clone()
            };

            // Extract parameters (optional)
            let parameters = if let Ok(params_obj) = desc.getattr("parameters") {
                Self::extract_parameters(&params_obj)?
            } else {
                Vec::new()
            };

            // Extract remappings (optional)
            let remappings = if let Ok(remaps_obj) = desc.getattr("remappings") {
                Self::extract_remappings(&remaps_obj)?
            } else {
                Vec::new()
            };

            log::debug!(
                "capture_composable_nodes: node='{}', {} parameters",
                node_name,
                parameters.len()
            );

            let capture = LoadNodeCapture {
                package,
                plugin,
                target_container_name: container_name.clone(),
                node_name: node_name.clone(),
                namespace,
                parameters,
                remappings,
                scope_id: None,
            };

            log::debug!(
                "Captured Python composable node from LoadComposableNodes: {} / {} (container: {})",
                capture.package,
                capture.plugin,
                capture.target_container_name
            );

            capture_load_node(capture);
        }

        Ok(())
    }

    /// Evaluate a condition object (same logic as Node)
    fn evaluate_condition(py: Python, condition: &Py<PyAny>) -> PyResult<bool> {
        let cond_ref = condition.bind(py);

        // Try calling evaluate() method on the condition object
        if let Ok(result) = cond_ref.call_method0("evaluate")
            && let Ok(bool_val) = result.extract::<bool>()
        {
            log::debug!("LoadComposableNodes condition evaluated to: {}", bool_val);
            return Ok(bool_val);
        }

        // Fallback: treat as truthy if we can't evaluate
        log::warn!("Failed to evaluate LoadComposableNodes condition, defaulting to true");
        Ok(true)
    }

    /// Extract target container name and namespace from a Py<PyAny>
    ///
    /// The target_container can be:
    /// - A ComposableNodeContainer instance
    /// - A string with the container name
    /// - A LaunchConfiguration or other substitution
    fn extract_target_container(
        py: Python,
        target: &Py<PyAny>,
    ) -> PyResult<(String, Option<String>)> {
        let target_ref = target.bind(py);

        // Try to extract as ComposableNodeContainer instance
        if let Ok(name_attr) = target_ref.getattr("name") {
            let name = Self::pyobject_to_string(&name_attr)?;
            let container_namespace = target_ref
                .getattr("namespace")
                .ok()
                .and_then(|ns| Self::pyobject_to_string(&ns).ok());

            // Build full target container name: ros_namespace + container_namespace + name
            // This must match how capture_container() builds the full name, which
            // combines the current ROS namespace (from push-ros-namespace stack)
            // with the container's own namespace.
            use crate::python::bridge::get_current_ros_namespace;
            let ros_namespace = get_current_ros_namespace();

            // Normalize container's namespace (same logic as capture_container)
            let container_ns = container_namespace
                .as_ref()
                .map(|ns| {
                    if ns.is_empty() {
                        String::new()
                    } else if ns.starts_with('/') {
                        ns.clone()
                    } else {
                        format!("/{}", ns)
                    }
                })
                .unwrap_or_default();

            // Combine ROS namespace with container namespace (same as capture_container)
            let full_namespace = if ros_namespace == "/" {
                if container_ns.is_empty() {
                    "/".to_string()
                } else {
                    container_ns
                }
            } else if container_ns.is_empty() {
                ros_namespace.clone()
            } else {
                format!("{}{}", ros_namespace, container_ns)
            };

            let full_name = Self::normalize_namespace_path(&full_namespace, &name);

            log::debug!(
                "Extracted target container from instance: {} (ros_ns='{}', container_ns='{}')",
                full_name,
                ros_namespace,
                container_namespace.as_deref().unwrap_or("")
            );
            return Ok((full_name, Some(full_namespace)));
        }

        // Try as string, LaunchConfiguration, or other substitution
        // Use pyobject_to_string to get the substitution string for output
        let name = Self::pyobject_to_string(target_ref)?;

        log::debug!(
            "Extracted target container from string/substitution: '{}'",
            name
        );

        // For namespace extraction, we need the resolved value (not the substitution string)
        // If this is a substitution, try to resolve it using perform() with real context
        let resolved_value = if target_ref.hasattr("perform")? {
            // Import create_launch_context helper
            use crate::python::api::utils::create_launch_context;

            // Try to resolve using real context
            if let Ok(context) = create_launch_context(py) {
                if let Ok(result) = target_ref.call_method1("perform", (context,)) {
                    result.extract::<String>().ok()
                } else {
                    None
                }
            } else {
                None
            }
        } else {
            None
        };

        // Use resolved value for namespace extraction if available, otherwise use name as-is
        let value_for_namespace = resolved_value.as_ref().unwrap_or(&name);

        // Extract the namespace from the path for composable nodes to inherit
        let namespace = if value_for_namespace.contains('/') {
            // Extract namespace from the path (everything before the last /)
            if let Some(last_slash_idx) = value_for_namespace.rfind('/') {
                if last_slash_idx == 0 {
                    // Path like "/container_name" -> namespace is "/"
                    Some("/".to_string())
                } else {
                    // Path like "/ns/container_name" -> namespace is "/ns"
                    Some(value_for_namespace[..last_slash_idx].to_string())
                }
            } else {
                None
            }
        } else {
            // Simple name without slashes -> no namespace to extract
            None
        };

        // Use the resolved value if available, otherwise fall back to the unresolved name
        let final_name = resolved_value.unwrap_or(name.clone());

        log::debug!(
            "Target container: '{}' (original: '{}'), extracted namespace: {:?}",
            final_name,
            name,
            namespace
        );

        Ok((final_name, namespace))
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

    /// Extract parameters from Python object
    fn extract_parameters(params_obj: &Bound<'_, PyAny>) -> PyResult<Vec<(String, String)>> {
        let mut parsed_params = Vec::new();

        if let Ok(params_list) = params_obj.cast::<pyo3::types::PyList>() {
            log::trace!("extract_parameters: {} items", params_list.len());

            for (idx, param_item) in params_list.iter().enumerate() {
                if let Ok(param_dict) = param_item.cast::<pyo3::types::PyDict>() {
                    log::trace!(
                        "extract_parameters: item {} is dict with {} keys",
                        idx,
                        param_dict.len()
                    );

                    // Check if this dict represents a parameter file
                    if param_dict.len() == 1
                        && let Some((key, value)) = param_dict.iter().next()
                    {
                        let key_str = key.extract::<String>().unwrap_or_default();

                        // Check if the value is a file path
                        if let Ok(path_str) = value.extract::<String>()
                            && is_yaml_file(&path_str)
                        {
                            match load_yaml_params(&path_str) {
                                Ok(yaml_params) => {
                                    log::trace!(
                                        "Loaded {} parameters from YAML {}",
                                        yaml_params.len(),
                                        path_str
                                    );
                                    parsed_params.extend(yaml_params);
                                    continue; // Skip normal dict processing
                                }
                                Err(e) => {
                                    log::warn!("Failed to load YAML file {}: {}", path_str, e);
                                }
                            }
                        }
                        let _ = key_str; // used for debugging if needed
                    }

                    // Normal dict processing (recursively flatten nested dicts)
                    Node::parse_dict_params(param_dict, "", &mut parsed_params)?;
                } else {
                    let type_name = param_item
                        .get_type()
                        .name()
                        .map(|n| n.to_string())
                        .unwrap_or_default();
                    log::trace!("extract_parameters: item {} type={}", idx, type_name);

                    // Check if it's a ParameterFile object
                    if type_name.contains("ParameterFile")
                        || type_name.contains("parameter_descriptions")
                    {
                        // Call __str__() to get the file path
                        if let Ok(str_method) = param_item.call_method0("__str__")
                            && let Ok(path_str) = str_method.extract::<String>()
                        {
                            // Check if it's a YAML parameter file
                            if is_yaml_file(&path_str) {
                                // Load and expand YAML parameter file
                                match load_yaml_params(&path_str) {
                                    Ok(yaml_params) => {
                                        log::trace!(
                                            "Loaded {} parameters from ParameterFile {}",
                                            yaml_params.len(),
                                            path_str
                                        );
                                        parsed_params.extend(yaml_params);
                                    }
                                    Err(e) => {
                                        log::warn!(
                                            "Failed to load parameter file {}: {}",
                                            path_str,
                                            e
                                        );
                                        // Fallback: store as __param_file for backward compatibility
                                        parsed_params.push(("__param_file".to_string(), path_str));
                                    }
                                }
                            } else {
                                // Non-YAML file path - store as reference
                                parsed_params.push(("__param_file".to_string(), path_str));
                            }
                        }
                    }
                    // Try extracting as string anyway
                    else if let Ok(path_str) = param_item.extract::<String>() {
                        // Check if it's a YAML parameter file
                        if is_yaml_file(&path_str) {
                            // Load and expand YAML parameter file
                            match load_yaml_params(&path_str) {
                                Ok(yaml_params) => {
                                    log::trace!(
                                        "Loaded {} parameters from {}",
                                        yaml_params.len(),
                                        path_str
                                    );
                                    parsed_params.extend(yaml_params);
                                }
                                Err(e) => {
                                    log::warn!("Failed to load parameter file {}: {}", path_str, e);
                                    // Fallback: store as __param_file for backward compatibility
                                    parsed_params.push(("__param_file".to_string(), path_str));
                                }
                            }
                        } else {
                            // Non-YAML file path - store as reference
                            parsed_params.push(("__param_file".to_string(), path_str));
                        }
                    }
                }
            }
        }

        Ok(parsed_params)
    }

    /// Extract remappings from Python object
    fn extract_remappings(remaps_obj: &Bound<'_, PyAny>) -> PyResult<Vec<(String, String)>> {
        let mut parsed_remaps = Vec::new();

        if let Ok(remaps_list) = remaps_obj.cast::<pyo3::types::PyList>() {
            for remap_item in remaps_list.iter() {
                if let Ok(remap_tuple) = remap_item.cast::<pyo3::types::PyTuple>()
                    && remap_tuple.len() == 2
                {
                    // Use pyobject_to_string to handle LaunchConfiguration objects
                    let from = Self::pyobject_to_string(&remap_tuple.get_item(0)?)?;
                    let to = Self::pyobject_to_string(&remap_tuple.get_item(1)?)?;
                    parsed_remaps.push((from, to));
                }
            }
        }

        Ok(parsed_remaps)
    }

    /// Convert Py<PyAny> to string (handles substitutions)
    fn pyobject_to_string(obj: &Bound<'_, PyAny>) -> PyResult<String> {
        let py = obj.py();
        let py_obj: Py<PyAny> = obj.clone().unbind();
        crate::python::api::utils::pyobject_to_string(py, &py_obj)
    }
}
