//! Mock ComposableNodeContainer class for launch_ros.actions

use super::composable_node::ComposableNode;
use crate::{captures::ContainerCapture, python::bridge::capture_container};
use pyo3::{prelude::*, types::PyDict};

/// Mock ComposableNodeContainer class
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import ComposableNodeContainer
/// from launch_ros.descriptions import ComposableNode
///
/// container = ComposableNodeContainer(
///     name='my_container',
///     namespace='/my_namespace',
///     package='rclcpp_components',
///     executable='component_container',
///     composable_node_descriptions=[
///         ComposableNode(package='pkg', plugin='Plugin', name='node'),
///     ]
/// )
/// ```
#[pyclass(module = "launch_ros.actions")]
#[derive(Clone)]
pub struct ComposableNodeContainer {
    name: String,
    namespace: Option<String>,
    #[allow(dead_code)] // Keep for API compatibility but not used in container record
    package: String,
    #[allow(dead_code)] // Keep for API compatibility but not used in container record
    executable: String,
    composable_nodes: Vec<Py<ComposableNode>>,
}

#[pymethods]
impl ComposableNodeContainer {
    #[new]
    #[pyo3(signature = (
        *,
        name,
        namespace=None,
        package,
        executable,
        composable_node_descriptions=None,
        condition=None,
        **_kwargs
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        py: Python,
        name: PyObject,
        namespace: Option<PyObject>,
        package: PyObject,
        executable: PyObject,
        composable_node_descriptions: Option<Vec<Py<ComposableNode>>>,
        condition: Option<PyObject>,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<Self> {
        let composable_nodes = composable_node_descriptions.unwrap_or_default();

        // Convert PyObject parameters to strings (may be substitutions)
        // For name and namespace, resolve substitutions if present
        log::debug!("ComposableNodeContainer::new: creating container");

        // Resolve name substitution
        let name_before = Self::pyobject_to_string(py, &name)?;
        let name_str = Self::resolve_pyobject_substitution(py, &name)?;
        if name_before != name_str {
            log::debug!(
                "ComposableNodeContainer name resolved: '{}' -> '{}'",
                name_before,
                name_str
            );
        }

        // Debug: Log if name is empty
        if name_str.is_empty() {
            use crate::python::bridge::with_launch_context;
            let config_keys =
                with_launch_context(|ctx| ctx.configurations().keys().cloned().collect::<Vec<_>>());
            log::warn!(
                "ComposableNodeContainer has empty name! Available configs: {:?}",
                config_keys
            );
        }

        // Resolve namespace substitution
        let namespace_str = namespace
            .map(|ns| {
                log::debug!("ComposableNodeContainer::new: processing namespace parameter");
                Self::resolve_pyobject_substitution(py, &ns)
            })
            .transpose()?;
        let package_str = Self::pyobject_to_string(py, &package)?;
        let executable_str = Self::pyobject_to_string(py, &executable)?;

        log::debug!(
            "ComposableNodeContainer::new: name='{}', namespace={:?}, package='{}', executable='{}'",
            name_str,
            namespace_str,
            package_str,
            executable_str
        );

        let container = Self {
            name: name_str.clone(),
            namespace: namespace_str.clone(),
            package: package_str,
            executable: executable_str,
            composable_nodes: composable_nodes.clone(),
        };

        // Evaluate condition (if present) and only capture if true
        let should_capture = if let Some(cond_obj) = &condition {
            let result = Self::evaluate_condition(py, cond_obj).unwrap_or(true);
            log::debug!(
                "ComposableNodeContainer name='{}' namespace={:?}: condition evaluated to {}",
                name_str,
                namespace_str,
                result
            );
            result
        } else {
            log::debug!(
                "ComposableNodeContainer name='{}' namespace={:?}: no condition, capturing",
                name_str,
                namespace_str
            );
            true // No condition means always capture
        };

        if should_capture {
            // Capture the container
            Self::capture_container(&container);
            log::debug!("Captured ComposableNodeContainer '{}'", name_str);
        } else {
            log::debug!(
                "Skipping container capture due to condition: {} (namespace: {:?})",
                name_str,
                namespace_str
            );
        }

        Ok(container)
    }

    fn __repr__(&self) -> String {
        format!(
            "ComposableNodeContainer(name='{}', namespace='{}')",
            self.name,
            self.namespace.as_deref().unwrap_or("/")
        )
    }
}

impl ComposableNodeContainer {
    /// Resolve a PyObject substitution by calling perform() if available
    ///
    /// For substitutions like LaunchConfiguration, this calls perform() to get the resolved value.
    /// For lists, recursively resolves each element.
    /// Otherwise falls back to pyobject_to_string.
    fn resolve_pyobject_substitution(py: Python, obj: &PyObject) -> PyResult<String> {
        use crate::python::api::utils::create_launch_context;
        use pyo3::types::PyList;

        let obj_ref = obj.as_ref(py);

        // Handle lists specially - resolve each element
        if let Ok(list) = obj_ref.downcast::<PyList>() {
            let mut result = String::new();
            for item in list.iter() {
                let item_resolved = Self::resolve_pyobject_substitution(py, &item.into())?;
                result.push_str(&item_resolved);
            }
            log::debug!("Resolved list substitution: '{}'", result);
            return Ok(result);
        }

        // Try to resolve using perform() with real context
        if obj_ref.hasattr("perform")?
            && let Ok(context) = create_launch_context(py)
            && let Ok(result) = obj_ref.call_method1("perform", (context,))
            && let Ok(resolved) = result.extract::<String>()
        {
            log::debug!("Resolved substitution via perform(): '{}'", resolved);
            return Ok(resolved);
        }

        // Fallback to regular conversion
        Self::pyobject_to_string(py, obj)
    }

    fn capture_container(container: &ComposableNodeContainer) {
        use crate::python::bridge::get_current_ros_namespace;

        // Get current ROS namespace from the stack
        let ros_namespace = get_current_ros_namespace();
        log::trace!(
            "Container capture: ros_namespace from stack: '{}'",
            ros_namespace
        );
        log::trace!(
            "Container capture: container.namespace: {:?}",
            container.namespace
        );

        // Normalize container's namespace
        let container_ns = container
            .namespace
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

        log::trace!(
            "Container capture: normalized container_ns: '{}'",
            container_ns
        );

        // Combine ROS namespace with container namespace
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

        log::trace!("Container capture: full_namespace: '{}'", full_namespace);

        let capture = ContainerCapture {
            name: container.name.clone(),
            namespace: full_namespace.clone(),
            package: Some(container.package.clone()),
            executable: Some(container.executable.clone()),
            cmd: Vec::new(), // Will be generated in to_record()
        };

        log::debug!(
            "Captured Python container: {} (namespace: {}, pkg: {}, exec: {}, ros_namespace: {})",
            capture.name,
            capture.namespace,
            container.package,
            container.executable,
            ros_namespace
        );

        capture_container(capture);

        // Capture each composable node as a load_node entry
        // Use full_namespace (includes ros_namespace) not container.namespace
        let full_ns_opt = Some(full_namespace);
        Python::with_gil(|py| {
            for node_obj in &container.composable_nodes {
                let node = node_obj.borrow(py);
                node.capture_as_load_node(&container.name, &full_ns_opt);
            }
        });
    }

    /// Evaluate a condition object (same logic as Node)
    fn evaluate_condition(py: Python, condition: &PyObject) -> PyResult<bool> {
        let cond_ref = condition.as_ref(py);

        log::debug!(
            "Evaluating container condition, type: {:?}",
            cond_ref.get_type().name()
        );

        // Try calling evaluate() method on the condition object
        if let Ok(result) = cond_ref.call_method0("evaluate")
            && let Ok(bool_val) = result.extract::<bool>()
        {
            log::debug!("Container condition evaluated to: {}", bool_val);
            return Ok(bool_val);
        }

        // Fallback: treat as truthy if we can't evaluate
        log::warn!("Failed to evaluate container condition, defaulting to true");
        Ok(true)
    }

    /// Convert PyObject to string (handles both strings and substitutions)
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        crate::python::api::utils::pyobject_to_string(py, obj)
    }
}
