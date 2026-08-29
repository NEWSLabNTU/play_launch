//! Mock `launch_ros` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

mod composable_node;
mod container;
mod helpers;
mod lifecycle_node;
mod load_composable;
mod node;
mod push_namespace;

pub use composable_node::ComposableNode;
pub use container::ComposableNodeContainer;
pub use lifecycle_node::{LifecycleNode, LifecycleTransition};
pub use load_composable::LoadComposableNodes;
pub use node::Node;
pub use push_namespace::{PopRosNamespace, PushRosNamespace};

use pyo3::{prelude::*, types::PyDict};

// ============================================================================
// Small action structs kept in mod.rs
// ============================================================================

/// Mock SetParameter action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import SetParameter
/// set_param = SetParameter(name='parameter_name', value='value')
/// ```
///
/// Sets a global ROS parameter that applies to all nodes
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct SetParameter {
    #[allow(dead_code)] // Stored for API compatibility
    name: String,
    #[allow(dead_code)] // Stored for API compatibility
    value: Py<PyAny>,
}

#[pymethods]
impl SetParameter {
    #[new]
    #[pyo3(signature = (*, name, value, **_kwargs))]
    fn new(
        py: Python,
        name: String,
        value: Py<PyAny>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<Self> {
        // Try to resolve the value if it's a substitution (like LaunchConfiguration)
        // Use perform() with context if available, otherwise fall back to string conversion
        let value_str = Self::resolve_value(py, &value)?;

        log::debug!("Python Launch SetParameter: {}={}", name, value_str);

        // Capture this as a global parameter in LaunchContext via thread-local
        {
            use play_launch_parser::bridge::with_launch_context;

            // Match Python boolean case: "False"/"True" not "false"/"true"
            let normalized_value = match value_str.as_str() {
                "false" => "False".to_string(),
                "true" => "True".to_string(),
                _ => value_str.clone(),
            };

            with_launch_context(|ctx| {
                ctx.set_global_parameter(name.clone(), normalized_value.clone());
            });
            log::debug!(
                "Captured SetParameter '{}' = '{}' to LaunchContext",
                name,
                normalized_value
            );
        }

        Ok(Self { name, value })
    }

    fn __repr__(&self) -> String {
        format!("SetParameter(name='{}')", self.name)
    }
}

impl SetParameter {
    /// Resolve a value Py<PyAny>, attempting to resolve substitutions if possible
    fn resolve_value(py: Python, obj: &Py<PyAny>) -> PyResult<String> {
        use crate::api::utils::create_launch_context;

        let obj_ref = obj.bind(py);

        // If it has a perform() method, try to resolve it with a real context
        if obj_ref.hasattr("perform")?
            && let Ok(context) = create_launch_context(py)
            && let Ok(result) = obj_ref.call_method1("perform", (context,))
            && let Ok(resolved) = result.extract::<String>()
        {
            log::debug!("Resolved SetParameter value via perform(): '{}'", resolved);
            return Ok(resolved);
        }

        // Fallback to regular string conversion
        crate::api::utils::pyobject_to_string(py, obj)
    }
}

/// Mock SetParametersFromFile action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import SetParametersFromFile
///
/// SetParametersFromFile(
///     PathJoinSubstitution([FindPackageShare('my_pkg'), 'config', 'params.yaml']),
///     node_name='my_node'  # Optional - if not specified, applies to all nodes
/// )
/// ```
///
/// Loads parameters from a YAML file and applies them to nodes.
/// For static analysis, we just capture the intent without loading the file.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct SetParametersFromFile {
    #[allow(dead_code)] // Stored for API compatibility
    filename: Py<PyAny>,
    #[allow(dead_code)] // Stored for API compatibility
    node_name: Option<String>,
}

#[pymethods]
impl SetParametersFromFile {
    #[new]
    #[pyo3(signature = (filename, *, node_name=None, **_kwargs))]
    fn new(
        filename: Py<PyAny>,
        node_name: Option<String>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        log::debug!(
            "Python Launch SetParametersFromFile: node_name={:?}",
            node_name
        );
        // For static analysis, we just capture the action
        // We don't actually load the YAML file since we're parsing structure, not executing
        Self {
            filename,
            node_name,
        }
    }

    fn __repr__(&self) -> String {
        if let Some(ref name) = self.node_name {
            format!("SetParametersFromFile(node_name='{}')", name)
        } else {
            "SetParametersFromFile(for all nodes)".to_string()
        }
    }
}

/// Mock RosTimer action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import RosTimer
/// RosTimer(period=10.0, actions=[action1, action2])
/// ```
///
/// Timer that uses ROS time instead of wall clock time.
/// Requires use_sim_time to be set for simulation time support.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct RosTimer {
    #[allow(dead_code)] // Keep for future use
    period: Py<PyAny>,
    #[allow(dead_code)] // Keep for future use
    actions: Option<Py<PyAny>>,
}

#[pymethods]
impl RosTimer {
    #[new]
    #[pyo3(signature = (*, period, actions=None, **_kwargs))]
    fn new(
        py: Python,
        period: Py<PyAny>,
        actions: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> PyResult<Self> {
        // Convert period to string for logging
        let period_str = if let Ok(f) = period.extract::<f64>(py) {
            f.to_string()
        } else if let Ok(i) = period.extract::<i64>(py) {
            i.to_string()
        } else {
            period.to_string()
        };

        log::debug!("Python Launch RosTimer: period={}", period_str);

        Ok(Self { period, actions })
    }

    fn __repr__(&self) -> String {
        "RosTimer(...)".to_string()
    }
}

/// Mock SetUseSimTime action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import SetUseSimTime
/// SetUseSimTime(True)
/// ```
///
/// Sets the 'use_sim_time' parameter in the current context.
/// This enables simulation time for ROS nodes.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct SetUseSimTime {
    value: bool,
}

#[pymethods]
impl SetUseSimTime {
    #[new]
    fn new(value: bool) -> Self {
        log::debug!("Python Launch SetUseSimTime: {}", value);
        Self { value }
    }

    fn __repr__(&self) -> String {
        format!("SetUseSimTime({})", self.value)
    }
}

/// Mock SetRemap action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import SetRemap
/// SetRemap(src='old_topic', dst='new_topic')
/// ```
///
/// Sets a remapping rule in the current context for nodes in the same scope.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct SetRemap {
    src: String,
    dst: String,
}

#[pymethods]
impl SetRemap {
    #[new]
    #[pyo3(signature = (src, dst, **_kwargs))]
    fn new(src: String, dst: String, _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>) -> Self {
        log::debug!("Python Launch SetRemap: {} -> {}", src, dst);
        Self { src, dst }
    }

    fn __repr__(&self) -> String {
        format!("SetRemap(src='{}', dst='{}')", self.src, self.dst)
    }
}

/// Mock SetROSLogDir action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import SetROSLogDir
/// SetROSLogDir('/path/to/log/dir')
/// ```
///
/// Sets the ROS log directory for nodes launched in the same scope.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct SetROSLogDir {
    #[allow(dead_code)] // Keep for future use
    log_dir: Py<PyAny>,
}

#[pymethods]
impl SetROSLogDir {
    #[new]
    fn new(log_dir: Py<PyAny>) -> Self {
        log::debug!("Python Launch SetROSLogDir: log directory provided");
        Self { log_dir }
    }

    fn __repr__(&self) -> String {
        "SetROSLogDir(...)".to_string()
    }
}
