//! Mock PushRosNamespace and PopRosNamespace actions for launch_ros.actions

use pyo3::prelude::*;

/// Mock PushRosNamespace action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import PushRosNamespace
/// PushRosNamespace('my_namespace')
/// ```
///
/// Pushes a namespace onto the namespace stack
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct PushRosNamespace {
    #[allow(dead_code)] // Stored for API compatibility
    namespace: Py<PyAny>,
    /// Whether this PushRosNamespace actually pushed a namespace onto the stack.
    /// Empty or root ("/") namespaces are no-ops in push_namespace(), so
    /// GroupAction must know whether to pop for each PushRosNamespace action.
    #[pyo3(get)]
    did_push: bool,
}

#[pymethods]
impl PushRosNamespace {
    #[new]
    fn new(py: Python, namespace: Py<PyAny>) -> PyResult<Self> {
        // Convert namespace to string
        let namespace_str = if let Ok(s) = namespace.extract::<String>(py) {
            s
        } else if let Ok(str_result) = namespace.call_method0(py, "__str__") {
            str_result.extract::<String>(py)?
        } else {
            namespace.bind(py).str()?.to_string()
        };

        log::debug!("Python Launch PushRosNamespace: '{}'", namespace_str);

        // Push onto the namespace stack and track whether it actually pushed
        use crate::bridge::push_ros_namespace;
        let did_push = push_ros_namespace(namespace_str);

        Ok(Self {
            namespace,
            did_push,
        })
    }

    fn __repr__(&self) -> String {
        "PushRosNamespace(...)".to_string()
    }
}

/// Mock PopRosNamespace action
///
/// Python equivalent:
/// ```python
/// from launch_ros.actions import PopRosNamespace
/// PopRosNamespace()
/// ```
///
/// Pops a namespace from the namespace stack
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct PopRosNamespace {}

#[pymethods]
impl PopRosNamespace {
    #[new]
    fn new() -> Self {
        log::debug!("Python Launch PopRosNamespace");

        // Pop from the namespace stack
        use crate::bridge::pop_ros_namespace;
        pop_ros_namespace();

        Self {}
    }

    fn __repr__(&self) -> String {
        "PopRosNamespace()".to_string()
    }
}
