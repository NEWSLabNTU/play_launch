//! Mock `launch_ros.actions.LifecycleNode` and `LifecycleTransition`.

use super::node::{Node, NodeSpec};
use pyo3::{prelude::*, types::PyDict};

/// `LifecycleNode(...)`: a `Node` as far as what it starts is concerned.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct LifecycleNode {
    spec: NodeSpec,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl LifecycleNode {
    #[new]
    #[pyo3(signature = (
        *,
        executable=None,
        package=None,
        name=None,
        namespace=None,
        parameters=None,
        remappings=None,
        ros_arguments=None,
        arguments=None,
        env=None,
        additional_env=None,
        condition=None,
        **_kwargs
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        executable: Option<Py<PyAny>>,
        package: Option<Py<PyAny>>,
        name: Option<Py<PyAny>>,
        namespace: Option<Py<PyAny>>,
        parameters: Option<Py<PyAny>>,
        remappings: Option<Py<PyAny>>,
        ros_arguments: Option<Py<PyAny>>,
        arguments: Option<Py<PyAny>>,
        env: Option<Py<PyAny>>,
        additional_env: Option<Py<PyAny>>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            spec: NodeSpec {
                package,
                executable,
                name,
                namespace,
                parameters,
                remappings,
                arguments,
                ros_arguments,
                env,
                additional_env,
            },
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "LifecycleNode(...)".to_string()
    }
}

impl LifecycleNode {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let spec = this.borrow().spec.clone();
        let label = Node::label(py, &spec);
        if !crate::api::visit::claim_execution(this.as_any(), &label) {
            return Ok(());
        }
        spec.capture(py).map(|_| ())
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
