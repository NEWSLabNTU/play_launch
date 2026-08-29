//! GroupAction action

use pyo3::prelude::*;

/// Mock GroupAction
///
/// Python equivalent:
/// ```python
/// from launch.actions import GroupAction
/// group = GroupAction(
///     actions=[action1, action2],
///     scoped=True,
///     forwarding=True
/// )
/// ```
///
/// Groups actions together with optional scoping
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct GroupAction {
    #[pyo3(get)] // Make actions directly accessible as an attribute (like LaunchDescription)
    pub actions: Vec<Py<PyAny>>,
    #[allow(dead_code)] // Keep for API compatibility
    scoped: bool,
    #[allow(dead_code)] // Keep for API compatibility
    forwarding: bool,
}

#[pymethods]
impl GroupAction {
    #[new]
    #[pyo3(signature = (actions, *, scoped=true, forwarding=true, **_kwargs))]
    fn new(
        py: Python,
        actions: Vec<Py<PyAny>>,
        scoped: Option<bool>,
        forwarding: Option<bool>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> PyResult<Self> {
        // CRITICAL FIX: GroupActions create a scoped namespace context.
        // When the actions list is passed in, any PushRosNamespace actions have already
        // been constructed and have pushed onto the LaunchContext's namespace stack. We need
        // to pop them after the GroupAction is done being used (simulating scope exit).
        //
        // Since we're in dump mode (not actually executing the launch system), we simulate
        // this by counting PushRosNamespace actions at the start and popping them immediately
        // after GroupAction construction completes.
        //
        // This ensures that namespace pushes from one GroupAction don't leak into the next
        // when multiple GroupActions are created in sequence (e.g., in a list comprehension).

        // Count PushRosNamespace actions that ACTUALLY pushed a namespace.
        // PushRosNamespace("") or PushRosNamespace("/") are no-ops in push_namespace(),
        // so we must only pop for those that actually changed the namespace stack depth.
        // Each PushRosNamespace stores a `did_push` field indicating whether its push was effective.
        let mut actual_push_count = 0;
        for action in &actions {
            if let Ok(type_name) = action
                .getattr(py, "__class__")
                .and_then(|cls| cls.getattr(py, "__name__"))
                .and_then(|name| name.extract::<String>(py))
            {
                if type_name == "PushRosNamespace" {
                    // Check if this PushRosNamespace actually pushed
                    if let Ok(did_push) = action.getattr(py, "did_push") {
                        if did_push.extract::<bool>(py).unwrap_or(true) {
                            actual_push_count += 1;
                        }
                    } else {
                        // Fallback: assume it pushed if we can't check
                        actual_push_count += 1;
                    }
                } else if type_name != "IncludeLaunchDescription" {
                    break;
                }
            }
        }

        // Pop only the namespaces that were actually pushed
        if actual_push_count > 0 {
            use play_launch_parser::bridge::pop_ros_namespace;
            for _ in 0..actual_push_count {
                pop_ros_namespace();
            }
            log::debug!(
                "GroupAction popped {} namespaces for scope cleanup",
                actual_push_count
            );
        }

        Ok(Self {
            actions,
            scoped: scoped.unwrap_or(true),
            forwarding: forwarding.unwrap_or(true),
        })
    }

    fn __repr__(&self) -> String {
        format!("GroupAction({} actions)", self.actions.len())
    }
}
