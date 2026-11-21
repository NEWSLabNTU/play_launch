//! Mock `launch_ros.actions.LifecycleNode` and `LifecycleTransition`.

use super::node::{Node, NodeSpec};
use pyo3::{prelude::*, types::PyDict};

/// `LifecycleNode(...)`: a `Node` as far as what it starts is concerned.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct LifecycleNode {
    spec: NodeSpec,
    /// `on_exit=Shutdown()`, as on `Node` (#0064): a `LifecycleNode` is an
    /// `ExecuteProcess`, so stock launch honours it the same way.
    on_exit_shutdown: Option<bool>,
    /// `autostart=` (Jazzy; phase 86): a bool or a substitution, resolved
    /// when the action executes, as `LifecycleNode.execute` does. `None`
    /// when not passed — Jazzy's default `False`, and Humble has no such
    /// argument at all.
    autostart: Option<Py<PyAny>>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
    /// The `(namespace, name)` of the capture this action made when it
    /// executed, so a later `ChangeState` event naming it
    /// (`matches_action(node)`) can find that capture (#0108).
    pub(crate) captured: Option<(Option<String>, Option<String>)>,
    /// Whether the launch emitted this node's configure and its activate
    /// transition (`ChangeState`, #0108).
    transitions: (bool, bool),
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
        autostart=None,
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
        autostart: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            on_exit_shutdown: super::node::on_exit_kwarg_shuts_down(_kwargs),
            autostart,
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
                respawn: super::node::RespawnArgs::from_kwargs(_kwargs),
                process: super::node::ProcessArgs::from_kwargs(_kwargs),
            },
            condition,
            captured: None,
            transitions: (false, false),
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
        let autostart = Self::resolve_autostart(py, this.borrow().autostart.as_ref())?;
        let on_exit_shutdown = this.borrow().on_exit_shutdown;
        let n = spec.capture_with(py, on_exit_shutdown, Some(autostart))?;
        this.borrow_mut().captured = Some((n.namespace, n.name));
        Ok(())
    }

    /// `perform_typed_substitution(context, autostart, bool)`: a Python bool
    /// as is, anything else performed to a string and coerced the way
    /// `launch` coerces to `bool`. A value `launch` would refuse raises here
    /// too, rather than reading as "no autostart".
    fn resolve_autostart(py: Python, autostart: Option<&Py<PyAny>>) -> PyResult<Option<bool>> {
        let Some(obj) = autostart else {
            return Ok(None);
        };
        if let Ok(b) = obj.bind(py).cast::<pyo3::types::PyBool>() {
            return Ok(Some(b.is_true()));
        }
        let text =
            crate::api::utils::resolve_tokens(&crate::api::utils::pyobject_to_string(py, obj)?);
        play_launch_parser::actions::node::coerce_launch_bool(&text)
            .map(Some)
            .ok_or_else(|| {
                pyo3::exceptions::PyValueError::new_err(format!(
                    "LifecycleNode(autostart={text:?}): cannot convert to bool"
                ))
            })
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

impl LifecycleNode {
    /// The `on_exit=` function, if given one (#0090).
    pub(crate) fn on_exit_fn(&self, py: Python) -> Option<Py<PyAny>> {
        self.spec.process.on_exit_fn(py)
    }
}

/// `lifecycle_msgs/msg/Transition` ids.
const TRANSITION_CONFIGURE: i64 = 1;
const TRANSITION_ACTIVATE: i64 = 3;

/// Execute `EmitEvent(event=ChangeState(lifecycle_node_matcher=…,
/// transition_id=…))` (#0108).
///
/// slam_toolbox's launch files (Jazzy) start their node as a `LifecycleNode`
/// and bring it up themselves: a configure event right after it, and an
/// `OnStateTransition` handler that emits activate once it is inactive. The
/// walk runs the handler's entities at startup, so both events reach here.
/// Once a node has both, its capture is `lifecycle_autostart: active`, which
/// is what play_launch drives after spawn; a configure alone is
/// `lifecycle_configure_only`, which it drives to inactive. A matcher this walk cannot read
/// (anything but `matches_action(node)` of a node it captured) is reported,
/// and the node is left as launched.
pub(crate) fn change_state(_py: Python, event: &Bound<'_, PyAny>) -> PyResult<()> {
    let transition: Option<i64> = event.getattr("transition_id")?.extract().ok();
    let matcher = event.getattr("lifecycle_node_matcher")?;
    let target = matcher
        .getattr("action")
        .ok()
        .and_then(|a| a.cast_into::<LifecycleNode>().ok());
    let Some(target) = target else {
        crate::api::utils::warn_user(
            "A ChangeState event names its lifecycle node with a matcher play_launch cannot              read (only matches_action(node) is understood), so the transition it asks for is              not driven.",
        );
        return Ok(());
    };
    let Some(key) = target.borrow().captured.clone() else {
        crate::api::utils::warn_user(
            "A ChangeState event names a LifecycleNode that has not been started, so the              transition it asks for is not driven.",
        );
        return Ok(());
    };
    let (configured, activated) = {
        let mut t = target.borrow_mut();
        match transition {
            Some(TRANSITION_CONFIGURE) => t.transitions.0 = true,
            Some(TRANSITION_ACTIVATE) => t.transitions.1 = true,
            _ => {}
        }
        t.transitions
    };
    if !configured {
        // An activate alone does nothing in launch either: the node is
        // unconfigured and refuses it.
        return Ok(());
    }
    let mut found = false;
    play_launch_parser::bridge::update_captured_nodes(|nodes| {
        if let Some(n) = nodes
            .iter_mut()
            .rev()
            .find(|n| n.lifecycle == Some(true) && n.namespace == key.0 && n.name == key.1)
        {
            if activated {
                n.lifecycle_autostart = Some(true);
                n.lifecycle_configure_only = None;
            } else if n.lifecycle_autostart != Some(true) {
                n.lifecycle_configure_only = Some(true);
            }
            found = true;
        }
    });
    if !found {
        crate::api::utils::warn_user(&format!(
            "The lifecycle node {:?} is configured and activated by the launch file, but              its record was already handed on (an include ran in between), so play_launch              does not drive those transitions.",
            key.1.as_deref().unwrap_or("<unnamed>")
        ));
    }
    Ok(())
}
