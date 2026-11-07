//! Mock `launch.event_handlers` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use pyo3::prelude::*;

/// Mock OnProcessStart event handler
///
/// Python equivalent:
/// ```python
/// from launch.event_handlers import OnProcessStart
///
/// OnProcessStart(
///     target_action=target_node,
///     on_start=[
///         LogInfo(msg='Node started'),
///     ]
/// )
/// ```
///
/// Triggers actions when a target process starts
#[pyclass(module = "launch.event_handlers", from_py_object)]
#[derive(Clone)]
pub struct OnProcessStart {
    #[allow(dead_code)] // Keep for API compatibility
    target_action: Option<Py<PyAny>>,
    on_start: Option<Py<PyAny>>,
}

#[pymethods]
impl OnProcessStart {
    #[new]
    #[pyo3(signature = (*, target_action=None, on_start=None, **_kwargs))]
    fn new(
        target_action: Option<Py<PyAny>>,
        on_start: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        Self {
            target_action,
            on_start,
        }
    }

    fn __repr__(&self) -> String {
        "OnProcessStart(...)".to_string()
    }
}

/// Mock OnProcessExit event handler
///
/// Python equivalent:
/// ```python
/// from launch.event_handlers import OnProcessExit
///
/// OnProcessExit(
///     target_action=target_node,
///     on_exit=[
///         LogInfo(msg='Node exited'),
///     ]
/// )
/// ```
///
/// Triggers actions when a target process exits
#[pyclass(module = "launch.event_handlers", from_py_object)]
#[derive(Clone)]
pub struct OnProcessExit {
    target_action: Option<Py<PyAny>>,
    /// One action, a list of them, or a callable -- as `launch` accepts.
    on_exit: Option<Py<PyAny>>,
}

#[pymethods]
impl OnProcessExit {
    #[new]
    #[pyo3(signature = (*, target_action=None, on_exit=None, **_kwargs))]
    fn new(
        target_action: Option<Py<PyAny>>,
        on_exit: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        Self {
            target_action,
            on_exit,
        }
    }

    fn __repr__(&self) -> String {
        "OnProcessExit(...)".to_string()
    }
}

/// Mock OnStateTransition event handler
///
/// Python equivalent:
/// ```python
/// from launch.event_handlers import OnStateTransition
/// from lifecycle_msgs.msg import Transition
///
/// OnStateTransition(
///     target_lifecycle_node=lifecycle_node,
///     goal_state='active',
///     entities=[
///         LogInfo(msg='Transitioned to active'),
///     ]
/// )
/// ```
///
/// Triggers actions when a lifecycle node transitions to a target state
#[pyclass(module = "launch.event_handlers", from_py_object)]
#[derive(Clone)]
pub struct OnStateTransition {
    #[allow(dead_code)] // Keep for API compatibility
    target_lifecycle_node: Option<Py<PyAny>>,
    #[allow(dead_code)] // Keep for API compatibility
    goal_state: Option<String>,
    entities: Option<Py<PyAny>>,
}

#[pymethods]
impl OnStateTransition {
    #[new]
    #[pyo3(signature = (*, target_lifecycle_node=None, goal_state=None, entities=None, **_kwargs))]
    fn new(
        target_lifecycle_node: Option<Py<PyAny>>,
        goal_state: Option<String>,
        entities: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        Self {
            target_lifecycle_node,
            goal_state,
            entities,
        }
    }

    fn __repr__(&self) -> String {
        format!("OnStateTransition(goal_state={:?})", self.goal_state)
    }
}

/// Mock OnShutdown event handler
///
/// Python equivalent:
/// ```python
/// from launch.event_handlers import OnShutdown
///
/// OnShutdown(
///     on_shutdown=[
///         LogInfo(msg='Shutting down'),
///     ]
/// )
/// ```
///
/// Triggers actions when the launch system shuts down
#[pyclass(module = "launch.event_handlers", from_py_object)]
#[derive(Clone)]
pub struct OnShutdown {
    #[allow(dead_code)] // Keep for API compatibility
    on_shutdown: Option<Vec<Py<PyAny>>>,
}

#[pymethods]
impl OnShutdown {
    #[new]
    #[pyo3(signature = (*, on_shutdown=None, **_kwargs))]
    fn new(
        on_shutdown: Option<Vec<Py<PyAny>>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        let action_count = on_shutdown.as_ref().map_or(0, |v| v.len());
        log::debug!(
            "Python Launch OnShutdown created with {} actions (limited support)",
            action_count
        );
        Self { on_shutdown }
    }

    fn __repr__(&self) -> String {
        format!(
            "OnShutdown({} actions)",
            self.on_shutdown.as_ref().map_or(0, |v| v.len())
        )
    }
}

/// What an `OnProcessExit` handler makes of its target's exit (#0081): the
/// target, when the handler names one and its `on_exit` shuts the launch
/// down (`Shutdown()`, or an `EmitEvent` of `launch.events.Shutdown`).
/// `launch` then ends when that process exits, exactly as for
/// `X(on_exit=Shutdown())`. Anything else is `None`: other exit actions are
/// not modelled, and an untargeted handler is reported by the caller.
pub(crate) enum ExitHandler {
    /// Not an `OnProcessExit`, or one whose `on_exit` does not shut down.
    Other,
    /// Shuts down when this action's process exits.
    ShutdownOn(Py<PyAny>),
    /// Shuts down when ANY process exits: no `target_action`.
    ShutdownOnAny,
}

pub(crate) fn exit_handler(py: Python, handler: &Bound<'_, PyAny>) -> ExitHandler {
    let Ok(h) = handler.cast::<OnProcessExit>() else {
        return ExitHandler::Other;
    };
    let h = h.borrow();
    let shuts_down = h.on_exit.as_ref().is_some_and(|e| {
        let e = e.bind(py);
        !e.is_callable() && crate::api::launch_ros::on_exit_shuts_down(e)
    });
    if !shuts_down {
        return ExitHandler::Other;
    }
    match &h.target_action {
        Some(t) if !t.bind(py).is_none() => ExitHandler::ShutdownOn(t.clone_ref(py)),
        _ => ExitHandler::ShutdownOnAny,
    }
}

/// The actions a handler runs as the launch COMES UP, which a real launch
/// executes and so the walk does too: `OnProcessStart`'s `on_start` and
/// `OnStateTransition`'s `entities`. A callable `on_start` needs the start
/// event, which a static walk does not have, so it is skipped. Handlers for
/// things stopping (`OnProcessExit`, `OnShutdown`) contribute nothing.
pub(crate) fn startup_entities(
    py: Python,
    handler: &Bound<'_, PyAny>,
) -> PyResult<Option<Py<PyAny>>> {
    let entities = if let Ok(h) = handler.cast::<OnProcessStart>() {
        h.borrow().on_start.as_ref().map(|e| e.clone_ref(py))
    } else if let Ok(h) = handler.cast::<OnStateTransition>() {
        h.borrow().entities.as_ref().map(|e| e.clone_ref(py))
    } else {
        None
    };
    Ok(entities.filter(|e| !e.bind(py).is_callable()))
}
