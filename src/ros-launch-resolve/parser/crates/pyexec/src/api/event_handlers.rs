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
    /// Shuts down when this action's process exits: on every exit, or, for
    /// an `on_exit` FUNCTION, on the exits it returned a shutdown for
    /// (#0090).
    ShutdownOn(Py<PyAny>, crate::api::visit::ExitOn),
    /// Shuts down when ANY process exits: no `target_action`, or a
    /// `target_action` that is a matcher function.
    ShutdownOnAny,
}

pub(crate) fn exit_handler(py: Python, handler: &Bound<'_, PyAny>) -> ExitHandler {
    let Ok(h) = handler.cast::<OnProcessExit>() else {
        return ExitHandler::Other;
    };
    let h = h.borrow();
    let target = h
        .target_action
        .as_ref()
        .filter(|t| !t.bind(py).is_none())
        .map(|t| t.clone_ref(py));
    let Some(on_exit) = h.on_exit.as_ref().map(|e| e.bind(py).clone()) else {
        return ExitHandler::Other;
    };
    // A matcher function is callable; a process action (a mock) is not.
    let target = target.filter(|t| !t.bind(py).is_callable());
    let on = if on_exit.is_callable() {
        let what = match &target {
            Some(_) => "an OnProcessExit handler".to_string(),
            None => "an OnProcessExit handler with no target_action".to_string(),
        };
        match crate::api::visit::probe_on_exit(
            py,
            &on_exit,
            target.as_ref().map(|t| t.bind(py)),
            &what,
        ) {
            Some(on) => on,
            None => return ExitHandler::Other,
        }
    } else if crate::api::launch_ros::on_exit_shuts_down(&on_exit) {
        crate::api::visit::ExitOn::Any
    } else {
        return ExitHandler::Other;
    };
    match target {
        Some(t) => ExitHandler::ShutdownOn(t, on),
        None => ExitHandler::ShutdownOnAny,
    }
}

/// The actions a handler runs as the launch COMES UP, which a real launch
/// executes and so the walk does too: `OnProcessStart`'s `on_start` and
/// `OnStateTransition`'s `entities`. Handlers for things stopping
/// (`OnProcessExit`, `OnShutdown`) contribute nothing.
///
/// A callable `on_start` is CALLED, as launch calls it, with a
/// `ProcessStarted`-shaped event and the launch context, and what it returns
/// is walked (#0101). It used to be skipped without a word, so a launch that
/// loads its composables from such a handler ran with an empty container.
/// The event carries the target action and its name; there is no process
/// yet, so `pid` is `None`. A function that raises is reported and skipped.
pub(crate) fn startup_entities(
    py: Python,
    handler: &Bound<'_, PyAny>,
) -> PyResult<Option<Py<PyAny>>> {
    if let Ok(h) = handler.cast::<OnProcessStart>() {
        let (target, on_start) = {
            let h = h.borrow();
            (
                h.target_action.as_ref().map(|t| t.clone_ref(py)),
                h.on_start.as_ref().map(|e| e.clone_ref(py)),
            )
        };
        let Some(on_start) = on_start else {
            return Ok(None);
        };
        if !on_start.bind(py).is_callable() {
            return Ok(Some(on_start));
        }
        return call_on_start(py, on_start.bind(py), target.as_ref().map(|t| t.bind(py)));
    }
    if let Ok(h) = handler.cast::<OnStateTransition>() {
        let entities = h.borrow().entities.as_ref().map(|e| e.clone_ref(py));
        if let Some(e) = &entities
            && e.bind(py).is_callable()
        {
            crate::api::utils::warn_user(
                "An OnStateTransition handler's entities is a function; play_launch does not \
                 call it, so the actions it would return are not part of this launch.",
            );
            return Ok(None);
        }
        return Ok(entities);
    }
    Ok(None)
}

/// Call a callable `on_start(event, context)` the way launch's
/// `OnProcessStart` does, and walk what it returns, as an `OpaqueFunction`'s.
fn call_on_start(
    py: Python,
    f: &Bound<'_, PyAny>,
    target: Option<&Bound<'_, PyAny>>,
) -> PyResult<Option<Py<PyAny>>> {
    let ns = py.import("types")?.getattr("SimpleNamespace")?;
    let kwargs = pyo3::types::PyDict::new(py);
    let name = target
        .and_then(|t| t.getattr("name").ok())
        .unwrap_or_else(|| py.None().into_bound(py));
    kwargs.set_item(
        "action",
        target.map_or_else(|| py.None().into_bound(py), |t| t.clone()),
    )?;
    kwargs.set_item("name", name)?;
    kwargs.set_item("pid", py.None())?;
    kwargs.set_item("cmd", pyo3::types::PyList::empty(py))?;
    kwargs.set_item("cwd", py.None())?;
    kwargs.set_item("env", py.None())?;
    let event = ns.call((), Some(&kwargs))?;
    let context = crate::api::utils::create_launch_context(py)?;
    // What the function returns is as invisible to launch's include-time
    // argument check as an `OpaqueFunction`'s, so it runs as opaque too.
    play_launch_parser::bridge::enter_opaque_function();
    let result = match f.call1((event, context)) {
        Ok(r) if r.is_none() => Ok(()),
        Ok(r) => crate::api::visit::visit_any(py, &r),
        Err(e) => {
            crate::api::utils::warn_user(&format!(
                "The on_start function of an OnProcessStart handler raised when called at \
                 parse time ({e}); the actions it would return are not part of this launch."
            ));
            Ok(())
        }
    };
    play_launch_parser::bridge::leave_opaque_function();
    result.map(|()| None)
}
