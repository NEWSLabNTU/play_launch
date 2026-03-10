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
#[pyclass(module = "launch.event_handlers")]
#[derive(Clone)]
pub struct OnProcessStart {
    #[allow(dead_code)] // Keep for API compatibility
    target_action: Option<PyObject>,
    #[allow(dead_code)] // Keep for API compatibility
    on_start: Option<Vec<PyObject>>,
}

#[pymethods]
impl OnProcessStart {
    #[new]
    #[pyo3(signature = (*, target_action=None, on_start=None, **_kwargs))]
    fn new(
        target_action: Option<PyObject>,
        on_start: Option<Vec<PyObject>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        let action_count = on_start.as_ref().map_or(0, |v| v.len());
        log::debug!(
            "Python Launch OnProcessStart created with {} actions (limited support)",
            action_count
        );
        Self {
            target_action,
            on_start,
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "OnProcessStart({} actions)",
            self.on_start.as_ref().map_or(0, |v| v.len())
        )
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
#[pyclass(module = "launch.event_handlers")]
#[derive(Clone)]
pub struct OnProcessExit {
    #[allow(dead_code)] // Keep for API compatibility
    target_action: Option<PyObject>,
    #[allow(dead_code)] // Keep for API compatibility
    on_exit: Option<Vec<PyObject>>,
}

#[pymethods]
impl OnProcessExit {
    #[new]
    #[pyo3(signature = (*, target_action=None, on_exit=None, **_kwargs))]
    fn new(
        target_action: Option<PyObject>,
        on_exit: Option<Vec<PyObject>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        let action_count = on_exit.as_ref().map_or(0, |v| v.len());
        log::debug!(
            "Python Launch OnProcessExit created with {} actions (limited support)",
            action_count
        );
        Self {
            target_action,
            on_exit,
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "OnProcessExit({} actions)",
            self.on_exit.as_ref().map_or(0, |v| v.len())
        )
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
#[pyclass(module = "launch.event_handlers")]
#[derive(Clone)]
pub struct OnStateTransition {
    #[allow(dead_code)] // Keep for API compatibility
    target_lifecycle_node: Option<PyObject>,
    #[allow(dead_code)] // Keep for API compatibility
    goal_state: Option<String>,
    #[allow(dead_code)] // Keep for API compatibility
    entities: Option<Vec<PyObject>>,
}

#[pymethods]
impl OnStateTransition {
    #[new]
    #[pyo3(signature = (*, target_lifecycle_node=None, goal_state=None, entities=None, **_kwargs))]
    fn new(
        target_lifecycle_node: Option<PyObject>,
        goal_state: Option<String>,
        entities: Option<Vec<PyObject>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        let entity_count = entities.as_ref().map_or(0, |v| v.len());
        log::debug!(
            "Python Launch OnStateTransition created for state '{:?}' with {} entities (limited support)",
            goal_state,
            entity_count
        );
        Self {
            target_lifecycle_node,
            goal_state,
            entities,
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "OnStateTransition(goal_state={:?}, {} entities)",
            self.goal_state,
            self.entities.as_ref().map_or(0, |v| v.len())
        )
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
#[pyclass(module = "launch.event_handlers")]
#[derive(Clone)]
pub struct OnShutdown {
    #[allow(dead_code)] // Keep for API compatibility
    on_shutdown: Option<Vec<PyObject>>,
}

#[pymethods]
impl OnShutdown {
    #[new]
    #[pyo3(signature = (*, on_shutdown=None, **_kwargs))]
    fn new(
        on_shutdown: Option<Vec<PyObject>>,
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
