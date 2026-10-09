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

use crate::api::utils::pyobject_to_string;
use play_launch_parser::bridge::with_launch_context;
use pyo3::{prelude::*, types::PyDict};

/// A global parameter's value as this context stores it: booleans in
/// Python's spelling, as `<set_parameter>` stores them.
fn normalize_global_value(value: String) -> String {
    match value.as_str() {
        "false" => "False".to_string(),
        "true" => "True".to_string(),
        _ => value,
    }
}

/// `SetParameter(name, value)`: appended to `global_params` when executed,
/// so every node executed AFTER it gets the parameter, until a scoped group
/// that created the list ends.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct SetParameter {
    name: Py<PyAny>,
    value: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl SetParameter {
    #[new]
    #[pyo3(signature = (name, value, *, condition=None, **_kwargs))]
    fn new(
        name: Py<PyAny>,
        value: Py<PyAny>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            name,
            value,
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "SetParameter(...)".to_string()
    }
}

impl SetParameter {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let me = this.borrow();
        let name = pyobject_to_string(py, &me.name)?;
        let value = normalize_global_value(node::Node::extract_param_value(me.value.bind(py))?);
        with_launch_context(|ctx| ctx.set_global_parameter(name, value));
        Ok(())
    }
}

/// `SetUseSimTime(value)`: `SetParameter('use_sim_time', value)`.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct SetUseSimTime {
    value: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl SetUseSimTime {
    #[new]
    #[pyo3(signature = (value, *, condition=None, **_kwargs))]
    fn new(
        value: Py<PyAny>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self { value, condition }
    }

    fn __repr__(&self) -> String {
        "SetUseSimTime(...)".to_string()
    }
}

impl SetUseSimTime {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let value = normalize_global_value(node::Node::extract_param_value(
            this.borrow().value.bind(py),
        )?);
        with_launch_context(|ctx| ctx.set_global_parameter("use_sim_time".to_string(), value));
        Ok(())
    }
}

/// `SetParametersFromFile(filename, node_name=None)`. `launch_ros` appends the
/// FILE to `global_params`; this model holds global parameters as values, so
/// it is reported rather than silently dropped.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct SetParametersFromFile {
    filename: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl SetParametersFromFile {
    #[new]
    #[pyo3(signature = (filename, *, condition=None, **_kwargs))]
    fn new(
        filename: Py<PyAny>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            filename,
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "SetParametersFromFile(...)".to_string()
    }
}

impl SetParametersFromFile {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let file = pyobject_to_string(py, &this.borrow().filename)?;
        play_launch_parser::bridge::note_unsupported_action(
            "SetParametersFromFile",
            Some(format!(
                "SetParametersFromFile({file}): a global parameter FILE is not modelled, so the \
                 nodes after it do not get its parameters. Pass the file in each node's \
                 `parameters=`."
            )),
        );
        Ok(())
    }
}

/// `RosTimer(period, actions)`: `TimerAction` on the ROS clock.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct RosTimer {
    pub(crate) actions: Py<PyAny>,
    pub(crate) period: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl RosTimer {
    #[new]
    #[pyo3(signature = (*, period, actions=None, condition=None, **_kwargs))]
    fn new(
        py: Python,
        period: Py<PyAny>,
        actions: Option<Py<PyAny>>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            actions: actions.unwrap_or_else(|| py.None()),
            period,
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "RosTimer(...)".to_string()
    }
}

impl RosTimer {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let (period, actions) = {
            let me = this.borrow();
            (me.period.clone_ref(py), me.actions.clone_ref(py))
        };
        crate::api::actions::run_timer(py, "RosTimer", &period, &actions)
    }
}

/// `SetRemap(src, dst)`: appended to `ros_remaps` when executed, ahead of
/// every node's own remappings from then on.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct SetRemap {
    src: Py<PyAny>,
    dst: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl SetRemap {
    #[new]
    #[pyo3(signature = (src, dst, *, condition=None, **_kwargs))]
    fn new(
        src: Py<PyAny>,
        dst: Py<PyAny>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            src,
            dst,
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "SetRemap(...)".to_string()
    }
}

impl SetRemap {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let me = this.borrow();
        let src = pyobject_to_string(py, &me.src)?;
        let dst = pyobject_to_string(py, &me.dst)?;
        with_launch_context(|ctx| ctx.add_remapping(src, dst));
        Ok(())
    }
}

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
        Self { log_dir }
    }

    fn __repr__(&self) -> String {
        "SetROSLogDir(...)".to_string()
    }
}
