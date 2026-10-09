//! Simple actions (`launch.actions`).

use crate::api::utils::pyobject_to_string;
use play_launch_parser::bridge::with_launch_context;
use pyo3::{prelude::*, types::PyDict};

/// `LogInfo(msg=...)`: logged when executed; nothing to model.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct LogInfo {
    msg: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl LogInfo {
    #[new]
    #[pyo3(signature = (*, msg, condition=None, **_kwargs))]
    fn new(
        msg: Py<PyAny>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self { msg, condition }
    }

    fn __repr__(&self) -> String {
        "LogInfo(...)".to_string()
    }
}

impl LogInfo {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let msg = pyobject_to_string(py, &this.borrow().msg)?;
        log::debug!("Python Launch LogInfo: {}", msg);
        Ok(())
    }
}

/// `SetEnvironmentVariable(name, value)`: every process started after it
/// inherits the variable, until a scoped group around it ends.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct SetEnvironmentVariable {
    name: Py<PyAny>,
    value: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl SetEnvironmentVariable {
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
        "SetEnvironmentVariable(...)".to_string()
    }
}

impl SetEnvironmentVariable {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let me = this.borrow();
        let name = pyobject_to_string(py, &me.name)?;
        let value = pyobject_to_string(py, &me.value)?;
        with_launch_context(|ctx| ctx.set_environment_variable(name, value));
        Ok(())
    }
}

/// `UnsetEnvironmentVariable(name)`.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct UnsetEnvironmentVariable {
    name: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl UnsetEnvironmentVariable {
    #[new]
    #[pyo3(signature = (name, *, condition=None, **_kwargs))]
    fn new(
        name: Py<PyAny>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self { name, condition }
    }

    fn __repr__(&self) -> String {
        "UnsetEnvironmentVariable(...)".to_string()
    }
}

impl UnsetEnvironmentVariable {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let name = pyobject_to_string(py, &this.borrow().name)?;
        with_launch_context(|ctx| ctx.unset_environment_variable(&name));
        Ok(())
    }
}

/// `ExecuteProcess(cmd=[...], name=None, additional_env=None)`: a process
/// that is not a ROS node, started with the environment in effect.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct ExecuteProcess {
    cmd: Py<PyAny>,
    name: Option<Py<PyAny>>,
    additional_env: Option<Py<PyAny>>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl ExecuteProcess {
    #[new]
    #[pyo3(signature = (*, cmd, name=None, additional_env=None, condition=None, **_kwargs))]
    fn new(
        cmd: Py<PyAny>,
        name: Option<Py<PyAny>>,
        additional_env: Option<Py<PyAny>>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            cmd,
            name,
            additional_env,
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "ExecuteProcess(...)".to_string()
    }
}

impl ExecuteProcess {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        if !crate::api::visit::claim_execution(this.as_any(), "ExecuteProcess") {
            return Ok(());
        }
        let me = this.borrow();
        // Each `cmd` element is one argument after substitution.
        let mut parts = Vec::new();
        for item in me.cmd.bind(py).try_iter()? {
            parts.push(pyobject_to_string(py, &item?.unbind())?);
        }
        if parts.is_empty() {
            return Ok(());
        }
        let name = me
            .name
            .as_ref()
            .map(|n| pyobject_to_string(py, n))
            .transpose()?;
        let mut env: Vec<(String, String)> =
            with_launch_context(|ctx| ctx.environment().into_iter().collect());
        if let Some(extra) = &me.additional_env {
            for item in extra.bind(py).call_method0("items")?.try_iter()? {
                let (k, v): (Py<PyAny>, Py<PyAny>) = item?.extract()?;
                env.push((pyobject_to_string(py, &k)?, pyobject_to_string(py, &v)?));
            }
        }
        let mark = crate::api::visit::lens();
        play_launch_parser::bridge::capture_node(play_launch_parser::captures::NodeCapture {
            executable: parts[0].clone(),
            arguments: parts[1..].to_vec(),
            name,
            env_vars: env,
            ..Default::default()
        });
        crate::api::visit::stamp_delay(mark);
        Ok(())
    }
}

/// `ExecuteLocal(process_description=...)`.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct ExecuteLocal {
    #[allow(dead_code)] // Keep for future use
    process_description: Option<Py<PyAny>>,
}

#[pymethods]
impl ExecuteLocal {
    #[new]
    #[pyo3(signature = (*, process_description=None, **_kwargs))]
    fn new(process_description: Option<Py<PyAny>>, _kwargs: Option<&Bound<'_, PyDict>>) -> Self {
        Self {
            process_description,
        }
    }

    fn __repr__(&self) -> String {
        "ExecuteLocal(...)".to_string()
    }
}

/// `TimerAction(period, actions)`: its actions execute when it fires, which
/// for the model means they start `period` seconds after the launch — timers
/// nested in it add. The period is performed when the timer executes.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct TimerAction {
    pub(crate) actions: Py<PyAny>,
    pub(crate) period: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl TimerAction {
    #[new]
    #[pyo3(signature = (*, period, actions, condition=None, **_kwargs))]
    fn new(
        period: Py<PyAny>,
        actions: Py<PyAny>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            actions,
            period,
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "TimerAction(...)".to_string()
    }
}

impl TimerAction {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let (period, actions) = {
            let me = this.borrow();
            (me.period.clone_ref(py), me.actions.clone_ref(py))
        };
        run_timer(py, "TimerAction", &period, &actions)
    }
}

/// Execute a timer's actions under its delay. A period that is not a number
/// when the timer executes leaves them undelayed, and is reported with what
/// it would have delayed named.
pub fn run_timer(py: Python, class: &str, period: &Py<PyAny>, actions: &Py<PyAny>) -> PyResult<()> {
    use crate::api::visit;
    let secs = if let Ok(v) = period.extract::<f64>(py) {
        Some(v)
    } else {
        pyobject_to_string(py, period)
            .ok()
            .and_then(|t| t.trim().parse::<f64>().ok())
    };
    let mark = visit::lens();
    visit::with_delay(secs, || visit::visit_any(py, actions.bind(py)))?;
    if secs.is_none() {
        let repr = period
            .bind(py)
            .repr()
            .and_then(|r| r.extract::<String>())
            .unwrap_or_else(|_| "?".to_string());
        let what = visit::describe_since(mark);
        if !what.is_empty() {
            play_launch_parser::bridge::note_unsupported_action(
                "timer",
                Some(format!(
                    "{class}(period={repr}) in a Python launch file: the period is not a \
                     number this parser can resolve, so these start immediately in the \
                     model: {}. Give the timer a literal period, or express the delay in \
                     XML/YAML `<timer period=…>`.",
                    what.join(", ")
                )),
            );
        }
    }
    Ok(())
}

/// `OpaqueCoroutine(coroutine=...)`: runs asynchronously in a live launch;
/// nothing to model.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct OpaqueCoroutine {
    #[allow(dead_code)] // Keep for future use
    coroutine: Py<PyAny>,
}

#[pymethods]
impl OpaqueCoroutine {
    #[new]
    #[pyo3(signature = (*, coroutine, **_kwargs))]
    fn new(coroutine: Py<PyAny>, _kwargs: Option<&Bound<'_, PyDict>>) -> Self {
        Self { coroutine }
    }

    fn __repr__(&self) -> String {
        "OpaqueCoroutine(...)".to_string()
    }
}
