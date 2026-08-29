//! Simple action types: LogInfo, SetEnvironmentVariable, UnsetEnvironmentVariable,
//! ExecuteProcess, ExecuteLocal, TimerAction, OpaqueCoroutine

use pyo3::prelude::*;

/// Mock LogInfo action
///
/// Python equivalent:
/// ```python
/// from launch.actions import LogInfo
/// log_action = LogInfo(msg='Information message')
/// log_action = LogInfo(msg=['Prefix: ', LaunchConfiguration('var'), ' suffix'])
/// ```
///
/// Logs an information message when the action is executed
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct LogInfo {
    msg: String,
}

#[pymethods]
impl LogInfo {
    #[new]
    #[pyo3(signature = (*, msg, **_kwargs))]
    fn new(
        py: Python,
        msg: Py<PyAny>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> PyResult<Self> {
        // Convert msg to string (handles strings, lists, LaunchConfiguration, etc.)
        let msg_str = Self::pyobject_to_string(py, &msg)?;

        // Log the message immediately
        log::info!("Python Launch LogInfo: {}", msg_str);
        Ok(Self { msg: msg_str })
    }

    fn __repr__(&self) -> String {
        format!("LogInfo(msg='{}')", self.msg)
    }
}

impl LogInfo {
    /// Convert a Py<PyAny> to a string (handles both strings and substitutions)
    /// Same logic as Node::pyobject_to_string
    fn pyobject_to_string(py: Python, obj: &Py<PyAny>) -> PyResult<String> {
        crate::api::utils::pyobject_to_string(py, obj)
    }
}

/// Mock SetEnvironmentVariable action
///
/// Python equivalent:
/// ```python
/// from launch.actions import SetEnvironmentVariable
/// set_env = SetEnvironmentVariable('VAR_NAME', 'value')
/// set_env = SetEnvironmentVariable('VAR_NAME', LaunchConfiguration('var'))
/// set_env = SetEnvironmentVariable('VAR_NAME', [LaunchConfiguration('prefix'), '/suffix'])
/// ```
///
/// Sets an environment variable
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct SetEnvironmentVariable {
    name: String,
    value: String,
}

#[pymethods]
impl SetEnvironmentVariable {
    #[new]
    fn new(py: Python, name: Py<PyAny>, value: Py<PyAny>) -> PyResult<Self> {
        // Convert PyObjects to strings (handles strings, substitutions, and lists)
        let name_str = Self::pyobject_to_string(py, &name)?;
        let value_str = Self::pyobject_to_string(py, &value)?;

        log::debug!(
            "Python Launch SetEnvironmentVariable: {}={}",
            name_str,
            value_str
        );
        Ok(Self {
            name: name_str,
            value: value_str,
        })
    }

    fn __repr__(&self) -> String {
        format!("SetEnvironmentVariable('{}', '{}')", self.name, self.value)
    }
}

impl SetEnvironmentVariable {
    /// Convert a Py<PyAny> to a string (handles strings, substitutions, and lists)
    /// Reuses the same pattern as LogInfo
    fn pyobject_to_string(py: Python, obj: &Py<PyAny>) -> PyResult<String> {
        crate::api::utils::pyobject_to_string(py, obj)
    }
}

/// Mock UnsetEnvironmentVariable action
///
/// Python equivalent:
/// ```python
/// from launch.actions import UnsetEnvironmentVariable
/// unset_env = UnsetEnvironmentVariable('VAR_NAME')
/// ```
///
/// Unsets an environment variable
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct UnsetEnvironmentVariable {
    name: String,
}

#[pymethods]
impl UnsetEnvironmentVariable {
    #[new]
    fn new(name: String) -> Self {
        log::debug!("Python Launch UnsetEnvironmentVariable: {}", name);
        Self { name }
    }

    fn __repr__(&self) -> String {
        format!("UnsetEnvironmentVariable('{}')", self.name)
    }
}

/// Mock ExecuteProcess action
///
/// Python equivalent:
/// ```python
/// from launch.actions import ExecuteProcess
/// proc = ExecuteProcess(
///     cmd=['command', 'arg1', 'arg2'],
///     cwd='/path/to/dir',
///     name='process_name',
///     output='screen'
/// )
/// # With substitutions:
/// proc = ExecuteProcess(
///     cmd=['ros2', 'run', LaunchConfiguration('package'), 'node'],
///     cwd=[FindPackageShare('my_pkg'), '/dir']
/// )
/// ```
///
/// Executes a non-ROS process
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct ExecuteProcess {
    cmd: Vec<String>,
    #[allow(dead_code)] // Keep for future use
    cwd: Option<String>,
    #[allow(dead_code)] // Keep for future use
    name: Option<String>,
    #[allow(dead_code)] // Keep for future use
    output: String,
}

#[pymethods]
impl ExecuteProcess {
    #[new]
    #[pyo3(signature = (*, cmd, cwd=None, name=None, output=None, **_kwargs))]
    fn new(
        py: Python,
        cmd: Vec<Py<PyAny>>,
        cwd: Option<Py<PyAny>>,
        name: Option<Py<PyAny>>,
        output: Option<String>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> PyResult<Self> {
        // Convert cmd elements to strings
        let cmd_strs: Result<Vec<String>, _> = cmd
            .iter()
            .map(|obj| Self::pyobject_to_string(py, obj))
            .collect();
        let cmd_vec = cmd_strs?;

        let cwd_str = cwd
            .map(|obj| Self::pyobject_to_string(py, &obj))
            .transpose()?;

        let name_str = name
            .map(|obj| Self::pyobject_to_string(py, &obj))
            .transpose()?;

        log::debug!("Python Launch ExecuteProcess: {:?}", cmd_vec);

        Ok(Self {
            cmd: cmd_vec,
            cwd: cwd_str,
            name: name_str,
            output: output.unwrap_or_else(|| "screen".to_string()),
        })
    }

    fn __repr__(&self) -> String {
        format!("ExecuteProcess(cmd={:?})", self.cmd)
    }
}

impl ExecuteProcess {
    /// Convert a Py<PyAny> to a string (handles strings, substitutions, and lists)
    /// Reuses the same pattern as SetEnvironmentVariable
    fn pyobject_to_string(py: Python, obj: &Py<PyAny>) -> PyResult<String> {
        crate::api::utils::pyobject_to_string(py, obj)
    }
}

/// Mock ExecuteLocal action
///
/// Python equivalent:
/// ```python
/// from launch.actions import ExecuteLocal
/// ExecuteLocal(
///     process_description=Executable(cmd=['ls', '-la']),
///     output='screen'
/// )
/// ```
///
/// Executes a process on the local system with more control than ExecuteProcess
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct ExecuteLocal {
    #[allow(dead_code)] // Keep for future use
    process_description: Option<Py<PyAny>>,
    #[allow(dead_code)] // Keep for future use
    cmd: Option<Vec<String>>,
    #[allow(dead_code)] // Keep for future use
    cwd: Option<String>,
    #[allow(dead_code)] // Keep for future use
    output: String,
}

#[pymethods]
impl ExecuteLocal {
    #[new]
    #[pyo3(signature = (*, process_description=None, cmd=None, cwd=None, output=None, shell=None, **_kwargs))]
    fn new(
        process_description: Option<Py<PyAny>>,
        cmd: Option<Vec<String>>,
        cwd: Option<String>,
        output: Option<String>,
        shell: Option<bool>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        // Log what we're executing
        if let Some(ref c) = cmd {
            log::debug!("Python Launch ExecuteLocal: cmd={:?}", c);
        } else {
            log::debug!("Python Launch ExecuteLocal: process_description provided");
        }

        if let Some(s) = shell {
            log::debug!("  shell={}", s);
        }

        Self {
            process_description,
            cmd,
            cwd,
            output: output.unwrap_or_else(|| "log".to_string()),
        }
    }

    fn __repr__(&self) -> String {
        if let Some(ref cmd) = self.cmd {
            format!("ExecuteLocal(cmd={:?})", cmd)
        } else {
            "ExecuteLocal(process_description=...)".to_string()
        }
    }
}

/// Mock TimerAction
///
/// Python equivalent:
/// ```python
/// from launch.actions import TimerAction
/// timer = TimerAction(period=10.0, actions=[action1, action2])
/// ```
///
/// Executes actions after a delay
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct TimerAction {
    #[allow(dead_code)] // Keep for future use
    period: f64,
    #[allow(dead_code)] // Keep for future use
    actions: Vec<Py<PyAny>>,
}

#[pymethods]
impl TimerAction {
    #[new]
    #[pyo3(signature = (*, period, actions, **_kwargs))]
    fn new(
        period: f64,
        actions: Vec<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        log::debug!("Python Launch TimerAction: period={}s", period);
        Self { period, actions }
    }

    fn __repr__(&self) -> String {
        format!(
            "TimerAction(period={}, {} actions)",
            self.period,
            self.actions.len()
        )
    }
}

/// Mock OpaqueCoroutine action
///
/// Python equivalent:
/// ```python
/// from launch.actions import OpaqueCoroutine
/// async def my_coroutine(context):
///     # async operations
///     pass
/// OpaqueCoroutine(coroutine=my_coroutine)
/// ```
///
/// Adds a Python coroutine function to the launch run loop.
/// For static analysis, we just capture that it was called.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct OpaqueCoroutine {
    #[allow(dead_code)] // Keep for future use
    coroutine: Py<PyAny>,
    #[allow(dead_code)] // Keep for future use
    args: Vec<Py<PyAny>>,
    #[allow(dead_code)] // Keep for future use
    func_kwargs: Option<Py<PyAny>>,
}

#[pymethods]
impl OpaqueCoroutine {
    #[new]
    #[pyo3(signature = (*, coroutine, args=None, kwargs=None, **_extra_kwargs))]
    fn new(
        coroutine: Py<PyAny>,
        args: Option<Vec<Py<PyAny>>>,
        kwargs: Option<Py<PyAny>>,
        _extra_kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        log::debug!("Python Launch OpaqueCoroutine: coroutine provided");
        Self {
            coroutine,
            args: args.unwrap_or_default(),
            func_kwargs: kwargs,
        }
    }

    fn __repr__(&self) -> String {
        "OpaqueCoroutine(...)".to_string()
    }
}
