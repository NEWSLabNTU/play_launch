//! Configuration management actions: SetLaunchConfiguration, RegisterEventHandler,
//! PushEnvironment, PopEnvironment, ResetEnvironment, AppendEnvironmentVariable,
//! PushLaunchConfigurations, PopLaunchConfigurations, ResetLaunchConfigurations,
//! UnsetLaunchConfiguration, Shutdown

use pyo3::prelude::*;

/// Mock SetLaunchConfiguration action
///
/// Python equivalent:
/// ```python
/// from launch.actions import SetLaunchConfiguration
/// set_config = SetLaunchConfiguration(name='config_name', value='value')
/// ```
///
/// Sets a launch configuration value
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct SetLaunchConfiguration {
    #[allow(dead_code)] // Stored for API compatibility
    name: PyObject,
    #[allow(dead_code)] // Stored for API compatibility
    value: PyObject,
}

#[pymethods]
impl SetLaunchConfiguration {
    #[new]
    #[pyo3(signature = (name, value, **_kwargs))]
    fn new(
        py: Python,
        name: PyObject,
        value: PyObject,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> PyResult<Self> {
        // Convert name to string (may be a substitution)
        let name_str = if let Ok(s) = name.extract::<String>(py) {
            s
        } else if let Ok(str_result) = name.call_method0(py, "__str__") {
            str_result.extract::<String>(py)?
        } else {
            name.to_string()
        };

        // Convert value to string
        let value_str =
            crate::python::api::utils::pyobject_to_string(py, &value).unwrap_or_default();

        log::debug!(
            "Python Launch SetLaunchConfiguration: {}={}",
            name_str,
            value_str
        );

        // Store in thread-local LaunchContext so subsequent LaunchConfiguration lookups resolve
        crate::python::bridge::try_with_launch_context(|ctx| {
            ctx.set_configuration(name_str, value_str);
        });

        Ok(Self { name, value })
    }

    fn __repr__(&self) -> String {
        "SetLaunchConfiguration(...)".to_string()
    }
}

/// Mock RegisterEventHandler action
///
/// Python equivalent:
/// ```python
/// from launch.actions import RegisterEventHandler
/// from launch.event_handlers import OnProcessStart
///
/// RegisterEventHandler(
///     OnProcessStart(
///         target_action=some_node,
///         on_start=[LogInfo(msg='Started!')]
///     )
/// )
/// ```
///
/// Event handlers allow actions to be triggered in response to events
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct RegisterEventHandler {
    #[allow(dead_code)] // Keep for API compatibility
    event_handler: PyObject,
}

#[pymethods]
impl RegisterEventHandler {
    #[new]
    #[pyo3(signature = (event_handler, **_kwargs))]
    fn new(event_handler: PyObject, _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>) -> Self {
        log::debug!("Python Launch RegisterEventHandler created (limited support)");
        Self { event_handler }
    }

    fn __repr__(&self) -> String {
        "RegisterEventHandler(...)".to_string()
    }
}

/// Mock PushEnvironment action
///
/// Python equivalent:
/// ```python
/// from launch.actions import PushEnvironment
/// PushEnvironment()
/// ```
///
/// Pushes the current environment state onto a stack.
/// This allows temporary environment modifications that can be reverted with PopEnvironment.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct PushEnvironment {}

#[pymethods]
impl PushEnvironment {
    #[new]
    #[pyo3(signature = (**_kwargs))]
    fn new(_kwargs: Option<&Bound<'_, pyo3::types::PyDict>>) -> Self {
        log::debug!("Python Launch PushEnvironment: pushing environment state");
        Self {}
    }

    fn __repr__(&self) -> String {
        "PushEnvironment()".to_string()
    }
}

/// Mock PopEnvironment action
///
/// Python equivalent:
/// ```python
/// from launch.actions import PopEnvironment
/// PopEnvironment()
/// ```
///
/// Pops the most recent environment state from the stack, restoring it.
/// Must be paired with a previous PushEnvironment.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct PopEnvironment {}

#[pymethods]
impl PopEnvironment {
    #[new]
    #[pyo3(signature = (**_kwargs))]
    fn new(_kwargs: Option<&Bound<'_, pyo3::types::PyDict>>) -> Self {
        log::debug!("Python Launch PopEnvironment: popping environment state");
        Self {}
    }

    fn __repr__(&self) -> String {
        "PopEnvironment()".to_string()
    }
}

/// Mock ResetEnvironment action
///
/// Python equivalent:
/// ```python
/// from launch.actions import ResetEnvironment
/// ResetEnvironment()
/// ```
///
/// Resets the environment to its initial state (before any modifications).
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct ResetEnvironment {}

#[pymethods]
impl ResetEnvironment {
    #[new]
    #[pyo3(signature = (**_kwargs))]
    fn new(_kwargs: Option<&Bound<'_, pyo3::types::PyDict>>) -> Self {
        log::debug!("Python Launch ResetEnvironment: resetting environment to initial state");
        Self {}
    }

    fn __repr__(&self) -> String {
        "ResetEnvironment()".to_string()
    }
}

/// Mock AppendEnvironmentVariable action
///
/// Python equivalent:
/// ```python
/// from launch.actions import AppendEnvironmentVariable
/// AppendEnvironmentVariable('PATH', '/custom/path')
/// AppendEnvironmentVariable('LD_LIBRARY_PATH', '/custom/lib', prepend=True, separator=':')
/// AppendEnvironmentVariable(LaunchConfiguration('var_name'), '/path', separator=LaunchConfiguration('sep'))
/// ```
///
/// Appends (or prepends) a value to an existing environment variable.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct AppendEnvironmentVariable {
    name: String,
    value: PyObject,
    #[allow(dead_code)]
    prepend: bool,
    #[allow(dead_code)]
    separator: String,
}

#[pymethods]
impl AppendEnvironmentVariable {
    #[new]
    #[pyo3(signature = (name, value, *, prepend=None, separator=None, **_kwargs))]
    fn new(
        py: Python,
        name: PyObject,
        value: PyObject,
        prepend: Option<PyObject>,
        separator: Option<PyObject>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> PyResult<Self> {
        // Convert name to string (handles strings, substitutions, and lists)
        let name_str = Self::pyobject_to_string(py, &name)?;

        // Handle prepend (bool or substitution resolving to bool)
        let prepend_val = if let Some(p) = prepend {
            if let Ok(b) = p.extract::<bool>(py) {
                b
            } else if let Ok(s) = Self::pyobject_to_string(py, &p) {
                // Parse string as bool (YAML rules: true, True, yes, 1, etc.)
                matches!(s.to_lowercase().as_str(), "true" | "yes" | "1")
            } else {
                false
            }
        } else {
            false
        };

        // Handle separator (default to ":")
        let sep_str = if let Some(s) = separator {
            Self::pyobject_to_string(py, &s)?
        } else {
            ":".to_string()
        };

        // Convert PyObject to string for logging
        let value_str = if let Ok(s) = value.extract::<String>(py) {
            s.clone()
        } else if let Ok(str_result) = value.call_method0(py, "__str__") {
            str_result
                .extract::<String>(py)
                .unwrap_or_else(|_| "<value>".to_string())
        } else {
            "<value>".to_string()
        };

        log::debug!(
            "Python Launch AppendEnvironmentVariable: {}{}{}{}",
            if prepend_val {
                "prepending "
            } else {
                "appending "
            },
            value_str,
            sep_str,
            name_str
        );

        Ok(Self {
            name: name_str,
            value,
            prepend: prepend_val,
            separator: sep_str,
        })
    }

    fn __repr__(&self, py: Python) -> String {
        let value_str = if let Ok(s) = self.value.extract::<String>(py) {
            s
        } else if let Ok(str_result) = self.value.call_method0(py, "__str__") {
            str_result
                .extract::<String>(py)
                .unwrap_or_else(|_| "<value>".to_string())
        } else {
            "<value>".to_string()
        };

        format!(
            "AppendEnvironmentVariable('{}', '{}', prepend={}, separator='{}')",
            self.name, value_str, self.prepend, self.separator
        )
    }
}

impl AppendEnvironmentVariable {
    /// Convert a PyObject to a string (handles strings, substitutions, and lists)
    /// Reuses the same pattern as SetEnvironmentVariable
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        crate::python::api::utils::pyobject_to_string(py, obj)
    }
}

/// Mock PushLaunchConfigurations action
///
/// Python equivalent:
/// ```python
/// from launch.actions import PushLaunchConfigurations
/// PushLaunchConfigurations()
/// ```
///
/// Pushes the current launch configurations onto a stack.
/// This allows temporary configuration modifications that can be reverted with PopLaunchConfigurations.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct PushLaunchConfigurations {}

#[pymethods]
impl PushLaunchConfigurations {
    #[new]
    #[pyo3(signature = (**_kwargs))]
    fn new(_kwargs: Option<&Bound<'_, pyo3::types::PyDict>>) -> Self {
        log::debug!("Python Launch PushLaunchConfigurations: pushing configuration state");
        Self {}
    }

    fn __repr__(&self) -> String {
        "PushLaunchConfigurations()".to_string()
    }
}

/// Mock PopLaunchConfigurations action
///
/// Python equivalent:
/// ```python
/// from launch.actions import PopLaunchConfigurations
/// PopLaunchConfigurations()
/// ```
///
/// Pops the most recent launch configurations from the stack, restoring them.
/// Must be paired with a previous PushLaunchConfigurations.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct PopLaunchConfigurations {}

#[pymethods]
impl PopLaunchConfigurations {
    #[new]
    #[pyo3(signature = (**_kwargs))]
    fn new(_kwargs: Option<&Bound<'_, pyo3::types::PyDict>>) -> Self {
        log::debug!("Python Launch PopLaunchConfigurations: popping configuration state");
        Self {}
    }

    fn __repr__(&self) -> String {
        "PopLaunchConfigurations()".to_string()
    }
}

/// Mock ResetLaunchConfigurations action
///
/// Python equivalent:
/// ```python
/// from launch.actions import ResetLaunchConfigurations
/// ResetLaunchConfigurations()
/// ```
///
/// Resets all launch configurations to their initial state (clearing any modifications).
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct ResetLaunchConfigurations {}

#[pymethods]
impl ResetLaunchConfigurations {
    #[new]
    #[pyo3(signature = (**_kwargs))]
    fn new(_kwargs: Option<&Bound<'_, pyo3::types::PyDict>>) -> Self {
        log::debug!(
            "Python Launch ResetLaunchConfigurations: resetting configurations to initial state"
        );
        Self {}
    }

    fn __repr__(&self) -> String {
        "ResetLaunchConfigurations()".to_string()
    }
}

/// Mock UnsetLaunchConfiguration action
///
/// Python equivalent:
/// ```python
/// from launch.actions import UnsetLaunchConfiguration
/// UnsetLaunchConfiguration('variable_name')
/// ```
///
/// Removes a specific launch configuration variable.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct UnsetLaunchConfiguration {
    name: String,
}

#[pymethods]
impl UnsetLaunchConfiguration {
    #[new]
    fn new(name: String) -> Self {
        log::debug!("Python Launch UnsetLaunchConfiguration: {}", name);
        Self { name }
    }

    fn __repr__(&self) -> String {
        format!("UnsetLaunchConfiguration('{}')", self.name)
    }
}

/// Mock Shutdown action
///
/// Python equivalent:
/// ```python
/// from launch.actions import Shutdown
/// Shutdown()
/// ```
///
/// Triggers a shutdown of the launch system when executed.
/// For static analysis, this is purely informational.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct Shutdown {
    #[allow(dead_code)] // Keep for future use
    reason: Option<String>,
}

#[pymethods]
impl Shutdown {
    #[new]
    #[pyo3(signature = (*, reason=None, **_kwargs))]
    fn new(reason: Option<String>, _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>) -> Self {
        if let Some(ref r) = reason {
            log::debug!("Python Launch Shutdown: reason={}", r);
        } else {
            log::debug!("Python Launch Shutdown");
        }
        Self { reason }
    }

    fn __repr__(&self) -> String {
        if let Some(ref r) = self.reason {
            format!("Shutdown(reason='{}')", r)
        } else {
            "Shutdown()".to_string()
        }
    }
}
