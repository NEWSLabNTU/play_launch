//! Variable/config lookup substitution types
//!
//! LaunchConfiguration, EnvironmentVariable, Parameter,
//! BooleanSubstitution, FindExecutable

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use pyo3::prelude::*;

use crate::python::api::utils as sub_utils;

use super::resolve_substitution_string;

/// Mock LaunchConfiguration substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import LaunchConfiguration
/// config = LaunchConfiguration('variable_name')
/// ```
///
/// When converted to string, returns substitution format: `$(var variable_name)`
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct LaunchConfiguration {
    variable_name: String,
    #[allow(dead_code)] // Store for API compatibility
    default: Option<String>,
}

#[pymethods]
impl LaunchConfiguration {
    #[new]
    #[pyo3(signature = (variable_name, *, default=None, **_kwargs))]
    fn new(
        variable_name: String,
        default: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        // Accept str, bool, or any type — convert to string for storage.
        // ROS 2 accepts SomeSubstitutionsType which includes bool, list, etc.
        let default = default.map(|obj| {
            Python::attach(|py| {
                obj.bind(py)
                    .str()
                    .map_or_else(|_| format!("{:?}", obj), |s| s.to_string())
            })
        });

        // Register default in LaunchContext so it's available for variable substitution.
        // Only set if not already present (CLI args and DeclareLaunchArgument take precedence).
        if let Some(ref default_val) = default {
            use crate::bridge::with_launch_context;
            with_launch_context(|ctx| {
                if ctx.get_configuration(&variable_name).is_none() {
                    ctx.set_configuration(variable_name.clone(), default_val.clone());
                }
            });
        }

        Self {
            variable_name,
            default,
        }
    }

    fn __str__(&self) -> String {
        format!("$(var {})", self.variable_name)
    }

    fn __repr__(&self) -> String {
        format!("LaunchConfiguration('{}')", self.variable_name)
    }

    /// Perform the substitution - extract the actual value from launch configurations
    ///
    /// In ROS 2, this method is called with a launch context to resolve the value.
    /// Uses the thread-local LaunchContext which already has all configurations
    /// with proper scope chain resolution.
    fn perform(&self, _context: &Bound<'_, PyAny>) -> PyResult<String> {
        use crate::bridge::with_launch_context;

        // Get value from LaunchContext (already resolves nested substitutions)
        let result = with_launch_context(|ctx| {
            let self_ref = format!("$(var {})", self.variable_name);
            if let Some(value) = ctx.get_configuration(&self.variable_name) {
                // If the context value is a circular self-reference like $(var same_name),
                // fall through to the default instead of returning the unresolved substitution.
                if value == self_ref {
                    if let Some(ref default) = self.default {
                        resolve_substitution_string(default, ctx)
                            .unwrap_or_else(|_| default.clone())
                    } else {
                        value
                    }
                } else {
                    value
                }
            } else if let Some(ref default) = self.default {
                // Resolve nested substitutions in the default value
                resolve_substitution_string(default, ctx).unwrap_or_else(|_| default.clone())
            } else {
                // Return empty string if not found (ROS 2 behavior)
                String::new()
            }
        });

        log::debug!(
            "LaunchConfiguration('{}').perform() -> '{}'",
            self.variable_name,
            result
        );

        Ok(result)
    }
}

/// Mock EnvironmentVariable substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import EnvironmentVariable
/// env_var = EnvironmentVariable('VAR_NAME')
/// # or with default
/// env_var = EnvironmentVariable('VAR_NAME', default_value='default')
/// ```
///
/// Returns substitution format: `$(env VAR_NAME)` or `$(optenv VAR_NAME default)`
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct EnvironmentVariable {
    name: String,
    default_value: Option<String>,
}

#[pymethods]
impl EnvironmentVariable {
    #[new]
    #[pyo3(signature = (name, *, default_value=None))]
    fn new(name: String, default_value: Option<String>) -> Self {
        Self {
            name,
            default_value,
        }
    }

    fn __str__(&self) -> String {
        if let Some(default) = &self.default_value {
            format!("$(optenv {} {})", self.name, default)
        } else {
            format!("$(env {})", self.name)
        }
    }

    fn __repr__(&self) -> String {
        format!("EnvironmentVariable('{}')", self.name)
    }
}

/// Mock Parameter substitution
///
/// Python equivalent:
/// ```python
/// from launch_ros.substitutions import Parameter
/// param_value = Parameter('my_parameter')
/// ```
///
/// Reads a ROS parameter value and returns it as a string
#[pyclass(module = "launch_ros.substitutions", from_py_object)]
#[derive(Clone)]
pub struct Parameter {
    name: Py<PyAny>,
}

#[pymethods]
impl Parameter {
    #[new]
    fn new(name: Py<PyAny>) -> Self {
        Self { name }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let name_str = sub_utils::pyobject_to_string(py, &self.name)?;
        Ok(format!("$(param {})", name_str))
    }

    fn __repr__(&self, py: Python) -> String {
        let name_str =
            sub_utils::pyobject_to_string(py, &self.name).unwrap_or_else(|_| "<name>".to_string());
        format!("Parameter('{}')", name_str)
    }

    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let name_str = sub_utils::perform_or_to_string(&self.name, py, context)?;
        Ok(format!("$(param {})", name_str))
    }
}

/// Mock BooleanSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import BooleanSubstitution
/// bool_val = BooleanSubstitution('true')
/// ```
///
/// Converts a value to a boolean string representation ("true" or "false")
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct BooleanSubstitution {
    value: Py<PyAny>,
}

#[pymethods]
impl BooleanSubstitution {
    #[new]
    fn new(value: Py<PyAny>) -> Self {
        Self { value }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let val_str = sub_utils::pyobject_to_string(py, &self.value)?;
        Ok(Self::to_boolean_string(&val_str))
    }

    fn __repr__(&self, py: Python) -> String {
        let val_str = sub_utils::pyobject_to_string(py, &self.value)
            .unwrap_or_else(|_| "<value>".to_string());
        format!("BooleanSubstitution('{}')", val_str)
    }

    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let val_str = sub_utils::perform_or_to_string(&self.value, py, context)?;
        Ok(Self::to_boolean_string(&val_str))
    }
}

impl BooleanSubstitution {
    fn to_boolean_string(s: &str) -> String {
        let lower = s.to_lowercase();
        match lower.as_str() {
            "true" | "1" | "yes" | "on" => "true".to_string(),
            "false" | "0" | "no" | "off" | "" => "false".to_string(),
            _ => "true".to_string(), // Non-empty strings are truthy
        }
    }
}

/// Mock FindExecutable substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import FindExecutable
/// exec_path = FindExecutable('python3')
/// ```
///
/// Searches PATH for an executable
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct FindExecutable {
    name: Py<PyAny>,
}

#[pymethods]
impl FindExecutable {
    #[new]
    fn new(name: Py<PyAny>) -> Self {
        Self { name }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let name_str = sub_utils::pyobject_to_string(py, &self.name)?;
        // Return placeholder - can't actually search PATH in static analysis
        Ok(format!("$(find-executable {})", name_str))
    }

    fn __repr__(&self, py: Python) -> String {
        let name_str =
            sub_utils::pyobject_to_string(py, &self.name).unwrap_or_else(|_| "<name>".to_string());
        format!("FindExecutable('{}')", name_str)
    }

    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let name_str = sub_utils::perform_or_to_string(&self.name, py, context)?;
        Ok(format!("$(find-executable {})", name_str))
    }
}
