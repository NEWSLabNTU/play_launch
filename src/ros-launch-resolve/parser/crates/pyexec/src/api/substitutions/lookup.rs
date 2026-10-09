//! Variable/config lookup substitution types
//!
//! LaunchConfiguration, EnvironmentVariable, Parameter,
//! BooleanSubstitution, FindExecutable

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use pyo3::prelude::*;

use crate::api::utils as sub_utils;

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
    #[pyo3(get)]
    variable_name: String,
    /// Used when the configuration is unset. Unlike the mock this replaces,
    /// it does NOT set the configuration — in `launch` a default belongs to
    /// this substitution only.
    #[pyo3(get)]
    default: Option<Py<PyAny>>,
}

#[pymethods]
impl LaunchConfiguration {
    #[new]
    #[pyo3(signature = (variable_name, *, default=None, **_kwargs))]
    fn new(
        py: Python,
        variable_name: Py<PyAny>,
        default: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> PyResult<Self> {
        Ok(Self {
            variable_name: sub_utils::pyobject_to_string(py, &variable_name)?,
            default,
        })
    }

    fn __str__(&self) -> String {
        format!("$(var {})", self.variable_name)
    }

    fn __repr__(&self) -> String {
        format!("LaunchConfiguration('{}')", self.variable_name)
    }

    /// The value now, else the default; unset with no default is
    /// `launch`'s `SubstitutionFailure`.
    fn perform(&self, py: Python, _context: &Bound<'_, PyAny>) -> PyResult<String> {
        use play_launch_parser::bridge::with_launch_context;
        if let Some(value) = with_launch_context(|ctx| ctx.get_configuration(&self.variable_name)) {
            return Ok(value);
        }
        if let Some(default) = &self.default {
            return sub_utils::pyobject_to_string(py, default);
        }
        Err(pyo3::exceptions::PyRuntimeError::new_err(format!(
            "launch configuration '{}' does not exist",
            self.variable_name
        )))
    }
}

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
