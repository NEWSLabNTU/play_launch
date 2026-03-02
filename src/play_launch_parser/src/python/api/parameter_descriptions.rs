//! Mock launch_ros.parameter_descriptions module
//!
//! Provides wrapper classes for parameter files and values.

#![allow(non_local_definitions)]

use pyo3::{prelude::*, types::PyDict};

/// Mock ParameterFile class
///
/// Python equivalent:
/// ```python
/// from launch_ros.parameter_descriptions import ParameterFile
///
/// param_file = ParameterFile('/path/to/params.yaml')
/// # Or with substitutions:
/// param_file = ParameterFile(LaunchConfiguration('config_file'))
/// ```
///
/// Wraps a parameter file path (which can be a substitution)
#[pyclass(module = "launch_ros.parameter_descriptions")]
#[derive(Clone)]
pub struct ParameterFile {
    param_file: PyObject,
    #[allow(dead_code)] // For API compatibility
    allow_substs: bool,
}

#[pymethods]
impl ParameterFile {
    #[new]
    #[pyo3(signature = (param_file, *, allow_substs=false, **_kwargs))]
    fn new(
        param_file: PyObject,
        allow_substs: Option<bool>,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<Self> {
        Ok(Self {
            param_file,
            allow_substs: allow_substs.unwrap_or(false),
        })
    }

    fn __repr__(&self, py: Python) -> String {
        let param_file_str = self.param_file.as_ref(py).to_string();
        format!("ParameterFile({})", param_file_str)
    }

    /// Get the parameter file path (converts to string if possible)
    ///
    /// Uses pyobject_to_string() to resolve substitutions (e.g., LaunchConfiguration)
    /// from the thread-local context, returning the actual file path instead of
    /// unresolved substitution syntax like "$(var config)".
    fn __str__(&self, py: Python) -> PyResult<String> {
        crate::python::api::utils::pyobject_to_string(py, &self.param_file)
    }
}

/// Mock ParameterValue class
///
/// Python equivalent:
/// ```python
/// from launch_ros.parameter_descriptions import ParameterValue
///
/// # Simple value
/// param = ParameterValue(42)
/// # With type specification
/// param = ParameterValue('42', value_type=int)
/// # With substitutions
/// param = ParameterValue(LaunchConfiguration('my_param'))
/// ```
///
/// Wraps a parameter value with optional type information
#[pyclass(module = "launch_ros.parameter_descriptions")]
#[derive(Clone)]
pub struct ParameterValue {
    value: PyObject,
    #[allow(dead_code)] // For API compatibility
    value_type: Option<PyObject>,
}

#[pymethods]
impl ParameterValue {
    #[new]
    #[pyo3(signature = (value, *, value_type=None, **_kwargs))]
    fn new(
        value: PyObject,
        value_type: Option<PyObject>,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<Self> {
        Ok(Self { value, value_type })
    }

    fn __repr__(&self, py: Python) -> String {
        let value_str = self.value.as_ref(py).to_string();

        if let Some(ref vtype) = self.value_type {
            let type_str = vtype.as_ref(py).to_string();
            format!("ParameterValue({}, value_type={})", value_str, type_str)
        } else {
            format!("ParameterValue({})", value_str)
        }
    }

    /// Get the parameter value as a string
    fn __str__(&self, py: Python) -> PyResult<String> {
        let obj_ref = self.value.as_ref(py);

        // Try direct string extraction
        if let Ok(s) = obj_ref.extract::<String>() {
            return Ok(s);
        }

        // Try integer
        if let Ok(i) = obj_ref.extract::<i64>() {
            return Ok(i.to_string());
        }

        // Try float
        if let Ok(f) = obj_ref.extract::<f64>() {
            return Ok(f.to_string());
        }

        // Try boolean
        if let Ok(b) = obj_ref.extract::<bool>() {
            return Ok(if b { "true" } else { "false" }.to_string());
        }

        // Try calling __str__ on the object (for substitutions)
        if let Ok(str_result) = obj_ref.call_method0("__str__")
            && let Ok(s) = str_result.extract::<String>()
        {
            return Ok(s);
        }

        // Fallback
        Ok(obj_ref.to_string())
    }
}
