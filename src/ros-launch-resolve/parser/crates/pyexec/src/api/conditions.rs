//! Mock `launch.conditions`.
//!
//! Conditions are evaluated when the action carrying them EXECUTES (see
//! `api::visit`), against the launch context as it is at that point, and with
//! `launch`'s own rule for a predicate string: `evaluate_condition_expression`
//! accepts `true`/`1` and `false`/`0`, case-insensitively, and raises for
//! anything else. The mock used to read `yes`/`on` as true and everything
//! else — a typo included — as false.

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use play_launch_parser::bridge::with_launch_context;
use pyo3::{
    exceptions::PyValueError,
    prelude::*,
    types::{PyDict, PyTuple},
};

/// Evaluate any condition object the way `launch` does: `condition.evaluate(context)`.
pub(crate) fn evaluate(py: Python, condition: &Bound<'_, PyAny>) -> PyResult<bool> {
    let context = crate::api::utils::create_launch_context(py)?;
    condition
        .call_method1("evaluate", (context,))?
        .extract::<bool>()
}

/// `launch.conditions.evaluate_condition_expression`.
fn condition_expression(py: Python, predicate: &Py<PyAny>) -> PyResult<bool> {
    let value = crate::api::utils::pyobject_to_string(py, predicate)?;
    match value.trim().to_lowercase().as_str() {
        "true" | "1" => Ok(true),
        "false" | "0" => Ok(false),
        _ => Err(PyValueError::new_err(format!(
            "invalid condition expression: Unexpected value '{value}', expected one of: \
             ['true', '1', 'false', '0']"
        ))),
    }
}

#[pyclass(module = "launch.conditions", from_py_object)]
pub struct IfCondition {
    predicate: Py<PyAny>,
}

impl Clone for IfCondition {
    fn clone(&self) -> Self {
        Python::attach(|py| Self {
            predicate: self.predicate.clone_ref(py),
        })
    }
}

#[pymethods]
impl IfCondition {
    #[new]
    #[pyo3(signature = (predicate_expression = None, *, predicate = None))]
    fn new(
        predicate_expression: Option<Py<PyAny>>,
        predicate: Option<Py<PyAny>>,
    ) -> PyResult<Self> {
        let predicate = predicate_expression
            .or(predicate)
            .ok_or_else(|| PyValueError::new_err("IfCondition needs a predicate expression"))?;
        Ok(Self { predicate })
    }

    #[pyo3(signature = (*_args, **_kwargs))]
    pub fn evaluate(
        &self,
        py: Python,
        _args: &Bound<'_, PyTuple>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<bool> {
        condition_expression(py, &self.predicate)
    }

    fn __repr__(&self) -> String {
        "IfCondition(...)".to_string()
    }
}

#[pyclass(module = "launch.conditions", from_py_object)]
pub struct UnlessCondition {
    predicate: Py<PyAny>,
}

impl Clone for UnlessCondition {
    fn clone(&self) -> Self {
        Python::attach(|py| Self {
            predicate: self.predicate.clone_ref(py),
        })
    }
}

#[pymethods]
impl UnlessCondition {
    #[new]
    #[pyo3(signature = (predicate_expression = None, *, predicate = None))]
    fn new(
        predicate_expression: Option<Py<PyAny>>,
        predicate: Option<Py<PyAny>>,
    ) -> PyResult<Self> {
        let predicate = predicate_expression
            .or(predicate)
            .ok_or_else(|| PyValueError::new_err("UnlessCondition needs a predicate expression"))?;
        Ok(Self { predicate })
    }

    #[pyo3(signature = (*_args, **_kwargs))]
    pub fn evaluate(
        &self,
        py: Python,
        _args: &Bound<'_, PyTuple>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<bool> {
        Ok(!condition_expression(py, &self.predicate)?)
    }

    fn __repr__(&self) -> String {
        "UnlessCondition(...)".to_string()
    }
}

/// The configuration's value, `None` when it is unset — what
/// `LaunchConfigurationEquals` compares.
fn configuration(name: &str) -> Option<String> {
    with_launch_context(|ctx| ctx.get_configuration(name))
}

fn expected(py: Python, expected: &Option<Py<PyAny>>) -> PyResult<Option<String>> {
    expected
        .as_ref()
        .map(|e| crate::api::utils::pyobject_to_string(py, e))
        .transpose()
}

#[pyclass(module = "launch.conditions", from_py_object)]
pub struct LaunchConfigurationEquals {
    variable_name: String,
    expected_value: Option<Py<PyAny>>,
}

impl Clone for LaunchConfigurationEquals {
    fn clone(&self) -> Self {
        Python::attach(|py| Self {
            variable_name: self.variable_name.clone(),
            expected_value: self.expected_value.as_ref().map(|e| e.clone_ref(py)),
        })
    }
}

#[pymethods]
impl LaunchConfigurationEquals {
    #[new]
    #[pyo3(signature = (launch_configuration_name, expected_value = None))]
    fn new(launch_configuration_name: String, expected_value: Option<Py<PyAny>>) -> Self {
        Self {
            variable_name: launch_configuration_name,
            expected_value,
        }
    }

    #[pyo3(signature = (*_args, **_kwargs))]
    pub fn evaluate(
        &self,
        py: Python,
        _args: &Bound<'_, PyTuple>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<bool> {
        Ok(configuration(&self.variable_name) == expected(py, &self.expected_value)?)
    }

    fn __repr__(&self) -> String {
        format!("LaunchConfigurationEquals('{}')", self.variable_name)
    }
}

#[pyclass(module = "launch.conditions", from_py_object)]
pub struct LaunchConfigurationNotEquals {
    variable_name: String,
    expected_value: Option<Py<PyAny>>,
}

impl Clone for LaunchConfigurationNotEquals {
    fn clone(&self) -> Self {
        Python::attach(|py| Self {
            variable_name: self.variable_name.clone(),
            expected_value: self.expected_value.as_ref().map(|e| e.clone_ref(py)),
        })
    }
}

#[pymethods]
impl LaunchConfigurationNotEquals {
    #[new]
    #[pyo3(signature = (launch_configuration_name, expected_value = None))]
    fn new(launch_configuration_name: String, expected_value: Option<Py<PyAny>>) -> Self {
        Self {
            variable_name: launch_configuration_name,
            expected_value,
        }
    }

    #[pyo3(signature = (*_args, **_kwargs))]
    pub fn evaluate(
        &self,
        py: Python,
        _args: &Bound<'_, PyTuple>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<bool> {
        Ok(configuration(&self.variable_name) != expected(py, &self.expected_value)?)
    }

    fn __repr__(&self) -> String {
        format!("LaunchConfigurationNotEquals('{}')", self.variable_name)
    }
}
