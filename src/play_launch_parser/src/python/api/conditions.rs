//! Mock `launch.conditions` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use crate::python::bridge::with_launch_context;
use pyo3::{
    prelude::*,
    types::{PyDict, PyTuple},
};

/// Mock IfCondition class
///
/// Python equivalent:
/// ```python
/// from launch.conditions import IfCondition
/// from launch.substitutions import LaunchConfiguration
///
/// condition = IfCondition(LaunchConfiguration('use_sim'))
/// ```
///
/// Evaluates to true if the predicate evaluates to a truthy value
#[pyclass(module = "launch.conditions")]
#[derive(Clone)]
pub struct IfCondition {
    predicate: PyObject,
}

#[pymethods]
impl IfCondition {
    #[new]
    fn new(predicate: PyObject) -> Self {
        Self { predicate }
    }

    /// Evaluate the condition
    ///
    /// Called by Python to check if the condition is true.
    /// The real ROS 2 IfCondition.evaluate(context) accepts a LaunchContext,
    /// but we resolve from thread-local state so we just ignore it.
    #[pyo3(signature = (*_args, **_kwargs))]
    pub fn evaluate(
        &self,
        py: Python,
        _args: &PyTuple,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<bool> {
        // Convert predicate to string and evaluate as boolean
        let value = self.predicate_to_string(py)?;
        Ok(Self::is_truthy(&value))
    }

    fn __repr__(&self) -> String {
        "IfCondition(...)".to_string()
    }
}

impl IfCondition {
    /// Convert predicate PyObject to string
    fn predicate_to_string(&self, py: Python) -> PyResult<String> {
        let pred_ref = self.predicate.as_ref(py);

        // Try direct string extraction
        if let Ok(s) = pred_ref.extract::<String>() {
            return Ok(s);
        }

        // Try calling perform() method first (for LaunchConfiguration substitutions)
        // This resolves the substitution to its actual value
        if let Ok(has_perform) = pred_ref.hasattr("perform")
            && has_perform
        {
            // Create a dummy context (not used by our LaunchConfiguration.perform())
            if let Ok(context) = py.eval("type('Context', (), {})()", None, None)
                && let Ok(result) = pred_ref.call_method1("perform", (context,))
                && let Ok(s) = result.extract::<String>()
            {
                return Ok(s);
            }
        }

        // Try calling __str__ method (for other substitutions)
        if let Ok(str_result) = pred_ref.call_method0("__str__")
            && let Ok(s) = str_result.extract::<String>()
        {
            return Ok(s);
        }

        // Fallback to repr
        Ok(pred_ref.to_string())
    }

    /// Check if a string value is truthy
    ///
    /// Truthy values: "true", "True", "1", "yes", "Yes", "on", "On"
    /// Falsy values: "false", "False", "0", "no", "No", "off", "Off", ""
    /// Substitutions: "$(var name)" are resolved from LaunchContext before evaluation
    fn is_truthy(value: &str) -> bool {
        // Resolve LaunchConfiguration substitutions
        let resolved_value = if value.starts_with("$(var ") && value.ends_with(')') {
            // Extract variable name from "$(var variable_name)"
            let var_name = &value[6..value.len() - 1]; // Skip "$(var " and ")"

            // Look up in LaunchContext via thread-local
            let resolved = with_launch_context(|ctx| ctx.get_configuration(var_name));
            if let Some(val) = resolved {
                val
            } else {
                // Variable not found - treat as empty/falsy
                log::warn!(
                    "LaunchConfiguration '{}' not found in launch args",
                    var_name
                );
                return false;
            }
        } else {
            value.to_string()
        };

        let lower = resolved_value.to_lowercase();
        matches!(
            lower.as_str(),
            "true" | "1" | "yes" | "on" if !lower.is_empty()
        )
    }
}

/// Mock UnlessCondition class
///
/// Python equivalent:
/// ```python
/// from launch.conditions import UnlessCondition
/// from launch.substitutions import LaunchConfiguration
///
/// condition = UnlessCondition(LaunchConfiguration('use_sim'))
/// ```
///
/// Evaluates to true if the predicate evaluates to a falsy value (inverted IfCondition)
#[pyclass(module = "launch.conditions")]
#[derive(Clone)]
pub struct UnlessCondition {
    predicate: PyObject,
}

#[pymethods]
impl UnlessCondition {
    #[new]
    fn new(predicate: PyObject) -> Self {
        Self { predicate }
    }

    /// Evaluate the condition (inverted)
    #[pyo3(signature = (*_args, **_kwargs))]
    pub fn evaluate(
        &self,
        py: Python,
        _args: &PyTuple,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<bool> {
        // Convert predicate to string and evaluate as boolean (inverted)
        let value = self.predicate_to_string(py)?;
        Ok(!IfCondition::is_truthy(&value))
    }

    fn __repr__(&self) -> String {
        "UnlessCondition(...)".to_string()
    }
}

impl UnlessCondition {
    /// Convert predicate PyObject to string (same logic as IfCondition)
    fn predicate_to_string(&self, py: Python) -> PyResult<String> {
        let pred_ref = self.predicate.as_ref(py);

        // Try direct string extraction
        if let Ok(s) = pred_ref.extract::<String>() {
            return Ok(s);
        }

        // Try calling perform() method first (for LaunchConfiguration substitutions)
        // This resolves the substitution to its actual value
        if let Ok(has_perform) = pred_ref.hasattr("perform")
            && has_perform
        {
            // Create a dummy context (not used by our LaunchConfiguration.perform())
            if let Ok(context) = py.eval("type('Context', (), {})()", None, None)
                && let Ok(result) = pred_ref.call_method1("perform", (context,))
                && let Ok(s) = result.extract::<String>()
            {
                return Ok(s);
            }
        }

        // Try calling __str__ method (for other substitutions)
        if let Ok(str_result) = pred_ref.call_method0("__str__")
            && let Ok(s) = str_result.extract::<String>()
        {
            return Ok(s);
        }

        Ok(pred_ref.to_string())
    }
}

/// Mock LaunchConfigurationEquals condition
///
/// Python equivalent:
/// ```python
/// from launch.conditions import LaunchConfigurationEquals
///
/// condition = LaunchConfigurationEquals('variable_name', 'expected_value')
/// ```
#[pyclass(module = "launch.conditions")]
#[derive(Clone)]
pub struct LaunchConfigurationEquals {
    variable_name: String,
    expected_value: String,
}

#[pymethods]
impl LaunchConfigurationEquals {
    #[new]
    fn new(variable_name: String, expected_value: String) -> Self {
        Self {
            variable_name,
            expected_value,
        }
    }

    #[pyo3(signature = (*_args, **_kwargs))]
    pub fn evaluate(
        &self,
        _py: Python,
        _args: &PyTuple,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<bool> {
        let actual = with_launch_context(|ctx| ctx.get_configuration(&self.variable_name));
        Ok(actual.as_deref() == Some(self.expected_value.as_str()))
    }

    fn __repr__(&self) -> String {
        format!(
            "LaunchConfigurationEquals('{}', '{}')",
            self.variable_name, self.expected_value
        )
    }
}

/// Mock LaunchConfigurationNotEquals condition
///
/// Python equivalent:
/// ```python
/// from launch.conditions import LaunchConfigurationNotEquals
///
/// condition = LaunchConfigurationNotEquals('variable_name', 'expected_value')
/// ```
#[pyclass(module = "launch.conditions")]
#[derive(Clone)]
pub struct LaunchConfigurationNotEquals {
    variable_name: String,
    expected_value: String,
}

#[pymethods]
impl LaunchConfigurationNotEquals {
    #[new]
    fn new(variable_name: String, expected_value: String) -> Self {
        Self {
            variable_name,
            expected_value,
        }
    }

    #[pyo3(signature = (*_args, **_kwargs))]
    pub fn evaluate(
        &self,
        _py: Python,
        _args: &PyTuple,
        _kwargs: Option<&PyDict>,
    ) -> PyResult<bool> {
        let actual = with_launch_context(|ctx| ctx.get_configuration(&self.variable_name));
        Ok(actual.as_deref() != Some(self.expected_value.as_str()))
    }

    fn __repr__(&self) -> String {
        format!(
            "LaunchConfigurationNotEquals('{}', '{}')",
            self.variable_name, self.expected_value
        )
    }
}
