//! Boolean/conditional substitution types
//!
//! NotSubstitution, AndSubstitution, OrSubstitution,
//! EqualsSubstitution, IfElseSubstitution, NotEqualsSubstitution

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use pyo3::prelude::*;

use crate::python::api::utils as sub_utils;

/// Mock NotSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import NotSubstitution
/// not_sub = NotSubstitution(some_condition)
/// ```
///
/// Returns the boolean NOT of the input
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct NotSubstitution {
    condition: Py<PyAny>,
}

#[pymethods]
impl NotSubstitution {
    #[new]
    fn new(condition: Py<PyAny>) -> Self {
        Self { condition }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        // Try to evaluate the condition
        if let Ok(s) = self.condition.extract::<String>(py) {
            let is_true = matches!(s.to_lowercase().as_str(), "true" | "1" | "yes");
            return Ok(if is_true { "false" } else { "true" }.to_string());
        }

        if let Ok(b) = self.condition.extract::<bool>(py) {
            return Ok(if b { "false" } else { "true" }.to_string());
        }

        // Fallback: call __str__ and negate
        if let Ok(str_result) = self.condition.call_method0(py, "__str__")
            && let Ok(s) = str_result.extract::<String>(py)
        {
            let is_true = matches!(s.to_lowercase().as_str(), "true" | "1" | "yes");
            return Ok(if is_true { "false" } else { "true" }.to_string());
        }

        Ok("true".to_string())
    }

    fn __repr__(&self) -> String {
        "NotSubstitution(...)".to_string()
    }
}

/// Mock AndSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import AndSubstitution
/// and_sub = AndSubstitution(left, right)
/// ```
///
/// Returns the boolean AND of two inputs
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct AndSubstitution {
    left: Py<PyAny>,
    right: Py<PyAny>,
}

#[pymethods]
impl AndSubstitution {
    #[new]
    fn new(left: Py<PyAny>, right: Py<PyAny>) -> Self {
        Self { left, right }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let left_val = sub_utils::pyobject_to_bool(&self.left, py)?;
        let right_val = sub_utils::pyobject_to_bool(&self.right, py)?;
        Ok(if left_val && right_val {
            "true"
        } else {
            "false"
        }
        .to_string())
    }

    fn __repr__(&self) -> String {
        "AndSubstitution(...)".to_string()
    }
}

/// Mock OrSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import OrSubstitution
/// or_sub = OrSubstitution(left, right)
/// ```
///
/// Returns the boolean OR of two inputs
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct OrSubstitution {
    left: Py<PyAny>,
    right: Py<PyAny>,
}

#[pymethods]
impl OrSubstitution {
    #[new]
    fn new(left: Py<PyAny>, right: Py<PyAny>) -> Self {
        Self { left, right }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let left_val = sub_utils::pyobject_to_bool(&self.left, py)?;
        let right_val = sub_utils::pyobject_to_bool(&self.right, py)?;
        Ok(if left_val || right_val {
            "true"
        } else {
            "false"
        }
        .to_string())
    }

    fn __repr__(&self) -> String {
        "OrSubstitution(...)".to_string()
    }
}

/// Mock EqualsSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import EqualsSubstitution
/// equals_sub = EqualsSubstitution(left, right)
/// ```
///
/// Returns true if two values are equal
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct EqualsSubstitution {
    left: Py<PyAny>,
    right: Py<PyAny>,
}

#[pymethods]
impl EqualsSubstitution {
    #[new]
    fn new(left: Py<PyAny>, right: Py<PyAny>) -> Self {
        Self { left, right }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let left_str = sub_utils::pyobject_to_string(py, &self.left)?;
        let right_str = sub_utils::pyobject_to_string(py, &self.right)?;
        Ok(if left_str == right_str {
            "true"
        } else {
            "false"
        }
        .to_string())
    }

    fn __repr__(&self) -> String {
        "EqualsSubstitution(...)".to_string()
    }

    /// Perform the substitution - evaluate both sides and compare
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let left_str = sub_utils::perform_or_to_string(&self.left, py, context)?;
        let right_str = sub_utils::perform_or_to_string(&self.right, py, context)?;
        Ok(if left_str == right_str {
            "true"
        } else {
            "false"
        }
        .to_string())
    }
}

/// Mock IfElseSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import IfElseSubstitution
/// ifelse_sub = IfElseSubstitution(condition, if_value, else_value)
/// ```
///
/// Returns if_value if condition is true, else returns else_value
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct IfElseSubstitution {
    condition: Py<PyAny>,
    if_value: Py<PyAny>,
    else_value: Py<PyAny>,
}

#[pymethods]
impl IfElseSubstitution {
    #[new]
    fn new(condition: Py<PyAny>, if_value: Py<PyAny>, else_value: Py<PyAny>) -> Self {
        Self {
            condition,
            if_value,
            else_value,
        }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let cond_val = sub_utils::pyobject_to_bool(&self.condition, py)?;
        let obj = if cond_val {
            &self.if_value
        } else {
            &self.else_value
        };

        sub_utils::pyobject_to_string(py, obj)
    }

    fn __repr__(&self) -> String {
        "IfElseSubstitution(...)".to_string()
    }

    /// Perform the substitution - evaluate condition and return appropriate value
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let cond_str = sub_utils::perform_or_to_string(&self.condition, py, context)?;
        let cond_val = matches!(cond_str.to_lowercase().as_str(), "true" | "1" | "yes");
        let obj = if cond_val {
            &self.if_value
        } else {
            &self.else_value
        };

        sub_utils::perform_or_to_string(obj, py, context)
    }
}

/// Mock NotEqualsSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import NotEqualsSubstitution
/// not_equals_sub = NotEqualsSubstitution(left, right)
/// ```
///
/// Returns true if two values are NOT equal
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct NotEqualsSubstitution {
    left: Py<PyAny>,
    right: Py<PyAny>,
}

#[pymethods]
impl NotEqualsSubstitution {
    #[new]
    fn new(left: Py<PyAny>, right: Py<PyAny>) -> Self {
        Self { left, right }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        let left_str = sub_utils::pyobject_to_string(py, &self.left)?;
        let right_str = sub_utils::pyobject_to_string(py, &self.right)?;
        Ok(if left_str != right_str {
            "true"
        } else {
            "false"
        }
        .to_string())
    }

    fn __repr__(&self) -> String {
        "NotEqualsSubstitution(...)".to_string()
    }

    /// Perform the substitution - evaluate both sides and compare
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let left_str = sub_utils::perform_or_to_string(&self.left, py, context)?;
        let right_str = sub_utils::perform_or_to_string(&self.right, py, context)?;
        Ok(if left_str != right_str {
            "true"
        } else {
            "false"
        }
        .to_string())
    }
}
