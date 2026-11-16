//! Boolean/conditional substitution types
//!
//! NotSubstitution, AndSubstitution, OrSubstitution,
//! EqualsSubstitution, IfElseSubstitution, NotEqualsSubstitution

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use pyo3::prelude::*;

use crate::api::utils as sub_utils;

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

    /// `launch`'s: `'true'`/`'false'`, the operand read as a boolean
    /// (`true`/`false`/`1`/`0`), anything else `SubstitutionFailure`.
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let value = sub_utils::perform_or_to_string(&self.condition, py, context)?;
        sub_utils::resolve_call("not", vec![value])
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

    /// `launch`'s: both operands read as booleans, `SubstitutionFailure`
    /// for one that is not.
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let left = sub_utils::perform_or_to_string(&self.left, py, context)?;
        let right = sub_utils::perform_or_to_string(&self.right, py, context)?;
        sub_utils::resolve_call("and", vec![left, right])
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

    /// `launch`'s: both operands read as booleans, `SubstitutionFailure`
    /// for one that is not.
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let left = sub_utils::perform_or_to_string(&self.left, py, context)?;
        let right = sub_utils::perform_or_to_string(&self.right, py, context)?;
        sub_utils::resolve_call("or", vec![left, right])
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

    /// `launch`'s comparison: booleans and numbers by value, anything else
    /// as text (the engine's `$(equals)`).
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let left_str = sub_utils::perform_or_to_string(&self.left, py, context)?;
        let right_str = sub_utils::perform_or_to_string(&self.right, py, context)?;
        sub_utils::resolve_call("equals", vec![left_str, right_str])
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

    /// `launch`'s: the condition read as a boolean (`SubstitutionFailure`
    /// for one that is not), then only the chosen branch performed.
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let cond_str = sub_utils::perform_or_to_string(&self.condition, py, context)?;
        let chosen = sub_utils::resolve_call("if", vec![cond_str, "1".into(), "0".into()])?;
        let obj = if chosen == "1" {
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

    /// `launch`'s comparison: booleans and numbers by value, anything else
    /// as text (the engine's `$(not-equals)`).
    fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
        let left_str = sub_utils::perform_or_to_string(&self.left, py, context)?;
        let right_str = sub_utils::perform_or_to_string(&self.right, py, context)?;
        sub_utils::resolve_call("not-equals", vec![left_str, right_str])
    }
}

/// Mock AnySubstitution / AllSubstitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import AnySubstitution, AllSubstitution
/// AnySubstitution(LaunchConfiguration('a'), 'false')
/// ```
macro_rules! variadic_boolean {
    ($name:ident, $op:literal) => {
        #[pyclass(module = "launch.substitutions", from_py_object)]
        pub struct $name {
            args: Vec<Py<PyAny>>,
        }

        impl Clone for $name {
            fn clone(&self) -> Self {
                Python::attach(|py| Self {
                    args: self.args.iter().map(|a| a.clone_ref(py)).collect(),
                })
            }
        }

        #[pymethods]
        impl $name {
            #[new]
            #[pyo3(signature = (*args, container=None))]
            fn new(
                args: &Bound<'_, pyo3::types::PyTuple>,
                container: Option<&Bound<'_, PyAny>>,
            ) -> PyResult<Self> {
                let mut all: Vec<Py<PyAny>> = args.iter().map(|a| a.unbind()).collect();
                if let Some(c) = container.filter(|c| !c.is_none()) {
                    for item in c.try_iter()? {
                        all.push(item?.unbind());
                    }
                }
                Ok(Self { args: all })
            }

            fn __str__(&self, py: Python) -> PyResult<String> {
                let context = sub_utils::create_launch_context(py)?;
                self.perform(py, context.bind(py))
            }

            fn __repr__(&self) -> String {
                concat!(stringify!($name), "(...)").to_string()
            }

            /// `launch`'s: every operand read as a boolean.
            fn perform(&self, py: Python, context: &Bound<'_, PyAny>) -> PyResult<String> {
                let mut values = Vec::new();
                for arg in &self.args {
                    values.push(sub_utils::perform_or_to_string(arg, py, context)?);
                }
                sub_utils::resolve_call($op, values)
            }
        }
    };
}

variadic_boolean!(AnySubstitution, "any");
variadic_boolean!(AllSubstitution, "all");
