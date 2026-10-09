//! Mock `launch.substitutions` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

mod conditional;
mod lookup;
mod package;

// Re-export all public types so `substitutions::LaunchConfiguration` etc. still work
pub use conditional::{
    AndSubstitution, EqualsSubstitution, IfElseSubstitution, NotEqualsSubstitution,
    NotSubstitution, OrSubstitution,
};
pub use lookup::{
    BooleanSubstitution, EnvironmentVariable, FindExecutable, LaunchConfiguration, Parameter,
};
pub use package::{
    AnonName, ExecutableInPackage, FileContent, FindPackage, FindPackageShare, LaunchLogDir,
    PathJoinSubstitution, TextSubstitution, ThisLaunchFile, ThisLaunchFileDir,
};

use pyo3::prelude::*;

use crate::api::utils as sub_utils;

// ============================================================================
// Helper Functions for Context Management
// ============================================================================

// ============================================================================
// Mock Substitution Classes (kept in mod.rs)
// ============================================================================

/// Mock PythonExpression substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import PythonExpression
/// expr = PythonExpression(["'value1' if condition else 'value2'"])
/// expr = PythonExpression(["'true' if '", LaunchConfiguration('mode'), "' == 'realtime' else 'false'"])
/// ```
///
/// Note: Limited support - we concatenate and return the expression as-is
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct PythonExpression {
    expression: String,
}

#[pymethods]
impl PythonExpression {
    #[new]
    fn new(py: Python, expression: Vec<Py<PyAny>>) -> PyResult<Self> {
        // Convert each element to string and concatenate
        let mut result = String::new();
        for obj in expression {
            let s = sub_utils::pyobject_to_string(py, &obj)?;
            result.push_str(&s);
        }
        Ok(Self { expression: result })
    }

    fn __str__(&self) -> String {
        // Return the concatenated expression
        self.expression.clone()
    }

    fn __repr__(&self) -> String {
        "PythonExpression(...)".to_string()
    }

    /// Perform the substitution - evaluate the Python expression
    fn perform(&self, py: Python, _context: &Bound<'_, PyAny>) -> PyResult<String> {
        // Evaluate the Python expression
        // Safety: This evaluates arbitrary Python code, but it comes from the launch file
        // which is already trusted (user-provided configuration)
        let code = std::ffi::CString::new(self.expression.as_str()).map_err(|e| {
            pyo3::exceptions::PyValueError::new_err(format!("Invalid expression: {}", e))
        })?;
        let result = py.eval(&code, None, None)?;

        // Convert result to string
        if let Ok(s) = result.extract::<String>() {
            Ok(s)
        } else if let Ok(b) = result.extract::<bool>() {
            Ok(if b { "true" } else { "false" }.to_string())
        } else if let Ok(i) = result.extract::<i64>() {
            Ok(i.to_string())
        } else if let Ok(f) = result.extract::<f64>() {
            Ok(f.to_string())
        } else {
            // Fallback to __str__
            result.call_method0("__str__")?.extract::<String>()
        }
    }
}

/// Mock Command substitution
///
/// Python equivalent:
/// ```python
/// from launch.substitutions import Command
/// cmd = Command(['echo', 'hello'])
/// ```
///
/// Executes a shell command and returns its output
#[pyclass(module = "launch.substitutions", from_py_object)]
#[derive(Clone)]
pub struct Command {
    command: Vec<Py<PyAny>>,
}

#[pymethods]
impl Command {
    #[new]
    fn new(command: Vec<Py<PyAny>>) -> Self {
        Self { command }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        // Convert command parts to strings
        let cmd_parts: Result<Vec<String>, _> = self
            .command
            .iter()
            .map(|obj| {
                if let Ok(s) = obj.extract::<String>(py) {
                    Ok(s)
                } else if let Ok(str_result) = obj.call_method0(py, "__str__") {
                    str_result.extract::<String>(py)
                } else {
                    Ok(obj.to_string())
                }
            })
            .collect();

        let parts = cmd_parts?;
        // Return as substitution format
        Ok(format!("$(command {})", parts.join(" ")))
    }

    fn __repr__(&self) -> String {
        format!("Command({} parts)", self.command.len())
    }
}
