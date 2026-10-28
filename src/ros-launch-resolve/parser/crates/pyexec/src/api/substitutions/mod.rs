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
/// The parts are kept and performed when the expression is, as in `launch`
/// (#0078): `generate_launch_description()` builds
/// `PythonExpression(['not ', LaunchConfiguration('use_composition')])`
/// (nav2_bringup) before the argument is declared, and joining the parts
/// then gave `not $(var use_composition)`, which `eval` refused.
#[pyclass(module = "launch.substitutions", from_py_object)]
pub struct PythonExpression {
    parts: Vec<Py<PyAny>>,
}

impl Clone for PythonExpression {
    fn clone(&self) -> Self {
        Python::attach(|py| Self {
            parts: self.parts.iter().map(|p| p.clone_ref(py)).collect(),
        })
    }
}

impl PythonExpression {
    /// The parts, performed now and joined.
    fn expression(&self, py: Python) -> PyResult<String> {
        let mut result = String::new();
        for part in &self.parts {
            result.push_str(&sub_utils::pyobject_to_string(py, part)?);
        }
        Ok(result)
    }
}

#[pymethods]
impl PythonExpression {
    #[new]
    #[pyo3(signature = (expression, python_modules=None))]
    fn new(expression: &Bound<'_, PyAny>, python_modules: Option<&Bound<'_, PyAny>>) -> Self {
        let _ = python_modules;
        // One substitution, a string, or a list or tuple of them.
        let parts = match expression.cast::<pyo3::types::PyList>() {
            Ok(list) => list.iter().map(|p| p.unbind()).collect(),
            Err(_) => match expression.cast::<pyo3::types::PyTuple>() {
                Ok(tuple) => tuple.iter().map(|p| p.unbind()).collect(),
                Err(_) => vec![expression.clone().unbind()],
            },
        };
        Self { parts }
    }

    fn __str__(&self, py: Python) -> PyResult<String> {
        self.expression(py)
    }

    fn __repr__(&self) -> String {
        "PythonExpression(...)".to_string()
    }

    /// Perform the substitution - evaluate the Python expression
    fn perform(&self, py: Python, _context: &Bound<'_, PyAny>) -> PyResult<String> {
        // Evaluate the Python expression
        // Safety: This evaluates arbitrary Python code, but it comes from the launch file
        // which is already trusted (user-provided configuration)
        let expression = self.expression(py)?;
        let code = std::ffi::CString::new(expression.as_str()).map_err(|e| {
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
    command: Py<PyAny>,
    on_stderr: Option<Py<PyAny>>,
}

#[pymethods]
impl Command {
    #[new]
    #[pyo3(signature = (command, on_stderr=None))]
    fn new(command: Py<PyAny>, on_stderr: Option<Py<PyAny>>) -> Self {
        Self { command, on_stderr }
    }

    /// `Command.perform`: the command, performed and CONCATENATED (a list is
    /// one string, as everywhere in `launch`), split with `shlex` and run;
    /// stdout as is.
    fn perform(&self, py: Python, _context: &Bound<'_, PyAny>) -> PyResult<String> {
        use play_launch_parser::substitution::types::{CommandErrorMode, Substitution};
        let cmd = sub_utils::pyobject_to_string(py, &self.command)?;
        let error_mode = match &self.on_stderr {
            Some(o) => match sub_utils::pyobject_to_string(py, o)?.as_str() {
                "fail" => CommandErrorMode::Strict,
                "warn" => CommandErrorMode::Warn,
                "ignore" => CommandErrorMode::Ignore,
                "capture" => CommandErrorMode::Capture,
                other => {
                    return Err(pyo3::exceptions::PyValueError::new_err(format!(
                        "expected 'on_stderr' to be one of: 'fail', 'ignore', 'warn' or \
                         'capture', got '{other}'"
                    )));
                }
            },
            None => CommandErrorMode::Strict,
        };
        let sub = Substitution::Command {
            cmd: vec![Substitution::Text(cmd)],
            error_mode,
        };
        play_launch_parser::bridge::with_launch_context(|ctx| sub.resolve(ctx))
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn __repr__(&self) -> String {
        "Command(...)".to_string()
    }
}
