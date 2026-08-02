//! Mock `launch` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use pyo3::prelude::*;

/// Mock LaunchDescription class
///
/// Python equivalent:
/// ```python
/// from launch import LaunchDescription
/// launch_desc = LaunchDescription([action1, action2, ...])
/// ```
#[pyclass(module = "launch")]
pub struct LaunchDescription {
    #[pyo3(get)]
    pub actions: Vec<PyObject>,
}

#[pymethods]
impl LaunchDescription {
    #[new]
    fn new(actions: Vec<PyObject>) -> Self {
        Self { actions }
    }

    fn __repr__(&self) -> String {
        format!("LaunchDescription({} actions)", self.actions.len())
    }
}
