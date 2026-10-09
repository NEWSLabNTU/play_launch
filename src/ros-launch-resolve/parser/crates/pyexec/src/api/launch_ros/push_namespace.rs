//! Mock `PushRosNamespace` / `PopRosNamespace`.
//!
//! `ros_namespace` is a launch configuration in `launch_ros`, so a scoped
//! group pops it and an include does NOT: a `PushRosNamespace` at the top
//! level of an included `.launch.py` applies to the includer's later actions.

use pyo3::prelude::*;

#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct PushRosNamespace {
    namespace: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl PushRosNamespace {
    #[new]
    #[pyo3(signature = (namespace, *, condition=None, **_kwargs))]
    fn new(
        namespace: Py<PyAny>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        Self {
            namespace,
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "PushRosNamespace(...)".to_string()
    }
}

impl PushRosNamespace {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let ns = crate::api::utils::pyobject_to_string(py, &this.borrow().namespace)?;
        play_launch_parser::bridge::push_ros_namespace(ns);
        Ok(())
    }
}

#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct PopRosNamespace {
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl PopRosNamespace {
    #[new]
    #[pyo3(signature = (*, condition=None, **_kwargs))]
    fn new(condition: Option<Py<PyAny>>, _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>) -> Self {
        Self { condition }
    }

    fn __repr__(&self) -> String {
        "PopRosNamespace()".to_string()
    }
}

impl PopRosNamespace {
    pub(crate) fn execute(_this: &Bound<'_, Self>, _py: Python) -> PyResult<()> {
        play_launch_parser::bridge::pop_ros_namespace();
        Ok(())
    }
}
