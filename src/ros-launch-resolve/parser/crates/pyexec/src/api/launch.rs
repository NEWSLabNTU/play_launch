//! Mock `launch` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

use pyo3::prelude::*;

/// Mock LaunchDescription class
///
/// Python equivalent (stock `launch`, `initial_entities` optional):
/// ```python
/// from launch import LaunchDescription
/// launch_desc = LaunchDescription([action1, action2, ...])
///
/// ld = LaunchDescription()  # nav2_bringup's way (#0078)
/// ld.add_action(action1)
/// ```
#[pyclass(module = "launch")]
pub struct LaunchDescription {
    #[pyo3(get)]
    pub actions: Vec<Py<PyAny>>,
}

#[pymethods]
impl LaunchDescription {
    #[new]
    #[pyo3(signature = (initial_entities=None, *, deprecated_reason=None))]
    fn new(
        initial_entities: Option<&Bound<'_, PyAny>>,
        deprecated_reason: Option<&Bound<'_, PyAny>>,
    ) -> PyResult<Self> {
        let _ = deprecated_reason;
        let mut actions = Vec::new();
        if let Some(entities) = initial_entities.filter(|e| !e.is_none()) {
            for entity in entities.try_iter()? {
                actions.push(entity?.unbind());
            }
        }
        Ok(Self { actions })
    }

    /// Stock `launch`'s name for the list.
    #[getter]
    fn entities(&self, py: Python) -> Vec<Py<PyAny>> {
        self.actions.iter().map(|a| a.clone_ref(py)).collect()
    }

    fn add_action(&mut self, action: Py<PyAny>) {
        self.actions.push(action);
    }

    fn add_entity(&mut self, entity: Py<PyAny>) {
        self.actions.push(entity);
    }

    fn __repr__(&self) -> String {
        format!("LaunchDescription({} actions)", self.actions.len())
    }
}
