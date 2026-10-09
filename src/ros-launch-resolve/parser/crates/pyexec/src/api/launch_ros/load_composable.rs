//! Mock `launch_ros.actions.LoadComposableNodes`.

use super::{composable_node::ComposableNode, container::ComposableNodeContainer};
use pyo3::prelude::*;

/// `LoadComposableNodes(composable_node_descriptions, target_container)`.
///
/// The target is a container action — its fully-qualified `node_name` — or a
/// name, performed and used as given. Every description is loaded: unlike
/// `ComposableNodeContainer`, `launch_ros` (Humble) does not consult a
/// description's own condition here.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct LoadComposableNodes {
    target_container: Py<PyAny>,
    composable_node_descriptions: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl LoadComposableNodes {
    #[new]
    #[pyo3(signature = (*, composable_node_descriptions, target_container, condition=None, **_kwargs))]
    fn new(
        composable_node_descriptions: Py<PyAny>,
        target_container: Py<PyAny>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        Self {
            target_container,
            composable_node_descriptions,
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "LoadComposableNodes(...)".to_string()
    }
}

impl LoadComposableNodes {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        if !crate::api::visit::claim_execution(this.as_any(), "LoadComposableNodes") {
            return Ok(());
        }
        let (target, descriptions) = {
            let me = this.borrow();
            (
                me.target_container.clone_ref(py),
                me.composable_node_descriptions.clone_ref(py),
            )
        };
        let target = target.bind(py);
        let target_name = if let Ok(container) = target.cast::<ComposableNodeContainer>() {
            ComposableNodeContainer::target_name(container, py)?
        } else {
            crate::api::utils::resolve_tokens(&crate::api::utils::pyobject_to_string(
                py,
                &target.clone().unbind(),
            )?)
        };
        for d in descriptions.bind(py).try_iter()? {
            let d = d?;
            let d = d.cast::<ComposableNode>()?;
            d.borrow().capture_load(py, &target_name)?;
        }
        Ok(())
    }
}
