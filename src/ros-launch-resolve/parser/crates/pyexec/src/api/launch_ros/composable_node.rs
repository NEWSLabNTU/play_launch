//! Mock `launch_ros.descriptions.ComposableNode`.
//!
//! A description, not an action: it is performed when the container or
//! `LoadComposableNodes` that holds it executes, the way
//! `get_composable_node_load_request` builds the LoadNode request.

use super::{
    helpers::{is_yaml_file, load_yaml_params_for_node},
    node::{
        make_namespace_absolute, parse_parameter_item, prefix_namespace, remappings, ros_namespace,
    },
};
use crate::api::utils::pyobject_to_string;
use play_launch_parser::{
    bridge::{capture_load_node, with_launch_context},
    captures::LoadNodeCapture,
};
use pyo3::{prelude::*, types::PyDict};

#[pyclass(module = "launch_ros.descriptions", from_py_object)]
#[derive(Clone)]
pub struct ComposableNode {
    package: Py<PyAny>,
    plugin: Py<PyAny>,
    name: Option<Py<PyAny>>,
    namespace: Option<Py<PyAny>>,
    parameters: Option<Py<PyAny>>,
    remappings: Option<Py<PyAny>>,
    extra_arguments: Option<Py<PyAny>>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl ComposableNode {
    #[new]
    #[pyo3(signature = (
        *,
        package,
        plugin,
        name=None,
        namespace=None,
        parameters=None,
        remappings=None,
        extra_arguments=None,
        condition=None,
        **_kwargs
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        package: Py<PyAny>,
        plugin: Py<PyAny>,
        name: Option<Py<PyAny>>,
        namespace: Option<Py<PyAny>>,
        parameters: Option<Py<PyAny>>,
        remappings: Option<Py<PyAny>>,
        extra_arguments: Option<Py<PyAny>>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            package,
            plugin,
            name,
            namespace,
            parameters,
            remappings,
            extra_arguments,
            condition,
        }
    }

    #[getter]
    fn package(&self, py: Python) -> Py<PyAny> {
        self.package.clone_ref(py)
    }

    #[getter]
    fn node_plugin(&self, py: Python) -> Py<PyAny> {
        self.plugin.clone_ref(py)
    }

    #[getter]
    fn node_name(&self, py: Python) -> Option<Py<PyAny>> {
        self.name.as_ref().map(|n| n.clone_ref(py))
    }

    #[getter]
    fn node_namespace(&self, py: Python) -> Option<Py<PyAny>> {
        self.namespace.as_ref().map(|n| n.clone_ref(py))
    }

    fn __repr__(&self) -> String {
        "ComposableNode(...)".to_string()
    }
}

impl ComposableNode {
    /// Whether this description's own `condition=` admits it — what
    /// `ComposableNodeContainer.execute` filters on.
    pub(crate) fn admitted(&self, py: Python) -> PyResult<bool> {
        match &self.condition {
            Some(c) if !c.is_none(py) => crate::api::conditions::evaluate(py, c.bind(py)),
            _ => Ok(true),
        }
    }

    /// Capture the LoadNode request this description makes, into the
    /// container named `target`, performed in the context the walk is in now.
    pub(crate) fn capture_load(&self, py: Python, target: &str) -> PyResult<()> {
        let package = pyobject_to_string(py, &self.package)?;
        let plugin = pyobject_to_string(py, &self.plugin)?;
        let node_name = self
            .name
            .as_ref()
            .map(|n| pyobject_to_string(py, n))
            .transpose()?
            .unwrap_or_default();
        let own_ns = self
            .namespace
            .as_ref()
            .map(|n| pyobject_to_string(py, n))
            .transpose()?;
        let base = ros_namespace();
        let namespace =
            make_namespace_absolute(prefix_namespace(base.as_deref(), own_ns.as_deref()))
                .unwrap_or_else(|| "/".to_string());

        let fqn = if namespace == "/" {
            format!("/{node_name}")
        } else {
            format!("{namespace}/{node_name}")
        };

        // Parameters: files are loaded here, for this node, because a LoadNode
        // request carries values, not files.
        let mut parameters = Vec::new();
        if let Some(params) = &self.parameters {
            let params = params.bind(py);
            if !params.is_none() {
                for item in params.try_iter()? {
                    for (key, value) in parse_parameter_item(&item?)? {
                        if key == "__param_file" && is_yaml_file(&value) {
                            match load_yaml_params_for_node(&value, &fqn) {
                                Ok(loaded) => parameters.extend(loaded),
                                Err(e) => {
                                    log::warn!("Failed to load parameter file {value}: {e}");
                                    parameters.push((key, value));
                                }
                            }
                        } else {
                            parameters.push((key, value));
                        }
                    }
                }
            }
        }

        // `ros_remaps` first, then the node's own.
        let mut remaps = with_launch_context(|ctx| ctx.remappings());
        remaps.extend(remappings(py, self.remappings.as_ref())?);

        let mut extra_args = std::collections::HashMap::new();
        if let Some(extra) = &self.extra_arguments {
            let extra = extra.bind(py);
            if !extra.is_none() {
                for item in extra.try_iter()? {
                    let item = item?;
                    if let Ok(dict) = item.cast::<PyDict>() {
                        let mut pairs = Vec::new();
                        super::node::Node::parse_dict_params(dict, "", &mut pairs)?;
                        extra_args.extend(pairs);
                    }
                }
            }
        }

        let mark = crate::api::visit::lens();
        capture_load_node(LoadNodeCapture {
            package,
            plugin,
            target_container_name: target.to_string(),
            node_name,
            namespace,
            parameters,
            remappings: remaps,
            extra_args,
            global_params: Some(
                with_launch_context(|ctx| ctx.global_parameters())
                    .into_iter()
                    .collect(),
            ),
            scope_id: None,
            start_delay_secs: None,
        });
        crate::api::visit::stamp_delay(mark);
        Ok(())
    }
}
