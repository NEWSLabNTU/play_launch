//! Mock `launch_ros.actions.ComposableNodeContainer`.

use super::{
    composable_node::ComposableNode,
    node::{Node, NodeSpec},
};
use play_launch_parser::{bridge::capture_container, captures::ContainerCapture};
use pyo3::{prelude::*, types::PyDict};

/// `ComposableNodeContainer(name, namespace, package, executable,
/// composable_node_descriptions=None, ...)`: a `Node` whose process is the
/// container, which then loads the descriptions whose own condition admits
/// them (`ComposableNodeContainer.execute`), into ITSELF — by its
/// fully-qualified name, the container's `node_name`.
#[pyclass(module = "launch_ros.actions", from_py_object)]
#[derive(Clone)]
pub struct ComposableNodeContainer {
    spec: NodeSpec,
    composable_node_descriptions: Option<Py<PyAny>>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
    /// The fully-qualified name once executed.
    executed_fqn: Option<String>,
}

#[pymethods]
impl ComposableNodeContainer {
    #[new]
    #[pyo3(signature = (
        *,
        name,
        namespace,
        package=None,
        executable=None,
        composable_node_descriptions=None,
        parameters=None,
        remappings=None,
        ros_arguments=None,
        arguments=None,
        env=None,
        additional_env=None,
        condition=None,
        **_kwargs
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        name: Py<PyAny>,
        namespace: Py<PyAny>,
        package: Option<Py<PyAny>>,
        executable: Option<Py<PyAny>>,
        composable_node_descriptions: Option<Py<PyAny>>,
        parameters: Option<Py<PyAny>>,
        remappings: Option<Py<PyAny>>,
        ros_arguments: Option<Py<PyAny>>,
        arguments: Option<Py<PyAny>>,
        env: Option<Py<PyAny>>,
        additional_env: Option<Py<PyAny>>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            spec: NodeSpec {
                package,
                executable,
                name: Some(name),
                namespace: Some(namespace),
                parameters,
                remappings,
                arguments,
                ros_arguments,
                env,
                additional_env,
            },
            composable_node_descriptions,
            condition,
            executed_fqn: None,
        }
    }

    /// `node_name`: the container's fully-qualified name once executed.
    #[getter]
    fn node_name(&self) -> Option<String> {
        self.executed_fqn.clone()
    }

    fn __repr__(&self) -> String {
        "ComposableNodeContainer(...)".to_string()
    }
}

impl ComposableNodeContainer {
    /// The name a `LoadComposableNodes` targets: the FQN it executed with, or
    /// — not executed yet — what it would expand to here.
    pub(crate) fn target_name(this: &Bound<'_, Self>, py: Python) -> PyResult<String> {
        if let Some(fqn) = this.borrow().executed_fqn.clone() {
            return Ok(fqn);
        }
        let spec = this.borrow().spec.clone();
        let n = spec.expand(py)?;
        Ok(fqn_of(&n.namespace, n.name.as_deref().unwrap_or_default()))
    }

    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let spec = this.borrow().spec.clone();
        let label = format!("container {}", Node::label(py, &spec));
        if !crate::api::visit::claim_execution(this.as_any(), &label) {
            return Ok(());
        }
        let n = spec.expand(py)?;
        let name = n.name.clone().unwrap_or_default();
        let namespace = n.namespace.clone().unwrap_or_else(|| "/".to_string());
        let fqn = fqn_of(&n.namespace, &name);

        let mark = crate::api::visit::lens();
        capture_container(ContainerCapture {
            name,
            namespace,
            package: Some(if n.package.is_empty() {
                play_launch_parser::actions::container::DEFAULT_CONTAINER_PACKAGE.to_string()
            } else {
                n.package
            }),
            executable: Some(if n.executable.is_empty() {
                play_launch_parser::actions::container::DEFAULT_CONTAINER_EXECUTABLE.to_string()
            } else {
                n.executable
            }),
            cmd: Vec::new(),
            ros_arguments: n.ros_arguments,
            parameters: n.parameters,
            params_files: n.params_files,
            remappings: n.remappings,
            arguments: n.arguments,
            env_vars: n.env,
            global_params: Some(n.global_params),
            scope_id: None,
            start_delay_secs: None,
        });
        crate::api::visit::stamp_delay(mark);
        this.borrow_mut().executed_fqn = Some(fqn.clone());

        let descriptions = this
            .borrow()
            .composable_node_descriptions
            .as_ref()
            .map(|d| d.clone_ref(py));
        if let Some(descriptions) = descriptions {
            let descriptions = descriptions.bind(py);
            if !descriptions.is_none() {
                for d in descriptions.try_iter()? {
                    let d = d?;
                    let d = d.cast::<ComposableNode>()?;
                    let d = d.borrow();
                    if d.admitted(py)? {
                        d.capture_load(py, &fqn)?;
                    }
                }
            }
        }
        Ok(())
    }
}

/// `prefix_namespace(namespace, name)`, absolute.
pub(crate) fn fqn_of(namespace: &Option<String>, name: &str) -> String {
    match namespace.as_deref() {
        None | Some("") | Some("/") => format!("/{name}"),
        Some(ns) => format!("{ns}/{name}"),
    }
}
