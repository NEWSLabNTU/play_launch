//! `GroupAction`.

use play_launch_parser::bridge::with_launch_context;
use pyo3::prelude::*;

/// `GroupAction(actions, scoped=True, forwarding=True,
/// launch_configurations=None)`. Executed as `launch`'s `get_sub_entities`
/// expands it:
///
/// - scoped and forwarding: `PushLaunchConfigurations`, `PushEnvironment`,
///   the actions, then both pops;
/// - scoped, not forwarding: the same, with `ResetEnvironment` and
///   `ResetLaunchConfigurations(launch_configurations)` after the pushes, so
///   the body sees ONLY the given configurations — no namespace, no global
///   parameters or remappings either, since those are configurations too;
/// - not scoped: `SetLaunchConfiguration` for each given configuration, then
///   the actions, with nothing popped.
///
/// The mock used to do none of this: its children had run in their own
/// constructors before it existed, so every group leaked whatever was set
/// inside it, and `condition=` was swallowed by `**kwargs`.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct GroupAction {
    #[pyo3(get)] // Make actions directly accessible as an attribute (like LaunchDescription)
    pub actions: Vec<Py<PyAny>>,
    scoped: bool,
    forwarding: bool,
    launch_configurations: Option<Py<PyAny>>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl GroupAction {
    #[new]
    #[pyo3(signature = (actions, *, scoped=true, forwarding=true, launch_configurations=None, condition=None, **_kwargs))]
    fn new(
        actions: Vec<Py<PyAny>>,
        scoped: bool,
        forwarding: bool,
        launch_configurations: Option<Py<PyAny>>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        Self {
            actions,
            scoped,
            forwarding,
            launch_configurations,
            condition,
        }
    }

    fn get_sub_entities(&self, py: Python) -> Vec<Py<PyAny>> {
        self.actions.iter().map(|a| a.clone_ref(py)).collect()
    }

    fn __repr__(&self) -> String {
        format!("GroupAction({} actions)", self.actions.len())
    }
}

impl GroupAction {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let (actions, scoped, forwarding, configurations) = {
            let me = this.borrow();
            (
                me.actions
                    .iter()
                    .map(|a| a.clone_ref(py))
                    .collect::<Vec<_>>(),
                me.scoped,
                me.forwarding,
                me.launch_configurations.as_ref().map(|c| c.clone_ref(py)),
            )
        };
        // The given configurations are evaluated before anything is reset.
        let given = super::configuration::evaluate_configurations(py, configurations.as_ref())?;

        if !scoped {
            with_launch_context(|ctx| {
                for (k, v) in given {
                    ctx.set_configuration_literal(k, v);
                }
            });
            for action in &actions {
                crate::api::visit::visit_any(py, action.bind(py))?;
            }
            return Ok(());
        }

        let saved = with_launch_context(|ctx| ctx.push_launch_configurations());
        if forwarding {
            with_launch_context(|ctx| {
                for (k, v) in given {
                    ctx.set_configuration_literal(k, v);
                }
            });
        } else {
            with_launch_context(|ctx| {
                ctx.reset_environment();
                ctx.reset_launch_configurations(given);
            });
        }
        let result = actions
            .iter()
            .try_for_each(|action| crate::api::visit::visit_any(py, action.bind(py)));
        with_launch_context(|ctx| ctx.pop_launch_configurations(saved));
        result
    }
}
