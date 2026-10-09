//! `DeclareLaunchArgument`.

use crate::api::utils::pyobject_to_string;
use play_launch_parser::bridge::with_launch_context;
use pyo3::{exceptions::PyRuntimeError, prelude::*};

/// `DeclareLaunchArgument(name, default_value=None, description=None,
/// choices=None)`. When executed, as in `launch`:
///
/// - an unset configuration takes the default (evaluated NOW);
/// - an unset configuration with no default is an error, recorded here and
///   reported by the traverser after the include's own required-argument check
///   (`unset_at_execute`);
/// - a value outside `choices` is an error.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct DeclareLaunchArgument {
    name: String,
    default_value: Option<Py<PyAny>>,
    description: Option<String>,
    choices: Option<Vec<String>>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl DeclareLaunchArgument {
    #[new]
    #[pyo3(signature = (name, *, default_value=None, description=None, choices=None, condition=None, **_kwargs))]
    fn new(
        name: String,
        default_value: Option<Py<PyAny>>,
        description: Option<String>,
        choices: Option<Vec<String>>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> Self {
        Self {
            name,
            default_value,
            description,
            choices,
            condition,
        }
    }

    #[getter]
    fn name(&self) -> &str {
        &self.name
    }

    fn __repr__(&self) -> String {
        format!("DeclareLaunchArgument('{}')", self.name)
    }
}

impl DeclareLaunchArgument {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let me = this.borrow();
        let name = me.name.clone();
        let mut unset = false;
        if with_launch_context(|ctx| ctx.get_configuration(&name)).is_none() {
            match &me.default_value {
                Some(default) => {
                    let value = pyobject_to_string(py, default)?;
                    with_launch_context(|ctx| ctx.set_configuration_literal(name.clone(), value));
                }
                None => unset = true,
            }
        }

        if let Some(choices) = &me.choices
            && let Some(value) = with_launch_context(|ctx| ctx.get_configuration(&name))
            && !choices.contains(&value)
        {
            return Err(PyRuntimeError::new_err(format!(
                "Argument '{name}' provided value '{value}' is not valid. Valid options are: {choices:?}"
            )));
        }

        play_launch_parser::bridge::capture_declaration(
            play_launch_parser::captures::DeclaredArgumentCapture {
                name,
                description: me.description.clone(),
                has_default: me.default_value.is_some(),
                // `capture_declaration` adds whether an `OpaqueFunction` is
                // running; a declaration under a condition is equally
                // invisible to launch's include-time check.
                opaque: crate::api::visit::under_condition(),
                unset_at_execute: unset,
            },
        );
        Ok(())
    }
}
