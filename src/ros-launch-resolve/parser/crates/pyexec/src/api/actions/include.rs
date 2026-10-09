//! `IncludeLaunchDescription`.

use crate::api::utils::{pyobject_to_string, resolve_tokens};
use play_launch_parser::bridge::with_launch_context;
use pyo3::prelude::*;

/// `IncludeLaunchDescription(launch_description_source, launch_arguments=None)`.
///
/// Executed as `launch` executes it: the source's location is performed, the
/// arguments are performed and set as launch configurations IN ORDER (a later
/// one may read an earlier one, and they persist afterwards — an include
/// scopes nothing), and the included file runs right here, before the next
/// action, through the traverser (`api::visit::include`). What it declares,
/// `<let>`s, pushes or sets is visible to every action after it.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct IncludeLaunchDescription {
    launch_description_source: Py<PyAny>,
    launch_arguments: Option<Py<PyAny>>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl IncludeLaunchDescription {
    #[new]
    #[pyo3(signature = (launch_description_source, *, launch_arguments=None, condition=None, **_kwargs))]
    fn new(
        py: Python,
        launch_description_source: Py<PyAny>,
        launch_arguments: Option<Py<PyAny>>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> PyResult<Self> {
        // `launch_arguments` is commonly `dict.items()` — a view that is
        // still fine to iterate later — or a generator, which is not. Take a
        // list of it now.
        let launch_arguments = match launch_arguments {
            Some(a) if !a.is_none(py) => {
                let bound = a.bind(py);
                let list = if let Ok(dict) = bound.cast::<pyo3::types::PyDict>() {
                    dict.items().into_any()
                } else {
                    pyo3::types::PyList::new(py, bound.try_iter()?.collect::<PyResult<Vec<_>>>()?)?
                        .into_any()
                };
                Some(list.unbind())
            }
            _ => None,
        };
        Ok(Self {
            launch_description_source,
            launch_arguments,
            condition,
        })
    }

    #[getter]
    fn launch_arguments(&self, py: Python) -> Py<PyAny> {
        self.launch_arguments
            .as_ref()
            .map(|a| a.clone_ref(py))
            .unwrap_or_else(|| pyo3::types::PyList::empty(py).into_any().unbind())
    }

    fn __repr__(&self) -> String {
        "IncludeLaunchDescription(...)".to_string()
    }
}

impl IncludeLaunchDescription {
    /// The resolved path of the file to include.
    fn file_path(&self, py: Python) -> PyResult<String> {
        let source = self.launch_description_source.bind(py);
        if let Ok(s) = source.extract::<String>() {
            return Ok(resolve_tokens(&s));
        }
        if let Ok(path) = source.call_method0("get_launch_file_path")
            && let Ok(path) = path.extract::<String>()
        {
            return Ok(path);
        }
        Ok(resolve_tokens(&pyobject_to_string(
            py,
            &self.launch_description_source,
        )?))
    }

    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let me = this.borrow();
        // A description passed directly is run in place, in this file.
        let source = me.launch_description_source.clone_ref(py);
        let inline = source
            .bind(py)
            .cast::<crate::api::launch::LaunchDescription>()
            .is_ok();
        let file_path = if inline {
            String::new()
        } else {
            me.file_path(py)?
        };

        let mut args = Vec::new();
        if let Some(list) = &me.launch_arguments {
            for item in list.bind(py).try_iter()? {
                let item = item?;
                let (k, v): (Py<PyAny>, Py<PyAny>) = item.extract()?;
                let key = pyobject_to_string(py, &k)?;
                let value = resolve_tokens(&pyobject_to_string(py, &v)?);
                // `SetLaunchConfiguration(name, value)`, in order: the next
                // argument sees this one.
                with_launch_context(|ctx| {
                    ctx.set_configuration_literal(key.clone(), value.clone())
                });
                args.push((key, value));
            }
        }
        drop(me);

        if inline {
            return crate::api::visit::visit_any(py, source.bind(py));
        }
        log::debug!(
            "Python Launch IncludeLaunchDescription: {} with {} args",
            file_path,
            args.len()
        );
        crate::api::visit::include(file_path, args)
    }
}
