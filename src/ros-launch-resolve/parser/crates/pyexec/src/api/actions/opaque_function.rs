//! `OpaqueFunction`.

use pyo3::{
    prelude::*,
    types::{PyDict, PyTuple},
};

/// `OpaqueFunction(function, args=None, kwargs=None)`: when executed,
/// `function(context, *args, **kwargs)` is called and the actions it returns
/// are executed in its place.
///
/// `context` is a live view of the launch context the walk is in
/// (`utils::LaunchContextView`): `context.launch_configurations` reads what
/// the actions before this one set — including what an included file
/// declared — and a write to it is seen by the actions after it.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct OpaqueFunction {
    function: Option<Py<PyAny>>,
    args: Option<Py<PyAny>>,
    kwargs: Option<Py<PyAny>>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl OpaqueFunction {
    #[new]
    #[pyo3(signature = (*, function=None, args=None, kwargs=None, condition=None, **_extra))]
    fn new(
        function: Option<Py<PyAny>>,
        args: Option<Py<PyAny>>,
        kwargs: Option<Py<PyAny>>,
        condition: Option<Py<PyAny>>,
        _extra: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            function,
            args,
            kwargs,
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "OpaqueFunction(...)".to_string()
    }
}

impl OpaqueFunction {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let (function, args, kwargs) = {
            let me = this.borrow();
            (
                me.function.as_ref().map(|f| f.clone_ref(py)),
                me.args.as_ref().map(|a| a.clone_ref(py)),
                me.kwargs.as_ref().map(|k| k.clone_ref(py)),
            )
        };
        let Some(function) = function else {
            return Ok(());
        };
        let context = crate::api::utils::create_launch_context(py)?;
        let mut call_args = vec![context];
        if let Some(args) = args {
            for a in args.bind(py).try_iter()? {
                call_args.push(a?.unbind());
            }
        }
        let call_args = PyTuple::new(py, call_args)?;
        let call_kwargs = match kwargs {
            Some(k) if !k.is_none(py) => Some(k.bind(py).cast::<PyDict>()?.clone()),
            _ => None,
        };

        // A declaration the function returns is invisible to launch's
        // include-time check, so the returned actions run as "opaque" too.
        play_launch_parser::bridge::enter_opaque_function();
        let result = function
            .bind(py)
            .call(call_args, call_kwargs.as_ref())
            .and_then(|returned| crate::api::visit::visit_any(py, &returned));
        play_launch_parser::bridge::leave_opaque_function();
        result
    }
}
