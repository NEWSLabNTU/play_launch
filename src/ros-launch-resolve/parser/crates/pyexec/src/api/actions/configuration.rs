//! Configuration and environment actions (`launch.actions`).
//!
//! Each records its arguments when constructed and does its work when the
//! walk EXECUTES it (`api::visit`), as `launch` does.

use crate::api::utils::pyobject_to_string;
use play_launch_parser::bridge::with_launch_context;
use pyo3::{prelude::*, types::PyDict};

/// `SetLaunchConfiguration(name, value)`.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct SetLaunchConfiguration {
    name: Py<PyAny>,
    value: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl SetLaunchConfiguration {
    #[new]
    #[pyo3(signature = (name, value, *, condition=None, **_kwargs))]
    fn new(
        name: Py<PyAny>,
        value: Py<PyAny>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            name,
            value,
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "SetLaunchConfiguration(...)".to_string()
    }
}

impl SetLaunchConfiguration {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let me = this.borrow();
        let name = pyobject_to_string(py, &me.name)?;
        let value = pyobject_to_string(py, &me.value)?;
        log::debug!("Python Launch SetLaunchConfiguration: {}={}", name, value);
        with_launch_context(|ctx| ctx.set_configuration_literal(name, value));
        Ok(())
    }
}

/// `UnsetLaunchConfiguration(name)`.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct UnsetLaunchConfiguration {
    name: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl UnsetLaunchConfiguration {
    #[new]
    #[pyo3(signature = (name, *, condition=None, **_kwargs))]
    fn new(
        name: Py<PyAny>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self { name, condition }
    }

    fn __repr__(&self) -> String {
        "UnsetLaunchConfiguration(...)".to_string()
    }
}

impl UnsetLaunchConfiguration {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let name = pyobject_to_string(py, &this.borrow().name)?;
        with_launch_context(|ctx| ctx.unset_configuration(&name));
        Ok(())
    }
}

/// `RegisterEventHandler(event_handler)`.
///
/// The handlers whose events a running launch produces as it comes up —
/// `OnProcessStart`, `OnStateTransition` — run their actions in a real launch,
/// so the walk executes them, in place. `OnProcessExit` and `OnShutdown` run
/// only when something stops, which is not part of the system being modelled.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct RegisterEventHandler {
    event_handler: Py<PyAny>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl RegisterEventHandler {
    #[new]
    #[pyo3(signature = (event_handler, *, condition=None, **_kwargs))]
    fn new(
        event_handler: Py<PyAny>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            event_handler,
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "RegisterEventHandler(...)".to_string()
    }
}

impl RegisterEventHandler {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let handler = this.borrow().event_handler.clone_ref(py);
        let entities = crate::api::event_handlers::startup_entities(py, handler.bind(py))?;
        if let Some(entities) = entities {
            crate::api::visit::visit_any(py, entities.bind(py))?;
        }
        Ok(())
    }
}

/// A unit action with no arguments but a condition.
macro_rules! unit_action {
    ($name:ident, $doc:literal, $body:expr) => {
        #[doc = $doc]
        #[pyclass(module = "launch.actions", from_py_object)]
        #[derive(Clone)]
        pub struct $name {
            #[pyo3(get)]
            condition: Option<Py<PyAny>>,
        }

        #[pymethods]
        impl $name {
            #[new]
            #[pyo3(signature = (*, condition=None, **_kwargs))]
            fn new(condition: Option<Py<PyAny>>, _kwargs: Option<&Bound<'_, PyDict>>) -> Self {
                Self { condition }
            }

            fn __repr__(&self) -> String {
                concat!(stringify!($name), "()").to_string()
            }
        }

        impl $name {
            pub(crate) fn execute(_this: &Bound<'_, Self>, _py: Python) -> PyResult<()> {
                $body
            }
        }
    };
}

unit_action!(
    PushEnvironment,
    "`PushEnvironment()`: save the environment for a later `PopEnvironment`.",
    {
        crate::api::visit::push_environment();
        Ok(())
    }
);
unit_action!(
    PopEnvironment,
    "`PopEnvironment()`.",
    crate::api::visit::pop_environment()
);
unit_action!(
    ResetEnvironment,
    "`ResetEnvironment()`: back to the environment the launch started with.",
    {
        with_launch_context(|ctx| ctx.reset_environment());
        Ok(())
    }
);
unit_action!(
    PushLaunchConfigurations,
    "`PushLaunchConfigurations()`: save every configuration (the namespace and \
     the global lists with them) for a later `PopLaunchConfigurations`.",
    {
        crate::api::visit::push_configurations();
        Ok(())
    }
);
unit_action!(
    PopLaunchConfigurations,
    "`PopLaunchConfigurations()`.",
    crate::api::visit::pop_configurations()
);

/// `ResetLaunchConfigurations(launch_configurations=None)`: clear every
/// configuration and set only the given ones, evaluated BEFORE the clear.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct ResetLaunchConfigurations {
    launch_configurations: Option<Py<PyAny>>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl ResetLaunchConfigurations {
    #[new]
    #[pyo3(signature = (launch_configurations=None, *, condition=None, **_kwargs))]
    fn new(
        launch_configurations: Option<Py<PyAny>>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            launch_configurations,
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "ResetLaunchConfigurations()".to_string()
    }
}

impl ResetLaunchConfigurations {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let keep = evaluate_configurations(py, this.borrow().launch_configurations.as_ref())?;
        with_launch_context(|ctx| ctx.reset_launch_configurations(keep));
        Ok(())
    }
}

/// Evaluate a `{name: value}` mapping of substitutions, in the current
/// context — what `ResetLaunchConfigurations` (and so `GroupAction(
/// forwarding=False)`) keeps.
pub(crate) fn evaluate_configurations(
    py: Python,
    mapping: Option<&Py<PyAny>>,
) -> PyResult<Vec<(String, String)>> {
    let mut out = Vec::new();
    let Some(mapping) = mapping else {
        return Ok(out);
    };
    let mapping = mapping.bind(py);
    if mapping.is_none() {
        return Ok(out);
    }
    let items = mapping.call_method0("items")?;
    for item in items.try_iter()? {
        let (k, v): (Py<PyAny>, Py<PyAny>) = item?.extract()?;
        out.push((pyobject_to_string(py, &k)?, pyobject_to_string(py, &v)?));
    }
    Ok(out)
}

/// `AppendEnvironmentVariable(name, value, prepend=False, separator=os.pathsep)`.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct AppendEnvironmentVariable {
    name: Py<PyAny>,
    value: Py<PyAny>,
    prepend: Option<Py<PyAny>>,
    separator: Option<Py<PyAny>>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl AppendEnvironmentVariable {
    #[new]
    #[pyo3(signature = (name, value, *, prepend=None, separator=None, condition=None, **_kwargs))]
    fn new(
        name: Py<PyAny>,
        value: Py<PyAny>,
        prepend: Option<Py<PyAny>>,
        separator: Option<Py<PyAny>>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self {
            name,
            value,
            prepend,
            separator,
            condition,
        }
    }

    fn __repr__(&self) -> String {
        "AppendEnvironmentVariable(...)".to_string()
    }
}

impl AppendEnvironmentVariable {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let me = this.borrow();
        let name = pyobject_to_string(py, &me.name)?;
        let value = pyobject_to_string(py, &me.value)?;
        let prepend = match &me.prepend {
            Some(p) => {
                if let Ok(b) = p.extract::<bool>(py) {
                    b
                } else {
                    matches!(
                        pyobject_to_string(py, p)?.to_lowercase().as_str(),
                        "true" | "1"
                    )
                }
            }
            None => false,
        };
        let separator = match &me.separator {
            Some(s) => pyobject_to_string(py, s)?,
            None => ":".to_string(),
        };
        let current = with_launch_context(|ctx| ctx.get_environment_variable(&name))
            .or_else(|| std::env::var(&name).ok())
            .unwrap_or_default();
        let combined = if current.is_empty() {
            value
        } else if prepend {
            format!("{value}{separator}{current}")
        } else {
            format!("{current}{separator}{value}")
        };
        with_launch_context(|ctx| ctx.set_environment_variable(name, combined));
        Ok(())
    }
}

/// `Shutdown(reason=...)`: ends a launch when it executes; nothing to model.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct Shutdown {
    #[allow(dead_code)] // Keep for future use
    reason: Option<String>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
}

#[pymethods]
impl Shutdown {
    #[new]
    #[pyo3(signature = (*, reason=None, condition=None, **_kwargs))]
    fn new(
        reason: Option<String>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        Self { reason, condition }
    }

    fn __repr__(&self) -> String {
        match &self.reason {
            Some(r) => format!("Shutdown(reason='{}')", r),
            None => "Shutdown()".to_string(),
        }
    }
}

// EmitEvent is defined as a pure Python class (not a Rust pyclass) so that
// Python subclasses can call super().__init__() normally.
// See register_emit_event_class() in mod.rs.
