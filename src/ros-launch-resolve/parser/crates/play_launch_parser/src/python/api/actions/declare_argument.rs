//! DeclareLaunchArgument action

use pyo3::prelude::*;

/// Mock DeclareLaunchArgument action
///
/// Python equivalent:
/// ```python
/// from launch.actions import DeclareLaunchArgument
/// arg = DeclareLaunchArgument(
///     'variable_name',
///     default_value='default',
///     description='Description of argument'
/// )
/// ```
///
/// For now, this is a placeholder. Launch arguments are typically
/// passed via command line, not captured from Python files.
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct DeclareLaunchArgument {
    name: String,
    #[allow(dead_code)] // Keep for future use when we fully support launch arguments
    default_value: Option<String>,
    #[allow(dead_code)] // Keep for future use when we fully support launch arguments
    description: Option<String>,
}

#[pymethods]
impl DeclareLaunchArgument {
    #[new]
    #[pyo3(signature = (name, *, default_value=None, description=None, **_kwargs))]
    fn new(
        py: Python,
        name: String,
        default_value: Option<Py<PyAny>>,
        description: Option<String>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> PyResult<Self> {
        use crate::bridge::with_launch_context;

        // Convert default_value Py<PyAny> to string (may be string, substitution, or list)
        // Special case: when default_value is a LaunchConfiguration with the same name,
        // we need to call perform() to resolve it using the LaunchConfiguration's own
        // stored default (not the context lookup, which would be circular).
        let default_str = default_value
            .map(|dv| {
                let obj_ref = dv.bind(py);
                let type_name = obj_ref
                    .get_type()
                    .name()
                    .map(|n| n.to_string())
                    .unwrap_or_default();
                if type_name == "LaunchConfiguration" {
                    // Try perform() which handles self-referencing defaults
                    let context = crate::python::api::utils::create_launch_context(py)?;
                    if let Ok(result) = obj_ref.call_method1("perform", (context,))
                        && let Ok(s) = result.extract::<String>()
                    {
                        return Ok(s);
                    }
                }
                Self::pyobject_to_string(py, &dv)
            })
            .transpose()?;

        // Register the default value in LaunchContext if not already set
        if let Some(ref default_val) = default_str {
            with_launch_context(|ctx| {
                // Only set if not already present (CLI args and include args take precedence)
                if ctx.get_configuration(&name).is_none() {
                    ctx.set_configuration(name.clone(), default_val.clone());
                    log::debug!(
                        "Registered launch configuration '{}' with default value '{}'",
                        name,
                        default_val
                    );
                } else {
                    log::debug!(
                        "Launch configuration '{}' already set, not overriding with default '{}'",
                        name,
                        default_val
                    );
                }
            });
        }

        Ok(Self {
            name,
            default_value: default_str,
            description,
        })
    }

    fn __repr__(&self) -> String {
        format!("DeclareLaunchArgument('{}')", self.name)
    }
}

impl DeclareLaunchArgument {
    /// Convert Py<PyAny> to string (handles strings, substitutions, and lists)
    /// For substitutions, this will call perform() to resolve them
    fn pyobject_to_string(py: Python, obj: &Py<PyAny>) -> PyResult<String> {
        crate::python::api::utils::pyobject_to_string(py, obj)
    }
}
