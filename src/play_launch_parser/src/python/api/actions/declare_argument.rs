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
#[pyclass(module = "launch.actions")]
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
        default_value: Option<PyObject>,
        description: Option<String>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> PyResult<Self> {
        use crate::python::bridge::with_launch_context;

        // Convert default_value PyObject to string (may be string, substitution, or list)
        let default_str = default_value
            .map(|dv| Self::pyobject_to_string(py, &dv))
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
    /// Convert PyObject to string (handles strings, substitutions, and lists)
    /// For substitutions, this will call perform() to resolve them
    fn pyobject_to_string(py: Python, obj: &PyObject) -> PyResult<String> {
        crate::python::api::utils::pyobject_to_string(py, obj)
    }
}
