//! IncludeLaunchDescription action

use crate::bridge::capture_include;
use pyo3::prelude::*;

/// Convert Py<PyAny> to string for launch arguments (handles strings, lists, substitutions)
/// CRITICAL: Unlike pyobject_to_string, this function RESOLVES LaunchConfiguration objects
/// instead of preserving them as "$(var name)" strings. This is necessary because include
/// arguments are used to set variables in the included file's context, not for replay.
fn pyobject_to_string_for_include_args(py: Python, obj: &Bound<'_, PyAny>) -> PyResult<String> {
    use pyo3::types::PyList;

    // Try direct string extraction first
    if let Ok(s) = obj.extract::<String>() {
        return Ok(s);
    }

    // Handle lists (concatenate elements recursively)
    if let Ok(list) = obj.cast::<PyList>() {
        let mut result = String::new();
        for item in list.iter() {
            let item_str = pyobject_to_string_for_include_args(py, &item)?;
            result.push_str(&item_str);
        }
        return Ok(result);
    }

    // CRITICAL: For LaunchConfiguration, resolve it using LaunchContext
    // This is different from regular parameter handling where we preserve the substitution
    let type_name = obj.get_type().name()?.to_string();
    if type_name == "LaunchConfiguration" {
        use crate::bridge::with_launch_context;

        // Try to get the variable name
        if let Ok(name_obj) = obj.getattr("variable_name")
            && let Ok(var_name) = name_obj.extract::<Vec<String>>()
            && var_name.len() == 1
        {
            // Look up the value in LaunchContext
            let resolved = with_launch_context(|ctx| ctx.get_configuration(&var_name[0]));
            if let Some(value) = resolved {
                log::debug!(
                    "Resolving LaunchConfiguration('{}') for include arg: '{}'",
                    var_name[0],
                    value
                );
                return Ok(value);
            } else {
                log::warn!(
                    "LaunchConfiguration('{}') not found in context for include arg",
                    var_name[0]
                );
            }
        }
    }

    // For other substitutions, try calling perform() with context
    if let Ok(perform_method) = obj.getattr("perform") {
        let context = crate::python::api::utils::create_launch_context(py)?;
        if let Ok(result) = perform_method.call1((context,))
            && let Ok(s) = result.extract::<String>()
        {
            return Ok(s);
        }
    }

    // Fallback: use __str__()
    if let Ok(str_result) = obj.call_method0("__str__")
        && let Ok(s) = str_result.extract::<String>()
    {
        return Ok(s);
    }

    Ok(obj.str()?.to_string())
}

/// Mock IncludeLaunchDescription action
///
/// Python equivalent:
/// ```python
/// from launch.actions import IncludeLaunchDescription
/// from launch.launch_description_sources import PythonLaunchDescriptionSource
///
/// include = IncludeLaunchDescription(
///     PythonLaunchDescriptionSource([path]),
///     launch_arguments={'arg': 'value'}.items()
/// )
/// ```
///
/// Includes another launch file (Python, XML, or YAML)
#[pyclass(module = "launch.actions", from_py_object)]
#[derive(Clone)]
pub struct IncludeLaunchDescription {
    #[allow(dead_code)] // Stored for API compatibility, used during construction
    launch_description_source: Py<PyAny>,
    #[allow(dead_code)] // Stored for API compatibility, used during construction
    launch_arguments: Vec<(String, String)>,
}

#[pymethods]
impl IncludeLaunchDescription {
    #[new]
    #[pyo3(signature = (launch_description_source, *, launch_arguments=None, **_kwargs))]
    fn new(
        py: Python,
        launch_description_source: Py<PyAny>,
        launch_arguments: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> PyResult<Self> {
        // Parse launch_arguments
        let mut args = Vec::new();
        if let Some(launch_args_obj) = launch_arguments {
            // Try to extract as dict
            if let Ok(dict) = launch_args_obj.bind(py).cast::<pyo3::types::PyDict>() {
                for (key, value) in dict.iter() {
                    let key_str = key.extract::<String>()?;
                    // Try to extract value as string, list, or substitution
                    let value_str = pyobject_to_string_for_include_args(py, &value)?;
                    args.push((key_str, value_str));
                }
            }
            // Try to extract as list/iterator of tuples
            else if let Ok(iter) = launch_args_obj.bind(py).try_iter() {
                for item in iter {
                    let item = item?;
                    if let Ok(tuple) = item.cast::<pyo3::types::PyTuple>()
                        && tuple.len() == 2
                    {
                        let key = tuple.get_item(0)?.extract::<String>()?;
                        let value_obj = tuple.get_item(1)?;
                        // Try to extract value as string, list, or substitution
                        let value = pyobject_to_string_for_include_args(py, &value_obj)?;
                        args.push((key, value));
                    }
                }
            }
        }

        // Extract file path from launch_description_source
        // Get the context from __main__ namespace if available (created by OpaqueFunction)
        let context = py
            .import("__main__")
            .ok()
            .and_then(|m| m.dict().get_item("context").ok().flatten());

        let file_path = if let Ok(path_str) = launch_description_source.extract::<String>(py) {
            // Direct string path
            path_str
        } else {
            // Try get_launch_file_path which will now use the context from __main__
            if let Ok(path_str) = launch_description_source.call_method0(py, "get_launch_file_path")
            {
                path_str
                    .extract::<String>(py)
                    .unwrap_or_else(|_| "unknown".to_string())
            } else if let Some(ctx) = context {
                // Fallback: try calling perform with context
                if let Ok(performed) = launch_description_source.call_method1(py, "perform", (ctx,))
                {
                    performed
                        .extract::<String>(py)
                        .unwrap_or_else(|_| "unknown".to_string())
                } else {
                    "unknown".to_string()
                }
            } else {
                "unknown".to_string()
            }
        };

        // Capture the include request with current ROS namespace
        {
            use crate::bridge::get_current_ros_namespace;
            let ros_namespace = get_current_ros_namespace();

            log::debug!("Capturing include with ROS namespace: '{}'", ros_namespace);

            capture_include(crate::captures::IncludeCapture {
                file_path: file_path.clone(),
                args: args.clone(),
                ros_namespace,
            });
        }

        log::debug!(
            "Python Launch IncludeLaunchDescription: file_path='{}' with {} args",
            file_path,
            args.len()
        );

        Ok(Self {
            launch_description_source,
            launch_arguments: args,
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "IncludeLaunchDescription({} args)",
            self.launch_arguments.len()
        )
    }
}
