//! OpaqueFunction action

use pyo3::{IntoPyObjectExt, prelude::*};

/// Mock OpaqueFunction action
///
/// Python equivalent:
/// ```python
/// from launch.actions import OpaqueFunction
/// def my_function(context):
///     return [Node(...), ...]
/// opaque = OpaqueFunction(function=my_function)
/// ```
///
/// This action executes a Python function and captures the returned actions.
#[pyclass(module = "launch.actions")]
#[derive(Clone)]
pub struct OpaqueFunction {
    function: Option<PyObject>,
}

#[pymethods]
impl OpaqueFunction {
    #[new]
    #[pyo3(signature = (*, function=None, **_kwargs))]
    fn new(
        function: Option<PyObject>,
        _kwargs: Option<&Bound<'_, pyo3::types::PyDict>>,
    ) -> PyResult<Self> {
        Ok(Self { function })
    }

    fn __repr__(&self) -> String {
        "OpaqueFunction(...)".to_string()
    }

    /// Execute the function and return the result
    /// This is called by our executor
    pub fn execute(&self, py: Python) -> PyResult<PyObject> {
        log::debug!("OpaqueFunction::execute called");
        if let Some(ref func) = self.function {
            log::debug!("OpaqueFunction has function, executing it");
            // Create a mock LaunchContext that provides access to launch configurations
            // This allows OpaqueFunction code to call LaunchConfiguration().perform(context)
            use crate::python::bridge::{get_current_ros_namespace, with_launch_context};
            let configs = with_launch_context(|ctx| ctx.configurations());
            let global_params = with_launch_context(|ctx| ctx.global_parameters());
            let ros_namespace = get_current_ros_namespace();

            log::debug!("OpaqueFunction context: ros_namespace='{}'", ros_namespace);

            // Helper function to resolve substitution strings from Rust
            #[pyfunction]
            fn resolve_substitution_string(sub_string: String) -> PyResult<String> {
                use crate::substitution::{
                    context::LaunchContext, parser::parse_substitutions,
                    types::resolve_substitutions,
                };

                // Parse the substitution string (e.g., "$(find-pkg-share pkg)/path")
                let subs = parse_substitutions(&sub_string).map_err(|e| {
                    pyo3::exceptions::PyValueError::new_err(format!(
                        "Failed to parse substitution: {}",
                        e
                    ))
                })?;

                // Resolve the substitutions
                let ctx = LaunchContext::new();
                resolve_substitutions(&subs, &ctx).map_err(|e| {
                    pyo3::exceptions::PyFileNotFoundError::new_err(format!(
                        "Failed to resolve substitution '{}': {}",
                        sub_string, e
                    ))
                })
            }

            // Create a context dict with launch_configurations and ros_namespace
            let context_code = r#"
class MockLaunchContext:
    def __init__(self, launch_configurations, ros_namespace, resolve_sub_fn):
        self.launch_configurations = launch_configurations
        # Store ros_namespace for OpaqueFunction access (e.g. context.launch_configurations.get('ros_namespace'))
        self.launch_configurations['ros_namespace'] = ros_namespace
        self.resolve_sub_fn = resolve_sub_fn

    def perform_substitution(self, sub):
        # Support LaunchConfiguration.perform(context)
        if hasattr(sub, 'variable_name'):
            value = self.launch_configurations.get(sub.variable_name, '')
            # If the value contains substitution syntax like $(...), resolve it
            if isinstance(value, str) and '$(' in value:
                try:
                    return self.resolve_sub_fn(value)
                except Exception as e:
                    # If resolution fails, return the original value
                    pass
            return value

        # Support FindPackageShare.perform(context)
        if hasattr(sub, 'package_name'):
            # Get the package name (might be a string or another substitution)
            package_name = sub.package_name
            if hasattr(package_name, 'perform'):
                package_name = package_name.perform(self)
            elif hasattr(package_name, '__iter__') and not isinstance(package_name, str):
                # It's a list of substitutions
                package_name = ''.join(str(self.perform_substitution(s)) for s in package_name)
            else:
                package_name = str(package_name)

            # Resolve using Rust function (construct a find-pkg-share substitution string)
            try:
                sub_str = f"$(find-pkg-share {package_name})"
                return self.resolve_sub_fn(sub_str)
            except Exception as e:
                # Fallback to string representation if resolution fails
                return str(sub)

        return str(sub)

context = MockLaunchContext(launch_configurations, ros_namespace, resolve_substitution_string)
"#;

            // Use __main__.dict() as namespace - this is the same namespace used by exec()
            // and has access to the isolated sys.modules with our mocks
            let main_module = py.import("__main__")?;
            let namespace = main_module.dict();

            // Add launch configurations to namespace
            let configs_dict = pyo3::types::PyDict::new(py);
            for (key, value) in configs {
                configs_dict.set_item(key, value)?;
            }

            // Include global parameters (from SetParameter actions) as 'global_params'
            // Real ROS 2 SetParameter.execute() stores params as
            // context.launch_configurations['global_params'] = [(name, value), ...]
            // Autoware code accesses: dict(context.launch_configurations.get("global_params", {}))
            if !global_params.is_empty() {
                let gp_list = pyo3::types::PyList::empty(py);
                for (name, value_str) in &global_params {
                    let py_value: PyObject = if let Ok(f) = value_str.parse::<f64>() {
                        if !value_str.contains('.') {
                            if let Ok(i) = value_str.parse::<i64>() {
                                i.into_py_any(py)?
                            } else {
                                f.into_py_any(py)?
                            }
                        } else {
                            f.into_py_any(py)?
                        }
                    } else if value_str == "True" || value_str == "true" {
                        true.into_py_any(py)?
                    } else if value_str == "False" || value_str == "false" {
                        false.into_py_any(py)?
                    } else {
                        value_str.as_str().into_py_any(py)?
                    };
                    let tuple =
                        pyo3::types::PyTuple::new(py, [name.as_str().into_py_any(py)?, py_value])?;
                    gp_list.append(tuple)?;
                }
                configs_dict.set_item("global_params", gp_list)?;
            }

            namespace.set_item("launch_configurations", configs_dict)?;

            // Add ros_namespace to namespace
            namespace.set_item("ros_namespace", ros_namespace)?;

            // Add the Rust substitution resolution function to namespace
            let resolve_fn = wrap_pyfunction!(resolve_substitution_string, py)?;
            namespace.set_item("resolve_substitution_string", resolve_fn)?;

            // Add Python builtins that OpaqueFunction might need
            // Import Path for file operations
            py.run(
                c"from pathlib import Path",
                Some(&namespace),
                Some(&namespace),
            )?;
            // Import yaml for YAML loading
            py.run(c"import yaml", Some(&namespace), Some(&namespace))
                .ok(); // Ignore if yaml not available

            // Create the context using the code above
            let context_cstr = std::ffi::CString::new(context_code).map_err(|e| {
                pyo3::exceptions::PyValueError::new_err(format!(
                    "context code contains null byte: {}",
                    e
                ))
            })?;
            py.run(&context_cstr, Some(&namespace), Some(&namespace))?;
            let context = namespace.get_item("context")?.unwrap();

            // Call the function with the context
            log::debug!("Calling OpaqueFunction with context");
            let result = func.call1(py, (context,))?;

            // Log the result type
            let result_type = result
                .bind(py)
                .get_type()
                .name()
                .map(|n| n.to_string())
                .unwrap_or_else(|_| "unknown".to_string());
            log::debug!("OpaqueFunction returned type: {}", result_type);

            // If it's a list, log the count
            if let Ok(list) = result.bind(py).extract::<Vec<PyObject>>() {
                log::debug!("OpaqueFunction returned list with {} items", list.len());
            }

            Ok(result)
        } else {
            Ok(py.None())
        }
    }
}
