//! Mock `launch_ros.utilities` module functions

use pyo3::prelude::*;

/// Mock make_namespace_absolute function
///
/// Python equivalent:
/// ```python
/// from launch_ros.utilities import make_namespace_absolute
/// abs_ns = make_namespace_absolute(namespace)
/// ```
///
/// Ensures namespace has a leading slash
#[pyfunction]
pub fn make_namespace_absolute(namespace: String) -> String {
    if namespace.is_empty() {
        "/".to_string()
    } else if namespace.starts_with('/') {
        namespace
    } else {
        format!("/{}", namespace)
    }
}

/// Mock prefix_namespace function
///
/// Python equivalent:
/// ```python
/// from launch_ros.utilities import prefix_namespace
/// prefixed = prefix_namespace(prefix, name)
/// ```
///
/// Joins a namespace prefix with a name
#[pyfunction]
#[pyo3(signature = (prefix, name))]
pub fn prefix_namespace(py: Python, prefix: PyObject, name: String) -> PyResult<String> {
    // Handle None prefix (treat as empty string)
    let prefix_str = if prefix.is_none(py) {
        String::new()
    } else {
        prefix.extract::<String>(py)?
    };

    let prefix = make_namespace_absolute(prefix_str);

    if name.is_empty() {
        Ok(prefix)
    } else if name.starts_with('/') {
        Ok(name)
    } else if prefix == "/" {
        Ok(format!("/{}", name))
    } else {
        Ok(format!("{}/{}", prefix, name))
    }
}

/// Register utilities module in Python
pub fn register_utilities_module(py: Python) -> PyResult<()> {
    let utilities_module = PyModule::new(py, "utilities")?;
    utilities_module.add_function(wrap_pyfunction!(
        make_namespace_absolute,
        &utilities_module
    )?)?;
    utilities_module.add_function(wrap_pyfunction!(prefix_namespace, &utilities_module)?)?;

    // Add utilities as a submodule of launch_ros
    let launch_ros = py.import("launch_ros")?;
    launch_ros.add_submodule(&utilities_module)?;

    Ok(())
}
