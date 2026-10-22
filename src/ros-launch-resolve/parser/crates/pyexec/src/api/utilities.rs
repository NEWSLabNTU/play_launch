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
pub fn prefix_namespace(py: Python, prefix: Py<PyAny>, name: String) -> PyResult<String> {
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

/// `launch.utilities.normalize_to_list_of_substitutions`: a string becomes a
/// `TextSubstitution`, a substitution a one-element list, an iterable its
/// elements normalized the same way.
#[pyfunction]
pub fn normalize_to_list_of_substitutions(
    py: Python,
    subs: &Bound<'_, PyAny>,
) -> PyResult<Vec<Py<PyAny>>> {
    let text_sub = py
        .import("launch.substitutions")?
        .getattr("TextSubstitution")?;
    let norm = |x: &Bound<'_, PyAny>| -> PyResult<Py<PyAny>> {
        if x.is_instance_of::<pyo3::types::PyString>() {
            let kwargs = pyo3::types::PyDict::new(py);
            kwargs.set_item("text", x)?;
            Ok(text_sub.call((), Some(&kwargs))?.unbind())
        } else {
            Ok(x.clone().unbind())
        }
    };
    if subs.is_instance_of::<pyo3::types::PyString>() || subs.hasattr("perform")? {
        return Ok(vec![norm(subs)?]);
    }
    subs.try_iter()?.map(|x| norm(&x?)).collect()
}

/// `launch.utilities.perform_substitutions(context, subs)`: each element
/// performed in the walk's context and concatenated. Goes through
/// `pyobject_to_string`, so a mock `LaunchConfiguration` reads the
/// configuration the walk holds, exactly as everywhere else.
#[pyfunction]
pub fn perform_substitutions(
    py: Python,
    _context: &Bound<'_, PyAny>,
    subs: &Bound<'_, PyAny>,
) -> PyResult<String> {
    let mut out = String::new();
    for sub in subs.try_iter()? {
        out.push_str(&crate::api::utils::pyobject_to_string(py, &sub?.unbind())?);
    }
    Ok(out)
}
