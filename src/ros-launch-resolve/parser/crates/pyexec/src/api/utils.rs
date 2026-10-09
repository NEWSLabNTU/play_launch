//! Shared utilities for Python API type conversions
//!
//! This module provides common helper functions for converting between
//! Python types and Rust types, particularly for handling ROS 2's
//! SomeSubstitutionsType pattern.

use pyo3::{
    IntoPyObjectExt,
    prelude::*,
    types::{PyDict, PyList, PyTuple},
};

/// A global parameter's value as `launch_ros` holds it: typed the way the
/// stored string reads (`1` an int, `1.0` a float, `true` a bool).
fn typed_parameter(py: Python, value_str: &str) -> PyResult<Py<PyAny>> {
    Ok(if let Ok(f) = value_str.parse::<f64>() {
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
        value_str.into_py_any(py)?
    })
}

/// The value `context.launch_configurations[name]` has in `launch`: a string
/// for an ordinary configuration, and the launch_ros entries that are not
/// strings — `ros_namespace` (the namespace pushed so far), `global_params`
/// and `ros_remaps` (lists of tuples) — read from where this context keeps
/// them.
fn configuration_value(py: Python, name: &str) -> PyResult<Option<Py<PyAny>>> {
    use play_launch_parser::bridge::with_launch_context;
    match name {
        "ros_namespace" => {
            let ns = with_launch_context(|ctx| ctx.current_namespace());
            Ok((ns != "/").then(|| ns.into_py_any(py)).transpose()?)
        }
        "global_params" => {
            let gp = with_launch_context(|ctx| ctx.global_parameters());
            if gp.is_empty() {
                return Ok(None);
            }
            let list = PyList::empty(py);
            for (k, v) in &gp {
                list.append(PyTuple::new(
                    py,
                    [k.into_py_any(py)?, typed_parameter(py, v)?],
                )?)?;
            }
            Ok(Some(list.into_any().unbind()))
        }
        "ros_remaps" => {
            let remaps = with_launch_context(|ctx| ctx.remappings());
            if remaps.is_empty() {
                return Ok(None);
            }
            let list = PyList::empty(py);
            for (a, b) in &remaps {
                list.append(PyTuple::new(py, [a, b])?)?;
            }
            Ok(Some(list.into_any().unbind()))
        }
        _ => with_launch_context(|ctx| ctx.get_configuration(name))
            .map(|v| v.into_py_any(py))
            .transpose(),
    }
}

fn configuration_names() -> Vec<String> {
    use play_launch_parser::bridge::with_launch_context;
    with_launch_context(|ctx| {
        let mut names: Vec<String> = ctx.configurations().into_keys().collect();
        if ctx.current_namespace() != "/" {
            names.push("ros_namespace".into());
        }
        if !ctx.global_parameters().is_empty() {
            names.push("global_params".into());
        }
        if !ctx.remappings().is_empty() {
            names.push("ros_remaps".into());
        }
        names.sort();
        names
    })
}

/// `context.launch_configurations`: a LIVE mapping over the launch context the
/// walk is executing in. An `OpaqueFunction` reads what the actions before it
/// set and may write what the actions after it read, as in `launch`; the mock
/// used to hand it a snapshot dict, and writes into it were lost.
#[pyclass(module = "launch.launch_context", mapping)]
pub struct LaunchConfigurations;

#[pymethods]
impl LaunchConfigurations {
    fn __getitem__(&self, py: Python, key: &str) -> PyResult<Py<PyAny>> {
        configuration_value(py, key)?
            .ok_or_else(|| pyo3::exceptions::PyKeyError::new_err(key.to_string()))
    }

    fn __setitem__(&self, py: Python, key: &str, value: Py<PyAny>) -> PyResult<()> {
        use play_launch_parser::bridge::with_launch_context;
        let text = pyobject_to_string(py, &value)?;
        with_launch_context(|ctx| ctx.set_configuration_literal(key.to_string(), text));
        Ok(())
    }

    fn __delitem__(&self, key: &str) {
        use play_launch_parser::bridge::with_launch_context;
        with_launch_context(|ctx| ctx.unset_configuration(key));
    }

    fn __contains__(&self, py: Python, key: &str) -> PyResult<bool> {
        Ok(configuration_value(py, key)?.is_some())
    }

    fn __len__(&self) -> usize {
        configuration_names().len()
    }

    fn __iter__(&self, py: Python) -> PyResult<Py<PyAny>> {
        Ok(PyList::new(py, configuration_names())?
            .into_any()
            .try_iter()?
            .into_any()
            .unbind())
    }

    #[pyo3(signature = (key, default = None))]
    fn get(&self, py: Python, key: &str, default: Option<Py<PyAny>>) -> PyResult<Py<PyAny>> {
        Ok(configuration_value(py, key)?.unwrap_or_else(|| default.unwrap_or_else(|| py.None())))
    }

    fn keys(&self) -> Vec<String> {
        configuration_names()
    }

    fn values(&self, py: Python) -> PyResult<Vec<Py<PyAny>>> {
        configuration_names()
            .iter()
            .map(|k| configuration_value(py, k).map(|v| v.unwrap_or_else(|| py.None())))
            .collect()
    }

    fn items(&self, py: Python) -> PyResult<Vec<(String, Py<PyAny>)>> {
        configuration_names()
            .into_iter()
            .map(|k| {
                let v = configuration_value(py, &k)?.unwrap_or_else(|| py.None());
                Ok((k, v))
            })
            .collect()
    }

    /// A plain dict snapshot, as `dict.copy()` would give.
    fn copy(&self, py: Python) -> PyResult<Py<PyAny>> {
        let d = PyDict::new(py);
        for (k, v) in self.items(py)? {
            d.set_item(k, v)?;
        }
        Ok(d.into_any().unbind())
    }

    #[pyo3(signature = (key, default = None))]
    fn setdefault(&self, py: Python, key: &str, default: Option<Py<PyAny>>) -> PyResult<Py<PyAny>> {
        if let Some(v) = configuration_value(py, key)? {
            return Ok(v);
        }
        let default = default.unwrap_or_else(|| py.None());
        self.__setitem__(py, key, default.clone_ref(py))?;
        Ok(default)
    }

    fn update(&self, py: Python, other: &Bound<'_, PyDict>) -> PyResult<()> {
        for (k, v) in other.iter() {
            self.__setitem__(py, &k.extract::<String>()?, v.unbind())?;
        }
        Ok(())
    }
}

/// The `context` an `OpaqueFunction` and every `perform(context)` receive: a
/// view onto the launch context the walk is executing in.
#[pyclass(module = "launch.launch_context")]
pub struct LaunchContextView;

#[pymethods]
impl LaunchContextView {
    #[getter]
    fn launch_configurations(&self, py: Python) -> PyResult<Py<LaunchConfigurations>> {
        Py::new(py, LaunchConfigurations)
    }

    /// `context.environment`: the process environment as the launch sees it
    /// so far — `os.environ` plus whatever `SetEnvironmentVariable` set.
    #[getter]
    fn environment(&self, py: Python) -> PyResult<Py<PyAny>> {
        use play_launch_parser::bridge::with_launch_context;
        let d = PyDict::new(py);
        for (k, v) in std::env::vars() {
            d.set_item(k, v)?;
        }
        for (k, v) in with_launch_context(|ctx| ctx.environment()) {
            d.set_item(k, v)?;
        }
        Ok(d.into_any().unbind())
    }

    /// `context.locals`: nothing this frontend models lives there.
    #[getter]
    fn locals(&self, py: Python) -> PyResult<Py<PyAny>> {
        Ok(py
            .import("types")?
            .getattr("SimpleNamespace")?
            .call0()?
            .unbind())
    }

    fn perform_substitution(slf: &Bound<'_, Self>, sub: &Bound<'_, PyAny>) -> PyResult<Py<PyAny>> {
        Ok(sub.call_method1("perform", (slf,))?.unbind())
    }
}

/// Resolve any `$(...)` tokens left in `s` — the string form some mock
/// substitutions use — against the context the walk is in. Text that does not
/// parse or resolve is returned unchanged.
pub fn resolve_tokens(s: &str) -> String {
    if !s.contains("$(") {
        return s.to_string();
    }
    use play_launch_parser::{
        bridge::try_with_launch_context,
        substitution::{parse_substitutions, resolve_substitutions},
    };
    let Ok(subs) = parse_substitutions(s) else {
        return s.to_string();
    };
    try_with_launch_context(|ctx| resolve_substitutions(&subs, ctx).ok())
        .flatten()
        .unwrap_or_else(|| s.to_string())
}

/// The `context` object handed to `perform()` and to an `OpaqueFunction`.
pub fn create_launch_context(py: Python) -> PyResult<Py<PyAny>> {
    Ok(Py::new(py, LaunchContextView)?.into_any())
}

/// Convert a Py<PyAny> to String, handling ROS 2's SomeSubstitutionsType pattern.
///
/// This function accepts three forms that match ROS 2's `SomeSubstitutionsType`:
/// 1. **Plain string**: `"literal_value"`
/// 2. **Substitution**: `LaunchConfiguration('var')` -> calls `__str__()` or `perform()`
/// 3. **List**: `[LaunchConfiguration('prefix'), '/suffix']` -> concatenates elements
///
/// This mirrors ROS 2's type definition:
/// ```python
/// SomeSubstitutionsType = Union[
///     Text,                              # Plain string
///     Substitution,                      # LaunchConfiguration, FindPackageShare, etc.
///     Iterable[Union[Text, Substitution]], # List of strings and/or substitutions
/// ]
/// ```
///
/// # Conditional Substitutions
///
/// Conditional substitutions (EqualsSubstitution, IfElseSubstitution, NotEqualsSubstitution, etc.)
/// need to evaluate during parsing to determine which path to take. These are handled specially:
/// - They call `perform()` with a real LaunchContext containing LaunchContext values
/// - This allows them to resolve LaunchConfiguration values and evaluate to "true"/"false"
///
/// # LaunchConfiguration Preservation
///
/// LaunchConfiguration substitutions are preserved as strings like "$(var node_name)" in the output.
/// This allows play_launch to re-resolve them at replay time with different values if needed.
///
/// # Examples
///
/// ```ignore
/// // Plain string
/// let result = pyobject_to_string(py, &py_string)?;
///
/// // LaunchConfiguration substitution
/// let result = pyobject_to_string(py, &launch_config)?;
/// // Result: "$(var variable_name)"
///
/// // Conditional substitution
/// let result = pyobject_to_string(py, &equals_sub)?;
/// // Result: "true" or "false" (evaluated with real context)
///
/// // List with mixed types
/// let result = pyobject_to_string(py, &py_list)?;
/// // Result: concatenated string from all elements
/// ```
///
/// # Conversion Strategy
///
/// The function tries multiple approaches in order:
/// 1. Direct string extraction (`extract::<String>()`)
/// 2. List handling (recursively convert and concatenate elements)
/// 3. Conditional substitutions: Call `perform()` with real context
/// 4. Other substitutions: Call `__str__()` to preserve substitution format
/// 5. Fallback to `to_string()` (Python repr)
///
/// # Errors
///
/// Returns `PyResult<String>` which will contain a PyErr if:
/// - The object cannot be converted to a string through any method
/// - Recursive conversion of list elements fails
///
/// # Performance
///
/// This function is called during launch file parsing (one-time cost),
/// not during node runtime, so performance impact is negligible.
pub fn pyobject_to_string(py: Python, obj: &Py<PyAny>) -> PyResult<String> {
    use pyo3::types::{PyBool, PyList};
    let obj_ref = obj.bind(py);

    // Try direct string extraction first (most common case)
    if let Ok(s) = obj_ref.extract::<String>() {
        return Ok(s);
    }

    // Handle booleans explicitly (convert to lowercase "true"/"false" for ROS2 compatibility)
    // Must check BEFORE integer extraction since Python bool is subclass of int
    if obj_ref.is_instance_of::<PyBool>()
        && let Ok(b) = obj_ref.extract::<bool>()
    {
        return Ok(if b { "true" } else { "false" }.to_string());
    }

    // Handle lists (concatenate elements recursively)
    if let Ok(list) = obj_ref.cast::<PyList>() {
        let mut result = String::new();
        for item in list.iter() {
            let item_str = pyobject_to_string(py, &item.into())?;
            result.push_str(&item_str);
        }
        return Ok(result);
    }

    // A LaunchConfiguration: its value now, else its default, else (unset,
    // which `launch` would refuse) its `$(var name)` spelling.
    if obj_ref
        .get_type()
        .name()
        .map(|n| n.to_string())
        .unwrap_or_default()
        == "LaunchConfiguration"
        && let Ok(name) = obj_ref.getattr("variable_name")
        && let Ok(name) = name.extract::<String>()
    {
        if let Some(value) =
            play_launch_parser::bridge::try_with_launch_context(|ctx| ctx.get_configuration(&name))
                .flatten()
        {
            return Ok(value);
        }
        if let Ok(default) = obj_ref.getattr("default")
            && !default.is_none()
        {
            return pyobject_to_string(py, &default.unbind());
        }
        return Ok(format!("$(var {name})"));
    }

    // Any other substitution is PERFORMED now, in the context the walk is in —
    // which is when `launch` performs it.
    if obj_ref.hasattr("perform")? {
        let context = create_launch_context(py)?;
        if let Ok(result) = obj_ref.call_method1("perform", (context,))
            && let Ok(s) = result.extract::<String>()
        {
            return Ok(resolve_tokens(&s));
        }
    }

    // Mocks that spell themselves as a `$(...)` token are resolved by the
    // substitution engine, against the same context.
    if let Ok(str_result) = obj_ref.call_method0("__str__")
        && let Ok(s) = str_result.extract::<String>()
    {
        return Ok(resolve_tokens(&s));
    }

    // Fallback to Python repr
    Ok(obj_ref.str()?.to_string())
}

/// Try `perform(context)` on a Py<PyAny>, then fall back to string conversion.
///
/// Used by substitution structs that need to resolve operands which may themselves
/// be substitutions (e.g., EqualsSubstitution comparing two LaunchConfigurations).
pub fn perform_or_to_string(
    obj: &Py<PyAny>,
    py: Python,
    context: &Bound<'_, PyAny>,
) -> PyResult<String> {
    let obj_ref = obj.bind(py);

    // Try perform() first (resolves nested substitutions)
    if obj_ref.hasattr("perform")?
        && let Ok(result) = obj_ref.call_method1("perform", (context,))
        && let Ok(s) = result.extract::<String>()
    {
        return Ok(s);
    }

    // Fallback to general string conversion
    pyobject_to_string(py, obj)
}

/// Convert a Py<PyAny> to a boolean value.
///
/// Handles bool extraction, string-to-bool conversion ("true"/"1"/"yes"),
/// and __str__() fallback. Used by And/Or/IfElse substitutions.
pub fn pyobject_to_bool(obj: &Py<PyAny>, py: Python) -> PyResult<bool> {
    if let Ok(b) = obj.extract::<bool>(py) {
        return Ok(b);
    }

    if let Ok(s) = obj.extract::<String>(py) {
        return Ok(matches!(s.to_lowercase().as_str(), "true" | "1" | "yes"));
    }

    if let Ok(str_result) = obj.call_method0(py, "__str__")
        && let Ok(s) = str_result.extract::<String>(py)
    {
        return Ok(matches!(s.to_lowercase().as_str(), "true" | "1" | "yes"));
    }

    Ok(false)
}

#[cfg(test)]
mod tests {
    use super::*;
    use pyo3::IntoPyObjectExt;

    #[test]
    fn test_pyobject_to_string_plain_string() {
        pyo3::Python::initialize();
        Python::attach(|py| {
            let s = "hello world";
            let py_str = s.into_py_any(py).unwrap();
            let result = pyobject_to_string(py, &py_str).unwrap();
            assert_eq!(result, "hello world");
        });
    }

    #[test]
    fn test_pyobject_to_string_list() {
        pyo3::Python::initialize();
        Python::attach(|py| {
            // Create a list: ["hello", " ", "world"]
            let list = pyo3::types::PyList::new(py, ["hello", " ", "world"]).unwrap();
            let py_list = list.into_any().unbind();
            let result = pyobject_to_string(py, &py_list).unwrap();
            assert_eq!(result, "hello world");
        });
    }

    #[test]
    fn test_pyobject_to_string_number() {
        pyo3::Python::initialize();
        Python::attach(|py| {
            let num: i64 = 42;
            let py_num = num.into_py_any(py).unwrap();
            let result = pyobject_to_string(py, &py_num).unwrap();
            assert_eq!(result, "42");
        });
    }
}
