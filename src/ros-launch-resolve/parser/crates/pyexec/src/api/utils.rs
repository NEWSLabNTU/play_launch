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

/// Create a LaunchContext-like Python object with access to launch configurations
/// This allows substitutions to resolve LaunchConfiguration values during perform()
pub fn create_launch_context(py: Python) -> PyResult<Py<PyAny>> {
    use play_launch_parser::bridge::with_launch_context;

    // Get resolved configurations and global parameters from the thread-local LaunchContext
    let configs = with_launch_context(|ctx| ctx.configurations());
    let global_params = with_launch_context(|ctx| ctx.global_parameters());

    // Create a simple context object that has a launch_configurations dict
    let context_class = py.eval(
        c"type('Context', (), {
            'launch_configurations': None,
            '__init__': lambda self, configs: setattr(self, 'launch_configurations', configs)
        })",
        None,
        None,
    )?;

    // Create a Python dict from our configurations
    let py_configs = PyDict::new(py);
    for (key, value) in &configs {
        py_configs.set_item(key, value)?;
    }

    // Include global parameters (from SetParameter actions) as 'global_params'
    // This matches real ROS 2 behavior: SetParameter.execute() stores parameters
    // as context.launch_configurations['global_params'] = [(name, value), ...]
    // Autoware code accesses them as: dict(context.launch_configurations.get("global_params", {}))
    if !global_params.is_empty() {
        let gp_list = PyList::empty(py);
        for (name, value_str) in &global_params {
            // Convert string values back to typed Python values (float/int/str)
            // so arithmetic in Python launch files works correctly
            let py_value: Py<PyAny> = if let Ok(f) = value_str.parse::<f64>() {
                // Check if it could be an integer (no decimal point in original)
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
            let tuple = PyTuple::new(py, [name.as_str().into_py_any(py)?, py_value])?;
            gp_list.append(&tuple)?;
        }
        py_configs.set_item("global_params", gp_list)?;
    }

    // Instantiate the context with our configs
    let context = context_class.call1((py_configs,))?;

    Ok(context.into())
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

    // Check if this is a substitution that needs to be evaluated during parsing
    // These substitutions need to call perform() to resolve their content/logic
    if is_evaluating_substitution(obj_ref)? {
        // Create a real launch context with access to LaunchContext
        let context = create_launch_context(py)?;

        // Call perform() to evaluate the condition
        if let Ok(result) = obj_ref.call_method1("perform", (context,))
            && let Ok(s) = result.extract::<String>()
        {
            return Ok(s);
        }
    }

    // Check if this is a LaunchConfiguration — try resolving from thread-local context
    // Some LaunchConfigurations (e.g., container_executable) are set via SetLaunchConfiguration
    // and should be resolved at parse time rather than preserved as $(var name)
    {
        let type_name = obj_ref
            .get_type()
            .name()
            .map(|n| n.to_string())
            .unwrap_or_default();
        if type_name == "LaunchConfiguration"
            && let Ok(str_result) = obj_ref.call_method0("__str__")
            && let Ok(s) = str_result.extract::<String>()
        {
            // s is "$(var name)" — extract 'name' and look up in context
            if let Some(var_name) = s.strip_prefix("$(var ").and_then(|s| s.strip_suffix(')'))
                && let Some(value) = play_launch_parser::bridge::try_with_launch_context(|ctx| {
                    ctx.get_configuration(var_name)
                })
                .flatten()
            {
                return Ok(value);
            }
            // Not in context — preserve as $(var name) for runtime resolution
            return Ok(s);
        }
    }

    // For other substitution objects, use __str__()
    // This preserves substitutions in the record.json output
    // The substitutions will be resolved later during actual launch execution
    if let Ok(str_result) = obj_ref.call_method0("__str__")
        && let Ok(s) = str_result.extract::<String>()
    {
        return Ok(s);
    }

    // Fallback to Python repr
    Ok(obj_ref.str()?.to_string())
}

/// Check if a Py<PyAny> is a substitution that needs evaluation during parsing.
///
/// Substitutions that need evaluation include:
/// - **Conditional substitutions**: EqualsSubstitution, NotEqualsSubstitution, IfElseSubstitution,
///   AndSubstitution, OrSubstitution, NotSubstitution (evaluate to "true"/"false")
/// - **Content substitutions**: FileContent (reads file and resolves nested substitutions),
///   PathJoinSubstitution (joins paths and resolves nested substitutions)
/// - **Expression substitutions**: PythonExpression (evaluates Python code)
///
/// These need to call perform() with real context during parsing to:
/// - Evaluate conditional logic (for conditionals)
/// - Resolve nested substitutions (for FileContent, PathJoinSubstitution)
/// - Read external content (for FileContent)
/// - Execute Python expressions (for PythonExpression)
///
/// LaunchConfiguration is NOT in this list - it should be preserved as "$(var name)"
/// for replay-time resolution.
fn is_evaluating_substitution(obj: &Bound<'_, PyAny>) -> PyResult<bool> {
    let type_name = obj.get_type().name()?.to_string();
    Ok(matches!(
        type_name.as_str(),
        // Conditional substitutions
        "EqualsSubstitution"
            | "NotEqualsSubstitution"
            | "IfElseSubstitution"
            | "AndSubstitution"
            | "OrSubstitution"
            | "NotSubstitution"
            // Content substitutions that need evaluation
            | "FileContent"
            | "PathJoinSubstitution"
            // Expression substitutions
            | "PythonExpression"
    ))
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
