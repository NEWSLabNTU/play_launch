//! `$(eval …)` — Python evaluation, per the ROS 2 launch specification.
//!
//! Lives here rather than in `substitution/` because it is the only thing in
//! that module that needs an interpreter, and `substitution/` is core (0897 W2).

pub(crate) fn eval_expr_pyo3(expr: &str) -> Result<String, crate::error::SubstitutionError> {
    use pyo3::{prelude::*, types::PyDict};

    // Unescaping happens in `evaluate_expression`, before the delimiter
    // decision that depends on it.

    // Convert ROS-style boolean literals to Python-style before eval.
    // ROS uses lowercase true/false, Python uses True/False.
    // Only replace standalone words (not inside quotes).
    let expr = crate::substitution::eval::replace_ros_booleans(expr);
    let expr = expr.as_str();

    // Trim whitespace — Python's compile('eval') rejects leading spaces
    // with IndentationError, even though eval() as a function accepts them.
    let expr = expr.trim();

    log::debug!("Evaluating $(eval {})", expr);

    Python::attach(|py| {
        let expr_cstr = std::ffi::CString::new(expr).map_err(|e| {
            crate::error::SubstitutionError::InvalidSubstitution(format!(
                "Expression contains null byte: {}",
                e
            ))
        })?;

        // Build restricted globals — only safe, pure builtins.
        // This prevents __import__, exec, eval, open, compile etc.
        let restricted_builtins = PyDict::new(py);
        for name in [
            "True",
            "False",
            "None",
            "int",
            "float",
            "str",
            "bool",
            "list",
            "tuple",
            "dict",
            "set",
            "len",
            "abs",
            "min",
            "max",
            "round",
            "sorted",
            "reversed",
            "enumerate",
            "zip",
            "map",
            "filter",
            "range",
            "isinstance",
            "type",
            "hasattr",
            "getattr",
            "repr",
            "format",
            "chr",
            "ord",
            "hex",
            "oct",
            "bin",
        ] {
            if let Ok(builtin) = py.import("builtins").and_then(|b| b.getattr(name)) {
                let _ = restricted_builtins.set_item(name, builtin);
            }
        }
        let globals = PyDict::new(py);
        let _ = globals.set_item("__builtins__", restricted_builtins);

        let result = py.eval(&expr_cstr, Some(&globals), None).map_err(|e| {
            crate::error::SubstitutionError::InvalidSubstitution(format!(
                "Failed to evaluate expression '{}': {}",
                expr, e
            ))
        })?;

        // Convert Python result to string
        let s = result.str().map_err(|e| {
            crate::error::SubstitutionError::InvalidSubstitution(format!(
                "Failed to convert eval result to string: {}",
                e
            ))
        })?;
        let value = s.to_str().map_err(|e| {
            crate::error::SubstitutionError::InvalidSubstitution(format!(
                "Failed to extract string: {}",
                e
            ))
        })?;

        // Normalize Python booleans to lowercase for ROS compatibility
        let normalized = match value {
            "True" => "true".to_string(),
            "False" => "false".to_string(),
            other => other.to_string(),
        };

        Ok(normalized)
    })
}
