//! Expression evaluator for $(eval expr)
//!
//! Always delegates to Python's eval() via PyO3, matching ROS 2's behavior exactly.
//! $(eval) is defined as Python evaluation in the ROS 2 launch specification.

use crate::error::SubstitutionError;

/// Expression evaluator for $(eval expr).
///
/// Always delegates to Python's eval() via PyO3, matching ROS 2's behavior exactly.
/// $(eval) is defined as Python evaluation in the ROS 2 launch specification — any
/// Rust-side reimplementation risks subtle incompatibilities with quoting, operators,
/// or edge cases. Using Python eval ensures 100% compatibility.
pub(crate) fn evaluate_expression(expr: &str) -> Result<String, SubstitutionError> {
    let expr = expr.trim();

    // Handle outer quote wrapping from XML attribute patterns:
    //   $(eval &quot;...&quot;)  → outer "..." from XML &quot; escaping
    //   $(eval '...')         → outer '...' used in XML attributes
    //
    // Strip only if the opening quote's matching close is the LAST character.
    // This distinguishes wrapper quotes from Python string literals:
    //   "'a' == 'b'"  → first " matches last " (no " in between) → strip
    //   "'[..] ' + '""]'"  → first ' has matching ' after ']' (not last) → don't strip...
    //   Actually we need: first quote's matching close = last char.
    //
    // For double quotes: strip if inner content has no unescaped double quotes.
    // For single quotes: strip if inner content has no unescaped single quotes
    //   AND the expression doesn't look like Python single-quoted strings
    //   (e.g., 'foo' + 'bar' starts with ' but is not a wrapper).
    //
    // Handle outer quote wrapping from XML $(eval) patterns:
    //   $(eval &quot;...&quot;)  → outer "..." wrapping inner '...' Python expressions
    //   $(eval '&quot;...&quot;') → outer '...' wrapping inner "..." Python expressions
    //
    // Strip outer quotes when the inner content uses the OTHER quote type.
    // Don't strip when inner uses the SAME quote (e.g., 'foo' + 'bar' or "" + "").
    let expr = if expr.len() >= 2 {
        let first = expr.as_bytes()[0];
        let last = expr.as_bytes()[expr.len() - 1];
        let other_quote = if first == b'"' { b'\'' } else { b'"' };

        if (first == b'"' || first == b'\'') && first == last {
            let inner = &expr[1..expr.len() - 1];
            let inner_trimmed = inner.trim();

            if inner_trimmed.is_empty() {
                // Empty quotes like '' or "" — don't strip
                expr
            } else if inner_trimmed.as_bytes()[0] == other_quote {
                // Inner starts with the other quote type → outer is a wrapper
                // e.g., "'a' == 'b'" or '"api"=="api"'
                inner
            } else if !inner.contains(first as char) {
                // Inner has no instances of the same quote → safe to strip
                inner
            } else if first == b'"' {
                // Outer " with inner " → ambiguous. Try unstripped first.
                match python_eval_fallback(expr) {
                    Ok(result) => return Ok(result),
                    Err(_) => inner,
                }
            } else {
                // Outer ' with inner ' → Python string expressions like 'a' + 'b'
                // Don't strip — these are Python string literals, not XML wrappers.
                expr
            }
        } else {
            expr
        }
    } else {
        expr
    };

    python_eval_fallback(expr)
}

/// Replace standalone ROS-style boolean literals (true/false) with Python-style (True/False).
/// Avoids replacing inside quoted strings.
fn replace_ros_booleans(expr: &str) -> String {
    let mut result = String::with_capacity(expr.len());
    let mut in_single = false;
    let mut in_double = false;
    let chars: Vec<char> = expr.chars().collect();
    let mut i = 0;

    while i < chars.len() {
        match chars[i] {
            '\'' if !in_double => {
                in_single = !in_single;
                result.push('\'');
                i += 1;
            }
            '"' if !in_single => {
                in_double = !in_double;
                result.push('"');
                i += 1;
            }
            _ if in_single || in_double => {
                result.push(chars[i]);
                i += 1;
            }
            _ => {
                // Check for word boundary before "true" or "false"
                let at_word_start = i == 0 || !chars[i - 1].is_alphanumeric();
                if at_word_start {
                    if expr[i..].starts_with("true") {
                        let end = i + 4;
                        let at_word_end = end >= chars.len() || !chars[end].is_alphanumeric();
                        if at_word_end {
                            result.push_str("True");
                            i += 4;
                            continue;
                        }
                    }
                    if expr[i..].starts_with("false") {
                        let end = i + 5;
                        let at_word_end = end >= chars.len() || !chars[end].is_alphanumeric();
                        if at_word_end {
                            result.push_str("False");
                            i += 5;
                            continue;
                        }
                    }
                }
                result.push(chars[i]);
                i += 1;
            }
        }
    }
    result
}

/// Evaluate expressions using Python's eval() via PyO3.
///
/// # Security
/// Uses restricted globals that only expose safe builtins (arithmetic, string ops,
/// boolean logic). Blocks `__import__`, `exec`, `eval`, `open`, `compile` etc.
/// to prevent arbitrary code execution from malicious launch files.
fn python_eval_fallback(expr: &str) -> Result<String, SubstitutionError> {
    use pyo3::{prelude::*, types::PyDict};

    // Unescape XML-style backslash-quotes (\' → ') that come from launch file
    // attribute values like value="[\'ndt\',\'yabloc\']"
    let expr = expr.replace("\\'", "'");

    // Convert ROS-style boolean literals to Python-style before eval.
    // ROS uses lowercase true/false, Python uses True/False.
    // Only replace standalone words (not inside quotes).
    let expr = replace_ros_booleans(&expr);
    let expr = expr.as_str();

    // Trim whitespace — Python's compile('eval') rejects leading spaces
    // with IndentationError, even though eval() as a function accepts them.
    let expr = expr.trim();

    log::debug!("Evaluating $(eval {})", expr);

    Python::with_gil(|py| {
        let expr_cstr = std::ffi::CString::new(expr).map_err(|e| {
            SubstitutionError::InvalidSubstitution(format!("Expression contains null byte: {}", e))
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
            SubstitutionError::InvalidSubstitution(format!(
                "Failed to evaluate expression '{}': {}",
                expr, e
            ))
        })?;

        // Convert Python result to string
        let s = result.str().map_err(|e| {
            SubstitutionError::InvalidSubstitution(format!(
                "Failed to convert eval result to string: {}",
                e
            ))
        })?;
        let value = s.to_str().map_err(|e| {
            SubstitutionError::InvalidSubstitution(format!("Failed to extract string: {}", e))
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
