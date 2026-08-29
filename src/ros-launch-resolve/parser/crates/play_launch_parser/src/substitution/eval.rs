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

    // Mirror what launch's frontend does, in the order it does it.
    //
    // `$(eval '...')` in an XML attribute is a QUOTED TEMPLATE: the grammar
    // (launch/frontend/grammar.lark, rule `single_quoted_template`) consumes
    // the outer quotes as delimiters, and `\'` inside is an escaped literal
    // quote which `replace_escaped_characters` — `re.sub(r'\\(.)', r'\1')` —
    // later turns back into `'`.
    //
    // So the delimiter decision must be made ESCAPE-AWARE, and unescaping must
    // happen after it. Doing them the other way round is what broke
    //
    //     if="$(eval '\'$(var pose_source)\' == \'aruco\'')"
    //
    // a form real `ros2 launch` accepts (measured, both branches): the
    // heuristic saw the escaped inner quotes as real ones, declined to strip
    // the outer pair, and unescaping then produced `''ndt' == 'aruco''` —
    // a Python SyntaxError.
    let expr = if outer_quotes_are_delimiters(expr) {
        &expr[1..expr.len() - 1]
    } else {
        expr
    };
    let expr = unescape(expr);

    python_eval_fallback(expr.trim())
}

/// Undo the frontend's escaping: a backslash escapes whatever follows it.
///
/// Matches launch's `replace_escaped_characters`, which is
/// `re.sub(r'\\(.)', r'\1', data)` — deliberately general rather than
/// `\'`-only, because that is what the reference implementation does to every
/// text fragment it builds.
fn unescape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars();
    while let Some(c) = chars.next() {
        if c == '\\' {
            // A trailing lone backslash has nothing to escape; keep it.
            match chars.next() {
                Some(next) => out.push(next),
                None => out.push('\\'),
            }
        } else {
            out.push(c);
        }
    }
    out
}

/// Whether the outer quote pair delimit a template rather than being part of a
/// Python string literal.
///
/// True when the expression opens with a quote, closes with an UNESCAPED quote
/// of the same kind, and contains no unescaped quote of that kind in between —
/// which is exactly when the frontend grammar would have treated them as
/// delimiters.
///
/// Counter-examples this must reject:
///   `'foo' + 'bar'`  — opens and closes with `'`, but the inner quotes are
///                      unescaped, so these are Python string literals.
///   `''`             — nothing between them; stripping yields an empty
///                      expression, which `eval` rejects.
fn outer_quotes_are_delimiters(expr: &str) -> bool {
    let bytes = expr.as_bytes();
    if bytes.len() < 2 {
        return false;
    }
    let quote = bytes[0];
    if quote != b'"' && quote != b'\'' {
        return false;
    }

    let inner = &expr[1..expr.len() - 1];
    if inner.trim().is_empty() {
        return false;
    }

    // Walk the inner text tracking escapes. The closing quote must be
    // unescaped, and no unescaped quote of the same kind may appear before it.
    let mut escaped = false;
    for &b in inner.as_bytes() {
        if escaped {
            escaped = false;
            continue;
        }
        if b == b'\\' {
            escaped = true;
        } else if b == quote {
            return false;
        }
    }
    // A trailing backslash would escape the closing quote, so it is not a
    // delimiter at all.
    !escaped && bytes[bytes.len() - 1] == quote
}

/// Replace standalone ROS-style boolean literals (true/false) with Python-style (True/False).
/// Avoids replacing inside quoted strings.
pub(crate) fn replace_ros_booleans(expr: &str) -> String {
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
    // `$(eval …)` is Python evaluation BY SPECIFICATION, so this is reachable
    // from XML and YAML, not only from `.launch.py`. A build with no Python
    // half must therefore fail HERE too — naming the expression, so the user
    // can see which attribute made an otherwise pure-XML tree need CPython.
    let backend = crate::python_backend::require(
        crate::python_backend::PythonNeed::EvalSubstitution,
        &format!("$(eval {expr})"),
    )
    .map_err(|e| SubstitutionError::InvalidSubstitution(e.to_string()))?;
    backend
        .eval_expr(expr)
        .map_err(SubstitutionError::InvalidSubstitution)
}

#[cfg(test)]
mod escape_tests {
    use super::*;

    /// The form that broke the golf cart's main launch file. The BOOLEAN VALUES
    /// were measured against real `ros2 launch` first, both branches of each
    /// comparison, so these pin conformance rather than our preference. The
    /// lowercase spelling is ours: `$(eval)` results feeding `if=`/`unless=`
    /// are normalized to "true"/"false" (see CLAUDE.md).
    #[test]
    fn escaped_quotes_inside_a_quoted_template_are_content() {
        assert_eq!(
            evaluate_expression(r"'\'ndt\' == \'aruco\''").unwrap(),
            "false"
        );
        assert_eq!(
            evaluate_expression(r"'\'aruco\' == \'aruco\''").unwrap(),
            "true"
        );
        // The `not in [...]` variant from the same file.
        assert_eq!(
            evaluate_expression(r"'\'ndt\' not in [\'cuda_ndt\', \'aruco\']'").unwrap(),
            "true"
        );
        assert_eq!(
            evaluate_expression(r"'\'aruco\' not in [\'cuda_ndt\', \'aruco\']'").unwrap(),
            "false"
        );
    }

    /// The `&quot;`-wrapped form used elsewhere in the same repository. It
    /// worked before and must keep working.
    #[test]
    fn double_quoted_template_still_unwraps() {
        assert_eq!(
            evaluate_expression("\"'ndt' == 'cuda_ndt'\"").unwrap(),
            "false"
        );
        assert_eq!(
            evaluate_expression("\"'cuda_ndt' == 'cuda_ndt'\"").unwrap(),
            "true"
        );
    }

    /// Outer quotes that are NOT delimiters: a Python expression that merely
    /// begins and ends with a string literal. Stripping here would corrupt it.
    #[test]
    fn python_string_literals_are_not_treated_as_delimiters() {
        assert_eq!(evaluate_expression("'foo' + 'bar'").unwrap(), "foobar");
        assert_eq!(evaluate_expression("'a' == 'a'").unwrap(), "true");
    }

    #[test]
    fn empty_and_degenerate_inputs_do_not_panic() {
        // `''` must not be stripped to nothing — eval('') is a SyntaxError.
        assert_eq!(evaluate_expression("''").unwrap(), "");
        assert!(evaluate_expression("").is_err());
    }

    #[test]
    fn unescape_matches_the_reference_rule() {
        // launch: re.sub(r'\\(.)', r'\1') — a backslash escapes ANY character.
        assert_eq!(unescape(r"\'a\'"), "'a'");
        assert_eq!(unescape(r"a\\b"), r"a\b");
        assert_eq!(unescape(r"no escapes"), "no escapes");
        // A lone trailing backslash has nothing to escape and is kept.
        assert_eq!(unescape(r"trailing\"), r"trailing\");
    }

    #[test]
    fn delimiter_detection_is_escape_aware() {
        assert!(outer_quotes_are_delimiters(r"'\'a\' == \'b\''"));
        assert!(outer_quotes_are_delimiters("\"'a' == 'b'\""));
        // Unescaped inner quotes of the same kind => Python literals.
        assert!(!outer_quotes_are_delimiters("'foo' + 'bar'"));
        // Nothing to unwrap.
        assert!(!outer_quotes_are_delimiters("''"));
        assert!(!outer_quotes_are_delimiters("1 + 2"));
        // A trailing backslash escapes the closing quote, so the pair is not
        // balanced and must not be treated as delimiters.
        assert!(!outer_quotes_are_delimiters(r"'a\'"));
    }
}
