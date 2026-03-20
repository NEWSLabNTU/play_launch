//! Expression evaluator for $(eval expr)
//!
//! Supports arithmetic (+, -, *, /, %, parentheses, integers, floats),
//! string comparison (==, !=), and string concatenation ('a' + 'b').
//!
//! For complex Python expressions (e.g. `in` operator, `len()`, `.split()`,
//! boolean `or`/`and`), falls back to Python's `eval()` via PyO3.

use crate::error::SubstitutionError;

/// Simple expression evaluator for $(eval expr)
/// Supports: +, -, *, /, %, parentheses, integers, floats, string comparison, and string concatenation
pub(crate) fn evaluate_expression(expr: &str) -> Result<String, SubstitutionError> {
    let expr = expr.trim();

    // Strip outer double-quote wrapping from XML $(eval &quot;...&quot;) pattern.
    // In XML, $(eval &quot;'str1' + 'str2'&quot;) produces outer double quotes around
    // single-quoted operands. Only strip when inner content uses single quotes,
    // to avoid breaking expressions like "foo" == "foo" or "" + 'x' + "".
    let expr = if expr.len() >= 4
        && expr.starts_with('"')
        && expr.ends_with('"')
        && expr[1..].trim_start().starts_with('\'')
        && expr[..expr.len() - 1].trim_end().ends_with('\'')
    {
        &expr[1..expr.len() - 1]
    } else if expr.len() >= 2
        && expr.starts_with('"')
        && expr.ends_with('"')
        && needs_python_eval(&expr[1..expr.len() - 1])
    {
        // Also strip for complex Python expressions (e.g., "'x' in list or false")
        &expr[1..expr.len() - 1]
    } else {
        expr
    };

    // Complex Python expressions — delegate to Python eval immediately
    if needs_python_eval(expr) {
        return python_eval_fallback(expr);
    }

    // Check for string comparison operators BEFORE stripping outer quotes
    // If the expression contains comparison operators, the quotes are part of the string literals
    if expr.contains("==") || expr.contains("!=") {
        // Only strip outer single-quotes if they truly wrap the ENTIRE expression
        let expr = if should_strip_outer_quotes(expr) {
            &expr[1..expr.len() - 1]
        } else {
            expr
        };
        return evaluate_string_comparison(expr);
    }

    // Check for string concatenation: 'str1' + 'str2' + ...
    // Must check before numeric evaluation since '+' is also arithmetic
    if is_string_concatenation(expr) {
        return evaluate_string_concat(expr);
    }

    // For numeric expressions, strip outer quotes if present
    let expr = if should_strip_outer_quotes(expr) {
        &expr[1..expr.len() - 1]
    } else {
        expr
    };

    // Try to parse and evaluate as a numeric expression
    match eval_expr(expr) {
        Ok(value) => {
            // If the result is a whole number, format without decimal
            if value.fract() == 0.0 {
                Ok(format!("{}", value as i64))
            } else {
                Ok(format!("{}", value))
            }
        }
        Err(_) => {
            // Fall back to Python eval() for complex expressions
            // (e.g. `in` operator, `len()`, `.split()`, `or`/`and`)
            python_eval_fallback(expr)
        }
    }
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

/// Check if an expression needs Python eval (contains Python-specific syntax).
fn needs_python_eval(expr: &str) -> bool {
    // Check for Python keywords/operators not supported by the Rust evaluator
    let keywords = [
        " in ", " or ", " and ", " not ", " if ", " else ", "len(", ".split(", ".strip(", ".join(",
    ];
    keywords.iter().any(|kw| expr.contains(kw))
}

/// Fall back to Python's eval() for expressions the Rust evaluator can't handle.
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

    log::debug!("Falling back to Python eval for: {}", expr);

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

/// Check if outer quotes truly wrap the entire expression
/// Returns true only if the opening quote's matching closing quote is at the end
fn should_strip_outer_quotes(expr: &str) -> bool {
    if expr.len() < 2 {
        return false;
    }

    let first_char = expr.chars().next().unwrap();
    let last_char = expr.chars().last().unwrap();

    // Check if starts and ends with matching quotes
    if (first_char == '"' && last_char == '"') || (first_char == '\'' && last_char == '\'') {
        // Find the matching closing quote for the opening quote
        let quote_char = first_char;
        let mut chars = expr.chars();
        chars.next(); // Skip the opening quote

        let mut found_closing = false;
        let mut pos = 1;

        for ch in chars {
            if ch == quote_char {
                // Found a matching quote - check if it's at the end
                if pos == expr.len() - 1 {
                    found_closing = true;
                }
                break;
            }
            pos += 1;
        }

        return found_closing;
    }

    false
}

/// Evaluate string comparison expressions (e.g., 'foo' == 'bar')
fn evaluate_string_comparison(expr: &str) -> Result<String, SubstitutionError> {
    // Handle == comparison
    if let Some(eq_pos) = expr.find("==") {
        let left = expr[..eq_pos].trim();
        let right = expr[eq_pos + 2..].trim();

        let left_val = strip_quotes(left);
        let right_val = strip_quotes(right);

        return Ok(if left_val == right_val {
            "true".to_string()
        } else {
            "false".to_string()
        });
    }

    // Handle != comparison
    if let Some(ne_pos) = expr.find("!=") {
        let left = expr[..ne_pos].trim();
        let right = expr[ne_pos + 2..].trim();

        let left_val = strip_quotes(left);
        let right_val = strip_quotes(right);

        return Ok(if left_val != right_val {
            "true".to_string()
        } else {
            "false".to_string()
        });
    }

    Err(SubstitutionError::InvalidSubstitution(format!(
        "Invalid comparison expression: {}",
        expr
    )))
}

/// Check if an expression is string concatenation (has '+' and quoted string operands)
///
/// Returns true for patterns like: `'str1' + 'str2'`, `"a" + "b" + "c"`
/// Returns false for numeric expressions like: `1 + 2`, `(3 + 4) * 2`
///
/// Note: Outer double quotes should already be stripped by evaluate_expression before calling this.
pub(crate) fn is_string_concatenation(expr: &str) -> bool {
    let trimmed = expr.trim();
    // Must contain '+' and at least one quoted string operand
    if !trimmed.contains('+') {
        return false;
    }
    // Split by '+' outside of quotes and check if any operand is a quoted string
    let parts = split_concat_operands(trimmed);
    parts.iter().any(|p| {
        let p = p.trim();
        (p.starts_with('\'') && p.ends_with('\'')) || (p.starts_with('"') && p.ends_with('"'))
    })
}

/// Evaluate string concatenation expression: 'str1' + 'str2' + ...
fn evaluate_string_concat(expr: &str) -> Result<String, SubstitutionError> {
    let mut result = String::new();
    for part in split_concat_operands(expr) {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        result.push_str(&strip_quotes(part));
    }
    Ok(result)
}

/// Split a concatenation expression by '+' while respecting quoted strings.
/// E.g., `'hello+world' + 'foo'` -> [`'hello+world'`, `'foo'`]
fn split_concat_operands(expr: &str) -> Vec<&str> {
    let mut parts = Vec::new();
    let mut start = 0;
    let mut in_single_quote = false;
    let mut in_double_quote = false;

    for (i, ch) in expr.char_indices() {
        match ch {
            '\'' if !in_double_quote => in_single_quote = !in_single_quote,
            '"' if !in_single_quote => in_double_quote = !in_double_quote,
            '+' if !in_single_quote && !in_double_quote => {
                parts.push(&expr[start..i]);
                start = i + 1;
            }
            _ => {}
        }
    }
    parts.push(&expr[start..]);
    parts
}

/// Strip surrounding quotes from a string
fn strip_quotes(s: &str) -> String {
    let s = s.trim();
    if (s.starts_with('"') && s.ends_with('"')) || (s.starts_with('\'') && s.ends_with('\'')) {
        s[1..s.len() - 1].to_string()
    } else {
        s.to_string()
    }
}

/// Evaluate arithmetic expression
fn eval_expr(expr: &str) -> Result<f64, String> {
    let tokens = tokenize(expr)?;
    let mut pos = 0;
    let result = parse_addition(&tokens, &mut pos)?;

    if pos < tokens.len() {
        return Err(format!("Unexpected token: {:?}", tokens[pos]));
    }

    Ok(result)
}

#[derive(Debug, Clone, PartialEq)]
enum Token {
    Number(f64),
    Plus,
    Minus,
    Multiply,
    Divide,
    Modulo,
    LeftParen,
    RightParen,
}

fn tokenize(expr: &str) -> Result<Vec<Token>, String> {
    let mut tokens = Vec::new();
    let mut chars = expr.chars().peekable();

    while let Some(&ch) = chars.peek() {
        match ch {
            ' ' | '\t' => {
                chars.next();
            }
            '+' => {
                tokens.push(Token::Plus);
                chars.next();
            }
            '-' => {
                tokens.push(Token::Minus);
                chars.next();
            }
            '*' => {
                tokens.push(Token::Multiply);
                chars.next();
            }
            '/' => {
                tokens.push(Token::Divide);
                chars.next();
            }
            '%' => {
                tokens.push(Token::Modulo);
                chars.next();
            }
            '(' => {
                tokens.push(Token::LeftParen);
                chars.next();
            }
            ')' => {
                tokens.push(Token::RightParen);
                chars.next();
            }
            '0'..='9' | '.' => {
                let mut num_str = String::new();
                while let Some(&ch) = chars.peek() {
                    if ch.is_ascii_digit() || ch == '.' {
                        num_str.push(ch);
                        chars.next();
                    } else {
                        break;
                    }
                }
                let num = num_str
                    .parse::<f64>()
                    .map_err(|_| format!("Invalid number: {}", num_str))?;
                tokens.push(Token::Number(num));
            }
            _ => {
                return Err(format!("Unexpected character: {}", ch));
            }
        }
    }

    Ok(tokens)
}

fn parse_addition(tokens: &[Token], pos: &mut usize) -> Result<f64, String> {
    let mut result = parse_multiplication(tokens, pos)?;

    while *pos < tokens.len() {
        match &tokens[*pos] {
            Token::Plus => {
                *pos += 1;
                let right = parse_multiplication(tokens, pos)?;
                result += right;
            }
            Token::Minus => {
                *pos += 1;
                let right = parse_multiplication(tokens, pos)?;
                result -= right;
            }
            _ => break,
        }
    }

    Ok(result)
}

fn parse_multiplication(tokens: &[Token], pos: &mut usize) -> Result<f64, String> {
    let mut result = parse_primary(tokens, pos)?;

    while *pos < tokens.len() {
        match &tokens[*pos] {
            Token::Multiply => {
                *pos += 1;
                let right = parse_primary(tokens, pos)?;
                result *= right;
            }
            Token::Divide => {
                *pos += 1;
                let right = parse_primary(tokens, pos)?;
                if right == 0.0 {
                    return Err("Division by zero".to_string());
                }
                result /= right;
            }
            Token::Modulo => {
                *pos += 1;
                let right = parse_primary(tokens, pos)?;
                if right == 0.0 {
                    return Err("Modulo by zero".to_string());
                }
                result %= right;
            }
            _ => break,
        }
    }

    Ok(result)
}

fn parse_primary(tokens: &[Token], pos: &mut usize) -> Result<f64, String> {
    if *pos >= tokens.len() {
        return Err("Unexpected end of expression".to_string());
    }

    match &tokens[*pos] {
        Token::Number(n) => {
            *pos += 1;
            Ok(*n)
        }
        Token::Minus => {
            *pos += 1;
            let value = parse_primary(tokens, pos)?;
            Ok(-value)
        }
        Token::LeftParen => {
            *pos += 1;
            let result = parse_addition(tokens, pos)?;
            if *pos >= tokens.len() || tokens[*pos] != Token::RightParen {
                return Err("Missing closing parenthesis".to_string());
            }
            *pos += 1;
            Ok(result)
        }
        _ => Err(format!("Unexpected token at position {}", pos)),
    }
}
