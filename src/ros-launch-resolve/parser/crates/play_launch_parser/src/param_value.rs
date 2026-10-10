//! What a `<param value=...>` becomes, the way `launch_ros` evaluates it.
//!
//! `launch_ros` reads an inline parameter value with `yaml.safe_load` — PyYAML,
//! i.e. YAML **1.1** resolution — after performing its substitutions
//! (`evaluate_parameter_dict`), unless the parameter names a `type`. So
//! `yes` is a boolean, `0x10` is 16, `1e3` is a string (PyYAML's float needs a
//! dot), a trailing newline from `$(command ...)` is gone, a multi-line value
//! is folded into one line, and a ` #` starts a comment.
//!
//! The record, the model and the spawner carry a parameter as TEXT and re-type
//! it by shape, so the value is rendered back to the text that re-types to
//! what `launch` produced: `True`/`False`, a decimal integer, a float with a
//! dot — and a STRING that would otherwise re-type (`'5'`, `type="str"` on
//! `1.0`, the empty string) as a YAML single-quoted scalar, which every
//! re-typing step reads as the string inside ([`quote_if_ambiguous`],
//! [`unquote`]).

/// Whether text would be re-typed as something other than this string by a
/// consumer that types by shape (the model builder, the spawner).
pub fn is_ambiguous(s: &str) -> bool {
    s.is_empty()
        || matches!(s, "true" | "True" | "false" | "False")
        || s.parse::<f64>().is_ok()
        || s.starts_with(['\'', '['])
}

/// A string parameter value as text that stays a string through re-typing:
/// as is when unambiguous, else YAML single-quoted.
pub fn quote_if_ambiguous(s: &str) -> String {
    if is_ambiguous(s) {
        format!("'{}'", s.replace('\'', "''"))
    } else {
        s.to_string()
    }
}

/// The string inside a YAML single-quoted scalar, if `s` is one.
pub fn unquote(s: &str) -> Option<String> {
    let inner = s.strip_prefix('\'')?.strip_suffix('\'')?;
    // Every quote inside must be a doubled one.
    let mut out = String::with_capacity(inner.len());
    let mut chars = inner.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '\'' && chars.next() != Some('\'') {
            return None;
        }
        out.push(c);
    }
    Some(out)
}

/// Evaluate a performed parameter value as `launch_ros` does, with an
/// optional `type=` (`str`, `int`, `float`, `bool`, `yaml`, `list_of_*`).
pub fn evaluate(text: &str, value_type: Option<&str>) -> Result<String, String> {
    match value_type {
        None | Some("yaml") => yaml_value(text),
        Some("str") => Ok(quote_if_ambiguous(text)),
        Some("int") => text
            .trim()
            .parse::<i64>()
            .map(|i| i.to_string())
            .map_err(|_| format!("'{text}' is not an int")),
        Some("float") => text
            .trim()
            .parse::<f64>()
            .map(render_float)
            .map_err(|_| format!("'{text}' is not a float")),
        Some("bool") => match text.trim() {
            "1" => Ok("True".into()),
            "0" => Ok("False".into()),
            other => match resolve_plain(other) {
                Scalar::Bool(b) => Ok(render_bool(b)),
                _ => Err(format!("'{text}' is not a bool")),
            },
        },
        Some(t) if t.starts_with("list_of_") => yaml_value(text),
        Some(other) => Err(format!("Got invalid type identifier '{other}'")),
    }
}

/// `yaml.safe_load(text)`, rendered.
pub fn yaml_value(text: &str) -> Result<String, String> {
    // `evaluate_parameter_dict`: an empty value is the empty string.
    if text.trim().is_empty() {
        return Ok(quote_if_ambiguous(""));
    }
    let trimmed = text.trim_start();
    let first = trimmed.chars().next().unwrap_or(' ');
    let block_seq = trimmed.starts_with("- ") || trimmed == "-" || trimmed.starts_with("-\n");
    if block_seq || matches!(first, '\'' | '"' | '[' | '{' | '|' | '>' | '!' | '&' | '*') {
        // Quoted, flow or block: let the YAML parser have it.
        return match serde_yaml_ng::from_str::<serde_yaml_ng::Value>(text) {
            Ok(v) => render_value(&v, first),
            // `convert_as_yaml(..., can_be_str=True)` keeps the text.
            Err(_) => Ok(quote_if_ambiguous(text)),
        };
    }
    // A plain scalar — unless the YAML parser sees a mapping in it, which
    // `launch_ros` refuses.
    if let Ok(serde_yaml_ng::Value::Mapping(_)) =
        serde_yaml_ng::from_str::<serde_yaml_ng::Value>(text)
    {
        return Err(format!(
            "Allowed value types are bytes, bool, int, float, str, Sequence[bool], Sequence[int], \
             Sequence[float], Sequence[str]. Got a mapping: '{text}'"
        ));
    }
    match resolve_plain(&fold_plain(text)) {
        Scalar::Null => Err(format!(
            "Allowed value types are bytes, bool, int, float, str, Sequence[bool], Sequence[int], \
             Sequence[float], Sequence[str]. Got None: '{text}'"
        )),
        s => Ok(render_scalar(s)),
    }
}

/// A plain multi-line scalar: comments stripped, each line trimmed, lines
/// folded with a space, an empty line kept as a newline.
fn fold_plain(text: &str) -> String {
    let mut lines: Vec<String> = Vec::new();
    for line in text.lines() {
        // ` #` (or `#` at the very start) begins a comment.
        let mut cut = line.len();
        let bytes = line.as_bytes();
        for (i, b) in bytes.iter().enumerate() {
            if *b == b'#' && (i == 0 || bytes[i - 1] == b' ' || bytes[i - 1] == b'\t') {
                cut = i;
                break;
            }
        }
        lines.push(line[..cut].trim().to_string());
    }
    while lines.last().is_some_and(|l| l.is_empty()) {
        lines.pop();
    }
    while lines.first().is_some_and(|l| l.is_empty()) {
        lines.remove(0);
    }
    let mut out = String::new();
    let mut pending_newlines = 0;
    for (i, line) in lines.iter().enumerate() {
        if line.is_empty() {
            pending_newlines += 1;
            continue;
        }
        if i > 0 {
            if pending_newlines > 0 {
                out.push_str(&"\n".repeat(pending_newlines));
            } else {
                out.push(' ');
            }
        }
        pending_newlines = 0;
        out.push_str(line);
    }
    out
}

/// A `data_type=bool` attribute, as `launch.utilities.type_utils` coerces
/// it: `1`/`0`, then whatever YAML reads as a boolean (`yes`, `off`, ...).
/// Anything else is an error, where this parser used to read it as true.
pub fn bool_attr(text: &str) -> Result<bool, String> {
    let text = match text {
        "1" => "true",
        "0" => "false",
        t => t,
    };
    match resolve_plain(text.trim()) {
        Scalar::Bool(b) => Ok(b),
        _ => Err(format!("Cannot convert value '{text}' to a bool")),
    }
}

#[derive(Debug, PartialEq)]
enum Scalar {
    Null,
    Bool(bool),
    Int(i64),
    Float(f64),
    Str(String),
}

/// PyYAML's implicit resolvers for a plain scalar (`resolver.py`).
fn resolve_plain(s: &str) -> Scalar {
    match s {
        "" | "~" | "null" | "Null" | "NULL" => return Scalar::Null,
        "yes" | "Yes" | "YES" | "true" | "True" | "TRUE" | "on" | "On" | "ON" => {
            return Scalar::Bool(true);
        }
        "no" | "No" | "NO" | "false" | "False" | "FALSE" | "off" | "Off" | "OFF" => {
            return Scalar::Bool(false);
        }
        ".inf" | ".Inf" | ".INF" | "+.inf" | "+.Inf" | "+.INF" => {
            return Scalar::Float(f64::INFINITY);
        }
        "-.inf" | "-.Inf" | "-.INF" => return Scalar::Float(f64::NEG_INFINITY),
        ".nan" | ".NaN" | ".NAN" => return Scalar::Float(f64::NAN),
        _ => {}
    }
    if let Some(i) = yaml11_int(s) {
        return Scalar::Int(i);
    }
    if let Some(f) = yaml11_float(s) {
        return Scalar::Float(f);
    }
    Scalar::Str(s.to_string())
}

/// `int`: `[-+]?0b[0-1_]+ | [-+]?0[0-7_]+ | [-+]?(0|[1-9][0-9_]*) |
/// [-+]?0x[0-9a-fA-F_]+ | [-+]?[1-9][0-9_]*(:[0-5]?[0-9])+`.
fn yaml11_int(s: &str) -> Option<i64> {
    let (neg, body) = match s.as_bytes().first()? {
        b'-' => (true, &s[1..]),
        b'+' => (false, &s[1..]),
        _ => (false, s),
    };
    if body.is_empty() {
        return None;
    }
    let digits =
        |t: &str, ok: fn(char) -> bool| !t.is_empty() && t.chars().all(|c| ok(c) || c == '_');
    let clean = |t: &str| t.replace('_', "");
    let value = if let Some(b) = body.strip_prefix("0b") {
        if !digits(b, |c| c == '0' || c == '1') {
            return None;
        }
        i64::from_str_radix(&clean(b), 2).ok()?
    } else if let Some(h) = body.strip_prefix("0x") {
        if !digits(h, |c| c.is_ascii_hexdigit()) {
            return None;
        }
        i64::from_str_radix(&clean(h), 16).ok()?
    } else if body == "0" {
        0
    } else if let Some(o) = body.strip_prefix('0') {
        if !digits(o, |c| ('0'..='7').contains(&c)) {
            return None;
        }
        i64::from_str_radix(&clean(o), 8).ok()?
    } else if body.contains(':') {
        // Sexagesimal: 1:30 == 90.
        let mut parts = body.split(':');
        let head = parts.next()?;
        if !head.starts_with(|c: char| ('1'..='9').contains(&c))
            || !digits(head, |c| c.is_ascii_digit())
        {
            return None;
        }
        let mut v: i64 = clean(head).parse().ok()?;
        for p in parts {
            if p.is_empty() || p.len() > 2 || !p.chars().all(|c| c.is_ascii_digit()) {
                return None;
            }
            let n: i64 = p.parse().ok()?;
            if n > 59 {
                return None;
            }
            v = v * 60 + n;
        }
        v
    } else {
        if !body.starts_with(|c: char| ('1'..='9').contains(&c))
            || !digits(body, |c| c.is_ascii_digit())
        {
            return None;
        }
        clean(body).parse().ok()?
    };
    Some(if neg { -value } else { value })
}

/// `float`: a dot is required, an exponent needs a sign —
/// `[-+]?([0-9][0-9_]*)?\.[0-9_]*([eE][-+][0-9]+)?` (plus sexagesimal).
fn yaml11_float(s: &str) -> Option<f64> {
    let body = s.strip_prefix(['-', '+']).unwrap_or(s);
    let (mantissa, exponent) = match body.find(['e', 'E']) {
        Some(i) => (&body[..i], Some(&body[i + 1..])),
        None => (body, None),
    };
    let dot = mantissa.find('.')?;
    let (int_part, frac_part) = (&mantissa[..dot], &mantissa[dot + 1..]);
    let ok_int = int_part.is_empty()
        || (int_part.starts_with(|c: char| c.is_ascii_digit())
            && int_part.chars().all(|c| c.is_ascii_digit() || c == '_'));
    let ok_frac = frac_part.chars().all(|c| c.is_ascii_digit() || c == '_');
    if !ok_int || !ok_frac || (int_part.is_empty() && frac_part.is_empty()) {
        return None;
    }
    if let Some(e) = exponent {
        let digits = e.strip_prefix(['-', '+'])?;
        if digits.is_empty() || !digits.chars().all(|c| c.is_ascii_digit()) {
            return None;
        }
    }
    s.replace('_', "").parse::<f64>().ok()
}

fn render_bool(b: bool) -> String {
    if b { "True" } else { "False" }.to_string()
}

/// A float as text that re-types as a float: keeps the dot.
///
/// YAML 1.1 reads a float only with a dot in the mantissa and a SIGNED
/// exponent, so Rust's `1e-6` would come back a string: it is written
/// `1.0e-6` (measured on Autoware's `mpc_weight_steer_acc: 0.000001`).
pub fn render_float(f: f64) -> String {
    if f.is_infinite() {
        return if f > 0.0 { ".inf" } else { "-.inf" }.to_string();
    }
    if f.is_nan() {
        return ".nan".to_string();
    }
    let s = format!("{f:?}");
    let (mantissa, exponent) = match s.split_once('e') {
        Some((m, e)) => (m, Some(e)),
        None => (s.as_str(), None),
    };
    let mut out = mantissa.to_string();
    if !out.contains('.') {
        out.push_str(".0");
    }
    if let Some(e) = exponent {
        out.push('e');
        if !e.starts_with('-') && !e.starts_with('+') {
            out.push('+');
        }
        out.push_str(e);
    }
    out
}

fn render_scalar(s: Scalar) -> String {
    match s {
        Scalar::Null => String::new(),
        Scalar::Bool(b) => render_bool(b),
        Scalar::Int(i) => i.to_string(),
        Scalar::Float(f) => render_float(f),
        Scalar::Str(s) => quote_if_ambiguous(&s),
    }
}

fn render_value(v: &serde_yaml_ng::Value, first: char) -> Result<String, String> {
    use serde_yaml_ng::Value;
    Ok(match v {
        Value::String(s) => quote_if_ambiguous(s),
        Value::Bool(b) => render_bool(*b),
        Value::Number(n) => {
            if let Some(i) = n.as_i64() {
                i.to_string()
            } else {
                render_float(n.as_f64().unwrap_or_default())
            }
        }
        Value::Sequence(items) => {
            let rendered: Vec<String> = items
                .iter()
                .map(|item| match item {
                    Value::String(s) => match resolve_plain(s) {
                        // A quoted element stays a string; render it quoted
                        // so the spawner keeps it one.
                        Scalar::Str(_) | Scalar::Null => format!("'{s}'"),
                        _ => format!("'{s}'"),
                    },
                    other => render_value(other, ' ').unwrap_or_default(),
                })
                .collect();
            format!("[{}]", rendered.join(", "))
        }
        Value::Null => {
            return Err("Allowed value types do not include None".to_string());
        }
        _ if first == '{' => return Err("a mapping is not a parameter value".to_string()),
        _ => return Err("unsupported parameter value".to_string()),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Each expectation is what `yaml.safe_load` (PyYAML) gives, rendered.
    #[test]
    fn yaml_1_1_resolution_like_pyyaml() {
        let cases = [
            ("true", "True"),
            ("yes", "True"),
            ("Off", "False"),
            ("7", "7"),
            ("0x10", "16"),
            ("010", "8"),
            ("1_000", "1000"),
            ("1:30", "90"),
            ("1.0", "1.0"),
            ("1.5e+3", "1500.0"),
            // PyYAML's float needs a dot and a signed exponent, so these are
            // STRINGS — quoted, so nothing downstream reads them as floats.
            ("1e3", "'1e3'"),
            ("1.0e3", "'1.0e3'"),
            ("-1e3", "'-1e3'"),
            ("-5", "-5"),
            ("hello world\n", "hello world"),
            ("a\nb", "a b"),
            ("x # comment", "x"),
            // A quoted value is a string, and stays one.
            ("'5'", "'5'"),
            ("\"quoted\"", "quoted"),
            ("[1, 2]", "[1, 2]"),
            ("[a, b]", "['a', 'b']"),
            ("", "''"),
            ("it's", "it's"),
        ];
        for (input, expected) in cases {
            assert_eq!(yaml_value(input).unwrap(), expected, "{input:?}");
        }
    }

    #[test]
    fn quoting_round_trips() {
        for s in ["5", "", "true", "it's", "'x'", "[1]", "plain"] {
            let q = quote_if_ambiguous(s);
            assert_eq!(unquote(&q).unwrap_or(q.clone()), s, "{q}");
        }
    }

    #[test]
    fn a_null_or_a_mapping_is_refused() {
        assert!(yaml_value("~").is_err());
        assert!(yaml_value("a: b").is_err());
    }

    #[test]
    fn a_type_skips_yaml() {
        assert_eq!(evaluate("1.0", Some("str")).unwrap(), "'1.0'");
        assert_eq!(evaluate("yes", Some("str")).unwrap(), "yes");
        assert_eq!(evaluate("1", Some("bool")).unwrap(), "True");
        assert_eq!(evaluate("3", Some("float")).unwrap(), "3.0");
    }

    #[test]
    fn a_float_in_exponent_form_still_reads_back_as_a_float() {
        assert_eq!(render_float(0.000001), "1.0e-6");
        assert_eq!(render_float(1e20), "1.0e+20");
        assert_eq!(render_float(2.5), "2.5");
        assert_eq!(render_float(3.0), "3.0");
        for f in [0.000001, 1e20, 1.5e-7] {
            assert_eq!(resolve_plain(&render_float(f)), Scalar::Float(f));
        }
    }
}
