//! Condition evaluation for if/unless attributes

use crate::{
    error::Result,
    substitution::{LaunchContext, parse_substitutions, resolve_substitutions},
    xml::{Entity, XmlEntity},
};

/// Whether an entity's `if=`/`unless=` let it execute, as `launch` evaluates
/// them (`IfCondition`/`UnlessCondition` → `evaluate_condition_expression`).
pub fn should_process_entity(entity: &XmlEntity, context: &LaunchContext) -> Result<bool> {
    conditions_allow(
        entity.optional_attr_str("if")?.as_deref(),
        entity.optional_attr_str("unless")?.as_deref(),
        context,
    )
}

/// `if=`/`unless=`, from any frontend. Both at once is an error in `launch`
/// ("if and unless conditions can't be used simultaneously").
pub fn conditions_allow(
    if_expr: Option<&str>,
    unless_expr: Option<&str>,
    context: &LaunchContext,
) -> Result<bool> {
    match (if_expr, unless_expr) {
        (Some(_), Some(_)) => Err(crate::error::ParseError::InvalidSubstitution(
            "if and unless conditions can't be used simultaneously".to_string(),
        )),
        (Some(e), None) => evaluate_condition(e, context),
        (None, Some(e)) => evaluate_condition(e, context).map(|b| !b),
        (None, None) => Ok(true),
    }
}

/// Evaluate a condition string (may contain substitutions)
fn evaluate_condition(condition: &str, context: &LaunchContext) -> Result<bool> {
    let subs = parse_substitutions(condition)?;
    let resolved = resolve_substitutions(&subs, context)
        .map_err(|e| crate::error::ParseError::InvalidSubstitution(e.to_string()))?;
    condition_value(&resolved)
}

/// `launch.conditions.evaluate_condition_expression`: `true`/`1` and
/// `false`/`0`, case-insensitively, and nothing else. This used to read
/// `yes`/`on`/`enabled` as true and anything unrecognised — a typo
/// included — as false.
pub fn condition_value(resolved: &str) -> Result<bool> {
    match resolved.trim().to_lowercase().as_str() {
        "true" | "1" => Ok(true),
        "false" | "0" => Ok(false),
        _ => Err(crate::error::ParseError::InvalidSubstitution(format!(
            "invalid condition expression, expected one of [true, 1, false, 0] but got \
             '{}'",
            resolved.trim().to_lowercase()
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_condition_is_true_1_false_or_0_and_nothing_else() {
        for t in ["true", "True", "TRUE", "1", "  true  "] {
            assert!(condition_value(t).unwrap(), "{t}");
        }
        for f in ["false", "FALSE", "0"] {
            assert!(!condition_value(f).unwrap(), "{f}");
        }
        // `launch` raises on each of these; they used to be true or false.
        for bad in ["yes", "on", "enabled", "no", "", "random"] {
            assert!(condition_value(bad).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn if_and_unless_together_is_an_error() {
        let context = LaunchContext::new();
        assert!(conditions_allow(Some("true"), Some("false"), &context).is_err());
    }

    #[test]
    fn test_evaluate_condition() {
        let mut context = LaunchContext::new();
        context.set_configuration("use_sim".to_string(), "true".to_string());
        context.set_configuration("debug".to_string(), "false".to_string());

        assert!(evaluate_condition("$(var use_sim)", &context).unwrap());
        assert!(!evaluate_condition("$(var debug)", &context).unwrap());
        assert!(evaluate_condition("true", &context).unwrap());
        assert!(!evaluate_condition("false", &context).unwrap());
    }
}
