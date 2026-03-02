//! Condition evaluation for if/unless attributes

use crate::{
    error::Result,
    substitution::{LaunchContext, parse_substitutions, resolve_substitutions},
    xml::{Entity, XmlEntity},
};

/// Evaluate whether an entity should be processed based on if/unless conditions
pub fn should_process_entity(entity: &XmlEntity, context: &LaunchContext) -> Result<bool> {
    // Check "if" attribute
    if let Some(if_condition) = entity.optional_attr_str("if")? {
        let result = evaluate_condition(&if_condition, context)?;
        if !result {
            return Ok(false);
        }
    }

    // Check "unless" attribute
    if let Some(unless_condition) = entity.optional_attr_str("unless")? {
        let result = evaluate_condition(&unless_condition, context)?;
        if result {
            return Ok(false);
        }
    }

    Ok(true)
}

/// Evaluate a condition string (may contain substitutions)
fn evaluate_condition(condition: &str, context: &LaunchContext) -> Result<bool> {
    // Parse and resolve substitutions
    let subs = parse_substitutions(condition)?;
    let resolved = resolve_substitutions(&subs, context)
        .map_err(|e| crate::error::ParseError::InvalidSubstitution(e.to_string()))?;

    // Evaluate as boolean
    Ok(is_truthy(&resolved))
}

/// Determine if a string value is "truthy"
pub fn is_truthy(value: &str) -> bool {
    let normalized = value.trim().to_lowercase();
    matches!(
        normalized.as_str(),
        "true" | "1" | "yes" | "y" | "on" | "enabled"
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_truthy() {
        assert!(is_truthy("true"));
        assert!(is_truthy("True"));
        assert!(is_truthy("TRUE"));
        assert!(is_truthy("1"));
        assert!(is_truthy("yes"));
        assert!(is_truthy("y"));
        assert!(is_truthy("on"));
        assert!(is_truthy("enabled"));
        assert!(is_truthy("  true  "));

        assert!(!is_truthy("false"));
        assert!(!is_truthy("0"));
        assert!(!is_truthy("no"));
        assert!(!is_truthy(""));
        assert!(!is_truthy("random"));
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
