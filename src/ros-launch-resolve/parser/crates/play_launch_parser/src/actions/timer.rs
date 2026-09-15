//! `<timer period="N">` — ROS 2's `TimerAction` on the XML/YAML frontend.
//!
//! `TimerAction` delays the actions nested inside it by `period` seconds,
//! measured from when the timer itself is visited (launch start, for a
//! top-level timer). It is a plain container action otherwise: the children
//! are ordinary launch actions and are parsed exactly as they would be
//! outside it.
//!
//! This parser used to fall through to the `Unsupported action type` arm,
//! which dropped the timer AND every node under it — the model came back
//! missing whole subsystems while `check` still exited 0. The delay itself is
//! carried on each produced record as
//! [`crate::record::NodeRecord::start_delay_secs`].
//!
//! `period` is a substitution-bearing attribute (`$(var startup_delay)` is
//! valid ROS 2), so it is stored unresolved and evaluated by the traverser
//! against the live context.

use crate::{
    error::{ParseError, Result},
    substitution::{Substitution, parse_substitutions},
    xml::Entity,
};

/// A parsed `<timer>` / `timer:` action. The children are NOT held here —
/// the traverser walks them from the entity directly, the same way `<group>`
/// does, so nested includes and scopes behave identically inside and outside
/// a timer.
#[derive(Debug, Clone, PartialEq)]
pub struct TimerAction {
    /// Unresolved `period` attribute, in seconds.
    pub period: Vec<Substitution>,
}

impl TimerAction {
    pub fn from_entity<E: Entity>(entity: &E) -> Result<Self> {
        // `period` is REQUIRED by ROS 2 (`TimerAction.parse` reads it
        // non-optionally); a `<timer>` without one is a launch file stock
        // `ros2 launch` refuses, so refusing here matches rather than
        // diverges.
        let period_str =
            entity
                .optional_attr_str("period")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "timer".to_string(),
                    attribute: "period".to_string(),
                })?;

        Ok(TimerAction {
            period: parse_substitutions(&period_str)?,
        })
    }
}

/// Parse a resolved `period` string into seconds.
///
/// Separate from [`TimerAction::from_entity`] because the YAML frontend
/// reaches the same value through a `Mapping` rather than an `Entity`, and
/// because resolution needs the context the traverser holds.
pub fn parse_period_secs(resolved: &str) -> Result<f64> {
    let secs: f64 = resolved.trim().parse().map_err(|_| {
        ParseError::InvalidSubstitution(format!(
            "<timer period=\"{resolved}\"> is not a number of seconds"
        ))
    })?;
    if !secs.is_finite() || secs < 0.0 {
        return Err(ParseError::InvalidSubstitution(format!(
            "<timer period=\"{resolved}\"> must be a non-negative, finite number of seconds"
        )));
    }
    Ok(secs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use roxmltree::Document;

    #[test]
    fn parses_a_literal_period() {
        let doc = Document::parse(r#"<timer period="3.0"/>"#).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let action = TimerAction::from_entity(&entity).unwrap();
        assert_eq!(action.period, vec![Substitution::Text("3.0".to_string())]);
    }

    #[test]
    fn keeps_a_substitution_period_unresolved() {
        let doc = Document::parse(r#"<timer period="$(var delay)"/>"#).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let action = TimerAction::from_entity(&entity).unwrap();
        assert_eq!(action.period.len(), 1);
        assert!(matches!(
            action.period[0],
            Substitution::LaunchConfiguration(_)
        ));
    }

    #[test]
    fn a_timer_without_a_period_is_an_error() {
        let doc = Document::parse(r#"<timer/>"#).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        assert!(TimerAction::from_entity(&entity).is_err());
    }

    #[test]
    fn a_negative_or_unparsable_period_is_an_error() {
        assert_eq!(parse_period_secs("2.5").unwrap(), 2.5);
        assert_eq!(parse_period_secs(" 0 ").unwrap(), 0.0);
        assert!(parse_period_secs("-1").is_err());
        assert!(parse_period_secs("soon").is_err());
    }
}
