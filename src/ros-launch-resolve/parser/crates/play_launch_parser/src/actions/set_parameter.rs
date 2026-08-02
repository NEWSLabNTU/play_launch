//! Set parameter action for global parameter setting

use crate::{
    error::{ParseError, Result},
    substitution::{Substitution, parse_substitutions},
    xml::Entity,
};

/// Set parameter action
#[derive(Debug, Clone, PartialEq)]
pub struct SetParameterAction {
    pub name: String,
    pub value: Vec<Substitution>,
}

impl SetParameterAction {
    pub fn from_entity<E: Entity>(entity: &E) -> Result<Self> {
        let name =
            entity
                .optional_attr_str("name")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "set_parameter".to_string(),
                    attribute: "name".to_string(),
                })?;

        let value_str =
            entity
                .optional_attr_str("value")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "set_parameter".to_string(),
                    attribute: "value".to_string(),
                })?;

        let value = parse_substitutions(&value_str)?;

        Ok(SetParameterAction { name, value })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::substitution::Substitution;
    use roxmltree::Document;

    #[test]
    fn test_parse_set_parameter_literal() {
        let xml = r#"<set_parameter name="use_sim_time" value="true" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = SetParameterAction::from_entity(&entity).unwrap();
        assert_eq!(action.name, "use_sim_time");
        assert_eq!(action.value, vec![Substitution::Text("true".to_string())]);
    }

    #[test]
    fn test_parse_set_parameter_with_substitution() {
        let xml = r#"<set_parameter name="robot_name" value="$(var name)" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = SetParameterAction::from_entity(&entity).unwrap();
        assert_eq!(action.name, "robot_name");
        assert_eq!(action.value.len(), 1);
        assert!(matches!(
            action.value[0],
            Substitution::LaunchConfiguration(_)
        ));
    }

    #[test]
    fn test_parse_set_parameter_missing_name() {
        let xml = r#"<set_parameter value="true" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let result = SetParameterAction::from_entity(&entity);
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_set_parameter_missing_value() {
        let xml = r#"<set_parameter name="param" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let result = SetParameterAction::from_entity(&entity);
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_set_parameter_numeric() {
        let xml = r#"<set_parameter name="timeout" value="30.5" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = SetParameterAction::from_entity(&entity).unwrap();
        assert_eq!(action.name, "timeout");
        assert_eq!(action.value, vec![Substitution::Text("30.5".to_string())]);
    }
}
