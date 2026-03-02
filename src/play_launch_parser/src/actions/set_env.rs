//! Environment variable manipulation actions

use crate::{
    error::{ParseError, Result},
    substitution::{Substitution, parse_substitutions},
    xml::Entity,
};

/// Set environment variable action
#[derive(Debug, Clone, PartialEq)]
pub struct SetEnvAction {
    pub name: String,
    pub value: Vec<Substitution>,
}

impl SetEnvAction {
    pub fn from_entity<E: Entity>(entity: &E) -> Result<Self> {
        let name =
            entity
                .optional_attr_str("name")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "set_env".to_string(),
                    attribute: "name".to_string(),
                })?;

        let value_str =
            entity
                .optional_attr_str("value")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "set_env".to_string(),
                    attribute: "value".to_string(),
                })?;

        let value = parse_substitutions(&value_str)?;

        Ok(SetEnvAction { name, value })
    }
}

/// Unset environment variable action
#[derive(Debug, Clone, PartialEq)]
pub struct UnsetEnvAction {
    pub name: String,
}

impl UnsetEnvAction {
    pub fn from_entity<E: Entity>(entity: &E) -> Result<Self> {
        let name =
            entity
                .optional_attr_str("name")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "unset_env".to_string(),
                    attribute: "name".to_string(),
                })?;

        Ok(UnsetEnvAction { name })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::substitution::Substitution;
    use roxmltree::Document;

    #[test]
    fn test_parse_set_env_literal() {
        let xml = r#"<set_env name="ROS_DOMAIN_ID" value="42" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = SetEnvAction::from_entity(&entity).unwrap();
        assert_eq!(action.name, "ROS_DOMAIN_ID");
        assert_eq!(action.value, vec![Substitution::Text("42".to_string())]);
    }

    #[test]
    fn test_parse_set_env_with_substitution() {
        let xml = r#"<set_env name="MY_VAR" value="prefix_$(var suffix)" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = SetEnvAction::from_entity(&entity).unwrap();
        assert_eq!(action.name, "MY_VAR");
        assert_eq!(action.value.len(), 2);
        assert_eq!(action.value[0], Substitution::Text("prefix_".to_string()));
        assert!(matches!(
            action.value[1],
            Substitution::LaunchConfiguration(_)
        ));
    }

    #[test]
    fn test_parse_set_env_missing_name() {
        let xml = r#"<set_env value="42" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let result = SetEnvAction::from_entity(&entity);
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_set_env_missing_value() {
        let xml = r#"<set_env name="MY_VAR" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let result = SetEnvAction::from_entity(&entity);
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_unset_env() {
        let xml = r#"<unset_env name="MY_VAR" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = UnsetEnvAction::from_entity(&entity).unwrap();
        assert_eq!(action.name, "MY_VAR");
    }

    #[test]
    fn test_parse_unset_env_missing_name() {
        let xml = r#"<unset_env />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let result = UnsetEnvAction::from_entity(&entity);
        assert!(result.is_err());
    }
}
