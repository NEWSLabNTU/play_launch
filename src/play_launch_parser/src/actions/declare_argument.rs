//! Declare argument action for argument metadata and validation

use crate::{
    error::{ParseError, Result},
    substitution::{Substitution, parse_substitutions},
    xml::Entity,
};

/// Declare argument action with metadata
#[derive(Debug, Clone, PartialEq)]
pub struct DeclareArgumentAction {
    pub name: String,
    pub default: Option<Vec<Substitution>>,
    pub description: Option<String>,
    pub choices: Option<Vec<String>>,
}

impl DeclareArgumentAction {
    pub fn from_entity<E: Entity>(entity: &E) -> Result<Self> {
        let name =
            entity
                .optional_attr_str("name")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "declare_argument".to_string(),
                    attribute: "name".to_string(),
                })?;

        let default = if let Some(default_str) = entity.optional_attr_str("default")? {
            Some(parse_substitutions(&default_str)?)
        } else {
            None
        };

        let description = entity.optional_attr_str("description")?;

        // Parse choices if present (comma-separated string)
        let choices = entity.optional_attr_str("choices")?.map(|choices_str| {
            choices_str
                .split(',')
                .map(|s| s.trim().to_string())
                .collect()
        });

        Ok(DeclareArgumentAction {
            name,
            default,
            description,
            choices,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::substitution::Substitution;
    use roxmltree::Document;

    #[test]
    fn test_parse_declare_argument_minimal() {
        let xml = r#"<declare_argument name="my_arg" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = DeclareArgumentAction::from_entity(&entity).unwrap();
        assert_eq!(action.name, "my_arg");
        assert!(action.default.is_none());
        assert!(action.description.is_none());
        assert!(action.choices.is_none());
    }

    #[test]
    fn test_parse_declare_argument_with_default() {
        let xml = r#"<declare_argument name="my_arg" default="default_value" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = DeclareArgumentAction::from_entity(&entity).unwrap();
        assert_eq!(action.name, "my_arg");
        assert_eq!(
            action.default,
            Some(vec![Substitution::Text("default_value".to_string())])
        );
    }

    #[test]
    fn test_parse_declare_argument_with_description() {
        let xml = r#"<declare_argument name="my_arg" description="This is a test argument" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = DeclareArgumentAction::from_entity(&entity).unwrap();
        assert_eq!(action.name, "my_arg");
        assert_eq!(
            action.description,
            Some("This is a test argument".to_string())
        );
    }

    #[test]
    fn test_parse_declare_argument_with_choices() {
        let xml = r#"<declare_argument name="mode" choices="fast, slow, medium" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = DeclareArgumentAction::from_entity(&entity).unwrap();
        assert_eq!(action.name, "mode");
        assert_eq!(
            action.choices,
            Some(vec![
                "fast".to_string(),
                "slow".to_string(),
                "medium".to_string()
            ])
        );
    }

    #[test]
    fn test_parse_declare_argument_full() {
        let xml = r#"<declare_argument
            name="robot_name"
            default="robot1"
            description="Name of the robot"
            choices="robot1, robot2, robot3"
        />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = DeclareArgumentAction::from_entity(&entity).unwrap();
        assert_eq!(action.name, "robot_name");
        assert_eq!(
            action.default,
            Some(vec![Substitution::Text("robot1".to_string())])
        );
        assert_eq!(action.description, Some("Name of the robot".to_string()));
        assert_eq!(
            action.choices,
            Some(vec![
                "robot1".to_string(),
                "robot2".to_string(),
                "robot3".to_string()
            ])
        );
    }

    #[test]
    fn test_parse_declare_argument_missing_name() {
        let xml = r#"<declare_argument default="value" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let result = DeclareArgumentAction::from_entity(&entity);
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_declare_argument_with_substitution_in_default() {
        let xml = r#"<declare_argument name="my_arg" default="prefix_$(var other)" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = DeclareArgumentAction::from_entity(&entity).unwrap();
        assert_eq!(action.name, "my_arg");
        assert!(action.default.is_some());
        let default = action.default.unwrap();
        assert_eq!(default.len(), 2);
        assert_eq!(default[0], Substitution::Text("prefix_".to_string()));
        assert!(matches!(default[1], Substitution::LaunchConfiguration(_)));
    }
}
