//! Arg action implementation

use crate::{
    error::{ParseError, Result},
    substitution::LaunchContext,
    xml::{EntityExt, XmlEntity},
};
use std::collections::HashMap;

/// Arg action representing a launch argument declaration
#[derive(Debug, Clone)]
pub struct ArgAction {
    pub name: String,
    pub default: Option<String>,
    pub description: Option<String>,
}

impl ArgAction {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        Ok(Self {
            name: entity
                .required_attr("name")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "arg".to_string(),
                    attribute: "name".to_string(),
                })?,
            default: entity.optional_attr("default")?,
            description: entity.optional_attr("description")?,
        })
    }

    /// Apply argument to context (use CLI override if available, otherwise use default)
    /// Priority: 1) Value already in context (CLI), 2) cli_args parameter, 3) default value
    pub fn apply(&self, context: &mut LaunchContext, cli_args: &HashMap<String, String>) {
        // Check if already set in context (from CLI args in constructor)
        if context.get_configuration(&self.name).is_some() {
            return; // Don't override CLI args
        }

        let value = cli_args
            .get(&self.name)
            .cloned()
            .or_else(|| self.default.clone());

        if let Some(v) = value {
            context.set_configuration(self.name.clone(), v);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::parse_xml_string;

    #[test]
    fn test_parse_arg() {
        let xml = r#"<arg name="file_path" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let arg = ArgAction::from_entity(&entity).unwrap();

        assert_eq!(arg.name, "file_path");
        assert!(arg.default.is_none());
        assert!(arg.description.is_none());
    }

    #[test]
    fn test_parse_arg_with_default() {
        let xml = r#"<arg name="file_path" default="/tmp/test.txt" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let arg = ArgAction::from_entity(&entity).unwrap();

        assert_eq!(arg.name, "file_path");
        assert_eq!(arg.default, Some("/tmp/test.txt".to_string()));
    }

    #[test]
    fn test_parse_arg_with_description() {
        let xml = r#"<arg name="file_path" description="Path to file" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let arg = ArgAction::from_entity(&entity).unwrap();

        assert_eq!(arg.description, Some("Path to file".to_string()));
    }

    #[test]
    fn test_apply_default() {
        let arg = ArgAction {
            name: "my_arg".to_string(),
            default: Some("default_val".to_string()),
            description: None,
        };

        let mut context = LaunchContext::new();
        arg.apply(&mut context, &HashMap::new());

        assert_eq!(
            context.get_configuration("my_arg"),
            Some("default_val".to_string())
        );
    }

    #[test]
    fn test_apply_cli_override() {
        let arg = ArgAction {
            name: "my_arg".to_string(),
            default: Some("default_val".to_string()),
            description: None,
        };

        let mut context = LaunchContext::new();
        let mut cli_args = HashMap::new();
        cli_args.insert("my_arg".to_string(), "cli_val".to_string());
        arg.apply(&mut context, &cli_args);

        assert_eq!(
            context.get_configuration("my_arg"),
            Some("cli_val".to_string())
        );
    }
}
