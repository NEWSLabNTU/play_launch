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
    /// `<choice value="…"/>` children.
    pub choices: Option<Vec<String>>,
}

impl ArgAction {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        use crate::xml::Entity;
        let choices: Vec<String> = entity
            .children()
            .filter(|c| c.type_name() == "choice")
            .map(|c| {
                c.optional_attr::<String>("value")
                    .ok()
                    .flatten()
                    .unwrap_or_default()
            })
            .collect();
        Ok(Self {
            name: entity
                .required_attr("name")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "arg".to_string(),
                    attribute: "name".to_string(),
                })?,
            default: entity.optional_attr("default")?,
            description: entity.optional_attr("description")?,
            choices: (!choices.is_empty()).then_some(choices),
        })
    }

    /// Execute the declaration — see [`declare`].
    pub fn apply(
        &self,
        context: &mut LaunchContext,
        cli_args: &HashMap<String, String>,
    ) -> Result<()> {
        if !context.has_configuration(&self.name)
            && let Some(v) = cli_args.get(&self.name)
        {
            context.set_configuration_literal(self.name.clone(), v.clone());
        }
        let default = self
            .default
            .as_deref()
            .map(crate::substitution::parse_substitutions)
            .transpose()?;
        declare(
            context,
            &self.name,
            default.as_deref(),
            self.description.as_deref(),
            self.choices.as_deref(),
        )
    }
}

/// `DeclareLaunchArgument.execute`, shared by every frontend: an unset
/// configuration takes the default, performed NOW (and only then — a default
/// is not evaluated when a value is already set); unset with no default is an
/// error; a value outside `choices` is an error.
///
/// The default used to be stored unperformed and resolved on every read, so
/// `<arg name="b" default="$(var a)"/>` followed by `<let name="a" .../>`
/// read the NEW `a`, and a `$(command)` or `$(anon)` default ran once per
/// read.
pub fn declare(
    context: &mut LaunchContext,
    name: &str,
    default: Option<&[crate::substitution::Substitution]>,
    description: Option<&str>,
    choices: Option<&[String]>,
) -> Result<()> {
    if !context.has_configuration(name) {
        match default {
            Some(subs) => {
                let value = crate::substitution::resolve_substitutions(subs, context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                context.set_configuration_literal(name.to_string(), value);
            }
            None => {
                return Err(ParseError::RequiredArgumentNotProvided {
                    name: name.to_string(),
                    description: description.unwrap_or("no description given").to_string(),
                    file: context
                        .current_file()
                        .map(|p| p.display().to_string())
                        .unwrap_or_default(),
                });
            }
        }
    }
    if let Some(choices) = choices
        && let Some(value) = context.get_configuration(name)
        && !choices.contains(&value)
    {
        return Err(ParseError::InvalidSubstitution(format!(
            "Argument '{name}' provided value '{value}' is not valid. Valid options are: {choices:?}"
        )));
    }
    Ok(())
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
            choices: None,
        };

        let mut context = LaunchContext::new();
        arg.apply(&mut context, &HashMap::new()).unwrap();

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
            choices: None,
        };

        let mut context = LaunchContext::new();
        let mut cli_args = HashMap::new();
        cli_args.insert("my_arg".to_string(), "cli_val".to_string());
        arg.apply(&mut context, &cli_args).unwrap();

        assert_eq!(
            context.get_configuration("my_arg"),
            Some("cli_val".to_string())
        );
    }
}
