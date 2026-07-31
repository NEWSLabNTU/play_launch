//! Include action implementation

use crate::{
    error::{ParseError, Result},
    substitution::{Substitution, parse_substitutions},
    xml::{Entity, EntityExt, XmlEntity},
};

/// Include action representing a nested launch file
#[derive(Debug, Clone)]
pub struct IncludeAction {
    pub file: Vec<Substitution>,
    /// Args as Vec to preserve order (later args can reference earlier ones).
    /// Values are parsed substitutions (resolved at use site in process_include).
    pub args: Vec<(String, Vec<Substitution>)>,
}

impl IncludeAction {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        let file_str =
            entity
                .required_attr_str("file")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "include".to_string(),
                    attribute: "file".to_string(),
                })?;
        let file = parse_substitutions(&file_str)?;

        // Parse child <arg> elements for passing arguments to the included file
        // Use Vec to preserve order (later args can reference earlier ones)
        let mut args = Vec::new();
        for child in entity.children() {
            // Child elements never reach `traverse_entity` — validate here.
            // `<arg>` under `<include>` is a distinct ROS 2 entity from a
            // top-level `<arg>` declaration (different accepted attributes —
            // see the `include-arg` spec in `xml/attr_spec.rs`), so route it
            // through that dedicated key instead of the generic `"arg"` spec
            // `validate_attrs` would derive from `child.type_name()`.
            if child.type_name() == "arg" {
                let names: Vec<&str> = child.attributes().into_iter().map(|(k, _)| k).collect();
                crate::xml::attr_spec::validate_named("include-arg", &names)?;
            } else {
                crate::xml::attr_spec::validate_attrs(&child)?;
            }
            if child.type_name() == "arg" {
                let name: String =
                    child
                        .required_attr("name")?
                        .ok_or_else(|| ParseError::MissingAttribute {
                            element: "arg".to_string(),
                            attribute: "name".to_string(),
                        })?;

                let value_str: String =
                    child
                        .required_attr("value")?
                        .ok_or_else(|| ParseError::MissingAttribute {
                            element: "arg".to_string(),
                            attribute: "value".to_string(),
                        })?;

                let value = parse_substitutions(&value_str)?;
                args.push((name, value));
            }
        }

        Ok(Self { file, args })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::parse_xml_string;

    #[test]
    fn test_parse_simple_include() {
        let xml = r#"<include file="/path/to/file.launch.xml" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let include = IncludeAction::from_entity(&entity).unwrap();

        assert_eq!(include.file.len(), 1);
        assert!(include.args.is_empty());
    }

    #[test]
    fn test_parse_include_with_args() {
        let xml = r#"<include file="/path/to/file.launch.xml">
            <arg name="param1" value="value1" />
            <arg name="param2" value="value2" />
        </include>"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let include = IncludeAction::from_entity(&entity).unwrap();

        assert_eq!(include.args.len(), 2);
        assert_eq!(include.args[0].0, "param1");
        assert_eq!(
            include.args[0].1,
            vec![Substitution::Text("value1".to_string())]
        );
        assert_eq!(include.args[1].0, "param2");
        assert_eq!(
            include.args[1].1,
            vec![Substitution::Text("value2".to_string())]
        );
    }

    #[test]
    fn test_parse_include_with_substitution() {
        let xml = r#"<include file="$(find-pkg-share demo_nodes_cpp)/launch/talker_listener.launch.xml" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let include = IncludeAction::from_entity(&entity).unwrap();

        // Should parse substitution in file path
        assert!(
            include.file.len() > 1 || matches!(include.file[0], Substitution::FindPackageShare(_))
        );
    }
}
