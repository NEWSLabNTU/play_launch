//! XML launch file parser

use crate::error::Result;
use std::path::Path;

/// Parse XML launch file and return content and document together
pub fn parse_xml_file(path: &Path) -> Result<String> {
    let content = std::fs::read_to_string(path)?;
    Ok(content)
}

/// Validate XML string
pub fn parse_xml_string(content: &str) -> Result<()> {
    let _ = roxmltree::Document::parse(content)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::entity::{Entity, EntityExt, XmlEntity};

    #[test]
    fn test_parse_simple_xml() {
        let xml = r#"<launch>
            <node pkg="demo_nodes_cpp" exec="talker" />
        </launch>"#;

        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let root = XmlEntity::new(doc.root_element());
        assert_eq!(root.type_name(), "launch");

        let children: Vec<_> = root.children().collect();
        assert_eq!(children.len(), 1);
        assert_eq!(children[0].type_name(), "node");
    }

    #[test]
    fn test_parse_with_attributes() {
        let xml = r#"<node pkg="demo_nodes_cpp" exec="talker" name="my_talker" />"#;

        let doc = roxmltree::Document::parse(xml).unwrap();
        let root = XmlEntity::new(doc.root_element());
        assert_eq!(root.type_name(), "node");

        let pkg: String = root.required_attr("pkg").unwrap().unwrap();
        assert_eq!(pkg, "demo_nodes_cpp");

        let exec: String = root.required_attr("exec").unwrap().unwrap();
        assert_eq!(exec, "talker");

        let name: String = root.required_attr("name").unwrap().unwrap();
        assert_eq!(name, "my_talker");
    }

    #[test]
    fn test_missing_required_attribute() {
        let xml = r#"<node exec="talker" />"#;

        let doc = roxmltree::Document::parse(xml).unwrap();
        let root = XmlEntity::new(doc.root_element());
        let result: crate::error::Result<Option<String>> = root.required_attr("pkg");
        assert!(result.is_err());
    }

    #[test]
    fn test_optional_attribute() {
        let xml = r#"<node pkg="demo_nodes_cpp" exec="talker" />"#;

        let doc = roxmltree::Document::parse(xml).unwrap();
        let root = XmlEntity::new(doc.root_element());
        let name: Option<String> = root.optional_attr("name").unwrap();
        assert!(name.is_none());
    }

    #[test]
    fn test_bool_coercion() {
        let xml = r#"<node pkg="demo" exec="node" respawn="true" />"#;

        let doc = roxmltree::Document::parse(xml).unwrap();
        let root = XmlEntity::new(doc.root_element());
        let respawn: bool = root.required_attr("respawn").unwrap().unwrap();
        assert!(respawn);
    }
}
