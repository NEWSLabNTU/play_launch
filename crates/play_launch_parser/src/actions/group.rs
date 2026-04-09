//! Group action implementation

use crate::{
    error::Result,
    substitution::Substitution,
    xml::{Entity, XmlEntity},
};

/// Group action for scoping namespaces and parameters
#[derive(Debug, Clone)]
pub struct GroupAction {
    pub namespace: Option<Vec<Substitution>>,
    /// Whether this group creates an isolated scope (default: true).
    /// When false, namespace/env changes leak to subsequent siblings.
    pub scoped: bool,
}

impl GroupAction {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        // Note: <group> does NOT have a namespace/ns attribute in standard
        // ROS 2. Namespace is set via <push-ros-namespace> inside the group.
        // We don't parse ns here — if present, it's silently ignored
        // (same as ROS 2 which rejects it with ValueError).
        let namespace = None;

        // Parse scoped attribute (default: true)
        let scoped = entity
            .optional_attr_str("scoped")?
            .map(|s| !s.eq_ignore_ascii_case("false"))
            .unwrap_or(true);

        Ok(Self { namespace, scoped })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::parse_xml_string;

    #[test]
    fn test_parse_group_with_ns_attr_ignored() {
        // <group ns="..."> is non-standard. The ns attribute is ignored.
        let xml = r#"<group ns="/my_namespace" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let group = GroupAction::from_entity(&entity).unwrap();

        assert!(group.namespace.is_none(), "ns attribute should be ignored");
    }

    #[test]
    fn test_parse_group_scoped_false() {
        let xml = r#"<group scoped="false" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let group = GroupAction::from_entity(&entity).unwrap();

        assert!(!group.scoped, "scoped=false should be parsed");
    }

    #[test]
    fn test_parse_group_scoped_default() {
        let xml = r#"<group />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let group = GroupAction::from_entity(&entity).unwrap();

        assert!(group.scoped, "default scoped should be true");
    }

    #[test]
    fn test_parse_group_without_namespace() {
        let xml = r#"<group />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let group = GroupAction::from_entity(&entity).unwrap();

        assert!(group.namespace.is_none());
    }
}
