//! Group action implementation

use crate::{
    error::Result,
    substitution::{Substitution, parse_substitutions},
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
        // `<group ns="…">` / `<group namespace="…">` — the launch-XML frontend
        // sugar for a leading `<push-ros-namespace>` inside the group (the IR
        // builder pushes `namespace` onto the stack for the group's body).
        // ROS 2's launch_xml, nano-ros's `nros-launch-parser` (RFC-0024), and
        // real Autoware/nav2 launch files all accept it, so honoring it keeps
        // the resolved model's node FQNs consistent across both runtimes
        // (previously this was dropped → `/alpha/talker` resolved as
        // `/talker`). `ns` takes precedence; `namespace` is the long spelling.
        let namespace = entity
            .optional_attr_str("ns")?
            .or(entity.optional_attr_str("namespace")?)
            .map(|s| parse_substitutions(&s))
            .transpose()?;

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
    fn test_parse_group_ns_attr_honored() {
        // `<group ns="…">` is the launch-XML sugar for a leading
        // push-ros-namespace; the ns attribute must be captured (the IR
        // builder pushes it onto the namespace stack for the body).
        let xml = r#"<group ns="/my_namespace" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let group = GroupAction::from_entity(&entity).unwrap();

        assert!(
            group.namespace.is_some(),
            "ns attribute must be captured, not ignored"
        );
    }

    #[test]
    fn test_parse_group_namespace_long_attr_honored() {
        let xml = r#"<group namespace="robot1" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let group = GroupAction::from_entity(&entity).unwrap();
        assert!(group.namespace.is_some(), "namespace attribute must be captured");
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
