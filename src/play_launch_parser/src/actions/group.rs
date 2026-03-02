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
}

impl GroupAction {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        let namespace = entity
            .optional_attr_str("ns")?
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        Ok(Self { namespace })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::parse_xml_string;

    #[test]
    fn test_parse_group_with_namespace() {
        let xml = r#"<group ns="/my_namespace" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let group = GroupAction::from_entity(&entity).unwrap();

        assert!(group.namespace.is_some());
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
