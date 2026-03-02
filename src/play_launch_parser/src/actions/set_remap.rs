//! Topic remapping actions

use crate::{
    error::{ParseError, Result},
    substitution::{Substitution, parse_substitutions},
    xml::Entity,
};

/// Set topic remapping action
///
/// Maps a topic name globally within the current scope (e.g., group).
/// All nodes in the scope will have this remapping applied.
#[derive(Debug, Clone, PartialEq)]
pub struct SetRemapAction {
    pub from: Vec<Substitution>,
    pub to: Vec<Substitution>,
}

impl SetRemapAction {
    pub fn from_entity<E: Entity>(entity: &E) -> Result<Self> {
        let from_str =
            entity
                .optional_attr_str("from")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "set_remap".to_string(),
                    attribute: "from".to_string(),
                })?;

        let to_str =
            entity
                .optional_attr_str("to")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "set_remap".to_string(),
                    attribute: "to".to_string(),
                })?;

        let from = parse_substitutions(&from_str)?;
        let to = parse_substitutions(&to_str)?;

        Ok(SetRemapAction { from, to })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::substitution::Substitution;
    use roxmltree::Document;

    #[test]
    fn test_parse_set_remap_literal() {
        let xml = r#"<set_remap from="objects" to="/perception/objects" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = SetRemapAction::from_entity(&entity).unwrap();
        assert_eq!(action.from, vec![Substitution::Text("objects".to_string())]);
        assert_eq!(
            action.to,
            vec![Substitution::Text("/perception/objects".to_string())]
        );
    }

    #[test]
    fn test_parse_set_remap_with_substitution() {
        let xml = r#"<set_remap from="input" to="$(var topic_name)" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let action = SetRemapAction::from_entity(&entity).unwrap();
        assert_eq!(action.from, vec![Substitution::Text("input".to_string())]);
        assert_eq!(action.to.len(), 1);
        assert!(matches!(action.to[0], Substitution::LaunchConfiguration(_)));
    }

    #[test]
    fn test_parse_set_remap_missing_from() {
        let xml = r#"<set_remap to="/topic" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let result = SetRemapAction::from_entity(&entity);
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_set_remap_missing_to() {
        let xml = r#"<set_remap from="topic" />"#;
        let doc = Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());

        let result = SetRemapAction::from_entity(&entity);
        assert!(result.is_err());
    }
}
