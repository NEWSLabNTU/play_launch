//! Let action implementation

use crate::{
    error::{ParseError, Result},
    xml::{Entity, XmlEntity},
};

/// Let action for defining scoped variables
#[derive(Debug, Clone)]
pub struct LetAction {
    pub name: String,
    pub value: String,
}

impl LetAction {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        Ok(Self {
            name: entity.required_attr_str("name")?.ok_or_else(|| {
                ParseError::MissingAttribute {
                    element: "let".to_string(),
                    attribute: "name".to_string(),
                }
            })?,
            value: entity.required_attr_str("value")?.ok_or_else(|| {
                ParseError::MissingAttribute {
                    element: "let".to_string(),
                    attribute: "value".to_string(),
                }
            })?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::parse_xml_string;

    #[test]
    fn test_parse_let() {
        let xml = r#"<let name="my_var" value="my_value" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let let_action = LetAction::from_entity(&entity).unwrap();

        assert_eq!(let_action.name, "my_var");
        assert_eq!(let_action.value, "my_value");
    }
}
