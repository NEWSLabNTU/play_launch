//! XML entity abstraction

use crate::error::{ParseError, Result};
use std::str::FromStr;

/// Entity trait for abstracting XML/YAML differences
pub trait Entity {
    /// Get entity type name (e.g., "node", "arg")
    fn type_name(&self) -> &str;

    /// Get attribute as string (internal implementation point)
    fn get_attr_str_impl(&self, name: &str, optional: bool) -> Result<Option<String>>;

    /// Get a required attribute as string. Returns error if missing.
    fn required_attr_str(&self, name: &str) -> Result<Option<String>> {
        self.get_attr_str_impl(name, false)
    }

    /// Get an optional attribute as string. Returns `Ok(None)` if missing.
    fn optional_attr_str(&self, name: &str) -> Result<Option<String>> {
        self.get_attr_str_impl(name, true)
    }

    /// Get all attributes as key-value pairs
    fn attributes(&self) -> Vec<(&str, &str)>;

    /// Get text content
    fn text(&self) -> Option<&str>;
}

/// Extension trait for type-aware attribute access
pub trait EntityExt: Entity {
    /// Get a required attribute with type coercion. Returns error if missing or unparseable.
    fn required_attr<T: FromStr>(&self, name: &str) -> Result<Option<T>> {
        match self.get_attr_str_impl(name, false)? {
            Some(value) => {
                let parsed = value.parse::<T>().map_err(|_| ParseError::TypeCoercion {
                    attribute: name.to_string(),
                    value: value.to_string(),
                    expected_type: std::any::type_name::<T>(),
                })?;
                Ok(Some(parsed))
            }
            None => Ok(None),
        }
    }

    /// Get an optional attribute with type coercion. Returns `Ok(None)` if missing.
    fn optional_attr<T: FromStr>(&self, name: &str) -> Result<Option<T>> {
        match self.get_attr_str_impl(name, true)? {
            Some(value) => {
                let parsed = value.parse::<T>().map_err(|_| ParseError::TypeCoercion {
                    attribute: name.to_string(),
                    value: value.to_string(),
                    expected_type: std::any::type_name::<T>(),
                })?;
                Ok(Some(parsed))
            }
            None => Ok(None),
        }
    }
}

// Blanket implementation for all Entity types
impl<T: Entity + ?Sized> EntityExt for T {}

/// XML entity implementation wrapping roxmltree::Node
pub struct XmlEntity<'a, 'input> {
    node: roxmltree::Node<'a, 'input>,
}

impl<'a, 'input> XmlEntity<'a, 'input> {
    pub fn new(node: roxmltree::Node<'a, 'input>) -> Self {
        Self { node }
    }
}

impl<'a, 'input> Entity for XmlEntity<'a, 'input> {
    fn type_name(&self) -> &str {
        self.node.tag_name().name()
    }

    fn get_attr_str_impl(&self, name: &str, optional: bool) -> Result<Option<String>> {
        match self.node.attribute(name) {
            Some(value) => Ok(Some(value.to_string())),
            None if optional => Ok(None),
            None => Err(ParseError::MissingAttribute {
                element: self.type_name().to_string(),
                attribute: name.to_string(),
            }),
        }
    }

    fn attributes(&self) -> Vec<(&str, &str)> {
        self.node
            .attributes()
            .map(|attr| (attr.name(), attr.value()))
            .collect()
    }

    fn text(&self) -> Option<&str> {
        self.node.text()
    }
}

impl<'a, 'input> XmlEntity<'a, 'input> {
    /// Return an iterator over child elements
    /// This avoids allocating a Vec on every call, enabling lazy evaluation
    pub fn children(&self) -> impl Iterator<Item = XmlEntity<'a, 'input>> {
        self.node
            .children()
            .filter(|n| n.is_element())
            .map(XmlEntity::new)
        // No .collect() - returns iterator directly
    }

    /// Return the 1-based line number of this element in the source document.
    pub fn line_number(&self) -> usize {
        let pos = self.node.range().start;
        let text_pos = self.node.document().text_pos_at(pos);
        text_pos.row as usize
    }
}
