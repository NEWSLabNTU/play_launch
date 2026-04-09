//! XML parsing module

pub mod entity;
pub mod parser;

pub use entity::{Entity, EntityExt, XmlEntity};
pub use parser::{parse_xml_file, parse_xml_string};
