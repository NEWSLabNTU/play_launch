//! Substitution module

pub mod context;
pub mod eval;
pub mod parser;
pub mod types;

pub use context::{ArgumentMetadata, LaunchContext};
pub use parser::parse_substitutions;
pub use types::{Substitution, resolve_substitutions};
