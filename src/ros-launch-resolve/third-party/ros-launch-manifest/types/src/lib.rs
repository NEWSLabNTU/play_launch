//! Launch manifest types and YAML parser.
//!
//! This crate defines the typed AST for launch manifest files and
//! provides a parser that reads YAML with source span tracking.

pub mod cond;
pub mod parse;
pub mod span;
pub mod subst;
pub mod types;

pub use cond::{evaluate as evaluate_condition, filter_manifest};
pub use parse::{
    ParseResult, parse_manifest, parse_manifest_str, parse_manifest_str_with_spans,
    parse_manifest_with_spans,
};
pub use span::{SpanIndex, Spanned};
pub use subst::{SubstError, resolve_args, substitute_manifest, substitute_str};
pub use types::*;
