//! Error types for the play_launch_parser

use thiserror::Error;

#[derive(Error, Debug)]
pub enum ParseError {
    #[error("XML parsing error: {0}")]
    XmlError(#[from] roxmltree::Error),

    #[error("Missing required attribute '{attribute}' on element '<{element}>'")]
    MissingAttribute { element: String, attribute: String },

    #[error("Missing required attribute '{attribute}' on element '<{element}>' in file {file}")]
    MissingAttributeWithContext {
        element: String,
        attribute: String,
        file: String,
    },

    #[error(
        "Type coercion failed for attribute '{attribute}' with value '{value}' (expected {expected_type})"
    )]
    TypeCoercion {
        attribute: String,
        value: String,
        expected_type: &'static str,
    },

    #[error("Unexpected element '<{child}>' in '<{parent}>'")]
    UnexpectedElement { parent: String, child: String },

    /// An attribute no ROS 2 action consumes. Mirrors `launch_xml`'s
    /// `assert_entity_completely_parsed()`, whose message reads
    /// `Unexpected attribute(s) found in `node`: {'machine'}`.
    #[error("Unexpected attribute(s) found in `{element}`: {{{attributes}}}")]
    UnexpectedAttribute {
        element: String,
        /// Comma-separated, sorted, each quoted: `'machine', 'bogus'`.
        attributes: String,
    },

    #[error("Invalid substitution syntax: {0}")]
    InvalidSubstitution(String),

    #[error("Invalid substitution in file {file}: {message}")]
    InvalidSubstitutionWithContext { message: String, file: String },

    #[error("File not found: {0}")]
    FileNotFound(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error(
        "Maximum include depth ({max}) exceeded at '{file}'. This may indicate a non-circular but excessively deep include chain."
    )]
    MaxIncludeDepthExceeded { max: usize, file: String },

    /// Cyclic include chain (only raised when `ParseOptions.strict_includes` is
    /// `true`; the default behavior is to log a warning and skip the cycle).
    /// `chain` is the existing include stack rendered as `a → b → c`; `file` is
    /// the canonicalized path that closes the cycle.
    #[error("Cyclic include detected: '{file}' is already in the include chain ({chain} → {file})")]
    CircularInclude { file: String, chain: String },

    #[error("Python error: {0}")]
    PythonError(String),
}

// `impl From<pyo3::PyErr>` used to live here. It was the last pyo3 reference in
// core outside `python/`, and it bought one `?` (0897 W2). The variant stays —
// reporting a Python failure is core's business; knowing pyo3's error TYPE is
// not.

#[derive(Error, Debug)]
pub enum SubstitutionError {
    #[error("Undefined variable: '{0}'. Did you forget to declare it with <arg> or <let>?")]
    UndefinedVariable(String),

    #[error(
        "Undefined environment variable: '{0}'. Make sure the variable is set in your environment."
    )]
    UndefinedEnvVar(String),

    #[error("Package '{0}' not found. Ensure the package is installed and sourced.")]
    PackageNotFound(String),

    #[error("Invalid substitution: {0}")]
    InvalidSubstitution(String),

    #[error("Command execution failed: {0}")]
    CommandFailed(String),
}

#[derive(Error, Debug)]
pub enum GenerationError {
    #[error("Substitution error: {0}")]
    Substitution(#[from] SubstitutionError),

    #[error("Package not found: {0}")]
    PackageNotFound(String),

    #[error("Executable not found: {0}")]
    ExecutableNotFound(String),

    #[error("IO error: {0}")]
    IoError(String),
}

pub type Result<T> = std::result::Result<T, ParseError>;
