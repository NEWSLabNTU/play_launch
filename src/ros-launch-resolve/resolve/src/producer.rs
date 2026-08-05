//! Who produced this SystemModel — the string stamped into
//! `meta.resolver.{tool,version}`.
//!
//! # Why this is not `env!("CARGO_PKG_NAME")`
//!
//! The model builder lives in this LIBRARY, but the library is not a thing
//! anybody runs. Two binaries link it: `play_launch` (the product a user
//! installs from PyPI) and `ros-launch-resolve` (the developer/nano-ros
//! integration binary that never ships in the wheel). Reading this crate's
//! own `CARGO_PKG_*` stamped the LIBRARY's name and version into every model
//! — including the ones `play_launch resolve` writes and users commit to git
//! — naming a binary they were never given, at a version unrelated to the
//! `0.9.0` they installed. `meta.resolver` exists so a consumer can tell what
//! built the artifact, so it has to name the thing that was actually run.
//!
//! # The contract
//!
//! Each CLI calls [`set`] once, early in `main`, with its own binary name and
//! version. The library never calls it. Callers that don't (a test, or a
//! consumer linking this crate directly) get the honest [`DEFAULT_TOOL`]
//! fallback rather than a wrong answer.

use std::sync::OnceLock;

/// Stamped when no CLI announced itself — e.g. a direct library consumer.
pub const DEFAULT_TOOL: &str = "ros-launch-resolve";
/// Version paired with [`DEFAULT_TOOL`]: this library crate's own.
pub const DEFAULT_VERSION: &str = env!("CARGO_PKG_VERSION");

static PRODUCER: OnceLock<(String, String)> = OnceLock::new();

/// Announce the running binary's identity. First call wins; later calls are
/// ignored (a `OnceLock`, so this is safe to call from anywhere and cannot
/// tear). Intended to be called once from each CLI's `main`.
pub fn set(tool: impl Into<String>, version: impl Into<String>) {
    let _ = PRODUCER.set((tool.into(), version.into()));
}

/// The `(tool, version)` pair to stamp into `meta.resolver`.
pub fn get() -> (String, String) {
    PRODUCER
        .get()
        .cloned()
        .unwrap_or_else(|| (DEFAULT_TOOL.to_string(), DEFAULT_VERSION.to_string()))
}

#[cfg(test)]
mod tests {
    /// Nothing announced itself in a unit-test binary, so the fallback is
    /// what a direct library consumer sees. Pinning it keeps the default
    /// honest rather than empty.
    #[test]
    fn the_default_names_this_library() {
        let (tool, version) = super::get();
        assert_eq!(tool, super::DEFAULT_TOOL);
        assert!(!version.is_empty());
    }
}
