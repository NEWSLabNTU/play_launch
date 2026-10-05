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
/// The `ros-launch-manifest` tag this crate was built against (phase 85 I3).
pub const RLM_TAG: &str = env!("RLM_PINNED_TAG");

static CHECKER: OnceLock<String> = OnceLock::new();

/// Name the binary that is checking contracts, as its `--version` reads
/// (phase 85 I3), e.g. `play_launch 0.13.0 (v0.13.0-3-gabc1234, rlm
/// v0.1.47)`. A contract refusal ends with it, so an unknown key that is
/// really an old checker says so. First call wins, like [`set`].
pub fn set_checker(identity: impl Into<String>) {
    let _ = CHECKER.set(identity.into());
}

/// The checker's identity: what [`set_checker`] stored, else this library
/// with its pinned rlm tag.
pub fn checker() -> String {
    CHECKER
        .get()
        .cloned()
        .unwrap_or_else(|| format!("{DEFAULT_TOOL} {DEFAULT_VERSION} (rlm {RLM_TAG})"))
}

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

    #[test]
    fn the_default_checker_names_the_rlm_tag() {
        let c = super::checker();
        assert!(c.contains(&format!("rlm {}", super::RLM_TAG)), "{c}");
        assert!(super::RLM_TAG.starts_with('v'), "{}", super::RLM_TAG);
    }
}
