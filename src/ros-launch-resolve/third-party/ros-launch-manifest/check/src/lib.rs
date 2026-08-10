//! Static checker for launch manifest contracts.
//!
//! Builds a dataflow graph from a [`Manifest`] and runs validation rules
//! to detect inconsistencies (wiring, QoS, rate, latency budgets, drops).

pub mod check;
pub mod emit;
pub mod graph;
pub mod rules;

pub use check::{
    CheckContext, CheckResult, Diagnostic, Severity, run_checks, run_checks_with_spans,
};

/// Path to this repository's checker fixtures (`tests/fixtures/<pkg>/manifest.yaml`).
///
/// Feature-gated behind `testdata` and intended for **dependent crates' tests**.
/// `ros-launch-resolve` builds real `ManifestIndex`es from these 24 fixture
/// packages; when it consumed this repository as a submodule it reached them
/// with a hard-coded `../third-party/ros-launch-manifest/tests/fixtures`,
/// which stops resolving the moment the dependency becomes a git dependency
/// (play_launch phase-55 W2 — cargo checks git deps out under
/// `~/.cargo/git/checkouts/` at a path nothing can predict).
///
/// Exposing the path from inside the repository that owns the fixtures keeps
/// them single-sourced. The alternative — copying 24 fixture packages into
/// each consumer — would silently go stale the first time a rule or schema
/// changed here.
///
/// `CARGO_MANIFEST_DIR` is this crate's directory in whatever checkout cargo
/// produced, so `../tests/fixtures` is correct for a path dependency, a git
/// dependency and a local `cargo test` alike.
#[cfg(feature = "testdata")]
pub fn fixture_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../tests/fixtures")
}
