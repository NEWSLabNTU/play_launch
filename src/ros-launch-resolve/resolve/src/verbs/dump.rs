//! `dump` — emit a SystemModel without replaying.
//!
//! Phase 47.B2 — `record.json` is retired as a dump artifact: `dump` always
//! emits the SystemModel (the same artifact `resolve` produces), and `dump
//! run` (which had no SystemModel form — a single executable has no launch
//! scope tree to build one from) is removed. `play_launch run` already
//! covers the single-node dump+replay-in-one use case.

use std::path::PathBuf;

use eyre::Result;
use tracing::info;

use crate::verbs::ParserBackend;
use crate::verbs::resolve::ResolveInputs;

/// Everything `dump` reads. Owned plain values — see [`crate::verbs`].
pub struct DumpInputs {
    /// Package name, or a path to a launch file.
    pub package_or_path: String,
    /// Launch file name, when `package_or_path` is a package name.
    pub launch_file: Option<String>,
    /// `KEY:=VALUE` launch arguments.
    pub launch_arguments: Vec<String>,
    /// Overlay root for user-supplied contracts.
    pub contracts: Option<PathBuf>,
    /// Disable the provider-sidecar channel.
    pub no_provider_contracts: bool,
    /// Scheduling platform file (v2 `.yaml` or legacy `.toml`).
    pub sched: Option<PathBuf>,
    /// Scheduling target the platform file must declare.
    pub target: String,
    /// Which launch frontend to parse with.
    pub parser: ParserBackend,
    /// Reject any `$(command ...)` substitution while parsing.
    pub block_commands: bool,
    /// Output path. Defaults to `system_model.yaml` when `None`.
    pub output: Option<PathBuf>,
}

/// Emit the SystemModel. Delegates straight to [`super::resolve::run`] (same
/// pipeline: contract/sched channel resolution, the Phase 46.5
/// stale-Python-install check, provenance hashing) so `dump` and `resolve`
/// share one code path instead of duplicating it.
pub fn run(inputs: DumpInputs) -> Result<()> {
    play_launch_parser::block_command_substitution(inputs.block_commands);

    let output = inputs
        .output
        .clone()
        .unwrap_or_else(|| PathBuf::from("system_model.yaml"));
    info!("Emitting SystemModel (dump = resolve, Phase 46.5 convergence)...");

    super::resolve::run(ResolveInputs {
        package_or_path: inputs.package_or_path,
        launch_file: inputs.launch_file,
        launch_arguments: inputs.launch_arguments,
        bringup_root: None,
        contracts: inputs.contracts,
        no_provider_contracts: inputs.no_provider_contracts,
        sched: inputs.sched,
        system: None,
        target: inputs.target,
        parser: inputs.parser,
        out: output.display().to_string(),
        explain: false,
    })?;

    info!("To run: play_launch up {}", output.display());
    Ok(())
}
