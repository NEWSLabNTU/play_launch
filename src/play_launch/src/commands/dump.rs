//! Dump command - emit a SystemModel without replaying
//!
//! Phase 47.B2 — `record.json` is retired as a dump artifact: `dump` always
//! emits the SystemModel (the same artifact `resolve` produces), and `dump
//! run` (which had no SystemModel form — a single executable has no launch
//! scope tree to build one from) is removed. `play_launch run` already
//! covers the single-node dump+replay-in-one use case.

use crate::cli::options::{DumpArgs, DumpSubcommand, ResolveArgs};
use eyre::Result;
use std::path::PathBuf;
use tracing::info;

/// Handle the 'dump' subcommand
pub fn handle_dump(args: &DumpArgs) -> Result<()> {
    let DumpSubcommand::Launch(launch_args) = &args.subcommand;
    play_launch_parser::block_command_substitution(launch_args.block_commands);
    dump_launch_model(launch_args, args.output.clone())?;
    Ok(())
}

/// `dump launch` — emit the SystemModel. Delegates straight to `resolve`
/// (same pipeline: contract/sched channel resolution, the Phase 46.5
/// stale-Python-install check, provenance hashing) so `dump` and `resolve`
/// share one code path instead of duplicating it.
fn dump_launch_model(
    launch_args: &crate::cli::options::LaunchArgs,
    output: Option<PathBuf>,
) -> Result<()> {
    let output = output.unwrap_or_else(|| PathBuf::from("system_model.yaml"));
    info!("Emitting SystemModel (dump = resolve, Phase 46.5 convergence)...");

    let resolve_args = ResolveArgs {
        package_or_path: launch_args.package_or_path.clone(),
        launch_file: launch_args.launch_file.clone(),
        launch_arguments: launch_args.launch_arguments.clone(),
        contracts: launch_args.common.contracts.clone(),
        no_provider_contracts: launch_args.common.no_provider_contracts,
        sched: launch_args.common.sched.clone(),
        system: None,
        target: launch_args.common.target.clone(),
        parser: launch_args.parser,
        out: output.display().to_string(),
        explain: false,
    };
    super::resolve::handle_resolve(&resolve_args)?;

    info!("To replay: play_launch replay {}", output.display());
    Ok(())
}
