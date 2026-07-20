//! Dump command - record launch execution without replaying
//!
//! Phase 46.5 — `dump` converges onto the same artifact `resolve` produces:
//! the SystemModel is now the default (and user-facing "one kind of dump");
//! `--format record` is a dev/parser-parity escape hatch that keeps writing
//! the legacy record.json (`scripts/compare_records.py`, `just
//! compare-dumps`). `dump run` (single executable, no launch scope tree) has
//! no SystemModel form and always writes record.json regardless of
//! `--format`.

use super::common::build_tokio_runtime;
use crate::cli::options::{DumpArgs, DumpFormat, DumpSubcommand, ParserBackend, ResolveArgs};
use eyre::{Context, Result};
use std::{path::PathBuf, time::Instant};
use tracing::{debug, info, warn};

/// Handle the 'dump' subcommand
pub fn handle_dump(args: &DumpArgs) -> Result<()> {
    match &args.subcommand {
        DumpSubcommand::Launch(launch_args) => {
            play_launch_parser::block_command_substitution(launch_args.block_commands);
            match args.format {
                DumpFormat::Model => dump_launch_model(launch_args, args.output.clone())?,
                DumpFormat::Record => dump_launch_record(launch_args, args.output.clone())?,
            }
        }
        DumpSubcommand::Run(run_args) => {
            if args.format == DumpFormat::Model {
                warn!(
                    "`dump run` has no SystemModel form (single executable, no launch scope \
                     tree) — writing record.json regardless of --format"
                );
            }
            let output = args
                .output
                .clone()
                .unwrap_or_else(|| PathBuf::from("record.json"));
            info!("Using Python parser (run mode)");

            let runtime = build_tokio_runtime()?;
            runtime.block_on(async {
                use crate::python::dump_launcher::DumpLauncher;

                let launcher = DumpLauncher::new().wrap_err(
                    "Failed to initialize dump_launch. Ensure ROS workspace is sourced.",
                )?;

                launcher
                    .dump_run(
                        &run_args.package,
                        &run_args.executable,
                        &run_args.args,
                        &output,
                    )
                    .await?;

                Ok::<(), eyre::Report>(())
            })?;

            info!("Dump completed successfully: {}", output.display());
        }
    }

    Ok(())
}

/// `dump launch` — the default: emit the SystemModel. Delegates straight to
/// `resolve` (same pipeline: contract/sched channel resolution, the Phase
/// 46.5 stale-Python-install check, provenance hashing) so `dump` and
/// `resolve` share one code path instead of duplicating it — "the user
/// perceives one kind of dump."
fn dump_launch_model(
    launch_args: &crate::cli::options::LaunchArgs,
    output: Option<PathBuf>,
) -> Result<()> {
    let output = output.unwrap_or_else(|| PathBuf::from("system_model.yaml"));
    info!("Emitting SystemModel (dump = resolve, Phase 46.5 convergence)...");

    let resolve_args = ResolveArgs {
        package_or_path: Some(launch_args.package_or_path.clone()),
        launch_file: launch_args.launch_file.clone(),
        record: None,
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

    info!("To replay: play_launch replay --model {}", output.display());
    Ok(())
}

/// `dump launch --format record` — the legacy record.json path, kept for
/// `scripts/compare_records.py` / `just compare-dumps` parser-parity
/// tooling. Not the user-facing default.
fn dump_launch_record(
    launch_args: &crate::cli::options::LaunchArgs,
    output: Option<PathBuf>,
) -> Result<()> {
    let output = output.unwrap_or_else(|| PathBuf::from("record.json"));

    match launch_args.parser {
        ParserBackend::Rust => {
            info!("Using Rust parser");
            dump_launch_rust_wrapper(
                &launch_args.package_or_path,
                launch_args.launch_file.as_deref(),
                &launch_args.launch_arguments,
                &output,
            )?;
        }
        ParserBackend::Python => {
            info!("Using Python parser");
            dump_launch_python_wrapper(
                &launch_args.package_or_path,
                launch_args.launch_file.as_deref(),
                &launch_args.launch_arguments,
                &output,
            )?;
        }
    }

    info!("Dump completed successfully: {}", output.display());
    info!(
        "To replay (deprecated): play_launch replay --input-file {}",
        output.display()
    );
    Ok(())
}

/// Dump launch file using Rust parser (for dump command)
fn dump_launch_rust_wrapper(
    package_or_path: &str,
    launch_file: Option<&str>,
    launch_arguments: &[String],
    output: &std::path::Path,
) -> Result<()> {
    let start = Instant::now();

    // 1. Resolve launch file path
    let launch_path = super::launch::resolve_launch_file(package_or_path, launch_file)?;

    debug!("Parsing launch file: {}", launch_path.display());

    // 2. Parse launch arguments (KEY:=VALUE format)
    let cli_args = super::parse_launch_arguments(launch_arguments);

    // 3. Call Rust parser
    let record = play_launch_parser::parse_launch_file(&launch_path, cli_args)
        .map_err(|e| eyre::eyre!("Rust parser error: {}\n\nHint: If you encounter parsing issues, try the Python parser:\n  play_launch dump launch {} {} --parser python",
            e,
            package_or_path,
            launch_file.unwrap_or(""),
        ))?;

    // 4. Write record.json
    let json_output =
        serde_json::to_string_pretty(&record).wrap_err("Failed to serialize record.json")?;

    std::fs::write(output, json_output)
        .wrap_err_with(|| format!("Failed to write record to {}", output.display()))?;

    let elapsed = start.elapsed();
    info!(
        "Parsing completed in {:.2}s (Rust parser)",
        elapsed.as_secs_f64()
    );

    Ok(())
}

/// Dump launch file using Python parser (for dump command)
fn dump_launch_python_wrapper(
    package_or_path: &str,
    launch_file: Option<&str>,
    launch_arguments: &[String],
    output: &std::path::Path,
) -> Result<()> {
    use crate::python::dump_launcher::DumpLauncher;

    let start = Instant::now();

    let runtime = build_tokio_runtime()?;
    runtime.block_on(async {
        let launcher = DumpLauncher::new()
            .wrap_err("Failed to initialize dump_launch. Ensure ROS workspace is sourced.")?;

        launcher
            .dump_launch(package_or_path, launch_file, launch_arguments, output)
            .await?;

        Ok::<(), eyre::Report>(())
    })?;

    let elapsed = start.elapsed();
    info!(
        "Parsing completed in {:.2}s (Python parser)",
        elapsed.as_secs_f64()
    );

    Ok(())
}
