//! Dump command - record launch execution without replaying

use crate::cli::options::{DumpArgs, DumpSubcommand, ParserBackend};
use eyre::{Context, Result};
use std::{collections::HashMap, time::Instant};
use tracing::{debug, info, warn};

/// Handle the 'dump' subcommand
pub fn handle_dump(args: &DumpArgs) -> Result<()> {
    info!("Recording launch execution (dump only, no replay)...");

    match &args.subcommand {
        DumpSubcommand::Launch(launch_args) => {
            // Choose parser based on selection
            match launch_args.parser {
                ParserBackend::Python => {
                    // Explicit Python mode
                    info!("Using Python parser (--parser python)");
                    dump_launch_python_wrapper(
                        &launch_args.package_or_path,
                        launch_args.launch_file.as_deref(),
                        &launch_args.launch_arguments,
                        &args.output,
                    )?;
                }
                ParserBackend::Rust => {
                    // Explicit Rust mode - fail if error, no fallback
                    info!("Using Rust parser (--parser rust)");
                    dump_launch_rust_wrapper(
                        &launch_args.package_or_path,
                        launch_args.launch_file.as_deref(),
                        &launch_args.launch_arguments,
                        &args.output,
                    )?;
                }
                ParserBackend::Auto => {
                    // Auto mode: Try Rust parser first, fallback to Python on error
                    info!("Using Rust parser (auto mode, will fallback to Python on error)");

                    match dump_launch_rust_wrapper(
                        &launch_args.package_or_path,
                        launch_args.launch_file.as_deref(),
                        &launch_args.launch_arguments,
                        &args.output,
                    ) {
                        Ok(()) => {
                            debug!("Rust parser completed successfully");
                        }
                        Err(e) => {
                            warn!("Rust parser failed: {}", e);
                            warn!("Falling back to Python parser...");

                            // Automatic fallback to Python
                            dump_launch_python_wrapper(
                                &launch_args.package_or_path,
                                launch_args.launch_file.as_deref(),
                                &launch_args.launch_arguments,
                                &args.output,
                            )
                            .wrap_err("Both Rust and Python parsers failed")?;
                        }
                    }
                }
            }
        }
        DumpSubcommand::Run(run_args) => {
            // Run mode only supported by Python (no Rust parser for single nodes)
            info!("Using Python parser (run mode)");

            let worker_threads = std::cmp::min(num_cpus::get(), 8);
            let max_blocking = worker_threads * 2;
            let runtime = tokio::runtime::Builder::new_multi_thread()
                .worker_threads(worker_threads)
                .max_blocking_threads(max_blocking)
                .thread_name("play_launch-worker")
                .enable_all()
                .build()?;

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
                        &args.output,
                    )
                    .await?;

                Ok::<(), eyre::Report>(())
            })?;
        }
    }

    info!("Dump completed successfully: {}", args.output.display());
    info!(
        "To replay: play_launch replay --input-file {}",
        args.output.display()
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
    let cli_args: HashMap<String, String> = launch_arguments
        .iter()
        .filter_map(|arg| {
            let parts: Vec<&str> = arg.splitn(2, ":=").collect();
            if parts.len() == 2 {
                Some((parts[0].to_string(), parts[1].to_string()))
            } else {
                warn!(
                    "Ignoring invalid launch argument (expected KEY:=VALUE): {}",
                    arg
                );
                None
            }
        })
        .collect();

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

    let worker_threads = std::cmp::min(num_cpus::get(), 8);
    let max_blocking = worker_threads * 2;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(worker_threads)
        .max_blocking_threads(max_blocking)
        .thread_name("play_launch-worker")
        .enable_all()
        .build()?;

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
