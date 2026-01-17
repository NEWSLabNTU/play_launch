//! Dump command - record launch execution without replaying

use crate::cli::options::DumpArgs;
use eyre::Context;
use tracing::info;

/// Handle the 'dump' subcommand
pub fn handle_dump(args: &DumpArgs) -> eyre::Result<()> {
    use crate::python::dump_launcher::DumpLauncher;

    info!("Recording launch execution (dump only, no replay)...");

    // Create tokio runtime with adaptive thread pool configuration
    let worker_threads = std::cmp::min(num_cpus::get(), 8);
    let max_blocking = worker_threads * 2;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(worker_threads)
        .max_blocking_threads(max_blocking)
        .thread_name("play_launch-worker")
        .enable_all()
        .build()?;

    // Run dump phase
    runtime.block_on(async {
        let launcher = DumpLauncher::new()
            .wrap_err("Failed to initialize dump_launch. Ensure ROS workspace is sourced.")?;

        match &args.subcommand {
            crate::cli::options::DumpSubcommand::Launch(launch_args) => {
                launcher
                    .dump_launch(
                        &launch_args.package_or_path,
                        launch_args.launch_file.as_deref(),
                        &launch_args.launch_arguments,
                        &args.output,
                    )
                    .await?;
            }
            crate::cli::options::DumpSubcommand::Run(run_args) => {
                launcher
                    .dump_run(
                        &run_args.package,
                        &run_args.executable,
                        &run_args.args,
                        &args.output,
                    )
                    .await?;
            }
        }

        Ok::<(), eyre::Report>(())
    })?;

    info!("Dump completed successfully: {}", args.output.display());
    info!(
        "To replay: play_launch replay --input-file {}",
        args.output.display()
    );

    Ok(())
}
