//! Launch command - record and replay launch file execution

use crate::cli::options::LaunchArgs;
use eyre::Context;
use std::path::PathBuf;
use tracing::info;

/// Handle the 'launch' subcommand
pub fn handle_launch(args: &LaunchArgs) -> eyre::Result<()> {
    use crate::python::dump_launcher::DumpLauncher;

    info!("Step 1/2: Recording launch execution...");

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

        // Use default record.json in current directory
        let record_path = PathBuf::from("record.json");

        launcher
            .dump_launch(
                &args.package_or_path,
                args.launch_file.as_deref(),
                &args.launch_arguments,
                &record_path,
            )
            .await?;

        Ok::<(), eyre::Report>(())
    })?;

    info!("Step 2/2: Replaying launch execution...");

    // Create replay args and call handle_replay
    let replay_args = crate::cli::options::ReplayArgs {
        input_file: PathBuf::from("record.json"),
        common: args.common.clone(),
    };

    super::replay::handle_replay(&replay_args)?;

    Ok(())
}
