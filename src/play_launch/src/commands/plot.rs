//! Plot command - generate resource usage visualizations

use crate::cli::options::PlotArgs;
use eyre::Context;
use tracing::info;

/// Handle the 'plot' subcommand
pub fn handle_plot(args: &PlotArgs) -> eyre::Result<()> {
    use crate::python::plot_launcher::PlotLauncher;

    info!("Generating resource usage plots...");

    // Create tokio runtime with adaptive thread pool configuration
    let worker_threads = std::cmp::min(num_cpus::get(), 8);
    let max_blocking = worker_threads * 2;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(worker_threads)
        .max_blocking_threads(max_blocking)
        .thread_name("play_launch-worker")
        .enable_all()
        .build()?;

    // Run plot phase
    runtime.block_on(async {
        let launcher = PlotLauncher::new()
            .wrap_err("Failed to initialize plotting module. Ensure Python 3 is installed.")?;

        launcher
            .plot(
                args.log_dir.as_deref(),
                &args.base_log_dir,
                args.output_dir.as_deref(),
                &args.metrics,
                args.list_metrics,
            )
            .await?;

        Ok::<(), eyre::Report>(())
    })?;

    if !args.list_metrics {
        info!("Plotting completed successfully");
    }

    Ok(())
}
