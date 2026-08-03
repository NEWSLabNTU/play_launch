//! `plot` — generate resource usage visualizations from execution logs.

use std::path::PathBuf;

use eyre::Context;
use tracing::info;

/// Everything `plot` reads. Owned plain values — see [`crate::verbs`].
pub struct PlotInputs {
    /// A specific log directory to plot (e.g. `play_log/2025-10-28_16-17-56`).
    pub log_dir: Option<PathBuf>,
    /// Base log directory to search for the latest execution.
    pub base_log_dir: PathBuf,
    /// Output directory for generated plots.
    pub output_dir: Option<PathBuf>,
    /// Metrics to plot. Empty = the analyzer's defaults.
    pub metrics: Vec<String>,
    /// List available metrics and return without plotting.
    pub list_metrics: bool,
}

pub fn run(inputs: PlotInputs) -> eyre::Result<()> {
    use crate::python::plot_launcher::PlotLauncher;

    info!("Generating resource usage plots...");

    // Create tokio runtime with adaptive thread pool configuration
    let runtime = super::build_tokio_runtime()?;

    // Run plot phase
    runtime.block_on(async {
        let launcher = PlotLauncher::new()
            .wrap_err("Failed to initialize plotting module. Ensure Python 3 is installed.")?;

        launcher
            .plot(
                inputs.log_dir.as_deref(),
                &inputs.base_log_dir,
                inputs.output_dir.as_deref(),
                &inputs.metrics,
                inputs.list_metrics,
            )
            .await?;

        Ok::<(), eyre::Report>(())
    })?;

    if !inputs.list_metrics {
        info!("Plotting completed successfully");
    }

    Ok(())
}
