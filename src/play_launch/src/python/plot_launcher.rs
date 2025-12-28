//! Wrapper for invoking play_launch_analyzer via embedded Python (PyO3)

use super::python_bridge;
use eyre::Result;
use std::path::Path;

/// Wrapper for invoking play_launch_analyzer plotting module via PyO3
pub struct PlotLauncher;

impl PlotLauncher {
    /// Create a new PlotLauncher
    pub fn new() -> Result<Self> {
        Ok(Self)
    }

    /// Execute play_launch_analyzer plotting module to generate resource plots
    ///
    /// # Arguments
    /// * `log_dir` - Specific log directory to plot
    /// * `base_log_dir` - Base log directory to search for latest execution
    /// * `output_dir` - Output directory for generated plots
    /// * `metrics` - List of metrics to plot
    /// * `list_metrics` - Whether to list available metrics and exit
    pub async fn plot(
        &self,
        log_dir: Option<&Path>,
        base_log_dir: &Path,
        output_dir: Option<&Path>,
        metrics: &[String],
        list_metrics: bool,
    ) -> Result<()> {
        // Run in a blocking task since PyO3 calls are synchronous
        let log_dir = log_dir.map(|p| p.to_path_buf());
        let base_log_dir = base_log_dir.to_path_buf();
        let output_dir = output_dir.map(|p| p.to_path_buf());
        let metrics = metrics.to_vec();

        tokio::task::spawn_blocking(move || {
            python_bridge::run_plot(
                log_dir.as_deref(),
                &base_log_dir,
                output_dir.as_deref(),
                &metrics,
                list_metrics,
            )
        })
        .await?
    }
}
