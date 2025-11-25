use eyre::{bail, Context, Result};
use std::{
    path::{Path, PathBuf},
    process::Stdio,
};
use tokio::{
    io::{AsyncBufReadExt, BufReader},
    process::Command,
};
use tracing::debug;

/// Wrapper for invoking play_launch_analyzer plotting module as a subprocess
pub struct PlotLauncher {
    python_path: PathBuf,
}

impl PlotLauncher {
    /// Find python3 command in PATH
    pub fn new() -> Result<Self> {
        // Find python3 binary (should be in PATH)
        let python_path = which::which("python3").wrap_err(
            "python3 command not found in PATH. \
             Ensure Python 3 is installed.",
        )?;

        debug!("Found python3 at: {}", python_path.display());

        Ok(Self { python_path })
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
        let mut cmd = Command::new(&self.python_path);

        // Execute as Python module: python3 -m play_launch.analyzer
        cmd.arg("-m").arg("play_launch.analyzer");

        // Add log directory argument if provided
        if let Some(dir) = log_dir {
            cmd.arg("--log-dir").arg(dir);
        }

        // Add base log directory argument
        cmd.arg("--base-log-dir").arg(base_log_dir);

        // Add output directory argument if provided
        if let Some(dir) = output_dir {
            cmd.arg("--output-dir").arg(dir);
        }

        // Add metrics arguments
        for metric in metrics {
            cmd.arg("--metrics").arg(metric);
        }

        // Add list-metrics flag if set
        if list_metrics {
            cmd.arg("--list-metrics");
        }

        debug!("Executing command: {:?}", cmd);

        // Execute with stdout/stderr captured for streaming
        cmd.stdout(Stdio::piped()).stderr(Stdio::piped());

        let mut child = cmd
            .spawn()
            .wrap_err("Failed to spawn play_launch_analyzer Python module")?;

        // Stream stdout
        if let Some(stdout) = child.stdout.take() {
            let reader = BufReader::new(stdout);
            let mut lines = reader.lines();

            tokio::spawn(async move {
                while let Ok(Some(line)) = lines.next_line().await {
                    println!("{}", line);
                }
            });
        }

        // Stream stderr
        if let Some(stderr) = child.stderr.take() {
            let reader = BufReader::new(stderr);
            let mut lines = reader.lines();

            tokio::spawn(async move {
                while let Ok(Some(line)) = lines.next_line().await {
                    eprintln!("{}", line);
                }
            });
        }

        // Wait for completion
        let status = child
            .wait()
            .await
            .wrap_err("Failed to wait for play_launch_analyzer Python module")?;

        if !status.success() {
            bail!(
                "play_launch_analyzer plotting failed with exit code: {}",
                status.code().unwrap_or(-1)
            );
        }

        Ok(())
    }
}
