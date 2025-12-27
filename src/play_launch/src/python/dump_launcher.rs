//! Wrapper for invoking dump_launch via embedded Python (PyO3)

use super::python_bridge;
use eyre::Result;
use std::path::Path;

/// Wrapper for invoking dump_launch via PyO3
pub struct DumpLauncher;

impl DumpLauncher {
    /// Create a new DumpLauncher
    pub fn new() -> Result<Self> {
        Ok(Self)
    }

    /// Execute dump_launch for a launch file
    ///
    /// # Arguments
    /// * `package_or_path` - Package name or path to launch file
    /// * `launch_file` - Launch file name (if package_or_path is a package)
    /// * `args` - Additional launch arguments
    /// * `output` - Path where record.json will be written
    pub async fn dump_launch(
        &self,
        package_or_path: &str,
        launch_file: Option<&str>,
        args: &[String],
        output: &Path,
    ) -> Result<()> {
        // Run in a blocking task since PyO3 calls are synchronous
        let package_or_path = package_or_path.to_string();
        let launch_file = launch_file.map(|s| s.to_string());
        let args = args.to_vec();
        let output = output.to_path_buf();

        tokio::task::spawn_blocking(move || {
            python_bridge::run_dump_launch(&package_or_path, launch_file.as_deref(), &args, &output)
        })
        .await?
    }

    /// Execute dump_launch for a single node (run mode)
    ///
    /// # Arguments
    /// * `package` - Package name
    /// * `executable` - Executable name
    /// * `args` - Node arguments
    /// * `output` - Path where record.json will be written
    pub async fn dump_run(
        &self,
        package: &str,
        executable: &str,
        args: &[String],
        output: &Path,
    ) -> Result<()> {
        let package = package.to_string();
        let executable = executable.to_string();
        let args = args.to_vec();
        let output = output.to_path_buf();

        tokio::task::spawn_blocking(move || {
            python_bridge::run_dump_run(&package, &executable, &args, &output)
        })
        .await?
    }
}
