use eyre::{bail, Context, Result};
use std::{
    path::{Path, PathBuf},
    process::Stdio,
};
use tokio::{
    io::{AsyncBufReadExt, BufReader},
    process::Command,
};
use tracing::{debug, info};

/// Wrapper for invoking dump_launch as a subprocess
pub struct DumpLauncher {
    python_path: PathBuf,
}

impl DumpLauncher {
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
        let mut cmd = Command::new(&self.python_path);
        cmd.arg("-m")
            .arg("play_launch.dump")
            .arg(package_or_path);

        // Add launch file if provided (package-based launch)
        if let Some(file) = launch_file {
            cmd.arg(file);
        }

        // Add launch arguments
        for arg in args {
            cmd.arg(arg);
        }

        // Add output path argument
        cmd.arg("--output");
        cmd.arg(output);

        debug!("Executing command: {:?}", cmd);

        // Execute with stdout/stderr captured for streaming
        cmd.stdout(Stdio::piped()).stderr(Stdio::piped());

        let mut child = cmd
            .spawn()
            .wrap_err("Failed to spawn dump_launch subprocess")?;

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
            .wrap_err("Failed to wait for dump_launch subprocess")?;

        if !status.success() {
            bail!(
                "dump_launch failed with exit code: {}",
                status.code().unwrap_or(-1)
            );
        }

        // Verify output file was created
        if !output.exists() {
            bail!(
                "dump_launch completed but output file not found: {}",
                output.display()
            );
        }

        info!("Dump completed successfully: {}", output.display());
        Ok(())
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
        let mut cmd = Command::new(&self.python_path);
        cmd.arg("-m")
            .arg("play_launch.dump")
            .arg(package)
            .arg(executable);

        // Add node arguments
        for arg in args {
            cmd.arg(arg);
        }

        // Add output path argument
        cmd.arg("--output");
        cmd.arg(output);

        debug!("Executing command: {:?}", cmd);

        // Execute with stdout/stderr captured for streaming
        cmd.stdout(Stdio::piped()).stderr(Stdio::piped());

        let mut child = cmd
            .spawn()
            .wrap_err("Failed to spawn dump_launch subprocess")?;

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
            .wrap_err("Failed to wait for dump_launch subprocess")?;

        if !status.success() {
            bail!(
                "dump_launch failed with exit code: {}",
                status.code().unwrap_or(-1)
            );
        }

        // Verify output file was created
        if !output.exists() {
            bail!(
                "dump_launch completed but output file not found: {}",
                output.display()
            );
        }

        info!("Dump completed successfully: {}", output.display());
        Ok(())
    }
}
