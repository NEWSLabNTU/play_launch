//! Python bridge module for calling embedded Python modules via PyO3
//!
//! This module provides functions to call the dump_launch and play_launch_analyzer
//! Python modules directly without spawning subprocesses.

use eyre::{bail, Context, Result};
use pyo3::{prelude::*, types::PyList};
use std::path::Path;
use tracing::{debug, info};

/// Initialize Python sys.path from PYTHONPATH environment variable
///
/// PyO3's embedded Python doesn't automatically inherit PYTHONPATH,
/// so we need to manually add paths from the environment.
/// Users must source ROS2 setup.bash before running play_launch.
fn setup_python_path(py: Python<'_>) -> PyResult<()> {
    let sys = py.import("sys")?;
    let path = sys.getattr("path")?;

    // Get PYTHONPATH from environment and add to sys.path
    if let Ok(pythonpath) = std::env::var("PYTHONPATH") {
        for p in pythonpath.split(':').rev() {
            if !p.is_empty() {
                path.call_method1("insert", (0, p))?;
            }
        }
    }

    Ok(())
}

/// Run dump_launch to record a launch file execution
///
/// This calls the Python dump_launch module in a blocking thread since
/// it uses asyncio internally.
pub fn run_dump_launch(
    package_or_path: &str,
    launch_file: Option<&str>,
    args: &[String],
    output: &Path,
) -> Result<()> {
    debug!(
        "Running dump_launch via PyO3: package_or_path={}, launch_file={:?}, output={}",
        package_or_path,
        launch_file,
        output.display()
    );

    // Run in a blocking thread since dump_launch uses asyncio
    let package_or_path = package_or_path.to_string();
    let launch_file = launch_file.map(|s| s.to_string());
    let args = args.to_vec();
    let output = output.to_path_buf();
    let output_for_check = output.clone();

    std::thread::spawn(move || -> Result<()> {
        Python::with_gil(|py| {
            // Setup Python path to include ROS2 packages
            setup_python_path(py).wrap_err("Failed to setup Python path")?;

            // Build sys.argv for argparse
            let sys = py.import("sys")?;
            let argv = PyList::empty(py);

            // argv[0] is the program name
            argv.append("dump_launch")?;

            // argv[1] is package_or_path
            argv.append(&package_or_path)?;

            // argv[2] is launch_file (if provided)
            if let Some(ref lf) = launch_file {
                argv.append(lf)?;
            }

            // Append launch arguments
            for arg in &args {
                argv.append(arg)?;
            }

            // Add output argument
            argv.append("--output")?;
            argv.append(output.to_str().unwrap_or("record.json"))?;

            // Set sys.argv
            sys.setattr("argv", argv)?;

            debug!("Calling play_launch.dump.main()");

            // Import and call the main function
            let dump_module = py
                .import("play_launch.dump")
                .wrap_err("Failed to import play_launch.dump module")?;

            let main_fn = dump_module
                .getattr("main")
                .wrap_err("Failed to get main function from dump module")?;
            let result = main_fn
                .call0()
                .wrap_err("Failed to call dump_launch main()")?;

            // Check return code
            let return_code: i32 = result.extract().unwrap_or(0);
            if return_code != 0 {
                bail!("dump_launch returned non-zero exit code: {}", return_code);
            }

            Ok(())
        })
    })
    .join()
    .map_err(|e| eyre::eyre!("dump_launch thread panicked: {:?}", e))??;

    // Verify output file was created
    if !output_for_check.exists() {
        bail!(
            "dump_launch completed but output file not found: {}",
            output_for_check.display()
        );
    }

    info!(
        "Dump completed successfully: {}",
        output_for_check.display()
    );
    Ok(())
}

/// Run dump_launch for a single node (run mode)
pub fn run_dump_run(package: &str, executable: &str, args: &[String], output: &Path) -> Result<()> {
    debug!(
        "Running dump_launch (run mode) via PyO3: package={}, executable={}, output={}",
        package,
        executable,
        output.display()
    );

    let package = package.to_string();
    let executable = executable.to_string();
    let args = args.to_vec();
    let output = output.to_path_buf();
    let output_for_check = output.clone();

    std::thread::spawn(move || -> Result<()> {
        Python::with_gil(|py| {
            // Setup Python path to include ROS2 packages
            setup_python_path(py).wrap_err("Failed to setup Python path")?;

            // Build sys.argv for argparse
            let sys = py.import("sys")?;
            let argv = PyList::empty(py);

            argv.append("dump_launch")?;
            argv.append(&package)?;
            argv.append(&executable)?;

            for arg in &args {
                argv.append(arg)?;
            }

            argv.append("--output")?;
            argv.append(output.to_str().unwrap_or("record.json"))?;

            sys.setattr("argv", argv)?;

            let dump_module = py
                .import("play_launch.dump")
                .wrap_err("Failed to import play_launch.dump module")?;

            let main_fn = dump_module
                .getattr("main")
                .wrap_err("Failed to get main function from dump module")?;
            let result = main_fn
                .call0()
                .wrap_err("Failed to call dump_launch main()")?;

            let return_code: i32 = result.extract().unwrap_or(0);
            if return_code != 0 {
                bail!("dump_launch returned non-zero exit code: {}", return_code);
            }

            Ok(())
        })
    })
    .join()
    .map_err(|e| eyre::eyre!("dump_launch thread panicked: {:?}", e))??;

    if !output_for_check.exists() {
        bail!(
            "dump_launch completed but output file not found: {}",
            output_for_check.display()
        );
    }

    info!(
        "Dump completed successfully: {}",
        output_for_check.display()
    );
    Ok(())
}

/// Run play_launch_analyzer to generate resource plots
pub fn run_plot(
    log_dir: Option<&Path>,
    base_log_dir: &Path,
    output_dir: Option<&Path>,
    metrics: &[String],
    list_metrics: bool,
) -> Result<()> {
    debug!(
        "Running play_launch_analyzer via PyO3: log_dir={:?}, base_log_dir={}",
        log_dir,
        base_log_dir.display()
    );

    let log_dir = log_dir.map(|p| p.to_path_buf());
    let base_log_dir = base_log_dir.to_path_buf();
    let output_dir = output_dir.map(|p| p.to_path_buf());
    let metrics = metrics.to_vec();

    std::thread::spawn(move || -> Result<()> {
        Python::with_gil(|py| {
            // Setup Python path to include ROS2 packages
            setup_python_path(py).wrap_err("Failed to setup Python path")?;

            // Build sys.argv for argparse
            let sys = py.import("sys")?;
            let argv = PyList::empty(py);

            argv.append("play_launch_plot")?;

            if let Some(ref dir) = log_dir {
                argv.append("--log-dir")?;
                argv.append(dir.to_str().unwrap_or(""))?;
            }

            argv.append("--base-log-dir")?;
            argv.append(base_log_dir.to_str().unwrap_or("./play_log"))?;

            if let Some(ref dir) = output_dir {
                argv.append("--output-dir")?;
                argv.append(dir.to_str().unwrap_or(""))?;
            }

            for metric in &metrics {
                argv.append("--metrics")?;
                argv.append(metric)?;
            }

            if list_metrics {
                argv.append("--list-metrics")?;
            }

            sys.setattr("argv", argv)?;

            debug!("Calling play_launch.analyzer.main()");

            let analyzer_module = py
                .import("play_launch.analyzer")
                .wrap_err("Failed to import play_launch.analyzer module")?;

            let main_fn = analyzer_module
                .getattr("main")
                .wrap_err("Failed to get main function from analyzer module")?;
            main_fn
                .call0()
                .wrap_err("Failed to call play_launch_analyzer main()")?;

            Ok(())
        })
    })
    .join()
    .map_err(|e| eyre::eyre!("play_launch_analyzer thread panicked: {:?}", e))??;

    Ok(())
}
