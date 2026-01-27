//! Launch command - record and replay launch file execution

use crate::cli::options::{LaunchArgs, ParserBackend};
use eyre::{Context, Result};
use std::{collections::HashMap, path::PathBuf, time::Instant};
use tracing::{debug, info, warn};

/// Dump launch file using Rust parser
fn dump_launch_rust(args: &LaunchArgs) -> Result<()> {
    let start = Instant::now();

    // 1. Resolve launch file path
    let launch_path = resolve_launch_file(&args.package_or_path, args.launch_file.as_deref())?;

    debug!("Parsing launch file: {}", launch_path.display());

    // 2. Parse launch arguments (KEY:=VALUE format)
    let cli_args: HashMap<String, String> = args
        .launch_arguments
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
        .map_err(|e| eyre::eyre!("Rust parser error: {}\n\nHint: If you encounter parsing issues, try the Python parser:\n  play_launch launch {} {} --parser python",
            e,
            args.package_or_path,
            args.launch_file.as_deref().unwrap_or(""),
        ))?;

    // 4. Write record.json
    let output_path = PathBuf::from("record.json");
    let json_output =
        serde_json::to_string_pretty(&record).wrap_err("Failed to serialize record.json")?;

    std::fs::write(&output_path, json_output)
        .wrap_err_with(|| format!("Failed to write record.json to {}", output_path.display()))?;

    let elapsed = start.elapsed();
    info!(
        "Parsing completed in {:.2}s (Rust parser)",
        elapsed.as_secs_f64()
    );

    Ok(())
}

/// Dump launch file using Python parser (fallback)
async fn dump_launch_python(args: &LaunchArgs) -> Result<()> {
    use crate::python::dump_launcher::DumpLauncher;

    let start = Instant::now();

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

    let elapsed = start.elapsed();
    info!(
        "Parsing completed in {:.2}s (Python parser)",
        elapsed.as_secs_f64()
    );

    Ok(())
}

/// Resolve launch file path from package name or direct path
pub(super) fn resolve_launch_file(
    package_or_path: &str,
    launch_file: Option<&str>,
) -> Result<PathBuf> {
    // Check if it looks like a direct file path
    if package_or_path.contains('/')
        || package_or_path.ends_with(".py")
        || package_or_path.ends_with(".xml")
        || package_or_path.ends_with(".yaml")
    {
        let path = PathBuf::from(package_or_path);
        if !path.exists() {
            return Err(eyre::eyre!("Launch file not found: {}", path.display()));
        }
        return Ok(path);
    }

    // Otherwise, treat as ROS package name
    let Some(file) = launch_file else {
        return Err(eyre::eyre!(
            "Launch file name required when using package name.\n\
             Usage: play_launch launch <package> <file.launch.py|xml>"
        ));
    };

    // Try to find package using ament_index (via environment variable)
    if let Ok(ament_prefix_path) = std::env::var("AMENT_PREFIX_PATH") {
        for prefix in ament_prefix_path.split(':') {
            let pkg_share = PathBuf::from(prefix).join("share").join(package_or_path);

            if pkg_share.exists() {
                // Try launch/ subdirectory first
                let launch_path = pkg_share.join("launch").join(file);
                if launch_path.exists() {
                    return Ok(launch_path);
                }

                // Try root of package share directory
                let root_path = pkg_share.join(file);
                if root_path.exists() {
                    return Ok(root_path);
                }
            }
        }
    }

    Err(eyre::eyre!(
        "Package '{}' not found or launch file '{}' doesn't exist.\n\
         Searched in: $AMENT_PREFIX_PATH/share/{}/launch/\n\
         \n\
         Make sure:\n\
         1. ROS 2 workspace is sourced (source install/setup.bash)\n\
         2. Package is built (colcon build --packages-select {})\n\
         3. Launch file exists in package/launch/ directory",
        package_or_path,
        file,
        package_or_path,
        package_or_path
    ))
}

/// Handle the 'launch' subcommand
pub fn handle_launch(args: &LaunchArgs) -> Result<()> {
    info!("Step 1/2: Recording launch execution...");

    // Choose parser based on selection
    match args.parser {
        ParserBackend::Rust => {
            // Rust mode (default) - fail if error, no fallback
            info!("Using Rust parser");
            dump_launch_rust(args)?;
        }
        ParserBackend::Python => {
            // Python mode
            info!("Using Python parser");

            // Create tokio runtime for Python parser (async)
            let worker_threads = std::cmp::min(num_cpus::get(), 8);
            let max_blocking = worker_threads * 2;
            let runtime = tokio::runtime::Builder::new_multi_thread()
                .worker_threads(worker_threads)
                .max_blocking_threads(max_blocking)
                .thread_name("play_launch-worker")
                .enable_all()
                .build()?;

            runtime.block_on(async { dump_launch_python(args).await })?;
        }
    }

    info!("Step 2/2: Replaying launch execution...");

    // Create replay args and call handle_replay
    let replay_args = crate::cli::options::ReplayArgs {
        input_file: PathBuf::from("record.json"),
        common: args.common.clone(),
    };

    super::replay::handle_replay(&replay_args)?;

    Ok(())
}
