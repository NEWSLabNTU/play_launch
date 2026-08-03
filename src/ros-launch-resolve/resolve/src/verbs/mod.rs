//! The launch-tree verbs, as library functions.
//!
//! `resolve`, `dump`, `check`, `contract eject` and `plot` are each offered by
//! TWO command-line interfaces: `play_launch` (the product a user installs,
//! which links a ROS runtime) and `ros-launch-resolve` (the developer /
//! integration binary that builds with no ROS at all). There is exactly ONE
//! implementation of each verb, and it lives here.
//!
//! # Why the verbs are in the library and not in a CLI
//!
//! They used to live in `ros-launch-resolve-cli`, which meant the only way for
//! `play_launch` to offer `resolve` was to tell users to install a second
//! binary — a developer tool that was never meant to be user-visible. Copying
//! the handlers into the second CLI instead would have left two
//! implementations to drift apart. Moving them down here gives both CLIs a
//! shared body and reduces each handler to argument mapping.
//!
//! # The rule this module obeys
//!
//! **The library must not know a CLI exists.** Every entry point takes a
//! plain `*Inputs` struct of owned values — no `clap` derives, no borrowed
//! `*Args`, no dependency on either CLI's `options` module — and no entry
//! point calls [`std::process::exit`]. [`check::run`] returns the exit code it
//! *intends*; deciding what to do with it is the caller's job.

use eyre::Context as _;
use tracing::debug;

pub mod check;
pub mod contract;
pub mod dump;
pub mod plot;
pub mod resolve;

pub use check::CheckInputs;
pub use contract::ContractEjectInputs;
pub use dump::DumpInputs;
pub use plot::PlotInputs;
pub use resolve::ResolveInputs;

/// Which launch-file frontend to parse with.
///
/// Deliberately NOT a `clap::ValueEnum`: each CLI keeps its own
/// clap-derived enum and maps into this one, so the library carries no
/// knowledge of how the choice reached it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ParserBackend {
    /// The Rust parser (default, fast, no fallback on error).
    #[default]
    Rust,
    /// The Python parser (slower, maximum compatibility).
    Python,
}

/// Maximum number of Tokio worker threads (async workload is mostly idle)
const MAX_WORKER_THREADS: usize = 8;

/// Build a Tokio multi-thread runtime with adaptive thread pool configuration.
///
/// Uses number of CPUs capped at `MAX_WORKER_THREADS` for efficiency.
/// Automatically adapts to platform: 2-core Pi, 4-core laptop, 8-core AGX Orin, 32-core server.
pub fn build_tokio_runtime() -> eyre::Result<tokio::runtime::Runtime> {
    let worker_threads = std::cmp::min(num_cpus::get(), MAX_WORKER_THREADS);
    let max_blocking = worker_threads * 2;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(worker_threads)
        .max_blocking_threads(max_blocking)
        .thread_name("play_launch-worker")
        .enable_all()
        .build()?;
    debug!(
        "Tokio runtime created ({} worker threads, {} max blocking threads)",
        worker_threads, max_blocking
    );
    Ok(runtime)
}

/// Resolve a launch file path from a package name or a direct path.
pub fn resolve_launch_file(
    package_or_path: &str,
    launch_file: Option<&str>,
) -> eyre::Result<std::path::PathBuf> {
    use std::path::PathBuf;

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

/// Parse a launch file into an in-memory [`crate::ros::launch_dump::LaunchDump`]
/// — the parser-agnostic intermediate `resolve`/`launch` build a SystemModel
/// from. Phase 47.B4/B5: no user-facing artifact results from this. The
/// Rust parser path never touches disk (parse → JSON string → deserialize,
/// entirely in memory — the same trick `resolve`'s Rust branch has used
/// since Phase 46.4). The Python parser path necessarily round-trips
/// through a private OS-temp scratch file (the PyO3 `dump_launch` bridge
/// itself only knows how to write JSON to a path) that is deleted before
/// this function returns — an interop detail, not a `record.json` companion
/// left for the user.
///
/// Returns the dump plus the resolved launch file path (used for
/// provenance hashing by callers).
pub async fn parse_to_launch_dump(
    package_or_path: &str,
    launch_file: Option<&str>,
    launch_arguments: &[String],
    parser: ParserBackend,
) -> eyre::Result<(crate::ros::launch_dump::LaunchDump, std::path::PathBuf)> {
    let launch_path = resolve_launch_file(package_or_path, launch_file)?;

    let dump = match parser {
        ParserBackend::Rust => {
            let cli_args = parse_launch_arguments(launch_arguments);
            let record =
                play_launch_parser::parse_launch_file(&launch_path, cli_args).map_err(|e| {
                    eyre::eyre!(
                        "Rust parser error: {e}\n\nHint: If you encounter parsing issues, try \
                         the Python parser:\n  play_launch <cmd> {package_or_path} {} --parser \
                         python",
                        launch_file.unwrap_or("")
                    )
                })?;
            let json = serde_json::to_string_pretty(&record)?;
            serde_json::from_str(&json)?
        }
        ParserBackend::Python => {
            let scratch_path = std::env::temp_dir().join(format!(
                "play_launch-parse-{}-{}.record.json",
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_nanos())
                    .unwrap_or(0),
            ));
            let launcher = crate::python::dump_launcher::DumpLauncher::new()
                .wrap_err("Failed to initialize dump_launch. Ensure ROS workspace is sourced.")?;
            launcher
                .dump_launch(
                    package_or_path,
                    launch_file,
                    launch_arguments,
                    &scratch_path,
                )
                .await?;
            let dump =
                crate::ros::launch_dump::load_launch_dump(&scratch_path).wrap_err_with(|| {
                    format!("loading Python-parsed record {}", scratch_path.display())
                })?;
            let _ = std::fs::remove_file(&scratch_path);

            // Phase 46.5 — fail loud on a stale pre-Phase-40.1 Python install
            // (missing `ScopeOrigin.path`). Shared across every caller so
            // stale usage can never silently pass anywhere.
            crate::ros::launch_dump::ensure_python_scope_paths(&dump)?;
            dump
        }
    };

    Ok((dump, launch_path))
}

/// Parse `KEY:=VALUE` launch arguments, warning on anything that isn't.
pub fn parse_launch_arguments(args: &[String]) -> std::collections::HashMap<String, String> {
    args.iter()
        .filter_map(|arg| {
            let parts: Vec<&str> = arg.splitn(2, ":=").collect();
            if parts.len() == 2 {
                Some((parts[0].to_string(), parts[1].to_string()))
            } else {
                tracing::warn!("Ignoring invalid launch argument (expected KEY:=VALUE): {arg}");
                None
            }
        })
        .collect()
}
