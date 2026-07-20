//! Launch command - parse, resolve, and replay a launch file in one shot
//!
//! Phase 47.B4 — the internal round-trip is fully in-memory: no
//! `record.json` (or any other file) is written to disk anywhere on this
//! path. The launch file is parsed straight into a [`LaunchDump`], a
//! SystemModel is built from it in memory, and both are handed directly to
//! the replay engine (`commands::replay::play`) — the same function
//! `play_launch replay` uses, just called in-process instead of through a
//! second CLI invocation.

use crate::cli::options::LaunchArgs;
use eyre::Result;
use std::{collections::BTreeMap, path::PathBuf};
use tracing::info;

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

/// Handle the 'launch' subcommand: parse → resolve → replay, all in memory.
pub fn handle_launch(args: &LaunchArgs) -> Result<()> {
    play_launch_parser::block_command_substitution(args.block_commands);

    let runtime = super::common::build_tokio_runtime()?;
    runtime.block_on(async move {
        info!("Step 1/3: Parsing launch file...");
        let (dump, launch_path) = super::common::parse_to_launch_dump(
            &args.package_or_path,
            args.launch_file.as_deref(),
            &args.launch_arguments,
            args.parser,
        )
        .await?;
        info!(
            "Parsed: {} node(s), {} container(s), {} composable node(s)",
            dump.node.len(),
            dump.container.len(),
            dump.load_node.len()
        );

        info!("Step 2/3: Resolving SystemModel...");
        let arg_binding: BTreeMap<String, String> =
            super::parse_launch_arguments(&args.launch_arguments)
                .into_iter()
                .collect();
        let model = super::resolve::build_checked_model(super::resolve::ModelBuildInputs {
            dump: &dump,
            launch_path: Some(&launch_path),
            arg_binding,
            contracts: args.common.contracts.as_deref(),
            no_provider_contracts: args.common.no_provider_contracts,
            sched: args.common.sched.as_deref(),
            system: None,
            target: args.common.target.as_str(),
            explain: false,
        })?;

        info!("Step 3/3: Replaying launch execution...");
        super::replay::play(dump, std::sync::Arc::new(model), &args.common).await
    })
}
