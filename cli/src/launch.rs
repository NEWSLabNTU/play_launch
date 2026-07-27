//! Locating a launch file from a package name or a path.
//!
//! `handle_launch` — which parsed, resolved and then REPLAYED in one shot —
//! stayed in play_launch with the replay engine it drives. What the resolver
//! needs from this module is only the path lookup (RFC-0060).

use eyre::Result;
use std::path::PathBuf;

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
