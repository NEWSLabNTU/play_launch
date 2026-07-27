//! Log directory creation utilities

use eyre::Context;
use std::{
    fs,
    path::{Path, PathBuf},
};

use super::timestamp::format_timestamp;

/// Create a directory to store logging data.
///
/// Creates a timestamped subdirectory under the base log directory.
/// Format: base_dir/YYYY-MM-DD_HH-MM-SS/ or base_dir/YYYY-MM-DD_HH-MM-SS-N/ if conflicts occur
pub fn create_log_dir(log_dir: &Path) -> eyre::Result<PathBuf> {
    // Create base directory if it doesn't exist
    if !log_dir.exists() {
        fs::create_dir_all(log_dir)
            .wrap_err_with(|| format!("unable to create base directory {}", log_dir.display()))?;
    }

    // Create timestamped subdirectory
    let timestamp = format_timestamp();
    let mut timestamped_dir = log_dir.join(&timestamp);

    // If directory already exists, add -1, -2, ... suffix
    if timestamped_dir.exists() {
        for n in 1..=1000 {
            timestamped_dir = log_dir.join(format!("{}-{}", timestamp, n));
            if !timestamped_dir.exists() {
                break;
            }
        }

        // Check if we exhausted all attempts
        if timestamped_dir.exists() {
            eyre::bail!(
                "unable to find available timestamped directory after 1000 attempts for timestamp {}",
                timestamp
            );
        }
    }

    fs::create_dir(&timestamped_dir)
        .wrap_err_with(|| format!("unable to create directory {}", timestamped_dir.display()))?;

    // Create or update "latest" symlink
    let latest_symlink = log_dir.join("latest");

    // Remove old symlink if it exists
    if latest_symlink.exists() || latest_symlink.symlink_metadata().is_ok() {
        let _ = fs::remove_file(&latest_symlink); // Ignore errors if symlink doesn't exist
    }

    // Create new symlink pointing to the timestamped directory
    #[cfg(unix)]
    {
        use std::os::unix::fs::symlink;
        if let Some(dir_name) = timestamped_dir.file_name() {
            symlink(dir_name, &latest_symlink).wrap_err_with(|| {
                format!(
                    "Failed to create 'latest' symlink: {} -> {}",
                    latest_symlink.display(),
                    timestamped_dir.display()
                )
            })?;
        }
    }

    Ok(timestamped_dir)
}
