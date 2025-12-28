use eyre::{bail, Context};
use std::{env, fs, path::PathBuf};

/// Get the prefix path for a ROS package by searching AMENT_PREFIX_PATH.
///
/// This function searches through the directories listed in AMENT_PREFIX_PATH
/// environment variable for the ament index marker file for the given package.
///
/// # Arguments
/// * `package_name` - Name of the ROS package to find
///
/// # Returns
/// The prefix path where the package is installed, or an error if not found
pub fn get_package_prefix(package_name: &str) -> eyre::Result<PathBuf> {
    let ament_prefix_path =
        env::var("AMENT_PREFIX_PATH").context("AMENT_PREFIX_PATH environment variable not set")?;

    for prefix in ament_prefix_path.split(':') {
        let marker_path = PathBuf::from(prefix)
            .join("share/ament_index/resource_index/packages")
            .join(package_name);

        if marker_path.exists() {
            return Ok(PathBuf::from(prefix));
        }
    }

    bail!("Package '{}' not found in AMENT_PREFIX_PATH", package_name)
}

/// Find the full path to an executable within a ROS package.
///
/// This function locates the package using get_package_prefix(), then searches
/// the package's lib directory for the specified executable.
///
/// # Arguments
/// * `package_name` - Name of the ROS package containing the executable
/// * `executable_name` - Name of the executable file to find
///
/// # Returns
/// The full path to the executable, or an error if not found
pub fn find_executable(package_name: &str, executable_name: &str) -> eyre::Result<PathBuf> {
    let prefix = get_package_prefix(package_name)
        .wrap_err_with(|| format!("Failed to find package '{}'", package_name))?;

    let lib_dir = prefix.join("lib").join(package_name);

    if !lib_dir.exists() {
        bail!(
            "Package '{}' library directory not found at {}",
            package_name,
            lib_dir.display()
        );
    }

    // Search for the executable in the lib directory and subdirectories
    find_executable_in_dir(&lib_dir, executable_name).wrap_err_with(|| {
        format!(
            "Executable '{}' not found in package '{}'",
            executable_name, package_name
        )
    })
}

/// Recursively search for an executable file in a directory.
fn find_executable_in_dir(dir: &PathBuf, executable_name: &str) -> eyre::Result<PathBuf> {
    for entry in
        fs::read_dir(dir).wrap_err_with(|| format!("Failed to read directory {}", dir.display()))?
    {
        let entry = entry?;
        let path = entry.path();

        if path.is_file() {
            if let Some(file_name) = path.file_name() {
                if file_name == executable_name {
                    // Verify it's actually executable
                    #[cfg(unix)]
                    {
                        use std::os::unix::fs::PermissionsExt;
                        let metadata = fs::metadata(&path)?;
                        let permissions = metadata.permissions();
                        if permissions.mode() & 0o111 != 0 {
                            return Ok(path);
                        }
                    }
                    #[cfg(not(unix))]
                    {
                        return Ok(path);
                    }
                }
            }
        } else if path.is_dir() {
            // Recursively search subdirectories
            if let Ok(found) = find_executable_in_dir(&path, executable_name) {
                return Ok(found);
            }
        }
    }

    bail!(
        "Executable '{}' not found in {}",
        executable_name,
        dir.display()
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_package_prefix_existing() {
        // Test with std_msgs which should always be available in a ROS environment
        let result = get_package_prefix("std_msgs");
        assert!(result.is_ok(), "std_msgs package should be found");

        let prefix = result.unwrap();
        assert!(
            prefix.join("share/std_msgs").exists(),
            "Package share directory should exist"
        );
    }

    #[test]
    fn test_get_package_prefix_nonexistent() {
        let result = get_package_prefix("nonexistent_package_12345");
        assert!(result.is_err(), "Nonexistent package should return error");
        assert!(
            result.unwrap_err().to_string().contains("not found"),
            "Error should mention package not found"
        );
    }

    #[test]
    fn test_find_executable_component_container() {
        // Test finding component_container which should exist in rclcpp_components
        let result = find_executable("rclcpp_components", "component_container");
        assert!(
            result.is_ok(),
            "component_container executable should be found"
        );

        let exe_path = result.unwrap();
        assert!(exe_path.exists(), "Executable path should exist");
        assert!(exe_path.is_file(), "Executable should be a file");

        // Verify it's in the correct package directory
        assert!(
            exe_path.to_string_lossy().contains("rclcpp_components"),
            "Executable should be in rclcpp_components directory"
        );
    }

    #[test]
    fn test_find_executable_component_container_mt() {
        // Test finding component_container_mt variant
        let result = find_executable("rclcpp_components", "component_container_mt");
        assert!(
            result.is_ok(),
            "component_container_mt executable should be found"
        );

        let exe_path = result.unwrap();
        assert!(exe_path.exists(), "Executable path should exist");
    }

    #[test]
    fn test_find_executable_nonexistent_package() {
        let result = find_executable("nonexistent_package_12345", "fake_executable");
        assert!(result.is_err(), "Nonexistent package should return error");
    }

    #[test]
    fn test_find_executable_nonexistent_executable() {
        let result = find_executable("std_msgs", "nonexistent_executable_12345");
        assert!(
            result.is_err(),
            "Nonexistent executable should return error"
        );
        assert!(
            result.unwrap_err().to_string().contains("not found"),
            "Error should mention executable not found"
        );
    }

    #[test]
    fn test_ament_prefix_path_required() {
        // Save current AMENT_PREFIX_PATH
        let original = env::var("AMENT_PREFIX_PATH").ok();

        // Use a closure to ensure restoration even on panic
        let test_result = std::panic::catch_unwind(|| {
            // Temporarily unset it
            env::remove_var("AMENT_PREFIX_PATH");

            let result = get_package_prefix("std_msgs");
            assert!(
                result.is_err(),
                "Should error when AMENT_PREFIX_PATH not set"
            );
            assert!(
                result
                    .unwrap_err()
                    .to_string()
                    .contains("AMENT_PREFIX_PATH"),
                "Error should mention AMENT_PREFIX_PATH"
            );
        });

        // Always restore original value, even if test panicked
        if let Some(path) = original {
            env::set_var("AMENT_PREFIX_PATH", path);
        }

        // Re-panic if test failed
        if let Err(e) = test_result {
            std::panic::resume_unwind(e);
        }
    }
}
