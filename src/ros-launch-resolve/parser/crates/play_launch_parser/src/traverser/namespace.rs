use super::super::LaunchTraverser;

impl LaunchTraverser {
    /// Apply namespace prefix to a path (handles both absolute and relative namespaces)
    pub(crate) fn apply_namespace_prefix(prefix: &str, path: &str) -> String {
        // If path is empty, return the prefix
        if path.is_empty() {
            return prefix.to_string();
        }

        // If path is already absolute and starts with prefix, don't duplicate
        if path.starts_with(prefix) {
            return path.to_string();
        }

        // If path is absolute (starts with /), join properly
        if path.starts_with('/') {
            // If prefix ends with / or path is just /, handle carefully
            if prefix == "/" {
                return path.to_string();
            }
            // Remove leading / from path and join with prefix
            let path_without_slash = path.strip_prefix('/').unwrap_or(path);
            if path_without_slash.is_empty() {
                return prefix.to_string();
            }
            return format!("{}/{}", prefix, path_without_slash);
        }

        // Path is relative - join with /
        if prefix.ends_with('/') {
            format!("{}{}", prefix, path)
        } else {
            format!("{}/{}", prefix, path)
        }
    }
}
