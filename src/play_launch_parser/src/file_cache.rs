use crate::error::Result;
use dashmap::DashMap;
use once_cell::sync::Lazy;
use std::{
    path::{Path, PathBuf},
    time::SystemTime,
};

/// Cached file content with modification time
struct CachedFile {
    content: String,
    modified: SystemTime,
}

/// Global file content cache
///
/// Thread-safe, lock-free reads. Bounded by actual files in workspace.
/// Expected size for Autoware: ~50-100 files × ~50KB/file = ~5-10MB total.
static FILE_CACHE: Lazy<DashMap<PathBuf, CachedFile>> = Lazy::new(DashMap::new);

/// Read file with caching and modification time validation
pub(crate) fn read_file_cached(path: &Path) -> Result<String> {
    let metadata = std::fs::metadata(path)?;
    let modified = metadata.modified()?;

    // Check cache with modification time validation
    if let Some(entry) = FILE_CACHE.get(path)
        && entry.modified == modified
    {
        log::trace!("File cache hit: {}", path.display());
        return Ok(entry.content.clone());
    }

    log::debug!("File cache miss: {}", path.display());

    // Read and cache
    let content = std::fs::read_to_string(path)?;
    FILE_CACHE.insert(
        path.to_path_buf(),
        CachedFile {
            content: content.clone(),
            modified,
        },
    );

    Ok(content)
}
