//! Coordinator for managing member actors
//!
//! Split into submodules:
//! - [`builder`]: MemberCoordinatorBuilder — collects member definitions and spawns actors
//! - [`handle`]: MemberHandle — external control and querying of running actors
//! - [`runner`]: MemberRunner — waits for actor completion and processes state events

mod builder;
mod handle;
mod runner;

pub use builder::MemberCoordinatorBuilder;
pub use handle::MemberHandle;
pub use runner::MemberRunner;

use super::web_query::MemberType;
use std::path::PathBuf;

/// Channel capacity for state events (node status changes → web UI)
pub(super) const STATE_EVENT_CHANNEL_SIZE: usize = 100;

/// Channel capacity for control events (commands → individual actors)
pub(super) const CONTROL_CHANNEL_SIZE: usize = 10;

/// Threshold for flagging a node as "noisy" (stderr output > 10 KB)
pub(super) const NOISY_STDERR_THRESHOLD: u64 = 10 * 1024;

/// Metadata about a member for web UI queries
#[derive(Clone)]
pub struct MemberMetadata {
    pub name: String,
    pub member_type: MemberType,
    pub package: Option<String>,
    pub executable: String,
    pub namespace: Option<String>,
    pub target_container: Option<String>,
    pub output_dir: PathBuf,
    pub respawn_enabled: Option<bool>,
    pub respawn_delay: Option<f64>,
    pub exec_name: Option<String>,
    pub node_name: Option<String>,
    pub auto_load: Option<bool>, // For composable nodes: auto-load when container starts
}

/// Helper function to read last N lines from a file
pub(super) fn read_last_n_lines(path: &std::path::Path, n: usize) -> eyre::Result<Vec<String>> {
    use std::io::{BufRead, BufReader};

    let file = std::fs::File::open(path)?;
    let reader = BufReader::new(file);
    let lines: Vec<String> = reader.lines().map_while(Result::ok).collect();

    let start = if lines.len() > n { lines.len() - n } else { 0 };

    Ok(lines[start..].to_vec())
}
