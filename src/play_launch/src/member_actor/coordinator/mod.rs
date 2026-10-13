//! Coordinator for managing member actors
//!
//! Split into submodules:
//! - [`builder`]: MemberCoordinatorBuilder — collects member definitions and spawns actors
//! - [`handle`]: MemberHandle — external control and querying of running actors
//! - [`runner`]: MemberRunner — waits for actor completion and processes state events

mod builder;
mod handle;
mod runner;
mod state_reducer;
pub use state_reducer::ExitRecord;
mod summary;

pub use builder::MemberCoordinatorBuilder;
pub use handle::MemberHandle;
pub use runner::MemberRunner;

use super::model::MemberType;
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
    /// Canonical collision-proof id (phase-50); assigned by the builder's
    /// IdAllocator at spawn — empty until then.
    pub id: String,
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

/// Bytes from the end of a file `read_last_n_lines` looks at: enough for the
/// few preview lines it is asked for, however large the file has grown.
const TAIL_BYTES: u64 = 64 * 1024;

/// The last `n` lines of a file, reading only its last [`TAIL_BYTES`].
///
/// It used to read the whole file and collect every line, and it runs for
/// every member on every `/api/nodes` call: a node that had logged a few
/// hundred megabytes made each node-list refresh read all of it.
pub(super) fn read_last_n_lines(path: &std::path::Path, n: usize) -> eyre::Result<Vec<String>> {
    use std::io::{Read, Seek, SeekFrom};

    let mut file = std::fs::File::open(path)?;
    let len = file.metadata()?.len();
    let start = len.saturating_sub(TAIL_BYTES);
    file.seek(SeekFrom::Start(start))?;
    let mut buf = Vec::with_capacity((len - start) as usize);
    file.read_to_end(&mut buf)?;
    let text = String::from_utf8_lossy(&buf);
    let mut lines: Vec<&str> = text.lines().collect();
    // Seeking into the middle of the file lands mid-line: drop the fragment.
    if start > 0 && !lines.is_empty() {
        lines.remove(0);
    }
    let from = lines.len().saturating_sub(n);
    Ok(lines[from..].iter().map(|l| l.to_string()).collect())
}

#[cfg(test)]
mod tail_tests {
    use super::*;

    #[test]
    fn the_last_lines_of_a_large_file_come_from_its_tail() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("err");
        let mut body = String::new();
        for i in 0..20_000 {
            body.push_str(&format!("line {i} with some padding to make it longer\n"));
        }
        std::fs::write(&path, &body).unwrap();
        assert!(body.len() as u64 > TAIL_BYTES);
        let last = read_last_n_lines(&path, 3).unwrap();
        assert_eq!(
            last,
            [
                "line 19997 with some padding to make it longer",
                "line 19998 with some padding to make it longer",
                "line 19999 with some padding to make it longer",
            ]
        );
    }

    #[test]
    fn a_short_file_returns_what_it_has() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("err");
        std::fs::write(&path, "a\nb\n").unwrap();
        assert_eq!(read_last_n_lines(&path, 5).unwrap(), ["a", "b"]);
    }
}
