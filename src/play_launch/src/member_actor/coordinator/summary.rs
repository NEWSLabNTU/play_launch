//! The ONE `MemberSummary` builder (phase-51.2) — replaces the three
//! copy-pasted construction blocks that used to live in `handle.rs`.

use super::MemberMetadata;
use crate::member_actor::model::{MemberState, MemberSummary, MemberType};
use std::path::Path;

/// Stderr file facts surfaced on every summary: mtime, size, tail preview.
pub(super) fn stderr_info(output_dir: &Path) -> (Option<u64>, u64, Option<Vec<String>>) {
    let stderr_path = output_dir.join("err");
    if let Ok(metadata) = std::fs::metadata(&stderr_path) {
        let size = metadata.len();
        let modified = metadata.modified().ok().and_then(|t| {
            t.duration_since(std::time::UNIX_EPOCH)
                .ok()
                .map(|d| d.as_secs())
        });
        let preview = if size > 0 {
            super::read_last_n_lines(&stderr_path, 5).ok()
        } else {
            None
        };
        (modified, size, preview)
    } else {
        (None, 0, None)
    }
}

/// Build a `MemberSummary` from metadata + the current mirrored state.
pub(super) fn build_summary(meta: &MemberMetadata, state: MemberState) -> MemberSummary {
    let pid = match &state {
        MemberState::Running { pid } => Some(*pid),
        _ => None,
    };
    let (stderr_last_modified, stderr_size, stderr_preview) = stderr_info(&meta.output_dir);

    MemberSummary {
        id: meta.id.clone(),
        name: meta.name.clone(),
        member_type: meta.member_type,
        state,
        pid,
        package: meta.package.clone(),
        executable: meta.executable.clone(),
        namespace: meta.namespace.clone(),
        target_container: meta.target_container.clone(),
        is_container: meta.member_type == MemberType::Container,
        exec_name: meta.exec_name.clone(),
        node_name: meta.node_name.clone(),
        stderr_last_modified,
        stderr_size,
        stderr_preview,
        respawn_enabled: meta.respawn_enabled,
        respawn_delay: meta.respawn_delay,
        auto_load: meta.auto_load,
        output_dir: meta.output_dir.clone(),
    }
}
