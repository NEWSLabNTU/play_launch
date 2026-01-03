//! Process management utilities

pub mod cleanup;
pub mod tree;

#[cfg(unix)]
pub mod pgid;

// Re-export commonly used items
pub use cleanup::kill_all_descendants;

#[cfg(unix)]
pub use pgid::kill_process_group;
