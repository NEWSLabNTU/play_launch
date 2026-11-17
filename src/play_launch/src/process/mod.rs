//! Process management utilities

pub mod cleanup;
pub mod tree;

#[cfg(unix)]
pub mod pgid;
#[cfg(unix)]
pub mod stop;

// Re-export commonly used items
pub use cleanup::kill_all_descendants;

#[cfg(unix)]
pub use pgid::{kill_process_group, stop_leftovers, stop_process_group_staggered};
#[cfg(unix)]
pub use stop::{StopSchedule, stop_at_shutdown};
