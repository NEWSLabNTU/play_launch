//! Scheduling apply-mode + re-export of the shared syscall core (which lives
//! in the lib so `play_launch_rt_helper` can use it too — see phase 38.10).
#![allow(dead_code)]

use clap::ValueEnum;

/// Re-exported so every existing `use crate::execution::sched_apply::{…}`
/// call site keeps compiling unchanged after the syscall core moved to the
/// lib. `execution` is a private module of the *binary* crate, so a `pub use`
/// here is not a public re-export: anything the bin doesn't itself name
/// (currently `SchedApplyError`, `thread_ids`) reads as unused.
#[allow(unused_imports)]
pub use play_launch::sched::{
    AppliedTier, SchedApplyError, SchedPolicy, apply_tier, has_sched_privilege, thread_ids,
};

/// How aggressively to apply scheduling.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, ValueEnum)]
pub enum SchedApplyMode {
    /// Resolve + report only; no syscalls.
    Off,
    /// Apply per process; log a warning and continue on failure.
    #[default]
    Warn,
    /// Any capability/apply failure aborts the run.
    Strict,
}
