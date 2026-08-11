//! Scheduling apply-mode + re-export of the shared syscall core (which lives
//! in the lib so `play_launch_rt_helper` can use it too — see phase 38.10).
#![allow(dead_code)]

/// Re-exported so every existing `use crate::execution::sched_apply::{…}`
/// call site keeps compiling unchanged after the syscall core moved to the
/// lib. `execution` is a private module of the *binary* crate, so a `pub use`
/// here is not a public re-export: anything the bin doesn't itself name
/// (currently `SchedApplyError`, `thread_ids`) reads as unused.
#[allow(unused_imports)]
pub use play_launch::sched::{
    AppliedTier, CpuSet, Reservation, SchedApplyError, SchedPolicy, Uclamp, apply_tier,
    has_sched_privilege, kernel_sched_support, thread_ids,
};

/// Re-exported from layer 2 (RFC-0060).
///
/// This was a second definition of the same three-variant enum. The knob is
/// resolution-time — the pipeline records the operator's intent in the model —
/// so it belongs with the pipeline, and two copies would drift the way the
/// `system.toml` deploy schema did (issue 0293). APPLYING the mode is still
/// this crate's job; that is what the syscall wrappers above are for.
pub use ros_launch_resolve::config::SchedApplyMode;
