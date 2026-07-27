//! Pipeline configuration shared by the library and its callers.
//!
//! These live here rather than in a CLI crate because the PIPELINE branches on
//! them. Keeping them next to `clap` argument definitions would invert the
//! layering — a library reaching into its own consumer's option types, which
//! is what `util::logging` used to do (it took the whole `Options` enum just
//! to read a `verbose` flag).

/// Which container implementation a composable node should run under.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum ContainerMode {
    /// `play_launch_container` with ComponentEvent publishing.
    Observable,
    /// `play_launch_container` with `clone(CLONE_VM)` per-node isolation.
    Isolated,
    /// The container named by the launch file, unmodified.
    Stock,
}

/// How aggressively scheduling is applied at run time.
///
/// A resolution-time knob: the pipeline records the operator's intent in the
/// model. ACTUALLY applying it is `sched_setscheduler`, which lives in
/// `play_launch` with the rest of the Linux runtime — the plan builder and the
/// syscall core stayed there (RFC-0060 layer 3).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, clap::ValueEnum)]
pub enum SchedApplyMode {
    /// Resolve + report only; no syscalls.
    Off,
    /// Apply per process; log a warning and continue on failure.
    #[default]
    Warn,
    /// Any capability/apply failure aborts the run.
    Strict,
}
