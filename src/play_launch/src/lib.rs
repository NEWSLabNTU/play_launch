//! Shared library code for play_launch binaries
//!
//! This library provides common functionality used by both the main play_launch
//! binary and the play_launch_io_helper daemon.

// nano-ros #285 / RFC-0059 — the resolve pipeline (`ros`, `commands`) used to
// live in the BINARY crate, reachable only by running `play_launch resolve`.
// nano-ros needs that pipeline as a library so it can ship its own small,
// distinctly-named helper instead of shelling out to a `play_launch` found on
// PATH (where an unrelated ROS 2 tool of the same name shadowed it).
//
// Everything main.rs used to declare is promoted here; `main.rs` is now a thin
// entry point over this library. `extern crate self as play_launch` keeps the
// modules that already spell `play_launch::{ipc, sched}` compiling unchanged
// now that they live inside the library itself.
extern crate self as play_launch;

pub mod cli;
pub mod commands;
pub mod diagnostics;
pub mod execution;
pub mod interception;
pub mod ipc;
pub mod member_actor;
pub mod monitoring;
pub mod process;
pub mod python;
pub mod ros;
pub mod runtime_enforcement;
pub mod sched;
pub mod util;
pub mod web;
