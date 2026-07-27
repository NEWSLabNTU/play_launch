//! Shared library code for play_launch binaries
//!
//! This library provides common functionality used by both the main play_launch
//! binary and the play_launch_io_helper daemon.

// RFC-0060 — this crate is layer 3: the Linux runtime. The launch-tree
// resolution pipeline lives in `ros-launch-resolve` (layer 2), which needs no
// ROS graph and which nano-ros and other descendants link directly.
//
// Everything main.rs used to declare is promoted here; `main.rs` is a thin
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
