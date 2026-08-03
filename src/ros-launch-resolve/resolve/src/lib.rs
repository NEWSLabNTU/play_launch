//! Resolve a ROS 2 launch tree into a checked [`SystemModel`].
//!
//! Layer 2 of the launch toolchain (RFC-0060): the ROS-coupled adapter between
//! `ros-launch-manifest` (spec, theory, algorithms — no ROS, no Python) and
//! `play_launch` (the Linux runtime).
//!
//! # The invariant
//!
//! This crate builds under plain `cargo` with **no ROS install, no ament
//! environment and no colcon**. CPython is the one external requirement, and
//! only because `.launch.py` has to execute against the user's interpreter.
//!
//! Anything needing a live ROS graph — the graph client, parameter proxy,
//! container readiness — stayed in `play_launch`. That is the whole point of
//! the split: consumers like nano-ros link resolution without inheriting a
//! runtime they cannot build (nano-ros issue 0285).
//!
//! # Shape
//!
//! ```text
//! launch tree --(parser)--> LaunchDump --(ros::model_builder)--> SystemModel
//!                                       + ros::manifest_loader (contracts)
//!                                       + ros::sched_loader    (scheduling)
//! ```

pub mod config;
pub mod model;
pub mod producer;
pub mod python;
pub mod ros;
pub mod util;
pub mod verbs;
