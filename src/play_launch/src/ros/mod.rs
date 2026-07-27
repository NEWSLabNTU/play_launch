//! ROS-graph facilities for the running system.
//!
//! The launch-tree RESOLUTION pipeline moved to the `ros-launch-resolve`
//! crate (RFC-0060 layer 2) — it needs no live ROS graph, and keeping it here
//! forced every consumer to link rclrs and the colcon-generated message crates
//! to reach it. What remains talks to a running system, which is this layer's
//! job.
pub mod ament_index;
pub mod container_readiness;
pub mod graph_builder;
pub mod parameter_conversion;
pub mod parameter_proxy;
pub mod parameter_types;
