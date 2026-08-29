//! The resolve pipeline over a parsed launch tree.
//!
//! Only the modules that need NO live ROS graph live here. `ament_index`,
//! `graph_builder`, `container_readiness`, `parameter_proxy` and the parameter
//! conversion helpers stayed in `play_launch` — they talk to a running system,
//! which is layer 3's job (RFC-0060).
pub mod causal_dag_global;
pub mod causal_graph;
pub mod graph_identity;
pub mod launch_dump;
pub mod manifest_graph;
pub mod manifest_loader;
pub mod model_builder;
pub mod sched_derive;
pub mod sched_loader;
