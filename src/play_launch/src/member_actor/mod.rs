//! Member actor module
//!
//! This module implements an actor-per-member pattern for lifecycle management.
//! Each member (regular node, container, or composable node) runs as a self-contained
//! actor task with its own state machine.
//!
//! # Architecture
//!
//! - **Actor**: Self-contained task managing its own lifecycle
//! - **State Machine**: Explicit state enum (visible in code)
//! - **Control Events**: Commands sent TO actors (Stop, Restart, etc.)
//! - **State Events**: Status updates sent FROM actors (Started, Exited, etc.)
//! - **Coordinator**: Lightweight coordinator for spawning/tracking actors
//!
//! # Example
//!
//! ```no_run
//! use play_launch::member_actor::{events::StateEvent, MemberCoordinator};
//!
//! # async fn example() -> eyre::Result<()> {
//! let mut coordinator = MemberCoordinator::new();
//!
//! // Spawn actors (implementation-specific)
//! // coordinator.spawn_regular_node(...).await?;
//!
//! // Process state events
//! while let Some(event) = coordinator.next_state_event().await {
//!     match event {
//!         StateEvent::Started { name, pid } => {
//!             println!("Node {} started with PID {}", name, pid);
//!         }
//!         StateEvent::Exited { name, exit_code } => {
//!             println!("Node {} exited with code {:?}", name, exit_code);
//!         }
//!         _ => {}
//!     }
//! }
//!
//! # Ok(())
//! # }
//! ```

// Actors are invoked through MemberActor trait dispatch (coordinator calls actor.run()),
// so Rust's dead code analysis can't trace internal method usage.
#![allow(dead_code)]

pub mod actor_traits;
pub mod container_actor;
pub mod container_control;
pub mod coordinator;
pub mod events;
pub mod regular_node_actor;
pub mod state;
pub mod web_query;

// Re-export commonly used types
pub use coordinator::{MemberCoordinatorBuilder, MemberHandle, MemberRunner};
pub use events::StateEvent;
pub use state::ActorConfig;
pub use web_query::{MemberState, MemberSummary, MemberType};
