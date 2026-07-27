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
//! - **Builder**: [`MemberCoordinatorBuilder`] collects member definitions, then
//!   `spawn`s them, yielding a [`MemberHandle`] (control) + [`MemberRunner`]
//!   (state-event stream)
//!
//! # Example
//!
//! ```no_run
//! use play_launch::member_actor::{MemberCoordinatorBuilder, events::StateEvent};
//!
//! # async fn example() -> eyre::Result<()> {
//! let builder = MemberCoordinatorBuilder::new();
//!
//! // Collect members (implementation-specific)
//! // builder.add_regular_node(...);
//!
//! // Spawn them; `handle` controls the members, `runner` reports their state.
//! let (_handle, mut runner) = builder.spawn(None).await;
//!
//! // Process state events
//! while let Some(event) = runner.next_state_event().await {
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

pub mod actor_traits;
pub mod container_actor;
pub mod container_control;
pub mod coordinator;
pub mod events;
pub mod member_id;
pub mod model;
pub mod regular_node_actor;

// Re-export commonly used types
pub use coordinator::{MemberCoordinatorBuilder, MemberHandle, MemberRunner};
pub use events::StateEvent;
pub use model::{ActorConfig, MemberState, MemberSummary, MemberType};
