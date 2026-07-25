//! Container actor with supervision of composable nodes.
//!
//! Phase-51.4 split (docs/design/executor-state-ownership.md):
//! - [`actor`]: the `ContainerActor` — child process + container state
//!   machine + the run loop.
//! - [`supervisor`]: `ComposableSupervisor` — composable map, load/unload
//!   queue, dispatch, completions.
//! - [`ros_client`]: service clients, ComponentEvent subscription, ALL
//!   `_container` service-name construction, and the service-call impls.
//! - [`component_events`]: ComponentEvent → composable transitions
//!   (methods on the supervisor).
//! - [`process_lifecycle`]: spawn/exit/stop/restart/respawn handling
//!   (methods on the actor).
//! - [`timing`]: every Duration const in one place (knobs in phase-52).
//!
//! Note: Containers have special spawning requirements (container_state_rx
//! must be obtained before spawning), so the coordinator still creates the
//! actor directly to get the state receiver.

mod actor;
mod component_events;
mod process_lifecycle;
mod ros_client;
mod supervisor;
mod timing;

pub use actor::{ContainerActor, ContainerActorParams};
pub use supervisor::ComposableNodeMetadata;
