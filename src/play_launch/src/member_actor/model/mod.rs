//! State model — one owner per fact (phase-51, docs/design/executor-state-ownership.md).
//!
//! - [`node_state`]: process-side state machines (regular nodes, containers)
//!   plus the actor configuration they run under.
//! - [`composable_state`]: composable-node state machine and THE single
//!   [`composable_state::BlockReason`] (the old `state::BlockReason` /
//!   `web_query::BlockReason` pair is gone).
//! - [`member_view`]: the web-facing derived view (`MemberState`,
//!   `MemberSummary`, `HealthSummary`) with `From` conversions from the
//!   actor-side states — hand-encoded mirror writes are a phase-50-era bug
//!   class, not a pattern.

pub mod composable_state;
pub mod member_view;
pub mod node_state;

pub use composable_state::{BlockReason, ComposableState};
pub use member_view::{HealthSummary, MemberState, MemberSummary, MemberType};
pub use node_state::{ActorConfig, ContainerState, NodeState};
