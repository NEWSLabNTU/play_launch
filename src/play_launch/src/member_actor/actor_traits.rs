//! Actor traits for member lifecycle management
//!
//! This module defines the core trait that all member actors implement.

use eyre::Result;

/// Core trait for all member actors
///
/// Each actor is a self-contained task that manages its own lifecycle.
/// The actor runs in a loop, handling state transitions, control events,
/// and shutdown signals.
pub trait MemberActor: Send + 'static {
    /// Run the actor to completion
    ///
    /// This method consumes the actor and runs until:
    /// - The actor reaches a terminal state (Stopped/Failed)
    /// - A shutdown signal is received
    /// - An unrecoverable error occurs
    async fn run(self) -> Result<()>;

    /// Get the member name
    fn name(&self) -> &str;
}
