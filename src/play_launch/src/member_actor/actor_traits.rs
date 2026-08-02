//! Actor traits for member lifecycle management
//!
//! This module defines the core trait that all member actors implement.

use eyre::Result;
use std::future::Future;

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
    ///
    /// Spelled `-> impl Future + Send` rather than `async fn` because every
    /// caller hands the future to `tokio::spawn`, which requires `Send`. An
    /// `async fn` in a trait leaves that bound unstated, so an implementation
    /// that captured a non-`Send` value across an await would fail at the
    /// spawn site instead of here.
    fn run(self) -> impl Future<Output = Result<()>> + Send;
}
