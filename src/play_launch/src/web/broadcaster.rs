//! Broadcaster for streaming StateEvents to SSE clients.
//!
//! Uses `tokio::sync::broadcast` so the sender never blocks — if a subscriber
//! falls behind by more than `CHANNEL_CAPACITY` events it receives a `Lagged`
//! error and the SSE handler sends a "refresh" event so the client re-syncs.

use crate::member_actor::StateEvent;
use tokio::sync::broadcast;

/// Shared broadcast buffer capacity.
///
/// During Autoware startup ~120 nodes produce Started + LoadSucceeded events
/// in rapid succession (~250 events).  512 gives comfortable headroom.
const CHANNEL_CAPACITY: usize = 512;

/// Broadcaster for StateEvents to SSE clients.
pub struct StateEventBroadcaster {
    tx: broadcast::Sender<StateEvent>,
}

impl StateEventBroadcaster {
    /// Create a new broadcaster.
    pub fn new() -> Self {
        let (tx, _) = broadcast::channel(CHANNEL_CAPACITY);
        Self { tx }
    }

    /// Subscribe to state events (for SSE connection).
    ///
    /// Returns a receiver that will receive all StateEvents broadcast after
    /// subscription.  If the receiver falls behind by more than
    /// `CHANNEL_CAPACITY`, the next `recv()` returns `RecvError::Lagged(n)`.
    pub fn subscribe(&self) -> broadcast::Receiver<StateEvent> {
        self.tx.subscribe()
    }

    /// Broadcast a state event to all subscribers.
    ///
    /// This is **synchronous** and never blocks — events are placed into a
    /// shared ring buffer.  Returns silently when there are no subscribers.
    pub fn broadcast(&self, event: StateEvent) {
        // send() only fails when there are zero receivers — that's fine.
        let _ = self.tx.send(event);
    }

    /// Get current subscriber count.
    #[allow(dead_code)]
    pub fn subscriber_count(&self) -> usize {
        self.tx.receiver_count()
    }
}

impl Default for StateEventBroadcaster {
    fn default() -> Self {
        Self::new()
    }
}
