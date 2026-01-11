//! Broadcaster for streaming StateEvents to SSE clients

use crate::member_actor::StateEvent;
use std::sync::Arc;
use tokio::sync::{mpsc, Mutex as TokioMutex};

/// Broadcaster for StateEvents to SSE clients
pub struct StateEventBroadcaster {
    /// Subscribers (each is a channel sender to SSE connection)
    subscribers: Arc<TokioMutex<Vec<mpsc::Sender<StateEvent>>>>,
}

impl StateEventBroadcaster {
    /// Create a new broadcaster
    pub fn new() -> Self {
        Self {
            subscribers: Arc::new(TokioMutex::new(Vec::new())),
        }
    }

    /// Subscribe to state events (for SSE connection)
    ///
    /// Returns a receiver that will receive all StateEvents broadcast after subscription
    pub async fn subscribe(&self) -> mpsc::Receiver<StateEvent> {
        let (tx, rx) = mpsc::channel(100);
        let mut subs = self.subscribers.lock().await;
        subs.push(tx);
        tracing::debug!("New SSE subscriber added (total: {})", subs.len());
        rx
    }

    /// Broadcast a state event to all subscribers
    ///
    /// Automatically removes disconnected subscribers
    pub async fn broadcast(&self, event: StateEvent) {
        let mut subs = self.subscribers.lock().await;

        // Remove disconnected subscribers
        let initial_count = subs.len();
        subs.retain(|tx| !tx.is_closed());
        let removed = initial_count - subs.len();
        if removed > 0 {
            tracing::debug!("Removed {} disconnected SSE subscribers", removed);
        }

        // Send to all active subscribers
        for tx in subs.iter() {
            // Ignore send errors (subscriber might have just disconnected)
            let _ = tx.send(event.clone()).await;
        }

        if !subs.is_empty() {
            tracing::trace!(
                "Broadcast {:?} to {} subscribers",
                event.event_type(),
                subs.len()
            );
        }
    }

    /// Get current subscriber count
    #[allow(dead_code)]
    pub async fn subscriber_count(&self) -> usize {
        let subs = self.subscribers.lock().await;
        subs.len()
    }
}

impl Default for StateEventBroadcaster {
    fn default() -> Self {
        Self::new()
    }
}

// Helper trait to get event type for logging
trait EventType {
    fn event_type(&self) -> &'static str;
}

impl EventType for StateEvent {
    fn event_type(&self) -> &'static str {
        match self {
            StateEvent::Started { .. } => "Started",
            StateEvent::Exited { .. } => "Exited",
            StateEvent::Respawning { .. } => "Respawning",
            StateEvent::Terminated { .. } => "Terminated",
            StateEvent::Failed { .. } => "Failed",
            StateEvent::LoadStarted { .. } => "LoadStarted",
            StateEvent::LoadSucceeded { .. } => "LoadSucceeded",
            StateEvent::LoadFailed { .. } => "LoadFailed",
            StateEvent::Blocked { .. } => "Blocked",
            StateEvent::NodeDiscovered { .. } => "NodeDiscovered",
            StateEvent::ListNodesRequested { .. } => "ListNodesRequested",
        }
    }
}
