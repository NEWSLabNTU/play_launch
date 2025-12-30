//! Event system for the event-driven architecture
//!
//! This module implements the event bus and all event types used for communication
//! between components in the refactored play_launch architecture.
//!
//! # Design
//! - All state changes go through events
//! - Event bus uses unbounded channels (events must never be dropped)
//! - Events enable reactive patterns and provide audit trail

use super::member::BlockReason;
use eyre::Result;
use tokio::sync::mpsc;

/// Events that flow through the system
#[derive(Debug, Clone)]
pub enum MemberEvent {
    // ===== Process Lifecycle (Containers & Regular Nodes) =====
    /// Process start was requested (before spawning)
    #[allow(dead_code)]
    ProcessStartRequested { name: String },

    /// Process has been spawned and is running
    ProcessStarted { name: String, pid: u32 },

    /// Process has exited (normally or abnormally)
    ProcessExited {
        name: String,
        exit_code: Option<i32>,
    },

    /// Process was successfully stopped (confirmed)
    ProcessStopped { name: String },

    /// Process failed (confirmed)
    ProcessFailed {
        name: String,
        exit_code: Option<i32>,
    },

    // ===== Composable Node Lifecycle =====
    /// Request to load a composable node
    LoadRequested { name: String },

    /// LoadNode service call has started
    LoadStarted { name: String },

    /// LoadNode service call succeeded
    #[allow(dead_code)]
    LoadSucceeded {
        name: String,
        full_node_name: String,
        unique_id: u64,
    },

    /// LoadNode service call failed
    #[allow(dead_code)]
    LoadFailed { name: String, error: String },

    /// Composable node is blocked (container unavailable)
    Blocked { name: String, reason: BlockReason },

    /// Composable node is unblocked (container became available)
    Unblocked { name: String },

    // ===== User Actions =====
    /// Generic start request (dispatched by member type)
    StartRequested { name: String },

    /// Request to stop a member
    StopRequested { name: String },

    /// Request to restart a member
    RestartRequested { name: String },

    /// Toggle respawn for a member
    RespawnToggled { name: String, enabled: bool },

    // ===== Bulk Operations =====
    /// Start all members
    StartAllRequested,

    /// Stop all members
    StopAllRequested,

    /// Restart all members
    RestartAllRequested,

    // ===== System Events =====
    /// Shutdown has been requested
    #[allow(dead_code)]
    ShutdownRequested,

    /// State of a member has changed (for web UI notifications)
    StateChanged { name: String },
}

/// Event bus for publishing and receiving events
///
/// Uses an unbounded channel to ensure events are never dropped.
/// If the event processing is slow, that's a bug to fix, not throttle.
#[derive(Clone)]
pub struct EventBus {
    tx: mpsc::UnboundedSender<MemberEvent>,
}

impl EventBus {
    /// Create a new event bus with a receiver
    ///
    /// Returns (EventBus, Receiver) where the receiver should be used by the EventProcessor
    pub fn new() -> (Self, mpsc::UnboundedReceiver<MemberEvent>) {
        let (tx, rx) = mpsc::unbounded_channel();
        (Self { tx }, rx)
    }

    /// Publish an event to the bus
    ///
    /// Returns an error if the event bus has been closed (receiver dropped)
    pub fn publish(&self, event: MemberEvent) -> Result<()> {
        self.tx
            .send(event)
            .map_err(|_| eyre::eyre!("Event bus closed"))
    }
}

impl Default for EventBus {
    fn default() -> Self {
        Self::new().0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_event_bus_basic() {
        let (bus, mut rx) = EventBus::new();

        // Publish an event
        bus.publish(MemberEvent::StartRequested {
            name: "test_node".to_string(),
        })
        .unwrap();

        // Receive the event
        let event = rx.recv().await.unwrap();
        match event {
            MemberEvent::StartRequested { name } => {
                assert_eq!(name, "test_node");
            }
            _ => panic!("Expected StartRequested event"),
        }
    }

    #[tokio::test]
    async fn test_event_bus_multiple_events() {
        let (bus, mut rx) = EventBus::new();

        // Publish multiple events
        bus.publish(MemberEvent::ProcessStarted {
            name: "node1".to_string(),
            pid: 1234,
        })
        .unwrap();

        bus.publish(MemberEvent::ProcessStarted {
            name: "node2".to_string(),
            pid: 5678,
        })
        .unwrap();

        // Receive both events in order
        let event1 = rx.recv().await.unwrap();
        match event1 {
            MemberEvent::ProcessStarted { name, pid } => {
                assert_eq!(name, "node1");
                assert_eq!(pid, 1234);
            }
            _ => panic!("Expected ProcessStarted event"),
        }

        let event2 = rx.recv().await.unwrap();
        match event2 {
            MemberEvent::ProcessStarted { name, pid } => {
                assert_eq!(name, "node2");
                assert_eq!(pid, 5678);
            }
            _ => panic!("Expected ProcessStarted event"),
        }
    }

    #[tokio::test]
    async fn test_event_bus_clone() {
        let (bus, mut rx) = EventBus::new();
        let bus_clone = bus.clone();

        // Both bus and clone can publish
        bus.publish(MemberEvent::StartRequested {
            name: "node1".to_string(),
        })
        .unwrap();

        bus_clone
            .publish(MemberEvent::StartRequested {
                name: "node2".to_string(),
            })
            .unwrap();

        // Receive both events
        let _ = rx.recv().await.unwrap();
        let _ = rx.recv().await.unwrap();
    }

    #[tokio::test]
    async fn test_event_bus_closed() {
        let (bus, rx) = EventBus::new();

        // Drop the receiver to close the bus
        drop(rx);

        // Publishing should fail
        let result = bus.publish(MemberEvent::ShutdownRequested);
        assert!(result.is_err());
    }
}
