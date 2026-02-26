//! Runner that waits for all actors to complete

use crate::member_actor::{events::StateEvent, web_query::MemberState};
use eyre::Result;
use std::{collections::HashMap, sync::Arc};
use tokio::{sync::mpsc, task::JoinHandle};

/// Runner that waits for all actors to complete
/// Takes mut self - no Arc<Mutex> needed!
pub struct MemberRunner {
    /// Task handles for all actors
    tasks: HashMap<String, JoinHandle<Result<()>>>,
    /// Receiver for state events from actors
    state_rx: mpsc::Receiver<StateEvent>,
    /// Shared state map (actors write directly, runner only reads for logging)
    shared_state: Arc<dashmap::DashMap<String, MemberState>>,
}

impl MemberRunner {
    /// Create a new MemberRunner (called from builder)
    pub(super) fn new(
        tasks: HashMap<String, JoinHandle<Result<()>>>,
        state_rx: mpsc::Receiver<StateEvent>,
        shared_state: Arc<dashmap::DashMap<String, MemberState>>,
    ) -> Self {
        Self {
            tasks,
            state_rx,
            shared_state,
        }
    }

    /// Get the next state event (for web UI forwarding)
    pub async fn next_state_event(&mut self) -> Option<StateEvent> {
        self.state_rx.recv().await
    }

    /// Wait for all actors to complete (takes mut self!)
    pub async fn wait_for_completion(self) -> Result<()> {
        use futures::stream::{FuturesUnordered, StreamExt};

        // Extract fields from self
        let Self {
            tasks,
            mut state_rx,
            shared_state: _shared_state,
        } = self;

        // Track task count before moving into FuturesUnordered
        let task_count = tasks.len();
        tracing::debug!("wait_for_completion: starting with {} tasks", task_count);

        // Move tasks into FuturesUnordered for concurrent completion handling
        let mut task_futures = FuturesUnordered::from_iter(
            tasks
                .into_iter()
                .map(|(name, task)| async move { (name, task.await) }),
        );

        let mut errors = Vec::new();
        let mut remaining_tasks = task_count;
        tracing::debug!("wait_for_completion: remaining_tasks = {}", remaining_tasks);

        // Process state events and task completions concurrently
        loop {
            tokio::select! {
                // Drain state events (actors write directly to shared_state)
                Some(event) = state_rx.recv() => {
                    tracing::debug!("State event: {:?}", event);
                    // Actors update shared_state directly, no need to update from events
                }

                // Process task completions
                Some((name, result)) = task_futures.next() => {
                    match result {
                        Ok(Ok(())) => {
                            tracing::debug!("Actor {} completed successfully", name);
                        }
                        Ok(Err(e)) => {
                            tracing::error!("Actor {} failed: {:#}", name, e);
                            errors.push(e);
                        }
                        Err(e) if e.is_cancelled() => {
                            tracing::debug!("Actor {} was cancelled", name);
                        }
                        Err(e) => {
                            tracing::error!("Actor {} panicked: {:#}", name, e);
                            errors.push(eyre::eyre!("Actor panicked: {}", e));
                        }
                    }

                    remaining_tasks -= 1;
                    if remaining_tasks == 0 {
                        break;
                    }
                }

                // All channels closed
                else => {
                    break;
                }
            }
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(eyre::eyre!(
                "Multiple actors failed: {}",
                errors
                    .iter()
                    .map(|e| e.to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            ))
        }
    }
}
