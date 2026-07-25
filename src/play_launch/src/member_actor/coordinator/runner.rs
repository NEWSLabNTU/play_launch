//! Runner that owns the StateEvent stream and waits for all actors.
//!
//! Phase-51.3 inversion: actors no longer write the shared web-state map —
//! the runner folds every StateEvent through `state_reducer::apply` (the
//! sole writer) as it passes the event on. Before this, the drain here was
//! a documented no-op and every actor triple-wrote its transitions.

use super::state_reducer;
use crate::member_actor::{events::StateEvent, model::MemberState};
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
    /// Shared state map — written ONLY via `state_reducer` from the event
    /// stream owned here
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

    /// Get the next state event (for web UI forwarding). Applies the state
    /// reducer before handing the event out, so the web mirror is updated
    /// no matter what the caller does with the event.
    pub async fn next_state_event(&mut self) -> Option<StateEvent> {
        let event = self.state_rx.recv().await?;
        state_reducer::apply(&self.shared_state, &event);
        Some(event)
    }

    /// Wait for all actors to complete (takes mut self!)
    pub async fn wait_for_completion(self) -> Result<()> {
        use futures::stream::{FuturesUnordered, StreamExt};

        // Extract fields from self
        let Self {
            tasks,
            mut state_rx,
            shared_state,
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
                // Reduce state events into the shared web mirror
                Some(event) = state_rx.recv() => {
                    tracing::debug!("State event: {:?}", event);
                    state_reducer::apply(&shared_state, &event);
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
