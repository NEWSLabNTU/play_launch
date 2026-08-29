//! Runner that owns the StateEvent stream and waits for all actors.
//!
//! Phase-51.3 inversion: actors no longer write the shared web-state map —
//! the runner folds every StateEvent through `state_reducer::apply` (the
//! sole writer) as it passes the event on. Before this, the drain here was
//! a documented no-op and every actor triple-wrote its transitions.

use super::state_reducer;
use crate::member_actor::{events::StateEvent, model::MemberState};
use eyre::Result;
use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};
use tokio::{
    sync::{mpsc, watch},
    task::JoinHandle,
};

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
    /// Members launched with `on_exit=Shutdown()`, by canonical member id
    shutdown_on_exit: HashSet<String>,
    /// The same shutdown lever `MemberHandle::shutdown` pulls. Stops the actors, but
    /// on its own it does not reap their children — see `shutdown_hook`.
    shutdown_tx: Arc<watch::Sender<bool>>,
    /// Full teardown, installed by the command layer. Stopping the actors is not
    /// enough: the processes are killed by signalling the process GROUP, and the
    /// replay-level watch is what stops the background tasks. Only the command layer
    /// holds those, so it supplies the whole sequence here.
    shutdown_hook: Option<Arc<dyn Fn() + Send + Sync>>,
}

impl MemberRunner {
    /// Create a new MemberRunner (called from builder)
    pub(super) fn new(
        tasks: HashMap<String, JoinHandle<Result<()>>>,
        state_rx: mpsc::Receiver<StateEvent>,
        shared_state: Arc<dashmap::DashMap<String, MemberState>>,
        shutdown_on_exit: HashSet<String>,
        shutdown_tx: Arc<watch::Sender<bool>>,
    ) -> Self {
        Self {
            tasks,
            state_rx,
            shared_state,
            shutdown_on_exit,
            shutdown_tx,
            shutdown_hook: None,
        }
    }

    /// Install the full teardown used when a required node exits. Without it, only the
    /// actors are stopped and the launched processes keep running.
    pub fn set_shutdown_hook(&mut self, hook: Arc<dyn Fn() + Send + Sync>) {
        self.shutdown_hook = Some(hook);
    }

    /// Honour `on_exit=Shutdown()`.
    ///
    /// In the launch system a node declared this way is *required*: when it exits, the
    /// whole launch is torn down. play_launch used to drop these handlers at dump time
    /// and warn about it, which is fine for an optional node and wrong for an
    /// orchestrator. SSv2's `scenario_test_runner` is one — it runs the scenario, exits
    /// 0, and expects to take the interpreter, preprocessor and visualization with it.
    /// Without this, those three ran forever: one interpreter was found still spinning
    /// 40 hours after its scenario had passed, holding a ROS domain and ~90 nodes.
    ///
    /// Pulling `shutdown_tx` stops every actor, which completes the runner task, which
    /// is what the command layer already treats as "the launch is over".
    fn honour_on_exit_shutdown(
        shutdown_on_exit: &HashSet<String>,
        shutdown_tx: &watch::Sender<bool>,
        shutdown_hook: Option<&Arc<dyn Fn() + Send + Sync>>,
        event: &StateEvent,
    ) {
        if let StateEvent::Exited { name, exit_code } = event
            && shutdown_on_exit.contains(name)
            && !*shutdown_tx.borrow()
        {
            tracing::info!(
                "[{}] exited ({}) and was declared on_exit=Shutdown: shutting down the launch",
                name,
                match exit_code {
                    Some(code) => format!("code {code}"),
                    None => "no exit code".to_string(),
                }
            );
            let _ = shutdown_tx.send(true);
            if let Some(hook) = shutdown_hook {
                hook();
            }
        }
    }

    /// Get the next state event (for web UI forwarding). Applies the state
    /// reducer before handing the event out, so the web mirror is updated
    /// no matter what the caller does with the event.
    pub async fn next_state_event(&mut self) -> Option<StateEvent> {
        let event = self.state_rx.recv().await?;
        state_reducer::apply(&self.shared_state, &event);
        Self::honour_on_exit_shutdown(
            &self.shutdown_on_exit,
            &self.shutdown_tx,
            self.shutdown_hook.as_ref(),
            &event,
        );
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
            shutdown_on_exit,
            shutdown_tx,
            shutdown_hook,
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
                    Self::honour_on_exit_shutdown(
                        &shutdown_on_exit,
                        &shutdown_tx,
                        shutdown_hook.as_ref(),
                        &event,
                    );
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

#[cfg(test)]
mod tests {
    use super::*;

    fn exited(name: &str) -> StateEvent {
        StateEvent::Exited {
            name: name.to_string(),
            exit_code: Some(0),
        }
    }

    /// The names are canonical member ids (`node:/name`), NOT display names — matching
    /// on the display name is exactly the bug that made this silently never fire.
    #[test]
    fn a_required_member_exiting_pulls_the_lever_and_runs_the_hook() {
        let (tx, _rx) = watch::channel(false);
        let tx = Arc::new(tx);
        let ran = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let ran_in_hook = ran.clone();
        let hook: Arc<dyn Fn() + Send + Sync> = Arc::new(move || {
            ran_in_hook.store(true, std::sync::atomic::Ordering::SeqCst);
        });
        let required: HashSet<String> = ["node:/scenario_test_runner".to_string()].into();

        MemberRunner::honour_on_exit_shutdown(
            &required,
            &tx,
            Some(&hook),
            &exited("node:/scenario_test_runner"),
        );

        assert!(*tx.borrow(), "shutdown lever should be pulled");
        assert!(
            ran.load(std::sync::atomic::Ordering::SeqCst),
            "the teardown hook is what actually reaps the processes"
        );
    }

    #[test]
    fn an_ordinary_member_exiting_leaves_the_launch_alone() {
        let (tx, _rx) = watch::channel(false);
        let tx = Arc::new(tx);
        let required: HashSet<String> = ["node:/scenario_test_runner".to_string()].into();

        MemberRunner::honour_on_exit_shutdown(&required, &tx, None, &exited("node:/talker"));

        assert!(
            !*tx.borrow(),
            "an unrelated node exiting must not end the launch"
        );
    }

    /// Every node in a launch exits during a normal shutdown; the hook must not be run
    /// again for each one.
    #[test]
    fn the_hook_does_not_run_again_once_shutdown_started() {
        let (tx, _rx) = watch::channel(true); // already shutting down
        let tx = Arc::new(tx);
        let calls = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let calls_in_hook = calls.clone();
        let hook: Arc<dyn Fn() + Send + Sync> = Arc::new(move || {
            calls_in_hook.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        });
        let required: HashSet<String> = ["node:/scenario_test_runner".to_string()].into();

        MemberRunner::honour_on_exit_shutdown(
            &required,
            &tx,
            Some(&hook),
            &exited("node:/scenario_test_runner"),
        );

        assert_eq!(calls.load(std::sync::atomic::Ordering::SeqCst), 0);
    }
}
