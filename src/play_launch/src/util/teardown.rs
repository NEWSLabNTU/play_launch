//! Leaving the ROS graph before the nodes stop.
//!
//! Destroying a client, a subscription or a node is not local: rmw announces
//! it to every participant it knows. At the end of a run those participants
//! are stopping too, and announcing to one that is already gone is slow —
//! measured on Autoware under `isolated` with Fast DDS, about a second per
//! destruction, with every destruction holding the node's lock, so they
//! queued: shutdown took 4 s to 30 s and twice never finished.
//!
//! So play_launch leaves first. When the run starts to end
//! ([`begin_shutdown`]), every holder of a ROS entity or of the node lets go
//! of it, the executor threads stop and drop theirs, and the context is
//! finalized while every peer is still alive. Only then are the nodes
//! signalled ([`left_ros_graph`]). Each step is ordinary destruction, so
//! whatever the middleware keeps (a DDS participant's shared-memory files,
//! say) is released the way it releases it — play_launch makes no
//! assumption about what that is. That holds after a third Ctrl-C too,
//! since by then the context is long gone; only a SIGKILL of play_launch
//! itself skips it.

use std::{
    sync::{
        Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

static SHUTTING_DOWN: AtomicBool = AtomicBool::new(false);

static CONTEXTS: Mutex<Vec<rclrs::ContextObserver>> = Mutex::new(Vec::new());

/// Mark the run as ending.
pub fn begin_shutdown() {
    SHUTTING_DOWN.store(true, Ordering::Release);
}

/// Whether the run is ending.
pub fn shutting_down() -> bool {
    SHUTTING_DOWN.load(Ordering::Acquire)
}

/// Register a context [`left_ros_graph`] waits for.
pub fn watch_context(context: &rclrs::Context) {
    CONTEXTS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .push(context.observer());
}

fn contexts_finalized() -> bool {
    CONTEXTS
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .iter()
        .all(|c| c.is_finalized())
}

/// Wait until every watched context is finalized, at most `timeout`.
/// Returns whether it was.
pub async fn left_ros_graph(timeout: Duration) -> bool {
    let start = Instant::now();
    loop {
        if contexts_finalized() {
            tracing::debug!(
                "left the ROS graph {} ms after shutdown began",
                start.elapsed().as_millis()
            );
            return true;
        }
        if start.elapsed() >= timeout {
            return false;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

/// Let go of ROS entities without blocking the caller: dropped on the
/// blocking pool (or here, outside a runtime), since a destruction talks to
/// every peer.
pub fn release<T: Send + 'static>(entities: T) {
    match tokio::runtime::Handle::try_current() {
        Ok(handle) => {
            handle.spawn_blocking(move || drop(entities));
        }
        Err(_) => drop(entities),
    }
}

/// Keep ROS entities until the run starts to end, then [`release`] them.
pub fn hold_until_shutdown<T: Send + 'static>(
    entities: T,
    mut shutdown: tokio::sync::watch::Receiver<bool>,
) {
    tokio::spawn(async move {
        let _ = shutdown.wait_for(|ending| *ending).await;
        release(entities);
    });
}

/// Abort a task that holds ROS entities when the run starts to end.
pub fn abort_at_shutdown(
    task: tokio::task::AbortHandle,
    mut shutdown: tokio::sync::watch::Receiver<bool>,
) {
    tokio::spawn(async move {
        tokio::select! {
            _ = shutdown.wait_for(|ending| *ending) => task.abort(),
            _ = async {
                while !task.is_finished() {
                    tokio::time::sleep(Duration::from_secs(1)).await;
                }
            } => {}
        }
    });
}

/// Holds ROS entities and hands them to [`release`] when dropped, for an
/// entity that lives as long as a task or a scope.
pub struct Held<T: Send + 'static>(Option<T>);

impl<T: Send + 'static> Held<T> {
    pub fn new(entities: T) -> Self {
        Self(Some(entities))
    }

    pub fn get(&self) -> &T {
        self.0.as_ref().expect("held until dropped")
    }
}

impl<T: Send + 'static> Drop for Held<T> {
    fn drop(&mut self) {
        if let Some(entities) = self.0.take() {
            release(entities);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[test]
    fn released_entities_are_dropped() {
        let entity = Arc::new(());
        drop(Held::new(entity.clone()));
        assert_eq!(Arc::strong_count(&entity), 1, "dropped outside a runtime");
    }

    #[tokio::test]
    async fn released_entities_are_dropped_on_the_blocking_pool() {
        let entity = Arc::new(());
        release(entity.clone());
        let start = Instant::now();
        while Arc::strong_count(&entity) > 1 && start.elapsed() < Duration::from_secs(5) {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert_eq!(Arc::strong_count(&entity), 1);
    }
}
