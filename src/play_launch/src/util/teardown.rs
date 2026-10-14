//! Releasing play_launch's own ROS entities without blocking the shutdown.
//!
//! Destroying a client, a subscription or a node is not local: rmw publishes
//! a graph change to every participant it knows, and under Fast DDS that is a
//! shared-memory write into each peer's port. At the end of a run those peers
//! are dying or already dead, and a write to the port of a peer killed
//! mid-way waits in `SharedMemTransport::find_port` (`sem_timedwait`) — about
//! a second each, and every destruction holds the node's lock, so they queue.
//! Measured on Autoware under `isolated`: fifteen container actors finishing
//! a second apart, shutdown from 4 s to 30 s, and twice not at all, with a
//! tokio worker parked in `rmw_destroy_client` and the runtime's drop waiting
//! on it.
//!
//! Once the run is ending nobody needs those announcements — the peers are
//! going away too — so an entity released then is leaked instead: its memory
//! is reclaimed when the process exits a moment later. Before that (a
//! container respawning) it is destroyed normally, but on the blocking pool,
//! so a dead peer delays nothing else. The cost of the leak is that our own
//! participant may leave its shared-memory files in `/dev/shm`, the same
//! state a SIGKILLed node leaves; `fastdds shm clean` removes them.

use std::sync::atomic::{AtomicBool, Ordering};

static SHUTTING_DOWN: AtomicBool = AtomicBool::new(false);

/// Mark the run as ending. Called before the process group is signalled, so
/// an actor that sees its child exit already knows why.
pub fn begin_shutdown() {
    SHUTTING_DOWN.store(true, Ordering::Release);
}

/// Whether the run is ending.
pub fn shutting_down() -> bool {
    SHUTTING_DOWN.load(Ordering::Acquire)
}

/// Let go of ROS entities: leaked once the run is ending, otherwise dropped
/// on the blocking pool (or here, outside a runtime).
pub fn release<T: Send + 'static>(entities: T) {
    if shutting_down() {
        std::mem::forget(entities);
        return;
    }
    match tokio::runtime::Handle::try_current() {
        Ok(handle) => {
            handle.spawn_blocking(move || drop(entities));
        }
        Err(_) => drop(entities),
    }
}

/// End the process once a run is over, without libc's exit handlers.
///
/// Those handlers run Fast DDS's static `DomainParticipantFactory`
/// destructor, which destroys every participant still open — ours, since its
/// node is never dropped — and each writer's destructor waits for the event
/// thread, itself sending heartbeats into dead peers' ports (`find_port`).
/// Measured on Autoware under `isolated`: two runs in six spent 24 s there
/// after `play()` had returned. Nothing those handlers do matters once the
/// run is over: the log file is written unbuffered, and the kernel releases
/// the rest. stdout and stderr are flushed first.
pub fn exit_now(code: i32) -> ! {
    use std::io::Write;
    let _ = std::io::stdout().flush();
    let _ = std::io::stderr().flush();
    // SAFETY: `_exit` only ends the process; nothing after it runs.
    unsafe { libc::_exit(code) }
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

    // One test: the flag is process-wide and only ever goes one way.
    #[test]
    fn released_entities_are_dropped_until_shutdown_and_leaked_after() {
        let before = Arc::new(());
        release(before.clone());
        assert_eq!(Arc::strong_count(&before), 1, "dropped outside a runtime");

        begin_shutdown();
        let after = Arc::new(());
        drop(Held::new(after.clone()));
        assert_eq!(Arc::strong_count(&after), 2, "leaked once shutting down");
    }
}
