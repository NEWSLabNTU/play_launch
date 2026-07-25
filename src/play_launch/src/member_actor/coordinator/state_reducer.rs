//! The SOLE writer of the shared web-state mirror (phase-51.3,
//! docs/design/executor-state-ownership.md).
//!
//! Actors own their state machines and emit `StateEvent`s; this reducer
//! folds that stream into `shared_state` (the `DashMap` the web layer
//! reads). Before phase-51 every transition was hand-written THREE times
//! (actor entry, this map, the event) and the coordinator-side event
//! consumer was a documented no-op; now a missed transition is a missing
//! event — visible in the SSE stream — rather than a silent mirror desync.
//!
//! Grep gate: `shared_state.insert` appears in THIS file only. The two
//! non-event entry points are explicit:
//! - [`set`] — builder initialization (initial Pending/Unloaded states,
//!   orphaned composables) and the handle's optimistic Loading/Unloading
//!   feedback on user-triggered load/unload.
//! - [`init`] — `entry().or_insert` so late initialization never clobbers
//!   a state the reducer already derived from a real event.

use crate::member_actor::{events::StateEvent, model::MemberState};
use dashmap::DashMap;

pub(crate) type SharedState = DashMap<String, MemberState>;

/// Fold one actor event into the mirror.
///
/// The mapping reproduces exactly what the pre-phase-51 actor-side writes
/// did; in particular `Exited` deliberately writes NOTHING — the follow-up
/// `Respawning`/`Terminated`/`Failed` event carries the resulting state
/// (same as before, when the actor only wrote the mirror on those paths).
pub(crate) fn apply(shared: &SharedState, event: &StateEvent) {
    let next: Option<MemberState> = match event {
        StateEvent::Started { pid, .. } => Some(MemberState::Running { pid: *pid }),
        StateEvent::Exited { .. } => None,
        StateEvent::Respawning { attempt, .. } => {
            Some(MemberState::Respawning { attempt: *attempt })
        }
        StateEvent::Terminated { .. } => Some(MemberState::Stopped),
        StateEvent::Failed { error, .. } => Some(MemberState::Failed {
            error: error.clone(),
        }),
        StateEvent::LoadStarted { .. } => Some(MemberState::Loading),
        StateEvent::LoadSucceeded { unique_id, .. } => Some(MemberState::Loaded {
            unique_id: *unique_id,
        }),
        StateEvent::LoadFailed { error, .. } => Some(MemberState::Failed {
            error: error.clone(),
        }),
        StateEvent::Unloaded { .. } => Some(MemberState::Unloaded),
        StateEvent::Blocked { reason, .. } => Some(MemberState::Blocked { reason: *reason }),
        StateEvent::ParameterChanged { .. } => None,
    };

    if let Some(state) = next {
        shared.insert(event.member_name().to_string(), state);
    }
}

/// Direct write for non-event states: builder registration (orphaned
/// composables) and the handle's optimistic Loading/Unloading feedback.
pub(crate) fn set(shared: &SharedState, id: &str, state: MemberState) {
    shared.insert(id.to_string(), state);
}

/// Initialize a member's state without clobbering anything the reducer
/// already wrote from a real event.
pub(crate) fn init(shared: &SharedState, id: &str, state: MemberState) {
    shared.entry(id.to_string()).or_insert(state);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::member_actor::model::BlockReason;

    #[test]
    fn started_maps_to_running() {
        let shared = SharedState::new();
        apply(
            &shared,
            &StateEvent::Started {
                name: "node:/a".into(),
                pid: 42,
            },
        );
        assert_eq!(
            *shared.get("node:/a").unwrap(),
            MemberState::Running { pid: 42 }
        );
    }

    #[test]
    fn exited_writes_nothing_until_terminated() {
        let shared = SharedState::new();
        apply(
            &shared,
            &StateEvent::Started {
                name: "node:/a".into(),
                pid: 42,
            },
        );
        apply(
            &shared,
            &StateEvent::Exited {
                name: "node:/a".into(),
                exit_code: Some(0),
            },
        );
        // Still Running — the resolution event carries the outcome.
        assert_eq!(
            *shared.get("node:/a").unwrap(),
            MemberState::Running { pid: 42 }
        );
        apply(
            &shared,
            &StateEvent::Terminated {
                name: "node:/a".into(),
            },
        );
        assert_eq!(*shared.get("node:/a").unwrap(), MemberState::Stopped);
    }

    #[test]
    fn composable_lifecycle() {
        let shared = SharedState::new();
        apply(
            &shared,
            &StateEvent::LoadStarted {
                name: "composable:/c".into(),
            },
        );
        assert_eq!(*shared.get("composable:/c").unwrap(), MemberState::Loading);
        apply(
            &shared,
            &StateEvent::LoadSucceeded {
                name: "composable:/c".into(),
                full_node_name: "/c".into(),
                unique_id: 7,
            },
        );
        assert_eq!(
            *shared.get("composable:/c").unwrap(),
            MemberState::Loaded { unique_id: 7 }
        );
        apply(
            &shared,
            &StateEvent::Blocked {
                name: "composable:/c".into(),
                reason: BlockReason::ContainerStopped,
            },
        );
        assert_eq!(
            *shared.get("composable:/c").unwrap(),
            MemberState::Blocked {
                reason: BlockReason::ContainerStopped
            }
        );
    }

    #[test]
    fn init_never_clobbers() {
        let shared = SharedState::new();
        apply(
            &shared,
            &StateEvent::Started {
                name: "node:/a".into(),
                pid: 42,
            },
        );
        init(&shared, "node:/a", MemberState::Pending);
        assert_eq!(
            *shared.get("node:/a").unwrap(),
            MemberState::Running { pid: 42 }
        );
    }
}
