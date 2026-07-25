# Design: Executor State Ownership — one owner per fact

**Status:** Implemented (2026-07-25, phase-51). Closes issues 0002/0003 (GH #2/#3).

## Problem

Composable/node state is triple-written by hand (actor entry, web
DashMap, StateEvent) with two divergent enum families and an event channel
whose coordinator-side consumer is a documented no-op. ContainerActor is a
god struct across five responsibilities.

## Decision — state

**Actor state is the sole truth; the web mirror is derived.**

```
member_actor/
├─ model/
│  ├─ node_state.rs        NodeState, ContainerState
│  ├─ composable_state.rs  ComposableState + the ONE BlockReason
│  └─ member_view.rs       MemberState/MemberSummary
│                          + impl From<&ComposableState> / From<&NodeState>
└─ coordinator/
   └─ state_reducer.rs     the ONLY writer of shared_state; consumes the
                           existing StateEvent stream (today ignored)
```

Actors stop writing `shared_state`; they emit `StateEvent` only. The
reducer converts via the `From` impls — hand-encoding sites disappear, and
a missed transition is a missing event (visible) rather than a silent
mirror desync. SSE consumes the same stream (unchanged wire shape).

Send failures on `state_tx` stop being `let _ =`: the actor logs at error
level with the member id (a dropped transition is a real defect).

## Decision — container split

```
container/
├─ actor.rs        process lifecycle only (spawn/exit/respawn/stop)
├─ supervisor.rs   composable map, load/unload queue, completions
├─ ros_client.rs   Load/List/Unload clients; owns ALL service-name
│                  construction (one `container_service(fqn, kind)` fn)
├─ component_events.rs  (role unchanged)
└─ timing.rs       every Duration const, populated from Config
```

`actor.rs` owns the child process and container state machine;
`supervisor.rs` owns composables and talks to `ros_client`. The structs
compose (supervisor holds a ros_client), ending the everything-impls-one-
struct pattern.

## Method / verification

Each migration step keeps behavior identical and lands with the cold
Autoware receipt (planning_simulator, 62/62 composables) as the acceptance
gate, plus `cargo test`. Order in phase-51 — From-impls first (additive),
reducer inversion second, struct split last.
