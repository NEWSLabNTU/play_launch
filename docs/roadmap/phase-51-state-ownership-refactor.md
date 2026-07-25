# Phase 51: State ownership refactor — reducer + container split

**Status:** 📋 Planned (2026-07-25). After phase-50 (identity keys reused).
**Fixes:** issues 0002 (GH #2), 0003 (GH #3).
**Design:** docs/design/executor-state-ownership.md.

## Work items

- **51.1 — model/ module.** Move NodeState/ContainerState/ComposableState;
  single BlockReason; `MemberState: From<&ComposableState> + From<&NodeState>`
  impls. Delete web_query's BlockReason. (Additive, no behavior change.)
- **51.2 — summary.rs.** One `build_summary()`; collapse the three copies
  in coordinator/handle.rs.
- **51.3 — state_reducer inversion.** Reducer becomes the sole
  shared_state writer, consuming the existing StateEvent channel; actors
  drop their shared_state field and direct inserts; `let _ = state_tx.send`
  becomes logged-on-error. SSE unchanged (same stream).
- **51.4 — ContainerActor split.** actor.rs (lifecycle) / supervisor.rs
  (composables) / ros_client.rs (clients + `container_service(fqn, kind)`
  replacing the four inline format!s and the `ends_with` check) /
  timing.rs (all Durations, still consts here — knobs come in phase-52).
- **51.5 — dead-code sweep.** runner.rs unused shared_state, record.json
  comment remnants (10 sites), `#[allow(dead_code)]` corpses
  (ControlEvent::Kill, ContainerControlEvent) — implement or delete.

## Acceptance

Behavior-identical: cold Autoware receipt PASS + 62/62 at every step;
`cargo test`; grep gates — zero `shared_state.insert` outside
state_reducer, zero `format!(".../_container/` outside ros_client, one
BlockReason definition.
