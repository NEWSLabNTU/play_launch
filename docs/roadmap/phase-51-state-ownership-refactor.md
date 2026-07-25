# Phase 51: State ownership refactor — reducer + container split

**Status:** ✅ Done (2026-07-25). After phase-50 (identity keys reused).
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

## Outcome (2026-07-25)

All work items landed; notes:

- **model/**: `node_state.rs` (NodeState/ContainerState/ActorConfig),
  `composable_state.rs` (ComposableState + THE BlockReason),
  `member_view.rs` (MemberState/MemberSummary/HealthSummary +
  `From<&NodeState>`/`From<&ComposableState>`). The `state.rs`/`web_query.rs`
  modules are deleted outright (no shims). SSE wire note: `Blocked.reason`
  is now snake_case (`container_not_started`) like REST — the UI never
  branched on the reason value.
- **summary.rs**: one `build_summary()` + `stderr_info()`; handle.rs's
  three copies collapsed.
- **state_reducer.rs**: sole `shared_state` writer, folding the StateEvent
  stream in `MemberRunner::next_state_event` (and the completion drain).
  Actors lost their `shared_state` field and all ~23 direct inserts.
  Non-event writes are explicit: `set()` (builder orphan registration,
  handle's optimistic Loading/Unloading) and `init()` (`or_insert`
  initialization that never clobbers a reduced state). `Exited`
  deliberately reduces to nothing — the follow-up Respawning/Terminated/
  Failed event carries the outcome, matching pre-51 actor writes.
  `let _ = state_tx.send` is gone: `events::emit` logs a dropped
  transition at error level.
- **Container split**: `actor.rs` (process + run loop), `supervisor.rs`
  (composable map/queue/completions), `ros_client.rs` (clients +
  ComponentEvent sub + ALL `_container` name construction + the service
  call impls), `component_events.rs` (supervisor methods), `timing.rs`
  (all 11 Durations; knobs come in phase-52). `composable_nodes.rs` and
  `service_calls.rs` are gone. `container_readiness.rs`'s discovery-side
  `ends_with` checks remain (they match remote graph names, they don't
  construct ours).
- **Sweep**: `ControlEvent::Kill` deleted (unreachable from any caller);
  runner's `shared_state` is now the reducer's target (was `_shared_state`
  ignored); stale record.json comment in web/mod.rs fixed. The
  `#[allow(dead_code)]` on test-only state predicates stays.

Gates: zero `shared_state.insert` outside state_reducer.rs; zero
`format!` of `_container` names outside ros_client.rs; exactly one
`pub enum BlockReason`; zero silent `let _ = state_tx.send`. 286 tests
green. Collision launch: 8/8 members, stop/start container cycles
blocked(container_stopped) → running/loaded via the reducer only.
Cold Autoware receipt (`just demo-all`): PASS.
