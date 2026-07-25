---
id: 2
title: "Composable state triple-write — actor entry + shared_state DashMap + ignored StateEvent; duplicated BlockReason"
status: resolved
type: tech-debt
severity: high
github: https://github.com/NEWSLabNTU/play_launch/issues/2
---

Every composable transition hand-writes three stores
(`container_actor/composable_nodes.rs:130-160` and 4 sibling sites, plus
`mod.rs:280-520`, `component_events.rs`): the actor's `ComposableState`,
the web `MemberState` DashMap, and a `StateEvent` — which
`coordinator/runner.rs:67-70` drains and explicitly ignores.
`ComposableState` and `MemberState` are different enums with no conversion
function; `BlockReason` exists twice with mismatched variants
(`state.rs:113` vs `web_query.rs:55`).

Fix: single-owner state + reducer — design
`docs/design/executor-state-ownership.md`, plan
`docs/roadmap/phase-51-state-ownership-refactor.md`.

## Resolution (phase-51, 2026-07-25)

`coordinator/state_reducer.rs` is now the sole `shared_state` writer,
consuming the previously-ignored StateEvent stream via
`From<&NodeState>`/`From<&ComposableState>` conversions in
`model/member_view.rs`. Actors emit events only (no mirror writes, no
`shared_state` field); `events::emit` logs any dropped event at error
level. One `BlockReason` (`model/composable_state.rs`) — the duplicated
actor/web pair and its hand-mapping are deleted.
