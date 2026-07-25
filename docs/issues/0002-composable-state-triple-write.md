---
id: 2
title: "Composable state triple-write — actor entry + shared_state DashMap + ignored StateEvent; duplicated BlockReason"
status: open
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
