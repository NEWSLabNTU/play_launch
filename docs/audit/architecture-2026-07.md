# Architecture & Web UI Audit — 2026-07-25

Refresh of the March 2026 audit, focused on executor structure, antipatterns,
and the web UI. Trigger: "the node page does not show all nodes."

## Headline bug — nodes missing from the node page

The node list is driven by `metadata: HashMap<String, MemberMetadata>`
(`coordinator/handle.rs:58-125`), keyed by a **bare, non-namespaced member
name** with silent-overwrite inserts. Four drop mechanisms:

1. **Name-key collisions (PRIMARY).** Only containers get `_N` dedup
   (`builder.rs:233-244`); regular nodes (`builder.rs:217`) and composables
   (`builder.rs:416`) plain-insert. Collisions that eat nodes:
   same node name across namespaces (`/robot1/camera` + `/robot2/camera` →
   one `camera`); `name=null` nodes falling back to a shared `exec_name`
   (`replay.rs:693-699`, common in Autoware); the literal `"unknown"` bucket
   (`replay.rs:699`); same composable node_name in two containers
   (`replay.rs:787`); cross-TYPE collisions (regular vs container vs
   composable share one keyspace — insertion order picks the winner).
   The same key threads `shared_state`, `control_channels`, `tasks` → health
   counts shrink and start/stop routes to the shadowing member.
2. **Orphaned composables dropped.** A composable whose
   `target_container_name` doesn't match any container is skipped with only
   a `warn!` and never registered (`builder.rs:325-442`, no-insert branches
   at 425-441) — absent from the UI entirely.
3. **Frontend re-collapses by name.** `store.js:151-157` rebuilds a
   `Map` keyed by `node.name`; Preact keys `key=${node.name}`
   (`NodeList.js:200`) — even a fixed backend would collapse client-side.
4. **Post-load members invisible until resync.** Incremental SSE drops
   events for unknown names (`store.js:174` "wait for full resync");
   existence only arrives via full `fetchNodes()` on (re)connect.

**Fix direction:** collision-proof identity (namespace-qualified FQN, plus
`_N` only for true duplicates) threaded through `metadata_map`,
`shared_state`, `control_channels`, `tasks`, and the frontend `nodes` Map +
Preact keys; register orphaned composables as `Blocked{ContainerNotFound}`
instead of dropping; add `member_added/removed` SSE events.

## Executor antipatterns (ranked)

### S1 — Triple-write state duplication (composables)
Every transition hand-writes 3 stores: `ComposableNodeEntry.state`
(actor truth), `shared_state: DashMap<String, MemberState>` (web mirror),
and a `StateEvent` on `state_tx` (e.g. `composable_nodes.rs:130-160`, and
again at 170-225, 270-290, 372-378, 577-595, plus `mod.rs` 280-520 and
`component_events.rs`). `ComposableState` and `MemberState` are different
enums with no conversion fn — each site re-encodes by hand. Meanwhile
`runner.rs:67-70` explicitly drains and IGNORES the StateEvents ("Actors
update shared_state directly"). Three stores, no single owner.

### S1 — `ContainerActor` god struct
`container_actor/mod.rs:126`: 20+ fields spanning process lifecycle, three
ROS service clients (creation inlined in a ~250-line `handle_pending`),
composable supervision, ComponentEvent bridging, and web state. The 5-file
split all `impl` the same struct — cosmetic.

### S2
- **Duplicated `BlockReason`** with mismatched variants (`state.rs:113` vs
  `web_query.rs:55`), hand-mapped at each insert.
- **3× copy-pasted `MemberSummary` construction** in `coordinator/handle.rs`
  (list_members 58-125, get_member_state 128-190, get_health_summary
  193-270).
- **String identity everywhere**: bare-name keys (see headline bug);
  `format!`-built service names scattered (`mod.rs:311,339,356,385`,
  `parameter_proxy.rs:31-40`) + an `ends_with("/_container/list_nodes")`
  check (`container_readiness.rs:185-189`); ComponentEvent name-fallback
  matching depends on FQN string equality surviving the ROS round-trip.
- **12 magic `Duration` consts** in `container_actor/mod.rs:39-70` (plus
  more in run.rs, signal_handler.rs, sse.rs, rt_helper_client.rs) — none
  configurable; Autoware-scale users can't touch `LOAD_TOTAL_BUDGET`.

### S3
- **Sleep-based synchronization**: `POST_SERVICE_READY_WARMUP` 200ms guess
  (`service_calls.rs:147-157`, self-labeled "CRITICAL FIX"), hardcoded SSE
  reconnect sleeps (`sse.rs:124,135,429,439`).
- **46 swallowed `let _ = send(...)`** — dropped StateEvents lose web
  transitions silently.
- **Unbounded channels** (6 sites): sharpest is the ComponentEvent bridge
  (`container_actor/mod.rs:391`) — DDS-rate producer, no backpressure.
- **"Startup complete with failures" warn-and-continue**
  (`signal_handler.rs:494`): partially failed launch looks started; counts
  computed but only logged — no exit-code/UI surfacing decision.
- **Stringly `eyre` throughout** (56 files) — callers can't distinguish
  busy/missing/rejected; only sched + ipc have typed errors.

### S4
- record.json-era comment remnants (10 sites incl. the removed
  `--input-file` doc block, `cli/options.rs:441-444`).
- `runner.rs` carries an unused `_shared_state`.
- `#[allow(dead_code)]` corpses: `ControlEvent::Kill`,
  `ContainerControlEvent`, state predicate methods.
- `cli/options.rs`: 903 LOC, 59 flags, flat — no `#[command(flatten)]`
  grouping; the natural home for the timeout knobs.

## Web UI organization gaps (ranked)

1. Bare-name-first identity; namespace only via click-to-filter — should be
   an FQN-first collapsible namespace tree (also the bug surface).
2. Tree view container-only + shallow; null-`container_name` composables
   silently fall to the flat bucket (`NodeList.js:98-105`).
3. No status-grouped sections/filters — failures hunted by eyeballing.
4. Filter is name-only (`NodeList.js:45-52`); no package/exec/state facets.
5. Three freshness mechanisms (SSE state, full-resync existence, 3s poll
   for activity sort + 500ms graph debounce) — unify on SSE with
   member_added/removed.
6. `get_node` duplicates `list_nodes`' container-resolution logic
   (`handlers.rs:116-146` vs `55-93`).
7. 10 hand-linked CSS files, no bundling; dead store code
   (`store.js:113-119` no-op loop).

## Proposed target structure

One owner per fact; web mirror derived, never hand-written:

```
member_actor/
├─ model/            node_state.rs · composable_state.rs (single BlockReason)
│                    member_view.rs (MemberState/Summary + From<…State>)
├─ container/        actor.rs (lifecycle only) · supervisor.rs (composables)
│                    ros_client.rs (clients + ALL service-name format!s)
│                    component_events.rs · timing.rs (all Durations ← Config)
├─ regular_node/actor.rs
├─ coordinator/      builder.rs · handle.rs (thin) · summary.rs (one builder)
│                    state_reducer.rs  ← SOLE writer of shared_state,
│                                         fed by the existing StateEvent
│                                         channel (today ignored)
└─ events.rs
```

**Migration order:** (1) FQN identity + central service-name helper;
(2) unify BlockReason + `From` impls; (3) extract `build_summary`;
(4) state-reducer inversion (actors stop writing shared_state);
(5) split ContainerActor; (6) group cli options + expose timing knobs.
Frontend: FQN keys → namespace tree → status facets → membership SSE.

## Quick wins
FQN keying + orphan-composable registration (the bug), BlockReason unify,
build_summary extraction, service-name helper, timing.rs consolidation,
record.json comment purge.
