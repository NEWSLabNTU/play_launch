# Phase 50: FQN member identity + full web visibility

**Status:** ✅ Done (2026-07-25).
**Fixes:** issues 0001 (GH #1), 0006-partial (GH #6).
**Design:** docs/design/member-identity.md, docs/design/web-ui-organization.md.

Nodes vanish from the web UI because every registry keys on bare names with
silent overwrite, and orphaned composables are dropped. This phase makes
every launched member visible, exactly once.

## Work items

- **50.1 — MemberId type + constructor.** `MemberKind` + fqn join rules +
  ordinal; canonical string form; unit tests for the collision cases from
  issue 0001 (cross-namespace, exec_name fallback, "unknown", cross-type,
  duplicate composable node_names).
- **50.2 — Coordinator/builder conversion.** `metadata_map`, `shared_state`,
  `control_channels`, `tasks` re-keyed on MemberId; the container-only `_N`
  counter replaced by kind-agnostic ordinal assignment; startup log prints
  registered==spawned counts (mismatch = hard warn).
- **50.3 — Orphaned composables registered** as
  `Blocked{ContainerNotFound}` instead of warn-and-drop
  (builder.rs:425-441); late-container recovery hooks in supervisor.
- **50.4 — REST + frontend keys.** `/api/nodes/:id` on canonical id;
  store Map + Preact keys on id; display name a field. Same-name nodes
  render disambiguated (`name — /ns`).
- **50.5 — Membership SSE.** `member_added`/`member_removed` events;
  store insert/remove replaces drop-and-resync (store.js:174); retire the
  3s activity poll.
- **50.6 — Namespace tree + status strip (UI).** Collapsible namespace →
  container → composable tree; clickable health strip; state/kind facets.

## Acceptance

- Synthetic launch with: 2× same node name in different namespaces, 2×
  name=null same executable, 1 orphaned composable, 1 name shared between
  a container and a regular node → node page shows ALL of them, health
  totals match spawn counts, start/stop routes to the right member.
- Cold Autoware receipt (simple-autoware-safety-island `just demo-all`)
  still PASS; node count on the page == "Spawning N" log line.

## Outcome (2026-07-25)

All work items landed; deviations from plan:

- **Fifth drop mechanism found:** the SystemModel itself
  (`structure.nodes: BTreeMap<FQN, NodeInstance>`, sole spawn source since
  47.B3) silently overwrote true duplicates in `model_builder`. Fixed with
  ordinal `#N` model keys — both instances spawn; the ROS `__node` remap
  derives from `inst.node_name`, so the suffix stays out of process args.
- **50.5 delivered as targeted refetch,** not `member_added/removed` SSE:
  membership is static after the builder today (no event source exists);
  `applyStateEvent` on an unknown id now triggers a debounced
  `fetchNodes()` instead of silently dropping. Real membership events
  belong with the phase-51 state-reducer, which will own the channel.
  The 3s activity poll stays — it refreshes stderr mtimes, not membership.
- **Launch-tree scope map residue:** `node_scope_map` still keys by display
  name (built pre-builder from the dump); the frontend falls back to a
  name-scan (`getNodeByKey`). Duplicates resolve to the first match there —
  same as pre-phase-50. Rekey belongs to phase-51's builder-ordered rework.

Acceptance: synthetic collision launch → 8/8 members on the node page
(`node:/robot1/camera`, `node:/robot2/camera`, `node:/listener`,
`node:/listener#2`, `node:/pipeline`, `container:/pipeline`,
`composable:/comp_talker` loaded, `composable:/orphan_comp` blocked);
`Registered 8` == `Spawning 8`; stop of `node:/robot2/camera` left its
cross-ns twin and both listeners running. Cold Autoware receipt
(`just demo-all`, simple-autoware-safety-island): PASS.
