# Phase 53: Web UI organization — remainder of issue 0006

**Status:** ✅ Done (2026-07-25).
**Fixes:** issue 0006 (GH #6) — the parts phase-50 didn't cover.
**Design:** docs/design/web-ui-organization.md.

## Work items

- **53.1 — backend dedup.** One shared container-resolution helper for
  `list_nodes` + `get_node` (handlers.rs duplicates it).
- **53.2 — frontend hygiene.** Delete dead store code; keep the
  hand-linked CSS files (bundling adds tooling for no user-visible win —
  dropped from scope, noted in the issue).
- **53.3 — kind facets.** Member-kind chips (nodes / containers /
  composables) alongside the phase-50 status facets.
- **53.4 — true nested namespace tree.** Multi-segment namespace
  hierarchy with per-level collapse (namespaces → containers → their
  composables → nodes), replacing the flat one-header-per-namespace
  grouping from phase-50.6. Flat mode stays as the secondary view.

## Deferred (with reason)

- `member_added`/`member_removed` SSE + retiring the 3s activity poll:
  membership is static after the builder today — the events would never
  fire. Belongs with dynamic-membership work; the unknown-id targeted
  refetch (50.5) covers the gap. The 3s poll refreshes stderr activity
  timestamps, which is not a membership concern.

## Acceptance

- Autoware node page: nested namespace tree (e.g. `/planning` →
  `/planning/scenario_planning` → containers → composables), collapse at
  any level; kind + status facets compose; behavior identical otherwise.
- `get_node` and `list_nodes` share one resolution helper (grep gate).
- Cold Autoware receipt PASS.

## Outcome (2026-07-25)

All items landed. Validation: grouping replicated against the live
Autoware /api/nodes (114 members) — nested tree resolves 22 namespace
headers up to 4 levels deep (`/planning/scenario_planning/lane_driving/
behavior_planning`), card counts sum to 114/114; cold receipt PASS
(68/68, engage first try, MRM 4.25 → 0.00 m/s).
