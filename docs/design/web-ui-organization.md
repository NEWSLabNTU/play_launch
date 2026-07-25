# Design: Web UI Organization

**Status:** Draft (2026-07-25). Closes issue 0006 (GH #6); depends on
member-identity.md. Implementation: roadmap phase-50 (data-visible parts)
+ follow-up UI work.

## Principles

FQN-first identity; problems surface without hunting; ONE freshness
mechanism.

## Node view

- **Collapsible namespace tree** as the primary mode: namespaces →
  containers (nesting their composables) → nodes. Flat list stays as a
  secondary mode. Keys = canonical MemberId strings.
- **Status summary strip** above the list (from HealthSummary), clickable:
  "3 failed" cross-filters the tree to failures.
- **Facet filters**: state, member kind, package, executable, container —
  extending the current name-only match.
- Orphaned composables render with a "container not found" badge
  (member-identity.md registers them).

## Freshness

Single mechanism: SSE. `/api/state/updates` gains `member_added` /
`member_removed` events; the store inserts/removes instead of dropping
unknown-name events and waiting for a full resync. Removes the 3s
activity poll and reduces reconnect resyncs to a consistency check.

## Backend cleanups

- One shared container-resolution helper for `list_nodes` + `get_node`
  (today duplicated, handlers.rs:55-93 vs 116-146).
- One `build_summary()` used by list/get/health (today 3 copies in
  coordinator/handle.rs).
