---
id: 6
title: "Web UI organization — bare-name list, shallow tree, no status facets, three freshness mechanisms"
status: resolved
type: enhancement
severity: medium
github: https://github.com/NEWSLabNTU/play_launch/issues/6
---

Ranked gaps (audit §C): bare-name-first identity (namespace only via
click-filter); container-only shallow tree — null-`container_name`
composables fall into the flat bucket (`NodeList.js:98-105`); no
status-grouped sections or state/package/exec filters
(`NodeList.js:45-52` name-only); SSE for state but full-resync for
existence (`store.js:174`) plus a 3s activity poll and 500ms graph
debounce; `get_node` duplicates `list_nodes` container resolution
(`handlers.rs:116-146` vs `55-93`); 10 hand-linked CSS files.

Design: `docs/design/web-ui-organization.md`; plan: phase-50.

## Resolution (phases 50 + 53, 2026-07-25)

Phase-50: canonical-id keys, namespace grouping, status facets,
clickable health strip, orphan visibility, unknown-SSE-id refetch.
Phase-51.2: one build_summary. Phase-52.3: failure banner.
Phase-53: true nested namespace tree (per-segment collapse), kind facet
chips, shared container-resolution helper for list_nodes/get_node, dead
store code removed.

Deferred by design: member_added/removed SSE (membership is static
until dynamic members exist; targeted refetch covers the gap) and CSS
bundling (tooling for no user-visible win).
