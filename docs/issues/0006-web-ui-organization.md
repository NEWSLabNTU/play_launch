---
id: 6
title: "Web UI organization — bare-name list, shallow tree, no status facets, three freshness mechanisms"
status: open
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
