# Phase 20: Web UI Modernization

**Status**: Planned
**Priority**: Medium (DX improvement, eliminates polling overhead, enables future features)
**Dependencies**: None (standalone frontend refactor, no backend API changes required)

## Overview

Replace htmx + vanilla JS with a lightweight reactive framework (Preact + htm) and
SSE-driven state management. The goal is zero-polling real-time updates, a single
client-side source of truth, and declarative component-based rendering.

### Motivation

The current web UI has grown organically across Phases 8–9 and accumulated several
architectural pain points:

1. **Two rendering paths**: Node cards are server-rendered in Rust (`render_node_card()`,
   ~400 lines of `format!()` HTML). Everything else (panel, diagnostics, tooltips,
   badges) is client-rendered in JS. Adding a new field requires changes in both.

2. **SSE infrastructure unused**: A `StateEventBroadcaster` pushes real-time events
   (`Started`, `Exited`, `LoadSucceeded`, etc.) to `/api/state/updates`, but no
   frontend code subscribes to it. Instead, 6 independent polling loops drive updates.

3. **No client-side data model**: Node state lives only in the DOM. Checking "is the
   panel open?" requires `classList.contains('open')`. Matching composable nodes to
   containers is re-derived from DOM attributes.

4. **htmx is a poor fit**: htmx excels at document-oriented CRUD. This app is a
   real-time dashboard with complex interactive state (panel, tabs, scroll position),
   streaming logs, hierarchical data, and multiple synchronized views.

### Current Polling Inventory (to be eliminated)

| Poll | Interval | Transport | Purpose |
|------|----------|-----------|---------|
| `hx-trigger="every 5s"` on `#node-list` | 5s | htmx GET → HTML | Node card list |
| `hx-trigger="every 5s"` on `#health-badges` | 5s | htmx GET → HTML | Health summary |
| `setInterval(fetchDiagnostics, 5000)` | 5s | fetch → JSON | Diagnostic list |
| `setInterval(updateDiagnosticCounts, 5000)` | 5s | fetch → JSON | Diagnostic badges |
| `setInterval(updateStderrIcons, 1000)` | 1s | DOM scan | Stderr icon animation |
| `setInterval(PID check, 2000)` | 2s | fetch → JSON | Detect node restarts |

### Target Architecture

```
Rust (axum)                             Browser (Preact + htm)
┌──────────────────┐                   ┌────────────────────────────┐
│                  │                   │                            │
│  JSON API        │──GET /api/nodes──>│  Initial load (once)       │
│    list_nodes()  │   (full state)    │         │                  │
│    get_node()    │                   │         ▼                  │
│    health()      │                   │  ┌────────────────┐        │
│    diagnostics() │                   │  │  nodeStore      │        │
│                  │                   │  │  (signal-based  │        │
│  SSE             │                   │  │   single source │        │
│    /state/updates│──StateEvent JSON─>│  │   of truth)     │        │
│    /logs/stdout  │──log lines───────>│  └───────┬────────┘        │
│    /logs/stderr  │                   │          │ reactive        │
│                  │                   │    ┌─────┴──────┐          │
│  POST actions    │<──user actions───│    ▼            ▼          │
│    start/stop    │                   │  <NodeCard>  <NodePanel>   │
│    load/unload   │                   │  <HealthBar> <DiagTable>   │
│    respawn/etc   │                   │  <LogViewer> <StderrIcon>  │
└──────────────────┘                   └────────────────────────────┘
```

### Technology Choice: Preact + htm (vendored, no build step)

- **Preact** (3KB gzip): React-compatible API, signals for state, hooks for effects
- **htm** (0.7KB): Tagged template literals → no JSX, no transpiler, no build step
- **Vendored locally** into `assets/js/vendor/` — no CDN, no network dependency
- Embedded via `rust-embed` like all other assets — works fully offline
- Result: declarative components with zero build tooling, zero external dependencies

The current htmx setup also loads from CDN (`unpkg.com`). The migration fixes this
by vendoring all JS dependencies locally.

**Vendor directory layout**:
```
src/web/assets/js/vendor/
├── preact.module.js        ← preact 10.x ESM bundle
├── preact-hooks.module.js  ← preact/hooks ESM bundle
├── signals.module.js       ← @preact/signals ESM bundle
└── htm.module.js           ← htm ESM bundle
```

Total vendored size: ~5KB gzipped. Downloaded once during development, committed
to the repo, and embedded into the binary at compile time.

```js
import { h, render } from '/assets/js/vendor/preact.module.js';
import { signal, computed } from '/assets/js/vendor/signals.module.js';
import htm from '/assets/js/vendor/htm.module.js';
const html = htm.bind(h);

// Reactive state
const nodes = signal(new Map());

// Component — re-renders automatically when nodes changes
function NodeCard({ node }) {
    return html`<div class="node-card ${statusClass(node.status)}">
        <span class="node-name">${node.name}</span>
        <button onClick=${() => startNode(node.name)}>Start</button>
    </div>`;
}
```

## Implementation Order

```
20.0 Backend: JSON-only API                    ⏳ planned
  └── 20.1 Client state store + SSE consumer   ⏳ planned
        └── 20.2 Node components                ⏳ planned
              └── 20.3 Panel + log components   ⏳ planned
                    └── 20.4 Diagnostics + cleanup  ⏳ planned
```

Each phase is independently deployable — the app works after each phase, with
an increasing percentage of the UI migrated.

---

## Phase 20.0: Backend — JSON-Only API

**Status**: Planned

Convert the remaining HTML-fragment endpoints to JSON. The backend becomes a
pure data API; all rendering moves to the frontend.

### Work Items

- [ ] Convert `list_nodes()` to return `Json<Vec<NodeSummary>>` instead of HTML
- [ ] Convert `health_summary()` to return `Json<HealthSummary>` instead of HTML badges
- [ ] Convert `load_node()` / `unload_node()` to return `Json<NodeSummary>` instead of HTML card
- [ ] Add `HealthSummary` struct to `web_types.rs` with `nodes_running`, `nodes_total`,
      `containers_running`, `containers_total`, `composable_loaded`, `composable_total`
- [ ] Keep all POST action endpoints unchanged (they already return text status)
- [ ] Keep SSE endpoints unchanged
- [ ] Remove `render_node_card()`, `render_clickable_ros_name()`, `clean_node_name()` from handlers.rs

### Files Changed

| File | Action |
|------|--------|
| `src/web/handlers.rs` | Rewrite `list_nodes()`, `health_summary()`, `load_node()`, `unload_node()` to return JSON. Delete `render_node_card()` and helper functions (~300 lines removed) |
| `src/web/web_types.rs` | Add `HealthSummary` struct |

### Passing Criteria

- [ ] `GET /api/nodes` returns `Content-Type: application/json` with `Vec<NodeSummary>`
- [ ] `GET /api/health` returns `Content-Type: application/json` with `HealthSummary`
- [ ] `POST /api/nodes/:name/load` returns JSON `NodeSummary`
- [ ] All POST action endpoints still return text status codes
- [ ] SSE endpoints unchanged
- [ ] `just build` succeeds
- [ ] Existing integration tests pass (they don't test the web UI)

---

## Phase 20.1: Client State Store + SSE Consumer

**Status**: Planned

Create the Preact app shell, the reactive state store, and connect it to
the SSE state updates endpoint. This phase renders nothing visible yet —
it replaces the data layer underneath.

### Design

**State store** — a single `signal(Map<string, NodeState>)` that holds all
node state. Updated from two sources:

1. **Initial load**: `GET /api/nodes` on page load → populate the map
2. **SSE stream**: `EventSource('/api/state/updates')` → merge each
   `StateEvent` into the map incrementally

The store also holds UI state (selected node, active tab, panel open/closed)
as separate signals.

**SSE event merging logic**:

| StateEvent | Store update |
|------------|-------------|
| `Started { name, pid }` | Set node status to Running, update pid |
| `Exited { name, exit_code }` | Set status to Stopped or Failed |
| `Respawning { name, ... }` | Set status to Running (respawning) |
| `Terminated { name }` | Set status to Stopped |
| `Failed { name, error }` | Set status to Failed |
| `LoadStarted { name }` | Set status to Loading |
| `LoadSucceeded { name, ... }` | Set status to Loaded |
| `LoadFailed { name, error }` | Set status to Failed |
| `Unloaded { name }` | Set status to Unloaded |
| `Blocked { name, reason }` | Set status to Blocked(reason) |

For fields not in the event (e.g. `stderr_size`, `respawn_delay`), the store
retains the last known value. A full refresh (`GET /api/nodes`) runs on SSE
reconnect to resync after any missed events.

### Work Items

- [ ] Vendor Preact libraries into `src/web/assets/js/vendor/`:
  - Download ESM bundles: `preact.module.js`, `preact-hooks.module.js`,
    `signals.module.js`, `htm.module.js`
  - Rewrite internal import specifiers to use relative paths
    (e.g. `from 'preact'` → `from './preact.module.js'`)
  - Commit vendored files to the repo
- [ ] Create `src/web/assets/js/app.js` as ES module entry point
- [ ] Create `src/web/assets/js/store.js` — Preact signals-based state store:
  - `nodes` signal: `Map<string, NodeSummary>`
  - `selectedNode` signal: `string | null`
  - `panelOpen` signal: `boolean`
  - `activeTab` signal: `'info' | 'stdout' | 'stderr'`
  - `theme` signal: `'light' | 'dark'`
  - `currentView` signal: `'nodes' | 'diagnostics'`
  - `diagnostics` signal: `DiagnosticStatus[]`
  - `healthSummary` signal: `HealthSummary`
- [ ] Create `src/web/assets/js/sse.js` — SSE connection manager:
  - Connects to `/api/state/updates` on init
  - On message: parse JSON, call `store.applyStateEvent(event)`
  - On error/reconnect: fetch `/api/nodes` for full resync
  - Exports nothing — side-effect module that writes to the store
- [ ] Update `index.html` to load `app.js` as `<script type="module">`
- [ ] Keep all existing JS files temporarily (old UI still works during migration)

### Files Changed

| File | Action |
|------|--------|
| `src/web/assets/js/vendor/*.module.js` | New: vendored Preact, signals, hooks, htm |
| `src/web/assets/js/app.js` | New: Preact app shell, imports store + sse |
| `src/web/assets/js/store.js` | New: reactive state store |
| `src/web/assets/js/sse.js` | New: SSE state event consumer |
| `src/web/assets/index.html` | Add `<script type="module" src="/assets/js/app.js">` |

### Passing Criteria

- [ ] Opening the page connects to `/api/state/updates` SSE (visible in browser DevTools Network tab)
- [ ] `store.nodes` signal contains all nodes after initial `GET /api/nodes`
- [ ] State events update `store.nodes` in real-time (verify by triggering a node start/stop and inspecting store in console)
- [ ] SSE reconnect triggers a full resync via `GET /api/nodes`
- [ ] No polling intervals for node state (the 5s htmx poll is removed for the new code path)
- [ ] Old UI still functions (graceful coexistence during migration)

---

## Phase 20.2: Node List Components

**Status**: Planned

Replace the htmx-driven node list with Preact components that render
reactively from the store. This is the largest single phase — it replaces
the server-rendered HTML cards with client-rendered components.

### Components

```
<App>
  <Header>
    <NavTabs />
    <HealthBar />           ← computed from store.nodes
    <DiagnosticBadges />    ← from store.diagnostics
    <ThemeToggle />
  </Header>
  <MainContainer>
    <LeftPanel>
      <NodesView>
        <SortControls />
        <SearchBox />
        <BulkOperations />
        <NodeList>
          <NodeCard node={...} />   ← one per node, grouped by container
          <NodeCard node={...} />
        </NodeList>
      </NodesView>
    </LeftPanel>
    <PanelResizer />
    <RightPanel />           ← Phase 20.3
  </MainContainer>
</App>
```

### Work Items

- [ ] Create `src/web/assets/js/components/NodeCard.js`:
  - Renders node type badge, name, PID, ROS name with clickable namespace segments
  - Control buttons (Start/Stop, Load/Unload, Load All/Unload All)
  - Auto-restart / Auto-load checkbox
  - Stderr activity icon (derived from `node.stderr_last_modified`)
  - Status-based CSS class (computed from `node.status`)
  - All buttons call `fetch()` POST actions (no htmx)
- [ ] Create `src/web/assets/js/components/NodeList.js`:
  - Groups nodes: regular nodes first, then containers with their composable children
  - Client-side sort (name, type, status) — operates on store data, not DOM
  - Client-side search filter — operates on store data, not DOM
- [ ] Create `src/web/assets/js/components/Header.js`:
  - `<HealthBar>` — computed signal from `store.nodes` (count running/total per type)
  - `<DiagnosticBadges>` — from `store.diagnostics`
  - Nav tabs, theme toggle
- [ ] Create `src/web/assets/js/components/BulkOperations.js`:
  - Start All / Stop All with confirm dialogs
- [ ] Wire `<NodeList>` into `<App>` as the primary view
- [ ] Remove htmx `hx-get="/api/nodes"` and `hx-trigger="every 5s"` from `#node-list`
- [ ] Remove htmx `hx-get="/api/health"` and `hx-trigger="every 5s"` from `#health-badges`
- [ ] Remove `hx-post`, `hx-swap`, `hx-target`, `hx-disabled-elt` from all button HTML
- [ ] Move stderr icon logic from `stderr-monitoring.js` into `<NodeCard>` component
  (derive icon state from `node.stderr_last_modified` signal — no polling needed)

### Files Changed

| File | Action |
|------|--------|
| `src/web/assets/js/components/NodeCard.js` | New |
| `src/web/assets/js/components/NodeList.js` | New |
| `src/web/assets/js/components/Header.js` | New |
| `src/web/assets/js/components/BulkOperations.js` | New |
| `src/web/assets/js/app.js` | Mount `<App>` with node list |
| `src/web/assets/index.html` | Simplify: remove htmx attributes, reduce to mount point |

### Passing Criteria

- [ ] Node list renders from store (not from server HTML)
- [ ] Node cards update in real-time when state events arrive (no 5s delay)
- [ ] Health badges update reactively (no polling)
- [ ] Sort and filter work on store data (client-side, instant)
- [ ] Start/Stop/Load/Unload buttons work via `fetch()` POST
- [ ] Auto-restart and Auto-load checkboxes work
- [ ] Stderr icons appear/disappear based on `stderr_last_modified`
- [ ] Container → composable node hierarchy preserved
- [ ] Clickable namespace segments work
- [ ] No htmx attributes remain in the DOM
- [ ] Zero polling intervals for node state or health

---

## Phase 20.3: Right Panel + Log Viewer Components

**Status**: Planned

Replace the right panel (detail view, log tabs) with Preact components.
Log streaming stays SSE-based but managed within the component lifecycle.

### Components

```
<RightPanel>
  <PanelHeader>
    <NodeTitle />
    <StateBadge />
    <CloseButton />
  </PanelHeader>
  <PanelTabs>
    <TabButton label="Info" />
    <TabButton label="stdout" />
    <TabButton label="stderr" />
  </PanelTabs>
  <PanelContent>
    <InfoTab node={...} />          ← JSON viewer
    <LogTab type="stdout" />        ← SSE log stream
    <LogTab type="stderr" />        ← SSE log stream
  </PanelContent>
</RightPanel>
```

### Work Items

- [ ] Create `src/web/assets/js/components/RightPanel.js`:
  - Reads `store.selectedNode`, `store.panelOpen`, `store.activeTab`
  - Renders nothing when `panelOpen` is false
  - Panel header with node name, state badge (reactive), close button
  - Tab switching updates `store.activeTab`
- [ ] Create `src/web/assets/js/components/InfoTab.js`:
  - JSON viewer for node details (port `renderJSON` logic)
- [ ] Create `src/web/assets/js/components/LogTab.js`:
  - Manages `EventSource` lifecycle via `useEffect` hook:
    - Opens SSE on mount / tab switch
    - Closes SSE on unmount / tab switch / node change
  - For composable nodes: connects to parent container's log endpoint
  - Shows connection status, auto-scroll, clear, scroll-to-bottom
  - 5000-line limit
  - Auto-reconnects on PID change (detected via store signal, not polling)
- [ ] Create `src/web/assets/js/components/PanelResizer.js`:
  - Drag to resize, persist width to localStorage
- [ ] Wire clicking "View" button in `<NodeCard>` to set `store.selectedNode`
  and `store.panelOpen = true`
- [ ] Remove PID change polling interval (PID changes arrive via SSE `Started` event)
- [ ] Remove old `panels.js`, `logs.js` files

### Files Changed

| File | Action |
|------|--------|
| `src/web/assets/js/components/RightPanel.js` | New |
| `src/web/assets/js/components/InfoTab.js` | New |
| `src/web/assets/js/components/LogTab.js` | New |
| `src/web/assets/js/components/PanelResizer.js` | New |
| `src/web/assets/js/panels.js` | Delete |
| `src/web/assets/js/logs.js` | Delete |

### Passing Criteria

- [ ] Clicking "View" opens the right panel with node details
- [ ] State badge updates in real-time (from store signal)
- [ ] Info tab shows JSON-formatted node details
- [ ] stdout/stderr tabs stream logs via SSE
- [ ] SSE connections are properly opened/closed on tab switch and panel close
- [ ] Composable node log tabs redirect to parent container's logs
- [ ] PID change triggers log reconnect (detected from SSE, no polling)
- [ ] Panel resize works and persists to localStorage
- [ ] Escape key closes panel
- [ ] No `setInterval` for PID checking remains

---

## Phase 20.4: Diagnostics View + Cleanup

**Status**: Planned

Migrate the diagnostics view to Preact, remove all legacy JS files,
remove htmx dependency, and clean up.

### Work Items

- [ ] Create `src/web/assets/js/components/DiagnosticsView.js`:
  - Fetches diagnostic data via SSE or periodic fetch (diagnostics don't have
    an SSE endpoint yet — keep 5s fetch for now, but from a `useEffect` hook
    that only runs when the diagnostics view is active)
  - Sortable table, search filter, level badges, relative timestamps
  - Ports all logic from `diagnostics.js`
- [ ] Move theme management into the store (`store.theme` signal)
- [ ] Delete legacy JS files:
  - `nodes.js` (sorting/filtering now in `<NodeList>`)
  - `htmx-handlers.js` (no more htmx)
  - `stderr-monitoring.js` (now in `<NodeCard>`)
  - `diagnostics.js` (now in `<DiagnosticsView>`)
  - `utils.js` (`renderJSON` and `escapeHtml` moved to components)
  - `theme.js` (moved to store)
- [ ] Remove htmx CDN `<script>` tags from `index.html` (`unpkg.com/htmx.org`)
- [ ] Remove htmx SSE extension CDN `<script>` tag from `index.html`
- [ ] Simplify `index.html` to a single `<div id="app">` mount point +
      `<script type="module" src="/assets/js/app.js">`
- [ ] Verify all CSS still applies (class names unchanged)
- [ ] Consider adding SSE endpoint for diagnostics (future — not required for this phase)

### Files Deleted

| File | Reason |
|------|--------|
| `src/web/assets/js/nodes.js` | Logic moved to `<NodeList>` component |
| `src/web/assets/js/htmx-handlers.js` | htmx removed |
| `src/web/assets/js/stderr-monitoring.js` | Logic moved to `<NodeCard>` component |
| `src/web/assets/js/diagnostics.js` | Logic moved to `<DiagnosticsView>` component |
| `src/web/assets/js/utils.js` | Helpers inlined in components |
| `src/web/assets/js/theme.js` | Moved to store |
| `src/web/assets/js/panels.js` | Already deleted in 20.3 |
| `src/web/assets/js/logs.js` | Already deleted in 20.3 |

### Passing Criteria

- [ ] Diagnostics view works: sortable table, search, level badges, timestamps
- [ ] Switching between Nodes and Diagnostics views works
- [ ] No htmx `<script>` tags in `index.html`
- [ ] No `<script>` tags for legacy JS files
- [ ] Only `app.js` (module) loaded in `index.html`
- [ ] Zero `setInterval` / `setTimeout` polling for node state
- [ ] Diagnostics polling only active when diagnostics view is visible
- [ ] Theme toggle works, persists to localStorage
- [ ] All CSS classes and visual appearance unchanged
- [ ] Web UI loads and functions with no network access (fully offline)
- [ ] No external CDN `<script>` or `<link>` tags remain in `index.html`
- [ ] `just build` succeeds
- [ ] `just test` passes (integration tests don't test web UI)
- [ ] `just quality` passes

---

## Summary

### Before vs After

| Aspect | Before (htmx + vanilla JS) | After (Preact + SSE) |
|--------|---------------------------|---------------------|
| Rendering | Split: server HTML + client JS | Client-only (Preact components) |
| State management | DOM (classList, attributes) | Reactive signals (single store) |
| Updates | 6 polling intervals (1–5s each) | 1 SSE connection (real-time) |
| Dependencies | htmx (14KB, CDN) + SSE ext (2KB, CDN) | Preact (3KB) + signals (1KB) + htm (0.7KB) — all vendored |
| Build step | None | None (vendored ES modules) |
| Offline support | No (htmx loaded from unpkg.com CDN) | Yes (all assets embedded in binary) |
| Node list | Server renders ~400 lines of `format!()` HTML | `<NodeCard>` component (~100 lines) |
| Adding a field | Change Rust HTML + JS rendering + CSS | Change component + CSS |
| Backend | Mixed (HTML fragments + JSON + SSE) | Pure (JSON + SSE) |
| JS files | 8 files, 700+ lines, global state | ~8 component modules, scoped state |
| Log streaming | SSE (unchanged) | SSE (unchanged, managed by hooks) |

### Risk Assessment

- **Low risk**: No backend logic changes. All business logic stays in Rust actors.
  The refactor only changes how data is presented.
- **Fully offline**: All JS dependencies vendored locally and embedded via
  `rust-embed`. No CDN, no network dependency. Works in air-gapped environments.
  (This also fixes the current htmx CDN dependency.)
- **No build step**: htm tagged templates have a tiny runtime parse cost (~0.1ms
  per render). Negligible for this scale of UI.
- **Vendored dependency updates**: Preact/htm updates require manually downloading
  new ESM bundles and committing them. Acceptable given they're stable, tiny
  libraries with infrequent releases.
- **Phased rollout**: Each phase produces a working app. Can pause at any phase.
