# Phase 26: Web UI Metrics Dashboard

**Status**: Complete
**Priority**: Medium (Observability)
**Dependencies**: Phase 24 (Parameter Control — established tab pattern), Monitoring system (existing)
**Completed**: 2026-03-01

---

## Overview

The monitoring system already collects comprehensive per-process and system-wide
metrics (CPU, RSS, I/O, GPU, network) and writes them to CSV files every 2 seconds.
However, this data was invisible in the Web UI — users had to wait until after a run
to analyze CSV files with `play_launch plot`.

This phase adds live metrics to the Web UI via two additions:

1. **Header badges** — system-wide CPU% and memory% always visible
2. **Metrics tab** — per-node charts in the right panel

### Performance-first design

Running hundreds of nodes (e.g., full Autoware: 73 nodes, production: 200+), the
design is **O(1) relative to node count** for SSE streaming:

- **System stats**: one 350-byte event broadcast every 2s — trivial
- **Per-node metrics**: tail `metrics.csv` for the **selected node only**, using
  the existing log-tailing SSE pattern — no new broadcaster, no all-node fanout

This means zero per-node SSE cost for unviewed nodes. At 200 nodes with 5 browser
tabs open, total added network is ~2 KB/s.

---

## Design

### Data flow

```
ResourceMonitor (existing, 2s interval)
  |
  |---> metrics.csv per node (existing, unchanged)
  |
  |---> SystemMetricsBroadcaster (NEW)
          |
          +--> SSE /api/metrics/system ---> Header badges (CPU%, Mem%)

Browser selects node
  |
  +--> SSE /api/metrics/node/:name ---> tail metrics.csv (reuse log SSE pattern)
                                        MetricsTab (per-node charts)
```

### Why not broadcast all nodes

| Approach | Network (200 nodes, 5 clients) | CPU | Complexity |
|----------|-------------------------------|-----|------------|
| Broadcast all nodes | 380 KB/s | serialize 200 structs/tick | New broadcaster, memory for all |
| Tail selected node only | 2 KB/s | parse 1 CSV line/tick | Reuse existing SSE infra |

The tail approach is 190x cheaper and reuses proven code (`sse.rs` log tailing).

### System stats payload

```json
{
  "cpu_percent": 42.3,
  "cpu_count": 8,
  "memory_used_bytes": 8589934592,
  "memory_total_bytes": 17179869184,
  "network_rx_rate_bps": 125000,
  "network_tx_rate_bps": 85000,
  "disk_read_rate_bps": 1048576,
  "disk_write_rate_bps": 524288,
  "gpu_utilization_percent": 65.0,
  "gpu_memory_used_bytes": 2147483648,
  "gpu_memory_total_bytes": 8589934592
}
```

~350 bytes per event. At 2s interval = 175 bytes/s per client.

### Per-node metrics (CSV tail)

The existing `metrics.csv` has 30 columns. The SSE handler tails the file (same as
stdout/stderr streaming) and sends each new line as a CSV-format SSE event. The
frontend parses the line and appends to a ring buffer for chart rendering.

No JSON conversion server-side — raw CSV is smaller and the frontend already needs
to parse for charting. This keeps the backend change minimal (reuse `tail_file_sse`).

---

## Phase 26.1: SystemMetricsBroadcaster — Complete

**New file**: `src/play_launch/src/web/metrics_broadcaster.rs`

A lightweight broadcaster that multicasts `SystemStats` snapshots to SSE subscribers.
Follows the same pub/sub pattern as `StateEventBroadcaster`.

- `SystemMetricsBroadcaster` with `subscribe()` / `broadcast()` pattern
- `SystemStatsSnapshot` struct with `#[derive(Clone, Serialize)]` and `From<&SystemStats>`
- Channel size 16 (small — metrics arrive every 2s)
- Skips broadcast when subscriber count is 0 (zero overhead in common case)
- Re-exported from `web::mod.rs` as `web::SystemMetricsBroadcaster` and `web::SystemStatsSnapshot`
- `WebState` extended with `metrics_broadcaster: Option<Arc<SystemMetricsBroadcaster>>`

---

## Phase 26.2: Wire broadcaster into monitoring loop — Complete

**Files modified**: `resource_monitor.rs`, `replay.rs`, `run.rs`

- `run_monitoring_task` takes optional `Arc<SystemMetricsBroadcaster>` parameter
- After `write_system_csv()` succeeds, converts to `SystemStatsSnapshot` and broadcasts
- `replay.rs` creates `SystemMetricsBroadcaster` before spawning monitoring task, passes `Arc` to both monitoring and `WebState`
- `run.rs` passes `None` (run mode has no replay metrics)

---

## Phase 26.3: SSE endpoints — Complete

**File modified**: `src/play_launch/src/web/sse.rs`, `src/play_launch/src/web/mod.rs`

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/metrics/system` | GET (SSE) | System-wide stats stream (from broadcaster) |
| `/api/metrics/node/:name` | GET (SSE) | Per-node metrics (tail `metrics.csv`) |

- `stream_system_metrics`: subscribes to `SystemMetricsBroadcaster`, streams JSON SSE events, 3s keepalive, returns 503 if monitoring disabled
- `stream_node_metrics`: tails `metrics.csv` using file-streaming pattern, sends last 60 lines on connect (2min history), polls at 500ms, returns 404 if node unknown

---

## Phase 26.4: Header badges (system CPU & memory) — Complete

**Files modified**: `Header.js`, `store.js`, `sse.js`, `navigation.css`

- Added `systemMetrics` signal to `store.js`
- Added second `EventSource` in `sse.js` connecting to `/api/metrics/system` with auto-reconnect
- `MetricsBadges` component in `Header.js` shows CPU%, Mem%, and GPU% (when present)
- Color coding: green (< 60%), yellow (60–85%), red (>= 85%) using existing theme variables
- CSS classes `.metrics-badge-ok`, `.metrics-badge-warning`, `.metrics-badge-critical`

---

## Phase 26.5: MetricsTab frontend component — Complete

**New files**: `MetricsTab.js`, `metrics.css`

### Chart features
- 4 chart metrics (CPU%, RSS, I/O Read, I/O Write) rendered as 80px-tall `<canvas>` charts
- 4 text-only metrics (Threads, FDs, State, TCP/UDP) shown as label/value rows
- Ring buffer (150 entries = 5min at 2s interval) with parallel timestamp array
- Grid lines (3 horizontal + vertical at each time label) for visual value reading
- Y-axis: max and min value labels using each metric's own formatter
- X-axis: relative time labels ("now", "-30s", "-1m", "-2m", etc.)
- Time label spacing adapts to panel width (min 50px gap, nice intervals: 10s/15s/20s/30s/60s/...)
- Canvas text colours resolved from CSS custom properties (`--text-secondary`, `--border-color`) via `getComputedStyle()` — works in both light and dark themes
- Charts scale to full panel width; hidden at narrow widths (< 280px) showing numbers only

### Time range selector
- Button bar at top of tab: 1m / 2m / 5m (default: 2m)
- `RingBuffer.window(ms)` slices data to selected range without discarding history
- Charts zoom in/out without losing buffered data

### CSV timestamp parsing
- Timestamps are ISO 8601 strings (`2026-03-01T12:34:56.789Z`), parsed with `Date.parse()` to epoch-ms

### SSE lifecycle
- Connects to `/api/metrics/node/:name` on mount, disconnects on unmount
- Header row detection for column mapping
- Composable nodes without own logs show notice about parent container

---

## Phase 26.6: Register tab in RightPanel — Complete

**File modified**: `src/play_launch/src/web/assets/js/components/RightPanel.js`

- Tab order: Info, Params, Topics, Metrics, stdout, stderr
- "Member Info" tab renamed to "Info"
- `MetricsTab` lazy-rendered only when active (disconnects SSE when hidden)
- Added CSS link for `metrics.css` in `index.html`

---

## Files created/modified

| File | Action | Phase |
|------|--------|-------|
| `src/play_launch/src/web/metrics_broadcaster.rs` | **Created** | 26.1 |
| `src/play_launch/src/web/mod.rs` | Modified (module, re-exports, WebState, routes) | 26.1, 26.3 |
| `src/play_launch/src/monitoring/resource_monitor.rs` | Modified (broadcaster param + broadcast call) | 26.2 |
| `src/play_launch/src/commands/replay.rs` | Modified (create broadcaster, wire to monitoring + WebState) | 26.2 |
| `src/play_launch/src/commands/run.rs` | Modified (pass None for metrics broadcaster) | 26.2 |
| `src/play_launch/src/web/sse.rs` | Modified (2 new SSE handlers) | 26.3 |
| `src/play_launch/src/web/assets/js/store.js` | Modified (systemMetrics signal) | 26.4 |
| `src/play_launch/src/web/assets/js/sse.js` | Modified (metrics EventSource) | 26.4 |
| `src/play_launch/src/web/assets/js/components/Header.js` | Modified (MetricsBadges component) | 26.4 |
| `src/play_launch/src/web/assets/css/navigation.css` | Modified (badge color classes) | 26.4 |
| `src/play_launch/src/web/assets/js/components/MetricsTab.js` | **Created** | 26.5 |
| `src/play_launch/src/web/assets/css/metrics.css` | **Created** | 26.5 |
| `src/play_launch/src/web/assets/index.html` | Modified (metrics.css link) | 26.5 |
| `src/play_launch/src/web/assets/js/components/RightPanel.js` | Modified (tab order, rename, Metrics tab) | 26.6 |

---

## Verification

### Build & test (automated) — Passed

- [x] `cargo build -p play_launch` compiles without errors
- [x] `just test` — 30/30 tests pass, 5 skipped

### Manual testing

- [ ] Start `play_launch replay` with Autoware config (73+ nodes)
- [ ] Header shows CPU% and Mem% badges that update every 2s
- [ ] Select a running node → Metrics tab shows live charts
- [ ] Time range selector (1m/2m/5m) zooms charts
- [ ] Switch between nodes → old SSE disconnects, new one connects
- [ ] Switch away from Metrics tab → SSE disconnects (verify in browser DevTools)
- [ ] Charts readable in both light and dark theme
- [ ] GPU badge only appears on NVIDIA systems
- [ ] No JavaScript errors in console during normal operation
