# Phase 8: Web UI

**Status**: ✅ Complete

**Completed**: 2025-12-18

**Estimated Effort**: 5-7 days

**Actual Effort**: ~6 days

---

## Goal

Add an optional web-based management interface for monitoring and controlling nodes during replay.

---

## Features

1. **Node List View**: Display all nodes with status badges (running/stopped/failed/noisy)
2. **Node Control**: Start, stop, and restart individual nodes
3. **Node Details**: View PID, command line, package/executable info
4. **Live Logs**: Stream stdout/stderr via Server-Sent Events (SSE)
5. **Health Summary**: Show counts of running, failed, and noisy nodes
6. **Container Hierarchy**: Visual grouping of containers and composable nodes
7. **Resizable Panels**: Adjustable left/right panel split
8. **Dark/Light Theme**: Respects user preferences

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Browser (htmx)                         │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    axum Web Server                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │ REST API    │  │ SSE Stream  │  │ Embedded Assets     │  │
│  │ /api/nodes  │  │ /api/logs   │  │ index.html (htmx)   │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    NodeRegistry                             │
│  - Tracks node name → Child process handle                  │
│  - Queries /proc/{pid} for status                           │
│  - Reads play_log/.../node/{name}/err for logs              │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    Filesystem                               │
│  play_log/YYYY-MM-DD_HH-MM-SS/                              │
│  ├── node/{name}/pid, out, err, cmdline, status             │
│  └── load_node/{name}/...                                   │
└─────────────────────────────────────────────────────────────┘
```

**Design Principles**:
- Filesystem as source of truth (no in-memory log storage)
- Single binary deployment (embedded HTML/JS assets)
- Minimal dependencies (htmx for reactivity, no build step)
- Default disabled (`--web-ui` flag to enable)

---

## API Endpoints

| Endpoint | Method | Response | Description |
|----------|--------|----------|-------------|
| `/` | GET | HTML | Serve embedded index.html |
| `/api/nodes` | GET | HTML fragment | List nodes (for htmx swap) |
| `/api/nodes/{name}` | GET | JSON | Node details |
| `/api/nodes/{name}/start` | POST | 200/4xx | Start node |
| `/api/nodes/{name}/stop` | POST | 200/4xx | Stop node |
| `/api/nodes/{name}/restart` | POST | 200/4xx | Restart node |
| `/api/nodes/{name}/logs/stdout` | GET | SSE stream | Stream stdout logs |
| `/api/nodes/{name}/logs/stderr` | GET | SSE stream | Stream stderr logs |
| `/api/health` | GET | HTML fragment | Health summary badges |

---

## Implementation Highlights

### 8.1 NodeRegistry

**File**: `src/play_launch/src/node_registry.rs`

Core structure for tracking and controlling nodes:
- `register_node()` - called when node spawns
- `get_status(name)` - Running/Stopped/Failed based on process state
- `list_nodes()` - return all nodes with summary info
- `start_node(name)` - spawn process, store Child handle
- `stop_node(name)` - send SIGTERM
- `restart_node(name)` - stop then start
- `get_health_summary()` - calculate badge counts

### 8.2 Web Server Module

**File**: `src/play_launch/src/web/mod.rs`

Using `rust-embed` to embed static assets in the binary:
```rust
#[derive(RustEmbed)]
#[folder = "src/web/assets/"]
struct Assets;

pub struct WebState {
    pub registry: Arc<Mutex<NodeRegistry>>,
}
```

### 8.3 SSE Log Streaming

**File**: `src/play_launch/src/web/sse.rs`

File tailing implementation:
- On connect: send last 100 lines
- Poll file for changes every 500ms
- Stream new lines as SSE events
- Handle file not existing (node not started)
- Cleanup on client disconnect

### 8.4 Frontend

**File**: `src/play_launch/src/web/assets/index.html`

Single HTML file with:
- Embedded CSS and JavaScript (no build step)
- htmx for reactive updates (polling every 2s)
- Server-Sent Events for live log streaming
- Dark/light theme toggle
- Resizable panel divider with drag support
- LocalStorage for preferences

---

## Dependencies

| Crate | Version | Purpose |
|-------|---------|---------|
| axum | 0.7 | Web framework (tokio-native) |
| tower-http | 0.6 | CORS middleware |
| rust-embed | 8 | Embed static files in binary |

---

## Usage

```bash
# Enable web UI with default port (8080)
play_launch replay --web-ui

# Custom port
play_launch replay --web-ui --web-ui-port 3000

# With monitoring enabled
play_launch launch autoware_launch planning_simulator.launch.xml \
    map_path:=/path/to/map \
    --web-ui \
    --enable-monitoring
```

Open browser to `http://localhost:8080` to access the interface.

---

## Key Improvements During Development

### UI/UX Enhancements
1. **Fixed SSE connection delay**: Added immediate comment event to prevent "Connecting..." delay
2. **Fixed panel scrolling**: Made header and right panel fixed, only node list scrolls
3. **Container hierarchy**: Visual grouping of composable nodes under containers
4. **Full node names**: Show exec_name as primary name with ROS node name for reference
5. **Node type badges**: Color-coded badges for node/container/composable types
6. **Resizable panels**: Drag divider to adjust left/right panel split
7. **Button visibility**: Fixed control button colors for better theme support
8. **Single control button**: Merged Start/Stop buttons based on node status

### Technical Fixes
1. **PID display**: Hide PID for composable nodes (they don't have separate processes)
2. **Status accuracy**: Proper status determination from filesystem (PID, status files)
3. **Noisy detection**: Track stderr file size growth, flag nodes exceeding threshold

---

## Testing

### Manual Test Scenarios

- [x] Start replay with `--web-ui`, open browser
- [x] Verify node list updates when nodes start/stop
- [x] Verify log streaming works (stdout and stderr)
- [x] Verify start/stop/restart controls work
- [x] Test with Autoware (15 containers, 54 composable nodes)
- [x] Test container hierarchy display
- [x] Test panel resizing and theme toggle
- [x] Test SSE reconnection after server restart

### Integration Tests

- [x] Tested with demo_nodes_cpp (simple nodes)
- [x] Tested with Autoware planning simulator (complex containerized setup)
- [x] Verified no performance impact when web UI disabled

---

## Success Criteria

- [x] `--web-ui` flag enables web server
- [x] Node list shows all nodes with correct status
- [x] Start/stop/restart buttons work
- [x] Live log streaming via SSE works
- [x] Health summary shows accurate counts (including noisy nodes)
- [x] Single binary deployment (no external files)
- [x] Minimal performance impact when disabled
- [x] Documentation updated in CLAUDE.md
- [x] Container hierarchy clearly visible
- [x] Resizable panels with persistent preferences
- [x] Theme support (dark/light)

---

## Known Limitations

1. **No individual composable node logs**: Composable nodes log to their container's stdout/stderr (by design)
2. **Container restart orphans composable nodes**: Restarting a container doesn't reload composable nodes (addressed in Phase 9)
3. **Status ambiguity**: "Running/Stopped" doesn't distinguish process counts from node counts (addressed in Phase 9)
4. **No load/unload for composable nodes**: Web UI doesn't support runtime load/unload operations (future enhancement)

---

## Future Enhancements

See [Phase 9: Web UI Status Refactoring](./phase-9.md) for planned improvements.

Optional future features:
- [ ] Add filtering by status (show only failed, show only running, etc.)
- [ ] Show sample error lines in node details
- [ ] Add "Follow" mode for log viewer (auto-scroll to bottom)
- [ ] WebSocket support for bidirectional communication
- [ ] Multi-user support with authentication
