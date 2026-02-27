# Phase 24: Web UI Parameter Control

**Status**: Complete
**Priority**: Medium (User Experience)
**Dependencies**: Phase 19 (Isolated Container), Phase 20 (Web UI — optional, works with current UI)

---

## Overview

Add a "Params" tab to the Web UI right panel that lists a node's runtime parameters
and lets users edit them. play_launch already knows every node name from `record.json`
and has a shared `rclrs::Node` with a spinning executor — so it can call ROS 2
parameter services directly, bypassing the DDS discovery step that makes
`rqt_reconfigure` slow.

See [ros2-parameter-services.md](../research/ros2-parameter-services.md) for the
underlying ROS 2 parameter service protocol.

---

## Design

### Data flow

```
Browser (ParametersTab.js)
  |                      ^
  | GET /api/nodes/:n/   | JSON response
  |     parameters       | + SSE parameter_changed events
  | POST /api/nodes/:n/  |
  |     parameters       |
  v                      |
Axum handlers (handlers.rs)
  |                      ^
  | member_handle.       | Result<Vec<ParamEntry>>
  |   get_parameters()   |
  |   set_parameter()    |
  v                      |
MemberHandle (handle.rs)
  |                      ^
  | ParameterProxy       |
  |   .list_describe_get |
  |   .set()             |
  v                      |
rclrs service clients ----> target node's parameter services
```

No new actor type or ControlEvent variant. Parameter operations are stateless
request/response — handled directly in MemberHandle via a helper struct.

### Zero discovery cost

Every ROS 2 node exposes six parameter services at predictable names:
`{node_fqn}/list_parameters`, `{node_fqn}/get_parameters`, etc. play_launch
already knows each node's fully-qualified name (namespace + name from `record.json`,
or `full_node_name` from `LoadSucceeded` for composable nodes), so it constructs
service client names directly — no `get_node_names_and_namespaces()` discovery call.

### Composable nodes

For composable nodes, the Params tab queries **both** the composable node's own
parameter services and the parent container's. Results display in two groups
("Node Parameters" / "Container Parameters"). The composable node's FQN comes from
`LoadSucceeded.full_node_name`; the container's FQN is already known at registration.

---

## Phase 24.1: ParameterProxy (Rust service client wrapper)

**New file**: `src/play_launch/src/ros/parameter_proxy.rs`

A struct wrapping the parameter service clients for one target node:

```rust
pub struct ParameterProxy {
    list_client:     rclrs::Client<rcl_interfaces::srv::ListParameters>,
    get_client:      rclrs::Client<rcl_interfaces::srv::GetParameters>,
    set_client:      rclrs::Client<rcl_interfaces::srv::SetParameters>,
    describe_client: rclrs::Client<rcl_interfaces::srv::DescribeParameters>,
}
```

**Constructor**: `ParameterProxy::new(ros_node: &Arc<rclrs::Node>, target_fqn: &str)`
- Builds service names from target FQN
- Creates four clients on the shared ROS node (reusing its executor thread)

**Methods**:

| Method | Description |
|--------|-------------|
| `list_and_describe()` | Calls `list_parameters` then `describe_parameters` → `Vec<ParamDescriptor>` |
| `get_values(names)` | Calls `get_parameters` → `Vec<ParamEntry>` |
| `list_all()` | Convenience: list → describe → get in one call → full snapshot |
| `set(name, value)` | Calls `set_parameters([param])` → `SetParamResult` |

All calls use `tokio::time::timeout(5s)`.

**Why not cache proxies**: service clients are cheap to create in rclrs (DDS entity
registration), and caching adds lifetime/cleanup complexity when nodes restart.
Creating on-demand is simpler and sufficient for interactive use.

---

## Phase 24.2: Parameter types

**New file**: `src/play_launch/src/ros/parameter_types.rs`

```rust
/// Wire-friendly parameter value enum (maps to ParameterType constants)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", content = "value")]
pub enum ParamValue {
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    BoolArray(Vec<bool>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<String>),
    ByteArray(Vec<u8>),
    NotSet,
}

/// Full parameter entry for the web UI
#[derive(Debug, Clone, Serialize)]
pub struct ParamEntry {
    pub name: String,
    pub value: ParamValue,
    pub type_name: String,       // "bool", "integer", "double", "string", etc.
    pub description: String,
    pub read_only: bool,
    pub integer_range: Option<IntegerRange>,     // {from, to, step}
    pub floating_point_range: Option<FloatRange>, // {from, to, step}
}

/// Result of a set operation
#[derive(Debug, Serialize)]
pub struct SetParamResult {
    pub successful: bool,
    pub reason: String,
}
```

Conversion helpers: `ParamValue <-> rcl_interfaces::msg::ParameterValue` (bidirectional).
Reuse existing `parameter_conversion.rs` helpers (`create_bool_parameter`, etc.) for
the `ParamValue -> ParameterValue` direction.

---

## Phase 24.3: MemberHandle integration

**File**: `src/play_launch/src/member_actor/coordinator/handle.rs`

Add fields:
- `shared_ros_node: Option<Arc<rclrs::Node>>` (already passed to `builder.spawn()`)
- `node_fqn_map: HashMap<String, String>` — member name -> fully-qualified ROS node name

New methods:

```rust
/// Get all parameters for a node. Creates a temporary ParameterProxy.
pub async fn get_parameters(&self, member_name: &str) -> Result<Vec<ParamEntry>>

/// Set a single parameter on a node.
pub async fn set_parameter(&self, member_name: &str, name: &str, value: ParamValue)
    -> Result<SetParamResult>
```

**FQN population**:
- **Process nodes/containers**: `format!("{}/{}", namespace, node_name)` at builder
  registration time
- **Composable nodes**: inserted when coordinator processes `LoadSucceeded { full_node_name }`

For nodes whose FQN isn't yet known (not started, load failed), `get_parameters`
returns an error that the frontend renders as "Node not running".

---

## Phase 24.4: Web API endpoints

**File**: `src/play_launch/src/web/handlers.rs`

| Endpoint                      | Method | Body              | Response          |
|-------------------------------|--------|-------------------|-------------------|
| `/api/nodes/:name/parameters` | GET    | —                 | `Vec<ParamEntry>` |
| `/api/nodes/:name/parameters` | POST   | `SetParamRequest` | `SetParamResult`  |

Request type:
```rust
#[derive(Deserialize)]
pub struct SetParamRequest {
    pub name: String,
    pub value: ParamValue,
}
```

Register in `create_router()` (`src/play_launch/src/web/mod.rs`).

---

## Phase 24.5: SSE parameter change events

Subscribe to `/parameter_events` topic (global, all nodes publish here) from the
shared ROS node. Filter events by managed node FQNs. Broadcast via existing
`StateEventBroadcaster`.

New `StateEvent` variant:
```rust
ParameterChanged {
    name: String,              // member name (not FQN)
    parameters: Vec<ParamUpdate>,  // { param_name, value }
}
```

**Task lifecycle**: follows the existing managed-task pattern in `replay.rs`:

1. **ROS callback → channel bridge**: create subscription on the rclrs executor
   thread, bridge to tokio via `unbounded_channel` (same pattern as ComponentEvent
   in `container_actor/mod.rs:375-384`):
   ```rust
   let (param_event_tx, param_event_rx) = tokio::sync::mpsc::unbounded_channel();
   let _sub = ros_node.create_subscription(
       "/parameter_events".reliable().keep_last(1000),
       move |msg: rcl_interfaces::msg::ParameterEvent| {
           let _ = param_event_tx.send(msg);
       },
   );
   ```

2. **Filter task**: spawned via `tokio::spawn`, receives `shutdown_signal` clone,
   exits its recv loop on shutdown:
   ```rust
   let param_shutdown = shutdown_signal.clone();
   let param_events_task = tokio::spawn(async move {
       loop {
           tokio::select! {
               Some(event) = param_event_rx.recv() => {
                   if let Some(member_name) = fqn_to_member.get(&event.node) {
                       let updates = /* extract changed + new params */;
                       if !updates.is_empty() {
                           broadcaster.broadcast(StateEvent::ParameterChanged {
                               name: member_name.clone(),
                               parameters: updates,
                           }).await;
                       }
                   }
               }
               _ = param_shutdown.changed() => break,
               else => break,
           }
       }
       Ok(())
   });
   ```

3. **Registered as NamedTask**: collected alongside `anchor`, `stats`, `monitor`,
   `web_ui`, etc. in the `FuturesUnordered` that `wait_for_completion` joins:
   ```rust
   named_tasks.push(NamedTask {
       name: "parameter_events",
       task: param_events_task,
   });
   ```

The `_sub` handle must be kept alive (stored alongside the task or in a variable
that outlives the select loop) — dropping it unsubscribes from the topic.

**Frontend**: `applyStateEvent` in `store.js` handles `parameter_changed` events
by notifying `ParametersTab` to refresh the affected rows.

---

## Phase 24.6: ParametersTab frontend component

**New file**: `src/play_launch/src/web/assets/js/components/ParametersTab.js`

### Behavior

1. **On mount / node selection**: `GET /api/nodes/{name}/parameters` -> populate list
2. **Display**: table with columns Name, Value, Type, Description
3. **Editing**: click a value cell -> inline editor appears
4. **Submit**: `POST /api/nodes/{name}/parameters` with name + new value
5. **SSE updates**: `parameter_changed` event refreshes affected rows
6. **Refresh button**: manual re-fetch
7. **Search box**: filter parameters by name substring (client-side)

### State awareness

| Node state                  | Params tab behavior                         |
|-----------------------------|---------------------------------------------|
| Running / Loaded            | Fetch and display; editing enabled          |
| Pending / Loading           | "Waiting for node to start..."              |
| Stopped / Failed / Unloaded | "Node not running — parameters unavailable" |
| Blocked                     | "Node blocked — parameters unavailable"     |

### Type-aware input controls

| Parameter type | Input element                      | Details                                 |
|----------------|------------------------------------|-----------------------------------------|
| `bool`         | Toggle switch                      | On/off, immediate submit                |
| `integer`      | `<input type="number" step="1">`   | Respects `integer_range` min/max        |
| `double`       | `<input type="number" step="any">` | Respects `floating_point_range` min/max |
| `string`       | `<input type="text">`              | Submit on Enter or blur                 |
| `*_array`      | Comma-separated text field         | Parse on submit; validate element types |
| `byte_array`   | Hex text field                     | Display as hex string                   |
| `read_only`    | Disabled input + lock icon         | Visually distinct, no edit affordance   |

Validation runs before submitting. On error (from `SetParametersResult.reason`),
show inline red text below the input. On success, flash the row green briefly.

---

## Phase 24.7: Register tab in RightPanel

**File**: `src/play_launch/src/web/assets/js/components/RightPanel.js`

Add fourth tab button ("Params") and content pane following the existing pattern
(lazy-rendered via conditional `html` template).

**CSS** additions in `panels.css` or new `parameters.css`:
- `.param-table` — full-width table with sticky header
- `.param-name` — monospace, truncatable with tooltip
- `.param-value` — right-aligned for numbers, left for strings
- `.param-readonly` — dimmed with lock icon
- `.param-input` — inline edit field
- `.param-error` / `.param-success` — inline feedback
- `.param-search` — search input at top

---

## Files to create/modify

| File                                                            | Action                                                       |
|-----------------------------------------------------------------|--------------------------------------------------------------|
| `src/play_launch/src/ros/parameter_types.rs`                    | **New** — ParamValue, ParamEntry, SetParamResult, ranges     |
| `src/play_launch/src/ros/parameter_proxy.rs`                    | **New** — ParameterProxy with service clients                |
| `src/play_launch/src/ros/mod.rs`                                | Add `pub mod parameter_types; pub mod parameter_proxy;`      |
| `src/play_launch/src/member_actor/coordinator/handle.rs`        | Add `shared_ros_node`, `node_fqn_map`, new methods           |
| `src/play_launch/src/member_actor/coordinator/builder.rs`       | Pass `shared_ros_node` to MemberHandle, build `node_fqn_map` |
| `src/play_launch/src/member_actor/coordinator/mod.rs`           | Update `node_fqn_map` on `LoadSucceeded`                     |
| `src/play_launch/src/member_actor/events.rs`                    | Add `ParameterChanged` variant                               |
| `src/play_launch/src/web/handlers.rs`                           | Add `get_node_parameters`, `set_node_parameter`              |
| `src/play_launch/src/web/mod.rs`                                | Register two new routes                                      |
| `src/play_launch/src/web/web_types.rs`                          | Add `SetParamRequest`                                        |
| `src/play_launch/src/commands/replay.rs`                        | Spawn `/parameter_events` subscriber                         |
| `src/play_launch/src/web/assets/js/components/ParametersTab.js` | **New** — tab component                                      |
| `src/play_launch/src/web/assets/js/components/RightPanel.js`    | Add Params tab                                               |
| `src/play_launch/src/web/assets/js/store.js`                    | Handle `parameter_changed` events                            |
| `src/play_launch/src/web/assets/css/panels.css`                 | Parameter table styles                                       |

---

## Verification

- `just build-rust` compiles
- Manual test: `play_launch replay` with a node declaring parameters → Params tab
  lists parameters, editing works, read-only params are disabled
- SSE: `ros2 param set` from CLI → Params tab updates live
- Composable: select a composable node → see both node + container parameters
- State: stop a node → Params tab shows "Node not running"
- `just test` — existing 353 tests still pass
