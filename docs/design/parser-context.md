# Parser Launch Context

## Overview

`LaunchContext` is the central state object during launch file parsing. It
holds configurations (arguments, variables), namespace state, remappings,
global parameters, and entity captures. It is used by both XML and Python
parsers.

Source: `src/play_launch_parser/.../substitution/context.rs`

## Architecture

```
LaunchContext
├── Scope chain (Arc + Local pattern)
│   ├── parent: Option<Arc<ParentScope>>     ← shared, immutable
│   └── local_*: HashMap/Vec                  ← owned, mutable
│
├── File state
│   ├── current_file: Option<PathBuf>
│   └── namespace_stack: Vec<String>
│
└── Entity captures (always local, not inherited)
    ├── captured_nodes: Vec<NodeCapture>
    ├── captured_containers: Vec<ContainerCapture>
    ├── captured_load_nodes: Vec<LoadNodeCapture>
    └── captured_includes: Vec<IncludeCapture>
```

## Scope Chain (Hybrid Arc + Local)

Context uses a **parent chain** pattern for efficient scope management.
When processing an `<include>`, the current context is frozen into an
`Arc<ParentScope>` and a new child context starts with empty local state.
Variable lookups walk the chain: local first, then parent, then
grandparent.

```
Root context:
  local: { vehicle_model: "sample_vehicle" }
  parent: None

  └─ Include sensing.launch.xml → child():
     local: { lidar_model: "velodyne" }
     parent: Arc { vehicle_model: "sample_vehicle" }

     └─ Include camera.launch.xml → child():
        local: { camera_id: "0" }
        parent: Arc { lidar_model: "velodyne",
                      parent: Arc { vehicle_model: "sample_vehicle" } }
```

`child()` is O(1) — it creates an `Arc` from the local state and starts
fresh. No deep cloning of the full context.

### Scope operations

| Method | What it does |
|--------|-------------|
| `child()` | Freeze local → Arc parent, return new empty-local context |
| `save_scope()` | Snapshot namespace depth + remap count for `<group>` |
| `restore_scope()` | Pop namespace/remaps back to snapshot |
| `set_configuration(name, value)` | Set variable in local scope |
| `resolve_configuration(name)` | Walk chain: local → parent → grandparent |

### Group vs Include scoping

| Construct | Scoping mechanism | Variable visibility |
|-----------|------------------|---------------------|
| `<include>` | `child()` — new context, parent chain | Child's variables invisible to parent |
| `<group>` | `save_scope()` / `restore_scope()` — same context | Variables leak to parent (by design) |
| YAML include | Direct processing on same context | Variables visible to subsequent actions |

YAML includes modify the parent context (not a child), which is how
Autoware's preset files work. See `docs/include_scoping_behavior.md`.

## Namespace Stack

Namespaces accumulate via `push_namespace()` / `pop_namespace()`:

```
Initial:        ["/"]
push("sensing"): ["/", "sensing"]         → /sensing
push("lidar"):   ["/", "sensing", "lidar"] → /sensing/lidar
pop():           ["/", "sensing"]         → /sensing
```

`current_namespace()` joins the stack: `"/" + "sensing" + "/" + "lidar"`.

Absolute namespaces (leading `/`) replace the stack.

## Entity Captures

During traversal, the parser captures entities into the context. Captures
are **always local** — a child context starts with empty capture lists.
After processing an include, the parent merges the child's captures.

| Capture type | Created by | Key fields |
|-------------|-----------|------------|
| `NodeCapture` | `<node>`, Python `Node()` | package, executable, name, namespace, params, remaps |
| `ContainerCapture` | `<node_container>`, Python `ComposableNodeContainer()` | name, namespace, package, executable |
| `LoadNodeCapture` | `<load_composable_node>`, Python `LoadComposableNode()` | plugin, target_container, node_name, namespace, params |
| `IncludeCapture` | `<include>`, Python `IncludeLaunchDescription()` | file_path, args, ros_namespace |

Captures are converted to `record.json` records via `to_record()`
implementations in `traverser/record_conv.rs`.

## Substitution Resolution

Variables are stored as parsed `Vec<Substitution>`, not as plain strings.
This enables lazy resolution — `$(var name)` inside a value is resolved
when the value is read, not when it's set.

```
set_configuration("topic", "$(var prefix)/pointcloud")
  → stores: [Var("prefix"), Text("/pointcloud")]

resolve_configuration("topic")
  → resolves Var("prefix") → "sensing"
  → returns: "sensing/pointcloud"
```

Resolution has a depth limit (20) to prevent infinite recursion.

## Planned Extensions

### Scope tracking (Phase 30)

Each capture gains a `scope: Vec<ScopeEntry>` recording the include chain
from root to the current launch file. See `docs/design/launch-context-tool.md`.

### Manifest loading (Phase 30)

A `ManifestLoader` alongside the context, queried at each `<include>` to
load the corresponding manifest file. See `docs/design/launch-manifest.md`.
