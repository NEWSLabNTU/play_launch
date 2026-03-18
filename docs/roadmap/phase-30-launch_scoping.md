# Phase 30: Launch Tree Scoping

**Status**: In progress (30.1–30.2 done)
**Priority**: High (Foundation for Phase 31 + debugging tool)
**Dependencies**: None (uses existing parser infrastructure)

---

## Overview

Refactor the parser to produce a **scoped record** where each node is
tagged with its position in the launch include tree. Shared context
(namespace, env, global params) lives once in a scope table, not duplicated
per node. Adapt the executor to read the new format without breaking
existing record.json files.

Design docs:
- `docs/design/record-format.md`
- `docs/design/parser-context.md`
- `docs/design/launch-context-tool.md`

## Scope Table Design

Nodes in the same launch file share most of their context. Only
node-specific fields (name, package, executable, params, remaps) differ.
The scope table captures shared context once:

```json
{
  "scopes": [
    {
      "id": 0,
      "pkg": "autoware_launch",
      "file": "planning_simulator.launch.xml",
      "ns": "",
      "args": {},
      "env": {},
      "global_params": [],
      "parent": null
    },
    {
      "id": 1,
      "pkg": "tier4_sensing_launch",
      "file": "sensing.launch.xml",
      "ns": "/sensing",
      "args": { "vehicle_model": "sample_vehicle" },
      "env": {},
      "global_params": [["use_sim_time", "true"]],
      "parent": 0
    }
  ],
  "node": [
    {
      "scope": 1,
      "name": "driver",
      "package": "lidar_driver",
      "executable": "lidar_driver_node",
      "params": [["fps", "10"]],
      "cmd": ["..."]
    }
  ]
}
```

**Properties:**
- `parent` references form the launch tree (no separate structure needed)
- `(pkg, file, ns, args)` is unique — duplicates are a parse error
  (duplicate invocations produce duplicate nodes, always a bug)
- Each scope stores only delta context (what's added at that level)
- Full context reconstructed by walking the parent chain

**What moves to scope** (shared): namespace, env, global_params, args
**What stays on node** (per-node): name, package, executable, params,
params_files, remaps, cmd, ros_args, respawn

## Work Items

### 30.1: Scope tracking in parser — done

- [x] Define `ScopeEntry` struct (id, pkg, file, ns, args, parent) with serde
- [x] Define `ScopeTable` helper (push, get, len, is_empty, into_entries)
- [x] Define `extract_package_from_path()` helper (ament `/share/<pkg>/` pattern)
- [x] Add `scope_table: ScopeTable` and `current_scope_id: usize` to `LaunchTraverser`
- [x] Create root scope in `traverse_file()` on first invocation
- [x] Push scope at XML includes (`include.rs`) — transfer table to child, take back
- [x] Push scope at XML-from-Python includes (`xml_include.rs`)
- [x] Stamp `scope_id` on XML nodes, executables, containers, load_nodes (`entity.rs`)
- [x] Stamp `scope_id` on Python captures during `into_record_json()` (`record_conv.rs`)
- [x] Output scope table in `RecordJson.scopes` field
- [x] Unit tests: ScopeTable operations, extract_package_from_path (5 tests)
- [x] Integration tests: scope tracking with simple launch and includes (2 tests)
- [x] Backward compat tests: old JSON without scopes, node scope optional (2 tests)
- [ ] Push scope at Python file execution (`python_exec.rs`) — not yet wired
- [ ] Detect duplicate `(pkg, file, ns, args)` — emit parse warning

### 30.2: Record format extension — done

- [x] Add `scopes: Vec<ScopeEntry>` to parser's `RecordJson` (skip_serializing_if empty)
- [x] Add `scope: Option<usize>` to `NodeRecord`,
  `ComposableNodeContainerRecord`, `LoadNodeRecord` (skip_serializing_if None)
- [x] `#[serde(default, skip_serializing_if)]` on all new fields
- [x] Test: old record.json (no scopes) still deserializes correctly
- [x] Test: new record.json round-trips with scope data
- [ ] Verify Autoware record.json gains correct scope table

### 30.3: Executor adaptation

- [ ] Add `scopes` and `scope` fields to executor's `LaunchDump` types
- [ ] Build scope chain resolver: `resolve_scope(id) → AccumulatedContext`
- [ ] During replay setup, merge scope context into node records
- [ ] Fallback: if `scopes` absent, use existing per-node fields
- [ ] Test: old record.json still replays correctly
- [ ] Test: new scoped record.json produces identical replay behavior
- [ ] Integration test: parse with scoping → replay → verify node startup

### 30.4: Context extraction tool

- [ ] Add `context` subcommand to CLI
- [ ] `--node <FQN>` — extract full context by walking scope chain
- [ ] `--launch <pkg> <file> [--namespace <ns>]` — extract parent-provided
  context for an inner launch file
- [ ] `--all` — list all invocations of a launch file
- [ ] Output: YAML with accumulated launch context
- [ ] Test: extract context for Autoware nodes, verify correctness

## Verification

```bash
just build-rust
just test
just test-all

# Context extraction
play_launch context autoware_launch planning_simulator.launch.xml \
    --node /sensing/camera/front/driver

play_launch context autoware_launch planning_simulator.launch.xml \
    --launch camera_driver camera.launch.xml --all
```

## Key Design Decisions

1. **Scope table IS the launch tree** — parent references form the tree.
   No separate structure.

2. **Delta context per scope** — each scope stores only what's added.
   Full context from walking the chain. Matches launch file semantics.

3. **Duplicate detection** — `(pkg, file, ns, args)` must be unique.
   Duplicate invocations are always a bug (produces duplicate node FQNs).

4. **Backward compat** — all new fields optional with `skip_serializing_if`.
   Old executor ignores new fields. New executor handles old format.
