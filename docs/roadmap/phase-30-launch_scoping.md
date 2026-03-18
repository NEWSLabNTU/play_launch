# Phase 30: Launch Tree Scoping

**Status**: In progress (30.1–30.5 done)
**Priority**: High (Foundation for Phase 31 + debugging tool)
**Dependencies**: None (uses existing parser infrastructure)

---

## Overview

Both parsers (Rust and Python) produce a **scoped record** where each node
is tagged with its position in the launch include tree. The scope table
enables manifest association, context extraction, and provenance tracking.

Design docs:
- `docs/design/record-format.md`
- `docs/design/parser-context.md`
- `docs/design/launch-context-tool.md`

## Scope Table Design

Nodes in the same launch file share most of their context. The scope table
captures the launch include tree with one entry per invocation:

```json
{
  "scopes": [
    {
      "id": 0,
      "pkg": "autoware_launch",
      "file": "planning_simulator.launch.xml",
      "ns": "/",
      "args": {},
      "parent": null
    },
    {
      "id": 1,
      "pkg": "tier4_sensing_launch",
      "file": "sensing.launch.xml",
      "ns": "/sensing",
      "args": { "vehicle_model": "sample_vehicle" },
      "parent": 0
    }
  ],
  "node": [
    { "scope": 1, "name": "driver", "package": "lidar_driver", ... }
  ]
}
```

**Properties:**
- `parent` references form the launch tree (no separate structure needed)
- `(pkg, file, ns, args)` is unique — duplicates are a parse error
- Scopes are informational metadata. Node execution uses the resolved
  values on `NodeRecord` (cmd, params, remaps) which are unchanged.

## Work Items

### 30.1: Scope tracking in Rust parser — done

- [x] Define `ScopeEntry`, `ScopeTable`, `extract_package_from_path` in `record/types.rs`
- [x] Add `scope_table` and `current_scope_id` to `LaunchTraverser`
- [x] Create root scope in `traverse_file()` on first invocation
- [x] Push scope at XML includes (`include.rs`) — transfer table to child, take back
- [x] Push scope at XML-from-Python includes (`xml_include.rs`)
- [x] Push scope at Python includes (`include.rs` Python path)
- [x] Push scope at YAML includes (`include.rs` YAML path)
- [x] Add `scope_id: Option<usize>` to `NodeCapture`, `ContainerCapture`, `LoadNodeCapture`
- [x] Stamp `scope_id` on captures when merging from child traversers (`include.rs`, `xml_include.rs`)
- [x] Stamp `scope_id` on captures created during Python execution (count-before/after pattern)
- [x] Stamp `scope_id` on XML records in `entity.rs` (nodes, executables, containers, load_nodes)
- [x] Use `capture.scope_id` in `record_conv.rs` when converting captures to records
- [x] Unit tests: ScopeTable, extract_package_from_path, serialization, backward compat (9 tests)
- [x] Autoware validation: 79 scopes, 119/119 entities with correct non-root scope IDs

### 30.2: Record format extension — done

- [x] Add `scopes: Vec<ScopeEntry>` to parser's `RecordJson`
- [x] Add `scope: Option<usize>` to `NodeRecord`, `ComposableNodeContainerRecord`, `LoadNodeRecord`
- [x] `#[serde(default, skip_serializing_if)]` on all new fields

### 30.3: Executor adaptation — done

Scopes are informational. No execution impact.

- [x] Add `ScopeEntry`, `scopes`, `scope` fields to executor's `LaunchDump` types
- [x] Fix construction sites (`run.rs`, `context.rs`, `node_cmdline.rs`)

### 30.4: Scope tracking in Python parser

The Python parser (`python/play_launch/dump/`) also needs scope tracking so
we can cross-validate results between parsers. The Python parser uses the
ROS 2 `launch` framework's visitor pattern to traverse the launch tree.

**Architecture:**

```
LaunchInspector
  └── visit_entity() / visitor dispatch
        ├── visit_node()                  → stamp scope_id on NodeRecord
        ├── visit_composable_node_container() → stamp scope_id
        ├── visit_load_composable_nodes() → stamp scope_id
        └── visit_include_launch_description()
              → push new scope entry (pkg, file, ns, args)
              → recurse into included launch description
              → pop scope
```

**Key files:**

| File                                                            | Change                                                                                   |
|-----------------------------------------------------------------|------------------------------------------------------------------------------------------|
| `python/play_launch/dump/launch_dump.py`                        | Add `ScopeEntry` dataclass, `scope` field to record types, `scopes` list to `LaunchDump` |
| `python/play_launch/dump/inspector.py`                          | Add scope stack + scope table to `LaunchInspector`, pass through visitor                 |
| `python/play_launch/dump/visitor/include_launch_description.py` | Push/pop scope on include processing                                                     |
| `python/play_launch/dump/visitor/node.py`                       | Stamp `scope_id` from current scope                                                      |
| `python/play_launch/dump/visitor/composable_node_container.py`  | Stamp `scope_id`                                                                         |
| `python/play_launch/dump/visitor/load_composable_nodes.py`      | Stamp `scope_id`                                                                         |
| `python/play_launch/dump/__main__.py`                           | Serialize `scopes` in output JSON                                                        |

**Work items — done:**

- [x] Add `ScopeEntry` dataclass + `extract_package_from_path()` to `launch_dump.py`
- [x] Add `scope: int | None = None` to all record types
- [x] Add `scopes: list[ScopeEntry]` and scope stack to `LaunchDump`
  with `push_scope()`, `pop_scope()`, `current_scope_id` property
- [x] Push scope in `visit_include_launch_description`, pop in `visit_entity`
  after sub-entity recursion completes
- [x] Stamp `scope_id` in `visit_node`, `visit_composable_node_container`,
  `visit_load_composable_nodes`
- [x] Serialization via `dataclasses.asdict()` — `_scope_stack` excluded
- [x] Autoware validation: 83 scopes, 119/119 entities scoped, 0 at root

### 30.5: Cross-parser scope comparison tests

Compare Rust and Python parser scope outputs for the same launch files.
The scope tables should match: same number of scopes, same parent
structure, same (pkg, file, ns) tuples. Per-node scope assignments should
agree.

**Comparison criteria:**

| Field                        | Must match       | Notes                                             |
|------------------------------|------------------|---------------------------------------------------|
| Number of scopes             | Exact            | Both parsers see the same include tree            |
| Scope (pkg, file, ns) tuples | Exact            | Same launch files, same namespaces                |
| Scope parent chain           | Exact            | Same tree structure                               |
| Per-node scope_id            | Same (pkg, file) | IDs may differ but must point to same scope entry |
| Scope args                   | Best effort      | Key-value pairs may differ in resolution order    |

**Work items — done:**

- [x] `scripts/compare_scopes.py`: compares scope count, (pkg, file)
  counts, scope identities with parents, entity counts, per-entity
  scope assignments
- [x] `just compare-scopes <pkg> <launch> [args...]` recipe
- [x] Validated on `simple_test/pure_nodes.launch.xml` — 1 scope, 2 nodes, PASS
- [x] Validated on `simple_test/all.launch.xml` — 1 scope, 6+1+2 entities, PASS
- [x] Validated on Autoware planning_simulator — 83 scopes, 119 entities, PASS
- [x] Fixed Python-to-Python include scope tracking in `python_exec.rs`
  (was missing scope push for `.py` and `.yaml` includes from Python)

### 30.6: Context extraction tool

- [ ] Add `context` subcommand to CLI
- [ ] `--node <FQN>` — show node's resolved values + scope provenance
- [ ] `--launch <pkg> <file> [--namespace <ns>]` — show include context
- [ ] `--all` — list all invocations of a launch file
- [ ] Output: YAML
- [ ] Test: extract context for Autoware nodes

## Verification

```bash
# Rust parser tests
just build-rust
cargo test -p play_launch_parser --test scope_tests
just test

# Python parser scope output
play_launch dump launch demo_nodes_cpp talker_listener.launch.xml \
    --parser python -o /tmp/python_record.json
python3 -c "import json; d=json.load(open('/tmp/python_record.json')); print(len(d.get('scopes',[])),'scopes')"

# Cross-parser comparison
python3 scripts/compare_scopes.py \
    tests/fixtures/simple_test/ \
    --rust-record /tmp/rust_record.json \
    --python-record /tmp/python_record.json

# Autoware comparison
just compare-scopes-autoware

# Context extraction
play_launch context autoware_launch planning_simulator.launch.xml \
    --node /perception/lidar/centerpoint
```

## Key Design Decisions

1. **Scope table IS the launch tree** — parent references form the tree.

2. **Scopes are informational** — node execution uses resolved values on
   `NodeRecord`. Scopes track provenance (which launch file), not
   execution context.

3. **Both parsers produce scopes** — cross-validation ensures correctness.
   The Python parser uses the ROS 2 launch framework's own include
   tracking, which is the ground truth.

4. **Comparison by (pkg, file, ns)** — scope IDs may differ between
   parsers. Comparison normalizes by the scope's identity tuple.
