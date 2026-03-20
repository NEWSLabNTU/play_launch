# Phase 30b: Group Scopes

**Status**: Planned
**Priority**: Medium (fixes B2, improves launch tree fidelity)
**Dependencies**: Phase 30 (Launch Tree Scoping — complete)

---

## Overview

Extend the scope table to include `<group>` blocks as anonymous scopes
alongside file scopes. This faithfully represents the launch file's
lexical structure and fixes B2 (group namespace not reflected in child
include scopes).

## Motivation

Currently, only `<include>` boundaries create scope entries. A `<group>`
that pushes a namespace wrapping an `<include>` is invisible in the scope
table — the child include's scope records `ns=/` instead of `ns=/sensing`.

With group scopes:
- The child include's parent is the group scope (correct namespace)
- The launch tree shows the full nesting structure
- Context extraction shows per-group namespace/env changes

## Design

### ScopeEntry changes

```rust
/// Origin of a scope — identifies the launch file.
/// None for group scopes (anonymous, no file).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScopeOrigin {
    pub pkg: Option<String>,
    pub file: String,
}

pub struct ScopeEntry {
    pub id: usize,
    /// None for group scopes, Some for file scopes.
    pub origin: Option<ScopeOrigin>,
    pub ns: String,
    pub args: HashMap<String, String>,
    pub parent: Option<usize>,
}
```

### JSON format

```json
{
  "scopes": [
    {
      "id": 0,
      "origin": { "pkg": "autoware_launch", "file": "planning_simulator.launch.xml" },
      "ns": "/",
      "parent": null
    },
    {
      "id": 1,
      "origin": null,
      "ns": "/sensing",
      "parent": 0
    },
    {
      "id": 2,
      "origin": { "pkg": "tier4_sensing_launch", "file": "sensing.launch.xml" },
      "ns": "/sensing",
      "parent": 1
    }
  ]
}
```

`origin: null` = group scope. `origin: { pkg, file }` = file scope.
Backward compatible: old scopes with flat `pkg`/`file` fields need
migration (one-time).

### Scope table growth

Autoware has ~83 file scopes. Groups with namespace add maybe 10-30
more entries (groups without namespace don't need scopes — they don't
change the context meaningfully). Estimate: 83 → ~110 total.

Groups **without** namespace/env changes are NOT recorded as scopes.
Only groups that modify the context (push namespace, set env, set
global params) create scope entries.

## Work Items

### 30b.1: ScopeEntry refactor

- [ ] Define `ScopeOrigin` struct with `pkg: Option<String>`, `file: String`
- [ ] Change `ScopeEntry` fields: replace `pkg: Option<String>` + `file: String`
  with `origin: Option<ScopeOrigin>`
- [ ] Update `ScopeTable::push()` to accept `Option<ScopeOrigin>`
- [ ] Add `ScopeTable::push_group(ns, parent)` convenience method
- [ ] Update `ScopeEntry` serde: `origin` serializes as null or object
- [ ] Update `extract_package_from_path` callers to produce `ScopeOrigin`
- [ ] Migrate executor's `ScopeEntry` (`launch_dump.rs`) to match
- [ ] Migrate Python parser's `ScopeEntry` (`launch_dump.py`) to match
- [ ] Update `compare_scopes.py` to handle `origin` field
- [ ] Update `context.rs` CLI to handle `origin` field
- [ ] Backward compat: old record.json with flat `pkg`/`file` fields
  should still deserialize (custom deserializer or migration note)
- [ ] Update tests for new ScopeEntry format

### 30b.2: Push group scopes in parser

- [ ] In `entity.rs` `"group"` handler: if group has namespace, push
  a group scope before traversing children, pop after
- [ ] Group scope: `origin: None`, `ns` = namespace after push,
  `parent` = current scope ID
- [ ] Only push scope for groups that modify context (have namespace
  attribute or contain `<push-ros-namespace>`)
- [ ] In Python parser: push group scope in visitor when group has
  namespace
- [ ] Update scope tests: verify group scopes appear in scope table
- [ ] Test B2 fix: `<group namespace="sensing">` wrapping `<include>`
  now has correct child scope `ns`

### 30b.3: Update record.json and executor

- [ ] Executor `ScopeEntry` matches new format
- [ ] `replay.rs` node_scope_map handles group scopes (groups have no
  nodes directly — only file scopes have nodes)
- [ ] Web API `/api/launch-tree` returns group scopes
- [ ] Verify Autoware record.json with group scopes

### 30b.4: Web UI Launch page

- [ ] `LaunchTreeView.js`: render group scopes differently from file
  scopes
  - File scopes: show package + filename (existing)
  - Group scopes: show namespace only, indented, lighter styling
  - Group scopes don't have expand/collapse (they contain no direct
    entities — entities belong to file scopes)
  - Group scopes act as visual namespace markers in the tree
- [ ] `LaunchPanel.js`: when clicking a group scope, show namespace
  and any env/params set at that level
- [ ] `launch-tree.css`: distinct styling for group scope rows
  (dimmer, no package name, namespace-only display)
- [ ] `context.rs` CLI: `--tree` shows group scopes with distinct
  formatting

### 30b.5: Cross-parser validation

- [ ] Update `compare_scopes.py` to compare group scopes
- [ ] Run cross-parser comparison on Autoware
- [ ] Verify group scope parity between Rust and Python parsers

## Key Design Decisions

1. **`origin: Option<ScopeOrigin>`**: clean null/object distinction.
   No `kind` field needed — presence of `origin` determines type.

2. **Only context-modifying groups create scopes**: groups that just
   scope conditions (`<group if="...">`) without namespace/env changes
   are not recorded. This avoids scope table bloat.

3. **Group scopes have no direct entities**: nodes belong to file
   scopes. The group scope is a namespace marker between the file
   scope and its children. The node_scope_map still maps to file
   scopes only.

4. **B2 fix is automatic**: once group scopes exist, the child
   include's parent is the group scope (which has the correct
   namespace), fixing B2 without any special-case code.

5. **Backward compatible JSON**: `origin: null` vs `origin: {...}`
   is unambiguous. Old records without group scopes still work
   (fewer scopes, same structure).
