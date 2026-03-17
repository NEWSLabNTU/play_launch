# Phase 30: Topic Manifest

**Status**: Planned
**Priority**: High (System Observability / Graph Validation)
**Dependencies**: Phase 25 (Topic Introspection — `GraphSnapshot`), Phase 29 (RCL Interception — latency data)

---

## Overview

Add a manifest system that describes the expected communication graph
(topics, services, actions) alongside launch files. The parser loads
manifests during launch file processing and embeds the expected graph into
`record.json`. The executor audits the runtime graph against it.

Design doc: `docs/design/topic-manifest.md`

### Why this matters

- ROS 2 launch files declare nodes but not topics — the actual graph is
  invisible until runtime
- Autoware has ~500 topics across ~110 nodes; no source of truth for what
  should be connected to what
- QoS mismatches between publishers and subscribers are only caught at
  runtime (or silently fail)
- No way to detect unintended topic additions/removals between releases

### Architecture

```
                           --manifest-dir
                                │
  Launch files ──→ Parser ──────┤──→ record.json
                      │         │      ├── node, container, load_node  (existing)
                      │         │      ├── topics: { ... }             (new)
                      │         │      ├── services: { ... }           (new)
                      │         │      └── manifest_sync: { ... }      (new)
                      │         │
                      └── Manifest files (per package/launch_file)
                            <manifest_dir>/<package>/<launch_stem>.yaml

  Executor
    ├── record.json → spawn nodes                        (existing)
    └── record.json.topics + GraphSnapshot → audit       (new)
```

## Crate: `play_launch_manifest`

New crate for manifest format types, parsing, and graph construction.
Located at `src/play_launch_manifest/`, added to workspace members.

### Why a separate crate

- Manifest types are shared between the parser (writes to record.json)
  and the executor (reads from record.json and audits)
- Clean dependency: both `play_launch_parser` and `play_launch` depend
  on `play_launch_manifest`, but not on each other
- Manifest format can be versioned independently
- Enables future standalone tooling (validate, diff, visualize manifests)

### Crate structure

```
src/play_launch_manifest/
├── Cargo.toml
└── src/
    ├── lib.rs
    ├── types.rs           # Manifest YAML types (serde)
    ├── graph.rs           # ExpectedGraph construction + resolution
    ├── resolve.rs         # Namespace resolution for relative names
    ├── accumulate.rs      # Interface accumulation logic
    ├── audit.rs           # Diff: ExpectedGraph vs GraphSnapshot
    └── qos.rs             # QoS compatibility checks
```

### Key types

```rust
/// A single manifest file (one per launch file)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Manifest {
    #[serde(default)]
    pub exclude_patterns: Vec<String>,
    #[serde(default)]
    pub topics: IndexMap<String, TopicDecl>,
    #[serde(default)]
    pub services: IndexMap<String, ServiceDecl>,
    #[serde(default)]
    pub actions: IndexMap<String, ActionDecl>,
    #[serde(default)]
    pub nodes: IndexMap<String, NodeDecl>,
    #[serde(default)]
    pub components: IndexMap<String, ComponentDecl>,
    #[serde(default)]
    pub interface: InterfaceDecl,
    #[serde(default)]
    pub sync: IndexMap<String, SyncPolicy>,
    #[serde(default)]
    pub requirements: Option<Requirements>,
}

/// Topic declaration — shorthand or full form
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum TopicDecl {
    /// Shorthand: just message type, default QoS
    Short(String),
    /// Full form: type + explicit QoS
    Full { r#type: String, qos: Option<QosDecl> },
}

/// Node declaration — lists of topic/service/action names
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NodeDecl {
    #[serde(default)]
    pub pub_: Vec<String>,       // "pub" is a keyword, use pub_
    #[serde(default)]
    pub sub: Vec<String>,
    #[serde(default)]
    pub srv: Vec<String>,
    #[serde(default)]
    pub cli: Vec<String>,
    #[serde(default)]
    pub action_server: Vec<String>,
    #[serde(default)]
    pub action_client: Vec<String>,
}

/// Component — inline or included
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComponentDecl {
    pub namespace: Option<String>,
    pub include: Option<IncludeRef>,
    // Inline fields (same as Manifest minus metadata)
    #[serde(default)]
    pub topics: IndexMap<String, TopicDecl>,
    #[serde(default)]
    pub nodes: IndexMap<String, NodeDecl>,
    #[serde(default)]
    pub components: IndexMap<String, ComponentDecl>,
    #[serde(default)]
    pub interface: InterfaceDecl,
    #[serde(default)]
    pub sync: IndexMap<String, SyncPolicy>,
    #[serde(default)]
    pub requirements: Option<Requirements>,
}

/// The resolved expected graph (all names absolute)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExpectedGraph {
    pub topics: IndexMap<String, ResolvedTopic>,
    pub services: IndexMap<String, ResolvedService>,
    pub actions: IndexMap<String, ResolvedAction>,
    pub sync: IndexMap<String, SyncPolicy>,
    pub requirements: IndexMap<String, Requirements>,
}

/// A resolved topic with absolute name
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResolvedTopic {
    pub r#type: String,
    pub qos: Option<QosDecl>,
    pub publishers: Vec<String>,    // node FQNs
    pub subscribers: Vec<String>,   // node FQNs
}
```

Note: `pub` is a Rust keyword. The YAML key `pub:` maps to a struct field
via `#[serde(rename = "pub")]` on the field or a custom deserializer.

### Dependencies

```toml
[dependencies]
serde = { version = "1", features = ["derive"] }
serde_yaml = "0.9"
serde_json = "1"
indexmap = { version = "2", features = ["serde"] }
```

## record.json Extensions

New optional fields added to `RecordJson` (parser crate) and `LaunchDump`
(executor crate). Existing fields unchanged — backward compatible.

```rust
// In play_launch_parser::record::types::RecordJson
#[serde(default, skip_serializing_if = "Option::is_none")]
pub expected_graph: Option<play_launch_manifest::ExpectedGraph>,

// In play_launch::ros::launch_dump::LaunchDump
#[serde(default, skip_serializing_if = "Option::is_none")]
pub expected_graph: Option<play_launch_manifest::ExpectedGraph>,
```

The executor checks `launch_dump.expected_graph.is_some()` to decide
whether to enable manifest auditing.

## Parser Integration

When `--manifest-dir` is provided, the parser loads manifest files during
include processing:

1. `traverse_entity()` encounters `<include pkg="X" file="Y.launch.xml">`
2. Existing: processes the launch file → node records
3. New: calls `manifest_loader.load("X", "Y")` → `Option<Manifest>`
4. If found: resolves names using current namespace, merges into
   `ExpectedGraphBuilder`
5. After all includes: `ExpectedGraphBuilder::build()` → `ExpectedGraph`
6. Serializes `ExpectedGraph` into `record.json`

```rust
// New field on the traverser / parser context
pub struct ManifestLoader {
    manifest_dir: PathBuf,
    cache: HashMap<(String, String), Option<Manifest>>,
}

impl ManifestLoader {
    pub fn load(&mut self, package: &str, launch_file: &str) -> Option<&Manifest> {
        // Look up <manifest_dir>/<package>/<stem>.yaml
        // Cache result (including misses)
    }
}
```

## Executor Integration

In `replay.rs`, after loading `record.json`:

```rust
if let Some(expected) = &launch_dump.expected_graph {
    let auditor = ManifestAuditor::new(expected.clone());
    // Pass auditor to the coordinator
    // Coordinator runs periodic audit via GraphSnapshot
}
```

The auditor runs after the stabilization period (existing graph snapshot
timer) and produces deviations written to
`play_log/<ts>/manifest_audit.json`.

See `docs/design/rcl-interception.md` for the graph discovery evolution
plan (polling → interception-based → blocking enforcement).

## Work Items

### 30.1: Manifest crate + types

- [ ] Create `src/play_launch_manifest/Cargo.toml` with serde deps
- [ ] Implement `types.rs`: `Manifest`, `TopicDecl`, `NodeDecl`,
  `ComponentDecl`, `InterfaceDecl`, `SyncPolicy`, `Requirements`,
  `QosDecl`, `IncludeRef`
- [ ] Handle YAML shorthand deserialization (`TopicDecl::Short` vs `Full`)
- [ ] Handle `pub` keyword: `#[serde(rename = "pub")]` for `NodeDecl.pub_`
- [ ] Implement `Manifest::from_yaml(path)` and `Manifest::to_yaml()`
- [ ] Unit tests: round-trip serialize/deserialize for all types
- [ ] Unit tests: shorthand topic parsing (`name: type` vs full form)
- [ ] Add crate to workspace `Cargo.toml`

### 30.2: Namespace resolution + graph construction

- [ ] Implement `resolve.rs`: `resolve_name(name, namespace) -> String`
  (relative → absolute, absolute → pass through)
- [ ] Implement `graph.rs`: `ExpectedGraphBuilder`
  - `add_manifest(manifest, namespace, package, launch_file)`
  - `build() -> ExpectedGraph`
- [ ] Resolution: apply namespace to all relative topic/node names
- [ ] Topic name matching: group publishers and subscribers by resolved name
- [ ] Type consistency check: all endpoints on same topic must have same type
- [ ] Unit tests: namespace resolution (relative, absolute, nested)
- [ ] Unit tests: graph construction from multiple manifests

### 30.3: Interface accumulation

- [ ] Implement `accumulate.rs`: given a component's children and their
  interfaces, compute the accumulated interface
- [ ] Rule: child publish not subscribed by sibling → accumulates
- [ ] Rule: child subscribe not published by sibling → accumulates
- [ ] Rule: absolute topics pass through unchanged
- [ ] Rule: explicit parent interface takes precedence
- [ ] Unit tests: accumulation with siblings, absolute topics, nesting

### 30.4: QoS compatibility

- [ ] Implement `qos.rs`: `check_compatibility(pub_qos, sub_qos) -> Vec<Issue>`
- [ ] Reliability: best_effort pub + reliable sub = incompatible
- [ ] Durability: volatile pub + transient_local sub = incompatible
- [ ] Unit tests: all QoS compatibility combinations

### 30.5: record.json integration

- [ ] Add `expected_graph: Option<ExpectedGraph>` to parser's `RecordJson`
- [ ] Add `expected_graph: Option<ExpectedGraph>` to executor's `LaunchDump`
- [ ] `#[serde(default, skip_serializing_if = "Option::is_none")]` for
  backward compat
- [ ] Test: old record.json (no expected_graph) deserializes correctly
- [ ] Test: new record.json round-trips with expected_graph

### 30.6: Parser manifest loading

- [ ] Add `--manifest-dir <path>` CLI flag to `DumpArgs` and `LaunchArgs`
- [ ] Implement `ManifestLoader` (dir lookup + cache)
- [ ] Integrate into parser's include processing: load manifest at each
  `<include>` when manifest_dir is set
- [ ] Build `ExpectedGraph` from loaded manifests
- [ ] Serialize into record.json
- [ ] Integration test: parse a launch file with `--manifest-dir`, verify
  record.json contains expected_graph

### 30.7: Executor audit

- [ ] Implement `audit.rs`: `diff(expected: &ExpectedGraph, actual: &GraphSnapshot) -> AuditReport`
- [ ] Deviation types: new topic, missing topic, new endpoint, missing
  endpoint, QoS mismatch, QoS incompatible
- [ ] Integrate into coordinator: periodic audit after stabilization
- [ ] Write `manifest_audit.json` to play_log on shutdown
- [ ] Log deviations via `warn!` / `info!` / `error!`
- [ ] Integration test: launch with manifest, verify audit output

### 30.8: Test fixtures

- [ ] Create `tests/fixtures/manifest_simple/`
  - Simple launch file (2-3 nodes, pub/sub)
  - Matching manifest file
  - Expected: clean audit (no deviations)
- [ ] Create `tests/fixtures/manifest_deviation/`
  - Launch file + manifest that intentionally differ
  - Expected: audit catches new topic, missing endpoint
- [ ] Create `tests/fixtures/manifest_qos/`
  - Launch file + manifest with QoS declarations
  - Expected: QoS compatibility check passes or reports mismatch
- [ ] Create `tests/fixtures/manifest_composable/`
  - Launch file with containers + composable nodes
  - Manifest treats composable nodes as regular nodes
  - Expected: correct node-to-manifest mapping
- [ ] Create `tests/fixtures/manifest_multi_include/`
  - Top-level launch includes sub-launch files
  - Manifest dir with per-launch-file manifests
  - Expected: expected graph correctly merges across includes
- [ ] Parser unit tests for manifest loading and graph construction
- [ ] Integration tests using `ManagedProcess` RAII guard

### 30.9: Capture (save-manifest-dir)

- [ ] Add `--save-manifest-dir <path>` CLI flag
- [ ] Track node → originating launch file mapping in parser
- [ ] After stabilization, partition runtime graph by launch file
- [ ] Strip namespace prefix → relative names
- [ ] Infer interface from unmatched topics
- [ ] Write per-launch-file YAML to manifest dir
- [ ] Integration test: capture → audit round-trip (capture manifests,
  then audit with them — should produce zero deviations)

## Verification

```bash
# Build
just build-rust

# Unit tests (manifest crate)
cargo test -p play_launch_manifest

# Parser integration tests
just test

# Full integration test (requires ROS environment)
just test-all

# Manual smoke test
play_launch launch demo_nodes_cpp talker_listener.launch.xml \
    --manifest-dir tests/fixtures/manifest_simple/manifests
```

## Key Design Decisions

1. **Separate crate**: manifest types shared between parser and executor
   without coupling them. Enables standalone manifest tooling.

2. **Optional field in record.json**: `expected_graph` is
   `Option<ExpectedGraph>` with `skip_serializing_if`. Old record.json
   files work unchanged. Auditing activates only when the field is present.

3. **Per-launch-file manifests**: each manifest describes one launch file.
   The parser handles the include tree and namespace application. Manifests
   are reusable across namespace contexts.

4. **Topics as first-class entities**: type + QoS on the topic, not on
   node endpoints. Connection by name matching (same as ROS 2). No edges.

5. **Shorthand syntax**: `topic_name: msg/Type` for the common case
   (default QoS). Keeps manifests compact — a component with 20 internal
   topics adds 20 one-liners.

6. **Two-phase audit**: parse-time (QoS compatibility, type consistency)
   + runtime (actual vs expected graph). Catches issues before and after
   node startup.

7. **Composable nodes as regular nodes**: the container relationship is a
   deployment detail. The manifest cares about the topic graph, where
   composable nodes are indistinguishable from regular nodes.

8. **`--manifest-dir` CLI flag**: explicit, user-specified. No automatic
   resolution (sidecar, central store) in v1. Simplifies implementation
   and avoids implicit behavior.
