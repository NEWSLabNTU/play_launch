# Phase 31: Launch Manifest

**Status**: Planned
**Priority**: High (System Observability / Graph Validation)
**Dependencies**: Phase 30 (Launch Tree Scoping — scope table in record.json), Phase 25 (GraphSnapshot)

---

## Overview

Manifest files describe the expected communication graph (topics, services,
actions, QoS, timing contracts) per launch file. This phase implements the
manifest crate and integrates it with the parser and executor.

Design docs:
- `docs/design/launch-manifest.md` — manifest format
- `docs/design/rcl-interception.md` — graph discovery evolution

## Work Items

### 31.1: Manifest crate — types + parsing

- [ ] Create `src/play_launch_manifest/Cargo.toml` (serde, serde_yaml,
  indexmap)
- [ ] `types.rs`: `Manifest`, `TopicDecl` (shorthand + full), `NodeDecl`,
  `ComponentDecl`, `InterfaceDecl`, `SyncPolicy`, `Requirements`, `QosDecl`
- [ ] Handle `pub` keyword: `#[serde(rename = "pub")]`
- [ ] Handle shorthand: `#[serde(untagged)]` for `TopicDecl`
- [ ] `Manifest::from_yaml(path)` and `Manifest::to_yaml()`
- [ ] Unit tests: round-trip, shorthand, edge cases
- [ ] Add crate to workspace

### 31.2: Graph construction + validation

- [ ] `resolve.rs`: namespace resolution (relative → absolute)
- [ ] `graph.rs`: `ExpectedGraphBuilder` → `ExpectedGraph`
- [ ] Topic name matching: group publishers/subscribers by resolved name
- [ ] Type consistency: endpoints on same topic must agree
- [ ] `accumulate.rs`: interface accumulation rules
- [ ] `qos.rs`: QoS compatibility checks (reliability, durability)
- [ ] Unit tests for each module

### 31.3: Test fixtures

- [ ] `tests/fixtures/manifest_simple/` — clean audit
- [ ] `tests/fixtures/manifest_deviation/` — intentional mismatches
- [ ] `tests/fixtures/manifest_qos/` — QoS validation
- [ ] `tests/fixtures/manifest_composable/` — composable nodes
- [ ] `tests/fixtures/manifest_multi_include/` — multi-file manifests

### 31.4: Parser integration (Path A)

- [ ] Add `--manifest-dir <path>` CLI flag
- [ ] `ManifestLoader`: dir lookup by `(pkg, file_stem)` + cache
- [ ] At each `<include>`, load manifest using scope table's `(pkg, file)`
- [ ] Resolve manifest names using scope's namespace
- [ ] Build `ExpectedGraph` incrementally during traversal
- [ ] Parse-time validation: QoS compat, type consistency
- [ ] Embed `ExpectedGraph` in record.json
- [ ] Integration test: parse with manifests, verify output

### 31.5: Post-hoc integration (Path B)

- [ ] Standalone: `play_launch manifest-check record.json --manifest-dir dir/`
- [ ] Join manifests to record via scope table's `(pkg, file)` keys
- [ ] Build `ExpectedGraph` from manifests + scope namespaces
- [ ] Output: audit report (static, no runtime needed)

### 31.6: Executor audit

- [ ] `audit.rs`: `diff(expected, actual) → AuditReport`
- [ ] Deviation types: new/missing topic, new/missing endpoint, QoS mismatch
- [ ] Integrate into coordinator: periodic audit after stabilization
- [ ] Write `manifest_audit.json` to play_log on shutdown
- [ ] Log deviations via `warn!` / `info!` / `error!`
- [ ] Integration test: launch with manifest, verify audit output

### 31.7: Capture

- [ ] `--save-manifest-dir <path>` CLI flag
- [ ] Use scope table to partition runtime graph by launch file
- [ ] Strip namespace prefix (scope provides the prefix)
- [ ] Infer interface from unmatched topics
- [ ] Write per-launch-file YAML to manifest dir
- [ ] Integration test: capture → audit round-trip (zero deviations)

### 31.8: Context extraction + manifest

- [ ] Extend `play_launch context` (Phase 30.4) with `--manifest-dir`
- [ ] Output includes manifest-derived context: expected topics, QoS,
  timing contracts, interface role
- [ ] `run-isolated` subcommand: generate standalone launch + input stubs

## Dependency Graph

```
Phase 30 (scoping)      Phase 31.1-31.3 (manifest crate)
  │                       │
  v                       │
Phase 30.3 (executor)     │     ← can develop in parallel
  │                       │
  └──────────┬────────────┘
             v
Phase 31.4-31.8 (integration)
```

## Verification

```bash
just build-rust
cargo test -p play_launch_manifest
just test
just test-all

play_launch launch demo_nodes_cpp talker_listener.launch.xml \
    --manifest-dir tests/fixtures/manifest_simple/manifests

play_launch manifest-check record.json --manifest-dir manifests/

play_launch context autoware_launch planning_simulator.launch.xml \
    --node /perception/lidar/centerpoint \
    --manifest-dir manifests/
```

## Key Design Decisions

1. **Separate crate**: manifest types shared between parser and executor.
   Enables standalone tooling.

2. **Two integration paths**: Path A (parse-time) for early validation.
   Path B (post-hoc) for standalone tooling. Both use the scope table
   from Phase 30 as the join key.

3. **Scope table is the bridge**: manifests are keyed by `(pkg, file)`.
   The scope table maps `(pkg, file)` → namespace + context. Joining them
   is a table lookup.

4. **Per-launch-file manifests**: reusable across namespace contexts.
   The parser applies namespace from the scope.

5. **Topics as first-class entities**: type + QoS on the topic.
   Connection by name matching (same as ROS 2).

6. **`--manifest-dir` CLI flag**: explicit, no auto-resolution in v1.
