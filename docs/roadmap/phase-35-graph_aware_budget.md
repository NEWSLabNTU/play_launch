# Phase 35: Graph-Aware Budget Checking

Follow-up to Phase 34. Phase 34 delivered the format migration and
cross-scope merge infrastructure but deferred the actual graph traversal
for scope path budget checks. This phase implements topology-aware
checking and closes the remaining migration gaps.

## Current State (after Phase 34)

The checker validates scope paths by:
- Matching paths across scopes by resolved (input, output) topic identity
- Checking `child_budget ≤ parent_budget` for matching paths in the
  ancestor chain (`budget-overflow`)
- Summing all node latencies + topic `max_transport_ms` in a scope as a
  conservative bound (`scope-budget`)

The checker does **not**:
- Build a dataflow graph from merged topics across the scope tree
- Trace actual paths between scope-path input/output topics
- Distinguish series from parallel topology (uses sum for everything)
- Re-run rate hierarchy after cross-scope merge

## Target State

The checker:
- Builds a global dataflow graph from the merged `ManifestIndex`
- For each scope path, finds all node-disjoint sub-paths between the
  resolved input topic(s) and output topic(s) within the scope's subtree
- Computes critical-path latency: sum within a series chain, max across
  parallel branches at fork/join points
- Reports the actual critical path nodes in the diagnostic
- Re-runs rate hierarchy checks on merged topics

## Phases

### 35.1 — Build global dataflow graph from merged ManifestIndex ✅ Done

Built a directed graph from the merged topic/service index:
- [x] New module: `src/play_launch/src/ros/manifest_graph.rs`
- [x] `GlobalDataflowGraph` struct (custom, no petgraph dependency)
- [x] `build_global_graph(&ManifestIndex) -> GlobalDataflowGraph`
- [x] `GlobalNode` carries: scope_id, fqn, paths, subscribers, publishers
- [x] `GlobalEdge` carries: from, to, topic, sub_endpoint, max_transport_ms, is_state
- [x] State edges marked via `is_state` (skipped in causal traversal)
- [x] Lookup tables: `out_edges`, `in_edges`, `topic_publishers`, `topic_subscribers`

**Done when:**
- [x] Graph builds for `manifest_pipeline` fixture (4 nodes, 5 topics)
- [x] Cross-scope publisher/subscriber pairs handled (resolved by FQN merge)
- [x] State edges separated from causal edges
- [x] Tests: `test_global_graph_builds_for_pipeline`

### 35.2 — Scope path bounded subgraph extraction ✅ Done

- [x] `subtree_scope_ids(index, root_scope_id) -> HashSet<usize>`
  computes scope subtree by walking parent links to fixed point
- [x] `ScopeSubgraph<'a>` view over the global graph
- [x] `subgraph_for_scope_path(graph, subtree, input_topics, output_topics)`
  identifies sources (publishers + subscribers of input topics in subtree)
  and sinks (publishers of output topics in subtree)
- [x] `out_causal_edges()` filters state edges and out-of-subtree edges

**Done when:**
- [x] Parent scope path subgraph includes all child-scope nodes
- [x] Child scope path subgraph includes only child's own nodes
- [x] Tests: `test_subtree_scope_ids_includes_descendants`

### 35.3 — Critical-path latency computation ✅ Done

- [x] `topo_sort()`: Kahn's algorithm restricted to subtree, skips state edges
- [x] `critical_path()`: forward DP computing
  `latency[node] = max(predecessor.latency + edge.transport) + node.processing`
- [x] Returns `CriticalPath { total_ms, nodes }` with reconstructed path
- [x] Handles multi-input nodes via max-over-predecessors (parallel join)
- [x] Periodic nodes: when node has empty `input: []`, processing only
- [x] State edges contribute no latency (not in DP)

**Done when:**
- [x] Series pipeline: latency = sum of nodes + transports
- [x] Fork-join: latency = max(branches) + fusion (verified by parallel fixture)
- [x] Critical path nodes reported in diagnostic
- [x] Tests: `test_critical_path_parallel_takes_max_not_sum`,
  `test_critical_path_diagnostic_when_budget_too_tight`

### 35.4 — Replace `scope-budget` sum check with critical path ✅ Done

- [x] Add `check_scope_path_critical_path()` to cross-scope checks in
  `manifest_loader.rs`
- [x] For each scope path, build subgraph and compute critical path
- [x] Emit `scope-budget` warning with critical path nodes when
  declared < computed
- [x] Per-manifest sum check **kept** as a conservative fallback for
  standalone checking; documented as such in `scope_budget.rs`
- [x] Both checks use the same `scope-budget` rule ID; the cross-scope
  one is precise, the per-manifest one is a conservative upper bound

**Done when:**
- [x] Parallel fork-join fixture (`manifest_parallel_pipeline`):
  declared 70ms = critical path (max(50,30)+20) → no warning, even
  though sum (100ms) exceeds budget
- [x] Critical-path diagnostic shows actual node trace
- [x] All 222 tests pass (40 manifest_loader, 182 manifest crates)

### 35.5 — Cross-scope rate hierarchy ✅ Done

- [x] `check_cross_scope_rate_hierarchy()` in `manifest_loader.rs`
- [x] For each merged topic with `rate_hz`:
  - [x] Each publisher endpoint's `min_rate_hz` must be >= `rate_hz`
  - [x] Effective delivery `rate_hz × (1 - max_drop_rate)` must be >=
    each subscriber's `min_rate_hz`
- [x] Skip `state: true` subscribers (no rate guarantee needed)
- [x] Look up endpoint properties via `GlobalNode.publishers/subscribers`
  (the global graph already carries `EndpointProps`)
- [x] Emit `rate-hierarchy` diagnostic on violation

**Refactoring:**
- [x] Build global dataflow graph once in `run_cross_scope_checks()` and
  reuse it for both critical-path (35.4) and rate-hierarchy (35.5)

**Test fixtures created:**
- [x] `manifest_rate_pub_slow/` — publisher with min_rate_hz: 5,
  channel rate_hz: 10
- [x] `manifest_rate_sub_demand/` — subscriber with min_rate_hz: 8

**Done when:**
- [x] Cross-scope publisher rate vs subscriber demand checked
- [x] Pub `min_rate_hz: 5` < channel `rate_hz: 10` → error
- [x] Subscriber-only manifest produces no rate error (publisher absent)
- [x] All 43 manifest_loader tests pass (was 40, +3 new)

### 35.6 — Migrate existing fixtures to ROS topic names ✅ Done

All existing fixtures already use valid ROS-style relative names
(`chatter`, `cropped_points`, `ndt_pose`, etc.) that resolve correctly
via `qualify_name()`. The Phase 34/35 additions established 9 fixtures
using absolute keys for cross-scope patterns, exceeding the "3 fixtures"
target:

**Cross-scope absolute-key fixtures (from Phase 34/35):**
- [x] `manifest_consistency_pub/sub/bad` — `/shared_data`
- [x] `manifest_path_parent/child_ok/child_overflow` — `/sensor/raw`, `/perception/objects`
- [x] `manifest_parallel_pipeline` — `/sensor/raw`, `/perception/lidar_objects`, etc.
- [x] `manifest_rate_pub_slow/rate_sub_demand` — `/shared_stream`

**Existing fixtures migrated in this phase:**
- [x] `manifest_pipeline/manifest.yaml` — added `/sensing/lidar/pointcloud`
  absolute input, fixed scope path to use a topic name
- [x] `manifest_ndt/manifest.yaml` — migrated external topics to absolute
  keys: `/sensing/lidar/pointcloud`, `/sensing/imu/imu_raw`,
  `/map/pointcloud_map`. Fixed scope path to use topic names.

**Left as-is** (functional with relative keys, no migration needed):
- `manifest_simple` (`chatter` is a valid relative name)
- `manifest_args`, `manifest_conditions`, `manifest_control_conditional`,
  `manifest_satisfiability`, `manifest_service_scope`, `manifest_periodic`,
  `manifest_multi_scope`, `manifest_violations` — all use valid
  ROS-style relative names

**Done when:**
- [x] All fixtures use valid ROS-style relative or absolute names
- [x] All tests pass (225 total)
- [x] 11 fixtures demonstrate absolute keys for cross-scope patterns
  (9 from Phase 34/35 + 2 migrated in 35.6)

### 35.7 — Add planned 34.x fixtures ✅ Done

- [x] `manifest_parallel_pipeline/` — already created in Phase 35.1–35.4
  (fork-join with lidar/camera → fusion, critical path = 70ms)
- [x] `manifest_standalone/` — sub-only topic with type, validates
  standalone without cross-scope merge. Subscribes to
  `/localization/kinematic_state` (absolute) with `max_age_ms: 100`.
- [x] `manifest_qos_match/` — exercises all qos-match rule cases:
  - `/map/vector_map` — canonical reliable + transient_local + keep_last
    + depth:1 (no diagnostic)
  - `/telemetry/depth_zero` — depth:0 with keep_last (error)
  - `/sensor/best_effort_transient` — best_effort + transient_local (warning)
  - `/debug/keep_all_with_depth` — keep_all + depth set (warning)

**Integration tests added (8):**
- [x] `fixture_standalone_parses_and_validates`
- [x] `fixture_standalone_no_per_manifest_errors`
- [x] `fixture_standalone_subscriber_max_age_ms`
- [x] `fixture_qos_match_parses`
- [x] `fixture_qos_match_depth_zero_error`
- [x] `fixture_qos_match_best_effort_transient_local_warning`
- [x] `fixture_qos_match_keep_all_with_depth_warning`
- [x] `fixture_qos_match_canonical_vector_map_clean`

All three fixtures added to `all_fixtures_round_trip`.

**Done when:**
- [x] Each fixture has corresponding integration tests
- [x] All 56 fixture tests pass (was 48, +8)
- [x] All 233 total tests pass

### 35.8 — Close out resolved design issues ✅ Done

- [x] Mark issue #18 done — CLI `--rule` filter implemented in 34.8
- [x] Mark issue #42 done — topology-aware critical-path in 35.1–35.4
- [x] Mark issue #43 done — scope path tracing algorithm specified and
  implemented in 35.1–35.4 (see `manifest_graph.rs`)
- [x] Update summary table to show only actually-open issues
- [x] Add a "Recently resolved" table mapping each resolved issue to
  its implementing phase

**Remaining open issues** (all require format extensions, tracked
for future phases):
- #44: `max_transport_ms` per-subscriber (format extension)
- #45: QoS pub/sub compatibility check (needs per-endpoint QoS)
- #49: Lifecycle nodes (format extension)

**Done when:**
- [x] `design-issues.md` reflects current state
- [x] Open issues count matches reality (3 open, was 6)

## Ordering and Dependencies

```
35.1 (build graph) ──→ 35.2 (subgraph) ──→ 35.3 (critical path) ──→ 35.4 (replace sum check)
                                                                          │
                                                                          ↓
35.5 (rate hierarchy) ────────────────────────────────────────────────→ 35.6 (fixtures)
                                                                          │
                                                                          ↓
35.7 (planned fixtures) ──→ 35.8 (close issues)
```

35.1–35.4 are the core graph work and must be done in order.
35.5 is independent (can be done in parallel).
35.6, 35.7, 35.8 are cleanup and can be done after the core work.

## Estimated Scope

| Phase | Type | Effort |
|-------|------|--------|
| 35.1  | New code (graph builder) | Medium |
| 35.2  | New code (subgraph extraction) | Small |
| 35.3  | New code (critical path DP) | Medium |
| 35.4  | Refactor (replace sum check) | Small |
| 35.5  | Cross-scope check addition | Small |
| 35.6  | Fixture cleanup | Medium |
| 35.7  | New fixtures + tests | Small |
| 35.8  | Doc cleanup | Trivial |

## Design Issues Addressed

- **#42**: Topology-unaware sum check → resolved by 35.3/35.4
- **#43**: Scope path tracing algorithm underspecified → resolved by 35.1–35.4
- **#18**: Per-rule CLI filter → already done in 34.8 (just needs status update)

## Out of Scope (deferred to future phases)

- **#44**: Per-subscriber `max_transport_ms` — format extension
- **#45**: Per-endpoint QoS overrides for pub/sub matrix check — format extension
- **#49**: Lifecycle nodes — format extension to mark managed nodes
