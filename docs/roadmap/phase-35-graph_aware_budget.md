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

### 35.6 — Migrate existing fixtures to ROS topic names

Cosmetic but important: existing fixtures use abstract topic keys that
work via relative resolution. Update them to use realistic ROS names so
the docs and reality match.

- [ ] Update `manifest_simple/manifest.yaml`: `chatter` → keep (it's
  a valid relative name)
- [ ] Update `manifest_pipeline/manifest.yaml`: ROS-style names
- [ ] Update `manifest_ndt/manifest.yaml`: realistic Autoware NDT topic
  names like `/localization/pose_estimator/pose`
- [ ] Update `manifest_periodic/manifest.yaml`
- [ ] Update `manifest_multi_scope/manifest.yaml`
- [ ] Update `manifest_violations/manifest.yaml`
- [ ] Update other fixtures as needed
- [ ] Update test assertions accordingly

**Done when:**
- [ ] All fixtures use either valid ROS-style relative names or
  absolute names
- [ ] All tests pass
- [ ] At least 3 fixtures use absolute names for cross-scope subscriptions

### 35.7 — Add planned 34.x fixtures

Three fixtures planned for Phase 34 but not created:

- [ ] `manifest_standalone/` — leaf manifest with sub-only topic + type
  declaration, validates standalone without cross-scope merge
- [ ] `manifest_qos_match/` — fixture exercising the qos-match rule
  end-to-end (depth=0 error, keep_all+depth warning, etc.)
- [ ] `manifest_parallel_pipeline/` — fork-join topology for testing
  35.3/35.4 critical-path computation

**Done when:**
- [ ] Each fixture has corresponding integration tests
- [ ] Tests pass

### 35.8 — Close out resolved design issues

- [ ] Mark issue #18 done in `design-issues.md` (CLI `--rule` filter
  was implemented in Phase 34.8)
- [ ] Mark issue #42 partially-resolved if topology-aware check is done
- [ ] Mark issue #43 done if dataflow tracing is implemented
- [ ] Update summary table

**Done when:**
- [ ] `design-issues.md` reflects current state
- [ ] Open issues count matches reality

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
