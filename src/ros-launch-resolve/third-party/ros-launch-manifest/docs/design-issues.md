# Design Issues

Design questions for the manifest format, with proposed solutions and
their resolutions. All issues 1–51 are now resolved.

## Resolved Issues

Issues resolved in prior phases, preserved in git history:

- **1–6, 8–16**: Args, substitutions, conditions, service contracts,
  doc fixes, parser bugs, unified scope interface, dangling entity
  checks, arg types + satisfiability, YAML `?` suffix — Done
  (Phases 32–33)
- **7, 19, 20, 21, 26, 27**: Global topics, `pub:`/`sub:` overloading,
  scope interface duplication, topic key naming, worked example
  indirection, ROS topic mapping — Superseded by #33
- **24**: Transport latency — Done (`max_transport_ms` on topics)
- **25**: Periodic formula double-counts — Done (upstream removed)
- **28**: `args:` position — Done (`version:` first in examples)
- **17**: Cross-scope service suppression — Resolved by #41
- **33**: Topic keys as ROS names — Done (spec + code). Resolves #7,
  #19, #20, #21, #26, #27
- **34**: Scope paths use topic names — Done (Option A)
- **35**: Parent manifest purpose — Done (E2E contracts via scope paths)
- **36**: Rate check cross-scope merge — Done (note added)
- **37**: Absolute names verbose — Accepted (tooling helps)
- **38**: Relative vs absolute guidance — Done (added to Topics section)
- **39**: `type:` field table — Done (consistent `yes` / `Error`)
- **40**: `global_topics:` note — Done (removed)
- **41**: Services follow topic pattern — Done (ROS names, cross-scope merge)
- **30**: Example error messages — Done (7 rules illustrated)
- **32**: Capture mode — Done (section added to launch-manifest.md)
- **29**: `exclude_patterns` — Done (replaces defaults, `[]` includes all)
- **31**: `correlation: latest` stamp — Done (primary input's stamp)
- **23**: Age on subscriber endpoints — Done (moved from paths)
- **22**: Drop composition — Done (static sanity only, composition is runtime)

---

## ~~17. Cross-Scope Service Wiring Has No Suppression Mechanism~~ — Resolved by #41

Services and actions now follow the same naming pattern as topics
(ROS names, cross-scope merge). The `service-wiring` rule checks the
merged service across the tree, so cross-scope clients no longer
produce orphan `cli:` warnings — they are wired by name matching.
No suppression mechanism is needed. See #41 for details.

---

## ~~18. CLI Should Support Per-Rule Filtering~~ — Done

Implemented in Phase 34.8. The `play_launch check` command now accepts
a repeatable `--rule <RULE_ID>` flag that filters diagnostics
(per-scope and cross-scope) by rule ID. The summary line shows the
active filter. Example:

```bash
play_launch check --manifest-dir manifests/ --rule consistency --rule budget-overflow <pkg> <launch>
```

---

## ~~22. Drop Composition Assumes Independence Despite Burstiness~~ — Done

Resolved: drop composition moved from static checker to **runtime
monitoring only**. The static checker now validates local consistency
only (values in range, scope drop rate not tighter than any topic's,
effective delivery meets subscriber demand). Chain composition and
`max_consecutive` enforcement are runtime concerns — they depend on
actual transport conditions. Appendix A retained as runtime monitoring
theory.

---

## ~~23. Age Verification Effectively Unimplementable Statically~~ — Done

Resolved by moving `max_age_ms` from scope paths to **subscriber
endpoints**. Age is now a data freshness constraint at the point of
consumption (`now - header.stamp` at `rcl_take`), checked at runtime
by the interception layer. No chain tracing needed. Static checking
verifies local consistency (age budget vs known latency budget) but
doesn't attempt a full chain proof.

---

## ~~29. `exclude_patterns` Override Behavior Undocumented~~ — Done

Documented: user declaration **replaces** defaults. `exclude_patterns: []`
includes all topics.

---

## ~~30. No Example Error Messages for Validation Rules~~ — Done

Added example diagnostics block to the Static Validation section
covering 7 rules: `endpoint-unique`, `wiring`, `rate-hierarchy`,
`budget-overflow`, `dangling-entity`, `consistency`, `satisfiability`.

---

## ~~31. `correlation: latest` Output Timestamp Unspecified~~ — Done

Specified: `correlation: latest` output stamp = **primary (first listed)
input's stamp**. Based on analysis of 9 Autoware fusion nodes — 7 of 9
propagate the primary input's timestamp. Secondary inputs enrich the
data but don't determine the timestamp. Age follows the primary branch
only. Updated launch-manifest.md and contract-theory.md (parallel
composition section + summary table).

---

## ~~32. Capture Mode Buried in Theory Appendix~~ — Done

Added "Generating Manifests from a Running System" section to
launch-manifest.md with `--save-manifest-dir` usage, what it generates,
and a link to the statistical derivation in contract-theory.md Appendix C.

---

## ~~33. Topic Keys as ROS Topic Names~~ — Done

Resolved (spec + code). Resolves #7, #19, #20, #21, #26, #27.

Code: `manifest_loader.rs` qualifies relative topic keys against the
scope namespace (`qualify_name`, `qualify_endpoint_ref`) and merges
topic declarations across scopes by FQN (`merge_topic`,
`ResolvedTopic`). Topic-level field consistency (`type:`, `rate_hz:`,
`qos:`, `max_transport_ms`) is enforced under the `consistency`
rule ID during merge.

### Design

Topic keys are ROS topic names — relative (resolved by the checker
using the scope's namespace from the launch tree) or absolute (`/`).
The same topic can appear in multiple manifests; `type:` must agree,
`pub:`/`sub:` are merged. Scope interface removed. `global_topics:`
removed.

### Data (Autoware 1.5.0, 182 topics)

| Category             | Count     | Key format |
|----------------------|-----------|------------|
| Within one subsystem | 147 (81%) | Relative   |
| Cross-subsystem      | 35 (19%)  | Absolute   |

327 topic×scope pairs: 71% sub-only, 26% pub-only, 3% both.
Only 4 topics have publishers in multiple scopes.

### Consistency Rule

- `type:` — required in every declaration, must match across scopes
- `rate_hz:`, `qos:` — must agree if declared in multiple scopes
- `pub:`, `sub:` — merged across scopes by the checker
- New rule: `topic-consistency` validates agreement

### Resolved Questions

1. **Duplicate declarations** → consistency rule (agree, not SSoT)
2. **Deep endpoint refs** → scope-local only (each scope refs own nodes)
3. **Discoverability** → checker merges by resolved name
4. **Namespace** → no `ns:` field; checker uses scope table at check time
5. **Standalone launch** → each manifest self-contained with `type:`

---

## ~~34. Scope-Level Paths Contradict Scope-Local Refs~~ — Done (Option A)

Scope paths now use **topic names** as input/output (not node/endpoint
refs). The checker traces dataflow between the named topics, considering
only nodes within the declaring scope's subtree. When parent and child
declare paths with the same resolved (input, output) topics,
`budget-overflow` checks child budget ≤ parent budget.

---

## ~~35. Parent Manifest Has No Purpose Without Scope Paths~~ — Done

Resolved by #34. Parent manifests declare E2E contracts (scope paths
with topic-name input/output) over child subtrees. The parent also
declares `includes:` for the manifest tree structure.

---

## ~~37. Absolute Topic Names Verbose for Sibling Scopes~~ — Accepted

Accept the verbosity (Option A/D). 81% of topics use short relative
keys. The 19% cross-subsystem topics use absolute names that match
`ros2 topic list` output. Capture mode generates the names
automatically. No format complexity needed.

---

## ~~41. Services Don't Follow the Topic Naming Pattern~~ — Done

Services and actions now follow the same pattern as topics: ROS names
as keys (relative or absolute), cross-scope merge, consistency rule.
`service-wiring` checks the merged service across the tree. Also
resolves #17 — cross-scope services are wired by name matching, no
orphan warnings.

---

## ~~17. Cross-Scope Service Wiring Has No Suppression Mechanism~~ — Resolved by #41

Cross-scope services are now wired by ROS name matching across the
manifest tree, same as topics. No orphan `cli:` warnings — the
`service-wiring` rule checks the merged service after cross-scope merge.

---

## ~~42. Topology-Unaware Sum Check Produces False Warnings~~ — Done

Resolved in Phase 35.1–35.4 (Option A: topology-aware check).

The cross-scope critical-path check in `manifest_loader.rs` builds a
global dataflow graph and uses forward DP with `max` over predecessors
at fork-join points, correctly handling parallel branches. The
`manifest_parallel_pipeline` fixture (lidar 50ms + camera 30ms →
fusion 20ms) verifies that `max(50, 30) + 20 = 70ms` is accepted
without false-warning on the sum (100ms).

The per-manifest `scope-budget` sum check remains as a conservative
fallback for standalone checking (documented as such in
`scope_budget.rs`). For full cross-scope trees, the precise
critical-path check supersedes it.

---

## ~~43. Scope Path Dataflow Tracing Underspecified~~ — Done

Resolved in Phase 35.1–35.4. The algorithm is now specified and
implemented in `src/play_launch/src/ros/manifest_graph.rs`:

- **Graph construction**: `build_global_graph(&ManifestIndex)` builds
  a cross-scope graph from merged topic publishers/subscribers.
  State edges (subscribers with `state: true`) are marked separately
  and skipped in causal traversal.
- **Subgraph extraction**: `subgraph_for_scope_path()` restricts to
  nodes in the scope's subtree and identifies sources/sinks from the
  scope path's resolved input/output topics.
- **Critical path**: `critical_path()` uses topological sort followed
  by forward DP. At each node, `latency = max(predecessor.latency +
  edge.transport) + node.processing`, which correctly handles:
  - Series chains (sum)
  - Fork-join (max at the join point)
  - Multi-input nodes (max over predecessors)
  - State edges (skipped)
- **Diamond patterns**: handled naturally by the topological sort —
  each node's latency is computed once based on its predecessors,
  regardless of how many paths converged on it.
- **Opaque vs transparent scopes**: the current implementation treats
  all scopes as transparent (walks into their nodes). Opaque-scope
  optimization (using declared child budgets without traversing) is
  a potential future improvement but not required for correctness.

---

## ~~44. `max_transport_ms` Ambiguous for Multi-Subscriber Topics~~ — Done

Resolved (spec + code).

### Problem

`max_transport_ms` is declared per topic, but a single ROS 2 topic can
have subscribers with different transport characteristics — one
intra-process (~0ms), one on the same machine via shared memory (<1ms),
one across a network bridge (5–10ms). A single value per topic can't
express this without forcing every path through the worst case.

Transport latency is a **deployment** property of the (publisher,
subscriber) pair, not of the topic itself. Endpoint-level QoS (#45)
does not solve this — QoS profile fields don't capture physical
topology.

### Design (Option B — per-subscriber override)

**Format extension.** `EndpointProps` (subscriber side, mirrored on the
publisher side for symmetry but unused there in v1) gains an optional
`max_transport_ms: f64` field. Topic-level `max_transport_ms` remains
as the default applied to every subscriber that does not override.

**Override rule.** Edge weight in critical-path computation:

```
edge[pub → sub].transport = sub.max_transport_ms
                         ?? topic.max_transport_ms
                         ?? 0
```

**Critical path becomes per-sink.** The DP recurrence in
`manifest_graph.rs::critical_path()` is updated to use the per-edge
weight:
`latency[node] = max_pred( latency[pred] + edge[pred → node].transport )
              + processing[node]`.
The change is local — replaces the current per-topic transport lookup
with a per-(pred, node) lookup.

**Pub-side intentionally omitted.** A publisher does not know which
subscriber will consume data over which transport. Subscribers know
their own consumption pattern (intra-process, SHM, network), so the
override naturally lives on the sub. Pub-side `max_transport_ms` is
not part of the format.

**Cross-scope.** Topic-level `max_transport_ms` stays under
`consistency` rule — must agree across declarations. Per-sub overrides
live on a node and are local to the declaring scope, no cross-scope
agreement required.

### Spec

`docs/launch-manifest.md` §Latency and Data Freshness now documents the
heterogeneous-transport case with the override rule, edge-weight
resolution, per-sink DP recurrence, and a worked example with
intra-process / SHM / network subscribers on the same topic. The
subscriber properties table includes `max_transport_ms`. The topic
field description notes overridability.

### Code — done

- `types/src/types.rs`: `EndpointProps.max_transport_ms` (sub only).
- `types/src/parse.rs`: endpoint parsing extended.
- `ros-launch-resolve` `resolve/src/ros/manifest_graph.rs::critical_path()`:
  per-(pred, node) edge weight lookup with the per-sub override.

---

## ~~45. No QoS Publisher-Subscriber Compatibility Check~~ — Done

Resolved (spec + code).

### Problem

The `qos-compat` rule validates that QoS field values are from the
allowed set (e.g., `reliability: maybe` → error). It does not check
**publisher-subscriber QoS compatibility** — one of the most common
ROS 2 deployment bugs:

- `best_effort` publisher + `reliable` subscriber → **incompatible**
  (no data flows)
- `volatile` publisher + `transient_local` subscriber →
  **incompatible** (late-joining subscriber misses data)

To express the mismatch, the format had to grow: in the original spec
QoS was declared once per topic, so there was no way to model a
publisher and subscriber that disagree on the same channel.

### Design

**Format extension.** `EndpointProps` (publisher and subscriber) gains
an optional `qos: QosDecl` field. Topic-level `qos:` remains as the
default, applied to every endpoint that does not specify its own.

**Override rule (field-level).** Effective QoS for an endpoint is
computed per field: `endpoint.qos.<f> ?? topic.qos.<f> ?? unspecified`.
An endpoint that overrides only `reliability` still inherits other
fields from the topic. Empty `qos: {}` on an endpoint inherits the
topic default in full. Overrides are silent.

**Cross-scope.** Topic-level `qos:` must still agree across scopes
(`consistency` rule). Endpoint-level overrides live on a node and are
local to its declaring scope — they do not participate in cross-scope
merge.

**`qos-match` rule.** After cross-scope merge, the checker computes
effective QoS for each pub and sub on each merged topic and checks
compatibility for every (pub, sub) pair. Rule: **offered ≥ requested**.

| Field         | Pub               | Sub               | Compatible? |
|---------------|-------------------|-------------------|-------------|
| `reliability` | `reliable`        | `reliable`        | yes |
| `reliability` | `reliable`        | `best_effort`     | yes |
| `reliability` | `best_effort`     | `reliable`        | **no** |
| `reliability` | `best_effort`     | `best_effort`     | yes |
| `durability`  | `transient_local` | `transient_local` | yes |
| `durability`  | `transient_local` | `volatile`        | yes |
| `durability`  | `volatile`        | `transient_local` | **no** |
| `durability`  | `volatile`        | `volatile`        | yes |

A field is checked only when both sides specify it (directly or
inherited). The checker does not assume ROS 2 defaults — multiple
profiles (sensor data, services, parameters) have different defaults,
so guessing is wrong. v1 covers `reliability` and `durability` only.
`liveliness`, `deadline`, and `lifespan` compatibility are deferred.

**Conditional endpoints.** When args gate publishers or subscribers,
`qos-match` runs per satisfiable arg model (sharing infrastructure with
`satisfiability`); errors are emitted only for pairs that coexist in
some valid configuration.

**Deferred.** Service/action endpoint QoS, `state-durability` lint
(warn when `state: true` sub uses `volatile`).

### Spec

`docs/launch-manifest.md` §Quality of Service documents the format,
override semantics, match rule, and example diagnostic.

### Code — done

- `types/src/types.rs`: `EndpointProps.qos: Option<QosDecl>`, with
  `QosDecl::effective(topic, endpoint)` field-level overlay.
- `types/src/parse.rs`: endpoint `qos:` parsing.
- `check/src/rules/qos_match.rs`: `qos-match` rule (structural checks +
  reliability/durability pub-sub compatibility).

---

## ~~46. No Guidance on Manifest Node Naming~~ — Done

Added to §Nodes: manifest node name must match the ROS 2 node name
(`name=` attribute / `__node:=` remap, as shown in `ros2 node list`).

---

## ~~47. Missing Inline Include Example~~ — Done

Added inline include example to §Includes.

---

## ~~48. `header.stamp` Propagation Stated as Rule but Is Convention~~ — Done

Softened to "should" with guidance: nodes that reset the stamp should
be modeled as periodic paths (`input: []`).

---

## ~~49. Lifecycle Nodes Not Addressed~~ — Done (Option B)

Added `lifecycle: Option<bool>` field on `NodeDecl`. When set to true,
the node is marked as a ROS 2 managed node, and runtime monitors
gate contract checks (rate, latency, age) on the node being in the
Active state. Static checking is unaffected.

Example:

```yaml
nodes:
  lidar_driver:
    lifecycle: true
    pub:
      pointcloud: { min_rate_hz: 10 }   # applies when driver is Active
```

Updated:
- `types/src/types.rs` — added `lifecycle` field
- `types/src/parse.rs` — parses `lifecycle` key, new test `test_lifecycle_node`
- `docs/launch-manifest.md` — Background section explains lifecycle
  semantics; Nodes format reference documents the field

**Out of scope for v1** (future work):
- Per-state contracts (different rate/latency per state)
- Activation ordering / dependency graph
- Parser auto-detection from launch file (`LifecycleNode` action)

---

## ~~50. `min_latency_ms` Poorly Motivated~~ — Done

Removed from both node and scope path field tables. Not used in any
rule, example, or validation.

---

## ~~51. No Way to Declare External Topics / Producers~~ — Done

Resolved (spec + code). Implementation: top-level `external_topics:`
block per Option B + per-topic `external:` flag per Option A.

### Implementation summary

- **types crate**: `Manifest::external_topics: BTreeMap<String, ExternalTopicDecl>`,
  `TopicDecl::external: Option<ExternalSide>`, new enum
  `ExternalSide { Pub, Sub, Both }`, new struct
  `ExternalTopicDecl { side, msg_type?, qos? }`.
- **parser**: parses `external_topics:` block (accepts `side:` or
  `external:` field name) and per-topic `external:` flag. Validates
  side value ∈ `{pub, sub, both}`. Substitution applied to
  `external_topics.<fqn>.type` like other type fields.
- **`manifest_loader`**: `ManifestIndex.externals: BTreeMap<Fqn, ExternalSide>`
  merged across all loaded manifests in `collect_externals()` (qualified
  per declaring scope's ns; conflicting sides upgrade to `Both`).
  Cross-scope `dangling-entity` skips externally-marked sides on both
  `no publishers` and the newly added `no subscribers` checks.
- **consistency rule**: cross-checks `external_topics.<fqn>.type` against
  any internal `topics.<fqn>.type` of the same FQN. Mismatches emit
  `consistency` errors (this immediately caught real TODO-type bugs in
  the Autoware contract repo).
- **per-manifest noise suppression**: in cross-scope mode
  (`--manifest-dir`), per-manifest `dangling-entity` and `service-wiring`
  rules are dropped before emission — the cross-scope merge is
  authoritative. Prevents O(n) duplicate warnings on legitimate
  cross-scope endpoints.

### Verification

Applied to `~/repos/autoware-contract`: 49 `external_topics:` entries
in `autoware_launch/planning_simulator.yaml`. End-to-end check result:
**0 errors, 0 warnings** across 63 manifests. Type cross-check surfaced
8 real type-mismatch bugs during the migration (now fixed).

### Layers implemented vs deferred

| Layer | Status |
|-------|--------|
| 1. Full `--manifest-dir` visibility for cross-scope merge | Done (pre-existing) |
| 2. Top-level `external_topics:` block at any manifest | Done |
| 3. Ancestor walk for standalone-leaf checks | Deferred (no demand yet) |
| 4. Per-topic `external:` flag | Done |

Layer 3 (collect ancestors' external_topics when checking only a
subtree) is the only deferred piece. Add when a real workflow needs
standalone-leaf checks with deep external dependencies.

---

## 51-archive. Original problem description (preserved for context)

### Problem

After cross-scope merge, the `dangling-entity` rule warns on any topic
with zero publishers (or zero subscribers) anywhere in the manifest
tree. This is correct when the manifest tree is meant to cover a
self-contained launch graph. It is **wrong** when the launch tree
intentionally consumes from systems outside the manifest scope:

- **Sensor drivers** publishing `/sensing/lidar/...` not authored as
  manifests yet
- **Vehicle interface** publishing `/vehicle/status/*` and consuming
  `/vehicle/command/*` (real hardware bridge or CARLA bridge)
- **Map loader** publishing `/map/vector_map`, `/map/pointcloud_map`,
  `/map/projector_info` (its own package, optionally manifest-covered)
- **TF broadcaster** publishing `/tf`, `/tf_static`
- **External cmd source** publishing `/external/local/*`,
  `/external/remote/*` (joystick, web teleop, etc.)
- **rviz2 / debug consumers** subscribing to debug topics
- **bag replay** providing data on any sensor topic
- **Cross-deployment dependencies** (e.g. perception subscribes to a
  topic that another deployment publishes)

In a real Autoware contract repo (`~/repos/autoware-contract`), the
post-Phase-9 check shows ~40 such warnings — every one is an external
producer or consumer. The warnings are noise that hides real problems.

### Why Existing Tools Don't Solve It

- `exclude_patterns:` filters topics out of cross-scope merge entirely;
  but these topics ARE consumed by our nodes — they just have an
  external producer. Excluding them silences the wiring check that
  validates our consumer-side declaration is correct.
- Manually adding stub manifests for every external system is tedious
  and pollutes the tree with non-actionable nodes whose only purpose
  is to satisfy `dangling-entity`.
- `# nolint` comments don't exist in the format.

### Options

| Option                                    | Effort | Description                                                                                      |
|-------------------------------------------|--------|--------------------------------------------------------------------------------------------------|
| A. `external: true` flag on topic         | Small  | Marks a topic's missing side as expected-external. Suppresses dangling-entity for that side only |
| B. Top-level `external_topics:` block     | Small  | Manifest-level list of FQNs known to come from outside. Same effect as A, less per-topic noise   |
| C. Stub-manifest convention               | Medium | Author thin manifests under `external/` package — node-less, pure topic decls. Status quo + tooling |
| D. CLI suppression flag                   | Small  | `play_launch check --external <fqn>...` per invocation                                           |
| E. Cross-scope dangling severity = info   | Trivial| Demote warning to info; lose the safety net entirely                                             |

### Recommendation

**B (preferred) + A (secondary).**

- B gives a single per-manifest declaration of expected external
  producers/consumers, with optional `type:` for cross-check:
  ```yaml
  external_topics:
    /tf:
      type: tf2_msgs/msg/TFMessage
      pub: external                # external system publishes
    /vehicle/command/control_cmd:
      type: autoware_control_msgs/msg/Control
      sub: external                # external system subscribes
    /map/vector_map:
      type: autoware_map_msgs/msg/LaneletMapBin
      pub: external
  ```
  Behavior: `dangling-entity` skips any FQN listed here on the matching
  side. `consistency` still applies (`type:` must agree with declarations
  in other scopes). `qos-match` runs against the listed `qos:` if
  declared.

- A as a shorthand for one-off cases: a topic block can mark its missing
  side directly without a separate top-level list. Useful inside a
  scope's leaf manifest:
  ```yaml
  topics:
    /sensing/lidar/concatenated/pointcloud:
      type: sensor_msgs/msg/PointCloud2
      external: pub                # consumed by us, produced externally
      sub: [lidar_processor/input]
  ```

### Cross-scope merge interaction

When a topic appears in `external_topics:` of scope A and in `topics:`
(with internal pubs) of scope B, the merge resolves to internal —
external is a fallback declaration, not authoritative. This is what
makes the convention safe under partial migration: if an internal
producer manifest is added later, the warning naturally goes away
without the external list needing edit.

### Status

Open. Spec change small (~20 lines in launch-manifest.md + a new field
on `Manifest` and `TopicDecl`). Code change small (filter
`dangling-entity` in `manifest_loader::run_cross_scope_checks` against
the merged external set). High value — removes ~95% of the dangling
warnings on a fully-migrated tree without losing the rule's safety net.

---

## Summary

All design issues 1–51 are now resolved. The summary table below
preserves the most recent phases.

**Recently resolved** (Phase 34/35):

| #  | Issue                               | Resolved in |
|----|-------------------------------------|-------------|
| 18 | Per-rule CLI filter                 | Phase 34.8  |
| 22 | Drop composition assumes independence | Phase 34 (runtime-only) |
| 23 | Age on subscriber endpoints         | Phase 34    |
| 29 | `exclude_patterns` override         | Phase 34    |
| 30 | Example error messages              | Phase 34    |
| 31 | `correlation: latest` stamp         | Phase 34    |
| 32 | Capture mode doc location           | Phase 34    |
| 33 | Topic keys as ROS names             | Phase 34    |
| 34 | Scope paths use topic names         | Phase 34    |
| 35 | Parent manifest purpose             | Phase 34    |
| 37 | Absolute name verbosity (accepted)  | Phase 34    |
| 41 | Services follow topic pattern       | Phase 34    |
| 42 | Topology-aware budget check         | Phase 35.1–35.4 |
| 43 | Scope path tracing algorithm        | Phase 35.1–35.4 |
| 46 | Node naming guidance                | Phase 34    |
| 47 | Inline include example              | Phase 34    |
| 48 | `header.stamp` as convention        | Phase 34    |
| 49 | Lifecycle node `lifecycle:` flag    | Phase 35 (post-35.8) |
| 44 | `max_transport_ms` per-sub override | Phase 35.9  |
| 45 | QoS pub/sub `qos-match` rule        | Phase 35.9  |
| 50 | `min_latency_ms` removed            | Phase 34    |
| 51 | External topics (`external_topics:` + per-topic `external:` flag) | Phase 35.10 |
