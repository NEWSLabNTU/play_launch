# Phase 34: Manifest Format Redesign

Migration from the current manifest implementation to the design
documented in `src/ros-launch-manifest/docs/launch-manifest.md`.

## Current State

The implementation supports the **v1 format**: abstract topic keys,
scope interface (`pub:`/`sub:` at top level), `global_topics:`,
`max_age_ms` on scope paths, static drop composition, and
`min_latency_ms`. 14 validation rules, Z3 satisfiability, span-tracked
diagnostics.

## Target State

The **redesigned format**: ROS topic names as keys, scope-local endpoint
refs, cross-scope consistency rule, `max_age_ms` on subscriber
endpoints, drop composition runtime-only, services follow topic pattern,
scope paths use topic names as input/output.

## Migration Summary

| Area                    | Current                                            | Target                                           | Breaking? |
|-------------------------|----------------------------------------------------|--------------------------------------------------|-----------|
| Topic keys              | Abstract local identifiers                         | ROS topic names (relative/absolute)              | Yes       |
| Scope interface         | `scope_pub`, `scope_sub`, `scope_srv`, `scope_cli` | Removed                                          | Yes       |
| `global_topics:`        | Separate section                                   | Removed (absolute keys in `topics:`)             | Yes       |
| Service/action keys     | Abstract identifiers                               | ROS names (same as topics)                       | Yes       |
| `max_age_ms`            | On `PathDecl` (scope paths)                        | On `EndpointProps` (subscriber)                  | Yes       |
| `min_latency_ms`        | On `PathDecl`                                      | Removed                                          | Yes       |
| Drop composition        | Static rules (`drop-rate`, `drop-consecutive`)     | Static sanity only (`drop-sanity`)               | Yes       |
| Scope path input/output | Endpoint refs (`node/endpoint`)                    | Topic names (relative/absolute)                  | Yes       |
| `max_transport_ms`      | Not implemented                                    | On `TopicDecl`                                   | Additive  |
| `consistency` rule      | Not implemented                                    | New rule (type/rate/QoS agreement across scopes) | Additive  |
| `qos-match` rule        | Not implemented                                    | New rule (pub/sub QoS compatibility)             | Additive  |
| Node naming             | Implicit                                           | Must match ROS 2 node name                       | Doc only  |

## Phases

### 34.1 — Types: Remove deprecated fields

Remove from `Manifest`:
- [ ] Remove `scope_pub`, `scope_sub`, `scope_srv`, `scope_cli`
- [ ] Remove `action_server`, `action_client`
- [ ] Remove `global_topics: BTreeMap<String, GlobalTopicDecl>`
- [ ] Delete `GlobalTopicDecl` struct

Remove from `PathDecl`:
- [ ] Remove `max_age_ms`
- [ ] Remove `min_latency_ms`

Add to `EndpointProps`:
- [ ] Add `max_age_ms: Option<f64>`

Add to `TopicDecl`:
- [ ] Add `max_transport_ms: Option<f64>`
- [ ] Verify `msg_type` required (parser errors on missing)

**Files:**
- [ ] `types/src/types.rs` — struct changes
- [ ] `types/src/parse.rs` — remove `parse_groups()`, `parse_global_topics()`,
  add `max_transport_ms` parsing, add `max_age_ms` to endpoint parsing,
  remove `min_latency_ms`/`max_age_ms` from path parsing
- [ ] Update all parse tests that reference removed fields

**Done when:**
- [ ] `cargo build -p ros-launch-manifest-types` compiles
- [ ] `cargo test -p ros-launch-manifest-types` passes (parse tests updated)

### 34.2 — Types: Remove scope interface from substitution and filtering

- [ ] `subst.rs` — remove `scope_pub`/`scope_sub` substitution loops
- [ ] `cond.rs` — remove scope interface condition filtering and cleanup

**Files:**
- [ ] `types/src/subst.rs`
- [ ] `types/src/cond.rs`

**Done when:**
- [ ] `cargo test -p ros-launch-manifest-types` passes

### 34.3 — Checker: Remove deprecated rules, add new rules

**Remove:**
- [ ] `drop_rate.rs` — static chain composition (replaced by `drop-sanity`)
- [ ] `drop_consecutive.rs` — static consecutive check (runtime-only)
- [ ] `rate_chain.rs` — scope export rate chain (no scope interface)

**Rename/refactor:**
- [ ] `wiring.rs` — remove scope interface refs, update for topic-name paths

**Add:**
- [ ] `drop_sanity.rs` — values in range, scope drop ≤ topic drop, effective
  delivery ≥ sub.min_rate_hz
- [ ] `consistency.rs` — stub (requires cross-scope merge in 34.5)

**Update:**
- [ ] `rules/mod.rs` — update `default_rules()` list
- [ ] `scope_budget.rs` — include `max_transport_ms` in sum check
- [ ] `dangling_entity.rs` — adjust for cross-scope merge (topics with 0 local
  publishers are OK if another scope publishes)
- [ ] `graph.rs` — remove scope interface nodes, scope path input/output are
  topic names not endpoint refs

**Done when:**
- [ ] `cargo build -p ros-launch-manifest-check` compiles
- [ ] `cargo test -p ros-launch-manifest-check` passes (with updated fixtures from 34.4)

### 34.4 — Test fixtures: Migrate to new format

Rewrite all 11 fixture manifests:
- [ ] Remove scope interface (`pub:`/`sub:` at top level)
- [ ] Remove `global_topics:`
- [ ] Topic keys → ROS topic names (relative or absolute)
- [ ] Service keys → ROS service names
- [ ] `max_age_ms` → subscriber endpoints
- [ ] Remove `min_latency_ms`
- [ ] Scope path input/output → topic names
- [ ] Add `max_transport_ms` where applicable

**Add new fixtures:**
- [ ] `manifest_consistency/` — cross-scope topic consistency (matching types
  OK, mismatching types error, rate agreement)
- [ ] `manifest_standalone/` — child manifest validates standalone
  (sub-only topic with type)
- [ ] `manifest_qos_match/` — QoS pub/sub compatibility

**Update test harness:**
- [ ] `check/tests/checker_tests.rs`
- [ ] `check/tests/fixture_tests.rs`

**Done when:**
- [ ] All fixture manifests parse without errors
- [ ] `cargo test -p ros-launch-manifest-check` passes
- [ ] No fixture uses removed fields (`scope_pub`, `global_topics`, `min_latency_ms`, etc.)

### 34.5 — Manifest loader: Cross-scope merge and consistency

- [ ] Build merged topic index after loading all manifests:
  - [ ] Group topic declarations by resolved name (using `qualify_name`)
  - [ ] Merge `pub:` and `sub:` lists across scopes
  - [ ] Validate `type:` agreement across scopes
  - [ ] Validate `rate_hz:` and `qos:` agreement when declared in multiple scopes
  - [ ] Emit `consistency` diagnostics for mismatches
- [ ] Merge services: `server:`/`client:` by resolved name, validate `type:` agreement
- [ ] Update `dangling_entity` check to use merged topic index
  (0 publishers across tree → warning, 0 servers → error)
- [ ] Run rate hierarchy checks on merged topics (publisher `min_rate_hz`
  from one scope vs subscriber `min_rate_hz` in another)
- [ ] Implement `consistency.rs` rule (full, not stub)

**Files:**
- [ ] `src/play_launch/src/ros/manifest_loader.rs` — merge logic
- [ ] `check/src/rules/consistency.rs` — full implementation

**Done when:**
- [ ] `manifest_consistency` fixture: type match passes, type mismatch errors
- [ ] `manifest_standalone` fixture: sub-only manifest validates standalone
- [ ] `manifest_loader` integration tests pass with merged topics
- [ ] `play_launch check` on Autoware manifests produces consistency diagnostics

### 34.6 — Scope path budget checks with topic-name input/output

- [ ] Scope path `input:`/`output:` resolved as topic names using `qualify_name()`
- [ ] Checker traces dataflow between resolved input/output topics,
  considering only nodes within the scope's subtree
- [ ] `budget-overflow`: when parent and child scopes declare paths with
  the same resolved (input, output) topics, child budget ≤ parent budget
- [ ] `scope-budget` sum check includes `max_transport_ms` from topics on the path

**Files:**
- [ ] `src/play_launch/src/ros/manifest_loader.rs` — resolve scope path topic names
- [ ] `check/src/rules/scope_budget.rs` — updated sum check
- [ ] `check/src/graph.rs` — scope path tracing with topic-name boundaries

**Done when:**
- [ ] Scope paths with relative topic names resolve correctly
- [ ] Scope paths with absolute topic names pass through
- [ ] Parent/child path budget-overflow detected on matching (input, output) pairs
- [ ] `max_transport_ms` included in sum check

### 34.7 — QoS compatibility rule

- [ ] Add `qos_match.rs` — check publisher QoS against subscriber QoS
  on merged topics using ROS 2 compatibility matrix
- [ ] `best_effort` pub + `reliable` sub → error
- [ ] `volatile` pub + `transient_local` sub → error
- [ ] Add to `default_rules()` in `rules/mod.rs`

**Files:**
- [ ] `check/src/rules/qos_match.rs` (new)
- [ ] `check/src/rules/mod.rs`

**Done when:**
- [ ] `manifest_qos_match` fixture: compatible QoS passes, incompatible errors
- [ ] Rule fires on merged topics (pub in one scope, sub in another)

### 34.8 — CLI and integration

- [ ] Update `play_launch check` output to show cross-scope diagnostics
  (file + scope context for consistency errors)
- [ ] Add `--rule <RULE_ID>` filter flag (design issue #18)
- [ ] Update CLAUDE.md test commands if rule count changes
- [ ] Verify `play_launch check` on Autoware manifests runs cleanly

**Files:**
- [ ] `src/play_launch/src/commands/manifest.rs`

**Done when:**
- [ ] `--rule satisfiability` shows only satisfiability results
- [ ] Cross-scope consistency errors show both source locations
- [ ] `just test` passes

## Ordering and Dependencies

```
34.1 (types) ──→ 34.2 (subst/cond) ──→ 34.3 (checker rules)
                                              │
34.4 (fixtures) ─────────────────────────────→ ├──→ 34.5 (cross-scope merge)
                                              │         │
                                              │         ├──→ 34.6 (scope paths)
                                              │         │
                                              │         └──→ 34.7 (QoS match)
                                              │
                                              └──→ 34.8 (CLI)
```

34.1–34.3 are the core format migration (can be done together).
34.4 updates tests to match.
34.5–34.7 add new cross-scope capabilities.
34.8 is CLI polish.

## Estimated Scope

| Phase | Files changed | Effort |
|-------|--------------|--------|
| 34.1 | 2 (types) | Small |
| 34.2 | 2 (subst, cond) | Small |
| 34.3 | ~8 (rules, graph) | Medium |
| 34.4 | ~15 (fixtures, tests) | Medium |
| 34.5 | 2 (loader, consistency) | Medium |
| 34.6 | 3 (loader, scope_budget, graph) | Medium |
| 34.7 | 2 (qos_match, mod) | Small |
| 34.8 | 1 (manifest cmd) | Small |

## Design Issues to Address During Migration

- **#42**: Topology-unaware sum check — consider during 34.6
- **#43**: Scope path tracing algorithm — specify during 34.6
- **#44**: `max_transport_ms` multi-subscriber — document as worst-case during 34.1
- **#45**: QoS pub/sub compatibility — implement in 34.7
- **#49**: Lifecycle nodes — document as limitation, defer
