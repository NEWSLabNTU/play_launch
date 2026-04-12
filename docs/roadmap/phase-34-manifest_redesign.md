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

### 34.1 — Types: Remove deprecated fields ✅ Done

Remove from `Manifest`:
- [x] Remove `scope_pub`, `scope_sub`, `scope_srv`, `scope_cli`
- [x] Remove `action_server`, `action_client`
- [x] Remove `global_topics: BTreeMap<String, GlobalTopicDecl>`
- [x] Delete `GlobalTopicDecl` struct

Remove from `PathDecl`:
- [x] Remove `max_age_ms`
- [x] Remove `min_latency_ms`

Add to `EndpointProps`:
- [x] Add `max_age_ms: Option<f64>`

Add to `TopicDecl`:
- [x] Add `max_transport_ms: Option<f64>`
- [x] Verify `msg_type` required (parser errors on missing)

**Files:**
- [x] `types/src/types.rs` — struct changes
- [x] `types/src/parse.rs` — remove `parse_groups()`, `parse_global_topics()`,
  add `max_transport_ms` parsing, add `max_age_ms` to endpoint parsing,
  remove `min_latency_ms`/`max_age_ms` from path parsing
- [x] Update all parse tests that reference removed fields

**Done when:**
- [x] `cargo build -p ros-launch-manifest-types` compiles
- [x] `cargo test -p ros-launch-manifest-types` passes (61/61)

### 34.2 — Types: Remove scope interface from substitution and filtering ✅ Done

- [x] `subst.rs` — remove `scope_pub`/`scope_sub` substitution loops
- [x] `cond.rs` — remove scope interface condition filtering and cleanup

**Files:**
- [x] `types/src/subst.rs`
- [x] `types/src/cond.rs`

**Done when:**
- [x] `cargo test -p ros-launch-manifest-types` passes (61/61)

### 34.3 — Checker: Remove deprecated rules, add new rules ✅ Done

**Remove:**
- [x] `drop_rate.rs` — static chain composition (replaced by `drop-sanity`)
- [x] `drop_consecutive.rs` — static consecutive check (runtime-only)
- [x] `rate_chain.rs` — scope export rate chain (no scope interface)

**Rename/refactor:**
- [x] `wiring.rs` — remove scope interface refs, update for topic-name paths

**Add:**
- [x] `drop_sanity.rs` — values in range, effective delivery ≥ sub.min_rate_hz
- [x] `consistency.rs` — stub (full implementation in 34.5)

**Update:**
- [x] `rules/mod.rs` — update `default_rules()` list
- [ ] `scope_budget.rs` — include `max_transport_ms` in sum check (deferred to 34.6)
- [ ] `dangling_entity.rs` — adjust for cross-scope merge (deferred to 34.5)
- [ ] `graph.rs` — scope path input/output as topic names (deferred to 34.6)

**Done when:**
- [x] `cargo build -p ros-launch-manifest-check` compiles
- [x] `cargo test -p ros-launch-manifest-check` passes (116/116: 7 unit + 61 checker + 48 fixture)

### 34.4 — Test fixtures: Migrate to new format ✅ Done (partial)

Rewrite all 11 fixture manifests:
- [x] Remove scope interface (`pub:`/`sub:` at top level)
- [x] Remove `global_topics:`
- [ ] Topic keys → ROS topic names (relative or absolute) — deferred to 34.5
- [ ] Service keys → ROS service names — deferred to 34.5
- [x] `max_age_ms` removed from paths (subscriber-side migration deferred)
- [x] Remove `min_latency_ms`
- [ ] Scope path input/output → topic names — deferred to 34.6
- [ ] Add `max_transport_ms` where applicable — deferred to 34.6

**Add new fixtures:**
- [ ] `manifest_consistency/` — cross-scope topic consistency (deferred to 34.5)
- [ ] `manifest_standalone/` — child manifest validates standalone (deferred to 34.5)
- [ ] `manifest_qos_match/` — QoS pub/sub compatibility (deferred to 34.7)

**Update test harness:**
- [x] `check/tests/checker_tests.rs`
- [x] `check/tests/fixture_tests.rs`

**Done when:**
- [x] All fixture manifests parse without errors
- [x] `cargo test -p ros-launch-manifest-check` passes
- [x] No fixture uses removed fields (`scope_pub`, `global_topics`, `min_latency_ms`)
- [x] `play_launch` `manifest_loader` tests pass (27/27)

**Note:** 34.4 covers structural migration (removing deleted fields). The
ROS-name topic key migration is bundled with 34.5 since it requires the
cross-scope merge logic to validate.

### 34.5 — Manifest loader: Cross-scope merge and consistency ✅ Done

- [x] Build merged topic index after loading all manifests:
  - [x] Group topic declarations by resolved name (using `qualify_name`)
  - [x] Merge `pub:` and `sub:` lists across scopes (deduplicated)
  - [x] Validate `type:` agreement across scopes
  - [x] Validate `rate_hz:` and `qos:` agreement when declared in multiple scopes
  - [x] Validate `max_transport_ms` agreement
  - [x] Emit `consistency` diagnostics for mismatches
- [x] Merge services: `server:`/`client:` by resolved name, validate `type:` agreement
- [x] Update `dangling-entity` check to use merged topic index
  (0 publishers across tree → warning, 0 servers → error)
- [ ] Run rate hierarchy checks on merged topics — deferred to 34.6
- [x] Add `ResolvedService` and `services` to `ManifestIndex`
- [x] Add `merge_diagnostics` to `ManifestIndex` (Vec<Diagnostic>)
- [x] Add `run_cross_scope_checks()` post-merge function

**Files:**
- [x] `src/play_launch/src/ros/manifest_loader.rs` — merge logic
- [x] `check/src/rules/consistency.rs` — kept as stub (cross-scope logic is in
  the manifest loader, not per-manifest checker rule)

**Test fixtures created:**
- [x] `manifest_consistency_pub/` — publisher-only with full contract
- [x] `manifest_consistency_sub/` — subscriber-only with matching type
- [x] `manifest_consistency_bad/` — subscriber-only with type mismatch

**Done when:**
- [x] type match across scopes passes
- [x] type mismatch produces consistency error
- [x] sub-only manifest validates standalone (publisher elsewhere)
- [x] sub-only with no publisher produces dangling-entity warning
- [x] publisher with no subscriber is OK (no dangling)
- [x] All 32 manifest_loader tests pass (27 existing + 5 new cross-scope tests)

### 34.6 — Scope path budget checks with topic-name input/output ✅ Done

- [x] Scope path `input:`/`output:` resolved as topic names using `qualify_name()`
- [x] New `ResolvedScopePath` struct stores resolved input/output FQNs
- [x] `scope_paths: Vec<ResolvedScopePath>` (was `HashMap<usize, Vec<...>>`)
- [x] `scope_parents: HashMap<usize, Option<usize>>` for ancestor lookup
- [x] `budget-overflow`: cross-scope check matches paths by resolved
  (input, output) topics + ancestor relationship; child budget ≤ parent budget
- [x] `scope-budget` sum check includes `max_transport_ms` from topics in scope
- [ ] Dataflow tracing between input/output topics — deferred (algorithm
  underspecified, see issue #43)

**Files:**
- [x] `src/play_launch/src/ros/manifest_loader.rs` — resolve scope path
  topic names, parent chain tracking, cross-scope budget-overflow check
- [x] `check/src/rules/scope_budget.rs` — sum check with `max_transport_ms`
- [ ] `check/src/graph.rs` — scope path tracing (deferred with #43)

**Test fixtures created:**
- [x] `manifest_path_parent/` — parent scope with E2E path budget
- [x] `manifest_path_child_ok/` — child with same boundaries, tighter budget
- [x] `manifest_path_child_overflow/` — child with same boundaries, larger budget

**Done when:**
- [x] Scope paths with relative topic names resolve correctly
- [x] Scope paths with absolute topic names pass through unchanged
- [x] Parent/child path budget-overflow detected on matching (input, output) pairs
- [x] Unrelated scopes (sibling) with matching paths do NOT trigger overflow
- [x] `max_transport_ms` included in `scope-budget` sum check
- [x] All 36 manifest_loader tests pass (32 + 4 new path tests)

### 34.7 — QoS compatibility rule ✅ Done

The original design issue (#45) called for a "best_effort pub + reliable
sub" check. In the current manifest model where topic QoS is a single
channel-level profile shared by both sides, this matrix check is **moot**
— both sides agree by construction. The cross-scope `consistency` rule
catches the case where two scopes declare the same topic with different
QoS values (Phase 34.5).

What `qos-match` *can* usefully catch is **structural validity** of a
single QoS profile that `qos-compat` doesn't check:

- [x] Add `qos_match.rs` — structural validity + DDS pitfall detection
- [x] `depth: 0` → error (cannot buffer any messages)
- [x] `keep_all` + `depth` → warning (depth ignored when history is keep_all)
- [x] `best_effort` + `transient_local` → warning (late joiners cannot
  receive historical data over best_effort)
- [x] Add to `default_rules()` in `rules/mod.rs`

**Files:**
- [x] `check/src/rules/qos_match.rs` (new)
- [x] `check/src/rules/mod.rs`

**Done when:**
- [x] depth=0 produces error
- [x] keep_all + depth produces warning
- [x] best_effort + transient_local produces warning
- [x] reliable + transient_local + keep_last + depth>0 (canonical "deliver to
  late joiners" pattern) produces no diagnostics
- [x] Topic without QoS declaration produces no qos-match diagnostics
- [x] All tests pass (66 checker tests, 5 new for qos-match)

**Note on per-endpoint QoS:** if future versions add per-endpoint QoS
overrides on `EndpointProps`, the qos-match rule should be extended to
also validate pub-vs-sub QoS compatibility using the ROS 2 matrix
(`best_effort` pub + `reliable` sub → no data flow). Tracked as a
follow-up.

### 34.8 — CLI and integration ✅ Done

- [x] Update `play_launch check` output to show cross-scope diagnostics
  via `render_cross_scope_diagnostics()` (rendered after per-scope output)
- [x] Add `--rule <RULE_ID>` filter flag (design issue #18) — repeatable
- [x] No CLAUDE.md test command changes needed (no hardcoded rule count)
- [x] `play_launch check --help` shows new `--rule` flag

**Files:**
- [x] `src/play_launch/src/cli/options.rs` — add `rule: Vec<String>` to `CheckArgs`
- [x] `src/play_launch/src/commands/manifest.rs` — filter, cross-scope rendering, summary

**Implementation details:**
- `filter_diagnostics()`: applies rule filter to a slice of diagnostics
- `render_cross_scope_diagnostics()`: renders `merge_diagnostics` from
  `ManifestIndex` (consistency, dangling-entity, budget-overflow) under
  a "── Cross-scope diagnostics ──" heading
- `print_summary()`: counts errors/warnings post-filter, includes a
  `[filter: rule1,rule2]` note when filter is active
- `has_filtered_errors()`: exit code reflects filtered error count
- JSON output mode supports both per-scope and cross-scope diagnostics

**Done when:**
- [x] `--rule <ID>` filter shows only matching diagnostics across all scopes
- [x] `--rule` is repeatable (`--rule consistency --rule satisfiability`)
- [x] Cross-scope consistency errors render under their own heading
- [x] Cross-scope diagnostics counted in the summary
- [x] Exit code reflects filtered error count
- [x] `cargo build` passes
- [x] All 36 manifest_loader tests pass

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
