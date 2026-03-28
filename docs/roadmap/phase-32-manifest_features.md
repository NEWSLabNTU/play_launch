# Phase 32: Manifest Format Features

**Status**: Complete (32.1–32.6 done, 32.4 dropped)
**Priority**: High
**Dependencies**: Phase 31 (manifest crates, static check CLI)
**Repo**: `src/ros-launch-manifest/` (types + check crates)

Design issues: `src/ros-launch-manifest/docs/design-issues.md`

---

## Prior Work (Phase 31)

| Phase | Description                                                                           | Tests |
|-------|---------------------------------------------------------------------------------------|-------|
| 31.1  | Types crate: 14 types, YAML parser, span index                                        | 24    |
| 31.2  | Checker crate: 9 validation rules, petgraph graph, codespan emitter                   | 54    |
| 31.3  | Test fixtures: 6 manifests (simple, pipeline, NDT, periodic, violations, multi-scope) | —     |
| 31.4  | Executor loading, integration tests, source spans                                     | 21    |
| 31.5  | Static check CLI: `play_launch check`                                                 | 11    |

---

## 32.1: Doc Fixes

Fix stale descriptions in `launch-manifest.md` to match current implementation.
No code changes.

- [x] 32.1.1: Fix "parser loads manifests" → "executor loads manifests" (design-issues #3a)
- [x] 32.1.2: Fix `max_latency_ms` → `max_response_ms` on service examples (#3b)
- [x] 32.1.3: Fix design doc cross-references (#3c)
- [x] 32.1.4: Add `cli:` documentation alongside `srv:`, clarify server vs client (#4)
- [x] 32.1.5: Clarify endpoint names are manifest-local identifiers (#5)
- [x] 32.1.6: Fix external include file naming (`.launch.yaml` → `.yaml`) (#8)

## 32.2: Args and Substitutions

Add `args:` declaration and `$(var ...)` substitution to the manifest format.
Enables exact topic name matching when topic names depend on launch arguments.

**Types crate:**
- [x] 32.2.1: Add `args: BTreeMap<String, Option<String>>` to `Manifest`
  - Value = default, None = required
  - Parser: `parse_args()` — plain value or null (same shorthand as endpoints)
- [x] 32.2.2: Add `$(var ...)` substitution engine (`types/src/subst.rs`)
  - `resolve_args()` — merge caller args over manifest defaults, error on missing required
  - `substitute_str()` — replace `$(var name)` with resolved value
  - `substitute_manifest()` — walk all string fields (topics, services, imports, exports, paths)
  - Error on unresolved `$(var ...)` or malformed `$(var unclosed`
- [x] 32.2.3: Unit tests (10 tests in `subst::tests`)
  - String substitution: basic, multiple, no vars, unresolved, malformed
  - Arg resolution: defaults, override, required present, required missing, passthrough
  - Manifest substitution: $(var ...) in topic type field

**Manifest loader:**
- [x] 32.2.4: Merge scope args over manifest defaults before substitution
  - `resolve_args()` called with `scope.args` and `manifest.args`
  - Warns and skips scope on required arg missing or substitution error
- [x] 32.2.5: Run substitution before namespace resolution and static checks
  - Pipeline: parse → resolve args → substitute → check → namespace → index
- [x] 32.2.6: Integration tests (3 tests in `manifest_loader::tests`)
  - Args with defaults resolved without scope override
  - Args overridden by scope args
  - No args section still works (backward compatible)
- [x] Test fixture: `tests/fixtures/manifest_args/manifest.yaml`

## 32.3: Conditions

Add `if:` and `unless:` to manifest entities. Enables conditional nodes and
topics based on launch arguments.

**Types crate (`cond.rs`):**
- [x] 32.3.1: Add `if_condition: Option<String>` and `unless_condition: Option<String>`
  to `NodeDecl`, `TopicDecl`, `ServiceDecl`, `ActionDecl`, `PathDecl`
  - YAML keys: `if:`, `unless:` (via `#[serde(rename)]`)
  - Parser: `yaml_string(yaml, "if")` / `yaml_string(yaml, "unless")` on all entity parsers
- [x] 32.3.2: Condition evaluator (`cond::evaluate`)
  - Boolean form: bare string → `"true"` = true, else false
  - Expression form: recursive descent parser for `==`, `!=`, `and`, `or`, parentheses
  - Quoted and bare word values, all string comparison
  - `cond::should_include(if_cond, unless_cond) -> bool`
- [x] 32.3.3: Filter function (`cond::filter_manifest`)
  - Removes nodes, topics, services, actions, paths where condition is false
  - Also filters node-level paths
- [x] 32.3.4: Unit tests (19 tests in `cond::tests`)
  - Boolean: true, false, empty, arbitrary strings
  - Comparison: ==, !=, bare words, quoted strings
  - Compound: and, or, parentheses, mixed
  - should_include: no conditions, if true/false, unless true/false, both
  - filter_manifest: 5 nodes + 2 topics filtered by if/unless
- [x] 32.3.5: Integration tests (3 tests in `manifest_loader::tests`)
  - Default args: feature_a included, feature_b excluded, legacy excluded, sensor_specific included
  - Overridden args: feature_a excluded, feature_b included, legacy included
  - Sensor model mismatch: sensor_specific excluded
- [x] Substitution engine updated to resolve `$(var ...)` in if/unless condition strings
- [x] Test fixture: `tests/fixtures/manifest_conditions/manifest.yaml`

**Manifest loader:**
- [x] 32.3.6: Run condition filtering after substitution, before namespace resolution
  - Pipeline: parse → resolve args → substitute → **filter conditions** → check → namespace → index

## ~~32.4: Import Topic Mapping~~ — Dropped

~~Add `topic:` field to imports for explicit topic name mapping.~~

**Dropped**: The parent manifest's `topics:` section already wires child imports
to actual topic names via `sub: [child_scope/import_group]`. The child import
only needs to declare the scope boundary — the resolved topic name is the
parent's responsibility. Adding `topic:` to imports would be redundant.

## 32.5: Service Static Checks

Add static validation rules for service wiring and type compatibility.

**Checker crate (2 new rules, total 11):**
- [x] 32.5.1: `service_wiring` rule (`service_wiring.rs`): every `cli:` endpoint
  on a node must have a matching `services:` entry with a non-empty `server:` list
- [x] 32.5.2: `service_type` rule (`service_type.rs`):
  - Service must have a `type` declared (error if empty)
  - Server refs must exist as `srv:` on their nodes (warning if not)
  - Client refs must exist as `cli:` on their nodes (warning if not)
- [x] 32.5.3: Unit tests in checker_tests.rs (4 tests):
  - Clean service wiring (no diagnostics)
  - Missing server (warning from service-wiring)
  - Missing service type (error from service-type)
  - Server not on node (warning from service-type)
- [x] 32.5.4: Fixture violations updated:
  - `orphan_client` node with `cli: { missing_service: {} }` (no matching server)
  - `typeless_service` in `services:` with no type
- [x] Fixture tests (2 tests): `fixture_violations_service_wiring`, `fixture_violations_service_type`

## 32.6: Update launch-manifest.md

Update the design doc to incorporate all changes from 32.1–32.5.

- [x] 32.6.1: Add `args:` section with examples (done during 32.2)
- [x] 32.6.2: Add `if:`/`unless:` section with examples (done during 32.3)
- [x] ~~32.6.3: Update imports section with `topic:` field~~ (dropped with 32.4)
- [x] 32.6.4: Update concepts list (5 → 7: added args, conditions), add quick
  example with args + conditions, add 11-rule validation table
- [x] 32.6.5: Stale content already fixed in 32.1 (parser loading, cross-refs,
  file naming, endpoint description, service max_response_ms)

## 32.7: Test Coverage Gaps

Fill test gaps identified after 32.1–32.6.

**Manifest crate fixture tests (`fixture_tests.rs`):**
- [x] 32.7.1: Added `manifest_args/` and `manifest_conditions/` to `all_fixtures_round_trip`
- [x] 32.7.2: `fixture_args_parses` + `fixture_args_clean` — verify args parsed, checker clean
- [x] 32.7.3: `fixture_conditions_parses` + `fixture_conditions_filter_with_defaults` —
  verify conditions parsed, filtered correctly with resolve+substitute+filter pipeline

**Checker integration tests (`checker_tests.rs`):**
- [x] 32.7.4: `test_args_conditions_combined` — manifest with `$(var ...)` in `if:` and
  topic type, checked in two configurations (sensor enabled/disabled), verifying
  nodes survive or are filtered and checker reports no errors in both cases

**CLI integration tests (`tests/tests/manifest_check.rs`):**
- [x] ~~32.7.5–32.7.6~~ — Not needed. CLI requires a launch file; args/conditions are
  exercised through manifest_loader integration tests with synthetic scope tables.
  The CLI flow is already covered by existing tests.

---

## Phase Order

```
32.1 (doc fixes) ───────────────────────────────┐
                                                │
32.2 (args + substitution) ──→ 32.3 (conditions)┤
                                                │
32.4 (import topic mapping) ────────────────────┤
                                                │
32.5 (service checks) ──────────────────────────┤
                                                │
32.6 (rewrite launch-manifest.md) ──────────────┘
```

32.1 can be done anytime. 32.2 must precede 32.3 (conditions depend on
substitution). 32.4 can be done in parallel with 32.2/32.3. 32.5 is
independent. 32.6 is last — incorporates everything.

---

## Verification

```bash
# Types + checker tests
cd src/ros-launch-manifest
cargo test

# Executor loader tests
cd src/play_launch
cargo test manifest_loader

# CLI integration tests
cd tests
cargo test --test manifest_check

# Autoware contracts (external repo)
play_launch check --manifest-dir ~/repos/autoware-contract/ \
    autoware_launch planning_simulator.launch.xml
```
