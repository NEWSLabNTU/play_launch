# Phase 32: Manifest Format Features

**Status**: Not started
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

**Types crate:**
- [ ] 32.3.1: Add `if_condition: Option<String>` and `unless_condition: Option<String>`
  to `NodeDecl`, `TopicDecl`, `ServiceDecl`, `PathDecl`, `IncludeDecl`
  - YAML keys: `if:`, `unless:`
  - Parser: extract before other fields
- [ ] 32.3.2: Condition evaluator
  - Boolean form: bare substitution → compare to `"true"`
  - Expression form: `value == value`, `value != value`, `and`, `or`, parentheses
  - All comparisons are string equality
  - `evaluate_condition(expr: &str) -> Result<bool>`
- [ ] 32.3.3: Filter function: remove entities where condition is false
  - `filter_by_conditions(manifest: &mut Manifest)` — removes nodes, topics,
    services, paths, includes where `if:` is false or `unless:` is true
- [ ] 32.3.4: Unit tests for condition evaluation
  - Boolean: `"true"` → true, `"false"` → false, `""` → false
  - Comparison: `"a" == "a"` → true, `"a" != "b"` → true
  - Compound: `"true" and "true"` → true, `"true" or "false"` → true
  - `unless:` inversion
- [ ] 32.3.5: Integration tests: manifest with conditional nodes, checked with
  different arg sets producing different entity sets

**Manifest loader:**
- [ ] 32.3.6: Run condition filtering after substitution, before namespace resolution
  - Pipeline: args merge → substitute → filter conditions → namespace → check

## 32.4: Import Topic Mapping

Add `topic:` field to imports for explicit topic name mapping.

**Types crate:**
- [ ] 32.4.1: Change `imports` from `BTreeMap<String, Vec<String>>` to support
  both short form (`[endpoints]`) and long form (`{ topic: ..., endpoints: [...] }`)
  - New type: `ImportDecl` (enum: list or struct)
- [ ] 32.4.2: Parser handles both forms
- [ ] 32.4.3: Unit tests for both import forms

**Manifest loader:**
- [ ] 32.4.4: Resolve `$(var ...)` in import `topic:` field
- [ ] 32.4.5: Use resolved topic name for wiring verification

## 32.5: Service Static Checks

Add static validation rules for service wiring and type compatibility.

**Checker crate:**
- [ ] 32.5.1: `service_wiring` rule: every `cli:` endpoint on a node must have a
  matching entry in `services:` with a `server:` list (within scope or imported)
- [ ] 32.5.2: `service_type` rule: client and server on the same `services:` entry
  must reference the same service type
- [ ] 32.5.3: Unit tests: missing server, type mismatch, clean wiring
- [ ] 32.5.4: Fixture: add service violations to `manifest_violations/`

## 32.6: Update launch-manifest.md

Rewrite the design doc to incorporate all changes from 32.1–32.5.

- [ ] 32.6.1: Add `args:` section with examples
- [ ] 32.6.2: Add `if:`/`unless:` section with examples
- [ ] 32.6.3: Update imports section with `topic:` field
- [ ] 32.6.4: Update all examples to use current naming conventions
- [ ] 32.6.5: Remove stale content (parser loading, old paths, etc.)

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
