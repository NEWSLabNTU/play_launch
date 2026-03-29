# Phase 33: Manifest Format v2

**Status**: In progress (33.1–33.3 complete)
**Priority**: Medium
**Dependencies**: Phase 32 (args, conditions, service checks)
**Design**: `src/ros-launch-manifest/docs/design-issues.md` #11–14

New manifest format features: unified scope interface, arg types,
dangling entity checks, and satisfiability analysis.

---

## 33.1: Parser — Record All Resolved Args (Bug Fix)

**In**: `src/play_launch_parser/`
**Design issue**: #11

The parser only records args explicitly passed from parent `<include>` tags.
Args with defaults in the launch XML that aren't overridden are resolved
but not stored in `scope.args`.

- [x] 33.1.1: Added `ScopeTable::update_args()` — updates scope args after traversal
- [x] 33.1.2: All 6 include paths call `update_args()` after file traversal:
  - `include.rs` line 260: XML include → `included_traverser.context.configurations()`
  - `include.rs` line 167: Python-from-XML → `self.context.configurations()`
  - `include.rs` line 199: YAML-from-XML → `self.context.configurations()`
  - `python_exec.rs` line 228: Python-to-Python → `self.context.configurations()`
  - `python_exec.rs` line 267: Python-to-YAML → `self.context.configurations()`
  - `xml_include.rs` line 113: XML-from-Python → `included_traverser.context.configurations()`
- [x] 33.1.3: Autoware validation: `behavior_planning` scope now has 174 args
  (was 166), including `input_traffic_light_topic_name` and `input_vector_map_topic_name`
- [x] 33.1.4: All 389 parser tests pass, quality clean

## 33.2: Unified Scope Interface

**In**: `src/ros-launch-manifest/` (types + check + play_launch)
**Design issue**: #12

Breaking change: replace `imports:`/`exports:` with top-level `pub:`/`sub:`/
`srv:`/`cli:` (and `action_server:`/`action_client:`).

**Types crate (`types.rs`):**
- [x] 33.2.1: Removed `imports` and `exports` from `Manifest`
- [x] 33.2.2: Added scope interface fields: `scope_pub`, `scope_sub`, `scope_srv`,
  `scope_cli`, `action_server`, `action_client` (serde renamed to `pub`/`sub`/etc.)

**Parser (`parse.rs`):**
- [x] 33.2.3: Removed `parse_groups(doc, "imports")` / `parse_groups(doc, "exports")`
- [x] 33.2.4: Added parsing for `pub:`, `sub:`, `srv:`, `cli:`, `action_server:`,
  `action_client:` at top level

**Substitution engine (`subst.rs`):**
- [x] 33.2.5: Walks all 6 new scope interface fields

**Condition filter (`cond.rs`):**
- [x] 33.2.6: `cleanup_dangling_refs` already handles scope fields (no change needed —
  it only cleans topic/service endpoint lists, not scope groups directly)

**Checker rules:**
- [x] 33.2.7: `wiring` rule updated — uses `scope_sub` instead of `imports`
- [x] 33.2.8: `rate_chain` rule updated — uses `scope_pub` instead of `exports`
- [x] 33.2.9: `service-wiring` — no change needed (already checks scope-level services)
- [x] 33.2.10: `optional-ref` — no change needed (only checks topic/service refs)

**Graph builder (`graph.rs`):**
- [x] 33.2.11: No change needed — graph uses includes for scope vertices, not imports/exports

**Manifest loader (`manifest_loader.rs`):**
- [x] 33.2.12: No change needed — loader doesn't reference imports/exports directly
- [x] 33.2.13: No change needed

**Test fixtures (all 8 updated):**
- [x] 33.2.14–33.2.21: All fixture YAMLs updated (`imports:` → `sub:`, `exports:` → `pub:`)

**Tests:**
- [x] 33.2.22: All checker_tests.rs inline YAML updated
- [x] 33.2.23: All fixture_tests.rs assertions updated (`.imports` → `.scope_sub`, etc.)
- [x] 33.2.24: Manifest loader tests — no changes needed
- [x] 33.2.25: 124 manifest + 27 loader + 390 parser tests pass, quality clean

**Autoware contract** (`~/repos/autoware-contract/`):
- [x] 33.2.26: Updated all 36 manifests: `imports:` → `sub:`, `exports:` → `pub:`
- [x] 33.2.27: End-to-end: 45 loaded, 0 errors, 149 warnings (unchanged)

## 33.3: Arg Type Declarations

**In**: `src/ros-launch-manifest/` (types + check)
**Design issue**: #14

Add `type: bool` and `choices: [...]` to arg declarations.

- [x] 33.3.1: Changed `args` from `Vec<String>` to `BTreeMap<String, ArgDecl>`
  where `ArgDecl` enum: `String` (default), `Bool`, `Choices(Vec<String>)`
- [x] 33.3.2: Parser handles: bare name/null (free), `{ type: bool }`,
  `{ choices: [a, b] }`, list form (all free)
- [x] 33.3.3: `resolve_args` validates values: bool arg rejects non-bool,
  choices arg rejects unlisted values (`InvalidArgValue` error)
- [x] 33.3.4: 5 new tests: bool valid/invalid, choices valid/invalid, parse typed YAML
- [x] 33.3.5: `ArgDecl::valid_values()` → `Option<Vec<&str>>` for enumeration

## 33.4: Dangling Entity Checks

**In**: `src/ros-launch-manifest/` (check)
**Design issue**: #13

Post-filter validation for structurally invalid entities.

- [ ] 33.4.1: `dangling_entity` rule: topics with 0 pub (warning), services
  with 0 server (error), actions with 0 server (error)
- [ ] 33.4.2: Empty entity removal: topics with 0 pub + 0 sub, services
  with 0 server + 0 client
- [ ] 33.4.3: Empty scope group removal in `cleanup_dangling_refs`
- [ ] 33.4.4: Tests for each dangling case
- [ ] 33.4.5: Fixture with conditional nodes that create dangling topics

## 33.5: Satisfiability Checking

**In**: `src/ros-launch-manifest/` (check)
**Design issue**: #14

Verify no valid arg combination produces dangling entities.

- [ ] 33.5.1: Tier 1 — enumeration for ≤15 finite-domain args:
  - Collect bool + choices args
  - Cartesian product of valid values
  - For each: substitute → filter → check dangling
  - Report specific arg values that cause problems
- [ ] 33.5.2: `unreachable` rule — detect conditions that are always false
  for all valid arg values (e.g., `bool == 'wtf'`)
- [ ] 33.5.3: Tests: variant-complete manifest, variant-incomplete manifest,
  unreachable node detection
- [ ] 33.5.4: (Optional) Tier 2 — Z3 SMT solver for >15 finite-domain args:
  - Feature-gated: `--features z3`
  - Encode args as Z3 enum sorts, conditions as Z3 constraints
  - Query: "is there an assignment where this topic has 0 pub?"
  - Report counterexample

## 33.6: Update Design Docs

- [ ] 33.6.1: Update `launch-manifest.md` — finalize unified interface section
  (remove "Note: planned" markers)
- [ ] 33.6.2: Update `design-issues.md` — mark #11–14 as done
- [ ] 33.6.3: Update `autoware-contract` manifests to use new features

---

## Phase Order

```
33.1 (parser bug fix) ──────────────────────────┐
                                                │
33.2 (unified scope interface) ─────────────────┤
                                                │
33.3 (arg types) ──→ 33.4 (dangling checks) ───┤
                          │                     │
                     33.5 (satisfiability) ─────┤
                                                │
33.6 (doc update) ──────────────────────────────┘
```

33.1 is independent. 33.2 is independent. 33.3 must precede 33.5
(types enable enumeration). 33.4 must precede 33.5 (dangling checks
are what satisfiability verifies). 33.6 is last.

---

## Verification

```bash
# Manifest crate tests
cd src/ros-launch-manifest && cargo test

# Executor loader tests
cd src/play_launch && cargo test manifest_loader

# CLI integration tests
cd tests && cargo test --test manifest_check

# Autoware contracts
cd ~/repos/autoware-contract && just check
```
