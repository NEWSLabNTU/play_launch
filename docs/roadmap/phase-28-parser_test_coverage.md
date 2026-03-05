# Phase 28: Parser Integration Test Coverage

**Status**: Planned
**Priority**: Medium (Quality / Reliability)
**Dependencies**: Phase 13 (Rust Parser), Phase 14 (Python Execution), Phase 22 (Launch IR)

---

## Overview

The Rust parser has 417 tests across unit, integration, and round-trip categories, but coverage is uneven across the three launch file formats (XML, Python, YAML). XML tests use fixture files for realistic scenarios; Python tests have extensive fixtures; YAML tests use only inline strings. Some actions lack dedicated integration tests in certain formats.

This phase closes the gaps by splitting test files by format, adding fixture-based integration tests for YAML, filling missing action coverage for XML and Python, and adding cross-format integration tests.

---

## Current State

### Test file inventory

| Test file | Tests | Style |
|-----------|-------|-------|
| `tests/xml_tests.rs` | 63 | 23 fixture-based XML + 40 inline YAML |
| `tests/python_tests.rs` | 36 | Mostly fixture-based |
| `tests/edge_cases.rs` | 18 | Inline XML |
| `tests/ir_tests.rs` | 20 | Inline |
| `tests/ir_eval_tests.rs` | 22 | Inline |
| `tests/integration_tests.rs` | 3 | Fixture-based (perf) |
| Unit tests (lib) | 237 | Inline |
| WASM round-trip | 18 | Fixture-based |

### Fixture file inventory

- XML: 17 fixture files in `tests/fixtures/launch/`
- Python: 16 fixture files + 13 in `tests/fixtures/launch/python/`
- YAML: **0 fixture files** (all inline via `write_yaml()`)
- Includes: 2 files in `tests/fixtures/includes/`

### Coverage gaps by action

| Action | XML gap | Python gap | YAML gap |
|--------|---------|------------|----------|
| `set_remap` | No integration test (U only) | -- | -- |
| `executable` | No integration test (U only) | -- | -- |
| `set_parameter` | No dedicated test | -- | -- |
| Cross-format include | XML->Py tested | Py->XML tested | YAML->Py **not tested** |
| `$(dirname)` in YAML | N/A | N/A | Cannot test with inline (tmpdir) |
| Fixture-based YAML | N/A | N/A | **No fixtures exist** |

---

## Work Items

### 28.0: Split test files by format

Split `xml_tests.rs` (63 tests: 23 XML + 40 YAML) into separate files so each format has its own test file. This must be done **before** all subsequent work items so new tests land in the correct file.

- [ ] Create `tests/yaml_tests.rs` with all `test_yaml_*` functions from `xml_tests.rs`
- [ ] Move `write_yaml()` helper to `yaml_tests.rs`
- [ ] Keep shared helpers (`get_fixture_path`, `get_includes_fixture_path`) accessible -- either duplicate or extract to a `tests/common/mod.rs`
- [ ] Remove moved functions from `xml_tests.rs`
- [ ] Verify `xml_tests.rs` contains only XML tests (~23)
- [ ] Verify `yaml_tests.rs` contains all YAML tests (~40)
- [ ] `just test` passes with no test count regression (417 total)

**Acceptance criteria:**
- [ ] `cargo test --test xml_tests` runs only XML tests
- [ ] `cargo test --test yaml_tests` runs only YAML tests
- [ ] Total test count unchanged (417)
- [ ] `just check` passes (zero clippy warnings, formatting clean)

### 28.1: YAML fixture files

Create fixture files in `tests/fixtures/launch/` to enable fixture-based YAML tests.

- [ ] `test_yaml_all_actions.launch.yaml` -- exercises arg, node, include, group, let, set_env, unset_env, push-ros-namespace, set_parameter, set_remap in one file
- [ ] `test_yaml_container.launch.yaml` -- node_container with multiple composable_node children, params, remaps
- [ ] `test_yaml_load_composable_node.launch.yaml` -- container + load_composable_node with target reference
- [ ] `test_yaml_executable.launch.yaml` -- executable action with env and args children
- [ ] `test_yaml_preset.launch.yaml` -- Autoware-style preset (declares args only, meant to be included by XML)
- [ ] `test_yaml_conditions.launch.yaml` -- if/unless on multiple action types
- [ ] `test_yaml_nested_groups.launch.yaml` -- nested groups with namespace stacking

**Acceptance criteria:**
- [ ] Each fixture file parses without error via `parse_launch_file()`
- [ ] Files are self-contained (no external package dependencies like `demo_nodes_cpp`)

### 28.2: YAML fixture-based integration tests

Add tests to `yaml_tests.rs` that load the fixture files from 28.1.

- [ ] `test_yaml_fixture_all_actions` -- parse `test_yaml_all_actions.launch.yaml`, verify node/container/load_node counts and field values
- [ ] `test_yaml_fixture_container` -- parse container fixture, verify container record fields, composable node params/remaps
- [ ] `test_yaml_fixture_load_composable_node` -- verify load_node records with target container name
- [ ] `test_yaml_fixture_executable` -- verify executable produces node record with correct cmd
- [ ] `test_yaml_fixture_preset_pattern` -- XML includes YAML preset, subsequent include uses preset variable (tests parent scope modification with real files)
- [ ] `test_yaml_fixture_conditions` -- parse with different arg overrides, verify correct nodes selected
- [ ] `test_yaml_fixture_dirname` -- verify `$(dirname)` resolves to fixture directory (not /tmp)

**Acceptance criteria:**
- [ ] All fixture-based tests pass
- [ ] `$(dirname)` test confirms resolution to `tests/fixtures/launch/` not a temp dir
- [ ] Preset pattern test confirms YAML variables are visible in parent XML scope

### 28.3: XML integration test gaps

Add dedicated XML fixtures and integration tests to `xml_tests.rs` for actions that only have unit tests.

- [ ] `test_xml_set_remap.launch.xml` fixture -- set_remap followed by node, verify remap in cmd output
- [ ] `test_xml_executable.launch.xml` fixture -- executable action with name and output
- [ ] `test_xml_set_parameter.launch.xml` fixture -- set_parameter followed by node, verify global_params
- [ ] `test_xml_set_remap` integration test
- [ ] `test_xml_executable` integration test
- [ ] `test_xml_set_parameter` integration test

**Acceptance criteria:**
- [ ] `set_remap` test verifies remap appears in node cmd as `from:=to`
- [ ] `executable` test verifies record goes into `node[]` array with correct cmd
- [ ] `set_parameter` test verifies global_params backfilled onto node records
- [ ] Parser features table updated from `U` to `T U` for these actions

### 28.4: Python integration test gap

Add to `python_tests.rs`:

- [ ] `test_python_set_remap_global` -- test `SetRemap` action sets global remap applied to subsequent nodes (currently only `test_remap_logging` tests node-level remaps)

**Acceptance criteria:**
- [ ] Test verifies global remap appears in node cmd output

### 28.5: Cross-format integration tests

Test format combinations not currently covered. Add YAML cross-format tests to `yaml_tests.rs`, Python cross-format tests to `python_tests.rs`.

- [ ] `test_yaml_include_python` -- YAML file includes a `.launch.py` file, verify nodes from Python are captured
- [ ] `test_python_include_yaml` -- Python file includes a `.launch.yaml` file, verify YAML arg declarations are visible
- [ ] `test_xml_include_yaml_preset_then_use` -- XML includes YAML preset, then uses variables it declared in a subsequent include (Autoware preset pattern with fixture files)

**Acceptance criteria:**
- [ ] YAML->Python include produces correct node records
- [ ] Python->YAML include demonstrates parent scope modification
- [ ] XML->YAML->XML preset chain resolves variables correctly

### 28.6: Documentation updates

- [ ] Update `docs/guide/parser-features.md` action table -- change `U` to `T U` where integration tests are added
- [ ] Update test counts in `docs/guide/parser-features.md`
- [ ] Update test counts in `CLAUDE.md`

---

## Verification

```bash
just test           # All tests pass, no regressions
just check          # Zero clippy warnings
just format         # No formatting diff
```

- [ ] Total test count increases (target: ~430+)
- [ ] All existing 417 tests still pass
- [ ] `docs/guide/parser-features.md` action table has no remaining `U`-only entries for XML
- [ ] Each format has its own test file: `xml_tests.rs`, `yaml_tests.rs`, `python_tests.rs`
