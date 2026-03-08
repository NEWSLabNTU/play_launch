# Phase 28: Parser Integration Test Coverage

**Status**: Complete
**Priority**: Medium (Quality / Reliability)
**Dependencies**: Phase 13 (Rust Parser), Phase 14 (Python Execution), Phase 22 (Launch IR)

---

## Overview

The Rust parser had 417 tests across unit, integration, and round-trip categories, but coverage was uneven across the three launch file formats (XML, Python, YAML). XML tests used fixture files for realistic scenarios; Python tests had extensive fixtures; YAML tests used only inline strings. Some actions lacked dedicated integration tests in certain formats.

This phase closed the gaps by splitting test files by format, adding fixture-based integration tests for YAML, filling missing action coverage for XML and Python, and adding cross-format integration tests.

---

## Results

### Test file inventory (after)

| Test file                    | Tests | Style                       |
|------------------------------|-------|-----------------------------|
| `tests/xml_tests.rs`         | 25    | Fixture-based + inline XML  |
| `tests/yaml_tests.rs`        | 50    | 42 inline + 8 fixture-based |
| `tests/python_tests.rs`      | 38    | Mostly fixture-based        |
| `tests/edge_cases.rs`        | 18    | Inline XML                  |
| `tests/ir_tests.rs`          | 22    | Inline                      |
| `tests/ir_eval_tests.rs`     | 20    | Inline                      |
| `tests/integration_tests.rs` | 3     | Fixture-based (perf)        |
| Unit tests (lib)             | 237   | Inline                      |

**Total**: 413 parser tests + 30 integration tests

### Fixture files added

- `test_yaml_all_actions.launch.yaml` — arg, node, group, let, set_env, unset_env, push-ros-namespace, set_parameter, set_remap
- `test_yaml_container.launch.yaml` — node_container with composable_node children, params, remaps
- `test_yaml_load_composable_node.launch.yaml` — container + load_composable_node with target
- `test_yaml_executable.launch.yaml` — executable action with env and args
- `test_yaml_preset.launch.yaml` — Autoware-style preset (declares args only)
- `test_yaml_conditions.launch.yaml` — if/unless on arg, node, group
- `test_yaml_nested_groups.launch.yaml` — nested groups + $(dirname) resolution
- `test_xml_set_remap.launch.xml` — set_remap followed by node
- `test_xml_executable.launch.xml` — executable action
- `test_xml_set_parameter.launch.xml` — set_parameter followed by node
- `test_python_set_remap_global.launch.py` — Node with remappings
- `test_yaml_include_python.launch.yaml` + `test_yaml_include_python_target.launch.py` — YAML→Python cross-format
- `test_python_include_yaml.launch.py` + `test_python_include_yaml_target.launch.yaml` — Python→YAML cross-format
- `test_xml_yaml_preset_chain.launch.xml` — XML→YAML preset chain

---

## Work Items

### 28.0: Split test files by format ✅

- [x] Created `tests/yaml_tests.rs` with all 42 `test_yaml_*` functions from `xml_tests.rs`
- [x] Moved `write_yaml()` helper to `yaml_tests.rs`
- [x] Duplicated `get_fixture_path()` helper (matches existing pattern)
- [x] Removed moved functions from `xml_tests.rs`
- [x] `xml_tests.rs` contains 21 XML tests, `yaml_tests.rs` contains 42 YAML tests
- [x] `just test` passes with no test count regression

### 28.1: YAML fixture files ✅

- [x] `test_yaml_all_actions.launch.yaml`
- [x] `test_yaml_container.launch.yaml`
- [x] `test_yaml_load_composable_node.launch.yaml`
- [x] `test_yaml_executable.launch.yaml`
- [x] `test_yaml_preset.launch.yaml`
- [x] `test_yaml_conditions.launch.yaml`
- [x] `test_yaml_nested_groups.launch.yaml`

### 28.2: YAML fixture-based integration tests ✅

- [x] `test_yaml_fixture_all_actions`
- [x] `test_yaml_fixture_container`
- [x] `test_yaml_fixture_load_composable_node`
- [x] `test_yaml_fixture_executable`
- [x] `test_yaml_fixture_preset_pattern`
- [x] `test_yaml_fixture_conditions`
- [x] `test_yaml_fixture_dirname`

### 28.3: XML integration test gaps ✅

- [x] `test_xml_set_remap` fixture + test
- [x] `test_xml_executable` fixture + test
- [x] `test_xml_set_parameter` fixture + test
- [x] Parser features table updated: `set_remap` and `executable` now `T U` for XML

### 28.4: Python integration test ✅

- [x] `test_python_set_remap_global` — tests Node remappings (SetRemap not available in ROS Humble; used Node(remappings=[...]) instead)

### 28.5: Cross-format integration tests ✅

- [x] `test_yaml_include_python` — YAML includes Python, captures nodes with namespace
- [x] `test_python_include_yaml` — Python includes YAML preset, sees declared variables
- [x] `test_xml_include_yaml_preset_then_use` — XML→YAML preset→XML variable resolution

### 28.6: Documentation updates ✅

- [x] `docs/guide/parser-features.md` — `set_remap` and `executable` XML column updated to `T U`
- [x] `CLAUDE.md` — test count updated to 413
- [x] `src/play_launch_parser/CLAUDE.md` — test counts and file inventory updated

---

## Verification

```
just test    → 413 parser tests + 30 integration tests, all pass
just check   → zero clippy warnings
cargo test --test xml_tests    → 25 tests
cargo test --test yaml_tests   → 50 tests
cargo test --test python_tests → 38 tests
```
