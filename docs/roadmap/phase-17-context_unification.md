# Phase 17: Context Unification

**Status**: Phases 17.1–17.6 complete, 17.7 (remaining parity) next
**Started**: 2026-02-06
**Priority**: High (Affects parser correctness and maintainability)

## Overview

Unify all parser state into a single `LaunchContext` structure, eliminating global statics and the duplicate `ParseContext` type. Then fix remaining parser discrepancies to achieve full Rust–Python parity.

## Completed Work

### Phase 17.1: Architecture Design (COMPLETE)

Delivered `docs/context_unification_design.md` with field mapping, migration plan, and decision to use LaunchContext's fully-qualified namespace semantics.

### Phase 17.2: LaunchContext Extended (COMPLETE)

Created `src/captures.rs` module with `NodeCapture`, `ContainerCapture`, `LoadNodeCapture`, `IncludeCapture`. Added 4 capture fields + 12 methods to LaunchContext. Captures are local-only (not inherited by `child()`).

### Phase 17.3 + 17.4: Unified Context Migration (COMPLETE)

- `LaunchTraverser.parse_context` field removed; uses single `context: LaunchContext`
- Thread-local changed from `ParseContext` to `*mut LaunchContext`
- All namespace sync code removed from `execute_python_file()`
- All capture/namespace operations use `self.context` only

### Phase 17.5: Global Static Removal — GLOBAL_PARAMETERS (COMPLETE)

- `GLOBAL_PARAMETERS` static deleted from `bridge.rs`
- `SetParameter` writes to LaunchContext via `with_launch_context()`
- `to_record()` methods accept `&Option<Vec<(String, String)>>` instead of reading globals
- XML actions read global params from context parameter
- `ParseContext` type deleted (`context.rs` removed, `pub use` removed)
- 306 tests pass, zero clippy warnings

### Phase 17.5.5: Eliminate LAUNCH_CONFIGURATIONS (COMPLETE)

Removed the last global static. All 10 usage sites (2 writers, 8 readers) migrated to `with_launch_context()`. `PythonLaunchExecutor` simplified to unit struct, `parking_lot` dependency removed. 306 tests pass, zero clippy warnings.

### Phase 17.6: Parser Parity Fixes (COMPLETE)

Fixed 6 concrete bugs in record generation. All 306 tests pass.

#### Changes Made

**`record/generator.rs`** — Fixes 1, 3, 4, 5, 6:
- **Fix 1**: `exec_name` always uses `executable` (was preferring node `name`)
- **Fix 3**: `__ns` only added to cmd when namespace is non-root (was always `__ns:=/`)
- **Fix 4**: Global parameters added as `-p key:=value` entries in cmd
- **Fix 5**: `--params-file` entries added to cmd for non-temp param files
- **Fix 6**: `resolve_executable_path()` uses `find_package_executable()` with fallback
- **Refactor**: `generate_node_record()` builds cmd inline via `build_node_command()` using already-resolved values; `generate_node_command()` delegates to `generate_node_record()`

**`python/bridge.rs`** — Fix 2 + shared helper:
- **Fix 2**: Container `exec_name` uses `self.name` (was `format!("{}-1", executable)`)
- `find_package_executable()` changed to `pub(crate)` for cross-module use
- Container cmd: exec path uses `find_package_executable()` with fallback; `__ns` only added when non-root

**`actions/container.rs`** — Consistent with above:
- Both `to_container_record()` and `to_node_record()`: exec path via `find_package_executable()`; `__ns` only added when non-root

#### Autoware Verification (46 nodes, 15 containers, 54 load_nodes)

| Fix                               | Metric                             | Result                   |
|-----------------------------------|------------------------------------|--------------------------|
| Fix 1: exec_name = executable     | exec_name matches executable field | **46/46**                |
| Fix 2: container exec_name = name | Old `-1` format eliminated         | **0/15** have old format |
| Fix 3: `__ns:=/` eliminated       | Root namespace in cmd              | **0/46** (was 46/46)     |
| Fix 4: global params in cmd       | Nodes with `use_sim_time` in cmd   | **32/46**                |
| Fix 5: `--params-file` in cmd     | Nodes with `--params-file`         | **19/46** (was 0)        |
| Fix 6: exec path resolution       | Resolved via AMENT_PREFIX_PATH     | **44/46** (was 0)        |

## Phase 17.7: Remaining Parser Parity Gaps

Entity counts match (46 nodes, 15 containers, 54 load_nodes). No exact node matches yet — every node has at least one remaining difference. The remaining gaps fall into 6 categories.

### 1. Global params missing from 14 XML nodes' cmd

**Affected**: 14/46 nodes (the 32 Python-captured nodes are correct)

**Root cause**: XML nodes are parsed into `NodeRecord` inline via `generate_node_record()`. At parse time, `context.global_parameters()` may be empty because `SetParameter` actions run in Python launch files included later. The `global_params` record field is backfilled for captures in `into_record_json()`, but the cmd is built at parse time and never updated.

**Fix approach**: Either (a) rebuild cmd for XML NodeRecords during the backfill step in `into_record_json()`, or (b) defer cmd generation entirely until all parsing is complete. Option (a) is simpler — iterate XML records and insert `-p key:=value` entries before the cmd's end.

### 2. Namespace field differs for 23 nodes

**Affected**: 23/46 nodes

**Breakdown**:
- 15 have different namespace values (e.g., Rust=`/` vs Python=`/system`)
- 4 Rust has None where Python has a namespace
- 4 Rust has a namespace where Python has None

**Root cause**: XML namespace resolution doesn't fully propagate group/include namespace context. Python launch files set namespace via `PushRosNamespace` which updates the thread-local LaunchContext, but the resolved namespace at parse time for XML nodes may not reflect the full include chain.

**Fix approach**: Investigate how namespace context flows through XML include chains. The namespace should accumulate through `<group>` and `<include>` scoping in the same way Python's `PushRosNamespace` does.

### 3. exec_name differs for 36 nodes

**Affected**: 36/46 nodes

**Breakdown**:
- 7 nodes: Rust executable missing `_node` suffix (e.g., `traffic_light_map_based_detector` vs `traffic_light_map_based_detector_node`)
- 29 nodes: Other naming differences between Rust's `executable` field and Python's `exec_name`

**Root cause**: Fix 1 correctly changed exec_name to always use the `executable` field. But the `executable` field itself differs from Python for these nodes — either the XML `exec` attribute is being parsed differently, or Python resolves executable names through a different mechanism (e.g., package metadata lookup).

**Fix approach**: Compare how Rust extracts the `exec` attribute from XML vs how Python's `Node` class resolves the executable name. May need to match Python's resolution logic.

### 4. params_files content differs for 28 nodes

**Affected**: 28/46 nodes

**Root cause**: Two distinct sub-issues:
- **Representation**: Rust stores resolved YAML content in `params_files[]`; Python extracts YAML params as inline `params[]` entries. Same data, different location.
- **Wrong file**: Some nodes reference a different YAML file entirely (e.g., map config instead of node-specific config). This indicates incorrect param file path resolution during include processing.

**Fix approach**: Align param file handling — either extract YAML to inline params (matching Python) or keep as files but ensure the correct file paths are resolved. The Python parser's `file_data` field (19 entries, empty in Rust) is related — it stores the raw param file contents.

### 5. Container ordering mismatch (9 containers shifted)

**Affected**: 9/15 containers (C4–C12)

**Symptom**: Container exec_names are correct individually but shifted by one position. Container 4 in Rust = Container 5 in Python, etc.

**Root cause**: A container is captured at a different position in the parsing sequence between Rust and Python. Likely one container is captured during XML processing in Rust but during Python processing in Python (or vice versa), causing all subsequent containers to shift.

**Fix approach**: Compare the capture order of containers between parsers. Identify which container appears at a different position and why. May be related to include processing order.

### 6. Executable path fallback for 2 nodes

**Affected**: 2/46 nodes still use `/opt/ros/humble/lib/` fallback

**Root cause**: These 2 packages are not installed in any AMENT_PREFIX_PATH location. Either the package name doesn't match the installed directory, or the executable name doesn't match the installed binary.

**Fix approach**: Log which packages fail resolution and verify against the actual filesystem. Low priority — affects only path format, not functionality.

### Priority Order

1. **Global params in XML cmd** (14 nodes) — Backfill during `into_record_json()`, straightforward
2. **Namespace resolution** (23 nodes) — Deepest architectural issue, highest parity impact
3. **params_files representation** (28 nodes) — Align with Python's inline extraction
4. **Container ordering** (9 containers) — Find the shifted container
5. **exec_name resolution** (36 nodes) — Match Python's executable resolution
6. **Exec path fallback** (2 nodes) — Low priority

## References

- `src/substitution/context.rs` — LaunchContext (unified context)
- `src/python/bridge.rs` — Thread-local context, capture helpers, `find_package_executable()`
- `src/python/executor.rs` — PythonLaunchExecutor (unit struct)
- `src/record/generator.rs` — `build_node_command()`, `resolve_executable_path()`
- `src/actions/container.rs` — XML container record generation
- `docs/context_unification_design.md` — Architecture document
