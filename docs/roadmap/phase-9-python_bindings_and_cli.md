# Phase 9: Python Bindings & CLI Polish

**Status**: 📋 Planned
**Priority**: HIGH (enables external adoption)
**Dependencies**: Phase 8 Complete ✅

---

## Overview

Make the parser usable by Python/C++ ROS developers who don't write Rust:

1. **Python package** (`pip install play-launch-parser`) — native extension via PyO3 + maturin
2. **CLI polish** — stdout-first output, human-readable summary, proper exit codes

### Architecture Decision: Separate Crate (Option B)

The parser already uses PyO3 to *embed* Python (for executing `.py` launch files). Exposing the parser *to* Python from the same crate creates conflicts:

- `auto-initialize` must not fire when Python loads us as an extension module
- `cdylib` + `rlib` in one crate complicates downstream linking
- Mixing embedding and extension concerns in one `lib.rs` is fragile

**Solution**: A thin `play_launch_parser_python` crate in the same workspace that depends on `play_launch_parser` and contains only the `#[pymodule]` definition.

```
play_launch_parser/
├── src/play_launch_parser/     # Rust library + CLI binary (unchanged)
│   ├── Cargo.toml              # [lib] rlib only
│   ├── src/lib.rs              # pub fn parse_launch_file(...)
│   └── src/main.rs             # CLI binary
├── python/                     # NEW: PyO3 extension module
│   ├── Cargo.toml              # cdylib, depends on play_launch_parser
│   ├── src/lib.rs              # #[pymodule] — thin wrapper
│   └── pyproject.toml          # maturin build backend
├── Cargo.toml                  # workspace: members = ["src/play_launch_parser", "python"]
└── justfile                    # build-python, test-python recipes
```

---

## Phase 9.1: Python Extension Crate Setup

**Goal**: Scaffold `python/` crate, build a wheel with maturin, verify `import play_launch_parser` works.

### 9.1.1: Crate scaffolding

- [ ] Create `python/Cargo.toml`:
  ```toml
  [package]
  name = "play_launch_parser_python"
  version = "0.1.0"
  edition = "2024"

  [lib]
  name = "play_launch_parser"   # module name seen by Python
  crate-type = ["cdylib"]

  [dependencies]
  play_launch_parser = { path = "../src/play_launch_parser" }
  pyo3 = { version = "0.24", features = ["extension-module"] }
  serde_json = "1"
  ```
  Key: `features = ["extension-module"]` — this tells PyO3 we are loaded *by* Python, not embedding it. It disables `auto-initialize`.

- [ ] Create `python/src/lib.rs` with minimal module:
  ```rust
  use pyo3::prelude::*;

  #[pymodule]
  fn play_launch_parser(m: &Bound<'_, PyModule>) -> PyResult<()> {
      m.add_function(wrap_pyfunction!(parse_file, m)?)?;
      m.add_function(wrap_pyfunction!(parse_package, m)?)?;
      Ok(())
  }
  ```

- [ ] Create `python/pyproject.toml`:
  ```toml
  [build-system]
  requires = ["maturin>=1.0,<2.0"]
  build-backend = "maturin"

  [project]
  name = "play-launch-parser"
  requires-python = ">=3.10"
  license = "Apache-2.0"
  description = "High-performance ROS 2 launch file parser"
  ```

- [ ] Add `"python"` to workspace `members` in root `Cargo.toml`

### 9.1.2: Build & smoke test

- [ ] `cd python && maturin develop` succeeds
- [ ] `python -c "import play_launch_parser"` does not crash
- [ ] Add `build-python` and `test-python` recipes to justfile

### Criteria

- [ ] `maturin build --release` produces a `.whl`
- [ ] `pip install <wheel>` and `import play_launch_parser` works in a clean venv
- [ ] No conflict with the embedded Python in the library crate (the extension crate never calls `auto-initialize`)

---

## Phase 9.2: Core Python API

**Goal**: Expose `parse_file()` and `parse_package()` that return native Python dicts.

### Design

The Python API returns plain dicts (not custom classes) so the output is immediately JSON-serializable and familiar to ROS developers used to `json.load()`.

**Conversion strategy**: `RecordJson` → `serde_json::Value` → Python dict via PyO3's `pythonize` crate. This avoids manually mirroring every struct field and stays in sync automatically as `RecordJson` evolves.

```python
from play_launch_parser import parse_file, parse_package

# Parse a launch file by path
result = parse_file("/opt/ros/humble/share/demo_nodes_cpp/launch/talker_listener.launch.xml")
result = parse_file("/path/to/file.launch.xml", args={"vehicle_model": "sample"})

# Parse by ROS package name (searches AMENT_PREFIX_PATH)
result = parse_package("autoware_launch", "planning_simulator.launch.xml")
result = parse_package("autoware_launch", "planning_simulator.launch.xml", args={...})

# Result is a plain dict
result["node"]        # list[dict] — standalone nodes
result["container"]   # list[dict] — composable node containers
result["load_node"]   # list[dict] — composable nodes
result["scopes"]      # list[dict] — launch tree (which file each node comes from)
result["variables"]   # dict — resolved launch arguments
result["file_data"]   # dict
```

### 9.2.1: `parse_file()` function

- [ ] Implement `#[pyfunction]` wrapping `play_launch_parser::parse_launch_file()`
- [ ] Accept `path: &str` and optional `args: Option<HashMap<String, String>>`
- [ ] Convert `RecordJson` to Python dict via `pythonize` (add `pythonize = "0.24"` dep)
- [ ] Map Rust errors to `PyRuntimeError` with the original error message
- [ ] Test: parse a simple XML launch file, verify `result["node"]` is a list of dicts

### 9.2.2: `parse_package()` function

- [ ] Implement package lookup (reuse `find_launch_file()` logic from `main.rs`)
- [ ] Accept `package: &str, file: &str, args: Option<HashMap<String, String>>`
- [ ] Raise `FileNotFoundError` when package/file not found (with helpful message listing `AMENT_PREFIX_PATH`)
- [ ] Test: parse `demo_nodes_cpp` talker_listener if available, or a test fixture

### 9.2.3: Module metadata

- [ ] `play_launch_parser.__version__` — read from `Cargo.toml` via `env!("CARGO_PKG_VERSION")`
- [ ] Docstrings on module and functions (visible in `help(play_launch_parser)`)

### Criteria

- [ ] `parse_file()` returns a dict matching `record.json` schema
- [ ] `parse_package()` resolves via `AMENT_PREFIX_PATH`
- [ ] Errors produce readable Python exceptions (not Rust panics)
- [ ] `result["node"][0]["cmd"]` is a Python list of strings (not a JSON string)
- [ ] Round-trip: `json.dumps(parse_file(...))` produces valid JSON identical to the Rust CLI output

---

## Phase 9.3: CLI Polish

**Goal**: Make `play_launch_parser` CLI ergonomic for piping and scripting.

### Current state

The CLI (`main.rs`) writes to a file (`-o record.json`) and logs to stderr. This is designed for internal use by `play_launch`. External users expect stdout output and human-readable summaries.

### 9.3.1: stdout-first output

- [ ] Default output to stdout when `-o` is not specified
- [ ] Keep `-o <path>` for file output (backward compatible)
- [ ] Suppress info-level log lines when writing JSON to stdout (they go to stderr already, but verify)

### 9.3.2: `--format` option

- [ ] `--format json` (default) — full `record.json` output
- [ ] `--format summary` — human-readable table:
  ```
  Launch: planning_simulator.launch.xml (autoware_launch)
  
  Nodes:          46
  Containers:     15
  Composable:     54
  Scopes:         83
  Variables:      127
  
  Nodes:
    /sensing/lidar/pointcloud_preprocessor    pointcloud_preprocessor    autoware_pointcloud_preprocessor
    /planning/scenario_planning/...           ...                        ...
  ```
- [ ] `--format names` — one node FQN per line (for piping to `grep`, `wc`, etc.):
  ```
  /sensing/lidar/pointcloud_preprocessor
  /planning/scenario_planning/lane_driving/...
  ...
  ```

### 9.3.3: Exit codes

- [ ] `0` — success
- [ ] `1` — parse error (invalid XML, unresolved substitution, circular include)
- [ ] `2` — file/package not found

### 9.3.4: `--quiet` and `--verbose` cleanup

- [ ] `--quiet` suppresses all log output (only JSON/summary to stdout)
- [ ] `--verbose` enables debug-level logging to stderr
- [ ] Default: info-level to stderr (current behavior, but verify it doesn't mix with stdout JSON)

### Criteria

- [ ] `play_launch_parser file foo.launch.xml | jq .node` works (JSON to stdout, logs to stderr)
- [ ] `play_launch_parser launch pkg file --format summary` prints a readable table
- [ ] `play_launch_parser launch pkg file --format names | wc -l` gives the node count
- [ ] Exit code is 2 when package not found
- [ ] `-o record.json` still works (backward compatible)
- [ ] `play_launch_parser file foo.launch.xml -o record.json` writes file and prints summary to stderr

---

## Phase 9.4: Packaging & Distribution

**Goal**: Publish the Python package and Rust binary so users can install without building from source.

### 9.4.1: maturin wheel build

- [ ] `maturin build --release` produces wheels for `manylinux_2_35` (Ubuntu 22.04+)
- [ ] Wheel works on systems without Rust toolchain
- [ ] Add `[tool.maturin]` config for stripping, compatibility tag
- [ ] GitHub Actions workflow: build wheels on push to `main` (Linux x86_64 + aarch64)

### 9.4.2: PyPI publishing

- [ ] Package name: `play-launch-parser`
- [ ] `pip install play-launch-parser` in a clean venv, `import play_launch_parser` works
- [ ] README.md with quick-start example (parse a launch file in 3 lines)
- [ ] Add `project.urls` (repository, documentation) to `pyproject.toml`

### 9.4.3: CLI binary distribution

- [ ] `cargo install play_launch_parser` works from crates.io (optional — evaluate demand)
- [ ] Alternatively: include CLI binary in the Python wheel via maturin `[tool.maturin] binaries = true`
  ```
  pip install play-launch-parser
  play-launch-parser launch autoware_launch planning_simulator.launch.xml
  ```

### 9.4.4: Python CLI entry point

- [ ] Add `[project.scripts]` to `pyproject.toml`:
  ```toml
  [project.scripts]
  play-launch-parser = "play_launch_parser:main"
  ```
- [ ] Implement `main()` Python entry point that delegates to the Rust binary, OR
- [ ] Use maturin's `#[pyfunction] fn main()` to expose the CLI as a Python-callable function
- [ ] Verify `play-launch-parser --help` works after `pip install`

### Criteria

- [ ] `pip install play-launch-parser && python -c "from play_launch_parser import parse_file"` works on Ubuntu 22.04+ without Rust
- [ ] `pip install play-launch-parser && play-launch-parser --help` shows CLI usage
- [ ] Wheels build in CI for x86_64 and aarch64
- [ ] No runtime dependency on ROS (only needs `AMENT_PREFIX_PATH` set for package resolution)

---

## Phase 9.5: Documentation & Examples

**Goal**: A ROS developer can go from `pip install` to parsed output in under 5 minutes.

### 9.5.1: README

- [ ] Quick start (3-line Python example)
- [ ] CLI usage with examples
- [ ] Output schema description (what each field means)
- [ ] Comparison with `ros2 launch --print` (why this tool exists)

### 9.5.2: Python examples

- [ ] `examples/parse_autoware.py` — parse Autoware, print node names
- [ ] `examples/launch_diff.py` — diff two launch configurations (different args)
- [ ] `examples/find_node.py` — find which launch file defines a specific node

### 9.5.3: Type stubs

- [ ] `play_launch_parser.pyi` stub file for IDE autocompletion
- [ ] TypedDict definitions for `NodeRecord`, `ContainerRecord`, `LoadNodeRecord`, `ScopeEntry`
- [ ] Ship stubs in the wheel (`py.typed` marker)

### Criteria

- [ ] `help(play_launch_parser.parse_file)` shows useful docstring
- [ ] IDE (VS Code / PyCharm) shows type hints for return values
- [ ] Examples run without modification on a system with Autoware installed

---

## Summary

| Phase | Deliverable | Effort |
|-------|-------------|--------|
| 9.1   | Crate scaffold, maturin build | 1 day |
| 9.2   | `parse_file()`, `parse_package()` | 1-2 days |
| 9.3   | CLI stdout, `--format`, exit codes | 1 day |
| 9.4   | Wheels, PyPI, CI | 1-2 days |
| 9.5   | README, examples, type stubs | 1 day |
| **Total** | | **5-7 days** |

### Key Risk: PyO3 Embed vs Extension Conflict

The parser crate uses `pyo3` with `auto-initialize` to embed Python for `.py` launch file execution. The new `python/` crate uses `pyo3` with `extension-module`. These are **mutually exclusive** PyO3modes within a single binary, but safe across separate crates because:

- The library crate (`rlib`) embeds Python — it calls `Python::with_gil()` and relies on `auto-initialize` to start the interpreter if needed
- The extension crate (`cdylib`) is loaded *by* Python — the interpreter already exists, `extension-module` disables `auto-initialize`
- When Python imports the extension module and the extension calls `parse_launch_file()`, the library's `auto-initialize` is a no-op because the interpreter is already running (started by the importing Python process)
- The two never conflict at runtime because `auto-initialize` checks whether an interpreter exists before starting one

This is a well-documented PyO3 pattern. The separate crate structure makes the boundary explicit.
