# Phase 9: Python Bindings & CLI Polish

**Status**: ✅ Complete (9.1–9.5)
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

**Solution**: A thin `play_launch_parser_python` crate that depends on `play_launch_parser` and contains only the `#[pymodule]` definition. The extension crate is **excluded** from the parser workspace (conflicting PyO3 features) and built independently via maturin.

```
play_launch_parser/
├── crates/
│   ├── play_launch_parser/     # Rust library + CLI binary (unchanged)
│   │   ├── Cargo.toml          # [lib] rlib only
│   │   ├── src/lib.rs          # pub fn parse_launch_file(...)
│   │   └── src/main.rs         # CLI binary
│   └── python/                 # PyO3 extension module (excluded from workspace)
│       ├── Cargo.toml          # cdylib, own [workspace], depends on ../play_launch_parser
│       ├── src/lib.rs          # #[pymodule] — thin wrapper
│       └── pyproject.toml      # maturin + uv build config
├── Cargo.toml                  # workspace: members = [crates/play_launch_parser], exclude = [crates/python]
└── justfile                    # build-python, test-python recipes
```

### Build Tooling

- **uv** manages the Python venv and dev dependencies (`uv sync` + `uv run`)
- **maturin** builds the cdylib and packages the wheel
- **direnv** (`.envrc`) activates the venv automatically when entering the project
- `just build` runs `build-rust build-colcon build-python` — the wheel path is printed at the end

---

## Phase 9.1: Python Extension Crate Setup ✅

**Status**: ✅ Complete

### 9.1.1: Crate scaffolding

- [x] Create `crates/python/Cargo.toml` with `[workspace]` marker, cdylib, `extension-module`
- [x] Create `crates/python/src/lib.rs` with `#[pymodule]`
- [x] Create `crates/python/pyproject.toml` with maturin backend and uv dev deps
- [x] Exclude `crates/python` from parser workspace (PyO3 feature conflict with `auto-initialize`)

### 9.1.2: Build & smoke test

- [x] `uv sync && uv run maturin develop --release` succeeds
- [x] `uv run python -c "import play_launch_parser"` works
- [x] `just build-python` and `just test-python` recipes in justfile
- [x] `just build` includes `build-python` and prints wheel path
- [x] `.envrc` activates the uv-managed venv via direnv

### Implementation Notes

- The extension crate needs its own `[workspace]` in `Cargo.toml` to prevent cargo from walking up to the parent `play_launch` workspace
- `maturin develop` installs into the venv for local testing; `maturin build` produces the distributable wheel
- `build-python` recipe cleans `target/wheels/` before `maturin build` so only one wheel is listed

---

## Phase 9.2: Core Python API ✅

**Status**: ✅ Complete

### Design

The Python API returns plain dicts (not custom classes) so the output is immediately JSON-serializable and familiar to ROS developers used to `json.load()`.

**Conversion strategy**: `RecordJson` → `serde_json::Value` → Python dict via `pythonize` crate. This avoids manually mirroring every struct field and stays in sync automatically as `RecordJson` evolves.

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

- [x] Implement `#[pyfunction]` wrapping `play_launch_parser::parse_launch_file()`
- [x] Accept `path: &str` and optional `args: Option<HashMap<String, String>>`
- [x] Convert `RecordJson` to Python dict via `pythonize` (add `pythonize = "0.24"` dep)
- [x] Map Rust errors to `PyRuntimeError` with the original error message
- [x] `FileNotFoundError` raised for missing files
- [x] Test: parse a simple XML launch file, verify `result["node"]` is a list of dicts

### 9.2.2: `parse_package()` function

- [x] Implement package lookup (searches `AMENT_PREFIX_PATH`)
- [x] Accept `package: &str, file: &str, args: Option<HashMap<String, String>>`
- [x] Raise `FileNotFoundError` when package/file not found (with helpful message listing `AMENT_PREFIX_PATH`)
- [x] Raise `FileNotFoundError` when `AMENT_PREFIX_PATH` is not set

### 9.2.3: Module metadata

- [x] `play_launch_parser.__version__` — read from `Cargo.toml` via `env!("CARGO_PKG_VERSION")`
- [x] Docstrings on module and functions (visible in `help(play_launch_parser)`)

### Verified

- [x] `parse_file()` returns a dict matching `record.json` schema
- [x] `parse_package()` resolves via `AMENT_PREFIX_PATH`
- [x] Errors produce readable Python exceptions (not Rust panics)
- [x] `result["node"][0]["cmd"]` is a Python list of strings (not a JSON string)
- [x] Round-trip: `json.dumps(parse_file(...))` produces valid JSON

---

## Phase 9.3: CLI Polish ✅

**Status**: ✅ Complete

### 9.3.1: stdout-first output

- [x] Default output to stdout when `-o` is not specified
- [x] Keep `-o <path>` for file output (backward compatible)
- [x] All log output goes to stderr (via `env_logger::Target::Stderr`)

### 9.3.2: `--format` option

- [x] `--format json` (default) — full `record.json` output
- [x] `--format summary` — human-readable table (nodes, containers, composable, scopes, variables)
- [x] `--format names` — one node FQN per line (for piping to `grep`, `wc`, etc.)
- [x] Global args (`-f`, `-o`, `-v`, `-q`) work before or after the subcommand via `global = true`

### 9.3.3: Exit codes

- [x] `0` — success
- [x] `1` — parse error (invalid XML, unresolved substitution, circular include)
- [x] `2` — file/package not found

### 9.3.4: `--quiet` and `--verbose` cleanup

- [x] `--quiet` suppresses all log output (only JSON/summary to stdout)
- [x] `--verbose` enables debug-level logging to stderr
- [x] Default: info-level to stderr, does not mix with stdout output
- [x] Broken pipe handling (graceful when piping to `head`/`grep`)

### Verified

- [x] `play_launch_parser file foo.launch.xml | jq .node` works
- [x] `play_launch_parser file foo.launch.xml --format summary` prints a readable table
- [x] `play_launch_parser file foo.launch.xml --format names | wc -l` gives the node count
- [x] Exit code is 2 when package/file not found
- [x] `-o record.json` still works (backward compatible)

### Implementation Notes

CLI logic is implemented in both `crates/play_launch_parser/src/main.rs` (standalone Rust binary) and `crates/python/src/lib.rs` (Python entry point via `_cli_main()`). The Python version raises `SystemExit` instead of calling `process::exit()` to avoid killing the Python interpreter.

---

## Phase 9.4: Packaging & Distribution ✅

**Status**: ✅ Complete (local build; CI and PyPI publishing deferred)

### 9.4.1: maturin wheel build

- [x] `maturin build --release` produces wheels
- [x] `just build-python` runs `uv sync`, `maturin develop`, `maturin build`, prints wheel path
- [x] `[tool.maturin]` config with `python-source = "."` for mixed Rust+Python sources
- [ ] GitHub Actions workflow: build wheels on push to `main` (Linux x86_64 + aarch64)

### 9.4.2: PyPI publishing

- [ ] Publish to PyPI as `play-launch-parser`
- [x] `pip install <wheel>` and `import play_launch_parser` works
- [x] README.md with quick-start example
- [x] Add `project.urls` (repository) to `pyproject.toml`

### 9.4.3: CLI in Python wheel

- [x] CLI exposed as Python entry point via `[project.scripts]` — no native binary bundling needed
- [x] `_cli_main()` pyfunction implements full CLI (json/summary/names, exit codes)
- [x] `play_launch_parser/_cli.py` thin wrapper calls `_cli_main(sys.argv)`
- [x] `play_launch_parser/__init__.py` re-exports from native `.so` module

### 9.4.4: Python CLI entry point

- [x] `[project.scripts]` in `pyproject.toml`:
  ```toml
  [project.scripts]
  play-launch-parser = "play_launch_parser._cli:main"
  ```
- [x] `play-launch-parser --help` works after `pip install`
- [x] `play-launch-parser file <path> --format summary` works
- [x] `play-launch-parser file <path> | jq .` produces valid JSON

### Verified

- [x] `pip install <wheel> && python -c "from play_launch_parser import parse_file"` works
- [x] `pip install <wheel> && play-launch-parser --help` shows CLI usage
- [x] Wheel contains: `__init__.py`, `_cli.py`, native `.so`, `entry_points.txt`
- [x] No runtime dependency on ROS (only needs `AMENT_PREFIX_PATH` for package resolution)

---

## Phase 9.5: Documentation & Examples ✅

**Status**: ✅ Complete

### 9.5.1: README

- [x] Quick start (Python API example)
- [x] CLI usage with all flags, exit codes, examples
- [x] Output schema description (all fields for node, container, load_node, scopes)
- [x] Requirements section

### 9.5.2: Python examples

- [x] `examples/parse_autoware.py` — parse Autoware, print node names
- [x] `examples/launch_diff.py` — diff two launch configurations (different args)
- [x] `examples/find_node.py` — find which launch file defines a specific node

### 9.5.3: Type stubs

- [x] `play_launch_parser/__init__.pyi` stub file for IDE autocompletion
- [x] TypedDict definitions for `NodeRecord`, `ContainerRecord`, `LoadNodeRecord`, `ScopeEntry`, `ParseResult`
- [x] Ship stubs in the wheel (`py.typed` marker)

### Verified

- [x] `help(play_launch_parser.parse_file)` shows useful docstring
- [x] `__init__.pyi` and `py.typed` included in wheel
- [x] Type stubs provide IDE autocompletion for return values

---

## Summary

| Phase | Deliverable | Effort | Status |
|-------|-------------|--------|--------|
| 9.1   | Crate scaffold, maturin + uv build | 1 day | ✅ Complete |
| 9.2   | `parse_file()`, `parse_package()` | 1-2 days | ✅ Complete |
| 9.3   | CLI stdout, `--format`, exit codes | 1 day | ✅ Complete |
| 9.4   | Wheels, CLI entry point | 1-2 days | ✅ Complete |
| 9.5   | README, examples, type stubs | 1 day | ✅ Complete |
| **Total** | | **5-7 days** | |

### Key Risk: PyO3 Embed vs Extension Conflict

The parser crate uses `pyo3` with `auto-initialize` to embed Python for `.py` launch file execution. The new `crates/python/` crate uses `pyo3` with `extension-module`. These are **mutually exclusive** PyO3 features, handled by:

- The extension crate is **excluded** from the parser workspace and has its own `[workspace]` marker
- The library crate (`rlib`) embeds Python — it calls `Python::with_gil()` and relies on `auto-initialize` to start the interpreter if needed
- The extension crate (`cdylib`) is loaded *by* Python — the interpreter already exists, `extension-module` disables `auto-initialize`
- When Python imports the extension module and the extension calls `parse_launch_file()`, the library's `auto-initialize` is a no-op because the interpreter is already running
- `cargo build --all-targets` in the parser workspace never touches the extension crate (it's excluded), avoiding linker errors from undefined Python symbols
