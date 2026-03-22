# Security Issues

Last updated: 2026-03-22

## Critical

### S1. `$(eval ...)` arbitrary code execution — RESOLVED

- **File**: `src/play_launch_parser/.../substitution/eval.rs`
- **Risk**: `py.eval()` called on expressions from launch files. A
  malicious launch file could run `$(eval __import__('os').system(...))`.
- **Fix**: `python_eval_fallback()` now passes restricted globals with a
  whitelist of safe builtins (arithmetic, string ops, type constructors).
  `__import__`, `exec`, `eval`, `open`, `compile` are blocked.
- **Test**: Autoware `$(eval)` expressions work unchanged.

### S2. `$(command ...)` arbitrary shell execution — RESOLVED

- **File**: `src/play_launch_parser/.../substitution/types.rs`
- **Risk**: `$(command)` runs arbitrary bash commands. Standard ROS 2
  feature.
- **Fix**: Enabled by default with `warn!` log. `--block-commands` CLI
  flag rejects all `$(command)` substitutions.
- **Test**: Autoware uses one `$(command xacro ...)` — correctly warns.

### S3. Python launch file arbitrary execution — DOCUMENTED

- **File**: `src/play_launch_parser/.../python/executor.rs`
- **Risk**: `.launch.py` files execute with `py.run()`. Inherent to
  ROS 2 — Python launch files are programs by design.
- **Status**: Cannot sandbox without breaking compatibility. All ROS 2
  tools (`ros2 launch`, `launch_testing`) execute them the same way.
- **Mitigation**: Only parse launch files from trusted sources.

## High

### S4. SSE broadcast blocks on slow subscriber — RESOLVED

- **File**: `src/play_launch/src/web/broadcaster.rs`
- **Risk**: `tx.send(event).await` on bounded `mpsc` channel blocks the
  broadcaster when any subscriber's queue is full — one slow SSE client
  stalls all state event delivery.
- **Fix**: Replaced `Vec<mpsc::Sender>` + `TokioMutex` with
  `tokio::sync::broadcast`. Sender is synchronous and never blocks.
  Slow subscribers receive `RecvError::Lagged(n)` — the SSE handler
  sends a `refresh` event so the client re-fetches full state.
  Buffer: 512 (state), 32 (metrics).
- **Files changed**: `broadcaster.rs`, `metrics_broadcaster.rs`, `sse.rs`,
  `sse.js`, `common.rs`, `replay.rs`, `resource_monitor.rs`.

### S5. Integer overflow in SPSC ring buffer allocation — RESOLVED

- **File**: `src/spsc_shm/src/lib.rs:122`
- **Risk**: `capacity * element_size` can overflow, causing small mmap
  followed by out-of-bounds writes.
- **Fix**: Uses `checked_mul` + `checked_add`, returns `io::Error` on
  overflow.

### S6. Unsafe raw pointer for thread-local context — RESOLVED

- **File**: `src/play_launch_parser/.../python/bridge.rs:292-362`
- **Risk**: `CURRENT_LAUNCH_CONTEXT` stores raw `*mut LaunchContext`.
  `get_current_launch_context()` was `pub`, allowing external code to
  cache the raw pointer and dereference after the guard drops.
- **Fix**: Made `get_current_launch_context()` private (`fn` not
  `pub fn`). External access only through `with_launch_context()` and
  `try_with_launch_context()`, which safely scope the borrow within
  a closure lifetime.

### S7. No include depth limit — RESOLVED

- **File**: `src/play_launch_parser/.../traverser/include.rs`
- **Risk**: Non-circular but deeply nested includes cause stack overflow.
- **Fix**: Configurable `max_include_depth` (default: 100) on
  `LaunchTraverser`. New `ParseOptions` struct and
  `parse_launch_file_with_options()` public API. Returns
  `ParseError::MaxIncludeDepthExceeded` when exceeded. Propagated to
  all child traversers (XML, Python, YAML, IR paths).
- **Test**: `test_max_include_depth_exceeded` in `scope_tests.rs`.

### S8. No file size limit on cached files — RESOLVED

- **File**: `src/play_launch_parser/.../file_cache.rs`
- **Risk**: Large files read into memory. Cache has no eviction.
- **Fix**: `metadata.len()` checked before `read_to_string()` — files
  exceeding 10 MB are rejected with `io::Error`. Cache entry cap not
  needed (bounded by include depth limit × file count, ~100 entries).

## Medium

### S9. CORS policy — RESOLVED

- **File**: `src/play_launch/src/web/mod.rs`
- **Risk**: Was `allow_origin(Any)` in all configurations.
- **Fix**: CORS restricted to bind address. `127.0.0.1` → localhost
  only. `0.0.0.0` → any origin (user explicitly opts in). Specific
  IP → that IP + localhost.

### S10. SSE connection exhaustion — RESOLVED

- **File**: `src/play_launch/src/web/sse.rs`
- **Risk**: No limit on concurrent SSE subscribers.
- **Fix**: Both SSE endpoints check `subscriber_count() >= 50` before
  creating a new receiver. Returns 503 "Too many SSE connections" when
  the limit is reached.

### S11. YAML parser deprecated — RESOLVED

- **File**: `src/play_launch_parser/.../Cargo.toml`
- **Risk**: `serde_yaml` 0.9 is deprecated, had past CVEs.
- **Fix**: Migrated to `serde_yml` 0.0.12 (drop-in replacement).

### S12. Include path validation — RESOLVED

- **File**: `src/play_launch_parser/.../traverser/include.rs`
- **Risk**: Include paths could reference non-launch files.
- **Fix**: `validate_include_path()` rejects files without `.xml`,
  `.py`, `.yaml`, `.yml` extensions. Applied in both XML and Python
  include paths.
- **Test**: `test_include_path_validation_blocks_non_launch`.

### S13. Interception hooks return error on missing runtime — NOT APPLICABLE

- **File**: `src/play_launch_interception/src/lib.rs`
- **Risk**: When `librcl.so` unresolvable, hooks return `RCL_RET_ERROR`.
- **Status**: The interception `.so` is only `LD_PRELOAD`-ed into ROS
  processes that already link `librcl.so`. If `librcl.so` is missing,
  the process fails to start before hooks execute. Returning
  `RCL_RET_ERROR` is correct for the impossible case.

### S14. Global package cache no eviction — NOT APPLICABLE

- **File**: `src/play_launch_parser/.../substitution/types.rs`
- **Risk**: Stale cache entries in long-running processes.
- **Status**: The parser is single-invocation — `parse_launch_file()`
  runs once at startup, not as a persistent service. Package paths
  don't change while a ROS system is running. If the parser ever
  becomes long-lived, a `clear_caches()` API is trivial to add.

### S15. `#![allow(unsafe_op_in_unsafe_fn)]` crate-wide — RESOLVED

- **File**: `src/play_launch_parser/.../lib.rs:6`
- **Risk**: Suppresses useful lint for entire crate, not just PyO3.
- **Fix**: Removed. The allow was for PyO3 0.20; current PyO3 0.24
  generates clean code. Build and all 389 tests pass without it.

### S16. Process tree scan per-recursion — ACCEPTED

- **File**: `src/play_launch/src/process/tree.rs`
- **Risk**: O(depth) full process table scans in `find_all_descendants`.
- **Status**: Accepted as low-risk. Called once at shutdown only, tree
  depth is ~3 (play_launch → container → node), total cost ~50ms.
  The recursive approach is actually more correct for a dynamic tree
  (processes may spawn/exit during shutdown) — a single upfront
  snapshot could miss late-spawned processes. Cancellation token
  already mitigates runaway scans.

## Positive Findings

| Area              | Finding                                                 |
|-------------------|---------------------------------------------------------|
| Static assets     | `rust_embed` prevents path traversal                    |
| SPSC ordering     | Correct Acquire/Release, cache-line padding             |
| Channels          | Bounded with appropriate backpressure                   |
| Orphan prevention | `PR_SET_PDEATHSIG(SIGKILL)` on all children             |
| Actor deadlocks   | Unidirectional channels, no cycles                      |
| XML parsing       | `roxmltree` ignores DTD/entities — no XXE               |
| Variable depth    | `MAX_RESOLUTION_DEPTH = 20` prevents infinite recursion |
| Log streaming     | Paths from member registry, not URL parameters          |
