# Security Issues

Last updated: 2026-03-20

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

### S4. SSE broadcast blocks on slow subscriber — OPEN

- **File**: `src/play_launch/src/web/broadcaster.rs:52`
- **Risk**: `tx.send(event).await` blocks on full channels. One slow
  SSE client stalls all state event delivery.
- **Fix**: Replace with `tx.try_send()` or `tokio::sync::broadcast`.

### S5. Integer overflow in SPSC ring buffer allocation — OPEN

- **File**: `src/spsc_shm/src/lib.rs:122`
- **Risk**: `capacity * element_size` can overflow, causing small mmap
  followed by out-of-bounds writes.
- **Fix**: Use `checked_mul` + `checked_add`.

### S6. Unsafe raw pointer for thread-local context — OPEN

- **File**: `src/play_launch_parser/.../python/bridge.rs:292-362`
- **Risk**: `CURRENT_LAUNCH_CONTEXT` stores raw `*mut LaunchContext`.
  `get_current_launch_context()` is `pub`, allowing stale pointer cache.
- **Fix**: Make getter private. Add debug assertion in guard constructor.

### S7. No include depth limit — OPEN

- **File**: `src/play_launch_parser/.../traverser/include.rs`
- **Risk**: Non-circular but deeply nested includes cause stack overflow.
- **Fix**: Add `MAX_INCLUDE_DEPTH = 100` check.

### S8. No file size limit on cached files — OPEN

- **File**: `src/play_launch_parser/.../file_cache.rs`
- **Risk**: Large files read into memory. Cache has no eviction.
- **Fix**: Max file size (10MB) + max cache entries (1000).

## Medium

### S9. CORS policy — RESOLVED

- **File**: `src/play_launch/src/web/mod.rs`
- **Risk**: Was `allow_origin(Any)` in all configurations.
- **Fix**: CORS restricted to bind address. `127.0.0.1` → localhost
  only. `0.0.0.0` → any origin (user explicitly opts in). Specific
  IP → that IP + localhost.

### S10. SSE connection exhaustion — OPEN

- **File**: `src/play_launch/src/web/broadcaster.rs`
- **Risk**: No limit on concurrent SSE subscribers.
- **Fix**: Cap at 50 subscribers. Return 503 when exceeded.

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

### S13. Interception hooks return error on missing runtime — OPEN

- **File**: `src/play_launch_interception/src/lib.rs`
- **Risk**: When `librcl.so` unresolvable, hooks return `RCL_RET_ERROR`
  instead of being inert. Silently breaks ROS applications.
- **Fix**: Return 0 (success/no-op) when runtime is None.

### S14. Global package cache no eviction — OPEN

- **File**: `src/play_launch_parser/.../substitution/types.rs`
- **Risk**: Stale cache entries in long-running processes.
- **Fix**: Add TTL or `clear_caches()` API.

### S15. `#![allow(unsafe_op_in_unsafe_fn)]` crate-wide — OPEN

- **File**: `src/play_launch_parser/.../lib.rs:6`
- **Risk**: Suppresses useful lint for entire crate, not just PyO3.
- **Fix**: Scope to PyO3 modules only or upgrade PyO3.

### S16. Process tree scan per-recursion — OPEN

- **File**: `src/play_launch/src/process/tree.rs`
- **Risk**: O(depth) full process table scans in `find_all_descendants`.
- **Fix**: Single scan, build parent→children map in memory.

## Positive Findings

| Area | Finding |
|------|---------|
| Static assets | `rust_embed` prevents path traversal |
| SPSC ordering | Correct Acquire/Release, cache-line padding |
| Channels | Bounded with appropriate backpressure |
| Orphan prevention | `PR_SET_PDEATHSIG(SIGKILL)` on all children |
| Actor deadlocks | Unidirectional channels, no cycles |
| XML parsing | `roxmltree` ignores DTD/entities — no XXE |
| Variable depth | `MAX_RESOLUTION_DEPTH = 20` prevents infinite recursion |
| Log streaming | Paths from member registry, not URL parameters |
