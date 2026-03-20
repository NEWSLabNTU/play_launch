# Bugs and Code Quality Issues

Last updated: 2026-03-20

## Parser Bugs

### B1. `push_ros_namespace` (underscore form) not handled — OPEN

- **File**: `src/play_launch_parser/.../traverser/entity.rs`
- **Severity**: Low
- The XML parser handles `push-ros-namespace` (hyphenated) but treats
  `push_ros_namespace` (underscored) as unknown. Both are valid ROS 2
  launch XML. Autoware uses the hyphenated form exclusively.
- **Fix**: Add `"push_ros_namespace"` to the match arm.

### B2. Scope `ns` incorrect for `<group namespace="...">` includes — OPEN

- **File**: `src/play_launch_parser/.../traverser/include.rs`
- **Severity**: Low (informational metadata only)
- When `<group namespace="sensing">` wraps `<include>`, the scope entry
  records `ns=/` instead of `ns=/sensing`. Node namespace is correct.
- **Impact**: Only affects `play_launch context --tree` display and
  scope metadata in `record.json`. Does not affect execution.
- **Root cause**: Scope `ns` captured from `include_context` (child)
  rather than `self.context` (parent with group namespace applied).
- **Note**: `<group>` is not a scope — it's a lexical block within a
  launch file. Scopes correspond to launch file invocations only.
- **Test**: `test_scope_group_namespace_attr_with_include` in
  `scope_tests.rs`.

### B3. `<group namespace="...">` attribute form not supported in Python — NOT A BUG

- **Severity**: N/A (ROS 2 framework behavior)
- The ROS 2 `launch` Python package does not support the `namespace`
  attribute on `<group>`. The standard form is `<push-ros-namespace>`
  inside the group. This is upstream ROS 2 behavior, not play_launch.

## Executor / Runtime Bugs

### B4. Stop/Restart don't wait for child after kill — OPEN

- **File**: `src/play_launch/src/member_actor/regular_node_actor.rs`
- **Severity**: Low
- `child.kill()` without `child.wait()` in Stop and Restart handlers
  creates a brief zombie until the Child handle is dropped.
- **Fix**: Add `child.wait().await.ok()` after kill.

### B5. Zombie anchor process on panic — OPEN

- **File**: `src/play_launch/src/process/pgid.rs`
- **Severity**: Low
- Anchor process zombie leaked if task panics before `wait()`.
- **Fix**: Wrap wait in a Drop guard.

### B6. `read_last_n_lines` reads entire file — OPEN

- **File**: `src/play_launch/src/web/sse.rs`
- **Severity**: Low
- Reads whole file into memory then takes last N lines. OOM risk on
  large log files (e.g., noisy ROS node producing GB of stderr).
- **Fix**: Reverse-seek approach.

## Parser Code Quality

### B7. `SystemTime::now().unwrap()` in anon substitution — OPEN

- **File**: `src/play_launch_parser/.../substitution/types.rs`
- **Severity**: Low
- Panics if system clock is before UNIX epoch. Extremely unlikely.
- **Fix**: Use `.unwrap_or_default()`.

### B8. No `$(command)` timeout — OPEN

- **File**: `src/play_launch_parser/.../substitution/types.rs`
- **Severity**: Low
- `Command::new("bash").output()` blocks indefinitely if the command
  hangs. Timeouts can be worked around but add defense in depth.

## Python Parser Code Quality

### B9. Broad exception swallowing in include args — OPEN

- **File**: `python/play_launch/dump/visitor/include_launch_description.py`
- **Severity**: Low
- `except Exception: pass` silently swallows arg resolution errors.
- **Fix**: Log at debug level.

### B10. Private member access in visitor — OPEN

- **File**: `python/play_launch/dump/visitor/node.py`
- **Severity**: Low
- Accesses `_Node__ros_arguments`, `_ExecuteLocal__respawn` etc.
  May break between ROS 2 distributions.
- **Fix**: Add try/except guards. Document version dependency.

## Web UI Bugs

### B11. OperationGuard Drop spawns detached task — OPEN

- **File**: `src/play_launch/src/web/handlers.rs`
- **Severity**: Low
- Detached tokio task in Drop may not execute during shutdown.
- **Status**: Acceptable — stale entries during shutdown are harmless.

## Phase 30 Bugs (all resolved)

### B12. Container scope-map key mismatch — RESOLVED

- Used `exec_name` instead of `name` for container member key.
- Fixed: `let member_name: &str = &c.name;` in `replay.rs`.

### B13. Python `_get_launch_file()` unprotected — RESOLVED

- Could raise exception, leaving scope stack shifted.
- Fixed: try/except + pop_scope in finally block.

### B14. No cycle protection in JS scope chain walk — RESOLVED

- LaunchPanel scope chain walk had no visited set.
- Fixed: Added `visited` Set in `LaunchPanel.js`.

### B15. Hardcoded byte offset in extract_package_from_path — RESOLVED

- Magic number `7` for `/share/` length.
- Fixed: `SHARE_PREFIX.len()`.
