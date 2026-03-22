# Bugs and Code Quality Issues

Last updated: 2026-03-22

## Parser Bugs

### B1. `push_ros_namespace` (underscore form) not handled — RESOLVED

- **File**: `src/play_launch_parser/.../traverser/entity.rs`
- **Severity**: Low
- The XML parser handled `push-ros-namespace` (hyphenated) but treated
  `push_ros_namespace` (underscored) as unknown. Both are valid ROS 2
  launch XML.
- **Fix**: Added `"push_ros_namespace"` to the match arm alongside
  `"push-ros-namespace"`. Same for `pop_ros_namespace`.

### B2. `<group ns="...">` is non-standard — RESOLVED

- **Severity**: Medium (was Low — reclassified as non-standard)
- Our Rust parser supported `<group ns="...">` as a namespace shorthand.
  **ROS 2 rejects this**: `ValueError: Unexpected attribute(s) in 'group': {'ns'}`.
  The only valid attributes on `<group>` are `if`, `unless`, and `scoped`.
- **Fix**: Dropped support. `GroupAction::namespace` is always `None`.
  Groups now create anonymous scope entries (`origin: null`) when
  `scoped="true"`. Namespace is set by `<push-ros-namespace>` inside
  the group (standard ROS 2 pattern).

### B3. `scoped="false"` attribute ignored — RESOLVED

- **File**: `src/play_launch_parser/.../traverser/entity.rs`
- **Severity**: Medium
- The parser always called `save_scope()`/`restore_scope()` for groups
  regardless of the `scoped` attribute.
- **Fix**: Conditional save/restore based on `group.scoped`. When
  `scoped="false"`, namespace/env changes leak to subsequent siblings.
  Group scope entry is only created for `scoped="true"`.
- **Tests**: `test_scoped_false_namespace_leaks`,
  `test_scoped_true_namespace_reverts` in `scope_tests.rs`.

### B4. `<push-ros-namespace>` is sequential, not group-level — DESIGN NOTE

- **Severity**: Info (not a bug — matches ROS 2 behavior)
- `<push-ros-namespace>` is a sequential action processed in document
  order. A node BEFORE the push gets no namespace; a node AFTER gets
  the namespace. Multiple pushes accumulate.
- **Verified**: Both Rust and Python parsers agree on all cases:
  ```
  node_before_push   ns=None       (correct)
  node_after_push    ns=/late_ns   (correct)
  double_push        ns=/l1/l2     (correct: accumulates)
  ```

## Executor / Runtime Bugs

### B5. Stop/Restart send SIGKILL without graceful shutdown — RESOLVED

- **File**: `src/play_launch/src/member_actor/regular_node_actor.rs`
- **Severity**: Medium (was Low)
- Stop and Restart control events called `child.kill()` (SIGKILL)
  directly — no graceful SIGTERM first. ROS nodes need time to
  flush logs, save state, and deregister from DDS.
- **Fix**: Added `graceful_kill()` function: SIGTERM → wait up to 5s
  → SIGKILL + wait if still alive. Used by both Stop and Restart
  handlers. Non-Unix fallback uses `child.kill()` directly.

### B6. Zombie anchor process on panic — ACCEPTED

- **File**: `src/play_launch/src/process/pgid.rs`
- **Severity**: Low
- Anchor process zombie leaked if task panics before `wait()`.
- **Status**: Accepted. The anchor runs `true` which exits in
  microseconds — it's already a zombie before any panic could occur.
  A single zombie consumes no resources (just a PID entry) and is
  reaped by init when play_launch exits. The panic scenario is
  unlikely (only a oneshot send + watch receive between spawn and
  wait).

### B7. `read_last_n_lines` reads entire file — RESOLVED

- **File**: `src/play_launch/src/web/sse.rs`
- **Severity**: Low
- Read whole file into memory then took last N lines. OOM risk on
  large log files (e.g., noisy ROS node producing GB of stderr).
- **Fix**: `read_last_n_lines` now seeks to max(0, EOF - 2 MB) and
  reads only the tail. Lines exceeding 8 KB are truncated with a
  byte-count marker: `"... (truncated, N bytes total)"`. Same
  truncation applied to live tailing in both log and metrics SSE
  streams. Oversized `line_buf` is shrunk after each truncated read.

## Parser Code Quality

### B8. `SystemTime::now().unwrap()` in anon substitution — ACCEPTED

- **File**: `src/play_launch_parser/.../substitution/types.rs`
- **Severity**: Low
- Panics if system clock is before UNIX epoch.
- **Status**: Accepted. A pre-epoch clock would break the entire ROS
  system (TF, logging, bag recording). The random component ensures
  uniqueness regardless.

### B9. No `$(command)` timeout — RESOLVED

- **File**: `src/play_launch_parser/.../substitution/types.rs`
- **Severity**: Low
- `Command::new("bash").output()` blocked indefinitely if the command
  hung (e.g., xacro on missing files).
- **Fix**: Wrapped with GNU `timeout` command: SIGTERM after 30s,
  SIGKILL after 5s grace period. Exit code 124 detected as timeout
  with a clear error message. `timeout` handles process group cleanup.
  No pipe deadlock since `output()` still drains pipes correctly.

## Python Parser Code Quality

### B10. Broad exception swallowing in include args — RESOLVED

- **File**: `python/play_launch/dump/visitor/include_launch_description.py`
- **Severity**: Low
- `except Exception: pass` silently swallowed arg resolution errors
  and launch file path resolution errors.
- **Fix**: Both `except Exception` handlers now log at `debug` level
  with the exception message. Other broad exception handlers in
  `node.py` and `composable_node_container.py` already logged at
  `warning`/`error` level — left as-is.

### B11. Private member access in visitor — ACCEPTED

- **File**: `python/play_launch/dump/visitor/node.py`
- **Severity**: Low
- Accesses `_Node__ros_arguments`, `_ExecuteLocal__respawn` etc.
  May break between ROS 2 distributions.
- **Status**: Accepted. The Python parser intentionally reuses ROS
  launch internals for compatibility. Private member access is the
  only way to extract certain node properties. Breakage across
  distributions is handled by testing against target ROS versions.

## Web UI Bugs

### B12. OperationGuard Drop spawns detached task — RESOLVED

- **File**: `src/play_launch/src/web/handlers.rs`
- **Severity**: Low
- `Drop` spawned a detached `tokio::spawn` to remove the entry from
  `operations_in_progress` (a `tokio::Mutex<HashSet>`), which could
  fail to execute during shutdown.
- **Fix**: Replaced `tokio::Mutex` with `std::sync::Mutex`. The
  critical sections are tiny (insert/remove on a HashSet, no `.await`
  while held). `Drop` now synchronously locks and removes. No
  detached task needed. `try_acquire` is no longer async.

## Resolved Bugs

### B13. Container scope-map key mismatch — RESOLVED

- Used `exec_name` instead of `name` for container member key.
- Fixed: `let member_name: &str = &c.name;` in `replay.rs`.

### B14. Python `_get_launch_file()` unprotected — RESOLVED

- Could raise exception, leaving scope stack shifted.
- Fixed: try/except + pop_scope in finally block.

### B15. No cycle protection in JS scope chain walk — RESOLVED

- LaunchPanel scope chain walk had no visited set.
- Fixed: Added `visited` Set in `LaunchPanel.js`.

### B16. Hardcoded byte offset in extract_package_from_path — RESOLVED

- Magic number `7` for `/share/` length.
- Fixed: `SHARE_PREFIX.len()`.

---

## Group Scope Design Notes

The following summarizes how `<group>` interacts with the scope system
and known edge cases.

### ROS 2 `<group>` semantics

| Attribute | Default | Meaning |
|-----------|---------|---------|
| `if` | (none) | Conditional group |
| `unless` | (none) | Conditional group (inverse) |
| `scoped` | `"true"` | Whether namespace/env changes revert after group |

The `<group>` tag does **NOT** have a `namespace` or `ns` attribute in
standard ROS 2. Namespace is set by `<push-ros-namespace>` inside the
group.

### How play_launch models groups

- **Phase 30b**: Groups with `scoped="true"` (default) create anonymous
  scope entries (`origin: null`). The web UI derives group labels from
  child scope namespaces (e.g., `group /sensing`).
- **`<group ns="...">`**: Non-standard, support dropped (B2). ROS 2
  rejects this attribute.
- **`<push-ros-namespace>`**: Sequential action, not tied to groups.
  Does not create a scope entry. Namespace changes are reflected in
  the nodes that follow it.
- **`scoped="false"`**: Namespace/env changes leak to siblings. No
  group scope entry created (B3, fixed).

### Implications for the scope table

Groups using `<push-ros-namespace>` (the standard pattern) create
anonymous group scope entries when `scoped="true"`. The namespace is
accumulated sequentially and captured at include boundaries. For
Autoware: 86 group scopes + 83 file scopes = 169 total scopes.
