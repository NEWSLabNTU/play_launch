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

### B2. `<group ns="...">` is non-standard — REVISED

- **Severity**: Medium (was Low — now reclassified as non-standard)
- Our Rust parser supports `<group ns="...">` as a namespace shorthand.
  **ROS 2 rejects this**: `ValueError: Unexpected attribute(s) in 'group': {'ns'}`.
  The only valid attributes on `<group>` are `if`, `unless`, and `scoped`.
- **Impact**: Launch files using `<group ns="...">` work in play_launch
  but fail in `ros2 launch`. This creates a portability trap.
- **Fix options**:
  1. Remove support (reject with clear error message)
  2. Keep as documented play_launch extension (warn on use)
- **Recommendation**: Option 2 — warn and document, since removing it
  could break existing play_launch users who rely on it.

### B3. `scoped="false"` attribute ignored — OPEN (NEW)

- **File**: `src/play_launch_parser/.../traverser/entity.rs`
- **Severity**: Medium
- The parser always calls `save_scope()`/`restore_scope()` for groups
  regardless of the `scoped` attribute. When `scoped="false"`, namespace
  changes should leak to subsequent siblings.
- **Reproduction**:
  ```xml
  <group scoped="false">
    <push-ros-namespace namespace="test"/>
    <node pkg="demo" exec="a" name="inside"/>
  </group>
  <node pkg="demo" exec="b" name="outside"/>
  <!-- Python: outside ns=/test (correct: namespace leaks) -->
  <!-- Rust:   outside ns=None (wrong: namespace reverted) -->
  ```
- **Impact**: Nodes after an unscoped group get the wrong namespace.
  Autoware does not use `scoped="false"`, so no current impact.
- **Fix**: Check `scoped` attribute in `GroupAction`. Only call
  `save_scope()`/`restore_scope()` when `scoped="true"` (default).

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
- **Implication for group scopes**: A group scope created in Phase 30b
  captures the namespace at the GROUP level (after `<group ns="...">`
  resolution). For groups using `<push-ros-namespace>`, the namespace
  is set sequentially INSIDE the group — the group scope doesn't
  capture it because `<push-ros-namespace>` doesn't trigger
  `group.namespace.is_some()`. This is correct: group scopes only
  exist for the `ns` attribute form (which is non-standard anyway).

## Executor / Runtime Bugs

### B5. Stop/Restart don't wait for child after kill — OPEN

- **File**: `src/play_launch/src/member_actor/regular_node_actor.rs`
- **Severity**: Low
- `child.kill()` without `child.wait()` in Stop and Restart handlers
  creates a brief zombie until the Child handle is dropped.
- **Fix**: Add `child.wait().await.ok()` after kill.

### B6. Zombie anchor process on panic — OPEN

- **File**: `src/play_launch/src/process/pgid.rs`
- **Severity**: Low
- Anchor process zombie leaked if task panics before `wait()`.
- **Fix**: Wrap wait in a Drop guard.

### B7. `read_last_n_lines` reads entire file — OPEN

- **File**: `src/play_launch/src/web/sse.rs`
- **Severity**: Low
- Reads whole file into memory then takes last N lines. OOM risk on
  large log files (e.g., noisy ROS node producing GB of stderr).
- **Fix**: Reverse-seek approach.

## Parser Code Quality

### B8. `SystemTime::now().unwrap()` in anon substitution — OPEN

- **File**: `src/play_launch_parser/.../substitution/types.rs`
- **Severity**: Low
- Panics if system clock is before UNIX epoch. Extremely unlikely.
- **Fix**: Use `.unwrap_or_default()`.

### B9. No `$(command)` timeout — OPEN

- **File**: `src/play_launch_parser/.../substitution/types.rs`
- **Severity**: Low
- `Command::new("bash").output()` blocks indefinitely if the command
  hangs. Timeouts can be worked around but add defense in depth.

## Python Parser Code Quality

### B10. Broad exception swallowing in include args — OPEN

- **File**: `python/play_launch/dump/visitor/include_launch_description.py`
- **Severity**: Low
- `except Exception: pass` silently swallows arg resolution errors.
- **Fix**: Log at debug level.

### B11. Private member access in visitor — OPEN

- **File**: `python/play_launch/dump/visitor/node.py`
- **Severity**: Low
- Accesses `_Node__ros_arguments`, `_ExecuteLocal__respawn` etc.
  May break between ROS 2 distributions.
- **Fix**: Add try/except guards. Document version dependency.

## Web UI Bugs

### B12. OperationGuard Drop spawns detached task — OPEN

- **File**: `src/play_launch/src/web/handlers.rs`
- **Severity**: Low
- Detached tokio task in Drop may not execute during shutdown.
- **Status**: Acceptable — stale entries during shutdown are harmless.

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

- **Phase 30b**: Groups with `ns="..."` attribute create anonymous scope
  entries (`origin: null`). This is a non-standard extension (B2).
- **`<push-ros-namespace>`**: Sequential action, not tied to groups.
  Does not create a scope entry. Namespace changes are reflected in
  the nodes that follow it.
- **`scoped` attribute**: Currently ignored (B3). Our parser always
  scopes. Fix requires checking the attribute before calling
  `save_scope()`/`restore_scope()`.

### Implications for the scope table

Groups using `<push-ros-namespace>` (the standard pattern) do NOT
create group scope entries. The namespace is accumulated sequentially
and captured at include boundaries. This means:

- For Autoware (all `<push-ros-namespace>`): 0 group scopes, as observed.
- Group scopes only appear for the non-standard `<group ns="...">` form.
- The `scoped="false"` bug (B3) could cause incorrect namespace
  propagation but does not affect scope entries (scopes are informational).

### Recommendations

1. **B2**: Warn when `<group ns="...">` is used. Document as extension.
2. **B3**: Fix `scoped` attribute handling — most impactful bug.
3. **B4**: No action needed — sequential `<push-ros-namespace>` is
   correct and matches ROS 2 behavior.
