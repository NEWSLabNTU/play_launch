# Phase 27: Runtime Dependency Check

**Status**: Complete
**Completed**: 2026-03-03
**Priority**: Medium (Distribution / UX)
**Dependencies**: Phase 21 (Build Optimization — PyPI distribution)

---

## Overview

play_launch is distributed via PyPI (`pip install play_launch`), but it depends on
ROS 2 system packages installed via `apt`. PyPI cannot declare apt dependencies, so
users who install via pip may hit confusing linker errors or missing-package failures
at runtime.

This phase adds a startup dependency check that detects missing system packages and
prints clear, actionable install instructions.

---

## Problem

The play_launch binary and its C++ companions (`component_container`, `component_node`)
link against shared libraries from several ROS 2 packages:

**Runtime (NEEDED in ELF):**

| Library | apt package | Transitive from ros-base? |
|---------|-------------|---------------------------|
| `librcl.so` | `ros-humble-rcl` | yes |
| `librclcpp.so` | `ros-humble-rclcpp` | yes |
| `librcl_interfaces__rosidl_*.so` | `ros-humble-rcl-interfaces` | yes |
| `libcomposition_interfaces__rosidl_*.so` | `ros-humble-composition-interfaces` | yes |
| `librosgraph_msgs__rosidl_*.so` | `ros-humble-rosgraph-msgs` | yes |
| `libdiagnostic_msgs__rosidl_*.so` | `ros-humble-diagnostic-msgs` | **no** |
| `libclass_loader.so` | `ros-humble-class-loader` | yes |
| `libament_index_cpp.so` | `ros-humble-ament-index-cpp` | yes |
| `libplay_launch_msgs__rosidl_*.so` | (bundled in wheel) | n/a |

**Build-only (rclrs vendored #[link] — stripped by --as-needed):**

| Library | apt package | Why |
|---------|-------------|-----|
| `libtest_msgs__rosidl_*.so` | `ros-humble-test-msgs` | vendored in rclrs |
| `libexample_interfaces__rosidl_*.so` | `ros-humble-example-interfaces` | vendored in rclrs |
| `libaction_msgs__rosidl_*.so` | `ros-humble-action-msgs` | vendored in rclrs (transitive) |
| `libunique_identifier_msgs__rosidl_*.so` | `ros-humble-unique-identifier-msgs` | vendored in rclrs (transitive) |

Most runtime deps are transitive from `ros-humble-ros-base`. The only explicitly
required package that users might not have is `ros-humble-diagnostic-msgs`.

The build-only deps are needed by anyone building from source because rclrs vendors
message types with `#[link]` attributes that require the `.so` at link time. The
linker's `--as-needed` flag strips them from the final binary since play_launch never
calls those symbols.

---

## Design

### Runtime check (replay/launch path)

Add `check_system_deps()` to `ros/ament_index.rs`. Called early in `handle_replay()`
before any ROS node creation. Checks that required packages exist in the ament index.

On missing packages, prints a single consolidated error with exact `apt install`
commands, then exits non-zero. Does NOT print one error per package — one block with
the full install command.

```
error: Missing ROS 2 system dependencies:
  - diagnostic_msgs

Install with:
  sudo apt install ros-humble-diagnostic-msgs

Or install all play_launch dependencies:
  rosdep install --from-paths <play_launch_src> --ignore-src -y
```

### rosdep (package.xml)

Update `package.xml` files to declare all dependencies properly:

- `play_launch/package.xml`: add `rosgraph_msgs` (runtime dep from rclrs)
- `play_launch/package.xml`: add `test_msgs`, `example_interfaces`,
  `action_msgs`, `unique_identifier_msgs` as `<build_depend>` (link-time only)
- `play_launch_container/package.xml`: add `class_loader`, `ament_index_cpp`
  (already linked, just not declared)

---

## Implementation

### 27.1: Runtime dependency check

- Add `check_system_deps()` to `ros/ament_index.rs`
- Call from `handle_replay()` before ROS node creation
- Detect ROS distro from `ROS_DISTRO` env var for apt package names
- Check packages via ament index (reuse `get_package_prefix()`)

### 27.2: Update package.xml

- Add missing `<depend>` and `<build_depend>` entries
- Ensures `rosdep install` works for source builds

---

## Verification

1. `pip install play_launch` on a system missing `ros-humble-diagnostic-msgs`
2. Run `play_launch launch ...` → should print clear error with install command
3. Install the package → should proceed normally
4. `rosdep install --from-paths src --ignore-src -y` → should install all deps
