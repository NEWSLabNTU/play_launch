# Phase 21: Build System Optimization

**Status**: Complete
**Priority**: Medium (DX improvement, CI reliability, eliminates duplicated logic)
**Dependencies**: None

## Overview

Clean up the build pipeline by extracting the artifact bundling step into a single script, adding incremental build recipes, fixing the wheel platform tag hack, and caching the CI build environment.

### Motivation

The current build has two pain points:

1. **Duplicated bundle logic**: The 20-line copy block that bridges colcon's `install/` to the wheel layout (`python/play_launch/{bin,lib,share}`) is duplicated between `justfile:46-69` and `.github/workflows/release-wheel.yml:104-127`. Every new artifact (e.g. adding `component_node`) requires updating both.

2. **CI overhead**: `release-wheel.yml` installs ROS2 + Rust + all build deps from scratch inside a bare `ubuntu:22.04` container on every run (~5-10 min of setup before any build work).

3. **Fragile platform tag**: The CI renames wheel files with `sed` to replace `py3-none-any` with `manylinux_2_35_x86_64`. This is brittle and produces incorrect wheel metadata.

4. **No incremental recipes**: `just build` always runs `colcon build` for all packages. Rust-only or C++-only changes don't need a full rebuild.

## Implementation Order

```
21.0 Extract bundle script + artifact manifest      complete
21.1 Incremental build recipes in justfile          complete
21.2 Fix wheel platform tag                         complete
21.3 Pre-built CI Docker image                      complete
```

---

## Phase 21.0: Extract Bundle Script + Artifact Manifest

**Status**: Complete

Extract the copy logic into `scripts/bundle_wheel.sh` with a declarative artifact manifest. Both `justfile` and CI call this single script.

### Work Items

- [x] Create `scripts/bundle_wheel.sh`:
  - Declare artifact mappings as an array at the top (source relative to `install/` -> dest relative to `python/play_launch/`)
  - Loop over manifest: `mkdir -p` dest dir, `cp` artifact, `chmod +x` binaries
  - Create ament index marker for `play_launch_container`
  - Verify all source artifacts exist before copying (fail fast with clear error)
- [x] Update `justfile` `build` recipe to call `scripts/bundle_wheel.sh`
- [x] Update `.github/workflows/release-wheel.yml` to call `scripts/bundle_wheel.sh`
- [x] Update `justfile` `clean` recipe to derive paths from the same manifest or call a `bundle_wheel.sh --clean` flag

### Artifact Manifest

```bash
ARTIFACTS=(
  "play_launch/lib/play_launch/play_launch:bin/play_launch"
  "play_launch/lib/play_launch/play_launch_io_helper:bin/play_launch_io_helper"
  "play_launch_msgs/lib/libplay_launch_msgs*.so:lib/"
  "play_launch_container/lib/play_launch_container/component_container:lib/play_launch_container/"
  "play_launch_container/lib/play_launch_container/component_node:lib/play_launch_container/"
  "play_launch_container/lib/libobservable_component_manager.so:lib/"
)
```

Adding a new artifact becomes a one-line addition to this array.

### Passing Criteria

- [x] `just build` produces identical wheel contents as before
- [x] `release-wheel.yml` produces identical wheel contents as before
- [x] Copy logic exists in exactly one place (`scripts/bundle_wheel.sh`)
- [x] Missing artifact in `install/` fails the script with a clear error message

---

## Phase 21.1: Incremental Build Recipes

**Status**: Complete

Add `--packages-select` recipes so developers can rebuild only what changed.

### Work Items

- [x] Rename current `build` to be the full build (colcon all + bundle + wheel)
- [x] Add `build-cpp` recipe: `colcon build --packages-select play_launch_msgs play_launch_container`
- [x] Add `build-rust` recipe: `colcon build --packages-select play_launch` (assumes C++ `install/` exists)
- [x] Add `build-wheel` recipe: `scripts/bundle_wheel.sh && uv build --wheel` (bundle + wheel only, no colcon)

### Resulting Recipes

```just
# Full build: colcon all + bundle + wheel
build:
    colcon build ... --base-paths src
    scripts/bundle_wheel.sh
    uv build --wheel

# C++ only (msgs + container)
build-cpp:
    colcon build --packages-select play_launch_msgs play_launch_container ...

# Rust only (assumes C++ install/ exists)
build-rust:
    colcon build --packages-select play_launch ...

# Bundle + wheel only (assumes colcon install/ is up to date)
build-wheel:
    scripts/bundle_wheel.sh
    uv build --wheel
```

### Passing Criteria

- [x] `just build` still does a full build (no behavior change for default workflow)
- [x] `just build-rust` skips C++ rebuild
- [x] `just build-cpp` skips Rust rebuild
- [x] `just build-wheel` skips colcon entirely

---

## Phase 21.2: Fix Wheel Platform Tag

**Status**: Complete

Replace the `sed`-based wheel rename with `wheel tags --platform-tag=... --remove`.

### Work Items

- [x] Replace `sed` rename with `wheel tags --platform-tag=${WHEEL_PLATFORM} --remove` in `release-wheel.yml`
- [x] Remove the `sed` rename block and `dist-final/` workaround from `release-wheel.yml`
- [x] Add `'wheel>=0.40'` to CI pip deps (for `wheel tags` CLI)

### Passing Criteria

- [x] Built wheel filename contains the correct platform tag without post-build renaming
- [x] `unzip -p *.whl '*/WHEEL'` shows correct `Tag:` line
- [x] No `sed` or `mv` workarounds for wheel naming remain in CI

---

## Phase 21.3: Pre-built CI Docker Image

**Status**: Complete

Build and publish a Docker image with all build dependencies pre-installed. The release workflow uses it instead of installing from scratch.

### Work Items

- [x] Create `docker/builder.Dockerfile` with:
  - Base: `ubuntu:22.04`
  - ROS2 Humble (ros-humble-ros-core + all required packages)
  - Rust toolchain (stable)
  - Python build tools (pip, build, wheel, colcon-cargo-ros2)
  - `just` command runner
  - `uv` (fast wheel builder)
- [x] Create `.github/workflows/builder-image.yml`:
  - Triggered on changes to `docker/builder.Dockerfile` or manual dispatch
  - Builds for `linux/amd64` and `linux/arm64` (QEMU + buildx)
  - Pushes to `ghcr.io/<owner>/play-launch-builder:humble` + SHA-tagged
  - GHA cache for layer reuse across builds
- [x] Update `release-wheel.yml` to use the pre-built image instead of bare `ubuntu:22.04`
- [x] Document image rebuild process in the Dockerfile

### Passing Criteria

- [x] `release-wheel.yml` build step reduced to colcon build + bundle + wheel (no apt/rustup/pip)
- [x] CI build time reduced (no apt-get install, no rustup on every run)
- [x] Builder image is versioned and reproducible (`:humble` + `:humble-<sha>` tags)
- [x] arm64 wheel builds still work via QEMU + multi-platform image

---

## Summary

| Phase | Change                            | Effort | Impact                           |
|-------|-----------------------------------|--------|----------------------------------|
| 21.0  | Bundle script + artifact manifest | 30 min | Eliminates duplicated copy logic |
| 21.1  | Incremental build recipes         | 15 min | Faster dev iteration             |
| 21.2  | Proper wheel platform tag         | 15 min | Removes fragile sed hack         |
| 21.3  | Pre-built CI Docker image         | 2 hr   | Cuts CI time ~5-10 min           |
