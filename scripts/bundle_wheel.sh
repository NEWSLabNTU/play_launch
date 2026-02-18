#!/usr/bin/env bash
# Bundle colcon install/ artifacts into python/play_launch/{bin,lib,share}
# for wheel packaging. Single source of truth — called by justfile and CI.
set -euo pipefail

# ---------------------------------------------------------------------------
# Artifact manifest: "source_in_install:dest_in_python_play_launch"
# Glob patterns are supported in source paths.
# ---------------------------------------------------------------------------
ARTIFACTS=(
  "play_launch/lib/play_launch/play_launch:bin/"
  "play_launch/lib/play_launch/play_launch_io_helper:bin/"
  "play_launch_msgs/lib/libplay_launch_msgs*.so:lib/"
  "play_launch_container/lib/play_launch_container/component_container:lib/play_launch_container/"
  "play_launch_container/lib/play_launch_container/component_node:lib/play_launch_container/"
  "play_launch_container/lib/libobservable_component_manager.so:lib/"
)

# Directories/files to create (ament index markers, etc.)
MARKERS=(
  "share/ament_index/resource_index/packages/play_launch_container"
)

# ---------------------------------------------------------------------------
# Resolve paths
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
INSTALL_DIR="$REPO_ROOT/install"
DEST_ROOT="$REPO_ROOT/python/play_launch"

# ---------------------------------------------------------------------------
# --clean: remove all bundled artifacts and exit
# ---------------------------------------------------------------------------
if [[ "${1:-}" == "--clean" ]]; then
  rm -rf "$DEST_ROOT/bin" "$DEST_ROOT/lib" "$DEST_ROOT/share"
  echo "Cleaned bundled artifacts from python/play_launch/{bin,lib,share}"
  exit 0
fi

# ---------------------------------------------------------------------------
# Verify install/ exists
# ---------------------------------------------------------------------------
if [[ ! -d "$INSTALL_DIR" ]]; then
  echo "Error: $INSTALL_DIR not found. Run 'colcon build --base-paths src' first." >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# Copy artifacts
# ---------------------------------------------------------------------------
copied=0
for entry in "${ARTIFACTS[@]}"; do
  src_pattern="${entry%%:*}"
  dest_rel="${entry##*:}"
  dest_dir="$DEST_ROOT/$dest_rel"

  # Expand globs
  # shellcheck disable=SC2206
  src_files=( $INSTALL_DIR/$src_pattern )

  for src in "${src_files[@]}"; do
    if [[ ! -e "$src" ]]; then
      echo "Error: artifact not found: $src" >&2
      exit 1
    fi
    mkdir -p "$dest_dir"
    cp "$src" "$dest_dir"
    basename_file="$(basename "$src")"
    dest_file="$dest_dir/$basename_file"
    # Make binaries and .so files executable
    if [[ "$dest_rel" == bin/* ]] || [[ "$basename_file" == *.so ]] || [[ ! "$basename_file" == *.* ]]; then
      chmod +x "$dest_file"
    fi
    echo "  $src_pattern -> $dest_rel$basename_file"
    copied=$((copied + 1))
  done
done

# ---------------------------------------------------------------------------
# Create markers
# ---------------------------------------------------------------------------
for marker in "${MARKERS[@]}"; do
  marker_path="$DEST_ROOT/$marker"
  mkdir -p "$(dirname "$marker_path")"
  touch "$marker_path"
done

echo "Bundled $copied artifacts to python/play_launch/{bin,lib,share}"
