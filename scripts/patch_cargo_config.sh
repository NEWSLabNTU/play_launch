#!/usr/bin/env bash
# Patch colcon-generated artifacts for compatibility with vendored rclrs 0.7 and
# rosidl_runtime_rs 0.6.
#
# The colcon-cargo-ros2 code generator emits rosidl_runtime_rs = "0.5" in message
# crate Cargo.toml files. Our vendored rosidl_runtime_rs is 0.6. This script:
#   1. Bumps the message crates' rosidl_runtime_rs dep from "0.5" to "0.6"
#   2. Adds [patch.crates-io] entries to ros2_cargo_config.toml for both
#      rosidl_runtime_rs and rclrs
#
# Runs AFTER colcon builds C++ packages, BEFORE the Rust build.

set -e

# 1. Patch message crate Cargo.toml files
for f in $(find build -name 'Cargo.toml' -path '*/rosidl_cargo/*' 2>/dev/null); do
    if grep -q 'rosidl_runtime_rs = "0.5"' "$f"; then
        sed -i 's/rosidl_runtime_rs = "0.5"/rosidl_runtime_rs = "0.6"/' "$f"
    fi
done

# 2. Patch ros2_cargo_config.toml
CONFIG="build/ros2_cargo_config.toml"

if [ ! -f "$CONFIG" ]; then
    echo "Warning: $CONFIG not found (C++ packages not built yet?)" >&2
    exit 0
fi

# Idempotent — only patch if not already present
if grep -q 'rosidl_runtime_rs' "$CONFIG"; then
    exit 0
fi

# Insert into [patch.crates-io] section (before [build])
sed -i '/^\[build\]/i rosidl_runtime_rs = { path = "src/vendor/rosidl_runtime_rs/rosidl_runtime_rs" }\nrclrs = { path = "src/vendor/ros2_rust/rclrs" }' "$CONFIG"
