colcon_flags := "--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release"

# Show available recipes
default:
    @just --list

# Install all dependencies
install-deps:
    #!/usr/bin/env bash
    set -e

    # Check for dirty submodules
    if git submodule status | grep -E '^\+'; then
        echo "Warning: Some submodules have uncommitted changes."
        git submodule status | grep -E '^\+'
        read -p "Discard untracked changes and update submodules? [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            git submodule foreach --recursive git clean -fd
            git submodule update --init --recursive
        else
            echo "Skipping submodule update. Please resolve manually."
            exit 1
        fi
    else
        git submodule update --init --recursive
    fi

    # Check for conflicting colcon packages
    if pip show colcon-ros-cargo &>/dev/null || pip show colcon-cargo &>/dev/null; then
        echo "Warning: Conflicting packages detected:"
        pip show colcon-ros-cargo 2>/dev/null | grep -E '^(Name|Version):' || true
        pip show colcon-cargo 2>/dev/null | grep -E '^(Name|Version):' || true
        echo
        read -p "Uninstall colcon-ros-cargo and colcon-cargo? (Required for colcon-cargo-ros2) [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            pip uninstall -y colcon-ros-cargo colcon-cargo 2>/dev/null || true
        else
            echo "Warning: colcon-cargo-ros2 may conflict with existing packages."
        fi
    fi

    # Install colcon-cargo-ros2
    pip install colcon-cargo-ros2

    source /opt/ros/humble/setup.bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

# Build all packages
build:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/humble/setup.bash
    colcon build {{colcon_flags}} --base-paths src

# Run play_launch with optional arguments (e.g., just run launch demo_nodes_cpp talker_listener.launch.py)
run *ARGS:
    #!/usr/bin/env bash
    set -e
    source install/setup.bash
    ros2 run play_launch play_launch {{ARGS}}

# Apply CAP_SYS_PTRACE to I/O helper (requires sudo)
setcap-io-helper:
    #!/bin/bash
    if [ ! -f install/play_launch/lib/play_launch/play_launch_io_helper ]; then
        echo "Error: play_launch_io_helper not found. Run 'just build' first."
        exit 1
    fi
    sudo setcap cap_sys_ptrace+ep install/play_launch/lib/play_launch/play_launch_io_helper
    getcap install/play_launch/lib/play_launch/play_launch_io_helper
    echo "✓ I/O helper ready (reapply after rebuild)"

# Verify I/O helper capability status
verify-io-helper:
    #!/bin/bash
    if [ ! -f install/play_launch/lib/play_launch/play_launch_io_helper ]; then
        echo "✗ I/O helper not found. Run 'just build' first."
        exit 1
    fi

    cap=$(getcap install/play_launch/lib/play_launch/play_launch_io_helper 2>/dev/null)
    if [ -z "$cap" ]; then
        echo "✗ I/O helper has no capabilities set"
        echo "  Run 'just setcap-io-helper' to enable I/O monitoring for privileged processes"
        exit 1
    elif echo "$cap" | grep -q "cap_sys_ptrace+ep"; then
        echo "✓ I/O helper ready: $cap"
        exit 0
    else
        echo "⚠ I/O helper has unexpected capabilities: $cap"
        echo "  Expected: cap_sys_ptrace+ep"
        echo "  Run 'just setcap-io-helper' to fix"
        exit 1
    fi

# Clean all build artifacts
clean:
    rm -rf build install log

# Build Debian package using standard Debian tools
build-deb:
    dpkg-buildpackage -us -uc -b -d

# Run all tests
test:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    colcon test --packages-select dump_launch play_launch
    colcon test-result --all --verbose

# Run linters (clippy + ruff)
lint:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    (cd src/play_launch && cargo clippy --all-targets --all-features -- -D warnings)
    (cd src/dump_launch && python3 -m ruff check .)

# Format code (Rust + Python)
format:
    #!/usr/bin/env bash
    (cd src/play_launch && cargo +nightly fmt)
    (cd src/dump_launch && ruff format .)
