colcon_flags := "--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release"

# Show available recipes
default:
    @just --list

# Install all dependencies
install-deps:
    #!/usr/bin/env bash
    set -e

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

# Build all packages with colcon
build:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/humble/setup.bash
    colcon build {{colcon_flags}} --base-paths src

# Build Python wheel with setuptools (copies binaries from colcon build)
build-wheel:
    #!/usr/bin/env bash
    set -e

    # Check if binaries exist from colcon build
    if [ ! -f install/play_launch/lib/play_launch/play_launch ]; then
        echo "Error: play_launch binary not found. Run 'just build' first."
        exit 1
    fi

    # Create bin directory and copy binaries
    echo "Copying binaries to python/play_launch/bin/..."
    mkdir -p python/play_launch/bin
    cp install/play_launch/lib/play_launch/play_launch python/play_launch/bin/
    cp install/play_launch/lib/play_launch/play_launch_io_helper python/play_launch/bin/
    chmod +x python/play_launch/bin/*

    # Build wheel with setuptools
    echo "Building wheel..."
    pip install --quiet build
    python -m build --wheel

    # Show output
    echo ""
    echo "Wheel built successfully:"
    ls -lh dist/*.whl

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
    rm -rf build install log dist target python/play_launch/bin/play_launch python/play_launch/bin/play_launch_io_helper

# Build source distribution
build-sdist:
    #!/usr/bin/env bash
    set -e
    pip install --quiet build
    python -m build --sdist

# Install wheel locally for testing
install-wheel:
    #!/usr/bin/env bash
    set -e
    pip install dist/play_launch-*.whl --force-reinstall

# Test pip-installed wheel (in a fresh venv)
test-wheel:
    #!/usr/bin/env bash
    set -e

    if ! ls dist/play_launch-*.whl &>/dev/null; then
        echo "Error: No wheel found in dist/. Run 'just build-wheel' first."
        exit 1
    fi

    # Create temp venv
    VENV_DIR=$(mktemp -d)
    python3 -m venv "$VENV_DIR"
    source "$VENV_DIR/bin/activate"

    # Source ROS2 (required runtime dependency)
    source /opt/ros/humble/setup.bash

    # Install wheel
    pip install dist/play_launch-*.whl pytest

    # Run tests
    echo "Testing pip-installed package..."
    play_launch --help
    python -c "from play_launch.dump import LaunchInspector; print('dump module: OK')"
    python -c "from play_launch.analyzer import main; print('analyzer module: OK')"

    # Run integration tests
    pytest tests/test_pip_install.py -v

    # Cleanup
    deactivate
    rm -rf "$VENV_DIR"
    echo "All tests passed!"

# Publish wheel to PyPI (requires PYPI_TOKEN env var)
publish-pypi:
    #!/usr/bin/env bash
    set -e

    if [ -z "$PYPI_TOKEN" ]; then
        echo "Error: PYPI_TOKEN environment variable not set."
        echo "Get a token from https://pypi.org/manage/account/token/"
        exit 1
    fi

    if ! ls dist/*.whl &>/dev/null; then
        echo "Error: No wheel found in dist/. Run 'just build-wheel' first."
        exit 1
    fi

    pip install twine
    twine upload dist/*.whl -u __token__ -p "$PYPI_TOKEN"

# Publish wheel to TestPyPI (for testing)
publish-testpypi:
    #!/usr/bin/env bash
    set -e

    if [ -z "$TESTPYPI_TOKEN" ]; then
        echo "Error: TESTPYPI_TOKEN environment variable not set."
        echo "Get a token from https://test.pypi.org/manage/account/token/"
        exit 1
    fi

    if ! ls dist/*.whl &>/dev/null; then
        echo "Error: No wheel found in dist/. Run 'just build-wheel' first."
        exit 1
    fi

    pip install twine
    twine upload --repository testpypi dist/*.whl -u __token__ -p "$TESTPYPI_TOKEN"

# Build Debian package using standard Debian tools
build-deb:
    cd debian && just build

# Run all tests
test:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    # Only test Python packages (Rust testing not compatible with colcon-cargo-ros2)
    # Skip integration tests that require dump_launch command in PATH
    colcon test --packages-select dump_launch --pytest-args -m "not integration"
    colcon test-result --all --verbose

# Run linters (clippy + ruff)
lint:
    #!/usr/bin/env bash
    set -e
    source install/setup.bash

    (
    cd src/play_launch &&
    cargo clippy --config ../../build/ros2_cargo_config.toml --all-targets --all-features -- -D warnings
    )

    (
    cd src/dump_launch &&
    python3 -m ruff check .
    )

# Format code (Rust + Python)
format:
    #!/usr/bin/env bash
    (cd src/play_launch && cargo +nightly fmt)
    (cd src/dump_launch && ruff format .)
