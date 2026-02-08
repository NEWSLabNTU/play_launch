# Detect ROS2 distro based on Ubuntu version
ros_distro := if `lsb_release -rs 2>/dev/null || echo "22.04"` == "24.04" { "jazzy" } else { "humble" }
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

    source /opt/ros/{{ros_distro}}/setup.bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

# Build Rust binaries with colcon and package Python wheel
build:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash

    # Step 1: Build Rust binaries with colcon
    echo "Building Rust binaries with colcon..."
    colcon build {{colcon_flags}} --base-paths src

    # Step 2: Copy binaries to Python package
    echo "Copying binaries to python/play_launch/bin/..."
    mkdir -p python/play_launch/bin
    cp install/play_launch/lib/play_launch/play_launch python/play_launch/bin/
    cp install/play_launch/lib/play_launch/play_launch_io_helper python/play_launch/bin/
    chmod +x python/play_launch/bin/*

    # Step 3: Build wheel with uv
    echo "Building wheel..."
    uv build --wheel

    # Show output
    echo ""
    echo "Build complete:"
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
    uv build --sdist

# Install wheel locally for testing
install-wheel:
    #!/usr/bin/env bash
    set -e
    pip install dist/play_launch-*.whl --force-reinstall

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
        echo "Error: No wheel found in dist/. Run 'just build' first."
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
        echo "Error: No wheel found in dist/. Run 'just build' first."
        exit 1
    fi

    pip install twine
    twine upload --repository testpypi dist/*.whl -u __token__ -p "$TESTPYPI_TOKEN"

# Run standard tests — parser unit tests + fast integration tests (~3s)
test:
    #!/usr/bin/env bash
    set -e
    echo "=== Parser unit tests ==="
    (cd src/play_launch_parser && cargo nextest run -p play_launch_parser --no-fail-fast --failure-output immediate-final)
    echo ""
    echo "=== Integration tests (fast) ==="
    (cd tests && cargo nextest run -E 'not binary(autoware) & not binary(io_stress) & not test(/launch/)' --no-fail-fast --failure-output immediate-final)

# Run all tests — parser unit + all integration including Autoware (~30s)
test-all:
    #!/usr/bin/env bash
    set -e
    echo "=== Parser unit tests ==="
    (cd src/play_launch_parser && cargo nextest run -p play_launch_parser --no-fail-fast --failure-output immediate-final)
    echo ""
    echo "=== Integration tests (all) ==="
    (cd tests && cargo nextest run --no-fail-fast --failure-output immediate-final)

# Run parser unit tests only
test-unit:
    #!/usr/bin/env bash
    set -e
    cd src/play_launch_parser
    cargo nextest run -p play_launch_parser --no-fail-fast --failure-output immediate-final

# Run all integration tests (simple + Autoware)
test-integration:
    #!/usr/bin/env bash
    set -e
    cd tests
    cargo nextest run --no-fail-fast --failure-output immediate-final

# Run simple workspace integration tests
test-simple:
    #!/usr/bin/env bash
    set -e
    cd tests
    cargo nextest run -E 'binary(simple_workspace)' --no-fail-fast --failure-output immediate-final

# Run Autoware smoke test (health check: node exits + LoadNode failures)
smoke-test-autoware:
    #!/usr/bin/env bash
    set -e
    cd tests
    cargo nextest run -E 'test(smoke_test)' --no-capture

# Run Autoware integration tests
test-autoware:
    #!/usr/bin/env bash
    set -e
    cd tests
    cargo nextest run -E 'binary(autoware)' --no-fail-fast --failure-output immediate-final

# Compare Rust vs Python parser outputs
compare-parsers:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash
    ./scripts/compare_parsers.sh

# Benchmark Rust vs Python parser performance
benchmark-parsers ITERATIONS="5":
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash
    ITERATIONS={{ITERATIONS}} ./scripts/benchmark_parsers.sh

# Run checks (clippy + rustfmt check + ruff)
check:
    #!/usr/bin/env bash
    set -e
    source install/setup.bash

    echo "=== play_launch (clippy) ==="
    (
    cd src/play_launch &&
    cargo clippy --config ../../build/ros2_cargo_config.toml --all-targets --all-features -- -D warnings
    )

    echo ""
    echo "=== play_launch_parser (clippy + rustfmt) ==="
    (cd src/play_launch_parser && just check)

    echo ""
    echo "=== Python (ruff) ==="
    python3 -m ruff check python/

# Check Web UI (HTML, CSS, JavaScript)
check-web-ui:
    #!/usr/bin/env bash
    set -e

    # Check if node_modules exists
    if [ ! -d "node_modules" ]; then
        echo "Installing Node.js dependencies..."
        npm install
    fi

    echo "Linting Web UI..."
    npm run lint

# Format code (Rust + Python)
format:
    #!/usr/bin/env bash
    (cd src/play_launch && cargo +nightly fmt)
    (cd src/play_launch_parser && cargo +nightly fmt)
    ruff format python/

# Run format and check
quality:
    #!/usr/bin/env bash
    set -e
    echo "Running format..."
    just format
    echo "Running checks..."
    just check

# Check version consistency across all files
version-check:
    python3 scripts/bump_version.py --check

# Bump version (patch, minor, or major)
bump-version TYPE:
    python3 scripts/bump_version.py {{TYPE}}

# Set explicit version
set-version VERSION:
    python3 scripts/bump_version.py --set {{VERSION}}
