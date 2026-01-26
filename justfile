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

# Test pip-installed wheel (in a fresh venv)
test-wheel:
    #!/usr/bin/env bash
    set -e

    if ! ls dist/play_launch-*.whl &>/dev/null; then
        echo "Error: No wheel found in dist/. Run 'just build' first."
        exit 1
    fi

    # Create temp venv
    VENV_DIR=$(mktemp -d)
    python3 -m venv "$VENV_DIR"
    source "$VENV_DIR/bin/activate"

    # Source ROS2 (required runtime dependency)
    source /opt/ros/{{ros_distro}}/setup.bash

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

# Run all tests
test:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash
    # Only test Python packages (Rust testing not compatible with colcon-cargo-ros2)
    # Skip integration tests that require dump_launch command in PATH
    colcon test --packages-select dump_launch --pytest-args -m "not integration"
    colcon test-result --all --verbose

# Test play_launch_parser library
test-parser:
    #!/usr/bin/env bash
    set -e
    cd src/play_launch_parser
    cargo test --lib

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

# Run linters (clippy + ruff)
lint:
    #!/usr/bin/env bash
    set -e
    source install/setup.bash

    (
    cd src/play_launch &&
    cargo clippy --config ../../build/ros2_cargo_config.toml --all-targets --all-features -- -D warnings
    )

    python3 -m ruff check python/

# Lint Web UI (HTML, CSS, JavaScript)
lint-web-ui:
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
    ruff format python/

# Run format and lint checks
quality:
    #!/usr/bin/env bash
    set -e
    echo "Running format..."
    just format
    echo "Running lint..."
    just lint

# Check version consistency across all files
version-check:
    python3 scripts/bump_version.py --check

# Bump version (patch, minor, or major)
bump-version TYPE:
    python3 scripts/bump_version.py {{TYPE}}

# Set explicit version
set-version VERSION:
    python3 scripts/bump_version.py --set {{VERSION}}

# Test play_launch in test/* workspaces (excluding autoware_planning_simulation)
test-workspaces:
    #!/usr/bin/env bash
    set -e

    # Ensure play_launch is built
    if [ ! -f install/setup.bash ]; then
        echo "Error: install/setup.bash not found. Run 'just build' first."
        exit 1
    fi

    # Source ROS2 and local install
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash

    echo "=================================================="
    echo "Testing play_launch in test/* workspaces"
    echo "Using local install (not pip-installed version)"
    echo "=================================================="
    echo ""

    # Test 1: composition_demo
    echo "=========================================="
    echo "Test 1/3: composition_demo"
    echo "=========================================="
    cd test/composition_demo
    make clean
    make test-all
    cd ../..
    echo ""

    # Test 2: io_stress
    echo "=========================================="
    echo "Test 2/3: io_stress"
    echo "=========================================="
    cd test/io_stress

    # Build io_stress package if needed
    if [ ! -f ../../install/io_stress/lib/io_stress/io_stress_node ]; then
        echo "Building io_stress package..."
        cd ../..
        colcon build --base-paths test/io_stress --symlink-install
        source install/setup.bash
        cd test/io_stress
    fi

    # Run a quick 5-second test
    echo "Running io_stress for 5 seconds..."
    timeout 5 play_launch launch io_stress io_stress.launch.xml --enable-monitoring || true

    # Verify record.json was created
    if [ -f record.json ]; then
        echo "✅ io_stress test passed (record.json created)"
    else
        echo "❌ io_stress test failed (no record.json)"
        exit 1
    fi

    make clean
    cd ../..
    echo ""

    # Test 3: set_parameter_test
    echo "=========================================="
    echo "Test 3/3: set_parameter_test"
    echo "=========================================="
    cd test/set_parameter_test

    # Test Python launch file
    echo "Testing test_set_param.launch.py..."
    play_launch dump launch test_set_param.launch.py
    if [ -f record.json ]; then
        echo "✅ Python launch file test passed"
        rm -f record.json
    else
        echo "❌ Python launch file test failed"
        exit 1
    fi

    # Test XML launch file
    echo "Testing test_set_param.launch.xml..."
    play_launch dump launch test_set_param.launch.xml
    if [ -f record.json ]; then
        echo "✅ XML launch file test passed"
        rm -f record.json
    else
        echo "❌ XML launch file test failed"
        exit 1
    fi

    cd ../..
    echo ""

    echo "=================================================="
    echo "✅ All workspace tests passed!"
    echo "=================================================="
