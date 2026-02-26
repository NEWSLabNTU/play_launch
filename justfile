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

# Full build: colcon + bundle + wheel
build:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    colcon build {{colcon_flags}} --base-paths src
    scripts/bundle_wheel.sh
    uv build --wheel
    echo ""
    echo "Build complete:"
    ls -lh dist/*.whl

# C++ only (msgs + container)
build-cpp:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    colcon build {{colcon_flags}} --packages-select play_launch_msgs play_launch_container --base-paths src

# Rust only (assumes C++ install/ exists)
build-rust:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    colcon build {{colcon_flags}} --packages-select play_launch --base-paths src

# Rust with WASM features (assumes C++ install/ exists)
build-wasm:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash
    cd src/play_launch && cargo build --release --features wasm --config ../../build/ros2_cargo_config.toml

# Bundle colcon artifacts + build wheel (no colcon rebuild)
build-wheel:
    #!/usr/bin/env bash
    set -e
    scripts/bundle_wheel.sh
    uv build --wheel

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

# Clean build artifacts
clean:
    rm -rf build install log dist target
    rm -rf tests/target
    rm -rf src/play_launch_parser/target src/play_launch_parser/build src/play_launch_parser/install src/play_launch_parser/log
    rm -rf .cargo/config.toml
    rm -rf play_log tmp *.egg-info
    scripts/bundle_wheel.sh --clean

# Deep clean (clean + node_modules, __pycache__, etc.)
clean-all: clean
    rm -rf node_modules __pycache__ python/**/__pycache__
    rm -f record.json junit.xml .tmp-extracted.js .tmp-extracted.css

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
    echo "=== C++ build check ==="
    source /opt/ros/{{ros_distro}}/setup.bash
    colcon build --packages-select play_launch_msgs play_launch_container --symlink-install --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1
    echo ""
    echo "=== Parser unit tests ==="
    (cd src/play_launch_parser && cargo nextest run -p play_launch_parser --no-fail-fast --failure-output final)
    echo ""
    echo "=== Integration tests (fast) ==="
    (cd tests && cargo nextest run -E 'not binary(autoware) & not binary(io_stress) & not test(/launch/)' --no-fail-fast --failure-output final)

# Run all tests — parser unit + all integration including Autoware (~30s)
test-all:
    #!/usr/bin/env bash
    set -e
    echo "=== C++ build check ==="
    source /opt/ros/{{ros_distro}}/setup.bash
    colcon build --packages-select play_launch_msgs play_launch_container --symlink-install --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1
    echo ""
    echo "=== Parser unit tests ==="
    (cd src/play_launch_parser && cargo nextest run -p play_launch_parser --no-fail-fast --failure-output final)
    echo ""
    echo "=== Integration tests (all) ==="
    (cd tests && cargo nextest run --no-fail-fast --failure-output final)

# Run parser unit tests only
test-unit:
    #!/usr/bin/env bash
    set -e
    cd src/play_launch_parser
    cargo nextest run -p play_launch_parser --no-fail-fast --failure-output final

# Run all integration tests (simple + Autoware)
test-integration:
    #!/usr/bin/env bash
    set -e
    cd tests
    cargo nextest run --no-fail-fast --failure-output final

# Run simple workspace integration tests
test-simple:
    #!/usr/bin/env bash
    set -e
    cd tests
    cargo nextest run -E 'binary(simple_workspace)' --no-fail-fast --failure-output final

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
    cargo nextest run -E 'binary(autoware)' --no-fail-fast --failure-output final

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

# Run checks (clippy + rustfmt check + ruff + cpplint + clang-format)
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

    echo ""
    echo "=== C++ (cpplint) ==="
    ament_cpplint {{cpp_packages}}

    echo ""
    echo "=== C++ (clang-format check) ==="
    ament_clang_format {{cpp_packages}}

# Generate TypeScript bindings from Rust types (ts-rs)
generate-bindings:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash
    echo "Generating TypeScript bindings..."
    (cd src/play_launch && cargo test --config ../../build/ros2_cargo_config.toml -- export_bindings)
    echo "Generated $(ls src/play_launch/bindings/*.ts | wc -l) type files in src/play_launch/bindings/"

# Check Web UI (HTML, CSS, JavaScript, TypeScript types)
check-web-ui:
    #!/usr/bin/env bash
    set -e

    # Check if node_modules exists
    if [ ! -d "node_modules" ]; then
        echo "Installing Node.js dependencies..."
        npm install
    fi

    echo "=== Web UI lint ==="
    npm run lint:html
    npm run lint:css
    npm run lint:js

    echo ""
    echo "=== TypeScript type check ==="
    npm run typecheck

# C++ packages to lint/format
cpp_packages := "src/play_launch_container"

# Format code (Rust + Python + C++)
format:
    #!/usr/bin/env bash
    (cd src/play_launch && cargo +nightly fmt)
    (cd src/play_launch_parser && cargo +nightly fmt)
    ruff format python/
    ament_clang_format --reformat {{cpp_packages}}

# Run format and check
quality:
    #!/usr/bin/env bash
    set -e
    echo "Running format..."
    just format
    echo "Running checks..."
    just check

# Run Autoware planning simulation using the project build
run-autoware *ARGS:
    #!/usr/bin/env bash
    set -e
    source tests/fixtures/autoware/activate_autoware.sh
    source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export DISPLAY=${DISPLAY:-:1}
    ros2 run play_launch play_launch launch \
        --web-addr 0.0.0.0:8080 \
        {{ARGS}} \
        autoware_launch planning_simulator.launch.xml \
        map_path:="$HOME/autoware_map/sample-map-planning" \
        vehicle_model:=sample_vehicle \
        sensor_model:=sample_sensor_kit

# Profile play_launch during Autoware execution (default DDS config)
profile-autoware:
    scripts/profile_autoware.sh

# Profile play_launch with optimized CycloneDDS config
profile-autoware-tuned:
    scripts/profile_autoware.sh --dds-tuned

# Check version consistency across all files
version-check:
    python3 scripts/bump_version.py --check

# Bump version (patch, minor, or major)
bump-version TYPE:
    python3 scripts/bump_version.py {{TYPE}}

# Set explicit version
set-version VERSION:
    python3 scripts/bump_version.py --set {{VERSION}}
