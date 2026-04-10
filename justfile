# Default cargo profile for builds and tests
cargo_profile := "dev-release"

# Detect ROS2 distro based on Ubuntu version
ros_distro := if `lsb_release -rs 2>/dev/null || echo "22.04"` == "24.04" { "jazzy" } else { "humble" }
colcon_flags := "--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release"

# Show available recipes
default:
    @just --list

# Install all dependencies (rosdep + colcon-cargo-ros2)
install-deps:
    #!/usr/bin/env bash
    set -e

    # Install colcon-cargo-ros2 for Rust support
    pip install colcon-cargo-ros2

    source /opt/ros/{{ros_distro}}/setup.bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

# Build all packages (Rust + colcon + Python wheel)
build: build-rust build-colcon build-python

# Build all packages with colcon
build-colcon:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash

    echo "Building packages with colcon..."
    colcon build {{colcon_flags}} --base-paths src --cargo-args --profile {{cargo_profile}}

# Build Rust packages only
build-rust:
	cargo build --all-targets --profile {{cargo_profile}}

# Run all tests (Rust + comparison + Autoware if available)
test:
    #!/usr/bin/env bash
    set -e

    echo "========================================="
    echo "Running all tests..."
    echo "========================================="

    # Run Rust unit tests
    just test-rust

    # Run comparison tests if directory exists
    if [ -d "tests/comparison_tests" ]; then
        echo ""
        echo "========================================="
        echo "Running comparison tests..."
        echo "========================================="
        just test-compare
    fi

    # Run Autoware tests if symlink exists
    if [ -L "tests/autoware_test/autoware" ]; then
        echo ""
        echo "========================================="
        echo "Running Autoware validation tests..."
        echo "========================================="
        just test-autoware
    fi

    echo ""
    echo "========================================="
    echo "✓ All tests passed!"
    echo "========================================="

# Run Rust unit tests (274 tests with nextest)
test-rust:
	cargo nextest run --cargo-profile {{cargo_profile}} --no-fail-fast

# Run colcon tests (ROS 2 integration tests)
test-colcon:
    #!/usr/bin/env bash
    set -e
    source /opt/ros/{{ros_distro}}/setup.bash
    source install/setup.bash

    colcon test --base-paths src
    colcon test-result --all --verbose

# Run comparison tests (Rust vs Python parser)
test-compare:
    #!/usr/bin/env bash
    set -e
    cd tests/comparison_tests
    ./run_tests.sh

# Run linters and formatters (clippy + rustfmt check)
check:
    #!/usr/bin/env bash
    # Run all checks even if some fail, then return error if any failed
    exit_code=0

    echo "Running rustfmt check..."
    if ! cargo +nightly fmt -- --check; then
        echo "❌ Format check failed"
        exit_code=1
    else
        echo "✓ Format check passed"
    fi

    echo ""
    echo "Running clippy..."
    if ! cargo clippy --profile {{cargo_profile}} --all-targets -- -D warnings; then
        echo "❌ Clippy check failed"
        exit_code=1
    else
        echo "✓ Clippy check passed"
    fi

    exit $exit_code

# Run quality checks (linters + Rust tests)
quality: check test-rust
    @echo ""
    @echo "========================================="
    @echo "✓ All quality checks passed!"
    @echo "========================================="

# Format all code with rustfmt
format:
    cargo +nightly fmt

# Clean all build artifacts
clean:
    rm -rf build install log target

# List ROS 2 packages in workspace
list-packages:
    #!/usr/bin/env bash
    source /opt/ros/{{ros_distro}}/setup.bash
    colcon list --base-paths src

# Download demo packages for testing
download-demos:
    #!/usr/bin/env bash
    set -e
    cd src

    if [ ! -d "demos" ]; then
        echo "Downloading ROS 2 demos..."
        git clone https://github.com/ros2/demos.git -b {{ros_distro}}
    else
        echo "demos directory already exists, skipping..."
    fi

    cd ..
    echo "Run 'just build' to compile the demos"

# Test Rust parser with Autoware launch files
test-autoware:
    #!/usr/bin/env bash
    set -e
    cd tests/autoware_test/scripts

    if [ ! -f "activate_autoware.sh" ]; then
        echo "ERROR: activate_autoware.sh script not found"
        echo "Create script: tests/autoware_test/scripts/activate_autoware.sh"
        echo "Example content:"
        echo "  #!/usr/bin/env bash"
        echo "  source /path/to/your/autoware/workspace/install/setup.bash"
        exit 1
    fi

    # Run comparison with default launch file (no args needed)
    ./compare_rust_python.py

# Compare Rust and Python parser outputs for Autoware (alias for test-autoware)
compare-autoware: test-autoware

# Build Python extension module and wheel
build-python:
    #!/usr/bin/env bash
    set -e
    cd crates/python
    if [ ! -d .venv ]; then
        python3 -m venv .venv
        .venv/bin/pip install maturin
    fi
    .venv/bin/maturin develop --release
    rm -f target/wheels/*.whl
    .venv/bin/maturin build --release
    echo ""
    echo "Wheel:"
    ls -1 target/wheels/*.whl

# Test Python bindings (import + smoke test)
test-python:
    #!/usr/bin/env bash
    set -e
    cd crates/python
    .venv/bin/python -c "import play_launch_parser; print(f'play_launch_parser {play_launch_parser.__version__}'); print('OK')"

# Benchmark Rust parser performance with Autoware
benchmark-autoware:
    #!/usr/bin/env bash
    cd tests/autoware_test
    ./scripts/benchmark.sh
