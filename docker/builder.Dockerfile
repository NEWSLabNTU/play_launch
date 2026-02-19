# Pre-built CI image for play_launch builds, tests, and wheel releases.
# Contains ROS2 Humble, Rust, and all build/test/lint dependencies.
#
# Rebuild when:
#   - ROS2 packages are added/changed in CMakeLists.txt or package.xml
#   - Rust toolchain version needs updating
#   - Build tool versions change (colcon-cargo-ros2, uv, just, etc.)
#
# The image is built and pushed by .github/workflows/builder-image.yml
# and consumed by .github/workflows/ci.yml and release-wheel.yml.
#
# Manual build (amd64 only, for testing):
#   docker build -f docker/builder.Dockerfile -t play-launch-builder:humble .

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# System packages + ROS2 Humble
# ---------------------------------------------------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      software-properties-common curl git build-essential pkg-config ca-certificates && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-pip python3-dev python3-venv python3-colcon-common-extensions \
      ros-humble-ros-base \
      ros-humble-ament-index-python \
      ros-humble-launch \
      ros-humble-rcl \
      ros-humble-rcutils \
      ros-humble-rmw \
      ros-humble-rosidl-typesupport-c \
      ros-humble-composition-interfaces \
      ros-humble-rcl-interfaces \
      ros-humble-builtin-interfaces \
      ros-humble-action-msgs \
      ros-humble-unique-identifier-msgs \
      ros-humble-rosgraph-msgs \
      ros-humble-example-interfaces \
      ros-humble-test-msgs \
      ros-humble-rclcpp-components \
      ros-humble-class-loader \
      ros-humble-ament-index-cpp \
      ros-humble-ros2launch \
      ros-humble-demo-nodes-cpp \
      ros-humble-ament-cpplint \
      ros-humble-ament-clang-format && \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Rust (stable)
# ---------------------------------------------------------------------------
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
    sh -s -- -y --default-toolchain stable --profile minimal && \
    rustc --version

# ---------------------------------------------------------------------------
# cargo-nextest (test runner)
# ---------------------------------------------------------------------------
RUN cargo install cargo-nextest --locked

# ---------------------------------------------------------------------------
# Python build/lint tools
# ---------------------------------------------------------------------------
RUN pip3 install --no-cache-dir colcon-cargo-ros2 build 'wheel>=0.40' ruff

# ---------------------------------------------------------------------------
# just (command runner)
# ---------------------------------------------------------------------------
RUN curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | \
    bash -s -- --to /usr/local/bin

# ---------------------------------------------------------------------------
# uv (fast Python package installer / wheel builder)
# ---------------------------------------------------------------------------
RUN curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR=/usr/local/bin sh

WORKDIR /workspace
