# ros-launch-resolve

Resolve a ROS 2 launch tree into a checked **SystemModel**.

This is the middle layer of the launch toolchain (RFC-0060):

| layer | repo | needs |
| --- | --- | --- |
| spec, theory, proofs, algorithms | [`ros-launch-manifest`](https://github.com/NEWSLabNTU/ros-launch-manifest) | nothing beyond serde |
| **launch tree → SystemModel** | **this repo** | CPython (for `.launch.py`) |
| Linux runtime + binary | [`play_launch`](https://github.com/NEWSLabNTU/play_launch) | ROS, rclrs, colcon |

## Why it is its own repo

The resolve pipeline used to live inside `play_launch`, next to the ROS graph
client and the process runtime. Anything wanting only "launch tree →
SystemModel" had to link all of it — including `play_launch_msgs`, which is not
a registry crate but is generated from an ament environment by
`colcon-cargo-ros2`. So reusing the resolver implied installing ROS and running
play_launch's colcon setup, which is impossible for the embedded consumers that
want it most.

Splitting it out makes the dependency honest: **this workspace builds under
plain `cargo`, with no ROS, no ament and no colcon.**

## Layout

```
resolve/   the pipeline: launch dump -> manifests -> model -> schedule
cli/       the `ros-launch-resolve` binary (resolve / dump / contract / plot)
parser/    play_launch_parser — launch XML and .launch.py (pyo3)
scripts/   check-layer2-isolation.sh — the no-ROS gate
tests/     isolation fixtures for that gate
```

`ros-launch-manifest` (the spec) is a **git dependency pinned by tag**, not a
vendored directory — phase-55 W2. It is the one thing both this repository's
parent and nano-ros link, so both pin the same tag rather than each carrying a
submodule pointer to it.

## Status

Lives inside the `play_launch` repository as a **separate cargo workspace**
(play_launch phase-55 W1, nano-ros RFC-0060 as amended). It was a standalone
repository until 2026-08-02; that history came across intact.

The separation that matters is the workspace `exclude` in play_launch's root
manifest, which keeps `rclrs`/`rosidl` out of this graph, plus the fact that
the resolver ships as a *binary*, which keeps `libpython` out of a consumer's
link. `scripts/check-layer2-isolation.sh` enforces both.
