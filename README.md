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
third-party/ros-launch-manifest   the spec
```

## Status

Extracted from `play_launch` with history preserved (`git log --follow` works
across the move). Wiring the crate boundaries is in progress — see nano-ros
phase-312.
