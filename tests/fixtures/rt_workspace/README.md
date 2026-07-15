# rt_workspace

A small, real, buildable ROS 2 workspace that is both:

1. **The runnable example** behind [`docs/guide/rt-scheduling.md`](../../../docs/guide/rt-scheduling.md)
   — clone, build, `--sched`, observe `SCHED_FIFO` on every thread.
2. **An integration fixture** exercising the RT apply-layer (Phase 38)
   against our own nodes, with a committed `system.toml` and manifest under
   test.

## Layout

```
rt_workspace/
├── justfile                              # build / dump / check / run
├── src/rt_demo/                          # ament_cmake package: rclcpp + std_msgs + rclcpp_components
│   ├── sensor_node                       # 100 Hz publisher on points_raw
│   ├── control_node                      # sub points_raw -> pub cmd (the FIFO-20 node)
│   └── filter_component                  # composable: sub points_raw -> pub points_filtered
├── launch/
│   ├── bringup.launch.xml                # sensor_node + control_node + 1 container w/ filter_component
│   └── bringup.contract.yaml             # provider sidecar (types, rates, max_age_ms)
├── contracts/rt_demo/launch/
│   └── bringup.contract.yaml             # user overlay: tightens control_node's max_age_ms
└── system.toml                           # control tier (FIFO 20, core 0), perception tier (FIFO 10)
```

`/perception` namespace: `sensor_node` + `perception_container` (loading
`filter_component`). `/control` namespace: `control_node` — the node
`system.toml`'s `[[assign]]` pins to `SCHED_FIFO` priority 20 on core 0.

## Build

```bash
just build
```

Builds `rt_demo` into this fixture's own `install/` (a separate colcon
workspace from the repo root — this fixture does not touch the repo's
`install/`, only reads it for the `play_launch` binary).

## Run

```bash
just run
```

Launches `bringup.launch.xml` with `system.toml` applied in warn mode (no
root or `setcap` required — the `play_launch_rt_helper` handles privileged
syscalls if capability-granted; otherwise you'll see benign warnings and
nodes still run, just without RT scheduling).

## Observe

```bash
pid=$(cat play_log/latest/node/control_node/pid)
for t in /proc/$pid/task/*; do chrt -p ${t##*/}; done   # per-thread policy
taskset -cp $pid                                        # affinity
```

See [`docs/guide/rt-scheduling.md`](../../../docs/guide/rt-scheduling.md) for
the full configuration reference, or `just verify-sched-rt` at the repo root
for an automated version of the check above.

## Validate the contract + sched spec

```bash
just check
```

Runs `play_launch check` twice: once against the provider sidecar
(`launch/bringup.contract.yaml`, shipped in `rt_demo`'s installed share dir)
and once with the user overlay (`contracts/`) layered on top, which
overrides `control_node`'s `max_age_ms` — demonstrating overlay-beats-provider
resolution. Both also validate `system.toml` for the `posix` (Linux RT)
target.

## Compare parsers

```bash
just dump-rust
just dump-python
just compare-dumps
```
