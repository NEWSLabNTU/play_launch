---
id: 28
title: "Rust parser dies with `KeyError: 'rear_overhang'` on the golf-cart stack; the Python parser resolves it"
status: open
type: correctness
severity: medium
---

# 0028 — `KeyError: 'rear_overhang'` from a Python launch file under the Rust parser

**Repo:** `play_launch` (`src/ros-launch-resolve/parser`, the embedded-Python execution
of `.launch.py` includes)
**Affects:** the Rust parser (the default) on any launch tree that includes an Autoware
`.launch.py` reading `vehicle_info["rear_overhang"]`

## What happens

play_launch 0.10.0, ROS 2 Humble, Autoware 1.5.0 apt, 2026-09-11:

```
$ play_launch resolve --parser rust golfcart_launch indoor_logging_sim.launch.xml -o out.yaml
Error: Rust parser error while parsing golfcart_launch: Python error: Python error: KeyError: 'rear_overhang'
Hint: if this is a parser limitation rather than a bad launch file, re-run the same command with `--parser python`
```

`--parser python` resolves the same launch, and stock `ros2 launch` runs it. The
error names neither the launch file nor the line; the candidates are the Autoware
`.launch.py` files that read the vehicle-info dictionary, e.g.
`single_lidar_common_launch/launch/nebula_node_container.launch.py`,
`tier4_perception_launch/.../ground_segmentation.launch.py` (Autoware 1.5.0,
`/opt/autoware/1.5.0/share`).

## Where to look

Those files build the dictionary with `autoware_vehicle_info_utils`'s
`get_vehicle_info_param()` / `add_vehicle_info` and then index it by key. Under
`ros2 launch` the keys come from `vehicle_info.param.yaml` of the vehicle description
found through the launch context. The KeyError says the embedded execution handed the
file a dictionary without that key: either the vehicle-info YAML was not located (the
`vehicle_model` argument or `find-pkg-share` not reaching the Python side), or the
dictionary was built from a different source. Not diagnosed; recorded so the golf-cart
side stops treating it as #0027.

## Consequence

The golf-cart replays (`just indoor-test up`, `just ntu-test up`) and `just launch` stay
on `--parser python`. #0027 is fixed; this is what keeps them there now.

## First steps

1. Make the Python-error path name the launch file (and, if the traceback allows, the
   line) — the message is the expensive part, as in #0024.
2. Reproduce with one Autoware `.launch.py` alone under `ros-launch-resolve resolve`,
   with `vehicle_model` set, and compare the dictionary the two execution paths build.
