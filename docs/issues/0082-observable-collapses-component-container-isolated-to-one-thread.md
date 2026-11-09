---
id: 82
title: "`observable` runs a `component_container_isolated` on one thread: an Isaac ROS H.264 encoder beside the ZED X then gets each frame twice"
status: open
type: defect
severity: high
---

# 0082 - observable collapses `component_container_isolated` to one thread

**Repo:** `play_launch` 0.30.0 (wheel, ROS humble, rlm v0.3.12); code read at
`8f29baad` (0.31.0)
**Affects:** `src/play_launch/src/execution/context.rs`
(`prepare_container_contexts_from_model`, the `--container-mode` override);
`src/play_launch_container/src/component_container.cpp` (executor choice)
**Found:** 2026-golf-cart, orin (AGX Orin, JetPack 6.2), 2026-11-10, turning
on ZED X stereo recording.

## What happens

The ZED X container (`zed.launch.xml` in golfcart_sensor_kit_launch) is a
`component_container_isolated` with `use_multi_threaded_executor: true`,
holding five components: `stereolabs::ZedCamera` and, per side, Isaac ROS 3.2
`ImageFormatConverterNode` (bgr8 to rgb8) and `h264_encoder::EncoderNode`
(NVENC). Encoder outputs: `/sensing/camera/zed/{left,right}/color/rect/image/h264`.

Probe: count messages and distinct `header.stamp`s on both H.264 topics over
8 s, with a plain-ROS subscriber on the wrapper's RGB JPEG
(`/sensing/camera/zed/rgb/color/rect/image/compressed`) as the recorder has.

| `--container-mode` | left h264 | right h264 | JPEG |
|---|---|---|---|
| `observable` | 120 msgs, 120 stamps | **241 msgs, 121 stamps** | 119 |
| `stock` | 122, 122 | 123, 123 | 118 |
| `isolated` | 15 Hz | 15 Hz | 15 Hz |
| `observable`, no JPEG subscriber | 239, 239 | 238, 238 | - |

Under `observable` one encoder emits every frame twice: the same stamp, the
first copy a normal frame (~280 kB), the second ~3 kB, an encoding of an
identical image. Which side is hit varies by run: an earlier observable run
had left at 30 Hz against right at 15. `stock` is clean. Without the extra
JPEG subscriber, observable is clean too, so it appears under load. A rosbag
recorded through it has one side's video doubled and its timing wrong; nothing
logs anything.

## Why (likely)

`prepare_container_contexts_from_model` replaces the container executable with
`play_launch_container` and adds `--use_multi_threaded_executor` only when the
launch file's executable is `component_container_mt`. A
`component_container_isolated` therefore becomes an observable container with
ONE `SingleThreadedExecutor` (`component_container.cpp`) for every component,
where the stock executable gives each component its own executor and thread
(multi-threaded when `use_multi_threaded_executor` is true, which is a ROS
parameter there, not a CLI flag, and play_launch_container reads neither).

So five components that each had a thread, three of them GXF/NITROS graphs
with their own scheduling, share one. That the starved encoder then receives
an input twice is NOT explained: NITROS delivers to an Isaac subscriber over
both its negotiated and its plain-ROS path, and the duplicate may come from
that hand-off racing under starvation. Not verified; the threading difference
is.

Separate from this: with the ZED wrapper's own NITROS output on
(`debug.disable_nitros: false`), the right encoder received both chains and
the left none under plain `ros2 launch` in the stock container, so Isaac ROS
has a duplication of its own. That one is not play_launch's.

## Reproduce

```bash
# stack down; ZED X and isaac_ros_h264_encoder installed
play_launch launch --container-mode observable \
  golfcart_sensor_kit_launch camera.launch.xml camera_model:=zedx zed_stereo_video:=true
# elsewhere: subscribe to .../rgb/color/rect/image/compressed, and count
# header.stamp duplicates on .../{left,right}/color/rect/image/h264
```

Any container whose stock executable is `component_container_isolated` (or a
`component_container` with `use_multi_threaded_executor` set as a parameter)
should show the same threading change; the ZED stereo graph is just where it
became visible.

## Expected

The observable container keeps the stock container's executor model:
`component_container_isolated` gets an executor (and thread) per component,
and `use_multi_threaded_executor` is honoured whether it comes as a flag or a
parameter.

## Workaround

`--container-mode stock` for the affected host. 2026-golf-cart runs its orin
in `stock` (`config/runtime.conf`) since 2026-11-10.
