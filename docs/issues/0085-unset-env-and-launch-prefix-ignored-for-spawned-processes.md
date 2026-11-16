---
id: 85
title: "`<unset_env>` in a scoped group, and a container's `launch-prefix`, do not reach the spawned process; `ros2 launch` applies both"
status: open
type: defect
severity: high
---

# 0085 - `<unset_env>` and a container's `launch-prefix` are ignored for the spawned process

**Repo:** `play_launch` 0.35.1 (pip, measured) and 0.18.2 (same behaviour);
code read at `eb8758ae` (0.36.0)
**Found:** AutoSDV, car B (AGX Orin, JetPack 6.2, Humble), 2026-11-17
**Affects:** the parser's `UnsetEnvAction`
(`play_launch_parser/src/actions/set_env.rs`) and the `launch-prefix`
attribute (accepted in `xml/attr_spec.rs`): neither reaches the spawn.

## Why it matters

The ZED X camera (Argus) fails at "=== CAMERA OPENING ===" with "(Argus)
Error BadParameter" when its process has `DISPLAY` or VirtualGL's
`LD_PRELOAD` faker set. Every terminal of a TurboVNC desktop has both.
Students launch from that desktop, so the ZED container must drop both
variables while the rest of the launch, RViz included, keeps them. Under
`ros2 launch` either mechanism does that. Under play_launch neither does,
so the ZED dies partway through startup and the student cannot tell why.

## Repro

`envtest.launch.xml`:

```xml
<launch>
  <group scoped="true">
    <unset_env name="DISPLAY"/>
    <unset_env name="LD_PRELOAD"/>
    <node_container pkg="rclcpp_components" exec="component_container" name="c_unset" namespace="/envtest"/>
  </group>
  <node_container pkg="rclcpp_components" exec="component_container" name="c_prefix" namespace="/envtest" launch-prefix="env -u DISPLAY -u LD_PRELOAD"/>
  <node_container pkg="rclcpp_components" exec="component_container" name="c_plain" namespace="/envtest"/>
</launch>
```

```sh
export DISPLAY=:2 LD_PRELOAD=libdlfaker.so
play_launch launch envtest.launch.xml &    # vs: ros2 launch envtest.launch.xml
sleep 15
for n in c_unset c_prefix c_plain; do
  p=$(pgrep -f "__node:=$n" | head -1)
  echo "$n: $(tr '\0' '\n' </proc/$p/environ | grep -E '^(DISPLAY|LD_PRELOAD)=')"
done
```

## Measured

- `c_unset`: `ros2 launch` has neither variable; play_launch has both.
- `c_prefix`: `ros2 launch` has neither; play_launch has both.
- `c_plain`: both under both launchers (correct).

play_launch's own record has no prefix
(`play_log/<run>/node/c_prefix/cmdline`):

```
.../play_launch/dist/humble/lib/play_launch_container/component_container --ros-args -r __ns:=/envtest -r __node:=c_prefix
```

Running play_launch's own container binary is expected; losing the
`env -u ...` in front of it is the bug.

## Expected

1. A process inside a scope with `<unset_env name="X"/>` starts without
   `X`, and `X` comes back after the scoped group, as in `ros2 launch`.
   Check `<set_env>` the same way (archived #0077 covered the Python
   dumper side).
2. `launch-prefix` on `<node_container>` (and `<node>`) is applied when the
   container is replaced by `play_launch_container`. If some mode cannot
   honour a prefix, fail loudly or warn rather than drop it silently.
3. A test comparing each process's environment with `ros2 launch`, along
   the lines of the repro.

## Workaround

None inside the launch file. Users must unset the variables in the shell
before play_launch, which also takes `DISPLAY` away from RViz.
