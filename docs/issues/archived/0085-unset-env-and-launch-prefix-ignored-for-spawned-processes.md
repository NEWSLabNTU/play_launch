---
id: 85
title: "`<unset_env>` in a scoped group, and a container's `launch-prefix`, do not reach the spawned process; `ros2 launch` applies both"
status: resolved
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

## Resolution (2026-11-17)

Fixed with #0087 (`cwd`, GitHub #27): what a process starts IN and UNDER
now reaches it, on every frontend and both start paths. The model carries it
(ros-launch-manifest v0.3.28, `NodeInstance::{env_unset, launch_prefix,
cwd}`), so `up <model.yaml>` applies what `launch` applies.

- **An unset is not a missing key.** The parser's environment overlay was a
  map of the variables launch SET, and `<unset_env>` removed a name from it:
  that undid a `<set_env>` and left an inherited variable (the shell's
  `DISPLAY`) in place. The overlay is now `name -> Option<value>`
  (`substitution::context::EnvOverlay`), `None` an unset, through scoped
  group push/pop, includes and the state that crosses to a `.launch.py`
  (`ContextState::environment_unset`, **pyexec C ABI 11 -> 12** on both
  sides). `$(env X)`/`$(optenv X)` and `EnvironmentVariable` after an unset
  read the variable as unset, as launch's `context.environment` does.
- Each process record carries `env_unset` (the unsets in effect where it was
  declared, minus names its own `<env>` sets again and names the launcher's
  environment lacks) and the spawn removes them before applying the launch
  file's variables (`NodeCommandLine::to_command`).
- **The prefix survives container replacement.** `launch-prefix` (XML, YAML)
  and `prefix=` (Python `Node`, `ExecuteProcess`, `ComposableNodeContainer`,
  `LifecycleNode`) are resolved by launch's rule
  (`actions/process.rs`: the action's own prefix, else the `launch-prefix`
  configuration filtered by `launch-prefix-filter`), split with shell rules,
  and prepended to argv, in front of `play_launch_container` too when
  `--container-mode` replaced the container's binary. The `cmdline` file
  shows it. Under `shell="true"` the prefix is part of the shell's command
  line, as launch joins it before the shell runs.
- The Python dumper (`--parser python`) records the same fields
  (dump format 3), and no longer leaves a launch file's `<set_env>`/
  `<unset_env>` in play_launch's own process environment after the parse:
  launch writes `os.environ` itself, and the dumper runs in-process, so a
  top-level unset used to reach every process, those declared before it too.

Tests: `tests/tests/process_spawn.rs` runs `ros2 launch` and play_launch on
the same fixtures (`tests/fixtures/process_spawn/`, XML and Python, both
parsers, and `up` from a resolved model) and requires each process's
environment and working directory to match, `env -u` and wrapper prefixes
on an `<executable>`, a `<node>` and a `<node_container>` under the default
mode included. Against `main` before the fix all six fail. Parser:
`process_spawn_tests.rs`; model: `processes_carry_env_unset_launch_prefix_and_cwd`.

Not covered here: a `.launch.py` `env=` (a CLEAN environment in launch) is
still laid over play_launch's own environment rather than replacing it.
