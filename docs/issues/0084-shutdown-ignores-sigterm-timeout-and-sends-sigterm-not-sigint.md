---
id: 84
title: "Shutdown ignores an action's `sigterm_timeout`/`sigkill_timeout` and starts with SIGTERM, where `ros2 launch` sends SIGINT and waits as declared"
status: open
type: defect
severity: medium
---

# 0084 - shutdown ignores `sigterm_timeout` and `sigkill_timeout`

**Repo:** `play_launch` 0.30.0 (measured); code read at `d7088cc1` (0.35.1), same behaviour
**Affects:** `src/play_launch/src/member_actor/regular_node_actor.rs` (`SHUTDOWN_SIGNAL_GRACE`,
the shutdown branch of the Running state), `src/play_launch/src/member_actor/container_actor/process_lifecycle.rs`
(the same for containers), `src/play_launch/src/commands/signal_handler.rs` (`initiate_shutdown`)
**Found:** scenario-world-bridge, roadmap 079 (ROS 2 compatibility), 2026-11-16: every
play_launch stack of swb ended its supervisor part way through its own stop.

## Principle

play_launch should behave as `ros2 launch` does wherever `ros2 launch` defines the behaviour.
A launch file written for `ros2 launch` declares how long each process may take to stop;
play_launch should honour it.

## What `ros2 launch` does (Humble)

On shutdown, for each `ExecuteProcess`/`Node`: SIGINT; after the action's `sigterm_timeout`
(default 5 s) SIGTERM; after a further `sigkill_timeout` (default 5 s) SIGKILL. Both are
per-action arguments (`ExecuteProcess(..., sigterm_timeout="20", sigkill_timeout="20")`).

## What play_launch does

1. `initiate_shutdown` sends **SIGTERM** to the process group (staggered), not SIGINT.
2. Each node actor then waits a fixed `SHUTDOWN_SIGNAL_GRACE` of **2 s**, logs
   `Still running 2s after shutdown began; stopping pid N directly`, and runs `graceful_kill`:
   SIGTERM, a fixed 5 s, SIGKILL.
3. The action's `sigterm_timeout` and `sigkill_timeout` are not read; there is no CLI option
   for the shutdown timeouts.
4. The console labels a received SIGINT as `(SIGTERM)`.

## Repro

A stub that ignores SIGINT and SIGTERM, declared with `sigterm_timeout=20`, `sigkill_timeout=20`:

```python
# stub.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
CODE = "import signal,time\nsignal.signal(signal.SIGINT, signal.SIG_IGN)\nsignal.signal(signal.SIGTERM, signal.SIG_IGN)\ntime.sleep(300)"
def generate_launch_description():
    return LaunchDescription([ExecuteProcess(cmd=["python3", "-c", CODE], name="stub",
                              sigterm_timeout="20", sigkill_timeout="20", output="screen")])
```

```bash
source /opt/ros/humble/setup.bash
play_launch launch --disable-web-ui --log-dir ./pl ./stub.launch.py > console.log 2>&1 &
P=$!; sleep 8; kill -INT $P; T0=$(date +%s); wait $P
echo "exited $(( $(date +%s) - T0 )) s after SIGINT"
```

| launcher | after one SIGINT |
|---|---|
| `ros2 launch` (Humble) | SIGINT to the stub, SIGTERM at 20 s, SIGKILL at 40 s, exit at 40 s |
| play_launch 0.30.0 | SIGTERM, "stopping pid directly" at 2 s, SIGKILL 5 s later, exit at 10 s |

## Why it matters

A supervisor process that needs time to stop its own children in order (swb's stack keeper
stops its pilots, then a CARLA server: up to about 70 s, declared `sigterm_timeout="90"`) is
killed after about 7 s. Its children are left to their own backstops; a CARLA server killed
that way can keep its RPC port, and the next server on that port crashes. swb now works around
it by asking its keeper to stop before stopping the launch; a launch file alone cannot.

A node that handles SIGINT but not SIGTERM (common in Python: `KeyboardInterrupt`) also takes a
different shutdown path under play_launch than under `ros2 launch`.

## Proposed fix

- On shutdown send each process **SIGINT** first, as `ros2 launch` does.
- Read `sigterm_timeout` and `sigkill_timeout` from the action (the parsers already see the
  `ExecuteProcess`/`Node` arguments) and use them per process, defaulting to 5 s and 5 s as
  `ros2 launch` does, instead of the fixed 2 s grace and 5 s ladder. Keep the staggered
  process-group order.
- Optionally, a CLI override for the defaults (`--sigterm-timeout`, `--sigkill-timeout`).
- Log the signal actually received (SIGINT vs SIGTERM).
- Test: the stub above exits at 40 s under play_launch, with SIGINT, then SIGTERM at 20 s,
  then SIGKILL at 40 s; a stub with no timeouts declared exits at the 5 s + 5 s defaults.
