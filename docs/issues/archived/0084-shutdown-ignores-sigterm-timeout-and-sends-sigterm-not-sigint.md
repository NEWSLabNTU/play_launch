---
id: 84
title: "Shutdown ignores an action's `sigterm_timeout`/`sigkill_timeout` and starts with SIGTERM, where `ros2 launch` sends SIGINT and waits as declared"
status: resolved
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

## Resolution (2026-11-17)

play_launch now stops each process as `ros2 launch` does: SIGINT, then
SIGTERM after the action's `sigterm_timeout`, then SIGKILL after its
`sigkill_timeout`, 5 s and 5 s when it declares none.

- **The timeouts reach the model** (ros-launch-manifest
  `NodeInstance::{sigterm_timeout, sigkill_timeout}`, `None` for 5). Both
  parsers resolve them in one rule (`actions/process.rs::resolve_timeout`,
  the dumper's `exit_probe.stop_timeouts`): the action's own value, else
  the launch configuration of the same name, since launch's default is
  literally `LaunchConfiguration('sigterm_timeout', default=5)`. The XML
  and YAML attributes (Jazzy) are read and no longer warned about as
  ignored; a value that is not a number is a parse error naming it. pyexec
  C ABI 13 -> 14 (13 went to #0093 first), Python dump format 3 -> 4.
- **SIGINT first.** `initiate_shutdown` still leaves the ROS graph first and
  still signals the group one process at a time, deepest first (the Fast
  DDS teardown race of 2026-10-15), now with SIGINT
  (`pgid::stop_process_group_staggered`), and marks the group signalled.
- **The ladder is per process** (`process/stop.rs::stop_at_shutdown`), run
  by each node and container actor from the moment the group's SIGINT was
  sent: wait `sigterm_timeout`, SIGTERM the pid, wait `sigkill_timeout`,
  SIGKILL. The fixed 2 s grace and 5 s ladder are gone. A member whose
  group was never signalled sends its own SIGINT after 5 s. The log lines
  are launch's: `failed to terminate '20' seconds after receiving 'SIGINT',
  escalating to 'SIGTERM'`.
- **What outlives the launch's processes** is stopped once every actor has
  ended (`pgid::stop_leftovers`): SIGTERM, staggered, SIGKILL 2 s later.
  `ros2 launch` leaves such orphans running; play_launch used to kill them
  with its group SIGTERM, and a shell's background job ignores the SIGINT
  that replaced it.
- **The console names the signal received**: `Received SIGINT; shutting
  down gracefully...` (it always said SIGTERM).

Not done: a CLI override for the default timeouts (the issue's optional
item); the `sigterm_timeout` launch configuration counts as it stands where
the process is declared, where launch reads it at shutdown, which differs
only when a launch file changes it after the process or inside a scoped
group. The second Ctrl-C still forces SIGTERM on the group, the third
SIGKILL (`ros2 launch` ignores a second SIGINT).

Measured with the issue's stub (20 s + 20 s, ignores both signals): `ros2
launch` exits 40.0 s after SIGINT; play_launch 42.2 s, SIGTERM at 20.1 s and
SIGKILL at 40.1 s, the last 2 s being play_launch's existing background-task
drain.

Tests: `tests/tests/shutdown_semantics.rs` runs `ros2 launch` and
play_launch (both parsers, and `up` from a resolved model) on
`tests/fixtures/shutdown/timeouts.launch.py`, two probes that record the
signals they receive, one declared 2 s + 2 s and one with the defaults, and
checks SIGINT at once, SIGTERM at 2 s and 5 s, and the exit at about 10 s.
Parser: `shutdown_tests.rs` (XML, YAML, Python, the configuration fallback,
a non-number refused). Negative control: with the source change reverted,
the three play_launch ladder tests fail.
