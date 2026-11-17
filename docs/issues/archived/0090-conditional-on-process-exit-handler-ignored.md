---
id: 90
title: "A conditional `OnProcessExit` handler (a callable `on_exit`) is ignored; a respawning node loops where `ros2 launch` shuts down"
status: resolved
type: defect
severity: medium
---

# 0090 - a conditional `OnProcessExit` handler is ignored

**Repo:** `play_launch` 0.35.0 and 0.36.0 (PyPI wheel), both parsers
**Affects:** the event-handler mocks in `src/ros-launch-resolve/parser/crates/pyexec/src/api/event_handlers.rs`
and the runtime's respawn path; #0081 covers only an `on_exit` that is a shutdown action
**Found:** scenario-world-bridge, roadmap 077 (its issue 044), 2026-11-17: a world server
refused for a usage error (exit 2) restarted every 10 s under play_launch, where `ros2 launch`
ended the launch once.

## Principle

play_launch should behave as `ros2 launch` does wherever `ros2 launch` defines the behaviour.
An event handler registered in a launch file is part of that behaviour.

## What `ros2 launch` does (Humble)

`OnProcessExit(target_action=X, on_exit=f)` with `f` a callable: when X exits, launch calls
`f(event, context)` and runs the actions it returns. A handler that returns `Shutdown` for one
exit code ends the launch on that code; on any other code it returns nothing and X's own
`respawn` takes over.

## What play_launch does

The handler is not evaluated at run time. #0081 (resolved 2026-11-08) recognises an `on_exit`
that *is* a shutdown action and marks the target required; a callable is not recognised, so
X respawns on every exit, the code it was meant to stop on included.

## Repro

```python
# exit2_conditional.launch.py
from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    stub = ExecuteProcess(cmd=["python3", "-c", "import sys; sys.exit(2)"], name="stub",
                          respawn=True, respawn_delay=2.0, output="screen")
    def on_exit(event, _context):
        if event.returncode != 2:
            return []
        return [LogInfo(msg="usage error: shutting down"),
                EmitEvent(event=Shutdown(reason="usage error"))]
    return LaunchDescription([stub, RegisterEventHandler(OnProcessExit(target_action=stub,
                                                                       on_exit=on_exit))])
```

```bash
source /opt/ros/humble/setup.bash
timeout 20 ros2 launch ./exit2_conditional.launch.py                       # 1 start, shut down
timeout 20 play_launch launch --disable-web-ui --log-dir log ./exit2_conditional.launch.py
timeout 20 play_launch launch --parser python --disable-web-ui --log-dir log ./exit2_conditional.launch.py
```

| launcher | starts in 20 s |
|---|---|
| `ros2 launch` (Humble) | 1, then the launch shuts down |
| play_launch 0.35.0, Rust parser / `--parser python` | 10 / 10 |
| play_launch 0.36.0, Rust parser / `--parser python` | 10 / 10 |

## Why it matters

A node with `respawn=True` that refuses its configuration (a usage error) loops forever under
play_launch instead of ending the launch with its message. swb now refuses every foreseeable
usage error at parse time to avoid it; an unforeseen one still loops.

## Proposed fix

Evaluate `OnProcessExit` handlers at run time as launch does: keep the callable (or the
action list) with its target, and on the target's exit call it with an event carrying
`returncode`, `pid`, `action` and `cmd`; run what it returns (at least `Shutdown`/`EmitEvent(Shutdown)`
and `LogInfo`), and suppress the respawn when the launch is shutting down. If a callable cannot
be run in the runtime, at least warn at parse time that a conditional handler is not honoured.
Test: the repro above starts the stub once and exits; with `sys.exit(1)` it respawns.

## Resolution (2026-11-17)

An `on_exit` that is a FUNCTION of the exit event, in an `OnProcessExit`
handler or given to the process itself, is honoured for the exit codes it
shuts down on.

**Design.** The runtime runs no Python, so the function cannot be called at
the exit. It is called at PARSE time instead, once for every code a process
can exit with (launch's `returncode`: a status 0 to 255, or `-N` for a death
by signal N, 1 to 64), with an event carrying `returncode`, `action`,
`pid`, `cmd`, `name` and the launch context, and the codes it returns a
shutdown for (`Shutdown()`, or an `EmitEvent` of `launch.events.Shutdown`)
are recorded. That is exact for a function of the exit code, which is what
such handlers are. It is refused with a warning, and nothing recorded, when
it is not: a function that raises, or one that answers differently when
called twice for the same code (it keeps state). Its prints are swallowed;
other actions it returns are reported as not run (`LogInfo` silently).
Rejected alternatives: embedding Python in the runtime (the layer-2 split
exists to avoid it), and a warning alone (the issue's minimum), which leaves
the loop.

- **Model**: ros-launch-manifest `NodeInstance::shutdown_on_exit_codes`, an
  `ExitCodes` written `only: [2]` or `except: [0]`, whichever is shorter. A
  function that shuts down on every code is `on_exit: shutdown`, which
  wins.
- **Rust parser** (`pyexec/src/api/exit_probe.rs`, `visit.rs`): the
  handler's bookkeeping from #0081 now carries which exits (`ExitOn`), so a
  handler registered before or after its target, several handlers on one
  target, and `ExecuteProcess(on_exit=<function>)` all merge. pyexec's
  warnings for these go to stderr when the dlopen'd object has no logger
  (the general case is #0093).
- **Python dumper** (`dump/exit_probe.py`, `visitor/action.py`): the same
  rule and spelling; `compare_models.py` compares `on_exit`,
  `shutdown_on_exit_codes` and the timeouts on every process.
- **Runtime**: the runner shuts the launch down when a member exits with a
  matching code (`member_actor/exit_policy.rs`), and the member does not
  respawn from that exit, as launch skips a respawn once shutting down.

Tests: `tests/tests/shutdown_semantics.rs` on
`tests/fixtures/shutdown/exit_handler.launch.py` (a respawning process
exiting with the `code` argument, a handler shutting down on 2): `ros2
launch`, both parsers and `up` start it once and end; with `code:=1` it
respawns under `ros2 launch` and both parsers. Parser: `shutdown_tests.rs`
(a code-2 handler, a handler registered before its target that always shuts
down, an `on_exit=` function on the process, a stateful and a raising one
refused). Negative control: with the source change reverted, the three
"ends on the handler's code" play_launch tests time out at 30 s with the
process respawning.
