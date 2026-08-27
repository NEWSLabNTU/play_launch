---
id: 24
title: "`play_launch run` inside a systemd user scope fails at spawn with a bare EPERM"
status: open
type: correctness
severity: medium
---

# 0024 — `run` cannot spawn when play_launch is itself inside a `systemd-run --user --scope`

**Repo:** `play_launch`
**Affects:** the `run` single-node spawn path
(`member_actor/regular_node_actor.rs::spawn`, `execution/node_cmdline.rs`'s
`pre_exec` hook)
**Found on:** 0.9.0, Ubuntu 22.04, cgroup v2, ROS 2 Humble

## Symptom

```
INFO  Running single node: carla_manual_control carla_manual_control
INFO  Registered 1 members (1 nodes, 0 containers, 0 composables)
ERROR [NODE 'carla_manual_control'] Unable to start: Operation not permitted (os error 1)
ERROR Check play_log/2026-08-27_23-07-25/node/carla_manual_control
ERROR Actor node:/carla_manual_control failed: Operation not permitted (os error 1)
```

The node's `err` and `out` files in that directory are **empty** and no process
appears, so the failure is in `Command::spawn()` itself rather than in the
executable. A `pre_exec` closure that returns `Err` surfaces exactly this way:
`spawn()` reports the child's errno and there is no child to log anything.

## Reproducer

The only difference between these two is the `systemd-run --user --scope`
wrapper around play_launch.

```bash
cd <workspace>
source /opt/autoware/1.5.0/setup.bash
source install/setup.bash

# A. plain shell -- works. Runs until the timeout kills it (exit 124).
timeout 40 play_launch run carla_manual_control carla_manual_control; echo "exit: $?"

# B. inside a user scope -- "Unable to start: Operation not permitted (os error 1)"
timeout 40 systemd-run --user --scope --unit=pl-eperm --collect -- \
    play_launch run carla_manual_control carla_manual_control
```

Any package/executable should do; the node here is a GUI tool, but it never
execs, so what it is does not matter.

## What is already ruled out

* **Not the executable.** `ros2 run carla_manual_control carla_manual_control`
  starts it fine, in or out of a scope, and so does A above.
* **Not cgroup `mkdir`.** `docs/issues`-adjacent reasoning in
  `execution/cgroup.rs` notes that a plain login shell cannot `mkdir` in its
  cgroup while a `systemd-run` scope can. Checked directly inside the scope
  used above:

  ```
  cgroup: /user.slice/user-1000005.slice/user@1000005.service/app.slice/pl-eperm.scope
  mkdir:  OK
  ```

  So creating the per-member group is permitted; whatever EPERMs is something
  else in the same hook.
* **Not `run` being unusable generally** — A works.

## The discriminator worth starting from

`launch` does **not** hit this in the same wrapper. On the same machine, at the
same moment, a `systemd-run --user --scope` wrapping `play_launch launch` was
running an Autoware stack with **89 `component_node` processes alive**, while
`run` in an identical scope could not spawn one node. Whatever the `run` path
does differently at spawn is the thing to look at.

`pre_exec` currently does three things (`node_cmdline.rs`): `set_pdeathsig`,
the phase-66 cgroup join, and `bias_oom_score`. The first propagates its error
and the third swallows failure by design, which narrows it, but this issue does
not claim which one it is -- the A/B and the `mkdir` check are what was actually
measured.

## Why it is worth fixing beyond the workaround

The workaround is easy once known: run the node outside the scope, or use
`ros2 run`. The cost is diagnosis. The message is a bare `Operation not
permitted` with no mention of cgroups, scopes or `pre_exec`; the node log
directory is created and left empty, which reads as "the binary was rejected";
and the launcher's own log says nothing. Someone hitting this has no thread to
pull. Naming the failing step, or reporting which `pre_exec` action returned the
errno, would turn a half-hour into a minute.

## Context

Found while wrapping every launch in `systemd-run --user --scope` for orphan
control -- a scope's cgroup can be killed as a unit, which reliably reaps trees
that survive `pkill` and `kill -PGID`. That wrapper is otherwise working well
across dozens of runs, including for `launch`, so "do not use scopes" is not a
satisfying answer.
