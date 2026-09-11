---
id: 24
title: "`play_launch run` inside a systemd user scope fails at spawn with a bare EPERM"
status: resolved
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

## Resolution

**The scope was incidental, and the failing step was not in `pre_exec` at
all.** It was the `setpgid(0, pgid)` that std's `Command::process_group()`
performs in the child *before* any `pre_exec` closure runs — and the group it
named had ceased to exist a few microseconds earlier. Reproduced on `main`
(the same `systemd-run --user --scope` wrapper, all default features on) on
the first try, then 0 of 30 further tries on an idle 32-core host: it is a
race, and the reporter's machine — running an 89-process Autoware stack in
the neighbouring scope — lost it every time. `strace -f` with a 20 ms delay
injected on `setpgid` (`-e inject=setpgid:delay_enter=20000`, only widening a
window the undelayed trace shows is two syscalls wide) makes it lose
deterministically:

```
1569561 setpgid(0, 1569522 <unfinished ...>
1569516 kill(1569522, SIGKILL)          = 0
1569516 wait4(1569522, [{WIFEXITED(s) && WEXITSTATUS(s) == 0}], 0, NULL) = 1569522
1569561 <... setpgid resumed>)          = -1 EPERM (Operation not permitted)
```

`1569561` is the forked child about to exec `talker`; `1569522` is the
anchor — the `true` that play_launch spawns into a fresh process group and
holds as a zombie so that `pgid` stays valid for every later spawn; `1569516`
is the play_launch thread running the anchor task's shutdown. Linux returns
EPERM, not ESRCH, for a process group that has no member in the caller's
session, which is why the errno pointed everywhere but here.

The chain that got the anchor reaped while a spawn was still in flight:

1. `commands/run.rs` wired the web server to a throwaway
   `tokio::sync::watch::channel` whose sender (`_shutdown_tx`) was dropped at
   the end of the block that created it. `web::run_server` reads a closed
   channel as "shut down", so every `run` with the web UI enabled logged
   `Web server shutting down...` about a millisecond after `Web UI available
   at`. (When port 8080 was already held — by the reporter's `launch` in the
   other scope — the bind failed even faster, with the same effect.)
2. `run`'s main loop treats any finished background task as the end of the
   run: `break`, then `shutdown_tx.send(true)`.
3. `process/pgid.rs::run_anchor_task` answered shutdown with
   `anchor.kill(); anchor.wait()` — reaping the zombie, and with it the
   process group.
4. The regular-node actor, already past its own shutdown check, called
   `Command::spawn()`; the child's `setpgid(0, pgid)` found no such group.

Why `launch` was never affected: `commands/up.rs` hands the web server
`shutdown_signal.clone()` — the run's real shutdown channel — so its web task
lives as long as the run and step 2 never fires. The `--disable-all` that
every existing `run` test passes has the same effect, which is why the
integration suite stayed green.

Two changes fix it, and a third makes the next one diagnosable:

- **The anchor now outlives every spawn.** `run_anchor_task` no longer
  reaps the zombie at shutdown; the group id stays valid until play_launch
  itself exits (`handle_shutdown_simple`'s `waitpid(-pgid)` sweep reaps it
  with everything else, and init does otherwise). A zombie costs a pid-table
  entry it held for the whole run anyway. Pinned by
  `process::pgid::tests::the_anchor_group_is_still_joinable_after_shutdown`,
  which failed with exactly this issue's `Operation not permitted` before the
  change.
- **`run`'s web server is wired to the run's shutdown channel**, as `up`'s
  is — so `run` also has a working web UI and monitoring past its first
  millisecond, which it never had.
- **Every child-side step names itself.** `setpgid` moved out of std's
  `process_group()` into the same `pre_exec` hook as `PR_SET_PDEATHSIG`, the
  cgroup join and the OOM bias (`execution/node_cmdline.rs::PreExec`), plus
  the container's control-fd `fcntl` in `process_lifecycle.rs`. A failure
  writes one line to the child's stderr — already the node's `err` file by
  then — in a single `writev(2)` of slices rendered *before* the fork (the
  pgid, the cgroup path, the OOM value; nothing allocates after it), e.g.
  `play_launch: pre_exec: setpgid(0, 1569522) failed: EPERM (errno 1)`. The
  parent reads that line back after `spawn()` fails
  (`spawn_failure_detail`), so the actor logs
  `Unable to start: Operation not permitted (os error 1) — pre_exec:
  setpgid(0, …) failed: EPERM (errno 1) — the process group play_launch
  spawns into no longer exists in this session (…)`, and a bundle read a
  week later has the same thread to pull. The two best-effort steps (cgroup
  join, `oom_score_adj`) still never fail the spawn, but they now report
  rather than swallow. Pinned by
  `a_pre_exec_failure_names_the_step_in_the_child_stderr`, which provokes
  the identical syscall and errno deterministically by asking to join init's
  process group.

Integration: `tests/tests/run_in_scope.rs` runs `play_launch run` with the
default features ON — once in a plain shell, once under
`systemd-run --user --scope` (`SKIP:` when the host has no user manager) —
and asserts the node publishes while the web server is still up. Both failed
on the pre-fix binary at the early `Web server shutting down...`.
