# Fault tolerance in a contract

`docs/guide/safety-function.md` builds one safety function that stops the
robot. That is enough when stopping is safe and the rest of the system can
be ignored. It is not enough when most of the system is safety critical:
a vehicle that must keep driving to a safe place, or a sensor whose loss
should be survived rather than reacted to. This guide covers the four
things a contract states for that case, what the checker derives from
each, and what the running system is checked for. Every example is a test
fixture under `tests/fixtures/`.

| You state | The checker derives |
|---|---|
| a redundant group and how many members it needs (`functions.<f>.m`) | when the group, and the mode that requires it, is lost |
| nothing about restarts (the launch file's `respawn` says it) | whether a restart restores the output inside the FTTI, from a run |
| which nodes must share nothing (`independence:`) | whether the deployment keeps them apart |
| which communication faults matter (`on:`) | which per-publisher checks run |

## 1. A redundant pair

Two LiDARs, either one enough. The hazard is losing both
(`contract_independence`):

```yaml
functions:
  scan_pair:
    all_of: [/scan_a, /scan_b]
    m: 1

hazards:
  drive_blind:
    integrity: ASIL_D
    monitors: [scan_pair]
    on: omission
    ftti: 500ms
    reaction: safety.stop
```

`m` is how many members must be live for the group to hold: `m: 1` of two
is 1oo2, and `m: 2` of three is 2oo3. Without `m`, an `all_of` group holds
while any member is live. At run time the observer counts live members, so
the mode that requires `scan_pair` is lost only when fewer than `m` publish,
and losing one LiDAR is not a hazard.

This is also how a level that cannot degrade in time is rescued: when the
reaction to losing a sensor cannot fit the FTTI, a second sensor in the
same group means the reaction is never needed for a single loss.

## 2. Redundancy that shares nothing

Two LiDARs in one container are one process under `observable` and
`stock`, so one crash takes both. `independence` states what the members
must not share:

```yaml
independence:
  - of: [lidar_a, lidar_b]
    not_sharing: [process, container, cpus]
```

`of` lists nodes or functions (a function stands for the nodes that publish
its members). Deployed in one container, the checker refuses it:

```text
error[independence-violated]: '/lidar_a' and '/lidar_b' must not share a
process, and share one: /lidar_a and /lidar_b (/sensing_container)
(composables in one container share a process under `observable` and
`stock`, the mode this check assumed; under `isolated` they do not -- pass
`--container-mode isolated` if that is how the system runs)
error[independence-violated]: '/lidar_a' and '/lidar_b' must not share a
container, and share one: /lidar_a and /lidar_b (/sensing_container)
```

The model does not record the container mode, so `check` assumes the
shared case and says so; `check --container-mode isolated` states the
other, and `play_launch launch` uses the mode it runs. `cpus` is checked
against the scheduling platform file's CPU sets; without one it is
reported unchecked (`info[independence-unchecked]`), never passed.

With no key at all, the checker also reports a container that mixes
integrity levels, since a fault in the lower node can take the higher one
with it:

```text
warning[interference-shared-process]: container /sensing_container runs
nodes of different derived integrity in one process: /lidar_a (ASIL_D),
/lidar_b (ASIL_D), /logger (none)
```

`contract_independence_ok` deploys the same contract apart and is clean.
There is no `host`: every node of one resolved model runs on one host, so
independence across hosts is the separation of two models.

## 3. A restart, credited from a run

A restart is the launch file's `respawn` and `respawn_delay`; the contract
never states it. The checker credits it when detection, the delay and the
node's startup fit the FTTI, and the startup time comes only from a run
(`contract_recovery`, a LiDAR driver with `respawn_delay="0.1"`):

```text
$ play_launch check bringup.launch.xml
info[recovery-uncredited]: hazard 'drive_blind': /safety/lidar_driver
respawns (after 100ms), but its restart is not credited: a startup time
comes only from a run (`check --against <run-dir>`)

$ play_launch check bringup.launch.xml --against play_log/<run>
info[recovery-credited]: hazard 'drive_blind': a restart of
/safety/lidar_driver restores /safety/lidar_driver's output in detection
100.00ms + respawn_delay 100.00ms + startup 150.00ms = 350.00ms, inside the
FTTI 500.00ms
```

A slower startup in another run gives `warning[recovery-unfit]` (`600.00ms,
past the FTTI 500.00ms -- the hazard degrades before the restart
completes. Shorten the restart, or add a redundant member`). A composable
restarts with its container, so its container's `respawn` is what counts.
At run time `hazard-recovered` says whether the guard came back inside the
FTTI or past it.

## 4. Communication faults

`on:` names the fault classes a hazard is about. Besides `omission`,
`late`, `loss` and `reported`, three classes concern messages that arrive
but are wrong:

| Class | A message whose |
|---|---|
| `repetition` | stamp equals the previous one from the same publisher |
| `resequencing` | stamp is earlier than the previous one from the same publisher |
| `insertion` | publisher the contract does not name on the topic |

The observer checks each per publisher, and only for a hazard whose `on:`
names the class: a stamp re-sent at rate is right for a state topic, so an
omitted `on:` does not turn these on. Statically, a subscriber detects one
of these only with `on_violation: { mechanism: application }`, because no
QoS event reports them.

## 5. When the commanded topic goes quiet

A level that commands a topic (`modes.<m>.commands`) expects it to keep
publishing. When the topic goes silent, what happens depends on the
consumer, often hardware outside the launch. Say what it does:

```yaml
topics:
  /cmd_vel_filtered:
    external: { side: sub, on_silence: { after: 500ms } }
```

With it, silence is the consumer's to handle; without it, silence at an
active level is `final-element-silent` at run time.

## Related

- `docs/guide/safety-function.md` — one safety function, end to end, and
  Simplex as a pattern.
- `docs/guide/runtime-enforcement.md` — every runtime rule.
- `docs/roadmap/phase-101-fault-tolerance.md` — how this was built.
- rlm `docs/design/fault-tolerance.md` — the design: the case studies it
  came from, and the models considered and not adopted.
