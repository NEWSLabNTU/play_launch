# Phase 65 — mixed isolation granularity, per composable

Status: **proposed**. This is phase 61's W3 ("per-node isolation granularity
... the single most valuable follow-up"), given a concrete design after a
second vehicle hit the cost.

## The report

An AutoSDV Autoware stack (51 nodes, 17 containers, 93 composables) on a
12-core AGX Orin. Under the default `--container-mode isolated`, launch plus
early runtime pinned the machine hard enough that the operator killed
play_launch to get the desktop back: load averages in the hundreds, every
core saturated. `ros2 launch` runs the same stack tolerably, because it loads
composables as threads. play_launch forks a process per composable to buy the
segfault boundary, per-node OOM accounting, cgroups and restart — the right
trade for the handful of nodes that crash, paid today by all 93.

## What the code already has

The two loading paths both exist, complete:

- `ObservableComponentManager` — loads the plugin **in-process as a thread**
  on the shared executor, with the component-event reporting the supervisor
  consumes. Zero-copy intra-process IPC intact.
- `CloneIsolatedComponentManager` — **fork+execs `component_node`** per
  composable: own process, own rclcpp context, own executor, own DDS
  participant. (The container-isolation design study argued for
  `clone(CLONE_VM)`; what shipped is fork+exec, and each child is therefore a
  full DDS participant.)

The selection is all-or-nothing per **run** (`--container-mode`). The cost is
not the boundary itself; it is paying it 93 times:

- ~110 DDS participants (54 nodes + 17 containers + 93 children minus
  threads) all discovering each other at startup — the same congestion that
  produces phase 64's spurious LoadNode timeouts.
- Phase 61 measured the mode gap on this stack: 10.2 vs 3.9 cores during
  startup, 4.0 vs 2.8 cores steady, 3.5 vs 1.4 GiB.

## The design

One container, one manager, **per-composable policy**: a composable whose
name matches the isolation list is fork+exec'd exactly as today; every other
composable is loaded as a thread exactly as `observable` does. Both code
paths already exist in the same package and share the
`ObservableComponentManager` base — the change is a dispatch decision in
`on_load_node`, not new machinery.

### Policy surface

```yaml
# play_launch config
container:
  isolation:
    default: thread          # thread | process
    isolate:                 # forced to process (glob on full node name)
      - /perception/object_recognition/detection/centerpoint/*
      - /sensing/lidar/*
    inline:                  # forced to thread, overrides isolate
      - /system/topic_state_monitor_*
```

- `--container-mode isolated` becomes sugar for `default: process`;
  `observable` for `default: thread` with no lists. Existing invocations keep
  their exact behavior.
- The supervisor resolves the policy per composable at spawn time and passes
  it to its own container; the container needs no globbing of its own.
- `stock` containers are untouched: no policy applies to a container that is
  not ours.

### What each composable keeps or gives up

| | thread (default) | process (isolate list) |
|---|---|---|
| segfault containment | no — takes container siblings | yes |
| per-node oom_score_adj, cgroup, kill, restart | container-level | yes |
| zero-copy intra-process IPC | yes | no |
| own DDS participant | no (shares container's) | yes |

Which is the correct default is a per-node judgment the integrator owns; the
point of this phase is that the judgment becomes expressible. The natural
isolate list is short: nodes with native/CUDA/TensorRT code that has crashed
before, nodes whose OOM kill must not take neighbors. On the AutoSDV stack
that is realistically 6-10 of the 93 — the ~30 `topic_state_monitor`-class
composables have no business each owning a process.

### Expected numbers

With ~8 isolated of 93 (extrapolating phase 61's mode gap linearly in child
count): startup CPU near the observable floor (~4.4 of 12 cores rather than
10.2), steady ~2.9 rather than 4.0 cores, DDS participants ~34 rather than
~110 — while the nodes that motivated isolation keep every guarantee they
have today. To be measured, not assumed; the phase-61 harness
(`scripts/edge-storm/`) applies unchanged.

## Work items

- W1: policy type + config parsing + `--container-mode` mapping; supervisor
  passes per-composable decisions. Mixed dispatch in the container
  (`on_load_node`: thread path vs fork+exec path by flag).
- W2: state/reporting: a threaded composable under a mixed container reports
  through component events (already true in observable); an isolated one
  through the child machinery (already true in isolated). The web UI should
  say which boundary each composable has.
- W3: measure on the AutoSDV stack with the edge-storm harness; record the
  numbers in this document.
- W4: revisit the advisory ("isolated starts N extra processes...") to
  recommend the policy list instead of a mode flip.

## Relation to phase 64

Independent and compounding: 64 takes the load-status conversation off rmw
for our containers; 65 shrinks the number of participants jamming rmw in the
first place. Either alone helps; both together should make a 150-member
launch on a 12-core box boring.
