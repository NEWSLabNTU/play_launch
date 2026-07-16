# rt_workspace

A small, real, buildable ROS 2 workspace that is both:

1. **The runnable example** behind [`docs/guide/rt-scheduling.md`](../../../docs/guide/rt-scheduling.md)
   — clone, build, launch, observe `SCHED_FIFO` on every thread. Demonstrates
   the Phase 41 v2 model: a `mapper` **derives** priorities from the
   contract's declared rates; a platform file supplies platform facts and
   explicit per-node overrides — you no longer hand-write every priority.
2. **An integration fixture** exercising the RT apply-layer (Phase 38)
   against our own nodes, with a committed manifest under test.

## Layout

```
rt_workspace/
├── justfile                                  # build / dump / check / run
├── src/rt_demo/                              # ament_cmake package: rclcpp + std_msgs + rclcpp_components
│   ├── sensor_node                           # 100 Hz publisher on points_raw
│   ├── control_node                          # sub points_raw -> pub cmd (the overridden-priority node)
│   └── filter_component                      # composable: sub points_raw -> pub points_filtered
├── launch/
│   ├── bringup.launch.xml                    # sensor_node + control_node + 1 container w/ filter_component
│   ├── bringup.contract.yaml                 # provider sidecar (types, rates, max_age_ms, criticality)
│   ├── bringup.system.posix.yaml             # provider platform file (v2): mapper=rate_monotonic + 1 override
│   └── bringup.system.zephyr.yaml            # Zephyr stub: proves per-target coexistence (nano-ros's file, not ours)
├── contracts/rt_demo/launch/
│   ├── bringup.contract.yaml                 # user overlay: tightens control_node's max_age_ms
│   └── bringup.system.posix.yaml             # user overlay: tweaks the provider's band + control_node's pin
└── system.toml                                # legacy bridge: hand-written tiers (kept as the living bridge test)
```

`/perception` namespace: `sensor_node` + `perception_container` (loading
`filter_component`) — both 100 Hz, priorities **derived** by
`bringup.system.posix.yaml`'s `mapper: rate_monotonic`. `/control` namespace:
`control_node` — pinned by an explicit `overrides:` entry (mirrors the
legacy `system.toml`'s hand-written FIFO 20 / core 0 outcome, reached a
different way).

## Build

```bash
just build
```

Builds `rt_demo` into this fixture's own `install/` (a separate colcon
workspace from the repo root — this fixture does not touch the repo's
`install/`, only reads it for the `play_launch` binary). `--symlink-install`
means newly-added sidecar files under `launch/` need a rebuild to appear as
symlinks in `install/.../share/rt_demo/launch/`, but resolution that
canonicalizes through the launch file's existing symlink finds them either
way.

## Run

```bash
just run-derived   # v2: no --sched — channel resolution finds the provider platform file
just run           # legacy bridge: explicit --sched system.toml
```

Both launch in warn mode (no root or `setcap` required — the
`play_launch_rt_helper` handles privileged syscalls if capability-granted;
otherwise you'll see benign warnings and nodes still run, just without RT
scheduling).

## Observe

```bash
pid=$(cat play_log/latest/node/control_node/pid)
for t in /proc/$pid/task/*; do chrt -p ${t##*/}; done   # per-thread policy
taskset -cp $pid                                        # affinity
```

See [`docs/guide/rt-scheduling.md`](../../../docs/guide/rt-scheduling.md) for
the full configuration reference, or `just verify-sched-rt` at the repo root
for an automated version of the check above.

## Validate the contract + sched spec

```bash
just check
```

Five cases, all expected to exit 0:

1. **v2 channel resolution** (no `--sched` at all) — `bringup.system.posix.yaml`
   is discovered as the provider sidecar; prints the
   `Scheduling platform file [provider]: ...` provenance line.
2. **`--explain`** against the explicit provider file — the merged table
   shows all three provenance kinds: `derived(rate_monotonic: ...)` for
   `sensor_node`/`filter_component`, `override(control_node)` for
   `control_node`, and `default (no timing facts)` for the fact-less
   container.
3. **User overlay tweak** (`--contracts contracts`, still no `--sched`) — the
   overlay's `bringup.system.posix.yaml` (wider band, different
   `control_node` pin) wins over the provider sidecar for the same
   `(stem, target)`.
4. **Legacy bridge**: `--sched system.toml` (the `manual` mapper via the
   `.toml` extension dispatch) — kept passing as the living bridge test.
5. **Legacy bridge + contract overlay**: same `system.toml`, but
   `--contracts contracts` layers the user's *contract* (not platform file —
   `system.toml` is passed explicitly and always wins its own channel)
   on top.

```bash
just check-zephyr-stub
```

Demonstrates per-target coexistence: `bringup.system.zephyr.yaml` resolves
through the identical provider-sidecar channel under `--target zephyr` and
**parses** (valid v2 schema, `target: zephyr`), but `check`'s derive step
currently errors — none of play_launch's built-in mappers derive a plan for
non-`posix` facts yet (RTOS derivation is nano-ros's job). This is expected,
documented behavior, not a bug; see `docs/guide/rt-scheduling.md`.

## Eject a provider config to edit locally

```bash
play_launch contract eject rt_demo bringup.launch.xml --into /tmp/my-overlay
```

Copies the resolved provider contract *and* platform file into an overlay
tree, ready to edit without touching `/opt`. See
`docs/guide/rt-scheduling.md`'s eject section.

## Compare parsers

```bash
just dump-rust
just dump-python
just compare-dumps
```
