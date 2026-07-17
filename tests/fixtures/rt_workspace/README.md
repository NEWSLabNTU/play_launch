# rt_workspace

A small, real, buildable ROS 2 workspace that is both:

1. **The runnable example** behind [`docs/guide/rt-scheduling.md`](../../../docs/guide/rt-scheduling.md)
   — clone, build, launch, observe `SCHED_FIFO` on every thread. Demonstrates
   the Phase 41 v2 model: a `mapper` **derives** priorities from the
   contract's declared rates; a platform file supplies platform facts and
   explicit per-node overrides — you no longer hand-write every priority.
   Phase 44.5 upgrades the contract to vocabulary v2 (explicit `trigger:` on
   every path) and composes the three nodes into one named chain
   (`points_to_cmd`), derived by the `chain_aware` mapper.
2. **An integration fixture** exercising the RT apply-layer (Phase 38)
   against our own nodes, with a committed manifest under test.

## Layout

```
rt_workspace/
├── justfile                                  # build / dump / check / run
├── src/rt_demo/                              # ament_cmake package: rclcpp + std_msgs + rclcpp_components
│   ├── sensor_node                           # 100 Hz publisher on points_raw (chain boundary)
│   ├── control_node                          # sub points_filtered -> pub cmd (chain sink, overridden-priority node)
│   └── filter_component                      # composable: sub points_raw -> pub points_filtered (chain segment)
├── launch/
│   ├── bringup.launch.xml                    # sensor_node + control_node + 1 container w/ filter_component
│   ├── bringup.contract.yaml                 # provider sidecar: vocab v2 triggers + `points_to_cmd` chain (44.5)
│   ├── bringup.system.posix.yaml             # provider platform file (v2): mapper=chain_aware + 1 override
│   └── bringup.system.zephyr.yaml            # Zephyr stub: proves per-target coexistence (nano-ros's file, not ours)
├── contracts/rt_demo/launch/
│   ├── bringup.contract.yaml                 # user overlay: tightens control_node's max_age_ms
│   └── bringup.system.posix.yaml             # user overlay: tweaks the provider's band + control_node's pin
└── system.toml                                # legacy bridge: hand-written tiers (kept as the living bridge test)
```

`/perception` namespace: `sensor_node` + `perception_container` (loading
`filter_component`). `/control` namespace: `control_node` — pinned by an
explicit `overrides:` entry (mirrors the legacy `system.toml`'s hand-written
FIFO 20 / core 0 outcome, reached a different way).

## The `points_to_cmd` chain (Phase 44.5)

`bringup.contract.yaml` declares one end-to-end chain across all three
nodes — the smallest honest decomposition vocabulary v2 supports:

```yaml
chains:
  points_to_cmd:
    semantics: reaction
    max_latency_ms: 30
    segments:
      - { scope: /, path: tick }               # sensor_node: trigger: {timer: {rate_hz: 100}}
      - { via: /perception/points_raw }
      - { scope: /, path: filter }              # filter_component: trigger: {input: [points_raw]}
      - { via: /perception/points_filtered }
      - { scope: /, path: control }              # control_node: trigger: {input: [points_filtered]}
```

`sensor_node`'s `tick` path is timer-triggered — the chain-aware mapper
(`docs/superpowers/specs/2026-07-17-chain-aware-mapper-design.md`) classifies
it a **clock boundary** (the chain's own source clock, not a crossing to
reduce by scheduling). `filter_component`'s `filter` and `control_node`'s
`control` are both input-triggered and adjacent, so they merge into one
**event segment**, ranked drain-toward-sink (`control_node` > `filter_component`
— downstream ranks above upstream). The boundary is placed below the whole
segment (its obligation is local: meet its own 10ms period).

Feasibility (computed honestly, by hand): `sampling_cost` = the `tick`
boundary's period alone (1000/100 Hz = 10ms; no declared `max_latency_ms` on
that path, so no extra exec cost — "no invented WCETs"). Declared
event-segment latency = `filter`'s 5ms + `control`'s 10ms = 15ms. Total
25ms fits comfortably inside the chain's 30ms budget (5ms honest slack) —
passes `chain-budget` cleanly, and 10ms sampling_cost is well under the
30ms budget, so `chain-sampling-feasibility` doesn't fire either.

### Declare → check → explain workflow

```bash
# 1. Declare: the chain lives in bringup.contract.yaml's `chains:` section
#    (see above) — no separate command, just YAML.

# 2. Check: `chain-link` (every segment/via resolves), `chain-budget`, and
#    `chain-sampling-feasibility` all run as part of the normal manifest
#    check — no chain-specific flag needed.
play_launch check --sched launch/bringup.system.posix.yaml rt_demo bringup.launch.xml

# 3. Explain: --explain shows the per-node chain provenance the mapper
#    derived (`derived(chain_aware: points_to_cmd segment drain 2/2) ->
#    prio 39` for filter_component, `derived(chain_aware: points_to_cmd
#    boundary RM period=10ms) -> prio 38` for sensor_node; control_node
#    shows `override(control_node)` instead — the platform file's explicit
#    pin always beats the chain-derived rank).
play_launch check --sched launch/bringup.system.posix.yaml --explain rt_demo bringup.launch.xml
```

**Known gap** (tracked, not fixed by this fixture): `--explain`'s tier table
also logs a `chain member ... does not match any schedulable node` warning
for all three chain members. This is a pre-existing FQN-qualification
mismatch (`.superpowers/sdd/p44-w4-report.md`'s carry-forward note): chain
segment node identity is resolved from the *contract* side (scope
namespace + bare node name, e.g. `/sensor_node`), while the actual
schedulable node comes from the *launch dump* side (the node's own
`namespace=` attribute, e.g. `/perception/sensor_node`) — this fixture's
nodes use exactly the bare-`namespace=`-outside-any-`<group>` shape that
triggers it. `check` still exits 0 (it's a warning); the derived
priorities for `filter_component`/`sensor_node` are correct as *numbers*
(and are asserted exactly in `tests/tests/rt_workspace.rs`), but applying
them via a real `launch --sched-apply` run would not currently reach the
real `/perception/*` processes — only `control_node`'s override does
(a different, unaffected resolution path). See
`.superpowers/sdd/p44-w5-report.md` for the full write-up.

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
   shows all provenance kinds: `derived(chain_aware: points_to_cmd ...)` for
   `sensor_node` (boundary) and `filter_component` (segment drain),
   `override(control_node)` for `control_node`, and `default (no timing
   facts)` for the fact-less container. Also prints the chain provenance
   case documented below ("The `points_to_cmd` chain").
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
