# Real-Time Scheduling Guide

`play_launch` can apply Linux real-time scheduling — `SCHED_FIFO`/`SCHED_RR`
policy, RT priority, and CPU affinity — to every process it launches, driven
by a portable scheduling spec. It works **without root**: the privileged
syscalls are delegated to a small capability-holding helper.

Since Phase 41, the recommended model is **derive + override**: a named
mapper derives priorities from your launch file + contract's declared rates
and deadlines; a small platform file supplies platform facts (the priority
band, isolated CPUs) and explicit per-node pins. You stop hand-writing every
priority number — you write the mapper and the exceptions.

```
you write                          play_launch resolves          the kernel sees
─────────                          ────────────────────          ───────────────
launch file + contract   ──┐
  (rates, deadlines)        ├──►  mapper derives  ──┐
platform file: resources ──┘                        ├──► SchedPlan ──► per-THREAD
platform file: overrides ───────────beats derived────┘   FQN → tier      SCHED_FIFO/RR
                                                                        + priority + affinity
```

This guide explains the configuration structure and, precisely, how each
piece of configuration maps onto the processes and threads the kernel
actually schedules.

## Quick start

A runnable copy of this example lives at `tests/fixtures/rt_workspace/` — a
real colcon workspace (`rt_demo`: a 100 Hz sensor publisher, a pinned control
node, and one composable filter) with the launch file, contract, and
platform files committed:

```bash
just build && just setcap        # one-time: grant the helper its capability
cd tests/fixtures/rt_workspace
just build                       # colcon build of the rt_demo package
just check                       # validate contracts + scheduling, several ways
just run-derived                 # launch — NO --sched flag: channel resolution finds it
```

`just run-derived` runs `play_launch launch rt_demo bringup.launch.xml
--sched-apply warn` with no `--sched` at all. Channel resolution (below)
discovers `launch/bringup.system.posix.yaml`, the provider platform file
shipped next to the launch file, and applies the plan it derives.

While it runs, prove every thread of the control node got the policy:

```bash
ps -eLo tid,cls,rtprio,psr,comm | grep control_node
```

(`just verify-sched-rt` at the repo root demonstrates the same check
end-to-end against a generated spec.)

The provider platform file (`launch/bringup.system.posix.yaml`) that drove
this — a mapper plus platform facts plus one explicit pin, no hand-assigned
priority table:

```yaml
target: posix
mapper: rate_monotonic

resources:
  rt_priority_band: { min: 10, max: 40 }
  isolated_cpus: [0]

overrides:
  control_node: { priority: 20, core: 0 }
```

`sensor_node` and `filter_component` (both declared at 100 Hz in the
contract) get their priorities **derived** by `rate_monotonic`; `control_node`
is **overridden** to an explicit pin (overrides always beat derived values).
A node with no rate/deadline facts and no override — `perception_container`,
the empty container shell — lands in the non-RT default tier automatically.

No `sudo` anywhere. Nothing runs as root.

---

## 1. Configuration structure

Three files, three roles, one file each:

| file | portable? | contains |
|---|---|---|
| **Launch file** (XML) | — | the graph SSoT: nodes, containers, composables, namespaces |
| **Contract** (`<stem>.contract.yaml`) | yes | platform-agnostic constraints: rates, deadlines, `criticality` |
| **Platform file** (`<stem>.system.<target>.yaml`) | per-target | platform facts (priority band, isolated CPUs) + explicit overrides, for ONE target |

The scatter across three files/formats is deliberate (ROS compatibility for
the launch file; separation of platform concerns for the other two) — see
[design §1](../superpowers/specs/2026-07-16-rt-config-v2-design.md#1-principles-revised).
`check --explain` (§1.3 below) answers the "information is scattered"
complaint by presenting the merged view without merging the storage.

### 1.1 The mapper — deriving instead of hand-writing

The platform file names a **mapper** — a named algorithm that turns your
launch+contract's declared rates/deadlines/criticality into priorities,
within the platform facts you give it:

```yaml
target: posix
mapper: rate_monotonic     # or: deadline_monotonic, manual
resources:
  rt_priority_band: { min: 10, max: 40 }   # the mapper's working range
  isolated_cpus: [0]
```

Built-in mappers:

| mapper | derives from | ordering |
|---|---|---|
| `rate_monotonic` | the node's fastest declared rate (`pub`/`sub` `min_rate_hz`, or the topic's own `rate_hz`) | higher rate → higher priority |
| `deadline_monotonic` | the node's tightest declared path `max_latency_ms` | tighter deadline → higher priority |
| `chain_aware` (Phase 44) | declared `chains:` (§1.7) plus the same rate/deadline facts as a fallback for everything not on a chain | chain members ranked by criticality + drain-toward-sink; everything else falls back to criticality-bucketed RM/DM |
| `manual` | nothing — requires the legacy `system.toml` bridge (§4) | hand-written tiers |

`rate_monotonic`/`deadline_monotonic` rank each node from **its own**
declared facts in isolation — a good fit when nodes don't need end-to-end
budget reasoning across a pipeline. Prefer `chain_aware` once you have a
multi-node causal pipeline with a budget that matters end-to-end (a
sensor→filter→control path, a perception→planning→control corridor): it
ranks chain members by their position in the pipeline (drain-toward-sink)
instead of their local rate/deadline alone, and gives you `chain-link` /
`chain-budget` / `chain-sampling-feasibility` checking for free. With zero
`chains:` declared, `chain_aware` degrades to exactly the criticality-RM/DM
fallback (§1.7) — it's a safe default even before you've authored a chain.

All derived mappers spread ranked nodes linearly across
`resources.rt_priority_band`; a node with no matching fact (no declared rate
for `rate_monotonic`, no declared path for `deadline_monotonic`, not on a
chain and fact-less for `chain_aware`) falls into the **default tier** —
`SCHED_OTHER`, unscheduled, reported as such by `--explain`.

Add a per-node `criticality: high | medium | low` hint to the **contract**
(not the platform file) if you want to record which nodes matter most — it's
a mapper hint with no numbers attached (future mappers may weight by it; the
current built-ins don't).

### 1.2 The platform file — facts + overrides

```yaml
target: posix                     # required; must match --target
mapper: rate_monotonic

resources:                        # platform facts the mapper works within
  rt_priority_band: { min: 10, max: 40 }   # POSIX RT range: 1..=99
  isolated_cpus: [0]

overrides:                        # explicit pins — always beat derived values
  control_node: { priority: 20, core: 0, sched_class: SCHED_FIFO }   # sched_class optional; FIFO if omitted
```

- **One file per target.** `posix` is the only target `play_launch` itself
  applies; RTOS targets (`zephyr`, `freertos`, ...) are separate files
  consumed by nano-ros through the identical shipping channels (§1.4) — no
  shared file, no section-level merging, so a Linux integrator and a board
  integrator never fight over one file.
- **Overrides selector**: full FQN or bare node name, same vocabulary as the
  legacy `[[assign]].nodes` selector. An override naming a node with no
  timing facts **promotes** it into RT scheduling (a pin is an explicit
  statement — `SCHED_FIFO` unless the override itself says otherwise). An
  override matching nothing warns, never errors.
- **Band clamp / contradiction checks** (design §6): a derived-or-overridden
  priority outside `rt_priority_band` warns and clamps under
  `--sched-apply warn`, errors under `strict`. A final priority order that
  contradicts the contract's own rate/deadline order (e.g. a slower node
  pinned above a faster one) is always a warning, citing both the contract
  and the platform file.

### 1.3 `--explain` — the merged view

```bash
play_launch check --sched launch/bringup.system.posix.yaml --explain \
    rt_demo bringup.launch.xml
```

```
FQN                               CLASS        PRIO  CORE  PROVENANCE
/perception/filter_component      SCHED_FIFO     25     -  derived(rate_monotonic: 100 Hz → prio 25)
/control/control_node             SCHED_FIFO     20     0  override(control_node)
/perception/sensor_node           SCHED_FIFO     10     -  derived(rate_monotonic: 100 Hz → prio 10)
/perception/perception_container  SCHED_OTHER     0     -  default (no timing facts)
system file: explicit(launch/bringup.system.posix.yaml)
contract[scope 0, rt_demo/bringup.launch.xml]: provider(.../launch/bringup.contract.yaml)
```

Every row's `PROVENANCE` column is one of:

| provenance | meaning |
|---|---|
| `override(<key>)` | an explicit `overrides:` entry pinned this node |
| `derived(<mapper>: <fact> → prio N)` | the mapper computed this from a declared rate/deadline |
| `derived(chain_aware: <chain> segment drain N/M) -> prio N` | `chain_aware` (§1.7) ranked this node as position N of M in an event segment, drain-toward-sink |
| `derived(chain_aware: <chain> boundary RM period=Xms) -> prio N` | `chain_aware` ranked this node as a clock boundary, by its own period (RM among boundaries) |
| `default (no timing facts)` | no override, no fact — `SCHED_OTHER`, unscheduled |

The footer lines show exactly which channel (§1.4) supplied the platform
file and each scope's contract — the answer to "where did this number come
from" in one command. Without a resolved platform file, `--explain` prints a
no-op note and exits 0 (it's a decoration on an already-optional feature,
not something that should fail `check`).

`--explain` is not `check`-only. `play_launch resolve` embeds the resolved
scheduling structure into the SystemModel (`execution.sched`), so
`resolve --explain` and `replay --model <model.yaml> --explain` render the
same table **from the model** — no re-derivation — byte-identical to
`check --explain` on the same inputs. The scheduling structure travels with
the artifact: an off-host consumer (nano-ros) reads the shared chains +
per-path requirement facts from `execution.sched` and runs its own RTOS
mapper over them (the PiCAS ranks are the Linux realization, ignored
off-host). See `docs/design/system-model-sched-ssot.md`.

### 1.4 Shipping channels — auto-apply, no flags needed

Both the contract and the platform file resolve through the same three
channels, first-hit-wins, **no merging across channels**:

1. **Explicit path** (`--sched <path>` for the platform file, `--contracts
   <dir>` for the contract overlay root) — always wins.
2. **User overlay**: `<overlay-root>/<pkg>/launch/<stem>.system.<target>.yaml`
   (and `<stem>.contract.yaml`). Overlay-root discovery, first existing of:
   `--contracts <dir>` → `$PLAY_LAUNCH_CONTRACTS` → `$XDG_CONFIG_HOME/play_launch/contracts`
   (default `~/.config/play_launch/contracts`) → `/etc/play_launch/contracts`.
3. **Provider sidecar**: installed next to the launch file
   (`share/<pkg>/launch/...`, read-only) — the package's shipped default.

**Auto-apply at launch**: when `--sched` is absent, `play_launch
launch`/`replay` resolve the platform file through these channels and
**apply it** — the provider sidecar is the vendor's trusted default, so RT
works out of the box on an installed system with zero flags. The overlay
exists to fill gaps (no vendor config, or none for your target) or tweak
numbers that don't fit. `--sched-apply off` disables all applying;
`--sched <path>` bypasses discovery entirely; `check` always validates,
never applies, regardless of channel. `--no-provider-contracts` disables the
provider channel for BOTH contracts and platform files.

`--target <t>` selects which target's file to resolve (default `posix`) —
overlay only ever shadows the provider for the *same* target, so a
board-specific Zephyr overlay can never shadow a Linux provider file.

### 1.5 Adjusting the vendor's numbers — `contract eject`

Editing should never mean patching `/opt`. `contract eject` copies the
resolved provider contract *and* platform file into the overlay tree, ready
to edit:

```bash
play_launch contract eject rt_demo bringup.launch.xml --into /tmp/my-overlay
# Ejected contract: .../rt_demo/launch/bringup.contract.yaml -> /tmp/my-overlay/rt_demo/launch/bringup.contract.yaml
# Ejected platform file: .../rt_demo/launch/bringup.system.posix.yaml -> /tmp/my-overlay/rt_demo/launch/bringup.system.posix.yaml
```

Without `--into`, it writes to the discovered overlay root (same discovery
as above) and errors if none exists. `--force` overwrites an existing
ejected copy; without it, `eject` refuses to touch a destination that
already exists. Once ejected, `play_launch check --contracts <root>` (or
just running normally, if `<root>` is on the discovery chain) picks up your
copy over the vendor's.

### 1.6 CLI summary

```bash
play_launch launch <pkg> <file>                                 # auto-apply via channels
play_launch launch <pkg> <file> --sched <path> [--sched-apply off|warn|strict]
play_launch check <pkg> <file> [--sched <path>] [--target <t>] [--explain]
play_launch contract eject <pkg> <file> [--target <t>] [--into <dir>] [--force]
```

| `--sched-apply` | behavior |
|---|---|
| `off` | resolve + report the tier table; **no syscalls** |
| `warn` *(default)* | apply; on failure log a warning, node keeps default scheduling |
| `strict` | any privilege or apply failure **aborts the run** — all-or-nothing RT |

### 1.7 Chains — composing paths into end-to-end budgets

A single node's declared rate or deadline (§1.1) only tells the mapper
about that node in isolation. A **chain** (Phase 44) composes several
nodes' `paths:` — possibly from different scopes — into one named,
end-to-end pipeline with its own budget, connected by explicit `via:`
topics. The `chain_aware` mapper (§1.1) is the consumer: it ranks a
chain's member nodes by their position in the pipeline instead of their
local facts alone.

**The clock-segmented model.** A real pipeline is rarely one unbroken
chain of message-driven callbacks — somewhere along it, a node reads its
input from a `timer`-triggered callback instead of reacting to it
directly (a periodic control loop polling the latest state, an EKF
draining a queue on its own clock). That node's timer is a **clock
boundary**: crossing it costs up to one timer period, and no priority
assignment can shrink that cost — only the boundary meeting its own
period matters. Between boundaries, a chain decomposes into **event
segments** — maximal runs of `trigger: input` paths — where scheduling
*does* control latency: the `chain_aware` mapper ranks a segment
drain-toward-sink (downstream nodes outrank upstream ones, so in-flight
data flushes to the segment's sink instead of being preempted by fresh
arrivals at the head), exactly as PiCAS (Choi et al., RTAS 2021)
prescribes for pure event chains.

So a chain declaration decomposes into an alternating `Segment` /
`Boundary` / `Segment` / ... sequence purely from each referenced path's
own `trigger:` (no node-kind guessing) — feasibility and priority both
fall out of that same decomposition. Full model + algorithm: [chain-aware
mapper design](../superpowers/specs/2026-07-17-chain-aware-mapper-design.md).

**Authoring workflow:**

1. **Declare paths with explicit triggers.** Every path that will
   participate in a chain needs an explicit `trigger:` — `timer`,
   `input`, `once`, or `spontaneous` (§[Path triggers](../../src/ros-launch-manifest/docs/launch-manifest.md#path-triggers-trigger)
   in the manifest format reference):

   ```yaml
   nodes:
     sensor_node:
       paths:
         tick:
           trigger: { timer: { rate_hz: 100 } }   # clock boundary
           output: [points_raw]
     filter_component:
       paths:
         filter:
           trigger: { input: [points_raw] }        # event segment
           output: [points_filtered]
           max_latency_ms: 5
   ```

2. **Declare the chain, connected by `via:` links.** `chains:` is a
   top-level, integrator-owned section (root contract or overlay) —
   segments alternate between `{ scope, path }` hops and `{ via: <topic> }`
   connectors; two path segments may never sit adjacent without an
   explicit `via:` between them (§[Cross-scope chains](../../src/ros-launch-manifest/docs/launch-manifest.md#cross-scope-chains-chains)):

   ```yaml
   chains:
     points_to_cmd:
       semantics: reaction
       max_latency_ms: 30
       segments:
         - { scope: /, path: tick }
         - { via: /perception/points_raw }
         - { scope: /, path: filter }
         - { via: /perception/points_filtered }
         - { scope: /, path: control }
   ```

3. **`check`.** The same command validates the chain alongside everything
   else — no chain-specific flag:

   ```bash
   play_launch check --sched launch/bringup.system.posix.yaml rt_demo bringup.launch.xml
   ```

   Three rules cover a chain end to end (full severities in the [manifest
   format reference](../../src/ros-launch-manifest/docs/launch-manifest.md#static-validation)):
   `chain-link` (error — every `via:` topic exists, is output by the
   preceding segment, and is consumed by the following one; a `via:`
   landing on a `timer` boundary is consumed through that node's own
   subscriptions, so boundaries can sit anywhere in the chain, not just
   first), `chain-budget` (warning — declared segment latencies plus
   sampling cost must fit the chain's `max_latency_ms`), and
   `chain-sampling-feasibility` (warning — sampling cost *alone* meeting
   or exceeding the budget means the chain is structurally infeasible;
   no scheduling assignment can fix it, only a period or architecture
   change can).

4. **`--explain`.** Shows the chain's provenance per node — its position
   in the `Segment` → `Boundary` → `Segment` (S·B·S) decomposition, not
   just a bare priority number:

   ```bash
   play_launch check --sched launch/bringup.system.posix.yaml --explain rt_demo bringup.launch.xml
   ```

   ```
   FQN                               CLASS        PRIO  CORE  PROVENANCE
   /perception/filter_component      SCHED_FIFO     39     -  derived(chain_aware: points_to_cmd segment drain 2/2) -> prio 39
   /perception/sensor_node           SCHED_FIFO     38     -  derived(chain_aware: points_to_cmd boundary RM period=10ms) -> prio 38
   /control/control_node             SCHED_FIFO     20     0  override(control_node)
   /perception/perception_container  SCHED_OTHER     0     -  default (no timing facts)
   ```

   `filter_component` and `sensor_node` show `derived(chain_aware: ...)`
   provenance — a segment drain position or a boundary's period,
   directly traceable to the chain declaration above. `control_node` is
   overridden (next step).

5. **Pin overrides — notice the drain-order warning.** An `overrides:`
   entry always beats the chain-derived rank, even for a chain member —
   but pinning a chain's sink *below* its derived rank breaks the
   drain-toward-sink guarantee for the rest of that chain, and
   `check`/`launch` say so out loud:

   ```
   warning: override pins chain member `/control/control_node` to priority 20 (derived 40)
     — below its chain-derived rank; drain-toward-sink ordering within its chain is no
     longer guaranteed
   ```

   This is `rt_workspace`'s own real output — `control_node` is pinned to
   20 by the platform file's `overrides:` (matching the legacy
   `system.toml` outcome, see §4), which sits below its chain-derived 40.
   The warning is informational (the override still wins, deliberately —
   "overrides win" is the same rule as everywhere else in this guide);
   it exists so an override that quietly inverts a chain's ordering isn't
   silently invisible.

**Runnable example**: [`tests/fixtures/rt_workspace/`](../../tests/fixtures/rt_workspace/)'s
`points_to_cmd` chain (three nodes, one boundary, one two-node segment) is
exactly the example above — see its README's ["The `points_to_cmd`
chain"](../../tests/fixtures/rt_workspace/README.md#the-points_to_cmd-chain-phase-445)
section for the full by-hand budget derivation. **At-scale example**: the
`autoware-contract` repository's `rt/vocab-v2` branch declares three real
chains against Autoware's planning-simulator launch tree —
`planning_to_trajectory` (8-hop, boundary-first, 300ms budget),
`control_to_actuation` (2-hop, 60ms budget), and `planning_to_actuation`
(their merge, exercising an *interior* clock boundary — a `via:` landing
on a non-first `timer` path) — validated with 0 `chain-link`/`chain-budget`/
`chain-sampling-feasibility` diagnostics across 64 manifests
(`.superpowers/sdd/p44-w6-report.md`, `p44-w6-fixes-report.md`).

---

## 2. How configuration maps to the RT execution model

### 2.1 Launch entities → OS processes

Everything `play_launch` launches is an OS process, and the process is the
unit a platform-file `overrides` selector (or a legacy `[[assign]]` rule)
targets:

| launch entity | OS process | scheduled? |
|---|---|---|
| `<node>` | one process per node | ✅ |
| `<node_container>` | one container process | ✅ (its own tier) |
| composable node (`isolated` mode, default) | one `component_node` process, fork+exec'd by the container | ✅ (its own tier) |
| composable node (`observable`/`stock` mode) | a *thread group inside* the container — no process of its own | ❌ only the container is schedulable |

### 2.2 Resolution: contract + platform file + launch → plan

At startup, before any process spawns:

1. The launch file is parsed to `record.json` (nodes, containers, composables,
   and the namespace **scope table**).
2. Every entity is flattened to a fully-qualified name — its own namespace,
   falling back to the launch scope's namespace: `/perception/lidar/voxel_grid`.
3. Contract facts (rates, deadlines, criticality) are extracted per node into
   `MapperInput`; the named mapper derives a plan within the platform file's
   `resources`; `overrides` are applied on top (override beats derived,
   always).
4. Validation happens here (priority range, core bounds, band clamp,
   rate/deadline contradictions) — bad specs fail before anything runs, in
   every mode except `warn`'s clamp-and-continue.

Each spawned member is handed its own resolved entry; nothing is re-resolved
at runtime.

### 2.3 Apply timing — and why it's per-thread

Linux scheduling attributes belong to **threads**, not processes. A ROS node
starts as one thread and grows to ~11 (DDS/rmw spawn ~10) within half a second
of exec. `play_launch` therefore applies the tier to **every thread**
(`/proc/<pid>/task/*`), and threads created afterwards inherit the policy from
their creator (glibc `PTHREAD_INHERIT_SCHED`):

| entity | applied when | threads at that moment |
|---|---|---|
| node / container | immediately after spawn (~1 ms) | 1 — later threads inherit |
| composable | on its `LOADED` event (after init) | ~11 — the per-thread sweep covers them all |
| respawned process | re-applied automatically on each respawn | — |

Two safety details on the composable path:
- The `ComponentEvent` carries the child's `pid` **and its start time** (the
  PID's kernel birth tick). The apply is skipped with a warning if the live
  process no longer matches — a recycled PID can never be scheduled by
  mistake.
- A composable whose event carries `pid = 0` (non-isolated modes) is skipped.

> Verification caveat: `chrt -p <pid>` reads **only the main thread**. To see
> what actually happened, check every TID — `just verify-sched-rt` does.

### 2.4 Privilege: the RT helper

An unprivileged process cannot request RT scheduling (`RLIMIT_RTPRIO` is 0 for
normal users). `play_launch` solves this without running anything as root:

```
play_launch  (uncapped, links ROS, runs as you)
    │  ApplySched{pid, tier}   — pipe IPC, once per process at spawn/load
    ▼
play_launch_rt_helper  (ROS-free, holds CAP_SYS_NICE only)
    → sched_setscheduler / sched_setaffinity on every thread of that pid
```

- `just setcap` (or `play_launch setcap`) grants `cap_sys_nice+ep` to the
  helper (and `cap_sys_ptrace+ep` to the I/O-monitoring helper). File
  capabilities do not change the user — the helper runs as *you*, with exactly
  one extra permission. **Capabilities live on the file inode: re-run
  `just setcap` after every rebuild.**
- The main `play_launch` binary must **never** be capability-granted: a file
  capability triggers secure-execution mode (`AT_SECURE`), the loader ignores
  `LD_LIBRARY_PATH`, and the binary can no longer find its ROS libraries.
  `setcap` strips a stray capability from it automatically; `verify` flags one
  as a fault. This is precisely why the helpers exist.
- The helper's lifetime is managed: it dies with the parent (`PDEATHSIG`), is
  killed on drop, shuts down gracefully on normal exit, and every request has
  a 5 s timeout — a wedged helper degrades per `--sched-apply`, never hangs
  the launch.
- Running as **root** still works (`sudo -E`, with `LD_LIBRARY_PATH`
  re-injected): the syscalls are then made directly, no helper needed. This is
  the fallback, not the recommended path.
- IPC cost is control-plane only: one round-trip per process at
  spawn/respawn/load. Steady-state cost is zero — the kernel does the
  scheduling from then on.

### 2.5 What the kernel ends up with

For a node overridden to `{ priority: 20, sched_class: "SCHED_FIFO", core: 0
}` (or derived to an equivalent plan entry):

- every thread: policy `SCHED_FIFO`, `rt_priority 20` — preempts all
  `SCHED_OTHER` work on its CPU; among RT threads, higher priority wins;
  `SCHED_RR` additionally time-slices equal priorities
- every thread: affinity mask `{0}` — the whole process is pinned to CPU 0
- unassigned (default-tier) nodes: untouched — normal `SCHED_OTHER`

Kernel-level notes:
- **RT throttling**: by default Linux caps RT execution at 95% of each second
  (`sched_rt_runtime_us`), so a runaway FIFO loop cannot wedge the machine.
- On kernels with `CONFIG_RT_GROUP_SCHED`, a cgroup with `rt_runtime = 0` can
  still refuse RT even with the capability — you'll see `EPERM` despite a
  correct setup.
- `SCHED_DEADLINE` is not applied on Linux yet.

---

## 3. Verifying

```bash
just verify               # capability state of all three binaries
just verify-sched-rt      # launches a fixture UNPRIVILEGED and checks EVERY thread
```

Expected `verify-sched-rt` output:

```
  talker    pid=12345  threads=11  fifo=11/11  prio=20  cpu=0
  listener  pid=12346  threads=11  fifo=11/11  prio=20  cpu=0
```

`fifo=11/11` is the number that matters: all threads, not just the main one.

Manual, for a live system:

```bash
pid=$(cat play_log/latest/node/<name>/pid)
for t in /proc/$pid/task/*; do chrt -p ${t##*/}; done   # per-thread policy
taskset -cp $pid                                        # affinity
```

## 4. Legacy `system.toml` — deprecated, still supported

The original hand-written schema (tiers with real priority numbers,
`[[assign]]` bindings) keeps working via a **bridge**: it parses to the same
internal plan as the `manual` mapper, target implied `posix`. File extension
selects the parser — `.yaml` is always the new schema, `.toml` is always the
legacy one:

```toml
[tiers.control]
class = "real_time"

[tiers.control.posix]
priority    = 20
sched_class = "SCHED_FIFO"
core        = 0

[[assign]]
tier  = "control"
nodes = ["control_node"]
```

```bash
play_launch check --sched system.toml rt_demo bringup.launch.xml
play_launch launch rt_demo bringup.launch.xml --sched system.toml --sched-apply warn
```

Full schema reference (tiers, placement, binding selectors, validation
rules): [`src/ros-launch-manifest/docs/scheduling.md`](../../src/ros-launch-manifest/docs/scheduling.md#toml-schema).

**This path is deprecated but fully supported** — it is not going away until
`nano-ros` migrates to the v2 schema (Phase 41.6, not yet scheduled). New
configuration should use the v2 platform-file model (§1). `--sched <path>`
with a `.toml` file always works identically to how it always has; nothing
here changes existing behavior.

## 5. Troubleshooting

| symptom | cause | fix |
|---|---|---|
| warn: `no privilege to apply ... cap_sys_nice` | helper not capability-granted | `just setcap` |
| worked yesterday, EPERM today | rebuild replaced the binaries — caps are per-inode | `just setcap` again |
| `error while loading shared libraries: lib...rosidl...` | someone ran `setcap` on the **main** binary (`AT_SECURE` drops `LD_LIBRARY_PATH`) | `just setcap` (it strips it) |
| `mapper 'X' requires resources.rt_priority_band` | platform file's `resources` doesn't declare a band, or `target` isn't `posix` | add `rt_priority_band` to `resources`; non-`posix` targets aren't derivable by play_launch's built-in mappers today (nano-ros's job) |
| ``platform file ... targets `zephyr`, but --target requested `posix` `` | wrong file resolved for the target, or wrong `--target` passed | check `--target`, or that the right per-target file is on disk |
| warning: `priority N outside band [min, max], clamping to ...` | derived or overridden priority exceeds the platform file's band | widen `rt_priority_band`, or fix the override |
| warning citing both a contract and a platform file | a pinned priority order contradicts the contract's declared rate/deadline order | re-check the override, or accept the warning if intentional |
| `--explain` prints a no-op note | no platform file resolved (no `--sched`, nothing on the overlay/provider channels) | pass `--sched <path>`, or ship one via the overlay/provider channels |
| `tier 'X': RT priority N out of range 1..=99` at startup (legacy `.toml`) | bad placement value | fix the spec — this is deliberate fail-fast |
| assign rule error: selector matches nothing (legacy `.toml`) | typo in node name / scope path | check FQNs with `play_launch context record.json --tree` |
| composable skipped: `pid no longer matches its start_time` | the composable died right after loading; its PID may have been recycled | benign — nothing to schedule |
| `chrt -p <pid>` shows `SCHED_OTHER` but the node "should" be RT | `chrt -p` reads only the main thread — or the node is in the default tier (no fact, no override) | check all TIDs; check contract facts / overrides coverage |
| EPERM with caps correct, inside a container/slice | cgroup `rt_runtime = 0` gate (`CONFIG_RT_GROUP_SCHED`) | provision RT bandwidth for the cgroup |
| strict run aborts at startup | that's strict working: no privilege or bad spec = no launch | `just setcap`, or use `warn` |

## 6. Related documents

- Design of record (v2, derived scheduling): [`docs/superpowers/specs/2026-07-16-rt-config-v2-design.md`](../superpowers/specs/2026-07-16-rt-config-v2-design.md)
- Spec schema in depth (v1 + v2): [`src/ros-launch-manifest/docs/scheduling.md`](../../src/ros-launch-manifest/docs/scheduling.md)
- Design of record (apply-layer): [`docs/superpowers/specs/2026-07-06-linux-sched-apply-layer-design.md`](../superpowers/specs/2026-07-06-linux-sched-apply-layer-design.md)
- Design of record (RT helper, per-TID, capabilities): [`docs/superpowers/specs/2026-07-14-rt-helper-design.md`](../superpowers/specs/2026-07-14-rt-helper-design.md)
- Design of record (chains, §1.7): [vocabulary v2](../superpowers/specs/2026-07-17-contract-vocabulary-v2-design.md) · [chain-aware mapper](../superpowers/specs/2026-07-17-chain-aware-mapper-design.md)
- Manifest format reference (triggers, `chains:`, rule severities): [`src/ros-launch-manifest/docs/launch-manifest.md`](../../src/ros-launch-manifest/docs/launch-manifest.md#vocabulary-v2)
- Roadmap and implementation history: [`docs/roadmap/phase-38-linux_rt_scheduling.md`](../roadmap/phase-38-linux_rt_scheduling.md), [`docs/roadmap/phase-41-rt_config_v2.md`](../roadmap/phase-41-rt_config_v2.md), [`docs/roadmap/phase-44-vocab_v2_chain_mapper.md`](../roadmap/phase-44-vocab_v2_chain_mapper.md)
