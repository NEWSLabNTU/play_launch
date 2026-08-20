# Changelog

Notable changes to `play_launch`. Versions follow [semantic versioning];
`0.x` still permits breaking changes in a minor release, and 0.9.0 uses that
allowance heavily.

[semantic versioning]: https://semver.org/

## Unreleased

### Diagnostics

Five fixes, all found by asking why a four-hour vehicle run produced 405,480
diagnostic statuses of which not one was WARNING or STALE.

**A silent publisher now reads STALE.** Level 3 only ever arrives from a
publisher alive enough to declare it, so a crashed node used to leave its last
status in the registry reading OK for the rest of the run.
`diagnostics.stale_after_ms` (default 30 s, `0` disables) ages an entry that
stops updating. The default is deliberately loose because real systems mix
100 Hz sensors with 0.1 Hz housekeeping checks; tighten it per deployment.

**Level transitions are no longer rate limited.** The 100 ms debounce is exactly
Autoware's 10 Hz publish interval, so a diagnostic going OK -> ERROR -> OK
inside one window was dropped from both the registry and `diagnostics.csv`. The
debounce still suppresses a publisher repeating itself at the same level.

**The registry no longer retains history.** Every accepted status was pushed
onto a `Vec` read by nothing — `get_history` had no callers. `diagnostics.csv`
is the durable record and always was.

**The diagnostics view streams instead of polling.** New SSE endpoint
`/api/diagnostics/stream` pushes on every level change and once a second
otherwise; the table could previously sit five seconds behind the system.
`/api/diagnostics/{list,counts}` remain for one-shot callers.

**`/diagnostics_agg` is no longer subscribed by default.** It exists only if the
classic `diagnostic_aggregator` is deliberately launched, and Autoware does not
ship that package at all. `diagnostics.topics` still takes a list.

## 0.9.0 — 2026-08-18

The largest release so far: 863 commits since 0.8.2 (2026-04-09). The theme is
that **`system_model.yaml` is now the one user-facing artifact**, and that the
scheduling layer became something you can measure rather than assert.

Read the *Breaking changes* section before upgrading. Several of them change
behaviour silently rather than erroring.

### Breaking changes

**`replay` is now `up`.** `play_launch up <model.yaml>` spawns from a resolved
SystemModel. The old verb remains as a hidden variant that errors and names the
replacement, and will be deleted at 1.0.0. Migration guide:
`docs/guide/cli-migration-0.9.md`.

**`record.json` is no longer a user-facing artifact.** It survives only as an
in-memory intermediate. Removed with it:

- `dump --format record` and `--format` entirely — `dump` always emits a
  SystemModel
- `up --input-file record.json` — the model is required, as a positional
  `<model.yaml>` or `--model <path>`. This now fails clap parsing rather than
  silently misparsing.
- `resolve --record <path>` (record-reuse mode)
- `dump run` — `play_launch run` covers dump+replay of a single node

`scripts/compare_records.py` and `scripts/compare_parsers.sh` are retired;
`scripts/compare_models.py` is the parity tool. Fixture justfiles keep only
the model-based `compare-dumps`.

**`<node machine=>` is rejected.** It is ROS 1 roslaunch syntax with no ROS 2
equivalent — `launch_ros`'s `Node.parse()` has no such attribute and
`launch_xml` rejects it, so a launch file using it could never be run by
`ros2 launch`. `Deploy.host` is gone with it. Multi-host launches now use a
standard `<arg>` plus `if=` and partition at resolve time; see
`docs/guide/multi-host.md`.

The parser gained per-element attribute allowlists (`xml/attr_spec.rs`) that
**error** on unknown attributes and **warn** on known-but-unimplemented ones,
on both the XML and YAML frontends, guarded by a differential test against real
ROS 2. Long-standing back-compat aliases (`<group namespace=>`,
`<push-ros-namespace ns=>`) stay warnings so existing files keep working.

**Composable nodes no longer inherit the container's namespace.** A
`<node_container namespace="X">` names the container *process*; ROS 2 does not
push it onto the composable nodes loaded into it. play_launch used to, doubling
a segment:

```text
before: /system/system_monitor/system_monitor/cpu_monitor
after:  /system/system_monitor/cpu_monitor      (matches `ros2 launch`)
```

This changes the names those nodes register with. **If you address such a node
by name — a remap, a diagnostics config, an RViz display, a per-node scheduling
override — check it after upgrading.** 8 of 145 nodes on one Autoware stack.

**Parameter sources emit in ROS order.** An inline `<param>` no longer always
beats a `<param from="...">` file; position decides, matching `launch_ros`.
A launch file relying on the old precedence will see different parameter values.

**`KEY:=VALUE` launch arguments are no longer silently dropped** in cases where
they previously were.

**Model keys for un-named nodes carry a `-N` ordinal.** A `<node>` with no
`name=` is keyed `/ns/<exec>-1`. The suffix marks the key as executable-derived
— precisely the set whose key is *not* the node's ROS graph name. See
*Known limitations*.

**An unknown `sched_class` is an error.** It used to become `SCHED_OTHER`
silently.

**`delay_load_node_millis` is removed** from play_launch's config, rather than
being resurrected, now that `max_concurrent_load_node_spawn` does what it always
claimed to (see *Fixed*). A dead declaration survives in the developer binary's
config struct, read by nothing.

**`ros-launch-resolve` is not shipped in the wheel.** It is a
developer/integration binary for consumers that resolve launch trees without a
ROS runtime. `pip install play_launch` is the whole product.

### Added

**Unified SystemModel.** `dump` and `resolve` emit the same complete model —
structure, contracts, and scheduling — and both parsers produce the *same* model
regardless of which one built it. `up` spawns from it directly.

**New verbs.** `up`, `measure`, `verify`, `setcap`, `context`. The surface is
twelve verbs: `launch`, `run`, `up`, `resolve`, `dump`, `check`, `plot`,
`contract`, `context`, `setcap`, `verify`, `measure`.

**`play_launch measure <run-dir> --model <m.yaml>`** turns a recorded run into a
platform-file `overrides:` fragment. It measures **CPU time and response time
separately and keeps them apart**: `budget_us` becomes a CBS *runtime*, which is
CPU time, while take→publish elapsed time carries preemption, blocking and DDS
wakeup. On one validated run a callback's response max was 40.43 ms against
3.05 ms of CPU — a design that measured elapsed time would have declared a 40 ms
reservation for a 3 ms callback. Budgets take the observed **maximum**, not p99:
under CBS an overrun is throttled to the next replenishment. Paths that cannot
be measured are printed with the reason rather than omitted.

**The full Linux scheduling surface.** The apply layer moved from
`sched_setscheduler(2)` to `sched_setattr(2)`, which is the only one that can
express `SCHED_DEADLINE`, uclamp, or `sched_flags`. Typed placement makes a
priority on `SCHED_OTHER` or a CPU mask on `SCHED_DEADLINE` unrepresentable.
New override vocabulary: `cpus`, `nice`, `uclamp_min`/`uclamp_max`, `budget_us`,
plus `reservations: off|required`. `SCHED_DEADLINE` is derived from declared
budgets and rates. `play_launch verify` reports whether the required cpuset
partition exists; `scripts/provision_rt_cpuset.sh` creates one.

**Cost is authorable.** `budget_us` is the only source for a reservation's
runtime. Absent a declared budget the cost is ABSENT — the
`feasible ON INCOMPLETE EVIDENCE` diagnostic now fires for real instead of
silently reading a deadline as a cost.

**Edge-machine behaviour.** Children get `oom_score_adj +300`
(`PLAY_LAUNCH_OOM_SCORE_ADJ`) so the kernel picks a launched node over the
desktop session. Composable spawn admission is governed by `MemAvailable`
rather than a worker count, so a container no longer serialises six TensorRT
builds into waves. Optional staged startup (`startup.order`) holds sensor
drivers until their consumers appear **in the ROS graph** — not merely until
`spawn()` returns. Guide: `docs/guide/edge-machines.md`.

**The composable ready-wait is bounded by liveness, not time.** A first-run
TensorRT engine build takes however long that board's GPU takes (~33 s cold,
~45 s cached, measured on one Orin), so there is no correct fixed timeout — a
wrong one SIGKILLs a node partway through normal work and repeats every launch.
The container now polls while the child is alive and reports progress every 15 s;
`PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS` (0 = default = no deadline) exists only
to bound a node that wedges.

**Interception artifacts.** `interception/events.jsonl` (per-message record,
read by `measure`, on by default), `interception/trace.json` (Chrome Trace Event
format — rows are nodes, flow arrows are messages), and
`interception/node_identity.tsv` (model key → pid → the node's real ROS name).

**Contract and scheduling shipping.** Platform files resolve through the same
provider-sidecar and user-overlay channels as contracts;
`play_launch contract eject` copies a resolved provider contract into the
overlay for editing; `play_launch check --sched --explain` prints the merged
plan with per-node provenance.

**Tooling.** `just bump-manifest <tag>` moves the pinned `ros-launch-manifest`
revision across all three manifests and all three lockfiles and verifies exactly
one revision resolves. `just test-parity` runs the cross-parser parity gates,
and `just test-all` now runs them. `just setcap` grants helper capabilities via
a throwaway container so the developer loop needs no password (rootless Docker
is refused rather than silently used).

### Fixed

**`Startup complete` could fire before anything had started** (issue 0016). The
readiness test asked "is nothing in flight?" when it meant "is everything done?"
— true at t=0 — and the progress task exits on completion, so it also silenced
every message that would have corrected it.

**A killed composable was reported as loaded** (issue 0019). The LoadNode
response means "spawn requested"; the container pre-assigns an id and returns
before the child is ready. A composable is now `Loaded` only when ListNodes
confirms it, and `Unavailable` — a container too busy to answer, which is the
normal state *while* a composable constructs — is no longer read as success.
This is what reported a SIGKILLed node as `84/84 loaded`.

**`max_concurrent_load_node_spawn` was dead config.** It was parsed, defaulted
and asserted in a unit test, and read by nothing: `dispatch_pending_loads`
drained its queue ungated, so all 84 composables of a 144-process launch hit
LoadNode at once. It is now enforced for `stock`/`observable` and deliberately
disabled for `isolated`, where LoadNode returns in milliseconds and a cap on
in-flight calls bounds nothing.

**`<choice>` was rejected as an unknown attribute.** It is a child entity of
`<arg>` in real ROS 2 (`declare_launch_argument.py:176`), and the YAML frontend
read it as an attribute — a regression against 0.5.1 that made some real launch
files unparseable.

**`$(eval)` conditional expressions returned `"false"`.** Expressions containing
` if `/` else ` were handled by the string-comparison evaluator instead of
delegating to Python.

**Quoting in `$(eval)`.** Escape-aware delimiter detection now runs before
unescaping, so `'ndt' == 'aruco'` is evaluated rather than raising.

**Scheduling correctness.** `band_violations` compared `SCHED_OTHER` tiers
against the RT band and "clamped" them into it, printing `SCHED_OTHER 10` — a
state Linux cannot represent, and fatal under `--sched-apply strict`. Demotion
left a stale mapper-derived priority. `class: real_time` shipped on best-effort
tiers. Application was never logged, so both halves of an A/B produced
byte-identical scheduling logs.

**`SCHED_FLAG_RESET_ON_FORK` on `SCHED_FIFO` broke the per-TID sweep**, because
the kernel resets scheduling in `sched_fork()`, which runs for *thread* creation
too. It is now set only for `SCHED_DEADLINE`, which the kernel requires it for.

**A declared `budget_us` never reached the chain** — cost lookup keyed by FQN
while overrides key by bare name.

**`model_builder` read the legacy `input:` field**, so every path written in
Vocabulary v2 (`trigger: { input: [...] }`) reached the model looking periodic.

**A stale `play_launch` Python install is refused at import** (issue 0020),
naming the offending `__file__`. Such an install emits container records without
`executable` and file scopes without `ScopeOrigin.path` — one produces an
unrecognisable serde error, the other no error at all.

**Capability messages say *why* a capability is missing** (issue 0015).
`setcap` records path, content hash and time, so the runtime distinguishes
"never granted" from "granted, and the binary has been replaced since" — the
latter being correct kernel behaviour that used to read as the former.

Also fixed: composable node container matching (5 nodes failed to find their
target), `<executable>` tags leaking `-p` ROS params to non-ROS processes, and
six integration tests left stale by a member rename.

### Known limitations

**A model key is not always a ROS graph name** (issue 0017). For a `<node>` with
no `name=`, the key is derived from the executable while the node registers its
compiled-in name — 17 of 145 on one Autoware stack. play_launch matches stock
`launch_ros` here and deliberately does not force `-r __node:=`, which would
rename the node away from its own default and break discovery for anything
addressing it by that name. `node_name.is_some()` is the discriminator, every
model now carries a diagnostic naming the affected keys, and the real name is
recorded in `interception/node_identity.tsv` when interception is enabled.

**Reservations lose to fixed priority on vanilla `rclcpp`.** Measured on one
CPU: RT off 217/1013 frames missed, `SCHED_FIFO` 9/1030, `SCHED_DEADLINE`
42/1038. The cause is CBS's sporadic release model versus an `rclcpp::spin()`
event loop; the "budgets too small" explanation was tested and rejected — larger
budgets were worse. Reported rather than tuned away:
`docs/reports/rt-mixed-criticality/reservations-result.md`.

**`--container-mode isolated` (the default) costs what fault isolation costs.**
It forks a process per composable: on a 12-core AGX Orin, 144 processes vs 60
under `observable`, 10.2 of 12 cores vs 3.9 during startup, 3.5 GiB vs 1.4. That
buys SIGSEGV containment, per-node `oom_score_adj`, memory cgroups and per-node
restart — none of which exist at thread granularity. Container mode is a safety
decision, not a performance one.

**`GET /api/launch-tree` degrades to empty** for a standalone `up --model <path>`
with no preceding launch step; there is no parsed dump to source it from.

**`play_launch context` still reads a `record.json`** as a developer tool for
old or hand-produced files. The model's `structure.scopes` lacks the
namespace/args granularity it needs.

### Notes

- Requires Ubuntu 22.04+ on x86_64 or aarch64. Wheels are published for both.
- The Rust parser remains the default; `--parser python` is the compatibility
  fallback. When the two disagree, Python's behaviour is correct.
- Pinned `ros-launch-manifest` v0.1.7.
