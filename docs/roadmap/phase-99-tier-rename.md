# Phase 99 - tier rename: the scheduling unit in the standards' word

**Status (2026-11-05): planned. Decided (user, 2026-11-05): `tier` is
renamed, in its own phase on both sides (phase 98 N7); T0 decided
(user, 2026-11-05): the new word is **task** (option A).**

**Progress (2026-11-05): T1 done, rlm `v0.3.10` (`a7580de`); T2 done,
play_launch 0.27.0.** rlm reads `[tasks.*]` / `group_tasks`, warns once per
old key (`system config: deprecated-spelling: ...`, naming the new key, the
tasks or components, and `migrate`), and `migrate` rewrites a `system.toml`
(`system_config::migrate_system_toml_str`; the verb picks it for any file
named `*system.toml`, and a directory walk finds them). The sched types are
renamed with `#[deprecated]` aliases for the type names, `DEFAULT_TIER` and
`is_single_tier`. A field or a variant cannot be aliased:
`ResolvedTaskTable.tasks`, `AssignRule.task`, the `SchedError` variants and
`BandViolation.task` move with the tag, so nano-ros's two
`ResolvedTierTable { tiers }` literals (`rtos_realizer.rs`) move in its W1.
play_launch says task in every message, the `--explain` block (`N task(s)`,
`  task <name> prio=...`), the evidence (`derived((applied): task ...)`), the
apply log (`applied task '<name>'`), the IPC (`AppliedTask`, `apply_task`)
and `run_info.json` `sched_unapplied[].task`, with `tier` beside it for
0.27 only; `resolve --system` prints `warning[deprecated-spelling]` per old
key. `tests/fixtures/system_tasks_deprecated` proves the old file warns and
migrates to the same execution layer. T3 next. Design of record: rlm
`docs/design/standard-names.md` M7 (design issue 57); the release schedule
is that document's section 6, as phase 98 ran it. Spans rlm, play_launch,
the safety island and nano-ros (its own phase: "follow rlm's tier rename").

## Why

`tier` is our word for one named set of scheduling parameters and its
placement, to which nodes and callback groups are bound: rlm's `TierDef`
(class, period, deadline, budget, deadline policy, spin period, `above`,
and per-platform priority, stack, core, scheduler class, preemption
threshold, time slice), the resolver's `ResolvedTier` (that set resolved
for one target, plus its members) and the model's `execution.tiers`.
What it becomes at run time differs by realizer, and the word says
nothing about either:

- **nano-ros (embedded):** one RTOS task (Zephyr thread, FreeRTOS task,
  NuttX/ThreadX thread) running an executor over the callback groups bound
  to it; since its phase 509 (RFC-0108) an authored tier is one band of
  preemption levels at one kernel priority. nano-ros already says "tier
  task" (614 mentions).
- **play_launch (Linux):** the resolver gives each node its own tier, and
  play_launch applies it to every thread of that node's process
  (`sched_setattr`/`sched_setscheduler` plus affinity, through
  `play_launch_rt_helper`).

"Tier" in ordinary English is a rank, which is the one thing it is not:
the priority is a field of it, derived by the priority assignment. It also
collides with three unrelated senses (TIER IV's `tier4_*` packages, which
every Autoware launch carries; nano-ros's CI and coverage tiers; nano-ros's
command tiers).

## Candidate terms

| Term | Source | What it names there | Fit |
|---|---|---|---|
| **task** | AUTOSAR OS / OSEK: a task has a priority, a stack, a preemptability and (AUTOSAR timing protection) an execution budget; the RTE maps runnables to tasks. RTOS APIs: FreeRTOS task, the Linux kernel's `task_struct`. The periodic task model (Liu and Layland): period, deadline, execution budget | the schedulable unit at one priority, carrying its stack, that several runnables are mapped into | close: nodes and callback groups bound to a tier are AUTOSAR's runnables mapped to a task; the head's period/deadline/budget are the task model's T/D/C. Loose where the task model means one recurring computation (a tier hosts several) |
| **scheduling context** | seL4 MCS (a capability holding budget and period, bound to a thread); nano-ros's own `SchedContext` (phase 110, "inspired by seL4 MCS") and its retired `[[scheduling.contexts]]` (RFC-0004 decision A replaced it with `[tiers]`) | the time-budget half of a thread's scheduling: in seL4 the priority and the core stay on the thread | partial: names the head, not the placement. Collides with nano-ros's live `SchedContext` (about 1,000 mentions), which is a different layer: one executor (one tier) may hold several sched contexts |
| **executor** | ROS 2 (rclcpp/rclc executor); RFC-0108's worker | the object that dispatches callbacks; in nano-ros a tier runs one | right for nano-ros's run time only: on Linux a tier is applied to a process, not an executor, and an executor carries no priority or placement in ROS 2 |
| partition | ARINC 653; ISO 26262-6 Annex D (freedom from interference) | a time AND space partition with its own memory | rejected: a tier isolates no memory and guarantees no time, so the word would be the category error S1 removed (`severity` holding an ASIL) |
| process | ARINC 653 process (period, time capacity, stack size, base priority, deadline: the closest attribute list of all) | the schedulable unit inside a partition | rejected: on Linux a process is the OS process play_launch spawns, a different thing |
| preemption level / band | Baker's Stack Resource Policy; preemption-threshold scheduling (Wang and Saksena); RFC-0108 | the order in which work may preempt other work | rejected: covers ordering and preemption, not the period, budget, stack and core a tier carries; "level" already names degradation, integrity and executor levels |

ISO 26262 names no unit for this; its software partitioning is the isolation
argument above, not a scheduling record.

## Decision slot (T0)

**Decided (user, 2026-11-05): A, task.** Recorded here and in the design
doc's M7 row.

| Option | Authored keys | Types | Model | For | Against |
|---|---|---|---|---|---|
| **A. task** (recommended) | `[tasks.<name>]`, `group_tasks` | `TaskDef`, `TaskPlatformSpec`, `ResolvedTask`, `ResolvedTaskTable`, `DEFAULT_TASK` | `execution.tasks` | AUTOSAR's runnable-to-task mapping is exactly our node/group-to-tier binding; every RTOS and the Linux kernel call the unit a task; nano-ros's "tier task" collapses to "task"; short, so keys and C names stay short | collides with the task model's one-computation sense: RFC-0108's per-callback "task record" (72 mentions in nano-ros) must become "callback record"; play_launch's tokio tasks (code only, about 450) stay |
| B. scheduling context | `[scheduling_contexts.<name>]`, `group_scheduling_contexts` | `SchedulingContextDef`, `ResolvedSchedulingContext` | `execution.scheduling_contexts` | names what the head is (a set of scheduling parameters), not what runs; no clash with any task sense | long keys and C names; seL4's sense excludes priority and core; nano-ros must first rename its own `SchedContext` (about 1,000 mentions) or carry two meanings; reverses nano-ros's RFC-0004 decision A |
| C. executor | `[executors.<name>]`, `group_executors` | `ExecutorDef`, `ResolvedExecutor` | `execution.executors` | the word every ROS user knows; matches what nano-ros runs | wrong on Linux (a process's scheduling, no executor involved); ROS 2's executor is a dispatcher with no priority, so the word misleads in the other direction |

**Recommendation: A, task.** It is the standard term for the unit that
holds a priority, a stack and a budget and into which several runnables are
mapped, it fits both realizers, and it is the shortest. Its one real cost,
nano-ros's per-callback "task record", is a local rename in nano-ros's phase.

Below, the new word is written `<new>` (`<News>` for the plural key, e.g.
`tasks`); T0 fills it in.

## Where `tier` is today

Counts on 2026-11-05 (rlm `v0.3.9` `ce34243`, play_launch `b7313dd9`,
nano-ros `d1b1b21823`, island `5099147`, demo `0041fbc`): every token
containing `tier` in tracked files, `frontier` excluded, split by a line
classifier into the scheduling sense, nano-ros's CI/test-tier sense and
TIER IV (`tier4_*`). The classifier is a keyword heuristic, so the split is
approximate (within a few percent); the totals are exact.

| Repo | all | scheduling | CI/test tier | TIER IV |
|---|---|---|---|---|
| rlm | 613 | 585 | 3 | 25 |
| play_launch | 1,549 | 1,421 | 58 | 70 |
| nano-ros | 28,611 | 22,875 | 5,656 | 80 |
| island | 245 | 158 | 3 | 84 |
| demo | 255 | 90 | 10 | 155 |

(Phase 98's design-doc count, 14,554 for nano-ros, was the bare word in
`.rs/.md/.yaml/.py/.js`; this one counts identifiers too, in every file
type.)

### rlm (585)

- **Types** (`sched`): `TierDef` (16), `TierPlatformSpec` (14),
  `ResolvedTier` (33), `ResolvedTierTable` (22, field `tiers`),
  `DEFAULT_TIER` (26, value `"default"`), `ResolvedTier::default_tier`,
  `is_single_tier`, `SchedError::UnknownTier`,
  `NodeMatchedByMultipleTiers`, `AssignRule.tier`. `ReservedBand` keeps
  its name.
- **Authored keys**: nano-ros's `system.toml` `[tiers.<name>]` (with
  `[tiers.<name>.<target>]` sub-tables) and `[[component]] group_tiers =
  { <group> = <tier> }`, parsed by rlm's `model::system_config`
  (`deny_unknown_fields`). No contract key and no platform-file key names a
  tier (the legacy `[tiers.*]` platform file was removed in v0.3.0). No
  rule id contains `tier`.
- **Model**: `execution.tiers` (name to `TierDef`); `execution.bindings`
  values are tier names (key unchanged). `upgrade_legacy_tier_keys`
  already rewrites `execution.tiers` durations.
- **Messages**: the three `SchedError` texts ("assign rule references tier
  ...", "tier `x` has no [tiers.x.<target>] sub-table", "tier `x`: ...")
  and the resolver's own.
- **Docs**: `docs/scheduling.md` (66 lines), `launch-manifest.md` (11),
  `model-boundary.md`, `contract-theory.md`, generated
  `format-reference.md`/`key-inventory.md` if they list the system keys;
  `design-issues.md` and `CHANGELOG.md` are history.
- Rust: 439 tokens; docs: 141.

### play_launch (1,421)

- **Rust, 693 tokens**: `sched_loader.rs` (232 lines), `sched_plan.rs`
  (114), `sched.rs` (101), `rt_helper_client.rs` (38), `model_builder.rs`
  (20: `synthesize_tiers`, `execution.tiers`), `commands/up.rs`. Own types:
  `AppliedTier` (the IPC record to `play_launch_rt_helper`, 40),
  `apply_tier`, `UnappliedComposableTier`, `tier_name`.
- **User-facing text, 59 string literals**: the `--explain` scheduling
  block ("Scheduling (posix, priority_assignment=...): N tier(s)", "  tier
  <name> prio=..."), a node's priority evidence when no rate or deadline
  derived it ("tier '<name>' -> prio <p>"), the resolve summary ("N
  tier(s)"), the apply log ("sched: pid ... (tier '<name>')"), the
  `sched_plan` refusals ("tier '<name>': RT priority ... out of range",
  "binding ... references undeclared tier"), and the composable note
  (`render_composable_tier_note`). No CLI flag, no `--explain` column
  header and no rule id contains `tier`.
- **Run artifacts**: `run_info.json` `sched_unapplied[].tier` (issue
  0035's composable rows).
- **Docs**: `docs/guide/rt-scheduling.md` (25 lines), CLAUDE.md (16). The
  roadmap, `docs/superpowers/`, issues and CHANGELOG are history.
- **Tests**: 76 tokens (`sched_apply.rs`, `rt_workspace.rs`, the
  `composable_tier_*` tests), fixture comments in `*.system.posix.yaml`.
  No play_launch fixture authors `[tiers.*]`.

### Safety island (158) and demo (90)

- Island: `system.toml` authors no tier since its phase 13 (the levels are
  derived; the file and three `CMakeLists.txt` comments say so). The rest
  is docs (95), CMake comments (26) and scripts (12). It moves with its
  play_launch/nano-ros pin bump; nothing in it is parsed.
- Demo: docs only (87). Its "tier 1/2/3" of the contract acts is a fourth,
  unrelated sense (an evaluation level) and stays.

### nano-ros (22,875)

Surveyed in nano-ros's own phase. In brief: model reads through rlm types
(431 tokens in 24 files), its own C ABI (64 distinct identifiers, 698
tokens in public headers: `nros_tier_spec_t`, `nros_tier_task_memory_t`,
`nros_board_*_run_tiers*`, `n_tiers`), its own Rust API (82 `pub` items),
three Kconfig symbols (`NROS_ZEPHYR_MAX_TIERS`, `..._TIER_STACK_SIZE`,
`..._TIER_STACKS_IN_DTCM`), CMake, generated code (40 golden files, 17
templates), 10 `system.toml` files authoring `[tiers.*]`, and docs (about
13,000 tokens, half of them in archived issues and phases).

## Units

| Unit | Work | Acceptance |
|---|---|---|
| T0 | The user picks the word (decision slot) | the slot and the design doc's M7 row say "decided: <new>" |
| T1 (done, rlm v0.3.10) | **Release R in rlm.** `system_config`: `[<News>.<name>]` and `group_<News>` read; `[tiers.*]` and `group_tiers` read for one release with one `deprecated-spelling` warning each naming the new key; `migrate` rewrites a `system.toml` (a TOML rewrite: it edits YAML contracts today). `sched` types renamed (`<New>Def`, `<New>PlatformSpec`, `Resolved<New>`, `Resolved<New>Table`, `DEFAULT_<NEW>`, the `SchedError` variants, `AssignRule.<new>`), each old name kept as a `#[deprecated] pub type` / `pub use` alias for this release so a consumer can bump the tag before renaming. Messages and docs say the new word. The model is unchanged (schema 4), as M1-M5 kept theirs at R | rlm release; a test per old key showing it warns and resolves to the same table; `migrate` turns a `[tiers.*]` `system.toml` into one that reads clean; nano-ros compiles unchanged against R (the aliases) |
| T2 (done, 0.27.0) | **play_launch at R.** `just bump-manifest`; the 59 user-facing strings, the `--explain` scheduling block and evidence text, `run_info.json` `sched_unapplied[].<new>` (with `tier` written beside it for one release, as M6 did with `carried_hops`); its own types (`AppliedTier`, `apply_tier`, `UnappliedComposableTier`; the IPC is between two binaries of one wheel, so no compatibility is needed); `rt-scheduling.md`, CLAUDE.md, `--help` text | `just check`, `just test-all` green; `--explain` on `rt_workspace` reads the new word with the same plan; `check-terms` unchanged (the word is not retired yet) |
| T3 | **Island, demo, nano-ros at R.** Island: docs and comments when it bumps its play_launch pin to the release carrying R (nothing parsed changes). nano-ros: its phase's W1 (bump to R, migrate its ten `system.toml` files) | island `check` verdicts and totals unchanged; nano-ros W1 merged |
| T4 | **Release R+1 (refuse)**, only after R is on PyPI and nano-ros W1 is merged: `[tiers.*]` and `group_tiers` are refused naming the new key and `migrate` (`removed` rows; `system.toml` keeps `deny_unknown_fields` as the backstop); **model schema 5**: `execution.<News>` written, `execution.tiers` rewritten on read for good (`SystemModel::from_yaml_str`, beside `upgrade_legacy_tier_keys`); the deprecated type aliases deleted; `run_info.json` drops `tier`. nano-ros follows in the same window (its W3: the model field read and the alias-free build) | one negative test per old key; a schema-4 model loads and resolves to the same table as its schema-5 twin; nano-ros builds against R+1 |
| T5 | **Release R+2 (purge)**: `tier` in both `check-terms` gates, with `tier4` (TIER IV packages) excluded by pattern, not by file | the gate lists `tier`; a negative control injects it; no `tier4_*` launch name trips it |

No rule id or contract key carries the word, so the `acknowledged:` and
`rule_names` steps of phase 98 have nothing to do here.

## Order

T0 first; nothing starts before the word is fixed. T1 + T2 as release R
(one rlm release, one play_launch release), then T3 (nano-ros W1 is the
item on the path: T4 waits for it). T4 one release later with nano-ros W3
in step, T5 the release after.

## Notes for whoever does it

- The authored key lives in nano-ros's file format but rlm's parser, so the
  alias and the refusal are rlm's; nano-ros only migrates its files.
- `execution.bindings` keeps its key: its values are the renamed unit's
  names, and a name is the user's (`mrm`, `derived-<node>`).
- `DEFAULT_<NEW>` keeps the value `"default"`; a saved model naming
  `default` in `bindings` must resolve unchanged.
- History (closed phases, design issues, CHANGELOG, `docs/superpowers/`)
  keeps `tier`; a status line names the rename where it matters.
- TIER IV's `tier4_*` package names, nano-ros's CI/coverage/command tiers
  and the demo's act tiers are other words that happen to be spelled the
  same; none of them moves.
