# Phase 80 - the plan and what was applied

Status: **planned** (2026-09-21). Follows issue
`docs/issues/0035-composable-tier-silently-dropped-in-non-isolated-containers.md`,
filed from brief C on 2026-09-18 and re-verified against 0.11.0 (`5eaa3191`;
`src/` is unchanged at `ec8d716a`). Relates to phase 44.4, which built the one
warning that covers a slice of this, and to phase 78, which owns the mapper
input and therefore owns nothing here: this phase changes no derivation, only
what is said about one.

## Why

The scheduling story is one sentence: derive the priority from the contract,
apply it per process. Under two of the three container modes the first half
happens and the second does not, and nothing says so.

`play_launch resolve --sched <platform>` derives one tier per node carrying a
rate or deadline fact, composables included. `check --sched --explain` prints
them (`verbs/check.rs:183-184`). `resolve` writes them into the model's
`execution.bindings`, which is what `SchedPlan::from_model`
(`sched_plan.rs:252`) reads at start. The container's own tier is then applied
at `container_actor/actor.rs:326-345`, on every entry to Running. The
composables' tiers are looked up per member at `up.rs:1104-1114` and handed to
the container actor, and there, on the LOADED event, the guard is
`component_events.rs:189-191`:

```rust
if config.sched_mode != crate::execution::sched_apply::SchedApplyMode::Off
    && pid > 0
    && let Some(tier) = sched_tier.as_ref()
```

The guard is correct. Under `observable` and `stock` a composable is a thread
pool inside the container process and has no pid of its own, so the load report
carries 0; under `isolated` it is fork+exec'd and has one. The comment above it
says exactly that (`component_events.rs:182-184`). What it does not do is
report: `pid == 0` with a tier present falls through with no `warn!`, no
`plan.warnings` entry, nothing in `play_launch.log`, and no change to the model
that still binds a tier to that FQN. The container gets its priority; every
composable inside it silently keeps the container's.

A user who reads `--explain`, sees `SCHED_FIFO 39` against a composable, and
then runs `ps -eLo tid,cls,rtprio` reads a contradiction with nothing between
the two to explain it. Autoware runs almost everything as a composable, so this
is the ordinary case rather than a corner of it.

The existing near-miss warning covers a narrow slice.
`chain_container_colocation_warnings` (`sched_plan.rs:408-460`, called from
`up.rs:714`) fires only when two or more CHAIN-MEMBER composables share a
non-isolated container - `members.len() >= 2` at `:451`. It is therefore silent
for:

- a single tiered composable in a container, which is the usual
  one-container-per-node-group shape;
- the `rate_monotonic` and `deadline_monotonic` mappers, whose plans carry no
  `chain_member_nodes` at all;
- any plan built by `SchedPlan::from_model`, which leaves `chain_member_nodes`
  empty by its own doc comment (`sched_plan.rs:44-51`) - and that is the only
  plan source `up` has since phase 47.B3 (`up.rs:693-704`).

The third bullet is the one that decides the scope: on the path a user actually
takes, the warning that exists can never fire.

## What it does

### The rule

A refusal belongs where the decision is knowable, not where the consequence
lands.

Container mode is a command-line flag, fixed before anything is spawned and
already consumed at `execution/context.rs:449-485` to rewrite every container's
package and executable. The LOADED handler is the last place in the run that
learns it, and the worst place to report it: by then one message per composable
arrives interleaved with startup, after the decision it describes is already
irreversible.

### Where the decision is knowable

`up.rs` has both halves at `:549-559` - `container_contexts` from
`prepare_container_contexts_from_model` and the composable contexts from
`prepare_composable_node_contexts_from_model`, whose `NodeInstance.container`
(`context.rs:596`) names each composable's target container - and the plan at
`:693-704`. The existing co-location call at `:714-721` sits exactly at that
join, which is why it is the placement this phase extends rather than replaces.

The refusal model is three statements below it: the missing-capability check at
`up.rs:733-737` warns under `--sched-apply warn` and `eyre::bail!`s under
`strict`, at the start boundary, before a single node is spawned.

### What is reported

Per composable that carries a tier and lives in a container that will be spawned
under `observable`, `stock` or `clone_vm`: the node FQN, the tier name and
priority (`AppliedTier.tier_name`, `.priority`, `sched.rs:305-324`), the
container FQN, the mode in force, and `--container-mode isolated` as the thing
that would schedule it. One line per composable, at `warn`. Under
`--sched-apply strict`, the same set is an error at the start boundary.

## Waves

- **W1 - decide before spawning.** In `up.rs`, beside the
  `chain_colocation_warnings_for_plan` call at `:714`, walk every composable
  context that resolves to a tier under `SchedPlan::for_fqn`
  (`sched_plan.rs:235`) and compare against `common.containers.container_mode`.
  `warn!` once per composable under `observable`/`stock`; `eyre::bail!` under
  `--sched-apply strict`, in the shape of `:733-737`. This covers the
  single-composable case, the non-chain mappers and the `from_model` plan - the
  three the co-location warning misses - and it is mode-aware in the one place
  that knows the mode.
- **W2 - keep the late guard, make it quiet on purpose.** The guard at
  `component_events.rs:189-191` stays as it is: it is also the failure-report
  path, where `pid == 0` means something else entirely (a LOAD_FAILED report,
  not a thread-pool composable). The `pid == 0` with a tier present case logs at
  `debug`, naming W1 as where the user was already told, so the two paths cannot
  be read as one in a log.
- **W3 - record the outcome, not just the intent.** The set of tiers derived and
  deliberately not applied is written where a later reader finds it: an entry in
  `plan.warnings`, so `check --explain` prints it beside the tier it just
  derived, and a field in `run_info.json`. The `run_info.json` half has a
  concrete constraint - `run_log::attach_or_warn` runs at `up.rs:298`, four
  hundred lines before the plan exists, and `RunInfo` (`run_log.rs:169-181`) is
  captured there and serialised in the same call (`run_log.rs:287-292`) - so the
  unapplied set is either a second write after `:721` or a deferred field, not a
  new `RunInfo::capture` argument. Why it matters concretely: `play_launch
  measure` reads a run directory and attributes cost per node; without this it
  can attribute a priority that was never set on any thread.
- **W4 - the test that keeps it honest.** In `tests/tests/sched_apply.rs`,
  beside `composable_scheduling_engages_on_isolated_container` (`:257`) and
  reusing its `write_sched_toml` (`:30`) and the `container_events` fixture
  (1 container, 2 composables): resolve a model whose composables carry tiers,
  launch under `--container-mode stock`, assert a warning naming each
  composable; the same run under `--sched-apply strict` must refuse before
  spawn. The second assertion is what stops the silent regression returning -
  under `--container-mode isolated` the same model must produce no such warning
  and the tiers must be applied, which the existing `:257` test already asserts
  from the other side.

## Gates

- `cargo nextest run -p play-launch-tests -E 'test(composable_tier)'` - the W4
  pair: a `stock` run whose stderr names every tiered composable, and the same
  invocation with `--sched-apply strict` exiting non-zero before any node
  spawns (no `play_log/<ts>/node/` entries written).
- The same fixture under `--container-mode isolated` produces no such warning
  and still prints the phase-38.9 apply line per composable, which is
  `composable_scheduling_engages_on_isolated_container` unchanged.
- `play_launch check --sched <platform> --explain` on a fixture with tiered
  composables prints, beside each derived tier, whether the container mode in
  force can apply it. `check` has no `--container-mode` (the co-location doc
  comment at `sched_plan.rs:380-393` states why), so the line names the mode
  assumed - the default - and says which modes cannot.
- `play_log/<ts>/run_info.json` from a `stock` run names the unapplied set; the
  same run under `isolated` names none.

## Limits

- **This does not make `observable` or `stock` able to schedule a composable.**
  That is a per-thread question. `container-isolation.md:226-241` measures it:
  `cpu` and `cpuset` are threaded controllers, so CPU control per node inside one
  process is possible in principle, but it requires a fixed node-to-thread
  mapping that the standard ROS 2 executors do not provide - callback dispatch is
  non-deterministic - and `memory`, `io` and `oom_score_adj` are process-only
  (`:264-272`). Buying it with `isolated` is not free either: phase 61 measured
  144 processes against 60, 10.2 of 12 cores against 3.9 during startup, peak
  load1 190 against 45, and 3.5 GiB against 1.4, on the same Autoware launch.
  This phase reports the trade; it does not move it.
- **It changes no derivation.** Whether two equal-period nodes should get equal
  priorities is issue 0039, which is a `ros-launch-manifest` question
  (`sched/src/mapper.rs:214-222`), and the mapper's input is phase 78's. A tier
  that is wrong before it is dropped stays wrong after this phase reports the
  drop.
- **It does not revisit the default container mode.** `isolated` is the default
  because a SIGSEGV cannot be contained inside a process
  (`container-isolation.md:213-224`) - a safety decision, not a performance one.
  A user who chooses `observable` for its measured startup cost is making a
  different trade deliberately, and this phase's job is to tell them what that
  trade also costs, not to make it for them.
