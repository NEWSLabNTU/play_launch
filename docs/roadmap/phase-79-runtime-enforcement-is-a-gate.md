# Phase 79 - runtime enforcement is a gate, or it says it is not

Status: **planned** (2026-09-21). Follows phase 36, which built the rule engine
and the `--enforce-rules` modes, and phase 73, which gave it the hazard
vocabulary and a clock. Those phases built the rules; this one makes the verb
that runs them capable of failing. Issues of record: #0031, #0032, #0033, and
item 4 of #0036.

## Why

`--enforce-rules strict` is documented as the CI mode - "First violation
triggers shutdown and non-zero exit (CI mode)" (`cli/options.rs:806`) - and is
not a gate. Three breaks, each independently true at 0.11.0 (`5eaa3191`),
compose into a run that is green with no measurement behind it, which is the
worst shape a gate can have.

**The default mode has no event source** (#0031). `enforce_rules` defaults to
`Warn` (`cli/options.rs:749-750`) and its help text promises "Warn: log
violations" (`:744-748`). The engine's only event source is interception - the
LD_PRELOAD hooks on `rcl_publisher_init`/`rcl_publish`/
`rcl_subscription_init`/`rcl_take` - and `InterceptionSettings::default()` has
`enabled: false` (`cli/config.rs:461-464`), reachable only from a `--config`
YAML because there is no CLI flag for it. `up.rs:607` creates
`interception_consumers` only when `runtime_config.interception.enabled`, and
`up.rs:1400` builds the engine only `if !interception_consumers.is_empty()`, so
the branch at `:1404-1415` that constructs a `RuleEngine` for any non-`Off`
mode is never reached. Measured 2026-09-18 against a `demo_nodes_cpp` talker at
1 Hz and an overlay contract demanding `min_rate_hz: 1000`: with
`interception: enabled: true` in a config file, three violation lines; without
it, no lines and exit 0. Nothing on the terminal distinguishes the two runs.
The integration harness carries the trap as a comment and works around it
(`tests/tests/runtime_enforcement.rs:110-117` writes an `interception_on.yaml`
for every test); a user has no such comment.

**Strict does not read the severity the engine writes** (#0032). `emit()` sets
`strict_violated` at `runtime_enforcement/mod.rs:866-867`, before `severity` is
looked at; `emit_repeatable()` does the same at `:910-911`. The `severity`
argument reaches only the log line (`:877`) and the JSONL record (`:879-894`).
`graph-deviation-runtime` is emitted at `Severity::Warning` (`:522-523`) by
design, because a launch tree is a subset of the running graph - and every
ROS 2 node creates `/rosout` and `/parameter_events` publishers before user
code runs. The strict run of 2026-09-18 was therefore over 57 ms after the
talker's first publisher was created, and the `rate-hierarchy-runtime` error
the contract was written to catch, which needs a 0.5 s window, was never
evaluated. The unit test that covers strict mode pins this: `emit`s a
graph-deviation, a warning, and asserts the flag is set
(`mod.rs:1968-2002`).

**The strict watcher does a third of a shutdown** (#0033). The signal path
does three things on the first SIGTERM - `kill_process_group(pgid, SIGTERM)`,
`shutdown_tx.send(true)`, `member_handle.shutdown()` - at
`commands/signal_handler.rs:113-117`. The `--on-startup-failure exit` path does
the same three at `:664-671`, under a comment reading "Mirror the signal path
EXACTLY". The strict watcher (`up.rs:1589-1602`) does the middle one only. Both
actor families assume the first one happened: `regular_node_actor.rs:371-380`
and `container_actor/process_lifecycle.rs:398-402` both comment
"kill_process_group() already sent SIGTERM to all processes" and go straight to
`child.wait()`. Nobody signalled the children, so the wait never returns, the
runner task never completes, and the main loop at `signal_handler.rs:136-152`,
which breaks only on runner completion or a real signal, spins on. Reproduced
twice: the process was still alive when an outside SIGINT arrived 14 s and then
45 s later, with its nodes running and their output forwarders already silent.
There is also no exit code to carry: the strict flag is never consulted beside
`startup_failed` at `up.rs:1679-1683`, and no integration test runs strict at
all - `tests/tests/runtime_enforcement.rs` passes `warn` or `off` and drops the
process rather than reading its status.

**And the feature has no page.** Outside `docs/issues/`, `grep -rl
enforce-rules docs` returns three roadmap files and nothing else. The seven
files in `docs/guide/` cover none of interception, `runtime_violations.jsonl`,
the rule table or the precondition above (#0036 item 4).

The order matters, because each wave's regression test is unreachable until the
previous one lands. A severity gate cannot be tested without an event source,
and a strict teardown cannot be tested without a trip that fires for the right
reason: today the only trip available in an integration run is `/rosout`.

## What it does

### The precondition is decided once, and said out loud

`InterceptionSettings.enabled` becomes `Option<bool>` so that "explicitly false
in the config" is distinguishable from "absent" - the distinction the current
`bool` with `#[serde(default)]` (`config.rs:425-427`) cannot carry. Resolution
happens once, in `Config::resolve` where `ResolvedRuntimeConfig.interception`
is built (`config.rs:928`), by a precedence with three sources: a new
`--interception on|off` CLI flag, then an explicit config value, then implied
on by `enforce_rules != Off`. The `info!` line that already prints
`Interception: enabled|disabled` (`up.rs:144-151`) names which of the three
decided.

Where it still resolves off under a non-`Off` mode - the user wrote
`interception: enabled: false` and left `--enforce-rules` at its default -
`warn!` once at startup naming the precondition. Under `Strict` that is an
error before the first spawn, not a silently green run.

### Severity means what the engine writes

`strict_violated` flips only for `Severity::Error`. Warnings keep their `warn!`
line and their `runtime_violations.jsonl` record, so nothing observable is
lost; what changes is which of them ends a run. `--strict-on warning` exists
for the other reading, as a threshold rather than a second mode.

Separately, `RuleEngine::new` seeds the runtime topic map with `/rosout`,
`/rosout_agg` and `/parameter_events` as implicitly external, so
`graph-deviation-runtime` reports only topics an author could plausibly have
declared. This is a runtime-view seeding, not a contract change:
`ContractView::externals` (`runtime_enforcement/view.rs:41`, filled at `:140`
from `index.externals`) stays exactly the set the contracts declare, and the
static checker is untouched.

### One teardown, used by every caller

The three steps become `initiate_shutdown(pgid, shutdown_tx, member_handle,
reason)` next to `CompletionContext` (`signal_handler.rs:23-28`), and the three
sites that need them call it. An actor stops assuming somebody else signalled
its child: the shutdown branch waits briefly and then uses the `graceful_kill`
that already exists for the Stop/Restart control events
(`regular_node_actor.rs:24-55`, used at `:537` and `:558`) - SIGTERM, five
seconds, SIGKILL. The reason reaches the exit code the way
`--on-startup-failure exit` already does.

## Waves

- **W1 - give the mode an event source, or say so.** `InterceptionSettings
  .enabled` becomes `Option<bool>` (`config.rs:425-427`, default `None` at
  `:461-464`); `--interception on|off` lands on `CommonOptions` beside
  `--disable-monitoring`; the precedence resolves in `Config::resolve`
  (`config.rs:928`) and is logged at `info` at `up.rs:144-151` naming the
  decider. A non-`Off` mode with interception off `warn!`s the precondition
  once, and under `Strict` returns `Err` before spawn. `--enforce-rules`'s help
  text (`options.rs:744-748`) names the precondition. The workaround comment
  and `interception_on.yaml` in `tests/tests/runtime_enforcement.rs:110-117`
  are deleted - the tests pass `--enforce-rules` and get an engine. Regression
  test: `up` with default options and a contract that any run violates emits
  either the violation or the no-event-source warning; exiting 0 with neither
  is the bug, and one test asserts the warning by passing an explicit
  `interception: enabled: false`.
- **W2 - strict honours the severity vocabulary it already writes.**
  `strict_violated` flips only at or above a threshold, default
  `Severity::Error`, in both `emit` (`mod.rs:866-867`) and `emit_repeatable`
  (`:910-911`); the log line and the JSONL record are unchanged.
  `--strict-on <severity>` sits beside `--enforce-rules` in `ContractOptions`
  (`options.rs:749`). `RuleEngine::new` (`mod.rs:212-227`) seeds
  `topic_hash_to_fqn` with `/rosout`, `/rosout_agg` and `/parameter_events`
  after the `view.topics` and `view.externals` passes. `strict_mode_trips
  _atomic_flag` (`mod.rs:1968-2002`) splits in two: the graph-deviation it
  fires today must NOT set the flag, and an error-severity emission must.
- **W3 - one teardown, and an exit code.** `initiate_shutdown(reason)` in
  `signal_handler.rs` replaces the bodies at `:113-117` and `:664-671` and is
  what the strict watcher calls at `up.rs:1596-1597`. The shutdown branches at
  `regular_node_actor.rs:371-380` and
  `container_actor/process_lifecycle.rs:398-402` wait briefly for a child that
  may already have been signalled, then `graceful_kill` it. The engine records
  the first violation that crossed the threshold (rule id and FQN) beside the
  flag; the watcher stores it, and `play()` returns `Err` naming it next to the
  `startup_failed` check at `up.rs:1679-1683`. Test: an integration test that
  plants a violated contract under `--enforce-rules strict` and asserts a
  non-zero exit within a few seconds, which needs a harness that reads the
  status instead of dropping the `ManagedProcess`
  (`tests/tests/runtime_enforcement.rs:148-156`).
- **W4 - `docs/guide/runtime-enforcement.md`.** Written last, so it documents
  the fixed behaviour rather than the broken one: the mechanism (LD_PRELOAD,
  the SPSC ring, the consumer task), the `interception` precondition as W1
  leaves it including the precedence, the mode table
  (`off | warn | strict | record-only`, and that `record-only` writes no
  JSONL - `mod.rs:879`), the rule-id table from `runtime_enforcement/mod.rs`
  with the contract field each rule reads (`graph-deviation-runtime` `:522`,
  `qos-match-runtime` `:569`/`:581`/`:1260`, `rate-hierarchy-runtime` `:646`,
  `max-age-runtime` `:691`, `drop-rate-runtime` `:739`/`:1352`,
  `max-latency-runtime` `:804`, `consistency-runtime` `:842`,
  `deadline-runtime` `:1283`, `liveliness-runtime` `:1303`/`:1331`), the JSONL
  shape (`RuntimeViolation`: `rule_id`, `severity`, `fqn`, `message`,
  `timestamp_ns`), and what the `hazard-*` rules observe -
  `hazard-detected` (`:1028`, `:1180`), `hazard-reaction` (`:1444`),
  `hazard-recovered` (`:985`) and `mode-availability` (`:1139`), from phases 73
  and 75. Brief C section 3 is the draft.

## Gates

- `cargo test -p play_launch runtime_enforcement::tests::strict` - the W2 pair:
  a warning-severity emission leaves the flag clear, an error-severity one sets
  it.
- `cargo nextest run -p play_launch_tests runtime_enforcement` - the W1
  regression test (default options, violated contract, violation line OR
  precondition warning, never neither) and the W3 exit test (strict, violated
  contract, non-zero exit within 10 s), both without any `--config` file.
- A default `play_launch launch --contracts <dir> pure_nodes.launch.xml`
  prints either a `[runtime]` violation line or the no-event-source warning.
  Exit 0 with neither is the bug this phase exists to close.
- The same invocation with `--enforce-rules strict` ends the run non-zero
  within seconds - the nodes are gone, the supervisor has exited, and no
  outside SIGINT was needed. The 2026-09-18 captures under
  `scratchpad/playlaunch/runtime/strict{,2}/` are the before.
- `--enforce-rules strict` on a contract that does not mention `/rosout` or
  `/parameter_events` survives node startup; those two produce no
  `graph-deviation-runtime` line at all.
- `grep -rl enforce-rules docs/guide` returns `runtime-enforcement.md`.

## Limits

- **The rule set does not change.** Which rules exist, what each measures and
  what severity it carries are phase 36 and phase 73 work; this phase changes
  only whether a mode has an event source, which severity ends a run, and what
  happens when one does. The one exception is the three rcl-internal topics,
  which is a seeding of the runtime view, not a new rule.
- **Interception is not free.** The per-message cost stays what phase 58 W2
  measured (~150 ns per publish/take, 0.15% of a core at 10k msg/s, from the
  `CLOCK_THREAD_CPUTIME_ID` read at 85 ns against `CLOCK_MONOTONIC`'s 17 ns).
  W1 turns it on for a non-`Off` mode because that is what the mode already
  claims to do; `--interception off` and `--enforce-rules off` both decline it,
  and the `info` line says which.
- **The default reading is that only an error ends a run.** `--strict-on
  warning` is offered for users who want the other one. It is not the default
  because the warning class exists for claims the launch tree cannot settle -
  `graph-deviation-runtime` is warn-only by design (phase 36.5) - and a gate
  that trips on those is the gate #0032 reports.
- **A derived tier that is never applied is not addressed here** (#0035). That
  is a scheduling-side silence, on the path between `derive_sched_plan` and the
  apply sweep, and it is phase 80.
- **`record-only` stays a collector**, but only of the JSONL. It writes no
  `runtime_violations.jsonl` (`emit` and `emit_repeatable` gate the write on
  `Warn | Strict`) and nothing in this phase gives it an exit code, because the
  mode's whole purpose is an offline pass over `interception/events.jsonl`.
  CORRECTION, measured in W4: it does NOT "evaluate no rules", as this line
  first claimed and as `--help` claimed with it. `observe()` early-returns only
  on `EnforceMode::Off` and `up` builds a `RuleEngine` for every non-`Off`
  mode, so `record-only` evaluates every rule and emits every `warn!` line --
  it simply writes no file. The old wording was accidentally true before W1,
  when the mode had no event source at all and therefore evaluated nothing.
