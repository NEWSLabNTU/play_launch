# Phase 94 - contract values read from the launch tree

**Status: ✅ complete (2026-10-27): wave 1 with rlm v0.2.4, wave 2 in play_launch alone (rlm unchanged); the safety island's four gaps fixed the same day (rlm unchanged).** Phase 85's D9, and DX 1.1's provenance for
numbers a parameter holds. Design of record: rlm
`docs/design/contract-values.md`.

## Why

A contract's configured numbers are typed twice, once in the launch tree and
once in the contract. Three keys can name a parameter to check it, and rlx
extension keys can read one by bare name; nothing else can. On `rt_av_demo`,
`check bringup.launch.xml lidar_hz:=100` reads a contract still saying 50 Hz,
and the safety island types each node's `update_rate` beside the `param:` that
checks it.

## What

`$(param [<node>] <name>) <unit>`, `$(var <name> [@<node>]) <unit>` and
`$(let <name>)` inside any quantity of a contract, evaluated as a
dimension-checked expression; a root `let:`; `params: … default:`. The
check form (`{ duration, param }`) stays for requirements. See the design for
the resolution table and the error messages.

## Wave 1 - references without parser changes

rlm:

- [x] W1.1 Grammar: a reference-bearing value parses to an unevaluated
      expression (`types`), wherever a quantity is accepted; a literal parses
      exactly as before. Root `let:` (ordered, no forward reference, no
      cycle). `params.<p>.default`. `source:` beside a reference is a parse
      error (`source-on-reference`). Field table rows, generated format
      reference, `GRAMMAR_VERSION`. *2026-10-27, rlm `types/src/values.rs`:
      the parser records each reference-bearing scalar as a `ValueRef` by its
      YAML path and leaves the key absent; a second pass
      (`parse_manifest_str_with_values`) puts evaluated numbers in place.*
- [x] W1.2 Evaluation: the expression subset (`+ - * /`, parentheses,
      numbers with units, references with a REQUIRED unit) on rlx's evaluator
      and unit table, against a caller-supplied lookup of parameter and
      configuration values. Dimension check against the key
      (`param-dimension`, with the inverse suggestion). *2026-10-27:
      `values::Lookup`, `Manifest::resolve_values`, `evaluate_text`; units
      and dimensions from `rlx::units`/`rlx::dim` (types depends on rlx
      without z3). The grammar's own small parser, since rlx's expression
      parser takes a unit only after a number.*
- [x] W1.3 Extension keys typed `param` accept `$(param <name>) <unit>`;
      `migrate` rewrites the bare form. *2026-10-27: rlm `ext-param-read`
      rewrite (bundled extensions' `param` keys); play_launch evaluates both
      spellings through `contract_values::ext_value` (the bare name becomes
      `$(param <name>) <unit>`, the key's `unit` else SI). Plain quantity
      keys (`entry_speed: $(let odd_speed)`) take the read form too.*
- [x] W1.4 Model: a number read through a reference carries its provenance
      (`<field>_source: { param | var | let }`); readers of the number are
      unchanged. Release. *2026-10-27, rlm v0.2.4 (a1b8081): a `sources:` map
      per entry; see Deviations.*

play_launch (resolver):

- [x] W1.5 Evaluate every reference per scope instance, from
      `launch_param_values` (`$(param)`) and the scope's include arguments and
      `<arg>` defaults (`$(var)`), BEFORE any rule reads the index. Errors
      `param-unset`, `param-node-unknown`, `param-node-missing`,
      `param-not-number`. No launch tree: the value is unknown and rules
      report incomplete evidence. *2026-10-27,
      `resolve/src/ros/contract_values.rs`: `load_manifests` parses each
      scope instance through `parse_scope_instance`, before the per-manifest
      checks, the cross-scope rules and the per-mode re-runs (which clone
      the resolved index). `LaunchParam.at` records where each value was set
      (a launch file, a parameter file's name, `<set_parameter>`); the
      parser now records a parameter file's path beside its content.*
- [x] W1.6 Instance tag on diagnostics of a contract bound to more than one
      scope (`<contract>:<line> [<ns>]`). *2026-10-27: on the value
      diagnostics and the `--explain` heading.*
- [x] W1.7 `--explain` prints each read value's provenance (the parameter's
      source and line). *2026-10-27: "Values read from the launch tree"; the
      line of an inline `<param>` and the `$(var)` chain behind it are W2.3.*
- [x] W1.8 Fixtures: the island excerpt (read form, check form, extension
      keys, `let`), `rt_av_demo` (a `ms` parameter, a command-line override,
      a shared argument), one negative fixture per error, one contract
      included twice. `rt_av_demo`'s contract migrated to the read form, and
      its derived schedule unchanged. *2026-10-27: `contract_values_island`
      (contract_takeover read from the launch tree: same verdicts and
      totals, row for row), `contract_values_{unset, node_unknown,
      node_missing, not_number, dimension, var_unset, source_on_ref}`,
      `contract_values_twice`; tests in `tests/tests/manifest_check.rs`. The
      ranked-plan snapshot gained the new fixtures and no existing plan
      moved. Negative control: `param-unset` disabled, the test fails.
      rt_av_demo: see Evidence.*
- [x] W1.9 Guide: a section in the contract guide; CHANGELOG.
      *2026-10-27: `docs/guide/first-contract.md` "Numbers the launch file
      already sets: read them".*

## Wave 2 - `<let>` and the configuration log

- [x] W2.1 Parser: a per-scope log of configuration events (set, group push,
      group pop) with file, line and sequence number, and each node's
      sequence number, in both frontends; crosses the pyexec C ABI (bump).
      *2026-10-27: `RecordJson.config_log` (`ConfigEvent { kind: set | unset
      | push | pop | reset, name, value, origin: cli | arg | let | include |
      group, file, line }`), `ScopeEntry.config_span` (`[start, end)`, the
      include's arguments included), `config_seq` on every node, container
      and composable record, and `NodeRecord.param_origins` (where an inline
      `<param>` written as a substitution was written). Every assignment in
      `LaunchContext` goes through `assign_configuration`, which logs it; the
      `.launch.py` half returns its events in `Produced.config_events` and
      the driver appends them at the hand-over, offsetting the captures'
      sequence numbers. pyexec C ABI 10 -> 11. The Python dumper records the
      same log from `launch`'s own actions (`dump/visitor/action.py`).*
- [x] W2.2 `$(var x)` reads the file's final state and refuses an ambiguous
      name (`var-ambiguous`); `$(var x @<node>)` reads the state where the
      node was declared. *2026-10-27, `contract_values.rs`: the log is
      replayed (`config_state`: a push saves, a pop restores, a reset
      clears) to the end of the contract scope's span, or to the node's
      `config_seq`.*
- [x] W2.3 `--explain` follows a parameter whose text came from a `$(var)` to
      its assignment or the command line. *2026-10-27: `LaunchParam.via`;
      `<- <param> bringup.launch.xml:29 = $(var lidar_hz)` `<- lidar_hz =
      100, command line`, one line per step.*
- [x] W2.4 Fixtures: a reassigned `<let>`, a `<let>` inside a scoped group,
      a child file's `<let>` reaching its parent; the Python frontend through
      the same tests. *2026-10-27: `contract_values_let` (XML) and
      `contract_values_let_py` (the same contract on a `.launch.py`), the
      Python dumper on `contract_values_let`, and the parser's
      `include_semantics` matrix, where every node name is built from
      `$(var …)` and is what stock `ros2 launch` starts: replaying the log to
      each node's position must spell its name, across XML, YAML and Python
      includes and scoped groups. Negative controls: the group pop left
      unlogged fails the replay test; the ambiguity check disabled fails the
      fixture test.*

## Decisions where the design is silent (wave 1)

- **A count takes a reference without a unit** (`max_consecutive:
  $(param n k)`): it has none. Everywhere else a missing unit is a parse
  error.
- **A `$(let)` takes no unit** after it; its entry carries one.
- **Where references are accepted**: every bound key and its braces form,
  a timer's and a topic's `rate_hz` (and `{ value }`), QoS `deadline`,
  `lease_duration`, `lifespan`, a mode's `window`, the two counts, `ingress`,
  `min_interarrival`, `budget`, `budget_period`, `sync`'s durations, and an
  extension key's value (a `param` key or a plain quantity). Not on a
  measurement record or `supervision:`, which are observations and platform
  terms, not configuration.
- **The dimension check runs where the launch tree is** (`param-dimension`
  from the evaluation, not the parser), so its message names the value and
  its source, and one wrong dimension does not drop the whole file the way a
  grammar error does.
- **Two rules beyond the table**: `var-unset` (no launch file sets the
  configuration) and `value-range` (a negative duration, a rate that is not
  positive, a count that is not whole).
- **An unknown timer rate leaves the trigger unknown** (no `trigger:`),
  never a guessed timer.
- **`$(var x)` in wave 1** reads the scope's own table for an include (the
  parser records it once the file has run) and, for the root, the command
  line and then the record's final `variables`. Wave 2 replaces this with the
  configuration log wherever a record carries one; it remains the path for
  a record without one (an older dump).
- **`$(param <node> <name>)` with a bare node** must name a node key of this
  contract (else `param-node-unknown`, near misses named); `/<fqn>` may be
  any node of the launch tree, and its `default:` is found when a contract
  node resolves to that FQN.
- **An extension's bare `param` key under a hazard** keeps its reading on
  the one guard publisher that sets the parameter, now spelled `$(param
  <fqn> <name>)` and evaluated through the same path; the read form there
  with one argument is `param-node-missing`.
- **A reference error on an extension key** is reported inside the rule
  that needed the value (`settle-param-unresolved`, `extension-eval`), with
  the rule id of the reference in parentheses.

## Decisions where the design is silent (wave 2)

- **"Every assignment made during that file"** is the scope's span: its
  include's arguments, its own sets and declarations, and what the files it
  included set, including sets inside scoped groups that later ended. Two
  assignments are the same value when they are equal as numbers (`0.5`,
  `0.50`) or as text.
- **A declaration that does not set** (an `<arg>` whose name is already set,
  by the command line or an include) is not an assignment and is not
  logged; its default is not a value the name ever had.
- **`var-unset` says why** when the name was assigned but is not in force
  where it is read: `(it is assigned at <let> bringup.launch.xml:12, out of
  scope here)`.
- **The suggestion's `@<node>`** names the contract's first node key; the
  author picks the right one.
- **Provenance is spelled as the file spells it**: `<let>`, `<arg>
  default`, `include argument`, `command line`; for a `.launch.py`
  `SetLaunchConfiguration` and `DeclareLaunchArgument default`.
- **`@<node>` takes a node key of the contract or a `/<fqn>`**, as
  `$(param <node> …)` does, and names near misses when it matches neither.

## Deviations

- **Model provenance is a `sources:` map, not `<field>_source` keys.** Each
  entry that holds a read number (`PathContract`, `SubContract`,
  `PubContract`, `SrvContract`, `TopicContract`, `HazardContract`,
  `ModeContract`) has one `sources:` map keyed by the model field it fills
  (`rate_hz`, `max_age_ms`, `qos.lease_duration_ms`, `window.duration_ms`),
  beside the number as the design wants; one field per struct instead of
  about twenty-five, and the nested fields (`qos.*`, `on_violation.*`,
  `safe_state.*`, `window.*`) need no field of their own. Each read is
  tagged `kind: param | var | let` rather than `{ param: { … } }`: the
  externally tagged form serializes as a YAML tag (`!param`), which not
  every model reader parses.
- **The instance tag is on the value diagnostics**, not on every rule's
  `<file>:<line>` (W1.6 asks for the diagnostics of a twice-bound contract;
  the other rules' sites are unchanged so existing fixtures' messages do
  not move).
- **`rt_av_demo`'s contract needed `use: { iso26262: "^1" }`** before it
  loaded at all: since 0.20 `deprecated-implicit-extension` is an error, and
  on `main` `check` exited 1 and `resolve` refused to emit a model. The
  baseline below is the contract with that line added (`migrate` adds it).

- **`contract_takeover_jitter` is migrated to the read form** (its four
  `decel`/`jerk` keys): it is the twin `migrate` must reproduce from
  `contract_spelling_deprecated`, and `migrate` now rewrites the bare form.
  Its findings and totals are unchanged.

- **The Python frontend records no line numbers** (a `.launch.py` action
  carries none; the Python dumper sees `launch`'s actions, which carry none
  either), so its provenance names the file alone.
- **A composable node has no `param_origins`**: its parameters reach the
  record as resolved values, and `--explain` names its launch file only.
- **W2.4's Python half is three tests, not the same test run twice**: the
  pyexec frontend through `contract_values_let_py`, the Python dumper
  through `resolve --parser python` on the XML fixture (both refusals, no
  other error), and the parser matrix's `.launch.py` cases.
- **The Python dumper gained the log too**, which the plan did not list:
  without it `resolve --parser python` read every `<let>` as `var-unset`,
  since that dumper records no configurations at all, and the two parsers'
  models would disagree on any contract reading one.

## Evidence (wave 1)

`rt_av_demo`, the read form against the `use:`-only contract, both with
`--sched bringup.system.posix.yaml --explain`: the check output differs only
in one line number, and the resolved model only in the contract's sha256,
that line number, and the new `sources:` maps; every priority, period and
tier is byte-identical. With `lidar_hz:=100`:

```text
/safety/lidar_driver   SCHED_FIFO  38  -  derived(chain_aware: lidar_to_brake boundary RM period=10ms) -> prio 38 (max over 2 paths)
/safety/lidar_driver/sample   10.00  unknown  10.00
-- Values read from the launch tree (--explain) --
  /safety/lidar_driver paths.sample.trigger.timer.rate_hz = 100 Hz  <- $(param rate_hz) Hz
                                                                    <- /safety/lidar_driver rate_hz = 100 Hz, bringup.launch.xml
  /safety/brake_controller sub.obstacles.min_rate_hz = 100 Hz       <- $(let lidar_rate)
                                                                    <- lidar_hz = 100 Hz, command line
```

where the contract before phase 94 printed `period=20ms` for any
`lidar_hz`.

Gates: `just check` clean (after the field census learned the two phase 94
readers, `types/src/values.rs` and `contract_values.rs`). `just test-all`:
two failures in its first run, both expectations this phase changes
(`migrate`'s edit count on the deprecated fixture, now 7 with the four
`ext-param-read` rewrites; and `settle-param-unresolved` naming the parameter
through `param-unset`'s message); both updated, and `manifest_check` (94
tests) and the resolver's 304 tests rerun green.

## Evidence (wave 2)

`rt_av_demo` with `lidar_hz:=100`, `check --explain`:

```text
  /safety/lidar_driver paths.sample.trigger.timer.rate_hz = 100 Hz  <- $(param rate_hz) Hz
                                                                    <- /safety/lidar_driver rate_hz = 100 Hz, <param> bringup.launch.xml:29 = $(var lidar_hz)
                                                                    <- lidar_hz = 100, command line
```

`contract_values_let`, the same verdicts from the Rust parser, the pyexec
frontend (`contract_values_let_py`) and the Python dumper:

```text
error[var-ambiguous]: bringup.contract.yaml:17: `$(var timeout) s` was assigned 0.5 (<let> bringup.launch.xml:7) and 2.0 (<let> bringup.launch.xml:9). Name the point: `$(var timeout @gate)`.
error[var-unset]: bringup.contract.yaml:22: `$(var inner_hz) Hz`: no launch configuration `inner_hz` is in force where bringup.launch.xml finishes (it is assigned at <let> bringup.launch.xml:12, out of scope here)
  /gate sub.cmd.max_age = 500 ms        <- $(var timeout @gate) s
  /other sub.cmd.min_rate_hz = 0.5 Hz   <- 1 / $(var timeout @other) s
  /inner pub.status.min_rate_hz = 5 Hz  <- $(var inner_hz @inner) Hz
  /source pub.cmd.min_rate_hz = 20 Hz   <- $(var child_hz) Hz
                                        <- child_hz = 20 Hz, <let> child.launch.xml:5
```

Gates: `just check` failed once on ruff (B905, a `zip()` without
`strict=` in the dumper), fixed and clean on the rerun. `just test-all`
green: parity gates (simple_test, container_events, lifecycle,
rt_workspace, autoware; lifecycle_autostart skips on Humble), stock parity
18 of 18, integration 321 of 321, resolver 308, parser 563/517/473 by
suite. The IR suite caught two `LaunchTraverser` literals missing the new
`include_line` field, #0040's shape again.

## Gaps the safety island found (2026-10-27)

The island migrated its real contract on `main` (1e4ba882): the three
timers to `rate_hz: $(param update_rate) Hz`, the four settle keys through
`migrate` (`decel: $(param target_acceleration) m/s^2`, ...), the ODD speed
to a root `let: { odd_speed: 8.33 m/s }` read by both hazards'
`entry_speed: $(let odd_speed)`, and two `acknowledged:` entries for the
`reaction-unguarded` at `hazards.{hpc_loss,odd_exit}.enters`. Four gaps,
reproduced on a copy of its contract and launch tree and fixed:

- **G1: `--explain` listed the core keys only.** The rlm parser records a
  core key's reference as a `ValueRef`, but it cannot record an extension
  key's: only the extension knows the key's dimension. Those were evaluated
  inside the derivation alone. `extensions::record_ext_reads` (after
  `prepare`, before any rule) now evaluates every extension key written as a
  reference, on a hazard or a node's settle, and adds it to the contract's
  `value_refs` with its value, so the listing and the model read it through
  the same code as a core key's. The listing is in the contract's order, and
  a `$(let)` names its entry: `let odd_speed: 8.33 m/s,
  safety_island.contract.yaml:445` for a literal, `let lidar_rate: $(var
  lidar_hz) Hz = 100 Hz, bringup.contract.yaml:19` and then the entry's own
  reads otherwise.
- **G2: the model's `sources:` missed the same reads.** Fixed by G1: the
  path's map gains `safe_state.settle.decel` and `.jerk` (the
  `<head>.<key>` form `model_field` already gives a nested key), the
  hazard's `entry_speed`. An extension key has no model field of its own, so
  its source is keyed by the contract key it fills. A bare `decel:
  target_acceleration` is not recorded: it is not a reference, and recording
  it would move every existing fixture's model.
- **G3: a derivation recorded the reference's spelling.** With the bare
  name the settle's inputs read `settle.decel: target_acceleration = -2.5
  m/s^2 (<file>)` and `hazard.entry_speed: 8.33 m/s`; with the read form
  `settle.decel: $(param target_acceleration) m/s^2 = …` and
  `hazard.entry_speed: $(let odd_speed)`, in the model and in the
  `settle-derived` trace (`($(let odd_speed))`). **A defect**: a derivation
  is about the number and where it came from, and the same number from the
  same parameter must read the same whichever spelling supplied it, or a
  migration that changes nothing changes the model and every reader that
  compares models. The reference is `sources:`'s and `--explain`'s to show.
  `extensions::derivation_text` records what the replaced spelling records:
  a single `$(param … <name>)` is `<name>`, a `$(let)` of a literal is the
  literal, any other single read is its value; an expression of several
  terms has no other spelling and is recorded as written.
- **G4a: the loader's count disagreed with the verdict.** `Loaded N
  manifest(s) … (0 errors, 12 warnings)` was printed before
  acknowledgement (and before `--strict` and `--against`), beside `check`'s
  `(0 errors, 10 warnings, 2 acknowledged)`. The counts are dropped from that
  line rather than moved: the loader cannot know the verdict, and `check`'s
  summary and `resolve`'s `SystemModel:` line are the counts.
- **G4b: `resolve` ignored `acknowledged:`.** `build_checked_model` now
  calls `acknowledge::apply` right after loading its own index (once per
  index: `check` and the model path never share one). The model's
  diagnostics are strings (rlm `Meta.diagnostics: Vec<String>`), so no rlm
  field was needed and rlm is unchanged (v0.2.4): an acknowledged finding is
  embedded as `[<rule>] acknowledged: <message> (at <path>; acknowledged[<i>]
  in <file>: <reason>)`. It is kept rather than omitted, so the artifact
  says what was accepted and why; `acknowledged` sits where the severity
  was, so nano-ros's `nros sync`, which keeps the `] warning:` lines, no
  longer reports it. An `ack-error` refuses the model like any error, and
  `ack-stale` is a warning there as in `check`. `resolve`'s summary counts
  warnings as `check` does (`10 warning(s), 2 acknowledged`): it used to
  count every embedded line, infos included.

Evidence on the island copy (`tmp/island`, the unmigrated and migrated
contract resolved from the same directory): `check --explain` lists nine
values (two entry speeds, three timer rates, four settle keys), each with its
parameter file or `let:` line, and ends `(0 errors, 10 warnings, 2
acknowledged)` where the unmigrated contract ends `(0 errors, 12 warnings)`.
The two models differ in exactly: the `sources:` maps (rate, decel and jerk
on the three paths, `entry_speed` on both hazards); the contract's
`sha256` in `meta.inputs`; the contract line numbers inside diagnostic
messages, since the migrated file is laid out differently (two timers
lose their `param:` line, `let:` and `acknowledged:` add eleven); and the two
`reaction-unguarded` lines, now `acknowledged` with their reason. With
`sources:`, the hash and the line numbers removed, only those two lines
differ; every derivation, settle, priority and total is identical.

Tests: `contract_values_island_reads_the_launch_tree_and_keeps_its_verdicts`
(the listing, the let entry, no `($(` in a derivation),
`contract_values_island_carries_every_read_into_the_model` (the extension
and `let` sources, the derivation's inputs), and
`resolve_embeds_an_acknowledged_warning_as_acknowledged` (`check_dx.rs`).
The island fixture's contract now declares `estop_op.update_rate` and types
`lease_ms` an integer, so it resolves to a model at all (`resolve` refused
it on the parameter declarations); its findings and the ranked-plan
snapshot are unchanged. Negative controls: `record_ext_reads` not called,
both island tests fail; `acknowledge::apply` not called on the model path,
the resolve test fails.

Gates: `just check` failed once on rustfmt (the two test files and one
`matches!`), clean on the rerun. `just test-all`: parser 563/517/473,
resolver 308, parity gates green (lifecycle_autostart skips on Humble),
integration 322 of 323; the one failure,
`interception::test_health_compares_running_parameters_with_the_launch`
(`could not set use_sim_time: 500`, a parameter service call through the
web UI that timed out under the suite's load), touches no contract and
passed on both reruns alone.

## Out of scope

A rung switched by a boolean parameter, indexing a list parameter,
`$(env)`, run-time parameter changes. See the design.

## Acceptance

- Every existing contract fixture resolves byte-identically (no reference,
  no change), and the ranked-plan snapshots do not move.
- `rt_av_demo` with `lidar_hz:=100` re-derives its rates from the override.
- Each error of the design's resolution table is reachable from a fixture,
  with the message the design shows.
