# Phase 94 - contract values read from the launch tree

**Status: 🔄 wave 1 complete (2026-10-27, rlm v0.2.4); wave 2 in progress.** Phase 85's D9, and DX 1.1's provenance for
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

- [ ] W2.1 Parser: a per-scope log of configuration events (set, group push,
      group pop) with file, line and sequence number, and each node's
      sequence number, in both frontends; crosses the pyexec C ABI (bump).
- [ ] W2.2 `$(var x)` reads the file's final state and refuses an ambiguous
      name (`var-ambiguous`); `$(var x @<node>)` reads the state where the
      node was declared.
- [ ] W2.3 `--explain` follows a parameter whose text came from a `$(var)` to
      its assignment or the command line.
- [ ] W2.4 Fixtures: a reassigned `<let>`, a `<let>` inside a scoped group,
      a child file's `<let>` reaching its parent; the Python frontend through
      the same tests.

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
  line and then the record's final `variables`.
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

## Out of scope

A rung switched by a boolean parameter, indexing a list parameter,
`$(env)`, run-time parameter changes. See the design.

## Acceptance

- Every existing contract fixture resolves byte-identically (no reference,
  no change), and the ranked-plan snapshots do not move.
- `rt_av_demo` with `lidar_hz:=100` re-derives its rates from the override.
- Each error of the design's resolution table is reachable from a fixture,
  with the message the design shows.
