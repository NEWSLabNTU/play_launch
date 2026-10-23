# Checking contracts in CI

`play_launch check` is the contract gate. This page is what a CI script can
rely on: which binary ran, the exit status, the negative-test form, the JSON
report, and output that survives a log. (Phase 85 "cheap first", items
I2-I5, I7, I8.)

## Which checker ran: `--version`

```
$ play_launch --version
play_launch 0.13.0 (v0.13.0-9-gabc1234, rlm v0.1.47)
```

The crate version, then `git describe --tags --always --dirty` of the tree
the binary was built from (omitted when built outside a git checkout), then
the `ros-launch-manifest` tag whose grammar the checker reads. A bare semver
could not tell apart two binaries that both said `0.12.0` and disagreed on the
same contract: every commit between two release tags carries the old version
string and a newer grammar. Print it at the top of a CI log.

## Exit status

| status | meaning |
|---|---|
| 0 | no Error-severity diagnostic (with `--expect`: exactly the expected rules erred) |
| 1 | an Error-severity diagnostic; an `--expect` mismatch; a dropped launch action; or the launch file itself failed to parse |
| 3 | a contract file was **refused** (`error[manifest-parse]`): it was dropped whole and not checked at all |

3 wins over every other outcome, `--rule` and `--expect` included. 2 is not
used, because clap exits 2 on a command-line usage error.

A refusal keeps its message and ends with one line naming the checker:

```
error[manifest-parse]: could not parse contract .../bringup.contract.yaml:12: at
'nodes.sensor_node.pub.points_raw.max_rate': unknown key in `pub/sub/cli.<endpoint>` ...
Every contract in this file is now UNCHECKED -- the file is dropped, not partially read
  this checker: play_launch 0.13.0 (v0.13.0-9-gabc1234, rlm v0.1.47); the contract's grammar may be newer than this binary
```

An unknown key may be the author's typo, or a key a newer grammar knows. The
line says which grammar this binary reads, so the two can be told apart.

## A negative test: `--expect <rule-id>`

A contract variant meant to fail one rule used to be tested by comparing the
exit status with 1, and a refusal also exited 1, so a checker too old for the
grammar passed every negative test. State the expected verdict instead:

```bash
play_launch check stage2-rungBudget.launch.xml --expect ladder-rung-budget
```

`--expect` is repeatable. The exit is 0 only when every Error-severity
diagnostic comes from one of the expected rules, each expected rule fired at
least once, nothing was refused and no launch action was dropped. Otherwise
one line says what differed, and the exit is 1 (3 for a refusal):

```
expect: ok -- the errors are exactly [error[ladder-rung-budget] x1]
expect: FAILED -- expected error[ladder-rung-budget] did not fire; unexpected error[rate-hierarchy] x4
expect: FAILED -- 1 contract file(s) were refused (error[manifest-parse]), so the expected rule(s) [rate-hierarchy] were never checked
```

The rule set is taken before `--rule` filtering: `--rule` narrows what is
printed, `--expect` states the whole verdict.

## The JSON report: `--format json`

With `--format json`, diagnostics go to **stdout** as JSON (logs and the
summary stay on stderr). Stdout holds zero or more pretty-printed JSON
arrays, one per group, in this order:

1. contracts that failed to load (`"file": "<load>"`), when any did;
2. one array per checked contract scope that has a diagnostic
   (`"file"` is the scope label: `<pkg>/<file> (scope N) [<channel>: <path>]`);
3. cross-scope diagnostics (`"file": "<cross-scope>"`), when any.

Each element:

| field | type | meaning |
|---|---|---|
| `file` | string | the group: `<load>`, `<cross-scope>`, or the scope label |
| `rule` | string | rule id, e.g. `ladder-rung-budget`, `manifest-parse` |
| `severity` | string | `error`, `warning` or `info` |
| `message` | string | the diagnostic text (may contain a newline: a refusal's checker line) |
| `path` | string | the dotted contract path the finding is about |
| `span` | object or null | `{ "start", "end" }` byte offsets into the contract, when known |
| `channel` | string | per-scope groups only: `overlay` or `provider` |
| `contract_path` | string | per-scope groups only: the contract file |

A refused file is an element with `"rule": "manifest-parse"` under
`"file": "<load>"`, and the exit status is 3. A reader that wants one value
should parse the stream as a sequence of arrays and concatenate them; the
verdict itself is the exit status. (There is no separate verdict object; this
documents the format as it is.)

## Output that survives a log

- **Colour** only when the stream is a terminal, and never when `NO_COLOR` is
  set to a non-empty value.
- **ASCII** (`--ascii`, and on by itself when stderr is not a terminal): `->`
  for the route arrow, `--` for an em dash, `-` / `|` / `+` for section rules
  and source frames, `?` for anything else outside ASCII. Log lines go through
  the same filter.
- **Width** (`--width N`): every output line longer than N characters is
  broken at a space and continued with its indentation plus four spaces. Off
  unless given, so a script's `grep` sees each diagnostic on one line.
- A contract that fails to load is printed once (the loader's own WARN line
  is suppressed under `check`, which renders the diagnostic itself).
- JSON on stdout is not transliterated or wrapped.

## The `--explain` fault-reaction table

When the contract declares hazards, `--explain` prints one row per (hazard,
rung), in milliseconds:

| column | meaning |
|---|---|
| HAZARD | the hazard |
| RUNG | the mode of its ladder this row is for |
| ROLE | `rung`, `window` (a rung the system waits in), `floor` (the last rung), or `skipped` (removed by the fault; the reason is in the notes under the table) |
| DETECT | how long until the fault is detected |
| WINDOWS | the route and window of every windowed rung passed on the way to this one, up to its deadline |
| ROUTE | this rung's reaction route: ONE number, the sum of its hops (path latencies and sampling waits) |
| SETTLE | time for the system to settle in the rung, and how it was obtained (`derived` from parameters, or `literal` as stated); `unknown` when neither; `window >=N` for a window |
| TOTAL | DETECT + WINDOWS + ROUTE + SETTLE |
| FTTI | the hazard's fault-tolerant time interval |
| SLACK | FTTI - TOTAL |

The per-hop breakdown of a ROUTE is in the matching `fault-reaction-budget`
info above the table.

## The `--explain` timer-period table

`--explain` also prints, per timer path, its PERIOD and the range it may take
(phase 93 S5), with every limit named in the notes under the table:

- MAX: how far the period may grow. Each row above that charges the period
  (a sampling hop, a `reported` detection, a window owner's tick, a measured
  `tick: { period }` term) allows `period + SLACK / count`, where count is how
  many periods the row charges (read off the rows by re-running the
  arithmetic one millisecond longer). The checks a period reaches are then
  re-run with longer periods, and a rule that fires first (a publisher's or
  subscriber's `min_rate_hz`, a window's expiry) is a limit of its own; so is
  a `max_age` the node evaluates on that tick (`mechanism: application` or
  `diagnostics`). A finding about a `source: derived` budget is not a limit:
  the budget is recomputed with the period, and the notes name it. `none`
  when nothing limits the period.
- MIN: the executor running the timer keeps up, `sum of work / period <= 1`
  over its timers (a container's composable nodes share one; any other node
  is its own). A timer's work is the measured work beside its tick in a
  measured budget (the terms other than `tick`), else its own measured or
  analysed `max_latency`, else the platform file's `budget_us` for the node.
  `unknown` when a timer on the executor states none.

A timer whose `rate_hz` states `source: allocated` must lie inside the range:
`error[period-range]` otherwise, and `warning[period-range]` while the lower
end is unknown.

## Supervised budgets

A budget with `supervise: { reaction | enters, mechanism }` (rlm v0.1.61) is
a claim that the runtime catches its miss and starts a fallback (phase 93).
The rules that read it:

- `supervised-fit`: per supervised budget a rung or floor row charges, an
  OVERRUN row (time before it + the budget + the detection delay + the
  reaction's route and settle) against the hazard's FTTI. `error` when it
  does not fit or cannot be computed, `info` with the arithmetic when it
  fits. The detection delay is `supervision.tick` + `supervision.switch`
  for `mechanism: timer`, and `supervision.tick` + the longest callback on
  the node's executor for `mechanism: executor`.
- `supervised-unlowered` (error): a supervised budget no runtime monitor
  checks. Two kinds lower: an input-triggered node path's `max_latency`
  (the route deadline: due = the take + the budget), and a node path's
  supervised settle (the standstill deadline, below). A settle's `response`
  supervised on its own does not: it is one term of the settle.
- A supervised settle (rlm v0.1.62): `safe_state.settle: { decel, jerk,
  response, supervise: { enters | reaction, mechanism, observes } }` (the
  braking profile is the `vehicle.braking` extension's, below)
  supervises the WHOLE settle, from the safe state's command to standstill
  (the derived profile, or the written duration, plus the response). The
  platform arms due = command + settle + how late it sees standstill, and
  a vehicle not at standstill by then starts the reaction. `observes:
  <node>/<subscriber>` names the input standstill is seen on; the
  observation delay is derived from the contract, never written on the
  supervision: that input's period (the slowest of its topic's `rate_hz`
  and its `min_rate_hz`) + its `max_transport` (the subscriber's, else the
  topic's). No `observes`, no period or no transport: `supervised-unlowered`
  says which. The overrun row for the rung charges the time before the
  settle + the settle at the hazard's entry speed + the observation + the
  detection + the reaction's route and settle:

  ```
  odd_exit  comfortable_stop (nodes.<op>.paths.operate.safe_state.settle)  overrun ...
  before it 10454.00ms + settle 10939.67ms (profile 9996.67ms + response 943.00ms)
  + observation 156.00ms (/mrm_handler/kinematic_state: period 100.00ms + max_transport 56.00ms)
  + detection 2.00ms + reaction scope path 'island.emergency_stop' 188.33ms + settle 4484.33ms
  = 26224.33ms
  ```

  The settle's terms (its response, a written duration) take the level
  `supervised`, so `measured-unsupervised` does not fire for them; CHECKED
  BY reads `runtime monitor (settle-deadline: reaction ..., <mechanism>,
  standstill seen on <subscriber>)`. The overrun case contains the rung's
  own total: when the rung itself does not fit, the rung's own row is the
  error and the overrun row says so as `info`, not a second error. The
  model carries the lowering on the safe state (`safe_state.supervise:
  { response_ms, observe_ms, observes, reaction, mechanism }`).
- `supervised-executor-long-callback`: a `mechanism: executor` supervision
  whose detection a callback can delay. The executor checks the deadline in
  its spin loop, between callbacks, so detection is the tick + the longest
  callback on that executor, and that is a bound only when every callback
  on it has an analysed bound. A callback that is only measured (or
  supervised, or unstated) can run past it; and when the supervised path's
  own callback is the one that overruns, its late output is published
  before the check runs, so the miss is seen only when the callback
  completes, late by the overrun. Fix: `mechanism: timer` (an independent
  timer and thread, which catches a long or stuck callback), or `source:
  analysed` on every callback of the executor. A warning, or an error under
  `policy: { unsupervised_measured: error }`.
- `measured-unsupervised`: a measured budget nothing supervises, a warning,
  or an error under `policy: { unsupervised_measured: error }`.

## Extensions: names, locks and deprecations

A contract names the extensions and the severity profile it uses:
`use: { vehicle.braking: "^1", iso26262: "^1" }`. `vehicle.braking` (the
braking profile: `entry_speed` on a hazard, `settle: { decel, jerk }` on a
safe state) and the seven reference profiles ship with the checker; any
other extension is an rlx file in an `rlm/` directory beside the contract
or in a package that registers the ament resource `rlm_extensions`. Three
things belong in CI:

- **The lock.** `play_launch check --update-lock <launch>` writes
  `rlm.lock` beside each contract: the version and sha256 of every extension
  and profile it resolved. Commit it, and run CI with `--locked`: a changed
  extension, a vanished version or an unlocked `use:` entry is then an
  error (`lock-mismatch`, `lock-missing`) rather than the warning it is
  without the flag (`lock-drift`). `--explain` lists every resolution with
  its full sha256.
- **Implicit extensions are refused.** A contract that writes the braking
  keys without `use: { vehicle.braking: "^1" }`, or hazard severities with
  no profile and no `severity_levels:`, fails with
  `error[deprecated-implicit-extension]` (play_launch 0.19 warned; 0.20
  refuses). `play_launch migrate <dir> --write` adds the `use:` lines (and gives a
  plain-number `entry_speed` its unit, `m/s`); `play_launch migrate <dir>
  --check` exits 1 while anything is left to rewrite.
- **Worst cases.** A derived value's direction is proved by Z3;
  `worst-case-unknown` is a warning that `check --strict` makes an error.

What an extension derives reaches the SystemModel as a plain number with
its source (`safe_state.settle_derived`, `hazards.<h>.ftti_derived`), so an
image built from the model never evaluates the extension.

## Two notices about what the arithmetic reads

- `info[declared-not-charged]`: a `max_transport` declaration (subscriber- or
  topic-level). The fault-reaction arithmetic (hazard detection, reaction
  routes, the table above) does not charge it yet; path and chain latencies
  do. One line per declaration, with the contract line. It goes away when
  phase 85 I1 charges the key on the guard edge.
- `warning[endpoint-unwired]`: an endpoint under a node's `sub:`/`pub:` that no
  topic wires (no `topics:` list in any scope names `<node>/<endpoint>`, and no
  topic derived from launch remaps carries it). Such an endpoint is in no graph
  edge and no count. An endpoint a path names as `input`/`output` is the
  `wiring` rule's and is not repeated here.
