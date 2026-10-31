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
3. cross-scope diagnostics (`"file": "<cross-scope>"`), when any;
4. acknowledged findings (`"file": "<acknowledged>"`), when any (see
   [Acknowledging a finding](#acknowledging-a-finding)). Their number is
   the count the summary prints as `N acknowledged`.

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
| `acknowledged` | bool | `true` only in the `<acknowledged>` group |
| `reason` | string | `<acknowledged>` only: the entry's reason |
| `acknowledged_by` | object | `<acknowledged>` only: `{ "contract_path", "entry" }`, the file and the index of the entry that matched |
| `scope` | number or string | `<acknowledged>` only: the scope id the finding belongs to, or `"<cross-scope>"` |

A refused file is an element with `"rule": "manifest-parse"` under
`"file": "<load>"`, and the exit status is 3. A reader that wants one value
should parse the stream as a sequence of arrays and concatenate them; the
verdict itself is the exit status. (There is no separate verdict object; this
documents the format as it is.)

## Acknowledging a finding

A warning that is true and kept on purpose (a reaction nothing guards,
because its guard is outside the system) makes every CI run say the same
thing, and a new warning then reads like the old ones. Name it in the
contract, once, with why (rlm v0.2.3):

```yaml
rlm: v0.2.3
acknowledged:
  - rule: reaction-unguarded
    at: hazards.lost_pose.reaction
    reason: the MRM operator is watched by the vehicle's own watchdog
```

`rule` is the id in `warning[<rule>]` and `at` the path the diagnostic
reports: the `at <path>` line under a cross-scope diagnostic, the label
under a source excerpt, or `path` in `--format json`. Matching is exact on
both. The list sits at the contract's root; all three keys are required.

```
-- Acknowledged --
  ack[endpoint-unwired]: bringup.contract.yaml:34: node '/dx/control_node' declares `sub: odom` ...
    at nodes.control_node.sub.odom -- acknowledged[0] in bringup.contract.yaml
    reason: odometry is read from a bag in this demo; nothing here publishes it

1 manifest(s) checked: 1 clean, 0 with errors (0 errors, 2 warnings, 1 acknowledged)
```

- An acknowledged warning or info is printed in that section and is no
  longer counted as a warning.
- **Errors cannot be acknowledged.** An entry naming one is
  `error[ack-error]`, and the error is printed as before.
- **An entry that matches nothing is `warning[ack-stale]`** (path
  `acknowledged[<i>]`): the finding is gone, or the rule or path is
  misspelled. An acknowledgement cannot outlive its finding.
- Exit status and `--expect` are unchanged. `--expect` counts errors, and
  acknowledgements touch none; an `ack-error` is an error like any other,
  so it fails an `--expect` that does not name it.
- A diagnostic about one contract is matched only against that contract's
  entries. A cross-scope diagnostic (the dotted path is still the
  contract's own, e.g. `hazards.<h>.reaction`) is matched against every
  contract's entries, since it names no single file.
- `--rule` filters acknowledged findings like any other.
- `resolve` (and `launch`) apply the same entries. An acknowledged finding
  stays in the model's `meta.diagnostics` with `acknowledged` where the
  severity was, the entry and its reason (`[reaction-unguarded]
  acknowledged: … (at hazards.<h>.enters; acknowledged[0] in
  bringup.contract.yaml: <reason>)`), so a tool that keeps the model's
  warnings no longer sees it as one; `resolve`'s summary counts it apart
  (`N warning(s), 1 acknowledged`). An `ack-error` refuses the model like
  any error.

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
  analysed `max_latency`, else the platform file's `budget` for the node.
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

## Service calls on a reaction route

A reaction route that crosses a service call (a handler calling an
operator's `operate`) is only as bounded as the call. In stock ROS 2 a call
can be lost to a missing server, a server that dies holding the request, a
lost request or response, a slow or refusing server, or a deadlock;
services report none of these, and rclcpp's client waits forever unless
the code gives up. So a client states the wait and what it does after it
(rlm v0.2.5):

```yaml
nodes:
  handler:
    concurrency: { exclusive: [] }        # the client's callback has a group of its own
    cli:
      operate:
        call: blocking
        timeout: $(param timeout_call_mrm_behavior) s
        on_timeout: { reaction: emergency_stop }       # a mode, or a scope path
        on_rejected: { reaction: emergency_stop }      # what it does on success: false
        max_transport: 1.5ms                           # the request leg
  operator:
    srv:
      operate: { idempotent: true, stale_after: 50ms }
services:
  /mrm/operate: { type: std_srvs/srv/Trigger, server: [operator/operate], client: [handler/operate],
                  max_transport: 1ms }                 # for clients that state none
```

`on_timeout` may retry: `{ retry: { attempts: 3, backoff: 10ms }, then: {
reaction: emergency_stop } }`, `attempts` counting every try. The route
through the call is charged the WORSE of two branches: success (the request
leg, the server's route, and for a blocking call whose route continues at
the client, the wait for the response) and failure (`timeout x attempts +
backoff x (attempts - 1)` + the route of the rung `on_timeout` falls to).
A rung whose effect is supervised (`safe_state.settle: { ..., supervise }`,
above) takes no failure branch at the call: the supervisor bounds it. The
`--explain` route note names each call:

```
call /sys/handler/operate -> /comfort/operate: failure = 100.00 ms timeout (cli.operate)
  + 10.00 ms emergency_stop route = 110.00 ms; route = the failure branch 110.00 ms (success 20.00 ms)
```

and a rung budget that the failure branch breaks says so:
`reaction 2010.00ms (the call's failure branch: 2000.00 ms timeout
(cli.operate) + 10.00 ms emergency_stop route)`. Only calls a reaction
route crosses are judged; a start-up call or a parameter client is not:

- `call-unbounded` (error): no `timeout:` and `on_timeout:`, and no
  supervised rung, on a rung that has a lower rung for `on_timeout:` to
  fall to.
- `call-unbounded-floor` (warning, also under `--strict`): the same on a
  ladder's FLOOR. A timeout cannot help there, since there is no lower rung
  to fall to: supervise the floor's effect, or guard its output
  (`reaction-unguarded`). A contract whose floor really does fire and
  forget can acknowledge it (`acknowledged:`, above).
- `call-retry-not-idempotent` (error): a retry, and a server that does not
  declare `idempotent: true`: a retry after a lost response runs the
  operation twice.
- `call-retry-unbounded` (error): a retry with no `attempts` or no `then:`.
- `call-late-effect` (warning): `on_timeout` falls to something that is not
  a lower rung of the call's own ladder, and no server declares
  `stale_after:`: a request the client gave up on may still act, and two
  reactions then act at once.
- `call-rejected-unhandled` (warning): a blocking call after which the
  route continues at the client, and no `on_rejected:`.
- `call-deadlock` (warning): a blocking call from a node with no
  `concurrency:` declaration, so every callback serialises and the
  response can only be taken by the callback that waits for it. A
  multi-threaded executor does not prevent it, because the default
  callback group is mutually exclusive. It stays a warning because code
  that spins an executor of its own for the wait is safe, and a contract
  cannot show that; a deadlock that does happen shows on the Health page
  as `call_unanswered`.

At run time, with interception on, the Health page reports
`call_unanswered` for a request still open past its client's `timeout:`
(10 s when it states none), and `play_log/<ts>/interception/calls.json`
counts each client's requests and answers and times the request leg.
`play_launch measure <run-dir> --model <m.yaml>` offers the leg's observed
maximum as the client's `max_transport`, and says whether the middleware
filled each request's source timestamp. `stale_after` relies on that
timestamp, and the checker trusts it: Fast DDS and Cyclone fill it; on
another middleware, run `measure` once to confirm.

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
