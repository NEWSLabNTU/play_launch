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
