# Phase 83 - a takeover the contract can state, and the checker can refuse

Status: **complete** (2026-09-28). The grammar shipped as
`ros-launch-manifest` **v0.1.46** (workspace 0.1.7, an API break:
`Manifest.functions` holds `FunctionDecl`, `Contracts.functions` holds
`FunctionContract`) and is pinned here. Follows phase 82 (what a detector may
see). Asked for by the Autoware Safety Island's phase 8 (unit phase8-W6), whose
design brief is `simple-autoware-safety-island`,
`docs/design/rtss-work-2026/brief-D-scenario.md` sections 1.2 to 1.6.

## Why

The island's RTSS@Work demo is a Drive Pilot-style takeover. The vehicle
leaves its operating domain; the driver is asked to take over within 10 s;
if nobody answers, the island stops the car in lane, and if the HPC dies the
island stops it alone. Most of that was already contract vocabulary
(`on: reported`, `modes` with `fallback`, `on_violation`, `safe_state`,
severity). Four things were not, and two checker behaviours broke the first
contract written for it:

1. **A fault by value.** `on: reported` says some node checks something;
   nothing named WHICH value, so no probe can evaluate it, no plot can draw
   the bound, and a function could be lost only by silence.
2. **A rung the system waits in and then leaves.** The WG's L4 fixture had to
   encode its `T_odd = 10 s` as an FTTI for want of one.
3. **The driver's answer**, which ends the ladder successfully.
4. **A settle that follows the speed.** The island's contract derived its
   2034 ms by hand from an assumed 3.0 m/s; its phase-7 runs braked from
   4.23 m/s, and the vehicle missed the 3 s interval while the island met
   every term of its own.
5. **F1 - `ladder-rung-budget` checked rungs the fault had removed.** With one
   ladder for `odd_exit` and `hpc_loss`, the comfortable-stop rung (the HPC's
   planner brakes) was checked against the HPC-loss interval, although losing
   the HPC is exactly what makes that rung unavailable. Issue #0057.
6. **F2 - the observer could not see an off-host reaction.** It recorded a
   reaction only on a host `Publish` of a sink; the island publishes its
   braking command on the MCU, so the host sees only `vehicle_cmd_gate`'s
   `Take`. Issue #0058.

## What changed

### The grammar (rlm v0.1.46)

| key | where | shape |
|---|---|---|
| `when` | hazard; function `{ of: [...], when: {...} }` | `{ field: <dotted path>, <op>: <constant> }`, exactly one of `equals`, `not_equals`, `lt`, `le`, `gt`, `ge`; upper snake case is a message constant name |
| `window` | mode | `10s` (unbound) or `{ duration, param: <node>.<parameter> }` |
| `exit` | windowed mode only | `{ on: <function>, to: <mode> }` |
| `entry_speed` | hazard | m/s, positive |
| `settle` | safe_state | a duration, or `{ decel: <param>, jerk: <param> }` with an optional measured `duration:` beside it |

Every misuse is a parse error naming the key. **An exit on a rung with no
window is a parse error too**, not a checker rule: both keys sit in one
mapping, the grammar is closed, and an exit anywhere else would be a general
transition the grammar deliberately does not have. The manifest-parse error
now carries the contract's line.

On a function, `when:` is the predicate that **loses** it, so a function that
should HOLD on a value is written with the negated operator:
`driver_took_over` is `when: { field: mode, not_equals: MANUAL }`. Brief D's
sketch wrote `equals: MANUAL` there, which under the loss reading means the
opposite of what it wanted; see "What the brief got wrong".

### The rules (`value_rules.rs`, and `check_fault_reaction`)

Every new message starts with the contract file and line.

| rule | severity | example line |
|---|---|---|
| `when-requires-reported` | error | `bringup.contract.yaml:53: hazard 'hpc_loss' names a value fault (stop == true) with on: [omission]` |
| `when-field-unknown` | error / warning | `... function 'typo_field' reads field 'autonomus' of '/availability', and demo_msgs/msg/Availability has no such field (it has stamp, stop, autonomous, comfortable_stop)`; a warning when the `.msg` is not on `AMENT_PREFIX_PATH` |
| `ladder-window-floor` | error | `... mode 'stopping' falls to 'parked' last, and 'parked' has a 30000ms window -- a floor you leave after a timeout is not a floor` |
| `window-param` | error | `... mode 'takeover' waits 10000.00ms, bound to handler.takeover_timeout, which resolves to 12.0 s = 12000.00ms` |
| `window-unbound` | warning | `... mode 'takeover' waits 10000ms, but the window names no param:` |
| `mode-exit-target` | error | `... mode 'takeover' exits to 'estop', which is a rung of the fallback: ladder of 'engaged'` |
| `mode-exit-unwired` | error | `... mode 'hold' exits on 'driver_took_over' (/control_mode), but no node that implements the rung (/holder) takes it on a path trigger` |
| `ladder-rung-budget` | error | now cumulative: `detection 120.00ms + 'takeover' reaction 110.00ms + window 10000.00ms + reaction 410.00ms + settle 9996.67ms = 20636.67ms against 20000.00ms` |
| `reaction-unbudgeted` | warning | now also on a graded rung the walk reaches with no settle; a **windowed** rung is exempt (its sink is a notification) |
| `settle-derived` | info | `v_r = a^2/(2j) = 2.5^2/(2*1.5) = 2.0833 m/s; v0 = 3 > v_r, so t = a/j + (v0 - v_r)/a = 2.5/1.5 + (3 - 2.0833)/2.5 = 1666.67 + 366.67 = 2033.33ms` |
| `settle-entry-missing` | warning | `hazard 'odd_exit_unbounded' reaches rung 'comfortable', whose settle is a braking profile ..., but the hazard states no entry_speed -- falling back to the literal settle 5000.00ms` |
| `settle-param-unresolved` | error | `the braking profile of /estop_op/on_timer cannot be read: 'target_decel' is not declared under nodes.estop_op.params` |
| `settle-conflict` | warning | `the literal settle 5000.00ms and the profile's 9996.67ms differ by 50.0%` |

The arithmetic, with a = |decel| and j = |jerk|: `v_r = a^2 / (2 j)`;
`t = sqrt(2 v0 / j)` if `v0 <= v_r`, else `t = a/j + (v0 - v_r)/a`. At
a = 2.5, j = 1.5 it gives 2033.33 ms from 3.0 m/s and 2525.33 ms from
4.23 m/s, the island's hand number and its phase-7 miss. The one copy is
`SettleProfile::settle_s` in rlm; the observer's model view calls it too.

`window-param` and the settle rules read **launch parameter values**, which
the checker never had: `ManifestIndex.launch_params` is built from the launch
dump in ROS's precedence (the ordered source list, globals first; files then
inline values otherwise), with `param_file_values` matching sections, so the
number checked is the number a spawn applies.

`when-field-unknown` resolves the topic's `type:` to a `.msg` through
`AMENT_PREFIX_PATH` (`<prefix>/share/<pkg>/msg/<Name>.msg`), follows a dotted
field through nested messages, and checks a constant name against the message
that declares the field. Layer 2 still needs no ROS: an unresolvable type is
a warning, "the field is UNCHECKED".

### The fixes

- **F1, ladder selection per hazard** (#0057). `value_rules::removed_by` is
  the set of functions a hazard's fault takes away: a `reported` fault removes
  the value functions on its topic, an omission, late or loss fault removes
  both kinds. A rung that requires one is skipped and **not charged its
  window**; `ladder-unterminated` uses the same set for the floor. On the demo
  contract `hpc_loss` skips the takeover request and the comfortable stop and
  lands on the floor in 4808.67 ms against 10 s; charged, the comfortable stop
  alone would have read 500 + 410 + 9996.67 = 10906.67 ms and failed a rung
  the fault makes unreachable.
  **It reverses phase 75's Autoware headline.** On the Autoware fixture
  (`tests/fixtures/autoware/contracts/tier4_system_launch`), phase 75 reported
  `comfortable_stop` unable to make the 2 s interval of `mode_unavailable`.
  That rung requires `operation_mode_availability`, the very function the
  omission removes, and `mrm_handler` agrees: an availability timeout forces
  EMERGENCY_STOP. The rung is now skipped and `autoware.rs` asserts that. The
  same holds for `contract_modes_bad`'s `degraded`, which now requires
  nothing the fault removes so that `ladder-rung-budget` still has a rung to
  fail on.
- **F2, an off-host sink is observed at its first host take** (#0058). In the
  live observer (`runtime_enforcement`) and in `measure`: a sink no host
  process publishes (declared `external: pub`, or never seen published) is
  judged by its host `Take`s, and the reaction is labelled
  `"observed_at":"take"` in `runtime_violations.jsonl` and in the message ("the
  link hop is inside the number"). A sink a host process does publish is never
  judged by its takes.
- **F3, verified: the walk continues through a service callback that
  publishes.** A server path triggered by its `srv:` endpoint is taken by
  `from_service` like any input-triggered path; on the demo contract the
  comfortable-stop route is `call_mrm` 110 ms, the operator's `operate`
  callback (publishes the velocity limit), the planner 300 ms: 410 ms. No code
  change was needed. What was needed is below.
- **A safe state commanded upstream keeps its settle.** The comfortable-stop
  operator commands the stop by publishing a velocity limit; the planner
  carries it to the actuator. The walk used to take a settle only from the
  path that publishes the sink, so the operator's profile was dropped at the
  planner hop. The nearest `safe_state` upstream on the branch now supplies it
  unless a hop nearer the sink declares one.
- **`wiring` (rlm)**: a path output naming a service endpoint is wired by the
  service; the island's own contract warned on every run.

### `check --explain`

Without a scheduling platform file `--explain` used to print a note and
nothing else. It now prints the fault-reaction arithmetic, one row per
(hazard, rung), with skipped rungs and why:

```
-- Fault-reaction budgets (--explain, ms) --
HAZARD    RUNG              ROLE     DETECT   WINDOWS   ROUTE           SETTLE     TOTAL      FTTI     SLACK
hpc_loss  takeover_request  skipped       -         -       -                -         -  10000.00         -
hpc_loss  comfortable_stop  skipped       -         -       -                -         -  10000.00         -
hpc_loss  emergency_stop    floor    500.00      0.00  143.33  4165.33 derived   4808.67  10000.00   5191.33
odd_exit  takeover_request  window   120.00      0.00  110.00  window 10000.00         -  30000.00         -
odd_exit  comfortable_stop  rung     120.00  10110.00  410.00  9996.67 derived  20636.67  30000.00   9363.33
odd_exit  emergency_stop    floor    120.00  10110.00  143.33  4165.33 derived  14538.67  30000.00  15461.33
```

That is the island's demo contract (`demo/l3/contracts/l3_takeover.*` there,
ODD bound 30 km/h). Its README quotes all three runs verbatim.

## Fixtures and tests

| fixture | what it proves |
|---|---|
| `tests/fixtures/contract_takeover/` | the four keys on a ladder that fits; F1 visible in `--explain`; F3's route; 2033.33 ms derived at 3.0 m/s; ships `ament/share/demo_msgs/msg/*.msg` so `when-field-unknown` resolves without Autoware |
| `contract_takeover_bad/` | `when-requires-reported`, `when-field-unknown` (no such field, unknown constant, not a scalar, unresolvable type), `ladder-window-floor`, `mode-exit-target`, `mode-exit-unwired`, `window-unbound` |
| `contract_takeover_budget/` | `window-param`, cumulative `ladder-rung-budget`, `settle-conflict`, `settle-entry-missing`, `settle-param-unresolved` (reported once, not once per hazard), `reaction-unbudgeted`, and the 4.23 m/s floor missing 3 s (3168.67 ms) |
| `contract_takeover_exit/` | an exit with no window refuses the file, at its line |

`tests/tests/manifest_check.rs`: four tests over those. Unit tests:
`runtime_enforcement` (an off-host sink is observed at its first take; a
host-published sink is never judged by its takes), `interception::measure`
(the same pair offline). The ranked-plan snapshot gains the four fixtures;
every existing plan is byte-identical.

## What the brief got wrong

- **The failing variants do not fail at the decided speed.** Brief D computed
  "a 12 s window" and "a speed bound 10 km/h higher" at 16.7 m/s (60 km/h),
  where the comfortable rung had 993 ms of slack. The user's decision is
  30 km/h, which leaves it 9363.33 ms: a 12 s window reads 22636.67 ms and a
  40 km/h bound 23416.67 ms, both inside 30 s. The smallest one-line breaks
  are a window over 19.36 s and a bound over 63.7 km/h; the island's variants
  use 20 s and 65 km/h.
- **`hpc_loss` cannot keep 3 s at 30 km/h.** The emergency stop alone takes
  4165.33 ms to standstill from 8.33 m/s; the demo declares 10 s.
- **`driver_took_over` had the wrong sign** under the one reading that makes
  functions consistent (above).
- **F3 was not a gap; the settle carry was.** The walk already crossed a
  srv-triggered path; what it lost was the operator's settle one hop later.
- **`window-param` and the settle rules need launch parameter values**, which
  the checker did not have; the brief assumed "resolved launch value" was at
  hand.

## Not in this phase

- The observer's `mode-window` and `mode-exit` events and the contract probe
  that evaluates `when:` on payloads (the island's W7). The model carries
  every key the probe needs (`FunctionContract.when`, `HazardContract.when`,
  `ModeContract.window/exit`).
- `check --budgets <file.json>`: the rows above as data for the timeline
  (W7). `ManifestIndex.budgets` is the source.
- A `link-budget` rule (rate x serialized bound against a declared link
  capacity). Optional in the brief; not started.
