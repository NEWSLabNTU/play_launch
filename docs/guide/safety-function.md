# A safety function in a contract

A safety function watches for a hazard and drives the system to a safe
state in time. This guide builds one for a mobile robot's protective stop,
one piece at a time, and shows what `play_launch check` says at each step
and what the running system is checked for. Every contract here is a test
fixture under `tests/fixtures/`, so each output quoted below is what the
checker prints today.

The robot is the shape of Safe-ROS (FMAS 2025): a LiDAR, a safety node that
publishes a too-close flag, a planner that publishes velocity commands, and
an interceptor that forwards the commands and replaces them with a stop
while the flag is set.

## 1. The hazard and its reaction

A hazard names what is watched, the fault classes that matter, and the time
the system has: the fault-tolerant time interval (FTTI). Its reaction is an
event chain that ends at the safe state.

```yaml
hazards:
  collision:
    integrity: ASIL_B
    requirement: { id: R1, source: "HARA rev 3, section 4.2" }
    monitors: [/too_close]
    on: reported
    when: { field: data, equals: true }
    ftti: 1s
    reaction: safety.stop
```

`requirement` names the requirement of your hazard analysis this hazard
implements. The checker adds `[requirement R1]` to every finding about the
hazard, so a finding can be traced back without opening the contract, and
`check --explain` states the hazard as one sentence:

```text
-- Hazards (--explain) --
  hazard 'collision' implements R1 (HARA rev 3, section 4.2) at ASIL_B: it
  watches /too_close for reported (when data == true); the system must reach
  a safe state within 1000.00ms; it enters mode 'driving', reaching 'stopped'
  (minimal risk condition) in 407.00ms. At 'stopped', /cmd_vel_filtered must
  carry linear.x == 0 and angular.z == 0.
```

## 2. A reaction that waits on traffic

Safe-ROS's interceptor publishes only from its command callback. The flag
is read as state, so the stop reaches the base only when the planner's next
command arrives, and never if the planner stops. Written as it is
(`contract_safe_ros`), the checker refuses it:

```text
error[reaction-waits-on-traffic]: hazard 'collision': /cmd_vel_interceptor
reads the monitored too_close as state, and its path 'gate' that reaches the
reaction's output is triggered by '/cmd_vel_in' (up to 100.00ms at its 10
Hz, and never if it stops) -- the reaction goes out only when that traffic
next arrives, which nothing bounds. Trigger a path on the monitored topic,
or on a timer
```

The fix is a path triggered by the report itself (`contract_safe_ros_fixed`):

```yaml
  cmd_vel_interceptor:
    sub:
      cmd_vel_in: {}
      too_close:
        on_violation: { on: [reported], reaction: stop }
    paths:
      gate:
        trigger: { input: [cmd_vel_in] }
        output: [cmd_vel]
      stop:
        trigger: { input: [too_close] }
        output: [cmd_vel]
        safe_state: { emits: cmd_vel, settle: 300ms }
```

## 3. What the safe state commands

A reaction that reaches the actuator topic is not yet a safe state: the
checker also needs to know what the level commands there. `commands` on a
mode states it as predicates over the message's fields
(`contract_commands`):

```yaml
modes:
  driving:
    requires: [path_clear]
    fallback: [stopped]
  stopped:
    requires: []
    reaction: safety.stop
    commands:
      /cmd_vel_filtered:
        - { field: linear.x, equals: 0 }
        - { field: angular.z, equals: 0 }
```

A commanded topic is a final element. Three static rules follow from the
graph, with no further key:

| Rule | Fires when | Fixture |
|---|---|---|
| `arbiter-bypass` | the topic does not have exactly one publisher, so a command can reach it around the level's reaction | `contract_commands_bypass` |
| `commands-unreached` | the level's reaction neither ends on the topic nor upstream of it | `contract_commands_unreached` |
| `commands-field-unknown` | a predicate names a field the message type does not have | `contract_commands_field` |

```text
error[arbiter-bypass]: mode 'stopped' commands '/cmd_vel_filtered', which
has 2 publishers (/gate, /teleop) -- a final element has exactly one, its
arbiter, or a command can reach it around the level's reaction
```

At run time (`play_launch launch`, with interception), from the moment the
level is entered plus the time the checker derived for its reaction, every
message on the commanded topic must satisfy the predicates, until the mode
above it is available again:

| Runtime rule | Fires when |
|---|---|
| `safe-state-violated` | a message on a commanded topic breaks a predicate after the level's bound: the gate sent one stop and went back to forwarding |
| `final-element-silent` | the commanded topic goes silent, and its consumer does not say what it does then (`topics.<t>.external: { side: sub, on_silence: { after: 500ms } }`) |
| `arbiter-bypass` | a second publisher is created on a commanded topic |
| `safe-state-unjudged` (warning) | a level commands a topic no hazard's reaction reaches, so there is no bound to judge from |

The fields are read in each publishing process by the interceptor, about
50 ns per field per publish. A predicate on a message constant name
(`MANUAL`) is checked statically but not at run time, since the model
carries the name and not its value.

## 4. Sizing the protective field

The checker has now proved the stop goes out within 407 ms of the report.
Whether the robot then stops before it hits anything depends on the
protective field, the speed and the brakes, which is physics, not timing.
That belongs in an extension. `mobile.protective_stop` is a reference
extension (rlm `types/extensions/reference/protective_stop.rlx`; copy it
into your package's `rlm/` directory). It checks

```text
field >= speed * reaction_total + speed^2 / (2 decel) + margin
```

where `reaction_total` is the time the checker charged for the reaction.
Read the field from the node that applies it, so the check judges the
number the node runs (`contract_protective_stop`):

```yaml
use:
  iso26262: "^1"
  mobile.protective_stop: "^0.1"

hazards:
  collision:
    ...
    deadline_by: mobile.protective_stop/stopping_distance
    speed: 1.0 m/s
    decel: 1.5 m/s^2
    margin: 0.1 m
    field: $(param /safety_node stop_distance) m
```

Safe-ROS stops at 5 cm:

```text
error[extension-check]: hazard 'collision': mobile.protective_stop/stopping_distance FAIL
driven = speed * reaction_total = 407 mm
braking = speed^2 / (2 decel) = 333.33 mm
required = driven + braking + margin = 840.33 mm
stopping_distance: required <= field: 840.33 mm <= 50 mm (margin -790.33 mm) FAIL
```

With `stop_distance` set to 0.9 m in the launch file
(`contract_protective_stop_sized`), the check passes, and the prover
confirms the worst case is at the stated speed:
`worst case: proved increasing in reaction_total, speed, margin`. Fields
that change with speed are degradation levels, one hazard and one check
each. The extension is illustrative: its shape follows ISO 3691-4 and ISO
13855, and no clause has been checked against the standards' text.

## 5. Watching the safety function itself

A safety node that hangs reports nothing, and a `reported` hazard then
never fires. The checker says so where a reaction ends with no subscriber
watching it:

```text
warning[reaction-unguarded]: hazard 'collision' reaction 'safety.stop' ends
at /cmd_vel and no subscriber there declares an `on_violation` -- a stalled
reaction would go unnoticed. Monitor it with a second hazard whose
`monitors:` name this output
```

The answer is a second hazard, not a new key: one whose `monitors:` is the
safety node's own output, with `on: omission`, so the safety node's silence
is itself a fault with a reaction.

## Simplex, as a pattern

Simplex runs an unverified advanced controller beside a verified safety
controller, and a decision module forwards the advanced command only while
a monitor says it keeps the system inside an envelope the safety controller
can recover from. It needs no new key (`contract_simplex`):

| Simplex part | Contract |
|---|---|
| the monitor's verdict | a value function the advanced mode requires (`functions.envelope: { of, when }`) |
| switching to the safety controller | the safe mode, the advanced mode's `fallback`; its reaction is the decision module's switch, triggered by the verdict |
| what the safety controller guarantees | the safe mode's `commands` on the one topic the decision module publishes |
| the safety side does not fail with the advanced one | `independence: [{ of: [advanced_ctrl, safety_ctrl], not_sharing: [process, container] }]` |

```yaml
functions:
  envelope:
    of: [/envelope_ok]
    when: { field: data, equals: false }

modes:
  advanced:
    requires: [envelope]
    fallback: [safe]
  safe:
    requires: []
    reaction: switch_to_safe
    commands:
      /cmd:
        - { field: linear.x, le: 0.5 }
```

The checker derives the switch time from the monitor's rate and the
decision module's path (`23.00ms + 101.00ms = 124.00ms` against a 200 ms
FTTI). At run time the advanced mode is lost when the monitor's latest
sample says `false`, the safe mode is entered, and from its bound every
`/cmd` must carry `linear.x <= 0.5`. The envelope itself, the region the
safety controller can recover from, is the monitor's argument, not the
contract's: the contract states the verdict and checks what follows from
it.

## Related

- `docs/guide/fault-tolerance.md` — redundancy, restarts and independence,
  for a system where most nodes are safety critical.
- `docs/guide/runtime-enforcement.md` — every runtime rule and its output.
- `docs/roadmap/phase-100-safety-functions.md` and
  `docs/roadmap/phase-102-safety-model-build-and-evaluation.md` — how this
  was built and evaluated.
- rlm `docs/design/safety-functions.md` — the design, and why Safe-ROS's
  shape was the starting point.
