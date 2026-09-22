# Runtime contract enforcement

`play_launch check` grades a contract against the launch file — the topics,
the endpoints, the arithmetic over declared rates and budgets. It cannot tell
you whether the system that came up actually publishes at 100 Hz, whether the
message on the wire is the type the contract names, or how long after a lidar
went quiet the brake command appeared. `--enforce-rules` does: it watches the
running system and reports each declared claim the run breaks, and in
`strict` it ends the run when one does.

It sees the run through **message interception**. `libplay_launch_interception.so`
is `LD_PRELOAD`ed into every process play_launch spawns and hooks four rcl
calls — `rcl_publisher_init`, `rcl_subscription_init`, `rcl_publish` and
`rcl_take` — writing a fixed-size record per event into a lock-free SPSC ring
in shared memory (one ring per child, `memfd` + `eventfd`), which a consumer
task in play_launch drains every 10 ms and feeds to the rule engine. That is
the engine's **only** event source: no interception, no rules, no matter what
`--enforce-rules` says. Everything in the "Preconditions" section below exists
because that sentence used to be true silently.

## Getting a violation on screen

The fixture is two `demo_nodes_cpp` nodes under `/pure_test`,
`tests/fixtures/simple_test/launch/pure_nodes.launch.xml`; the talker
publishes `/pure_test/chatter` at about 1 Hz. Write a contract that demands
more than that.

Contracts arrive through the overlay channel, a tree mirroring the launch
tree: `<overlay>/<pkg>/launch/<stem>.contract.yaml`. A launch file given as a
raw path resolves to the package name `_`, so this one goes at
`contracts/_/launch/pure_nodes.contract.yaml`:

```yaml
version: 1
nodes:
  talker:
    pub:
      chatter:
        min_rate_hz: 1000
  listener:
    sub:
      chatter: {}
topics:
  /pure_test/chatter:
    type: std_msgs/msg/String
    pub: [talker/chatter]
    sub: [listener/chatter]
```

```bash
play_launch launch \
  --contracts ./contracts \
  --enforce-rules warn \
  tests/fixtures/simple_test/launch/pure_nodes.launch.xml
```

Two lines matter. The first is printed before anything spawns and says where
the events are coming from:

```
INFO  Interception: on (implied by --enforce-rules warn)
```

The second lands about six seconds in — the rate rule needs half a second of
publishing to estimate a rate, and the talker's own first message takes a
while:

```
WARN  [runtime] topic '/pure_test/chatter' publisher '/pure_test/talker/chatter'
      measured rate 1.20 Hz < declared min_rate_hz (1000) (rate-hierarchy-runtime)
```

and in the bundle, `play_log/<ts>/runtime_violations.jsonl`:

```json
{"rule_id":"rate-hierarchy-runtime","severity":"error","fqn":"/pure_test/chatter","message":"topic '/pure_test/chatter' publisher '/pure_test/talker/chatter' measured rate 1.20 Hz < declared min_rate_hz (1000)","timestamp_ns":2174550538351001}
```

Run the same thing with `--enforce-rules strict` and it stops being a report:

```
WARN  [runtime] topic '/pure_test/chatter' publisher ... (rate-hierarchy-runtime)
WARN  [runtime] Strict enforcement violated (rate-hierarchy-runtime on /pure_test/chatter)
      — initiating shutdown
Error: --enforce-rules strict: runtime contract violation rate-hierarchy-runtime
       on /pure_test/chatter ended the run
```

Exit status 1, six seconds after startup, nodes gone. That is the CI shape:
the transcript names the rule and the topic, so a red build says what it
measured.

The same works from a resolved model, because the model carries the contracts
the resolve saw:

```bash
play_launch resolve --contracts ./contracts -o system_model.yaml \
  tests/fixtures/simple_test/launch/pure_nodes.launch.xml
play_launch up --enforce-rules strict system_model.yaml
```

Enforcement on `up` reads `contracts:` out of the model — a model resolved
without `--contracts` carries none, and the engine then has nothing to
enforce but the structure (see `graph-deviation-runtime` below).

## The modes

`--enforce-rules <mode>`, default `warn`.

| mode | rules | `runtime_violations.jsonl` | exit |
|---|---|---|---|
| `off` | not evaluated; no engine is built | not written | unaffected |
| `warn` (default) | evaluated, each violation logged at `warn!` | written | unaffected — a violating run still exits 0 |
| `strict` | evaluated and logged | written | **non-zero** on the first violation at or above `error`, naming rule and FQN |
| `record-only` | evaluated and logged | **not written** | unaffected |

`record-only` is the odd one. Its purpose is to collect
`interception/events.jsonl` for an offline pass (`play_launch measure`, the
Chrome trace, `scripts/verify_graph.py`) without producing a violations
artifact — but it does build the engine and does evaluate the rules, so you
will still see `[runtime]` lines on the terminal. Only the JSONL is withheld.

`strict` is the CI mode, and it is the only one that can fail a pipeline.
`warn` is the mode to develop against: a violating run still exits 0, so the
first read of a new contract is a list rather than a stop.

## Preconditions: the mode has an event source, or says it has none

The engine watches interception and nothing else, so the two settings have to
agree. They are resolved in one place, by precedence, highest first:

1. `--interception on|off` on the command line
2. an explicit `interception: enabled:` in the `--config` YAML
3. implied **on** when `--enforce-rules` is anything but `off`
4. otherwise off

The startup `info` line reports the answer *and the decider*, because those
are different questions — one tells you whether rules will run, the other
tells you which file to edit:

```
INFO  Interception: on (implied by --enforce-rules warn)
INFO  Interception: off (--interception off)
INFO  Interception: off (default)
```

A non-`off` mode can still end up with no event source two ways, and neither
is allowed to be quiet.

**You switched interception off deliberately.** Under `warn` or `record-only`
this warns, once, before spawning:

```
WARN  --enforce-rules warn has no event source: interception is disabled by
      --interception off. The runtime rule engine observes ONLY the LD_PRELOAD
      interception layer, so no rule will be evaluated and no
      runtime_violations.jsonl will be written. Pass `--interception on` (or drop
      the explicit disable) to enable it, or `--enforce-rules off` to stop asking
      for enforcement.
```

Under `strict` the same text is an **error before the first spawn** — exit
non-zero, no `play_log` bundle, no node started. A gate that exits 0 having
measured nothing is worse than one that fails.

**The library is not on disk.** `libplay_launch_interception.so` is a separate
build artifact (`just build-interception`; the wheel ships it in `lib/`).
play_launch looks at `$PLAY_LAUNCH_INTERCEPTION_SO`, then `<bin>/../lib/`,
then the in-tree development paths. Not finding it gets the same ruling: a
warning under `warn`, a refusal before spawn under `strict`, naming the
library rather than just the mode.

Interception is not free, and turning a mode on is what asks for it: about
150 ns per publish or take (0.15% of a core at 10k msg/s, measured in phase
58 W2 — `CLOCK_THREAD_CPUTIME_ID` at 85 ns against `CLOCK_MONOTONIC`'s 17).
It also writes `interception/events.jsonl`, one ~110-byte line per message,
which is on by default; `interception: { events: false }` in a `--config`
file turns that file off for a long run on a high-rate system without
disabling enforcement.

## What `strict` actually does

**It trips at `error`, not at `warning`.** The engine writes two severities,
and they mean different things: `error` is a claim the contract made and the
run broke; `warning` is something the tool noticed but cannot settle from the
launch tree alone. `graph-deviation-runtime` is warn-only by design — a
running graph is always a superset of what a launch file declares — so a gate
that tripped on warnings tripped 57 ms into every run, before any rule with a
measurement window had a chance to evaluate. Warnings keep their log line and
their JSONL record; what they no longer do is end a run. (The threshold is a
seam in the engine, not a flag: there is no `--strict-on` to lower it today.)

**The three rcl-internal topics are implicitly external.** Every ROS 2 node
creates `/rosout` and `/parameter_events` publishers before a line of user
code runs, and no author writes a requirement about them, so the runtime view
seeds `/rosout`, `/rosout_agg` and `/parameter_events` as known topics and
`graph-deviation-runtime` never fires on them. This is a seeding of the
*runtime* view only — the contracts' own `external_topics:` set is untouched,
and the static checker does not know about it.

**The teardown is a real teardown.** On the first violation at or above the
threshold the watcher calls the same `initiate_shutdown` the SIGTERM path and
`--on-startup-failure exit` call: SIGTERM to the process group, the shutdown
broadcast, and the member handle's shutdown, in that order. It used to send
only the broadcast, which left every actor waiting on a child nobody had
signalled — the supervisor stayed alive with its nodes still running until an
outside SIGINT arrived.

**The exit message names the violation.** `play()` returns an error built from
the first violation that crossed the threshold, recorded before the flag is
published so the watcher can never see a trip with no reason beside it:

```
Error: --enforce-rules strict: runtime contract violation <rule-id> on <fqn> ended the run
```

Checked before `--on-startup-failure`'s own check, because a member that
failed during a strict teardown is a consequence of the teardown, not the
cause.

## The rules

Thirteen rule ids. "Contract field" is what a rule reads to have an opinion;
a rule with none is comparing the running system against itself.

| rule id | severity | reads | fires when |
|---|---|---|---|
| `graph-deviation-runtime` | warning | the model's topic set — `structure.topics` (declared `topics:` plus topics derived from the launch file's own remaps) and `contracts.externals` | a publisher or subscription is created on a topic that set does not contain. The three rcl-internal topics are exempt |
| `consistency-runtime` | error | `topics.<t>.type` | the runtime message type (from rosidl introspection) disagrees with the declared one |
| `qos-match-runtime` | error | — (observed QoS on both sides) | an observed publisher's reliability or durability cannot satisfy an observed subscriber's, at endpoint creation; and on every DDS `OFFERED_QOS_INCOMPATIBLE` / `REQUESTED_QOS_INCOMPATIBLE` |
| `rate-hierarchy-runtime` | error | `nodes.<n>.pub.<e>.min_rate_hz` | the measured publish rate falls below the declared floor. Needs ≥0.5 s of publishing; re-checked every 1024 publishes and at least every 5 s, so a 1 Hz topic cannot escape its contract by never reaching a count |
| `max-age-runtime` | error | `nodes.<n>.sub.<e>.max_age` | `now − header.stamp` at a take exceeds the declared age. Read on the consumer's poll, so it can overstate age by up to the 10 ms poll interval |
| `drop-rate-runtime` | error / warning | `topics.<t>.drop.max_count` | error: the worst subscriber's measured `(published − taken) / published` exceeds the declared budget, checked every 256 takes. warning: a DDS `MESSAGE_LOST` event |
| `max-latency-runtime` | error | `paths.<p>.max_latency` (scope path) | the same `header.stamp` reaches the path's output topic later than the budget allows after its publish on the input topic |
| `deadline-runtime` | warning | `qos.deadline` (applied to the node as a `qos_overrides.*` parameter — phase 74) | DDS reports a missed deadline on either side |
| `liveliness-runtime` | error / warning | `qos.liveliness`, `qos.lease_duration` | error: DDS `LIVELINESS_LOST` on a publisher. warning: `LIVELINESS_CHANGED` on a subscription — the topic's publisher set changed shape |
| `hazard-detected` | error | `hazards.<h>.guards`; threshold from the reacting subscriber's `sub.<e>.max_age` or `qos.lease_duration` | a guard topic goes silent past the threshold — or DDS reports it lost liveliness or missed its deadline first. Absent a declared detector the threshold is 10× the guard's own median period, floor 50 ms. Silence produces no event, so it is judged on the consumer's 10 ms tick |
| `hazard-reaction` | warning / error | `hazards.<h>.ftti`, `paths.<p>.safe_state.settle` | the first publish on the reaction path's output topic that carries no provenance from the dead guard. Warning when `reaction + settle` fits the FTTI (with the slack named), **error** when it exceeds it. The message also reports when the observer itself noticed, which is usually later than the node's own watchdog |
| `hazard-recovered` | warning | — | the guard resumes publishing, with how long it was out |
| `mode-availability` | warning / error | `modes.<m>.requires` / `fallback`, `functions.<f>` | a mode becomes unavailable because a function it requires has a silent guard. Warning naming the rung it falls to, **error** when no rung of the ladder is available either. Reported once per transition, not once per event |

Three things about the table worth knowing before you read output against it.

**Not every FQN is a topic.** For the `hazard-*` and `mode-availability`
rows, the `fqn` field carries the hazard or mode key.

**Some rules repeat and some do not.** The measurement rules are deduplicated
by `(rule_id, fqn)` — a talker that is slow stays one line however long the
run. The DDS-sourced rules and the hazard/mode rules are not, because each
emission represents new occurrences the middleware counted or a new
transition in the system.

**Lifecycle nodes are gated.** `rate-hierarchy-runtime` and `max-age-runtime`
skip an endpoint whose node declares `lifecycle: true` unless a transition
event has been observed putting it in `Active`. Ordinary nodes are always
enforceable.

## `runtime_violations.jsonl`

`play_log/<ts>/runtime_violations.jsonl`, one JSON object per line, flushed on
write so a killed run still leaves a readable file. Written in `warn` and
`strict` only; created lazily, so **no file means no violation** rather than
nothing having run. Five fields:

| field | type | meaning |
|---|---|---|
| `rule_id` | string | one of the thirteen above |
| `severity` | `"warning"` \| `"error"` | `error` is what `strict` ends a run on |
| `fqn` | string | the topic FQN, or the hazard/mode key for the `hazard-*` and `mode-availability` rules |
| `message` | string | the rendered violation, with the measured value and the declared one |
| `timestamp_ns` | integer | `CLOCK_MONOTONIC` ns, taken from the interception event where one exists — the same clock as `interception/events.jsonl`, so the two join |

The engine also writes `play_log/<ts>/discovered_topic_types.tsv` on
shutdown — every `(topic, message type)` pair seen at runtime, whether or not
a contract mentioned it. It is the fastest way to backfill `type:` fields
while writing a first contract for an existing system.

## Limits

**`--enforce-rules` is inert on `run`.** `play_launch run <pkg> <exec>`
accepts the flag and does nothing with it: that path never sets up
interception and never builds a rule engine, so the precondition check does
not run either. `play_launch run --enforce-rules strict --interception off`
starts the node without a word — no `Interception:` line, no refusal, and a
bundle with no `interception/` directory and no violations file. Use `launch`
or `up`.

**The engine sees only what crosses rcl in a process play_launch spawned.**
A node you started by hand in another terminal, a node inside a container
play_launch did not fork, anything that talks over a raw DDS API rather than
rcl, and any message that never reaches `rcl_publish`/`rcl_take` are all
invisible. This is why `graph-deviation-runtime` is a warning: absence of an
endpoint in the interception stream is not proof the endpoint does not exist.

**A latency or age rule needs a `header.stamp`.** `max-age-runtime` and
`max-latency-runtime` read the stamp out of the message; a message type
without a `std_msgs/Header` is counted but never timed.

**A rate rule needs a window.** Nothing fires in the first half second of a
topic's life, by construction — a measured rate over two messages is not a
rate.

**Interception costs what it costs.** ~150 ns per message, plus
`events.jsonl` growing with traffic while `interception.events` is on.

## Adjacent: blocking instead of reporting

`--block-unauthorized-endpoints` is the other half of
`graph-deviation-runtime`: instead of reporting an endpoint on an undeclared
topic, the `.so`'s `rcl_publisher_init` / `rcl_subscription_init` hooks refuse
to create it (`RCL_RET_TOPIC_INVALID`). The allowlist is written to
`play_log/<ts>/expected_graph.txt` from the merged contracts and handed to
every child. It is off by default and should stay off unless you know the
nodes handle an init failure — many crash. If no contract declares any topic
the feature disables itself with a warning, since an empty allowlist would
refuse every endpoint in the system.

## See also

- `docs/design/fault-reaction-primitives.md` — where `hazards:`,
  `on_violation:` and `safe_state:` come from, and the FDTI/FRTI arithmetic
  the `hazard-*` rules check live
- `docs/design/operational-modes.md` — `functions:`, `modes:` and the
  fallback ladder `mode-availability` walks
- `docs/roadmap/phase-36-runtime_enforcement.md` — the rule engine's origin
- `docs/roadmap/phase-79-runtime-enforcement-is-a-gate.md` — why the
  precondition, the severity threshold and the teardown are as described here
- `docs/guide/rt-scheduling.md` — the other consumer of the same contracts
