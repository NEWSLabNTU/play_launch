# Getting a first contract for a system that has none

A launch manifest states what a system must achieve. Writing the first one for
an existing stack is mostly transcription: naming the nodes, their endpoints,
the topics between them and the message types — before any requirement gets
written down at all. On a 144-node stack that transcription is the part that
stops people, and it is also the part a machine can do, because a run already
knows every one of those facts.

`play_launch contract capture` does it. Point it at a recorded run and a
SystemModel and it prints a contract file to stdout.

```sh
# 1. run the system once, with interception on
cat > interception.yaml <<'EOF'
interception:
  enabled: true
  stats: true
EOF
play_launch launch --config interception.yaml my_pkg bringup.launch.xml

# 2. resolve the model that run came from
play_launch resolve my_pkg bringup.launch.xml -o system_model.yaml

# 3. capture
play_launch contract capture play_log/latest --model system_model.yaml \
    > contracts/my_pkg/launch/bringup.contract.yaml

# 4. check it — this should be clean before you edit a line of it
play_launch check my_pkg bringup.launch.xml --contracts contracts
```

Step 4 is not a formality. A capture that produced a contract its own run
violates would be lying about one of the two, so the emitted file is meant to
pass `check` with no errors as it stands. If it does not, that is a defect in
the verb.

It prints to stdout and writes nothing, for the same reason
[`play_launch measure`](rt-scheduling.md) never writes the platform file:
where a contract lives, and whether it replaces the one already there, is the
author's decision.

The model is required for the same reason `measure` requires one. A contract's
node keys are reconciled against the launch dump, so they have to be keys the
dump knows — and for a node the launch file did not name, that key is *not* the
node's ROS name (issue #0017). Only the model can tell the two apart.

## What it emits, and what it refuses to

**It emits structure and facts**: `nodes:` with their `pub:`/`sub:` endpoints,
and `topics:` with the message type and both sides of the wiring.

**It emits no requirements.** No `rate_hz`, no `min_rate_hz`, no
`max_latency`, no `max_age`, no `max_jitter`, no `paths:`, no `criticality`.
Not because they are hard to guess but because they are not observable. A run
can tell you a topic published at 9.97 Hz; it cannot tell you whether 10 Hz was
*required*, and that difference is the whole of what a contract is for. The
observed numbers are there — as comments, on the topic they belong to, so you
can turn one into a requirement by deleting a `#` and deciding. This is the
same discipline `play_launch measure` follows when it prints measured costs
under a header saying where they belong.

**It emits no conditions.** A run is *one branch* of the launch file, with
every `if:` and `unless:` already resolved and nothing on disk recording which
way each went. A contract should mirror the launch file's structure, conditions
included; a capture cannot recover them, and the emitted header says so. Launch
the same file with different arguments and you get a different capture. Before
reading a diff between a capture and a hand-written contract as a
disagreement, account for that.

**It refuses, loudly, rather than guessing.** Everything it could not carry is
listed in a comment block at the end of the file, with the reason:

| refusal | why |
|---|---|
| a topic whose message **type** is not on disk | the grammar requires `type:` on every topic. Since issue #0047 this is rare — see below |
| endpoints on a node the model does not carry | a process play_launch spawned can host nodes the launch file never named — a container's own node, an rclcpp-internal one. A contract may not claim one |
| a subscriber cut to break a causal cycle | `causal-dag` refuses a cyclic dataflow graph. See below |

A topic with a publisher and no subscriber (or the reverse) *is* emitted, with
a comment, and `check` reports it as a `dangling-entity` warning saying it "may
be published by an external system". That is the correct verdict: the run
cannot tell an external publisher from a node that failed to start. You decide,
and record the decision with `external: pub`.

`--include-infra` keeps `/rosout`, `/parameter_events`, `/tf` and friends,
which are excluded by default: every node touches them, no launch file
describes them, and declaring them would put two extra endpoints on all 144
nodes of a real system.

## Where the message type comes from

`interception/endpoints.tsv` records every publisher and subscription
*created*, with remap-resolved topic names and — since issue #0047 — the
message type, read by introspection off the type support the init hook already
holds. That is where both the wiring and the type come from, and it is the only
source that knows about an endpoint no message ever crossed.

The distinction is not academic: phase 77 measured **982 endpoints created and
63 carrying a message** on one Autoware run. The traffic-keyed summaries
(`stats_summary.json`, `frontier_summary.json`) are still read — the observed
counts and rates have no other source, and their `msg_type` remains the
fallback for a bundle recorded before the column existed (a five-column
`endpoints.tsv`). A topic whose type is in neither place cannot be emitted at
all and is listed with its reason; fill those in by hand from
`ros2 topic info -v`, or re-record with a current build.

## Node keys are absolute, and that is deliberate

The emitted `nodes:` keys are absolute FQNs (`/perception/foo/bar`), which the
checker passes through verbatim. The shorter bare-name spelling is reconciled
through the launch dump, and for a node the launch file did not name that
lookup **misses**: measured, a bare `talker-1` resolved to `/talker-1` — a node
that does not exist — while `check` reported the file clean (issue #0048). A
missing check is better than a silent misattachment.

It costs one thing, and it is worth knowing which. When deciding whether
`state: true` is set on an endpoint, the checker splits the reference on its
*first* slash, so on an absolute reference that flag has no effect. Everything
else, vertex lookup included, matches the reference exactly. If you want
`state: true` on an endpoint, shorten that one node's key to a bare name — and
only do it for a node the launch file names. Confirm with
`play_launch check ... --export-graph g.json`, which prints the FQN each
contract key resolved to.

## Cycles

A feedback loop is normal — a controller reading back what it commanded, a
simulator closing a physical loop — and `causal-dag` rejects a cyclic dataflow
graph outright. The contract's answer is `state: true` on the subscriber that
closes the loop, marking that edge as a stored value rather than a causal
dependency.

A run cannot tell which edge that is. Both look identical on the wire, and
guessing would invent a semantic every downstream rule inherits, where an
omission only leaves the graph sparse. So the capture **cuts** one subscriber
per cycle and names it, with the cycle, in the refusal block. Decide which edge
is stored, shorten that node's key to a bare name, and put the edge back with
the flag.

## After the capture

The captured file is a description. Turning it into a contract means adding the
requirements, and the tool has something to say at each step:

- `paths:` on a node — its `trigger:` and `output:` are facts about the code,
  and once they are there the checker derives routes, topic rates and
  end-to-end totals rather than asking you to write them.
- `play_launch measure <run-dir> --model <m.yaml>` — measured per-path CPU cost
  as a platform-file fragment, and measured `min_latency` floors as contract
  comments.
- `play_launch check --emit diagnostics-params` — `diagnostic_updater`
  parameters restated from the bounds you have declared, once you have declared
  some.
- `scripts/verify_graph.py <run-dir> --model <m.yaml>` — the inverse walk:
  grades the model's topic graph against a run, and reports any edge the run
  contradicts. Still a script rather than a verb, and deliberately: it grades a
  derivation during development, where the capture answers a question a user of
  the shipped tool asks on day one.

## Related

- `docs/design/contract-primitives.md` — the rule this verb follows: a
  contract states what the code does and what it must achieve, and anything
  computable from those two is derived, never written.
- `docs/roadmap/phase-77-graph-verified.md` — `endpoints.tsv`, and what it
  took to make the recorded topic names correct under remapping.
- `docs/issues/0044-capture-mode-documented-but-not-implemented.md`
  and `docs/issues/0047-endpoints-tsv-carries-no-message-type.md` — why this
  was a script first, and what had to land before it could be a verb.
