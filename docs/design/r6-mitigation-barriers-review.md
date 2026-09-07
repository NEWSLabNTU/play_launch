# R6 (mitigation barriers) — verification before implementing

Status: **review, 2026-09-08.** Verdict: **the rule is sound in intent and
wrong in three specifics, unusable on the corpus for a fourth reason, and
missing a check we are uniquely able to make.** Do not implement as written.

Subject: `criticality-from-hazards.md` §R6. Phase 72 implemented R1–R5 and
left R6 open; this checks it against ISO 26262's actual decomposition rules
and against the two real stacks in reach (`rt_av_demo`, Autoware 1.5.0).

## What R6 gets right

The motivation is correct and measured. Phase 72 propagates severity to
every causal ancestor, and the design's own objection stands: nobody
develops an entire autonomy stack to ASIL D. Real architecture puts a small
trusted element beside the complex one rather than making the complex one
safe — Simplex, and ISO 26262's `ASIL D → ASIL D(D) + QM(D)` decomposition
pattern. Expressing that in a contract is the right thing to want, and the
manifest is the only artifact that owns the graph needed to check it.

## F1 — "upstream of the barrier" is not a graph relation. It must be dominance.

R6 says severity propagates "at full strength until the walk reaches a node
that the mitigation channel bounds; from there upstream it continues at
`residual`."

A barrier is not upstream of the pipeline. It is a **parallel path to the
same actuator**: `estop_monitor → /safety/estop_cmd → gate`, beside
`lidar → detector → brake → gate`. No pipeline node is upstream of the
channel in any graph sense, so the rule as written has no defined starting
point.

The relation it means is **dominance**: node `N` may attenuate only if
*every* causal route from `N` to the hazard's control action passes through
the barrier. If one route bypasses it, that route is unbounded and full
severity stands.

This is not pedantry — it is the difference between a sound rule and one
that silently under-classifies. A pipeline with a second path to the
actuator (Autoware has several: `/external/selected/control_cmd` reaches
`vehicle_cmd_gate` beside `/control/trajectory_follower/control_cmd`) would
be attenuated on the strength of a barrier that does not cover it.

**Fix:** compute the dominator set of the control action's ancestor
subgraph. We own the graph; dominators are a standard algorithm and the
walk already exists in phase 72.

## F2 — `residual` is free-choice, and ISO 26262's decomposition table is not

R6 lets the author write any `residual`, while saying the channel "inherits
the full severity". ISO 26262-9 Clause 5 permits only these splits:

| from | to |
|---|---|
| ASIL D | D(D) + QM(D) · C(D) + A(D) · B(D) + B(D) |
| ASIL C | C(C) + QM(C) · B(C) + A(C) |
| ASIL B | B(B) + QM(B) · A(B) + A(B) |
| ASIL A | A(A) + QM(A) |

The design's own worked example — channel ASIL D, residual **ASIL B** — is
not in the table. If the channel takes the full D, the other element is
QM(D); if you want the pipeline at B, the channel is B(D) too, and it is
then not a Simplex monitor.

**Fix:** `residual` is not authored freely. Either the author names a legal
pair and the checker validates it against the table, or `residual` is
*derived* from the pattern and the channel's own level. A free `residual` is
a decomposition that no assessor would accept, produced by a tool that
implies it would.

## F3 — disjoint data ancestors is necessary, not sufficient — and we know better

R6 calls the disjoint-ancestor test "the single most valuable check in the
design". It verifies one necessary condition for independence, and this
repository documents at least three ways two data-disjoint nodes still fail
together:

- **Same container.** `--container-mode isolated` is the default *because*
  one segfault takes down every composable sharing a container
  (`container-isolation.md`). Autoware resolves to 15 containers holding 70
  composables. A barrier and the pipeline it bounds inside one container is
  a common-cause failure with disjoint ancestors.
- **Same cgroup / OOM group.** Phase 66 measured `oom.group=1 bystander=dead`.
- **Same host, same CPU, same DDS domain.** Phase 60's own conclusion: we
  provide the temporal-budget half of freedom from interference, not
  isolation of cache, memory bandwidth or DDS internals.

**Fix:** the independence check gains the tests we can actually make —
different container, and (where `execution.deploy` says so) different host —
and stops describing itself as verifying independence. It verifies the
absence of *data* common cause and of *colocation* common cause. Everything
else (power, clock, silicon) is outside the contract, and the check should
say so rather than imply completeness.

## F4 — on the corpus we have, no barrier passes, and the graph is not there anyway

Measured on Autoware 1.5.0, `planning_simulator.launch.xml`:

| candidate barrier | its inputs | independent? |
|---|---|---|
| `autonomous_emergency_braking` | `/perception/obstacle_segmentation/pointcloud`, `/perception/object_recognition/objects`, `/control/trajectory_follower/lateral/predicted_trajectory` | **no** — shares perception, and reads the controller's own output |
| `mrm_emergency_stop_operator` | `/control/command/control_cmd` (the gate's output) | **no** — downstream of the whole pipeline |

Both would be rejected. That verdict is *correct*: neither is an
ISO-26262-independent channel, and a production vehicle gets independence
from a separate ECU with separate sensors, which no launch manifest can
see. But it means R6 on this corpus produces "your architecture has no
independent barrier" for every candidate — true, unactionable, and
indistinguishable from the rule being broken.

Worse, the input R6 needs is largely absent. The Autoware model resolves to
**119 nodes and 0 topics**: `structure.topics` comes from contracts, and
only four scopes have any. The ancestor closure R6 attenuates over does not
exist for that stack. The design names this risk with one anecdote (a topic
with five subscribers and no publishers); the measurement is worse — there
is no graph at all, while **62 nodes carry 371 remaps** that describe the
wiring and nothing lowers them into one.

**Consequence for sequencing:** R6 is not the next thing to build.
Deriving the topic graph from remaps is, because R6, phase 72's criticality,
`scope-budget` and every other graph rule are all only as good as it.

## F5 — three smaller holes

- **Nothing requires the channel to reach the hazard's control action.** A
  mistyped `channel:` attenuates a whole pipeline and checks nothing. It
  must be a *reachability* precondition, and its failure an error.
- **A topic cannot hold criticality; its publishers can.** "The channel
  inherits the full severity" must mean the publishing node and, by R2 (no
  decay), that node's own ancestors — which is what makes the monitor's
  sensor ASIL D too. Worth stating, because it is the cost of the pattern.
- **Multiple mitigations bounding one hazard is undefined.** Max, min, or
  all-must-hold? The safe reading is that attenuation needs *every* route
  dominated by *some* barrier, which falls out of F1 if F1 is done properly.

## F6 — no verification story

Every phase since 67 has ended on a running system: `just jitter`,
`just fault`, the Autoware ladder. A mitigation barrier makes a claim that
is *directly testable* with machinery phase 73 and 75 already built — stop
the pipeline, confirm the safe state is still reached through the channel,
and confirm it is reached within the FTTI. R6 has no such step, and it is
the one rule whose whole value is a claim about what happens when something
fails.

## Recommendation

1. **Do not implement R6 next.** Derive the topic graph from remaps first
   (F4). Without it, R6, phase 72's criticality and every graph rule are
   operating on four scopes out of eighty-three.
2. When R6 is implemented, change three things: dominance rather than
   "upstream" (F1), the ISO decomposition table rather than a free
   `residual` (F2), and colocation added to the independence test with the
   claim narrowed to what it actually verifies (F3).
3. Add the reachability precondition and the runtime check (F5, F6).

The rule is worth having. As written it would attenuate routes it does not
cover, emit decompositions ISO 26262 does not permit, and call a partial
independence test complete — on a graph that, for the one real stack we
have, is empty.
