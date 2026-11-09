# The contract safety model: evaluation

Phase 102 M5. Phases 100 and 101 added a safety model to the contract:
what a degradation level commands (`modes.<m>.commands`), the requirement a
hazard implements, an m-of-n count on a redundant group, independence
between nodes, what an external consumer does on silence, three
communication fault classes, and restarts credited from a run. Phase 102
set seven questions and their pass lines before any result. This report
answers them.

`results.md` holds every number below; `generate.py` regenerates it from
the fixtures and from the recorded runs in `data/`. Each row names the
fixture, test or run it came from.

## Verdict

| # | Question | Pass line | Result |
|---|---|---|---|
| E1 | Can the cases be written with the decided primitives? | every case written; each gap ruled | **Pass.** 12 cases written; 6 gaps, each ruled (one grammar fix, one defect fixed, four out of scope or extension territory). |
| E2 | Does each rule fire on its defect? | 100% of seeded defects; 0 findings on every twin | **Fail, 16 of 17.** Every rule fires on its defect and every twin is quiet; one seeded defect, a stop that never releases, has no rule. |
| E3 | False positives on real systems? | no false error; every warning true or explained | **Pass.** No false error; one false warning in the first Autoware draft was a true finding about the draft (below). |
| E4 | Do the derived bounds match runs? | each measured time within its bound; credit decided as the run went | **Partly.** Credit and run agree in both cases, and the first safe message is within its bound in both. One measured restart (1401.91 ms) exceeds its credited total (1133.0 ms). |
| E5 | Runtime cost? | under 1% of a core; ns per event | **Pass, weakly.** No difference between the arms above run-to-run noise (play_launch 14.8% vs 14.0% of a core, interception 1.10% vs 1.36%); 52.2 ns per field per publish. The idle simulator published only 26 to 40 commanded messages a run, so the runs bound the cost, not measure it. |
| E6 | Author burden? | fewer lines than the dropped forms; no fact written twice | **Pass.** 24 lines against 58; no fact written twice once rlm v0.3.14 was pinned (below). |
| E7 | Same answer in nano-ros? | same hazard, level and verdict | **Not run.** The image cannot report the level it fell to, nor any value fault (M4's table). |

## E1 expressiveness

Every use case of rlm `docs/design/safety-functions.md` section 4 is a
fixture: U1 is `contract_commands` and `contract_protective_stop_sized`,
U2 is the island's and Autoware's contracts (E3), U3 to U7 are
`contract_case_u3_cobot` to `contract_case_u7_rail`. A Simplex system
(`contract_simplex`) and the fault-tolerance cases
(`contract_independence_ok`, `contract_recovery`) are written too. Each
checks with no error; the one warning they share, `reaction-unguarded`, is
true of every one of them (nothing watches the actuator topic).

Six gaps were found, each with a ruling (results.md, "Gaps found writing
them"). Two matter:

- **A string that looks like a constant.** U4's safe state is the flight
  mode `LAND`. A predicate value in upper snake case is read as a message
  constant name, quoted or not, so `equals: "LAND"` is refused on a
  `std_msgs/String` (`commands-field-unknown`: it declares no constant).
  The fixture uses mavros' spelling `AUTO.LAND`. Ruling: a quoted YAML
  scalar is a string; an rlm change.
- **A topic declared with a type alone was dropped.** Autoware's
  comfortable stop commands `/planning/scenario_planning/max_velocity`, a
  topic the launch file's remaps wire and no contract declares; the
  contract only needs its type. rlm's conditional cleanup removed every
  topic left without endpoints, including one that never had any, and the
  resolver would not have wired a contract topic from the remaps anyway.
  Both halves are fixed (rlm v0.3.14 `types/src/cond.rs`, and
  `derive_topics_from_remaps`), and play_launch pins v0.3.14: the Autoware
  contract states the topic's type and nothing else.

## E2 detection

Eight static defects, each beside its correct twin, all reported by the
intended rule and all twins quiet. Eight runtime defects are seeded as
synthetic events in the observer's unit tests or on the running system
(E4); E2 added the missing positive case for `resequencing`, which had
only its negative control.

One seeded defect has no rule: **a stop that never releases** (Safe-ROS's
own defect, in rlm `safety-functions.md` 2.3). The invariant judges a
level only while it is active, and nothing judges that the commanded
topic leaves the level's predicates once the mode above is available
again. Ruling: a runtime rule `level-not-released`, recorded as follow-up.
The pass line is 100%, so E2 fails.

## E3 false positives

`data/e3_*.txt` are `check` on the three systems with `commands` added on
their minimal risk condition levels:

- **The island** (`~/repos/simple-autoware-safety-island`, branch
  `phase102-e3-safety-model`): clean under `--locked --strict`. Its six
  `measured-unsupervised` warnings predate this work and are true (measured
  maxima nothing supervises). Migrating its `guards:` to `monitors:` was
  needed first: the island's pinned play_launch is older than phase 98.
- **Autoware's overlay contracts**: the same findings as before the change
  (`data/e3base_autoware_before.txt`): the deliberate
  `fault-reaction-time` error of `mode_unavailable_tight`, eleven
  `causal-dag-global` cycles analysed in phase 77, two true
  `reaction-unguarded`. The first draft commanded
  `max_velocity_candidates` and got `arbiter-bypass` (two publishers, the
  motion velocity planner and the comfortable stop operator). That was
  true: several planners propose limits there and the external velocity
  limit selector takes the least, so the selector's output is the final
  element. It also got `commands-field-unknown` (the topic had no declared
  type), true as well.
- **rt_av_demo**: clean; at run time (E4) the safety model's own rules
  reported nothing (`safe-state-violated`, `final-element-silent`, runtime
  `arbiter-bypass`: 0). The run's other findings predate it: seven
  `graph-deviation-runtime` for the topics the contract does not describe
  and three `rate-hierarchy-runtime` for the rate lost while the lidar was
  dead, both true.

## E4 timing

`just fault-recovery` in `examples/rt_av_demo`: the lidar's process exits
at 8 s and respawns after 0.1 s (`fits`) or 1.0 s (`unfit`). For each run,
`check --against` the run gives the static credit, and the live observer's
`hazard-recovered` says whether the guard came back inside the 500 ms FTTI.

| Case | Static | Credited total | Measured outage | Live | First safe message | Detection + route |
|---|---|---|---|---|---|---|
| fits | `recovery-credited` | 269.0 ms | 221.19 ms | inside | 107.16 ms | 107.0 ms |
| unfit | `recovery-unfit` | 1133.0 ms | 1401.91 ms | past | 102.0 ms | 107.0 ms |

The credit decided as the runs went in both cases, the first safe message
came within the level's derived bound (with the 40 ms allowance `just
fault` uses for DDS wake-ups), and the level's `commands` held throughout.
The `unfit` restart took 269 ms longer than credited: the credit takes the
node's startup from its first spawn (33 ms), and the respawned process,
started on a host running other builds, took about 400 ms. A startup
measured once is a sample, not a bound. Ruling: credit from the slowest
startup in the run (respawns included) and say how many were seen; until
then a credit near the FTTI is optimistic. Recorded as follow-up.

Running it found a defect: `startup_timeline.json` was written once, at
startup complete, before the interceptor had reported any node's first
endpoint or first publish, so `check --against` could never credit a
restart from a real run (`recovery-uncredited: the run records no
startup`). The fixtures' hand-written timelines had hidden it. The file is
now rewritten while its rows change.

## E5 runtime cost

`data/e5_run.sh`: Autoware 1.5.0's planning simulator for 120 s per run,
with the overlay contracts' `commands` fields read (`fields`) and without
them (`nofields`), alternating, two runs each. play_launch's own CPU from
each bundle's per-subsystem totals (the method of 2026-10-13):

| Run | play_launch | interception (drain and observer) |
|---|---|---|
| fields-1 | 15.2% | 1.12% |
| nofields-1 | 12.4% | 1.42% |
| fields-2 | 14.4% | 1.08% |
| nofields-2 | 15.6% | 1.30% |

The arms differ by less than the runs within an arm do, in both
directions. The reason is the input: with no goal set, the gate published
`/control/command/control_cmd` 20 and 13 times a run and the velocity
limit selector not at all (`fields.json`), so the field reader ran 26 to
40 times in two minutes. In the publishing process the reader costs
**52.2 ns per field per publish** (`field_hook_cost`, release, this host;
M0b measured 50 ns). At 10,000 declared-field publishes a second that is
0.05% of a core; the observer's evaluation of a value is counted in the
interception figure above and was not measured apart. A driven run, where
the gate publishes at its rate, is what would measure rather than bound
it.

## E6 author burden

| Contract | Lines added (no comments) | Same facts, dropped forms |
|---|---|---|
| island | 9 | 21 |
| Autoware overlay | 12 | 22 |
| rt_av_demo | 5 | 15 |

The dropped forms are `final_elements:` (channel, arbiter, safe predicate
per level, silence) and `recovery:`, written for the same facts in
`data/e6_dropped_forms.yaml`. The field census reports no new write-only
field (`requirement` is read since M3). The first Autoware draft wrote one
fact twice, the endpoints of `/planning/scenario_planning/max_velocity`
that the launch file's remaps state, because rlm v0.3.13 dropped a
type-only topic (E1); with v0.3.14 pinned the entry is its type alone.

## E7 same answer in nano-ros

Not run. nano-ros#3341 carries the fields, and its image reports hazard
detection, reaction and recovery, but not the level it fell to when that
level answers in time, and nothing for a value fault (`commands`) or the
three communication classes. The island is also pinned to an older
nano-ros. E7 can be run only on an omission fault, once the island moves to
a nano-ros with #3341 and the image reports its level.

## Follow-up

- rlm: a quoted predicate value is a string (E1).
- play_launch: `level-not-released` (E2); credit a restart from the slowest
  startup a run saw (E4).
- nano-ros: report the active level; E7 after that.
