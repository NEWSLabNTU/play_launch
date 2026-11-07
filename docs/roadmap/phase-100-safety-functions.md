# Phase 100 - safety functions: what Safe-ROS teaches the contract

**Status (2026-11-07): planned, revised three times the same day;
nothing started.** Design of record: rlm `docs/design/safety-functions.md`
(design issue 58). Held to the contract's primitives rule, the work is one
new key, `modes.<m>.commands: { <topic>: [predicates] }`, plus
`hazards.<h>.requirement`; a commanded topic is a final element, its
arbiter is its one publisher in the resolved graph, and the bypass,
invariant and silence checks are derived. The `final_elements:` section
of the second revision is dropped. Use-case specifics are extensions.
Spans rlm, play_launch and nano-ros (pull requests from
`~/repos/nano-ros-pl`).

## Why

Safe-ROS (University of Manchester, FMAS 2025, arXiv 2511.14433) is a ROS
safety layer whose decision logic is formally verified and whose timing is
not stated. Our contracts are the reverse: they bound detection, reaction
and settle against the FTTI, and say nothing about:

- **what** the safe state commands: the observer accepts any publish on
  `safe_state.emits` as the reaction, so a gate that sends the wrong
  command on time passes;
- **which node** has the last word on an actuator topic: nothing checks
  that every command passes through the gate, and nothing notices a second
  publisher;
- **which requirement** of the hazard analysis a hazard implements: a
  hazard has free-text `description:` only.

Its published code also shows two shapes the checker should name:

- a reaction emitted only when the overridden command traffic arrives
  (`cmd_vel_interceptor` publishes from its `/cmd_vel` callback only);
- a 5 cm trigger distance never checked against the robot's speed.

All keys are additive, so no unit here follows phase 98's rename schedule.

## Units

| Unit | Work | Acceptance |
|---|---|---|
| F0 | **A Safe-ROS fixture.** A contract and launch file shaped like Safe-ROS's system: a LiDAR, a safety node publishing a stop flag, an interceptor whose only path is triggered by `/cmd_vel` and which reads the flag as `state: true`, and the base. Record what `check` says today about the reaction route. | The fixture is in rlm's `tests/fixtures`. The phase doc records today's verdict on the route, which decides F4. |
| F1 | **`modes.<m>.commands` (S1).** rlm: grammar and model (`ModeContract.commands`), predicates in the `when:` shape checked against `.msg` (`commands-field-unknown`). Derived static rules on every commanded topic: `arbiter-bypass` (more than one publisher in the resolved graph), `reaction-bypasses-arbiter` (a level's reaction route reaches the topic without passing its one publisher). | One fixture per rule, each with a negative control. The island's and Autoware's minimal risk condition levels state `commands` on the vehicle command topic and check clean, or each finding is recorded. |
| F2 | **The invariant at run time (S1).** play_launch's observer: from the reaction's derived bound until the level is left, every message on a commanded topic satisfies the level's predicates (`safe-state-violated`); silence is judged by the consumer's own silence behaviour, its `sub.max_age` + `on_violation` or an external side's `on_silence` (phase 101), else `final-element-silent`; a publisher created on the topic by another node is runtime `arbiter-bypass`. The interceptor reads only the declared fields. | `rt_av_demo`'s `just fault` checks the invariant through the whole emergency stop. Fixtures: a gate that sends one safe message and then forwards commands again (fails), a stop that never releases after `exit:` (fails), a correct gate (passes). |
| F3 | **`hazards.<h>.requirement` (S2).** rlm: grammar row `{ id, source }`, model field, the id in every diagnostic about the hazard. play_launch: `check --explain` prints each hazard as one sentence, with its bound and its level's `commands`. | The Autoware and island contracts carry requirement ids. A golden test pins the sentence for one hazard. |
| F4 | **`reaction-waits-on-traffic` (S3).** Only if F0 shows the walk charges nothing, or charges the input's rate without saying so. A derived rule, no key. | F0's fixture is refused; the same fixture with a reaction-triggered path checks clean. No existing fixture's verdict moves, or each move is explained. |
| F5 | **`mobile.protective_stop` reference extension (S4).** Illustrative, on `deadline_by`: `field >= v * reaction_total + v^2/(2*decel) + margin`, field read with `$(param ...) m`. | F0's fixture refuses 5 cm at any speed and nonzero reaction time, and accepts a field sized by the formula. |
| F6 | **Guide.** `docs/guide/safety-function.md`, linked from `first-contract.md`: a mobile robot's protective stop built from F0 to F5, showing what each part catches that the Safe-ROS design misses, and the safety function's own monitor as a hazard. | The example's contract is F0's fixture. `check-terms` passes. |
| F7 | **nano-ros.** `ModeContract.commands` and the requirement in a pull request; whether the image checks the invariant is recorded here. | nano-ros pins the rlm tag that carries the fields and its CI is green. |
| F8 | **Investigate a nuclear I&C profile (S5).** Whether IEC 61513 / IEC 61226 classification fits a profile's `integrity_scale` and `criticality` map. | A recorded yes (then a profile proposal) or no (and why) in the design doc. |

Order: decide the design doc's §7 questions (where the invariant starts,
whether `commands` sits on modes only); F0, which
decides F4; then F1 and F3 in one rlm release, F2 on it, F5 and F6, and
F7 after the rlm release.

## Not in scope

- Proving a node's code (Safe-ROS's Dafny and AJPF work). A contract
  checks the running system against what it states. `requirement.source`
  can name a proof as evidence.
- A relational predicate (a speed limit that is a function of a measured
  distance). Discrete zones as degradation levels cover the practice.
- Independence of the safety layer from the function it guards. `arbiter`
  is dominance only. Separate process, container, host and sensors remain
  the R6 review's question (`docs/design/r6-mitigation-barriers-review.md`).

## Sources

- Safe-ROS paper: <https://arxiv.org/abs/2511.14433>
- Safe-ROS code, read 2026-11-07: <https://github.com/dianabenjumea/Safe-ROS>
  (`cmd_vel_interceptor_node.py`, `RosEnv.java`, `remote-inspection.gwen`,
  `inspectionVerificationEnv.java`, `dafny/cmd_vel_interceptor_node.dfy`)
