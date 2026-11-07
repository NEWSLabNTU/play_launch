# Phase 100 - safety functions: what Safe-ROS teaches the contract

**Status (2026-11-07): planned. Nothing started.** Design of record: rlm
`docs/design/safety-functions.md` (design issue 58), which has the
analysis of Safe-ROS, the proposed keys S1-S6 and what is not adopted.
Spans rlm (grammar, model, checker rules), play_launch (the runtime
observer, `--explain`, fixtures, the guide) and nano-ros (the model fields,
pull requests from `~/repos/nano-ros-pl`).

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
| F1 | **`safe_state.value` (S1).** rlm: grammar row, parse with the `when:` predicate shape, `value-field-unknown` against `.msg`, model field. play_launch: the runtime observer checks the first post-fault publish on `emits` and reports `safe-state-value` (error) naming field and value. The interceptor needs a field read by introspection for the declared paths only, as it already does for TF and `/clock`. | A fixture whose gate publishes the right value passes. One that publishes a nonzero `linear.x` fails with `safe-state-value`, verified by a negative control. `rt_av_demo`'s `just fault` reports the value as checked. |
| F2 | **`topics.<t>.arbiter` (S2).** rlm: grammar row, model field, `arbiter-bypass` (error: another publisher in the resolved graph) and `arbiter-unused` (warning: no reaction route reaches the topic through the arbiter). play_launch: the runtime observer reports `arbiter-bypass` when a publisher is created on the topic by any other node, using `EndpointOwner` (#0080 keeps it per endpoint). | Static and runtime bypass each have a fixture and a negative control. Autoware's control contract states `vehicle_cmd_gate` as the arbiter of `/control/command/control_cmd` and checks clean, or the finding is recorded. |
| F3 | **`hazards.<h>.requirement` (S3).** rlm: grammar row `{ id, source }`, model field, the id in every diagnostic about the hazard. play_launch: `check --explain` prints each hazard as one sentence with its numbers and id. | The Autoware and island contracts carry requirement ids. `--explain` prints one sentence per hazard. A golden test pins the sentence for one hazard. |
| F4 | **`reaction-waits-on-traffic` (S5).** Only if F0 shows the walk charges nothing, or charges the input's rate without saying so. rlm rule, error, naming the fix (a timer path, or a path triggered by the reaction topic). | F0's fixture is refused with the new rule. The same fixture with a timer path on the interceptor checks clean. No existing fixture's verdict moves, or each move is explained. |
| F5 | **`mobile.protective_stop` reference extension (S4).** rlm, illustrative like the other reference extensions: `threshold >= v * reaction_total + v^2/(2*decel) + margin`, worst case proved, threshold read with `$(param ...) m`. | F0's fixture with this extension refuses 5 cm at any speed and nonzero reaction time, and accepts a threshold sized by the formula. The extension reference regenerates. |
| F6 | **Guide.** A worked example, `docs/guide/safety-function.md`, linked from `first-contract.md`: a mobile robot's protective stop, built from F0 to F5, showing what each key catches that the Safe-ROS design misses: the SIF's own silence, the traffic-dependent reaction, the bypass, the speed. | The example's contract is F0's fixture, so `play_launch check` on it is the test. `check-terms` passes. |
| F7 | **nano-ros.** The model fields from F1 to F3 in a pull request. nano-ros may generate a value check and an arbiter check in the image's route monitors. Its decision is recorded here. | nano-ros pins the rlm tag that carries the fields and its CI is green. |
| F8 | **Investigate a nuclear I&C profile (S6).** Whether IEC 61513 / IEC 61226 classification fits a profile's `integrity_scale` and `criticality` map. | A recorded yes (then a profile proposal) or no (and why) in the design doc. |

Order: F0 first, since it decides F4. F1, F2 and F3 are independent and
can ship in one rlm release. F5 and F6 follow. F7 follows the rlm release
that carries F1 to F3.

## Not in scope

- Proving a node's code (Safe-ROS's Dafny and AJPF work). A contract
  checks the running system against what it states. `requirement.source`
  can name a proof as evidence.
- Independence of the safety layer from the function it guards. `arbiter`
  is dominance only. Separate process, container, host and sensors remain
  the R6 review's question (`docs/design/r6-mitigation-barriers-review.md`).

## Sources

- Safe-ROS paper: <https://arxiv.org/abs/2511.14433>
- Safe-ROS code, read 2026-11-07: <https://github.com/dianabenjumea/Safe-ROS>
  (`cmd_vel_interceptor_node.py`, `RosEnv.java`, `remote-inspection.gwen`,
  `inspectionVerificationEnv.java`, `dafny/cmd_vel_interceptor_node.dfy`)
