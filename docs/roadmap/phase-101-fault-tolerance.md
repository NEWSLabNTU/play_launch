# Phase 101 - fault tolerance: when most of the system is safety critical

**Status (2026-11-07): planned; nothing started. Follows phase 100, whose
final element it builds on. The design doc's §6 open questions come before
any code.** Design of record: rlm `docs/design/fault-tolerance.md` (design
issue 59): twelve case studies (aircraft flight control, spacecraft FDIR,
ARINC 653, fail-operational driving, nuclear defence in depth, closed-loop
insulin, ventilators, surgical robots, Auto-GCAS, EN 50159, AUTOSAR
watchdogs), what they share, the additions, and the larger models
considered and not adopted. Spans rlm, play_launch and nano-ros (pull
requests from `~/repos/nano-ros-pl`).

## Why

Phase 100 gives a safety function its final element. Large systems are
not made safe by one safety function: some phases have no safe state, and
safety rests on redundancy, recovery before degradation, and the
independence of redundant members. Today the contract can say that a
redundant group is lost only when every member is (`all_of`) and nothing
else of this. Three things play_launch already has make the additions
checkable rather than declarative: `respawn` and the measured startup
timeline (a restart's time), the deployment facts (process, container,
host, CPU set, memory group), and the interceptor's per-endpoint owner and
stamps (repetition, resequencing, insertion).

## Units

| Unit | Work | Acceptance |
|---|---|---|
| T0 | **Decide** the design doc's §6 questions: where `k_of` agreement sits, whether a restart's time may be stated, whether `fail_operational` is declared or derived, which deployment facts count for `independent:`. | Each answered in the design doc; its status line says so. |
| T1 | **`independent:` and freedom from interference (§4.5).** rlm: `functions.<f>.independent`, `final_elements.<e>.independent_of`/`independent`, model fields; resolver rule `independence-violated` from process, container, host and CPU set; `interference-shared-process` (warning) when nodes of different derived integrity share a process. | Fixtures for each shared fact with negative controls. Autoware under `observable` reports the shared-process cases of its control stack, or they are recorded; under `isolated` the rule is silent. |
| T2 | **`recovery:` (§4.2).** rlm grammar and model; the checker credits a step only when detection plus its time fits the FTTI (`recovery-unfit`, warning). A restart's time is the node's `respawn_delay` plus its startup from `startup_timeline.json` when a run is given (`check --against`), else stated or absent. play_launch's observer reports which step the run actually took and how long it took. | `rt_av_demo`: a killed detector with `respawn` is credited when its restart fits and degrades when it does not, both measured. A recovery step whose node has no `respawn` is refused. |
| T3 | **`functions.<f>.k_of` (§4.1).** rlm grammar and model; the observer counts healthy members and, with `agree:`, compares the stated field across members within the tolerance. | A 2-of-3 fixture: one silent member keeps the function, two lose it, one member off by more than the tolerance is reported. |
| T4 | **`fail_operational:` modes (§4.3).** rlm: `fail-operational-uncovered` (error) for a hazard in such a mode covered by degradation alone. | A fixture with a covered and an uncovered hazard. The island's and Autoware's contracts are checked, and each finding is recorded. |
| T5 | **`on:` gains `repetition`, `resequencing`, `insertion` (§4.6).** rlm closed set; play_launch's observer detects each from stamps and endpoint owners. | One fixture per class with a negative control. `insertion` on a final element's channel and phase 100's `arbiter-bypass` report one finding, not two. |
| T6 | **Guide.** `docs/guide/fault-tolerance.md`: a redundant sensor pair, a restarted planner, a fail-operational driving mode, and what the checker says about each. | Its contracts are the fixtures above. `check-terms` passes. |
| T7 | **nano-ros.** The model fields in a pull request; which of the runtime checks the image generates is recorded here. | nano-ros pins the rlm tag that carries the fields and its CI is green. |

Order: T0; T1 first (independence is the argument the others rest on, and
it needs no runtime work); T2 and T5 next (they use what play_launch
already records); T3 and T4; T6 and T7 last.

## Not in scope

- Logical (control-flow) supervision, AUTOSAR's third supervision.
- Corruption as a fault class: not observable above DDS.
- Diversity of redundant members (dissimilar software or hardware): a
  stated claim, not a deployment fact.
- Generating a GSN safety case from a checked contract: a possible later
  export, which phase 100's requirement trace makes possible.
