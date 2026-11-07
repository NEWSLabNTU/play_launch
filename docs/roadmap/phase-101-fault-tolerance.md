# Phase 101 - fault tolerance: when most of the system is safety critical

**Status (2026-11-07): planned, revised the same day to the primitives
rule; nothing started. Follows phase 100.** Design of record: rlm
`docs/design/fault-tolerance.md` (design issue 59): twelve case studies
from aircraft, spacecraft, nuclear, driving and medical systems. In the
core: `k` on a redundant group, one `independence:` form, `on_silence` on
an external subscriber side, three `on:` classes. Derived with no key:
restart credit from `respawn`, standby, fail-operational, freedom from
interference. Use-case specifics are extensions. Spans rlm, play_launch
and nano-ros.

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
| T0 | **Decide** the design doc's §6 questions: `k` or IEC 61508's `m`; whether a startup time may be stated without a run; which deployment facts count for `independence`. | Each answered in the design doc. |
| T1 | **`independence:` and freedom from interference (§4.5).** rlm: top-level `independence: [{ of, not_sharing }]`, model field; resolver rule `independence-violated` from process, container, host and CPU set; derived `interference-shared-process` (warning) for nodes of different derived integrity in one process. | Fixtures for each shared fact with negative controls. Autoware under `observable` reports its shared-process cases or they are recorded; under `isolated` the rule is silent. |
| T2 | **Recovery credit (§4.2), no key.** The checker credits a node's `respawn` when detection + `respawn_delay` + its startup (from `check --against <run>`'s `startup_timeline.json`) fits the FTTI; `recovery-unfit` (warning) otherwise. A redundant group above `k` after a loss keeps the mode. The observer reports which recovery the run took and how long. | `rt_av_demo`: a killed detector with `respawn` is credited when its restart fits and degrades when it does not, both measured. Without a run the restart is not credited and the output says so. |
| T3 | **`functions.<f>.k` (§4.1).** rlm grammar and model; members may be value functions. The observer counts held members. | A 2-of-3 fixture: one silent member keeps the function, two lose it; a member lost by value counts as lost. |
| T4 | **`on_silence` on an external subscriber side (§4.4).** rlm grammar and model; a level may end in silence, settled by `after` plus the consumer's response. | A fixture whose external base stops on silence passes with a silent gate; without `on_silence` it fails with `final-element-silent`. |
| T5 | **`on:` gains `repetition`, `resequencing`, `insertion` (§4.6).** rlm closed set; the observer detects each from stamps and endpoint owners. | One fixture per class with a negative control. `insertion` on a commanded topic and `arbiter-bypass` report one finding. |
| T6 | **Guide.** `docs/guide/fault-tolerance.md`: a redundant sensor pair, a restarted planner, a level that cannot degrade in time and is rescued by redundancy, and what the checker says about each. | Its contracts are the fixtures above. `check-terms` passes. |
| T7 | **nano-ros.** The model fields in a pull request; which runtime checks the image generates is recorded here. | nano-ros pins the rlm tag and its CI is green. |

Order: T0; T1 (independence is the argument the rest rests on, and needs
no runtime work); T2 and T5 (they use what play_launch already records);
T3 and T4; T6 and T7 last.

## Not in scope

- Logical (control-flow) supervision, AUTOSAR's third supervision.
- Corruption as a fault class: not observable above DDS.
- Diversity of redundant members (dissimilar software or hardware): a
  stated claim, not a deployment fact.
- Value agreement across redundant members: needs a runtime extension
  hook; recorded in the design doc.
- Generating a GSN safety case from a checked contract: a possible later
  export, which phase 100's requirement trace makes possible.
