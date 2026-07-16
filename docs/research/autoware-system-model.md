# Autoware System Model Study — Report

**Status:** Study complete (Phase 42, waves 1–4)
**Scope:** Autoware 1.5.0, `planning_simulator` launch, planning-simulation
vehicle model, engaged autonomous-driving scenario (goal pose set + engaged,
vehicle verifiably moving).
**Design draft:** [autoware-system-model-study.md](../design/autoware-system-model-study.md)
(Q1–Q5 definitions). **Progress ledger:**
`.superpowers/sdd/p42-progress.md`. **Wave reports:** `p42-w1-report.md`
(declared graph export), `p42-w2-report.md` (measured/engaged run),
`p42-w3-report.md` (source census) — all in `.superpowers/sdd/`.

This report answers Q1–Q5 from the design draft and distills the
consequences for three downstream tracks: the contract vocabulary proposal
(42.5), the `check` rule engine, and the chain-aware mapper design (42.6).
Every claim below is tagged with the lens that produced it — **declared**
(contract/manifest parse), **measured** (Phase 29 interception on a live
run), or **source** (reading Autoware C++ source) — and a file pointer so a
reader can re-derive it.

## 1. Method

Three lenses over one system, per the design draft:

1. **Declared** — `play_launch check --contracts <autoware-contract>
   --export-graph declared.json` (new in 42.1) walks the already-loaded
   `ManifestIndex` and reuses Phase 35's `manifest_graph::build_global_graph()`
   (which carries an `is_state` flag per edge) to export nodes, topics,
   pub/sub edges, intra-node `paths:`, cross-node `scope_paths:`, and
   detected cycles as JSON/DOT. Tool: `src/play_launch/src/ros/causal_graph.rs`;
   schema: `docs/design/causal-graph-export.md`.
2. **Measured** — the existing Phase 29 RCL interception layer
   (`libplay_launch_interception.so`, frontier + stats plugins) run against
   the live planning_simulator with the engaged-scenario runbook
   (`tests/fixtures/autoware/scripts/test_autonomous_drive.py`: initial pose
   → `/api/routing/set_route_points` → `/api/operation_mode/change_to_autonomous`),
   verified moving (velocity_status 1.37 m/s at engage+60s). 42.0 added a
   name catalog (topic names/types persisted in the summaries, no more
   riding on an incidental `discovered_topic_types.tsv`) and ring-drop
   counters (both read 0 for this run — no data lost to buffer overflow).
   Join tool: `scripts/p42_join.py` (committed), folds remap aliases from
   `record.json` and corrects a double-count bug found in this wave (§8d).
3. **Source** — read Autoware C++ source
   (`~/repos/autoware/1.5.0-ws/src/`) for 9 nodes chosen to cover every
   trigger/junction pattern reachable from the declared graph and the
   unbroken cycle, with `file:line` citations for every claim. No source
   was modified; read-only study.

All three lenses target the *same* concrete run so declared/measured/source
claims about the same topic or node can be cross-checked directly, not
just triangulated in the abstract.

## 2. The graph in numbers

### Declared (contract parse, W1)

| metric | count |
|---|---|
| nodes | 74 |
| topics | 169 |
| pub_edges | 143 |
| sub_edges | 171 — causal 38 / state 133 / required 12 |
| node_paths (intra-node) | 17 |
| scope_paths (cross-node) | 4 |
| cycles | 13 — 12 cut by a `state:` edge, 1 not |
| nodes with `criticality:` | 6 |

Source: `.superpowers/sdd/p42-w1-report.md`, `tmp/p42/declared.json`.

### Measured (engaged run, W2)

| metric | idle baseline | engaged (this study) |
|---|---|---|
| interception events | 489,945 | 1,542,744 |
| measured topics (stats) | 204 | 610 |
| declared topics with activity (alias-folded) | 81/169 | **119/169** |
| frontier topics with non-zero stamp | 0/143 | **94/435** |
| ring-drop counter | n/a (pre-42.0) | **0** |
| declared-rate topics within 20% of declaration | n/a | **16/18 with data** |

Source: `.superpowers/sdd/p42-w2-report.md`.

### Rate table highlights (declared vs measured, post double-count fix)

| cluster | declared | measured | note |
|---|---|---|---|
| 40 Hz (localization/vehicle status, 6 topics) | 40 | 39.96–40.01 | exact |
| 10 Hz (planning/perception, 11 topics, 9 with data) | 10 | 9.72–10.03 | exact-ish |
| 30 Hz `/control/trajectory_follower/control_cmd` | 30 | 33.32 | +11%, upstream-driven |
| 30 Hz `/control/command/control_cmd` | 30 | 21.03 | **-29.9%, mode-window artifact (§7)** |
| 5 Hz `/vehicle/doors/status` | 5 | 5.01 (under pre-remap alias) | remap-alias miss, not a real delta |

Full table: `.superpowers/sdd/p42-w2-report.md` §"Rate deltas"; machine copy
in worktree `tmp/p42/joined.csv` / `join_report.md` (uncommitted, per-wave
working data).

## 3. Q1 — Cycles

**Declared:** 13 cycles in the merged cross-scope graph; 12 are cut by at
least one `state:` (polled, non-causal) edge. 1 is not:

```
/vehicle_cmd_gate → /simple_planning_simulator   via /control/command/control_cmd
/simple_planning_simulator → /controller_node_exe via /localization/acceleration
/controller_node_exe → /vehicle_cmd_gate          via /control/trajectory_follower/control_cmd
```

spanning three contract files (`vehicle_cmd_gate`, `simple_planning_simulator`,
`tier4_control_launch`). `check` reports 0 errors on this exact run despite
the cycle. (Declared: `p42-w1-report.md`.)

**Two independent causes, both confirmed by source (W3):**

1. **A checker gap, not a declaration gap.** `check`'s `causal-dag` rule
   (`ros-launch-manifest/check/src/rules/causal_dag.rs`) builds its
   `DataflowGraph` **per manifest file**. A cycle spanning 3 files is
   structurally invisible to it regardless of how the edges are tagged.
2. **An authoring bug, not a vocabulary gap.** Reading the three
   constructors (`p42-w3-report.md` §Q1) shows only 1 of the 3 edges is
   truly causal:
   - `vehicle_cmd_gate → simulator` (`control_cmd`): consumer lambda
     overwrites a cached value, read only by the simulator's own 40 Hz
     timer (`simple_planning_simulator_core.cpp:163-165`). **Take-latest,
     not causal.**
   - `simulator → controller` (`acceleration`): `InterProcessPollingSubscriber`,
     read once per `ctrl_period` tick (`controller_node.hpp:101-102`).
     **Polled, not causal.**
   - `controller → vehicle_cmd_gate` (auto `control_cmd`): bound directly,
     republished synchronously inside the callback
     (`vehicle_cmd_gate.cpp:364-370`). **Genuinely causal.**

   The manifest already has the primitive (`state: true`) to describe this
   correctly, and gets 2 of 3 edges right — `vehicle_cmd_gate.contract.yaml`
   and `control.contract.yaml` both tag their polled sub correctly, with the
   latter even carrying the comment "polled, not causal". The bug is that
   `simple_planning_simulator.contract.yaml`'s `control_cmd` sub is declared
   `min_rate_hz: 30` with **no `state: true`**, while its behaviorally
   identical siblings in the same constructor (`gear_cmd`,
   `turn_indicators_cmd`, `hazard_lights_cmd` — all populated by the same
   lambda-store pattern) *are* tagged `state: true`. Inconsistent
   application of an existing field, within one file.

**Measured:** the cycle is hot in the engaged run — all three member
topics carried thousands of messages (2.9k/3.8k/2.9k) over the run window
(`p42-w2-report.md` §"Q1 material"). This is a real, running feedback loop
that no per-file check sees, not a dormant declaration curiosity.

**Verdict:** the `state:` vocabulary is **sufficient** to describe this
cycle correctly (2-of-3 cut once the one-line fix is applied); the residual
work is process, not schema: (a) the one-line contract fix in
`simple_planning_simulator.contract.yaml` (listed as an action item, not
performed — out of scope for this read-only study; see §9), and (b) a
cross-scope variant of the `causal-dag` rule so `check` can see cycles that
span manifest files at all, independent of how well they're tagged.

## 4. Q2 — Junctions

**Measured (W2):** both TIMEX junction species — over/undersampling
(data-age) and full-drain (reaction) — are directly observed, not just
theorized:

- `/localization/kinematic_state` (40 Hz pub, 18 declared subs, 2
  causal/16 state): per-subscriber endpoint takes (via the remap-alias
  accident, see §8d) range from **all-message** (mission_planner,
  manual_lane_change_handler, remaining_distance_time_calculator — 3834
  takes) down through controller (~3193, ≈33 Hz), control_validator
  (~2914, ≈30 Hz), to planning-side consumers (~880-958, ≈10 Hz). One 40 Hz
  source, four distinct effective consumption rates.
- `/planning/scenario_planning/scenario` (10 Hz, 4 subs, 2 causal/2 state):
  takes/pubs = **4.00 exactly** — every subscriber consumes every message.
  A pure all-take junction, coexisting in the same graph as the
  kinematic_state sampling junction above.
- Pattern: **state-declared edges measured with all-take behavior, and
  causal-declared edges measured with sampling behavior, both occur.**
  `state:` as declared today expresses *intent* (cycle-cutting permission),
  not measured consumption rate — junction-sampling behavior is an
  orthogonal fact the current boolean cannot and should not be conflated
  with.

**Source (W3):** 6 distinct input-handling patterns across 9 nodes, of
which the current `causal`/`state`/`required` triad fully distinguishes
only 2:

| Pattern | Example | Semantics needed |
|---|---|---|
| Lambda take-latest | `simple_planning_simulator` gear/turn/hazard subs (`simple_planning_simulator_core.cpp:128-139`) | data-age only — well served by `state: true` |
| Polling-wrapper take-latest | `controller_node_exe` (`controller_node.hpp:92-104`), `behavior_path_planner_node` (`behavior_path_planner_node.cpp:174-261`) | data-age only — same, no gap |
| Buffered queue-drain | `EKFLocalizer` `pose_queue_`/`twist_queue_` (`ekf_localizer.cpp:137-215`, `:286-309`) | data-age of oldest queued item **plus** a distinct "backlog growth" failure mode `state: true` does not express |
| `message_filters` bounded-window sync | `calibration_status_classifier_node` `ApproximateTime` (`calibration_status_classifier_node.cpp:174-196`) | reaction-time: "N inputs within Δt" — no field exists |
| Custom N:1 event+timeout fan-in, degraded fallback | `concatenate_and_time_sync_node` `CloudCollector` (`cloud_collector.ipp:34-119`) | reaction-time with partial-match-as-valid-outcome — richer than sync, no field exists |
| 1:1 causal pass-through | `CropBoxFilter` (`crop_box_filter_node.cpp:221-269`), `vehicle_cmd_gate`'s `onAutoCtrlCmd` (`vehicle_cmd_gate.cpp:364-370`) | reaction-time only — well served by `causal: true` |

**Which need data-age vs reaction-time semantics:** take-latest and
polling-wrapper patterns (the large majority of `state:` edges measured
above) need only **data-age** — "how stale is the one retained value."
Sync-policy patterns (`message_filters`, the custom collector) need
**reaction-time** framed around a bounded window across multiple inputs,
which today collapses to independent per-sub `max_age_ms` with no
cross-sub correlation. Queue-drain needs both, plus a backlog-growth
signal neither vocabulary axis carries.

## 5. Q3 — Chain composability

The design draft's question: can end-to-end chains (sensor → actuation) be
*composed* from the per-scope `paths:` we have, or is a first-class
cross-scope chain declaration needed?

**Declared chaining (`tmp/p42/declared.json`, 4 `scope_paths`):**

| scope_path | input_topics | output_topics |
|---|---|---|
| `behavior` | `/planning/mission_planning/route` | `/planning/.../behavior_planning/path` |
| `motion_planning` | `/planning/.../behavior_planning/path` | `/planning/.../lane_driving/trajectory` |
| `control` | `/planning/trajectory` | `/command/control_cmd` |
| `grid_map` | `/perception/obstacle_segmentation/pointcloud` | `/perception/occupancy_grid_map/map` |

`behavior`'s output topic string-matches `motion_planning`'s input topic
exactly — that one hop **does chain today**, by direct FQN equality, with
no cross-scope declaration needed. `motion_planning`'s output
(`/planning/scenario_planning/lane_driving/trajectory`) does **not**
string-match `control`'s input (`/planning/trajectory`) — these are
different topics. Checking the declared graph directly: `/planning/trajectory`
has 5 declared subscribers (`controller_node_exe`, `simple_planning_simulator`,
`autoware_operation_mode_transition_manager`, `control_validator`,
`lane_departure_checker`) but **zero declared publishers** anywhere in
`pub_edges` — the topic that `control`'s scope_path claims as its input has
no producer in the declared graph at all. The real producer chain is
`scenario_selector` (consumes `lane_driving/trajectory`, causal,
`min_rate_hz: 30` — `sub_edges`) → `velocity_smoother` → (eventually)
`/planning/trajectory`, but neither of those hops is captured as its own
`scope_paths` entry, and the final republish under the `/planning/trajectory`
FQN isn't in `pub_edges` at all (declared: `tmp/p42/declared.json`).

**Measured confirmation that the physical chain exists** even where the
declared graph doesn't connect it: `/planning/trajectory` is measured
active (878 pubs / 6483 takes, `join_report.md` fan-in table) and its
declared 10 Hz rate matches upstream `lane_driving/trajectory` (10.02–10.03
Hz) — the two topics are clearly the same causal step at runtime, just not
joined by name in the manifest.

**Frontier evidence for the full sensor→actuation span (measured, W2):**
the perception→planning leg is frontier-observable end-to-end — pointcloud
→ detection/tracking/prediction objects → behavior path → trajectory, all
with fresh `header.stamp` within ~65 ms of each other at shutdown. The
planning→actuation leg is **not** frontier-observable: `autoware_control_msgs/Control`
and vehicle command/report types carry a bare top-level `stamp` field, not
a nested `header.stamp`, so the frontier plugin's introspection (which
walks for `header.stamp`) skips them entirely. So even where the physical
chain plainly exists (per rates and per source), our current *instrument*
cannot confirm data freshness across the actuation boundary — a
measurement-tooling gap, not evidence the chain doesn't exist.

**Verdict:** declared `scope_paths` compose across scopes only when the
topic FQN happens to match exactly at the boundary (1 of the 3 planning
hops observed here does; the other doesn't, and the reason is a topic-name
mismatch through an unrepresented intermediate producer, not a missing
composition mechanism per se). **This needs a cross-scope chain
declaration to answer fully** — not because the physical chain is in
doubt (source + rate-match confirm it), but because (a) the manifest has
no first-class way to say "these two scope_paths are the same causal
chain, joined at this topic-family," and (b) the frontier tooling cannot
verify the actuation leg's freshness even in principle until it learns to
read bare top-level `stamp` fields. Recommending a specific chain-linking
schema is 42.5's job; this study's contribution is: composability is not
automatic today, and the concrete failure mode (unrepresented
producer-side bridge) is now on record.

## 6. Q4 — Triggers

**Source census (9 nodes, W3):**

| category | count | nodes |
|---|---|---|
| Timer-driven (self-clocked) | 4 | `simple_planning_simulator`, `controller_node_exe`, `behavior_path_planner_node`, `ekf_localizer` |
| Input/event-driven | 2 | `CropBoxFilter` (1:1), `concatenate_and_time_sync_node` (N:1 + timeout) |
| Hybrid (hard-coded timer+event mix in one node) | 2 | `vehicle_cmd_gate` (event for its primary control_cmd output, timer for the rest), `VelocitySmootherNode` (trigger = trajectory event, N auxiliary inputs polled) |
| Runtime-configurable across all three disciplines | 1 | `calibration_status_classifier_node` (service/timer/pure-event selected by a parameter) |

**Implication (source, W3):** `rate_hz` is a real, source-provable fact
only for the 4 pure-timer nodes (`create_timer`/`create_wall_timer` with a
fixed period). For the 2 pure-event nodes, a declared rate is inherited
from upstream (`CropBoxFilter`'s output rate == its lidar driver's rate),
not a property of the node itself. For the 2 hybrid nodes, **a single
`rate_hz` field cannot describe the node at all** —
`vehicle_cmd_gate`'s safety-critical primary output has no rate of its own
(mirrors whichever auto/remote/emergency source is currently active), while
its secondary outputs genuinely are timer-paced. And the
runtime-configurable node proves trigger discipline is sometimes a
*deployment parameter*, not a fixed build-time property — the same
synchronized data structure feeds three different trigger disciplines
depending on config.

This directly explains the Phase 41 finding of "11/13 RT nodes declare the
same 10 Hz": several of those nodes genuinely are independent timers that
could carry distinct real rates (a config/copy-paste issue, fixable by
authoring more carefully), but the highest-criticality nodes in the loop
cannot be honestly described by a node-level rate field *at all*,
regardless of what value is entered — a category error, not a laziness
artifact.

## 7. Q5 — Rates

**Verdict: the 10 Hz monoculture is (mostly) measured truth, not a
declaration artifact.** 9 of 11 declared-10 Hz topics had measured data;
all measured 9.72–10.03 Hz. The declared 40 Hz cluster is exact (6/6 at
39.96–40.01 Hz). The planning/perception pipeline genuinely runs at 10 Hz,
driven by the sensing/simulation cadence — authors did not fabricate a
uniform rate; §6 shows the deeper problem is that identical measured rates
hide different trigger semantics (a 10 Hz timer node vs. a 10 Hz
input-driven node look identical in the rate table but behave completely
differently under load).

**Mode-dependence lesson:** `/control/command/control_cmd` measured 21.03
Hz against a 30 Hz declaration (-29.9%, the one real flagged delta,
`p42-w2-report.md` §"Rate deltas"). This is a **window-average artifact
with signal**, not a bug: `avg_pub_rate_hz` spans the topic's full active
window (138.6 s), which includes ~50 s pre-engagement where
`vehicle_cmd_gate` publishes at an idle rate; once actually running, the
upstream `control_cmd` measured 33.32 Hz. Lesson for the mapper: a single
scalar rate per run is not enough for mode-dependent topics — the same
topic can be honestly declared at one rate, correctly hit that rate while
engaged, and still show a large "delta" purely because the measurement
window straddles two operating modes.

## 8. Consequences

### a. Contract vocabulary — candidates with motivating evidence (proposals, not decisions)

- **Node-level trigger semantics field** (e.g. `trigger: timer | input |
  hybrid`), **including a way to express hybrid nodes** where a single
  field is insufficient. Motivated directly by `vehicle_cmd_gate` (its
  safety-critical primary output is event-triggered while secondary outputs
  are timer-paced — one boolean/enum per node cannot capture this) and
  `calibration_status_classifier_node` (trigger discipline is a deployment
  parameter across three disciplines, not a build-time constant) — §6, W3.
- **Per-subscription buffering discriminator beyond `state: bool`** (e.g.
  `buffer: latest | queue`). Motivated by `EKFLocalizer`'s genuine bounded
  queue-drain (`ekf_localizer.cpp:137-215`), which today maps to the same
  `state: true` as every take-latest pattern despite having a materially
  different failure mode (backlog growth vs. silent staleness) — §4, W3.
- **Per-subscriber consumption-rate expectation, separate from `state:`.**
  Motivated by the measured coexistence of state-tagged all-take junctions
  (`/planning/scenario_planning/scenario`, 4.00 takes/pub) and
  causal-tagged sampling junctions (perception fan-out) — §4, W2. `state:`
  should stay a cycle-cutting/intent primitive; consumption-rate is an
  orthogonal, independently-useful fact for the mapper.
- **Cross-scope chain / mode conditioning?** Two open needs, not yet a
  concrete schema: (i) a way to link `scope_paths` across scope boundaries
  when the connecting topic isn't a literal FQN match (§5's
  `motion_planning`→`control` gap), and (ii) some notion that a measured
  rate or chain latency is conditioned on operating mode (§7's
  control_cmd case) rather than a single scalar per run. Both flagged as
  "needs design," not proposed as fields here — 42.5's job.
- **Junction sync-policy declaration** (e.g. `sync_policy: exact |
  approximate`, `max_interval_ms`, N-of-M partial-match). Motivated by
  `calibration_status_classifier_node`'s `message_filters::ApproximateTime`
  and `concatenate_and_time_sync_node`'s `CloudCollector` timeout-fallback
  publish — neither can be expressed today except as N independent `sub:`
  entries with no correlation between them — §4, W3.

### b. Checker

- **Cross-scope `causal-dag` rule.** `check`'s existing rule builds its
  `DataflowGraph` per manifest file (`ros-launch-manifest/check/src/rules/causal_dag.rs`)
  and is structurally blind to cycles spanning multiple contract files —
  the Q1 unbroken cycle passed `check` cleanly despite being a real,
  3-file, currently-hot loop. A merged-graph variant is needed independent
  of how well any individual file tags its edges.
- **Authoring lint: `state:` consistency across sibling subscriptions.**
  Motivated directly by the Q1 bug: `simple_planning_simulator.contract.yaml`
  tags 3 of 4 behaviorally-identical lambda-store subs `state: true` and
  misses the 4th (`control_cmd`). A rule that flags a node whose `paths:`
  entry declares itself self-clocked (`input: []`) yet has `sub:` entries
  without `state: true` — or more generally, subs populated by the same
  code pattern with inconsistent tagging — would have caught this
  mechanically.

### c. Mapper (what the evidence licenses, for 42.6)

- **Criticality is needed because rates cannot discriminate on their
  own.** §6/§7 show identical 10 Hz declarations spanning nodes with
  completely different trigger semantics and different safety roles
  (`vehicle_cmd_gate`'s primary output vs. a pure pipeline filter) — a
  rate-only or bucket-by-rate mapper cannot tell these apart; criticality
  (already in the schema, presently inert per Phase 41) is the field that
  can, and this study is direct evidence it should be load-bearing rather
  than decorative.
- **Chains are observable in the measured data and partially declared** —
  the mapper can lean on `scope_paths` for the hops that do chain by exact
  FQN (behavior→motion_planning here), but must not assume all declared
  `scope_paths` compose into full sensor→actuation chains automatically;
  §5's gap (unrepresented producer bridge, non-matching FQNs) is a concrete
  case the mapper's chain-builder needs to handle or reject explicitly
  rather than silently mis-link.
- **Mode-awareness caveat.** A chain-aware mapper that reads a single
  measured or declared rate per topic risks the same window-averaging trap
  as §7's `control_cmd` case — any rate or latency fed to a scheduling
  decision should be understood as possibly mode-conditioned, not an
  invariant.

### d. Infra fixes

- **Stamped-topic double-count (source fix needed, not done here).**
  `stats_summary.json` measured exactly 2x the true rate for every message
  type carrying `header.stamp` (declared-40 Hz topics measured 80.02,
  declared-10 Hz measured 20.0x), while bare-`stamp` types measured
  correctly. Root cause (confirmed, W2): `FrontierPlugin` and `StatsPlugin`
  each independently emit their own `Publish`/`Take` event for stamped
  messages (`plugins/frontier.rs::send_publish`, `plugins/stats.rs::on_publish`),
  and the consumer (`play_launch/src/interception/mod.rs::process_event`)
  counts every `Publish` as +1, double-counting. `scripts/p42_join.py`
  works around this by halving counts for message types proven stamped;
  the real fix (a distinct event kind per plugin, or consumer-side dedup)
  is a follow-up action item, not performed in this study.
- **Control-msg bare-stamp limitation (document, not fixable our side).**
  `autoware_control_msgs/Control` and the vehicle command/report types use
  a top-level `stamp` field, not a nested `header.stamp` — an Autoware
  message-design choice outside this project's control. The frontier
  plugin's `header.stamp`-only introspection cannot see freshness across
  the planning→actuation boundary as a result (§5). Extending the
  interceptor to also recognize bare top-level `stamp` fields is a
  reasonable infra follow-up; changing the upstream message type is not in
  scope.

## 9. Limitations

Carried forward from W2's limitations register (`.superpowers/sdd/p42-w2-report.md`
§"What the infra cannot see"), still open as of this report:

1. **Per-endpoint takes only via the remap-alias accident.** Endpoints
   that subscribe by wire name (rather than a pre-remap creation name)
   share one counter and are unattributable; a first-class per-handle key
   in the `.so` would make the Q2 evidence complete instead of partial.
2. **Stamped-type double count** — corrected in analysis (halved when
   provably stamped), not fixed at source (§8d).
3. **Window-averaged rates** — one scalar per topic over its own active
   window; mode transitions smear (the `control_cmd` case, §7). Needs
   windowed/percentile rates or raw event timestamps to resolve properly.
4. **No executor/callback timing** — trigger semantics (Q4) required the
   source lens because the interception layer has no visibility into
   timer vs. callback dispatch; this is a structural limit of an
   `rcl_publish`/`rcl_take`-level hook, not a bug.
5. **No per-edge causality confirmation** — frontier is per-topic
   max-stamp; it cannot confirm "polled, non-causal" semantics for an
   individual declared `state:` edge (only aggregate take/pub ratios,
   which are suggestive but not proof for any single subscription).
6. **`~`-remap aliases can miss** when the runtime node name differs from
   the launch-record name (observed once: the door simulator).
7. **Runtime graph deviations** (800 `graph-deviation-runtime` warnings,
   undeclared topics at endpoint creation) were not analyzed in this
   study — same class of finding as Phase 41, out of scope here.
8. **Source census sample size.** 9 nodes, chosen to cover every
   trigger/junction pattern reachable from the declared graph rather than
   a random or exhaustive sample — sufficient to establish that each
   pattern exists and to classify it precisely, but not a claim about the
   *distribution* of patterns across all ~74 declared nodes. A broader
   census would strengthen (but is unlikely to overturn) the Q4/Q2
   conclusions, since the patterns found are generic ROS 2/Autoware
   idioms (timer, take-latest, `message_filters`, polling wrapper) rather
   than node-specific one-offs.
9. **Single scenario, single run.** All measured numbers are from one
   ~2.5 minute engaged planning_simulator run; rate stability and fan-in
   ratios are not re-verified across repeated runs or alternate scenarios
   (parking, different maps) in this study.

## Action items (not performed here, per study scope)

- Fix `simple_planning_simulator.contract.yaml`'s `control_cmd` sub to add
  `state: true` (§3, §8b) — a one-line contract change; left to a future
  wave/PR against `autoware-contract`, not this read-only study.
- Fix the stamped-topic double-count at its source (§8d).
- Consider teaching the frontier plugin to recognize bare top-level
  `stamp` fields (§8d), so the planning→actuation leg becomes observable.
