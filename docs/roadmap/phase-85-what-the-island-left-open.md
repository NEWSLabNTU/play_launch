# Phase 85 - what the island's board runs left open

Status: **planned** (2026-10-01). Follows phase 84. Records the open work
found by the Autoware Safety Island's RTSS@Work demo on the S32K344 board:
`simple-autoware-safety-island`, `docs/takeover-trace.md` section 10 ("The
board budget", phase8-W30, follow-ups F1-F3) and section 11 ("Fresh runs
against the board-derived budgets", phase8-W31, "Runtime contract
violations"), and `docs/roadmap/phase-8-rtss-work-demo.md` entries W30 and
W31. The grammar items (D1-D3, and in whole or part D5, D7-D10, I9,
I10) belong to `ros-launch-manifest` (rlm, `~/repos/ros-launch-manifest`);
they are tracked here because the checker that reads them is
play_launch's.

Extended 2026-10-05 with the checker's developer-experience gaps from the
same island (D5-D10, I2-I11, T4-T9). "DX n.m" below is an entry of the
island's DX/UX survey (where the full list lives: see the end of this
file).

## Why

phase8-W30 sized the island handler's `call_mrm` hop from all 13 W8 board
runs, each term its observed maximum + 20 %:

- link (gate publish on the host -> island take, over the UART): 57 ms
  (max 47.43);
- tick (a take waits for the next 100 ms tick, ticks up to 114.84 ms
  apart): 118 ms (100 + 18);
- work (tick start -> the reaction's last publish): 31 ms (max 25.04);
- `call_mrm` = 206 ms (was 110). `driver_exit` (tick + work, no link)
  = 149 ms (was 100).

The link is also stated as `max_transport: 57ms` on the handler's
`operation_mode_availability` subscriber, the per-subscriber key rlm has
for it. play_launch 0.13.0 does not read it in the fault arithmetic:
`walk_reaction` (`manifest_loader.rs`) sums path latencies and sampling
periods only, and a reported fault's detection is the publisher's period
plus its path latency. With an external publisher the nominal graph has no
edge to weigh either, so the island contract's `check --explain` output is
byte-identical with and without the key. The 57 ms is therefore counted
inside `call_mrm`, which also charges it on the first hop after the
window's deadline, where no message crosses the link: there it is slack.

Three more costs have no key to be stated in. A timer has a rate and no
release jitter, so the tick share (100 + 18) lives inside `call_mrm` and in
the island's `tools/timeline/analysis.py` as `TICK_MS`. A service edge has
no transport or queueing key, so the comfortable-stop operator's
serve-after-tick cost (0.85-1.18 ms after the handler returns) is also
inside `call_mrm`; it cannot go on the operator's own path, because a
node-path `max_latency` also becomes nano-ros's derived node deadline and a
runtime latency monitor in the image. And an on-demand topic cannot say it
has no minimum rate.

phase8-W31 ran nine fresh acts on an image built from W30's contract. Every
row passed. The executor's violation ring, read over SWD at bring-up, was
already full with 8 start-up entries, 2 of them `rate-hierarchy-runtime`
on the comfortable-stop operator's `clear_velocity_limit` and
`max_velocity_candidates`: on-demand topics the contract gives a 10 Hz
`min_rate_hz`. The 206 ms `max-latency-runtime` monitors bound one
dispatch: the longest handler callback that published a monitored topic
was 24.93 ms. The route the checker charges is the wait for the tick plus
the link plus the work, which no runtime check measures. The link also
reached 67.02 ms in one run (r01), past the stated 57.

The same island also shows what it costs to write and check a contract
with play_launch (the island's DX/UX survey, 2026-10-03). Below, CON is the
island's `src/safety_island_bringup/launch/safety_island.contract.yaml`
(804 lines, 546 of them comments) and "the source build" is
`build/.cargo_target/play_launch/release/play_launch`.

- Version skew that the error does not name (DX 2.1-2.3). The play_launch
  on PATH (`~/.local/bin/play_launch`) prints `play_launch 0.12.0` and
  fails the island contract with "at 'hazards.hpc_loss.entry_speed':
  unknown key in `hazards.<name>` ... Every contract in this file is now
  UNCHECKED", exit 1. Nothing says which version introduced the key or
  which version the contract needs; the contract states only `version: 1`
  (CON:103). Checked 2026-10-05: on main, `git describe --tags` is
  `v0.13.0-1-gc199013f` and `src/play_launch/Cargo.toml:3` is
  `version = "0.13.0"`, so the version was bumped; the tag (12be6f6a,
  2026-09-29 20:00 +0800) is the release commit itself. The source build
  prints `play_launch 0.12.0` and passes the contract because it was built
  at 07:52 that day, before the release commit: every commit between the
  two tags carries the old string and the new grammar. The 0.13.0 venv
  binary (`/mnt/mx500/aeon/worktrees/w22-venv-upg/bin/play_launch`) prints
  `play_launch 0.13.0`. A bare semver cannot tell these apart.
- CI reads exit codes only (DX 2.4). The island's
  `.github/check-contracts.sh` expects exit 1 for six contracts meant to
  fail one named rule (lines 21-35) and compares `$got = $want` (line
  49). A `manifest-parse` refusal also exits 1, so a checker too old to
  parse the variants passes them as "ok".
- Output that does not survive a log or an ASCII document (DX 2.5, 2.6).
  `--explain` prints U+2192 in the route, an em dash, and U+2500 rules;
  lines run to 463 characters; ANSI colour is emitted when piped (the CI
  script strips it with sed, check-contracts.sh:48, 53). `check --help`
  documents `--explain` as the scheduling plan's provenance
  (`src/play_launch/src/cli/options.rs:363-371`); the hazard table the
  island and its timeline read (HAZARD RUNG ROLE DETECT WINDOWS ROUTE
  SETTLE TOTAL FTTI SLACK) is not mentioned.
- `--export-graph` carries less than the walk uses (DX 2.7). `GraphExport`
  (`src/ros-launch-resolve/resolve/src/ros/causal_graph.rs:36-45`) has
  nodes, topics, pub/sub edges, node and scope paths, cycles: no services,
  no externals, no hazards or ladder, and a timer path reads
  `"input": []`. The deck's figure script re-reads the contract YAML to
  draw what the export leaves out
  (`~/Downloads/contract-e2e-slides/scripts/contract_graph.py:6`, `:166`).
- Declared, silently not charged (DX 1.3): `max_transport: 57ms`
  (CON:638) changes no output, and nothing says so (I1 is the fix; I7 is
  the notice until then).
- The contract is never compared with the code (DX 1.6). An omitted
  endpoint is found at boot ("the eleventh subscription failed at boot
  with an opaque transport error", CON:23-28; services under-derived
  MAX_QUERYABLES, CON:76-80); a phantom endpoint over-sizes pools and is
  never found.
- The head comment carries rlm's semantics (DX 1.1). CON:1-102 explains,
  among island facts, things the grammar docs and messages should say
  (listed in D7).

## Design

- **D1 (island F2, rlm): a release-jitter key on a timer trigger.** A timer
  trigger has a rate and no release jitter. The island states the tick
  share (100 + 18 ms) inside `call_mrm`'s 206 ms and as `analysis.TICK_MS`.
  A jitter key on the timer would let `walk_reaction` and `window-expiry`
  charge period + jitter themselves, and leave `call_mrm` holding only the
  call. Decide the key's name and place (the trigger, next to the rate)
  and how `window-expiry` compares a first hop against period + jitter.
  (DX 1.2.)
- **D2 (island F3, rlm with nano-ros): a service edge's cost, and budget
  versus deadline.** A service edge has no transport or queueing key. A
  node-path `max_latency` doubles as nano-ros's node deadline and its
  runtime monitor, so the comfortable operator's serve-after-tick cost
  (0.85-1.18 ms) cannot be stated where it happens without changing the
  image's scheduling. Needs a key on the service edge, and a decision on
  separating "budget" (what the checker charges) from "deadline / monitor"
  (what the image schedules and checks at run time). Cross-repo with
  nano-ros. (DX 1.2, 1.4.)
- **D3 (rlm): an on-demand topic with no minimum rate.** There is no way to
  say "no minimum rate". The island contract's 10 Hz `min_rate_hz` on the
  comfortable-stop operator's `clear_velocity_limit` and
  `max_velocity_candidates` produced 2 `rate-hierarchy-runtime` violations
  at start-up on the board (W31 ring readout). Decide: an on-demand key in
  rlm, or the contract just drops `min_rate_hz` on those topics. (DX 1.5.)
- **D4 (with nano-ros): route versus callback.** What the runtime latency
  monitor checks (one dispatch's elapsed time) is not the route the checker
  charges (tick wait + link + work). Decide what a route-level runtime
  check means, so that the contract's static and runtime halves state the
  same quantity, or say explicitly that they do not. (DX 5.3.)
- **D5 (DX 2.1-2.3, rlm with play_launch): a contract states the grammar
  it needs.** A `rlm: v0.1.47` (or `min_tool_version:`) key in the
  contract header, next to `version: 1`, so a checker whose grammar is
  older refuses with "this contract needs rlm >= v0.1.47; this checker
  reads v0.1.45 (play_launch 0.12.0)" instead of an unknown key and
  UNCHECKED. Plus a since-version per key: `Field` in rlm's
  `types/src/field_table.rs:159-171` gains a `since:` column, so the
  format reference and a refusal can say when a key arrived. Note the
  limit: a binary cannot know keys newer than itself, so the header key
  is what makes the old binary's message right; the since-column serves
  the docs and the newer binary. Decide the key's name, whether it is
  required, and how it relates to `version: 1`.
- **D6 (DX 1.6, with nano-ros): check the contract against the code.**
  Diff each node's declared endpoints (pub, sub, srv, cli, with type and
  QoS) against the endpoints the code creates: an omitted one is today
  found at boot, a phantom one never. Inputs, in order of cost: (a)
  nano-ros's generated entity table / registration counts for the image
  (build time, exact for the island); (b) a runtime introspection dump
  of a host run (rmw_introspect or the graph cache: node -> endpoints);
  (c) the board's own liveliness tokens, which already describe the image
  (CON:82-88 quotes the four SS/SC lines). Proposed shape: `play_launch
  check --against <dump.json>`, rule `contract-vs-code`, one diagnostic
  per missing or extra endpoint. Decide the dump format with nano-ros.
  Not untracked there: nano-ros phase 463 and its issue 1419 already
  design the declared-vs-created endpoint cross-check on the image side,
  so (a) is their dump and this item is the checker's consumer of it,
  not a second design (found 2026-10-05 while writing nano-ros phase
  478). The cheap half needs no input: an endpoint under `sub:`/`pub:` that no
  `topics:` entry wires is dropped today with no diagnostic (DX 1.6); I8
  makes that a warning.
- **D7 (DX 1.1, rlm): what the island's head comment had to explain.**
  Each of these is rlm semantics the author wrote out in CON because no
  doc line or message says it; each belongs in the key's line of rlm's
  generated format reference and, where it changes a number, beside that
  number in `--explain`:
  - endpoint keys are local names; an absolute path makes
    `/node//abs/path` (CON:32-35; I9 refuses it);
  - a timer is a node path with an empty `input` (CON:66-74);
  - an omitted service is not neutral: pools are sized from the list
    (CON:76-80; D6);
  - `min_rate_hz` is a requirement, not a mechanism; detection comes from
    `max_age` / `lease_duration` under the stated `mechanism:`
    (CON:163-167, CON:620-625);
  - a window is a least time; the rung below is charged its route from
    the deadline, and the first hop after the deadline crosses no link
    (CON:203-220);
  - where a tick is charged: a downstream node's tick as a sampling hop
    of one period across a service edge (CON:192-197, CON:474-479);
  - settle is derived from the operator's parameter file and
    `entry_speed`, with the formula (CON:226-231);
  - `max_transport` is stated and not charged in 0.13.0 (CON:626-638,
    CON:565-567, CON:689-692; I1, I7);
  - a node-path `max_latency` also becomes nano-ros's node deadline and a
    runtime monitor, so a cost is hidden elsewhere on purpose
    (CON:436-442; D2);
  - a scope path's `max_latency` is the nominal traversal, not the fault
    route (CON:678-692);
  - every path of a node serialises unless declared otherwise
    (CON:320-324);
  - derived criticality is the max over hazards, never the sum
    (CON:330-335);
  - `severity_levels` defaults to ISO 26262's scale; writing it out is
    only for the reader (CON:338-341).
  The two on-demand publishers carry `min_rate_hz: 10` with no comment at
  all (CON:455-456): the grammar gave the author no way to say why (D3).
  Also decide DX 1.1's provenance field (a `source:` per number, e.g. the
  parameter file and key it was read from) so the "where each number
  comes from" block (CON:156-241) can be data.
- **D8 (DX 2.9, rlm with play_launch): acknowledge a true warning in the
  file.** The two `reaction-unguarded` warnings are true and kept
  (CON:307-318), so CI is never warning-free and a new warning looks like
  the old ones; the clean run also prints `derivable-rate` infos asking
  to delete declarations the island keeps. An in-file "acknowledged, with
  reason" the checker reads (rule id + target + reason), reported as
  acknowledged rather than dropped.
- **D9 (DX 1.8, rlm with play_launch): generalise the parameter binding.**
  The window is bound to `takeover_request_timeout` (`window-param`) and
  settle is derived from parameter files, but the 500 ms detection is
  restated by hand as `lease_duration` / `max_age` (CON:158-167) and the
  timer rates are asserted in prose to equal each `update_rate`
  (CON:42-44). A `param:` reference on any number, checked like
  `window-param`.
- **D10 (DX 1.9, rlm with the WG): criticality decomposition.** Derived
  criticality is reachability, so a QM node ranks equal to the ASIL-B
  fallback that bounds it; the `bounded_by:` relation is a WG proposal
  only. Recorded here because the checker derives it; no work until the
  WG rules.

## Implementation

- **I1 (island F1): transport on the reaction walk.** Charge
  `sub.max_transport ?? topic.max_transport` on the guard edge in
  `walk_reaction` and in a reported fault's detection, but not on the first
  hop after a window's deadline (the owner reads its own clock; nothing
  crosses a link there). Today 0.13.0's `walk_reaction` sums path latencies
  and sampling periods only, and the island contract's `--explain` output is
  byte-identical with and without the key. After this, the island's
  `call_mrm` drops 206 -> 149 ms and the window ends within 10,149 ms.
  `check --explain` should show the transport term where it is charged.
  (DX 1.2, 1.3; I7 is the notice until this lands.)
- **I2 (DX 2.2): `--version` says what it is.** Print the git describe
  and the pinned rlm tag, e.g. `play_launch 0.13.0 (v0.13.0-1-gc199013f,
  rlm v0.1.47)`. Today `#[command(version)]`
  (`src/play_launch/src/cli/options.rs:60`) prints the Cargo version only,
  which is how two binaries both printing 0.12.0 disagree on the island
  contract. The rlm tag is the one in `src/play_launch/Cargo.toml:39-42`;
  a build script can read it, or rlm can export its own version constant.
  Wheels built from a dirty or untagged tree should say so.
- **I3 (DX 2.1, with D5): refusals name the checker.** Every
  `manifest-parse` refusal ends with the checker's own version and rlm
  tag ("this checker: play_launch 0.12.0, rlm v0.1.45; a newer grammar may
  know this key"), and, once D5 lands, a header-key mismatch is its own
  error naming both versions. Today an unknown key reads as the author's
  typo even when it is the tool that is old.
- **I4 (DX 2.4): a verdict a CI script can trust.** `--format json`
  exists (`options.rs:354-356`) but its verdict is not documented as a
  contract and a parse refusal shares exit 1 with a rule failure. Give a
  parse refusal its own exit code (e.g. 2), and add `--expect <rule-id>`
  (repeatable): exit 0 only if exactly those rules report errors, so a
  negative test cannot pass on a refusal. Document the JSON verdict
  fields (files loaded / refused, per-rule counts).
- **I5 (DX 2.5, 2.6): output hygiene.** When stdout is not a TTY: no ANSI
  colour (and honour `NO_COLOR`, plus `--color never|always|auto`), and
  ASCII only (`->` for U+2192, `--` for the em dash, `-` rules for
  U+2500), also selectable with `--ascii`; wrap diagnostics to the
  terminal or a `--width N` (463-character lines today); print a parse
  failure once (today a WARN log line and the error block). Document
  the `--explain` hazard table in `check --help` and the user guide: one
  line per column (HAZARD RUNG ROLE DETECT WINDOWS ROUTE SETTLE TOTAL
  FTTI SLACK), and that ROUTE is one number per hop.
- **I6 (DX 2.7): complete `--export-graph`.** Add to `GraphExport`
  (`causal_graph.rs:36-45`): services and service edges (server, client,
  type), externals (`external: pub|sub` and the named FQN), hazards with
  their guard, detector, ladder and modes, path triggers (a timer path as
  `{"timer": {"rate_hz": N}}` instead of `"input": []`), and per-node
  attributes the walk uses (rates, derived criticality). Bump `version`.
  Done when the deck's `contract_graph.py` draws the island without
  re-reading the YAML.
- **I7 (DX 1.3): say when a stated key is not charged.** An info
  `declared-not-charged` for a key the grammar accepts and the fault
  arithmetic does not read: `max_transport` today (CON:638), named with
  where it would be charged and the phase that will (I1). Generalise via
  rlm's `Kind` column on `Field` (`field_table.rs:159-171`), so a new key
  that the checker has not wired yet gets the notice for free. Retire it
  per key as I1-style work lands.
- **I8 (DX 1.6, cheap half of D6): an unwired endpoint is a warning.** An
  endpoint under a node's `sub:`/`pub:` that no `topics:` entry wires is
  dropped from the counts with no diagnostic; warn `endpoint-unwired`,
  naming the node and key.
- **I9 (DX 1.7, rlm): refuse `/` in an endpoint key.** Endpoint keys are
  local names; an absolute path is concatenated onto the node FQN into a
  `/node//abs/path` endpoint that matches no topic (CON:32-35). Refuse a
  key containing `/` at parse time, with the message saying "local name;
  remap the topic in the launch file or wire it under `topics:`".
- **I10 (DX 1.10, rlm): a better nearest-key suggestion, and every
  unknown key at once.** `nearest` (rlm `types/src/field_table.rs:1244`)
  is Levenshtein within a length budget (2 for 5-8 characters), so
  `max_rate` (meant `max_rate_hz`, distance 3) gets "did you mean
  `max_age`?" (distance 2) (evidence: the deck's
  `brief-C-playlaunch-usage.md:458-490`, case b3). Prefer a key the typo
  is a prefix of, or that differs by a unit suffix (`_hz`, `_ms`), and
  list ties. `reject_unknown_keys` (`types/src/parse.rs:1579`) returns on
  the first key; collect all unknown keys in the file into one refusal.
- **I11 (DX 2.8): name the install prefix behind each scope.** A verdict
  changes with the shell: from a login shell the demo's host_ws shadows
  stock `tier4_system_launch` and an Autoware check parses 163 scopes
  instead of 169, every MRM hazard then `hazard-unguarded`. Print, under
  `--explain` or `-v`, the resolved prefix per package, and flag a
  package resolved from more than one prefix on the search path.

## Test/check

- **T1: the F1 regression.** `--explain` must change when `max_transport`
  is present. An island-style fixture (external publisher behind a
  transport, a windowed rung owned by a 100 ms tick) expecting `call_mrm`
  149 and the window's end within 10,149 ms, and no transport on the hop
  after the deadline. (DX 1.3.)
- **T2: fixtures for D1, D2 and D3** once the keys exist: a jittered timer
  charged period + jitter by the walk and by `window-expiry`; a service
  edge whose cost is charged without becoming a node deadline; an on-demand
  topic that `rate-hierarchy` does not hold to a minimum rate.
- **T3: the open Dependabot alerts** on NEWSLabNTU/play_launch. 6 open, all
  severity high, all in the root `package-lock.json`: 5 on `fast-uri`
  (#40, #41, #42, #43, #46) and 1 on `js-yaml` (#45). Bump or override the
  two packages and confirm the alerts close.
- **T4: version and refusal fixtures (I2, I3, D5).** `--version` matches
  `play_launch X.Y.Z (<describe>, rlm vA.B.C)`; a fixture with an unknown
  key refuses with the checker's rlm tag in the message; once D5 lands, a
  contract stating a newer `rlm:` is refused with both versions named and
  exit 2.
- **T5: CI verdict (I4).** A parse-refusal fixture exits 2 and its JSON
  says refused; `--expect ladder-rung-budget` passes on a fixture failing
  only that rule and fails on one failing another rule or refusing to
  parse.
- **T6: output hygiene (I5).** `check --explain` on the island-style
  fixture, piped: no byte >= 0x80, no ESC, every line <= the default
  width; with `NO_COLOR` set on a TTY, no ESC.
- **T7: export completeness (I6, I8, I7).** The island-style fixture's
  export has the operate service edges, the external publisher, both
  hazards and the ladder, and a timer trigger with its rate; an unwired
  endpoint fixture warns `endpoint-unwired`; a `max_transport` fixture
  prints `declared-not-charged` until I1, then T1 takes over.
- **T8: rlm grammar (I9, I10).** In rlm: `pub: { /abs/topic: ... }` is
  refused; `max_rate` suggests `max_rate_hz`; a file with two unknown
  keys names both.
- **T9: re-verify two stale observations on 0.13.0 / rlm v0.1.47.**
  DX 2.10: `check safety_island.launch.xml` from inside the launch
  directory printed the cross-scope header and no cross-scope
  diagnostics on 0.10.0. DX 1.11: grammar docs showing spellings that are
  parse errors (play_launch issues 0036, 0038 are marked resolved).
  Close each or open an issue.

## Consumers / cross-repo

- The island contract change waits on I1: `call_mrm` 206 -> 149 ms, and
  the link's `max_transport` re-sized from 57 to about 81 ms (+20 % over
  W31's 67.02) once more runs size it again. Until I1 ships the re-size
  moves no verdict, because 0.13.0 reads neither number in the fault
  arithmetic.
- D2 and D4 pair with the nano-ros phase for the island's findings: the
  budget / deadline split and a route-level runtime check need both
  toolchains to agree.
- D1-D3 land in rlm first and are pinned here, as phases 83 and 84 were.

## Where the full list lives

The island repo carries the whole survey, every gap with its evidence,
owner and tracker, including the nano-ros and island-side ones this phase
does not take: `simple-autoware-safety-island`,
`docs/dx-ux-gaps-2026-10.md` (sections 1-2 are mostly this phase's;
section 8 ranks the top 10, where version skew is 2nd, contract-vs-code
3rd and the uncharged budget keys 6th; section 9 splits cheap from
structural). Liveliness and derived-count reporting, and the heap, are
nano-ros's and are not taken here.

## Cheap first

Each under a day, one repository:

- I2 `--version` with git describe and the rlm tag (play_launch).
- I3 refusals name the checker's version (play_launch).
- I4 a distinct exit code for a parse refusal, and `--expect`
  (play_launch).
- I5 no colour / ASCII when not a TTY, `--width`, parse failure once,
  `--explain` table documented (play_launch).
- I7 the `declared-not-charged` notice for `max_transport` (play_launch).
- I8 `endpoint-unwired` (play_launch).
- I9 refuse `/` in an endpoint key (rlm).
- I10 prefix / unit-suffix suggestion and all unknown keys at once (rlm).
- T9 the two re-verifications.

Structural: D5-D10, and I6 (a day or two, no design question) and I11.
