# Phase 90 - a tidy grammar: fold, rename and remove contract keys

Status: **draft** (2026-10-15). Stage 1 of 2. Stage 2 (an extension system
for domain vocabulary, and moving the domain keys into it) gets its own
phase once its design is settled; nothing here depends on that design, and
nothing here moves a key out of the core.

The grammar belongs to `ros-launch-manifest` (rlm,
`~/repos/ros-launch-manifest`); it is tracked here because the checker that
reads it is play_launch's, and every change lands in both, then in the
consumers (nano-ros's code generation, and the contracts that use the keys).

## Why

rlm v0.1.58 has about 150 keys in 33 contexts. Read by what they mean,
they are a handful of primitives: entities (nodes, endpoints, channels,
parameters), paths, bounds (a duration, rate or count, where it applies,
where it comes from, which parameter sets it), predicates, modes, hazards
and safe states. The grammar spells some of them in several places or
several ways, and a few keys carry more than one meaning:

- **Bounds are written ~15 ways**, and the same bound sometimes in two
  places: `max_transport` on a subscriber and on a topic; `drop` on a path
  and on a topic. `window` re-implements a budget's `{ duration, param,
  source }`, and `within` and `ftti` are deadlines with their own rules.
- **`reaction` means three things.** On a hazard the island contract writes
  a mode (`reaction: l3_engaged`) while the field table documents "the
  scope path whose route reaches the safe state"; on a mode it is a scope
  path; on `on_violation` it is a path on the subscriber's own node.
- **Two names for the external side of a topic:** `topics.<n>.external`
  and the `external_topics` section.
- **Severity and criticality overlap:** `severity` is declared per hazard on
  a `severity_levels` scale whose default is ISO 26262's ASIL list, and
  `criticality` is a consequence derived from it, but the two are spelled
  as if independent.
- **Settle has two shapes** (a written duration, or a braking profile by
  parameter name), and the response added in v0.1.58 sits beside the
  profile's keys.

Each of these costs a contract author a decision that has one right answer,
and costs the checker a rule that reconciles two spellings.

## Principles

1. **Readable names stay.** A key a contract author would reach for
   (`max_latency`, `max_age`, `ftti`, `window`) keeps its name. Folding
   happens underneath: one bound type in rlm's types and one rule set in the
   checker, with the keys as its spellings.
2. **One place per fact.** Where the grammar accepts the same fact in two
   places, one is kept and the other is removed.
3. **One meaning per key.** A key that names different kinds of things in
   different contexts is split or renamed.
4. **No domain keys move in this phase** (`entry_speed`, `decel`, `jerk`,
   the ASIL default). Stage 2 moves them; this phase only shapes the core
   so they can move without another reshuffle.
5. **Migration is mechanical.** Every removed or renamed key has a fix-it in
   the checker's diagnostic and a rewrite in a migration command, so no
   contract is migrated by hand.

## Units

### G1. Inventory

- What: a generated table of every key (context, kind, since, who reads it:
  the checker's rules, `--explain`, nano-ros's code generation, the
  runtime monitors), from `types/src/field_table.rs` and a grep of the
  consumers. Marks keys no consumer reads, keys accepted in two places, and
  keys with more than one documented meaning.
- Gate: the table is in rlm's docs; every row below cites it; any further
  candidates it finds are added here before G2 starts.

### G2. One bound type

- What: rlm's types get one `Bound` (value with unit; kind: latency, age,
  transport, jitter, response, rate, count, deadline, least-time; source;
  parameter binding). Every bound key parses into it; the checker's rules
  read `Bound`, not per-key structs. `window` becomes a least-time bound
  with the same map form as a budget (no separate `Window` context).
  `within` and `ftti` are deadlines. No key is renamed by this unit.
- Gate: every existing contract fixture gives byte-identical `check` and
  `--explain` output; the format reference lists the bound keys under one
  heading with their kinds.

### G3. One place per fact

- `max_transport`: keep it on the subscriber (it is the hop into one
  subscriber, and the fault arithmetic charges it there); remove the topic
  form, or keep it only as shorthand that expands to every subscriber of
  the topic (decide in G1 from who writes it).
- `drop`: keep it where the loss is observed (decide path vs topic in G1).
- `external_topics` vs `topics.<n>.external`: keep one.
- Gate: the removed spelling fails with a fix-it naming the kept one; the
  migration command rewrites it; fixtures and the island's contracts
  migrated.

### G4. One meaning per key: `reaction`

- What: split the three meanings. Proposal: a hazard `enters:` a mode (the
  ladder's entry), a mode `reached_by:` (or keeps `reaction:`) the scope
  path to its safe state, and `on_violation.reaction` stays (a path on the
  node). Decide names in this unit; the field table's descriptions then
  match what contracts write.
- Gate: the hazard form that names a mode, and the one that names a path
  (if any contract does), both migrate; a fixture of each.

### G5. Severity and criticality

- What: `severity` stays a label on a declared ordered scale; the
  severity-to-criticality mapping is stated (today it is implicit in the
  checker) and `criticality` is derived only, never written (refuse a
  written one with a fix-it). The ASIL default scale stays in this phase
  (stage 2 moves scales to profiles).
- Gate: a contract writing `criticality` gets the fix-it; derived
  criticality unchanged on every fixture.

### G6. Settle's shape

- What: `settle` takes one map form, `{ duration | <profile keys>,
  response }`, so a written duration and a derived profile sit in the same
  place and `response` applies to both (a written duration that already
  includes the response states `response: 0` or omits it). This is the
  slot stage 2's models plug into; the profile keys stay as they are now.
- Gate: settle-derived output unchanged; a written duration plus response
  sums in `--explain`.

### G7. Dead keys

- What: remove what G1 finds no consumer reads and no contract writes, with
  a fix-it for one release, or document why each stays.
- Gate: G1's table has no unexplained dead key.

### G8. Migration and release

- What: `play_launch` gains (or extends) a migration command that rewrites
  a contract from the pre-90 spellings; the checker names it in every
  removed-key diagnostic. rlm release (minor bump, the grammar version
  says which spellings it accepts); play_launch release; nano-ros pin PR
  (its code generation reads the model, so its readers move with G2);
  contracts migrated in the consumers.
- Gate: the migration command run on every contract in the consumers
  produces contracts that check with the same verdicts and `--explain`
  totals; nano-ros images build from the migrated contracts.

## Order

G1 first (it may add or drop candidates). G2 next (internal, no visible
change). G3-G7 in any order, each a breaking spelling change with its
fix-it. G8 last, one release for the lot.

## Not in this phase

- The extension system and its language, and moving `entry_speed`,
  `decel`, `jerk` and the severity scales out of the core (stage 2).
- New semantics: no rule changes what it accepts beyond the spellings
  above.
