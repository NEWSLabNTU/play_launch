# Phase 91 - domain extensions: rlx, profiles and the vehicle keys moved out

Status: **in progress** (2026-10-17): E0 done (no-go: rlx owns its
expression core); E1 and E2 done (rlm v0.1.65). Stage 2 of 2; phase 90 (a tidy grammar) is complete. Design: rlm `docs/design/extension-system.md` (with
`extension-language-exploration.md` and `extension-examples.md`). Work lands
in rlm (format, language, profiles), play_launch (loader, evaluator, Z3,
explain, packaging, migration) and the consumers (nano-ros reads the
exported model; contracts adopt `use:`).

## Units

### E0. Spike: embed a restricted Numbat (1-2 days)

- Embed Numbat in a scratch crate; remove `print`, `assert`, `now`,
  `random` from the prelude; refuse recursion; port vehicle.braking and
  rail.ebd; record a trace of every binding and branch; translate one
  expression to Z3.
- Gate: a go/no-go note in the design doc: binary size and build time
  added, what could not be restricted, whether its parser accepts the 5.3
  character rules or needs a pre-pass. No-go means rlx's own expression
  core (E2 grows by about 2-3 kLOC).

**Done (2026-10-17): no-go.** rlm `docs/design/extension-system.md`
section 13 (rlm `d002e1f`), spike in rlm `docs/design/spikes/e0-numbat/`.
Numbat 1.24.0 checks dimensions at load in every branch, and both ports
compute the documented numbers (vehicle.braking 4422.67 ms at exact
30 km/h; rail.ebd 64.35 s and 1529.75 m, its table passed as a list of
structs since Numbat has no lambdas). It costs 74 crates, +2.7 MB stripped,
a 6.5 s clean build on 48 cores. It fails on three counts:
`Context::interpret` checks and runs in one call, so a file's statements
execute before a loader can refuse them; a bodyless `fn` declaration binds
to Rust FFI by name, undoing any prelude restriction (`fn random() ->
Scalar` brought `random()` back); and the expression tree is private, so
no trace, no Z3 translation and no refusal of self-recursion (accepted).
Its tokenizer accepts confusables, bidi controls and non-NFC text, so 5.3
needs a pre-pass regardless. rlx gets its own expression core; E2 grows by
about 2-3 kLOC as planned. Found on the way: the design example's
`30 km/h` beside 4421.33 ms is the arithmetic of the island's written
8.33 m/s, so E8's identical-totals gate keeps values as written.

### E1. Core interface (rlm)

- `hazard.detection` (a bound); `hazard.deadline_by`; `reaction_total` as a
  read-only per-hazard value; `source: derived` with a derivation record;
  `derive-conflict`.
- Gate: fixtures with a hand-written derivation record check as if derived;
  `--explain` prints `derived by`.

**Done (2026-10-17).** rlm v0.1.65 (`bc57ebe`, GRAMMAR_VERSION 0.1.65): `source: { derived: { by,
version, sha256, inputs, trace } }` (`types::Derivation`, boxed so a
timer's trigger stays small), `hazards.<h>.detection` (a bound) and
`deadline_by` (`types::CheckRef`), `reaction_total` refused as read-only.
play_launch: the stated detection is added BEFORE the derived FDTI in
every rung row (DETECT), so stating it never buys slack; `deadline-unchecked`
(warning) on every `deadline_by` until E4 evaluates checks, and the row
carries no verdict without an `ftti`; `hazard-no-requirement` (error) for
a hazard with neither; `derive-conflict` (error) for a settle derived by a
braking profile and as a derived duration; the SOURCE cell and a "Derived
values" block in `--explain` (`derived by <ext> <ver> (sha256 <12>)`, the
inputs and the trace); "Reaction totals" in `--explain`, each hazard's
floor total, which E4 hands to extension checks as `reaction_total`.
Fixtures `contract_derived_record` (40 stated + 100 derived detection, a
derived settle: 497 of 500 ms), `contract_deadline_by`,
`contract_derive_conflict`, `contract_hazard_no_requirement`; tests
`e1_*` in `tests/manifest_check.rs`. One choice the design left open:
`detection` is additive (physics, then what the detectors take), the
conservative reading, rather than replacing the derived detection.

### E2. rlx front end (rlm)

- The extension format (`extension`, `requires`, `key`, `input`, `derive`,
  `check`, `report`, `fn`, `where`, `worst_case`, `domain`, `dimension`),
  over E0's expression core.
- Character rules (5.3): NFC, identifier classes, confusables, reserved
  symbols, ASCII twins, bidi and zero-width refused; canonical ASCII form;
  `rlx fmt --ascii | --unicode`.
- Gate: a corpus of refused files, one per rule, each with its diagnostic;
  round-trip `fmt` is the identity on the canonical form.

**Done (2026-10-17).** rlm v0.1.65, crate `rlx/` (`ros-launch-manifest-rlx`,
lib `rlx`, binary `rlx check | fmt --ascii | --unicode`), about 3.4 kLOC
with tests, one dependency (`unicode-normalization`). Pipeline: the
file-wide character rules, the tokenizer (identifier and symbol rules),
a hand-written parser (first error only), then the tree refusals
(recursion; `rlx-shadow`, a binding named like a unit, since `2 h` with a
binding `h` would silently mean hours). Thirteen diagnostic codes; the
refused corpus has 29 files with exact expected lines and fails if a code
has no example. The six `.rlx` examples parse verbatim (a test fails if
they drift from the doc), each with a committed Unicode variant;
`fmt --ascii` is the identity on the canonical form and inverts
`fmt --unicode`. The formatter swaps operator spellings in place, so
comments and aligned columns survive. Grammar decisions the examples
left open (units against variables, implicit multiplication, `^` above
unary minus, `and` as binding separator or conjunction, clause order)
are in the crate docs; what the design leaves open for E3 (canonical
form against Greek identifiers, the one-script rule refusing `Δt`, how
an expression names a key, `param ... unit`) is in rlm
`extension-system.md` section 12. Reviewed against inputs outside the
corpus (recursion, a Cyrillic look-alike, a unit-named key, a bidi
override in a comment, round trips).

### E3. Units and load-time checks (rlm)

- Dimension checking in every branch at load; key dimensions against
  contract values (`entry_speed: 15` refused, `15 s` refused with both
  dimensions named, `km/h` converted).
- Gate: the exploration's unit-bug example (`sqrt(2 v0 / a)`) refused at
  load; every reference extension loads clean.

### E4. Evaluator and trace (play_launch)

- Evaluate derives and checks after core timing; feed derived bounds to the
  core rules; record the trace; `--explain` prints it (and `--pretty`).
- Gate: the seven domain examples print the derivations in
  `extension-examples.md`; core verdicts on every existing fixture unchanged.

### E5. Worst-case proofs (play_launch)

- Encode `worst_case` over `domain` for Z3 (monotonicity in each named
  input, with side conditions for division and `sqrt`); proved / refuted
  with counterexample / unknown; `--strict`.
- Gate: vehicle, medical, rail proved; the UAV claim refuted with the
  counterexample, then proved with the suggested domain; a deliberately
  wrong direction refuted.

### E6. Profiles (rlm, play_launch)

- The profile format; the core's severity rules read a profile's scale and
  criticality map; reference profiles `iso26262` (with decomposition as an
  rlx `fn`), `iec61508`, `iec61511`, `do178c`, `iec62304`, `iso13849`,
  `en50129`.
- Gate: derived criticality unchanged on every fixture under `iso26262`;
  a SIL fixture under `iec61508`.

### E7. Packaging, resolution and identity (play_launch)

- ament resource `rlm_extensions`; `use:` with semver ranges; bare and
  qualified keys; sha256 of the canonical form in `--explain`; `rlm.lock`,
  `--locked`, `--update-lock`.
- Gate: two packages declaring the same key force the qualified spelling;
  a changed extension under `--locked` is refused.

### E8. vehicle.braking and the migration (all)

- The bundled `vehicle.braking` extension; implicit load with
  `deprecated-implicit-extension`; the ASIL default moved to the bundled
  `iso26262` profile on the same schedule; `play_launch migrate` adds
  `use:`.
- Gate: every existing contract checks with identical verdicts and totals,
  with only the deprecation warnings; migrated contracts check clean.

### E9. Reference extensions (rlm or a sibling package)

- `machine.iso13855`, `medical.occlusion`, `uav.descent`, `rail.ebd`,
  `process.pst` as worked examples and test fixtures, each with its
  contract and expected `--explain`. Domain constants marked illustrative;
  standard clauses verified before any is called a reference.
- Gate: all load, prove their worst cases, and match the examples doc.

### E10. Export to nano-ros (play_launch, nano-ros PR)

- Derived bounds appear in the exported model as plain bounds with their
  source; nano-ros reads them unchanged.
- Gate: an image built from a contract using `vehicle.braking` has the same
  monitors and sizing as one with the values written by hand.

### E11. Docs and releases

- Format reference split into core and extensions; an extension author's
  guide; rlm minor release, play_launch release, nano-ros pin PR, consumers
  migrated.

## Order

E0 first (it decides E2's size). E1 and E2 in parallel; E3 after E2; E4
after E1 and E3; E5 and E6 after E4; E7 alongside E5/E6; E8 after E4, E6,
E7; E9 alongside E8; E10 after E4; E11 last.

## Not in this phase

- Export to SysML v2 or AADL.
- An editor extension (abbreviation input, hover traces); `rlx fmt` and the
  CLI first.
