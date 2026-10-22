# Phase 91 - domain extensions: rlx, profiles and the vehicle keys moved out

Status: **in progress** (2026-10-22): E0 done (no-go: rlx owns its
expression core); E1 and E2 done (rlm v0.1.65); E3 and E4 done (rlm v0.1.66);
E5 done (rlm v0.1.70). Stage 2 of 2; phase 90 (a tidy grammar) is complete. Design: rlm `docs/design/extension-system.md` (with
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

**Done (2026-10-17).** rlm v0.1.66 (`4a2ad1f`, in the `rlx` crate). Dimensions are integer exponents over the seven SI base
dimensions; a built-in table names the common ones and `dimension X = ...`
adds more (any order; a cycle, an unknown name or a redefined built-in is
refused). Each unit has a dimension and an f64 factor to SI; affine
temperature units are refused. Every expression in every branch is checked
at load, used or not: the exploration's `sqrt(2 v0 / a)` is refused with
"the argument is Time; the result should be a Time, so the argument should
be Time^2". Name resolution: binding, `sum over` variable, `fn` parameter,
then a key or input by its last segment (or full path); two keys sharing a
last segment are `rlx-ambiguous-key`, so a bare name is never ambiguous.
`reaction_total : Time` is built in. The value API checks a contract value
against its key type: `30 km/h` converts to 8.33 m/s, a bare `15` for a
Velocity "needs a unit", `15 s` names both dimensions, a table's keys must
increase. All six examples load clean, verbatim (none is dimensionally
wrong); the refused corpus has 51 files, 22 new; every value the example
contracts write parses to the SI value their arithmetic uses. Reviewed
against inputs outside the corpus (an error in an uncalled `fn`, `min` over
mixed dimensions, a domain bound in the wrong dimension). A derive may
target only `settle.duration`, `hazard.detection` or `hazard.ftti`.

### E4. Evaluator and trace (play_launch)

- Evaluate derives and checks after core timing; feed derived bounds to the
  core rules; record the trace; `--explain` prints it (and `--pretty`).
- Gate: the seven domain examples print the derivations in
  `extension-examples.md`; core verdicts on every existing fixture unchanged.

**Done (2026-10-17)**, with two examples left to E8 and one to E6 (below).
rlm v0.1.66: `use: { <name>: "<req>" }` (`VersionReq`, cargo's `^ ~ = >=`)
and the keys extensions declare, kept as written in `HazardDecl.ext` and
`SafeState.settle_ext` when the contract has `use:` (an unknown key is
still an error without it; an inline include's `use:` scopes its own
keys); the evaluator `rlx::eval` (built by a subagent): `eval_derive`,
`eval_check`, `eval_report` with the domain checked first, eight
evaluation errors with spans and no panics, a deterministic trace,
`Env::reads`, `param_value`, and `identity` (sha256 of the canonical ASCII
form). play_launch `ros/extensions.rs`, in the order the design fixes:
`prepare` before the fault walk (find extensions in the contract's `rlm/`
and `share/<pkg>/rlm/` on `AMENT_PREFIX_PATH`, highest matching version;
refuse unknown keys, bad values, missing extensions; derive
`hazard.detection` and `hazard.ftti` into the hazard as bounds with a
derivation record, so every core rule, the model and `--explain` read them
as written ones), `derived_settle` in the settle lookup (per hazard, since
a settle may read the hazard's keys), and `run_checks` after the walk
(checks and reports with `reaction_total`, the floor's total). Hazard
`param` keys read the guard publisher's parameter, settle `param` keys the
safe state's node's.

Gate: four of the design's examples run as fixtures on contract_fault's
chain, each printing its derivation: `contract_ext_machinery` (safety
distance PASS as the hazard's `deadline_by`, 611.2 mm <= 650 mm),
`contract_ext_medical` (detection 50.48 min derived, the ftti fails, the
bolus report), `contract_ext_uav` (settle 50952.38 ms derived, energy check
PASS), `contract_ext_process` (ftti 4 s derived and judged against);
`contract_ext_unknown` pins the refusals. Core verdicts on every existing
fixture are unchanged (the ranked-plan snapshot only gains the new
fixtures; `test-all` green). Left: the vehicle and rail examples need E8,
because the core still parses `entry_speed` (a plain m/s number) and
`decel`/`jerk`; the aerospace example is a profile (E6); `--pretty` is not
implemented (traces print in canonical ASCII). The evaluator found two
defects in the examples, fixed in rlm: process.pst's `trip_setpoint` lacked
`unit bar` (9 read as 9 Pa, ftti 22 s), and the UAV contract stated none
of the settle's parameters.

### E5. Worst-case proofs (play_launch)

- Encode `worst_case` over `domain` for Z3 (monotonicity in each named
  input, with side conditions for division and `sqrt`); proved / refuted
  with counterexample / unknown; `--strict`.
- Gate: vehicle, medical, rail proved; the UAV claim refuted with the
  counterexample, then proved with the suggested domain; a deliberately
  wrong direction refuted.

**Done (2026-10-22).** rlm v0.1.70 (`b28353e`); semantics and encoding in
rlm `extension-system.md` section 6.1. Two placement choices against the
unit's heading: the encoder lives in rlx (`rlx::prove`, feature `smt`,
default on, z3 0.12 as the checker's), beside the evaluator whose name
resolution and definedness it must match, and not in play_launch; and the
gate's "Z3 over the declared domain" is read strictly: the prover assumes
only what the declarations state (`magnitude` is not negative, table keys
increase), so a claim needing a sign the extension does not write is
refuted and the checker names the sign. play_launch (`ros/extensions.rs`,
`worst_case`) proves each item at the contract's point where it is
evaluated: tables at the keys the contract writes, the point filtering the
suggested domain, memoized per (extension sha256, item, point).
`worst-case-refuted` (error), `worst-case-unknown` (warning; `check
--strict`, new on both CLIs, makes it an error), and a `worst case: proved
...` line in the trace and the derivation record. Without `smt` every
stated claim is `worst-case-unknown` and no Z3 is linked (checked with a
`default-features = false` build).

Gate, as run:

- Vehicle, medical, rail proved: rlx `tests/prove.rs` (braking 4 claims;
  rail's 3 at its written keys, its settle also at 1 to 3 symbolic
  entries); play_launch's `contract_ext_medical` prints `worst case: proved
  decreasing in min_rate; increasing in compliance, alarm_threshold`. The
  vehicle and rail fixtures in play_launch still wait for E8.
- UAV refuted, then proved with the suggestion:
  `e5_uav_claims_as_written_are_refuted_with_the_domain_that_holds`
  (`contract_ext_uav_refuted`, five refutations, slow_altitude's help
  `land_speed <= descent_speed, land_speed > 0 m/s`; `max_altitude <=
  slow_altitude` also proves it and is withheld, since the contract's 120 m
  and 10 m are outside it), and `contract_ext_uav` with those domains
  added proves all five; rlx's own test adds every suggestion to the file
  and proves the result.
- A wrong direction refuted: braking with `decreasing in entry_speed`
  (rlx `a_wrong_direction_is_refuted_...`). Further controls: a domain
  made empty reads `vacuous`, never proved; a fractional power reads
  unknown (`contract_ext_unproved`, a warning, two errors under
  `--strict`); a dropped `compliance > 0` is refuted with exactly that
  suggestion.

Found on the way: four of the six reference extensions did not prove as
written (a negative compliance, rise rate, deceleration or delay turns
each direction over), and Z3's solver `timeout` is not honoured by nlsat
(a braking claim ran past two minutes under five seconds), hence the
watchdog. Rail's distance check at three symbolic entries is not decided
in two minutes; at its written keys it is decided in milliseconds, which
is why a consumer passes them.

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
