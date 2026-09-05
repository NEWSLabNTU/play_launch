# Phase 69 — the field table: one source for the contract grammar

Status: **W0–W4 complete** (manifest crate `v0.1.22`).

Design of record: [docs/design/contract-primitives.md](../design/contract-primitives.md)
(fact vs requirement vs consequence) and
[docs/design/contract-axes.md](../design/contract-axes.md) (the completeness
survey). This phase adds no vocabulary and removes no vocabulary. It makes the
vocabulary **enumerable**, so that the two campaigns those documents describe
can be run mechanically instead of by hand.

## Why

Three findings, all measured on 2026-09-04.

### 1. A typo silently deletes a declaration

`types/src/parse.rs` is 1695 hand-written lines with **no unknown-key
rejection anywhere**. Injecting three typos into
`tests/fixtures/contract_rates/launch/bringup.contract.yaml` —

```yaml
max_latencyy: 5ms        # was max_latency
bogus_field: {a: 1}
rate_hzz: 100            # was rate_hz
```

— produced `1 manifest(s) checked: 1 clean, 0 with errors`, exit 0. A 5 ms
budget vanished with no diagnostic. Worse: the `info[derivable-rate]` line that
fires on the clean fixture **disappeared**, because the rule reads the field the
typo destroyed. The one check that would have pointed at the mistake was
silenced by the mistake.

This is the same class as issue #0020 (a stale pip install shadowing the real
binary) and the phase-64 stale-submodule misreading: the artifact looks right
and behaves wrong.

The opposite ruling already exists in this repository. The launch XML parser
carries per-element attribute allowlists
(`src/ros-launch-resolve/parser/crates/play_launch_parser/src/xml/attr_spec.rs`,
2026-07-31) that **error** on an unknown attribute and **warn** on a
known-but-unsupported one. The manifest parser has no equivalent.

### 2. The grammar lives in three places, none of them normative

| where | what it is | binds the input? |
|---|---|---|
| `types/src/types.rs` | structs + serde attributes | **no** — serde is used for *serialize only* |
| `types/src/parse.rs` | hand-written parser | yes, de facto |
| `docs/launch-manifest.md` §Format Reference | prose + examples | no |

Nothing checks the three against each other.

### 3. The drift is already six fields wide

Of 66 struct fields in `types.rs`, **22 are absent from the Format Reference**
and **six appear nowhere in the entire 1752-line document**:

`lease_duration`, `min_latency`, `concurrency`, `exclusive`, `max_count`,
`tolerate`

`lifespan`, `deadline`, `history` and `liveliness` are documented only outside
the reference section.

A "formal specification of the contract file format" is therefore not a writing
task. Nothing today can tell you such a document is complete, because nothing
enumerates the language.

### 4. A rejected manifest is silently dropped, and `check` still exits 0

Appending a top-level `chains:` block (removed in phase 68, and rejected by
`reject_chains` with a good message) to the same fixture produced:

```
WARN Failed to parse manifest .../bringup.contract.yaml: at 'chains': ...
```

`check` exited **0**, and the `Loaded 1 manifest(s)` line was gone — the file
parsed to nothing, so **every contract in it stopped being checked**, silently.

This inverts the value of any rejection. Today a stray key is ignored and the
rest of the file is still checked; adding an unknown-key error without fixing
the surfacing would trade that for "one bad key disables the whole file, and
nothing fails". Fixing the surfacing is therefore a **precondition** of W3, not
a follow-up.

## Scope

The normative artifact becomes a **field table**. Everything else is generated
from it or checked against it:

```
FIELD TABLE  ──┬─→ parse.rs allowlist     (unknown key = error)
               ├─→ Format Reference       (generated; a test fails on hand drift)
               └─→ consumer census        (phase 70)
```

Row shape:

| column | meaning |
|---|---|
| `yaml_name` | the key as written in a contract file |
| `aliases` | deprecated spellings (`max_latency_ms`, …) |
| `context` | which container it is legal in (`NodeDecl`, `PathDecl`, `QosDecl`, …) |
| `kind` | `fact` \| `requirement` \| `consequence` \| `advisory` |
| `consumer` | the rule, mapper arithmetic or runtime monitor that reads it |
| `doc_anchor` | where the Format Reference documents it |
| `status` | `live` \| `deprecated` \| `unimplemented` |

`kind` is the contract-primitives rule made mechanical. A row typed
`consequence` must name its derivation; a row typed `fact` or `requirement`
must name a consuming reader. `status: unimplemented` is the only way to have
neither, and it is a declaration that the gap is known.

**Not in this phase**: the consumer census itself (phase 70), the
derivation/agreement corpus metric, and the axis witness table. This phase
builds the table and the rejection; those consume it.

## Waves

### W0 — a parse error must fail the check (precondition)

A manifest that fails to parse is currently a `WARN` that leaves `check` at
exit 0 and removes the file from the run. A file the user asked to check and
that could not be read is an **error**: it must count in the error tally, be
reported where the other diagnostics are, and set a non-zero exit. Without this
W3 is a regression.

### W1 — measure the blast radius

Before choosing a severity, count what an allowlist would reject across the
whole corpus: every `*.contract.yaml` under `tests/fixtures/`, `examples/`, the
manifest repo's own fixtures, and Autoware where available. A hard error is
only available if the count is small, or if every hit is a real defect.

Deliverable: a canonical key-path inventory (user-chosen map keys collapsed to
`*`) and the diff against the de facto accepted set read out of `parse.rs`.

**Done, 2026-09-05.** 42 distinct contract files (11 `*.contract.yaml` under
`play_launch`, 28 `manifest.yaml` fixtures and 3 contract fixtures in the
manifest repo; `install/` copies deduplicated by hash). **132 distinct
canonical key paths** from 63 schema leaf names.

Exactly **one** key in the whole corpus is not read by anything:

```
nodes.*.srv.*.request          1 file, 1 occurrence
  ros-launch-manifest/tests/fixtures/manifest_ndt/manifest.yaml:40
```

`SrvEndpointProps` has one field, `max_response`. So an allowlist rejects
**1 of 42 files (2.4%) and 1 of 132 key paths (0.8%)** — a hard error with a
single fixture fix, not a deprecation window.

Two things the measurement also settled:

- Four near-miss pairs exist (`min_latency`/`max_latency`, `pub`/`sub`,
  `srv`/`sub`, `sync.tolerance`/`miss.tolerate`) and **all four are real,
  distinct keys**. The edit-distance suggestion must therefore be budgeted, or
  it will confidently propose the wrong field.
- The legacy `_ms` spellings are not a rump: `max_latency_ms` appears in **14
  files against `max_latency`'s 10** — the deprecated form is still the
  majority spelling. That is a phase-70 retirement question, not this phase's.

The platform-file side needs none of this: `sched/src/platform.rs` already
carries `deny_unknown_fields`. The contract parser was the outlier.

**W0 done, 2026-09-05.** A parse failure is now a `Severity::Error` in a new
`ManifestIndex::load_diagnostics`, kept apart from `merge_diagnostics` because
the file it names is ABSENT from `index.manifests` — every per-manifest tally
would otherwise count it as clean. It prints in its own section, *before* the
sections that tally only the files that loaded; it counts in `total_errors`;
it drives exit 1; and `resolve`/`dump` refuse to emit a SystemModel on it.
"No manifests found" and "found, but none could be read" are now different
verdicts.

### W2 — the table

`types/src/field_table.rs`: a `const` table, one row per (context, key). Chosen
over a YAML sidecar so that the table is compiled and a missing row is a build
break rather than a runtime surprise.

### W3 — rejection

`parse.rs` consults the table at each schema-key context. Unknown key at a
schema-key context is an **error** naming the context and the nearest accepted
key. Contexts whose keys are user-chosen (`nodes:`, `topics:`, `pub:`, …) are
explicitly exempt and marked as such in the table, so the exemption is data
rather than an omission.

Also fixed here: three typos must not report `1 clean`.

Depends on W0 — see finding 4.

### W4 — the generated reference

The Format Reference section of `docs/launch-manifest.md` is generated from the
table. A test fails when the checked-in document and the table disagree, which
is what makes the six-fields-wide drift unrepeatable.

**W2–W4 done, 2026-09-05.** `types/src/field_table.rs` carries 102 rows over
18 contexts. `parse.rs` calls `reject_unknown_keys` at every one; the ten
author-keyed containers have no `Context` variant, so the allowlist
structurally cannot reach them. `docs/format-reference.md` (198 lines) is
generated by `render_markdown()` and pinned by a test;
`UPDATE_FORMAT_REFERENCE=1 cargo test -p ros-launch-manifest-types`
regenerates it.

Four table invariants are tested rather than assumed: no key twice in one
context, every alias naming a live canonical key in its own context, every row
documented, and rows contiguous by context (a split context would render two
sections of the same heading).

Three details that turned out to matter:

- **A removed key keeps its own message.** `chains:` and `jitter:` have
  dedicated handlers that name the replacement; those run first and win. The
  generic check is the backstop for the ones that do not have a handler — which
  is how `segments:` was found. Phase 68 named it in the `chains:` migration
  text but never rejected it, so a stray `segments:` stayed silent.
- **The suggestion is budgeted, not nearest-wins.** The corpus contains four
  genuine near-miss pairs (`min_latency`/`max_latency`, `pub`/`sub`,
  `srv`/`sub`, `sync.tolerance`/`miss.tolerate`). An unbudgeted "did you mean"
  would confidently propose the wrong field; beyond the budget the diagnostic
  lists the accepted keys instead.
- **`parse_qos` had no `ctx`.** It now takes one, so a bad QoS key reports
  where it is rather than at the document root.

### What it found on the first run

`check/tests/checker_tests.rs::test_clean_pipeline` declared the two topics of
its scope path under **top-level `pub:`/`sub:` blocks that were never part of
the grammar**. They were silently discarded, so the scope path named topics
that did not exist. The test asserted `errs.is_empty()` and passed — for the
absence of a check rather than the success of one, the same shape as the three
vacuous tests phase 68 found. It now shrinks the budget below the pipeline
total and requires `scope-budget` to fire.

The one dead corpus key, `srv.<endpoint>.request` in `manifest_ndt`, was itself
a second copy of the type already declared on `services.trigger_ndt.type` —
the campaign's own failure mode, sitting unread in a fixture.

## Acceptance

0. A manifest that fails to parse fails the check, with a non-zero exit.
1. Every accepted key in `parse.rs` has exactly one table row, enforced by a
   test that fails on an orphan in either direction.
2. An unknown key at a schema-key context fails the check, with the context and
   a suggestion.
3. The corpus resolves byte-identically to before the phase — this phase
   changes what is *rejected*, never what an accepted contract means.
4. The Format Reference is generated, and the six undocumented fields are
   documented by construction.

## Not fixed here

Two failures pre-date this phase and were confirmed at baseline (same three
failures with this work stashed), so they are recorded rather than absorbed:

- `just check-layer2-isolation` fails on all three checks —
  `libplay_launch_parser_pyexec.so` is not built anywhere in the tree, so the
  driver has no Python half to `dlopen`. It belongs to the CPython-pin removal
  (`4798eab`, `a4104d7`), not to contract parsing.
- `pyload`'s `candidates()` had a collapsible `if` (clippy, `-D warnings`) and
  `pyexec/src/c_abi.rs` had rustfmt drift, both from the same commits. Fixed
  in passing because they gate `just check`; neither is this phase's work.

## Follow-ups this phase deliberately left

- **The `kind` and `consumer` columns** — phase 70's consumer census. Until
  then the table says what is *legal*, not what is *read*.
- **A second silent-loss class the table does not cover: wrong TYPES.**
  `max_count: 5` (an integer where a `"N / W"` string is expected) still
  parses to `None` in silence, and so do a bare-scalar `output:`, a
  non-boolean `lifecycle:`, and a quoted number in any `yaml_f64` field.
  The unknown-key check catches a misspelled key, not a mistyped value. The
  helper inventory taken during W1 lists every such site.
- **`trigger.timer` sibling keys** are now rejected, but `qos.reliability`,
  `qos.durability`, `qos.history`, `qos.liveliness` and `criticality` still
  accept **any string value** without validation. A closed value set is the
  same argument as a closed key set, one level down.
- **The legacy `_ms` spellings.** `max_latency_ms` is in 14 corpus files
  against `max_latency`'s 10 — the deprecated form is still the majority. The
  table now makes the retirement mechanical (`Status::DeprecatedAlias` names
  its canonical replacement); the decision is phase 70's.
