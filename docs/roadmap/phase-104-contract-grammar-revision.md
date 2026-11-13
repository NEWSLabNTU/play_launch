# Phase 104 - the contract grammar revision

**Status (2026-11-14): S1 done** (rlm v0.3.15 and v0.3.16). The two items
left for a decision were taken on 2026-11-14: `tolerance` is removed
(v0.3.16; `migrate` folds it into an existing `sync:` only, and reports a
path without one, because adding `sync:` turns the derived rate from a sum
into a minimum), and an inline subscriber `max_transport` is deprecated in
favour of links (`deprecated-spelling` at v0.3.16, refused at the next
release, held to it by rlm's `field_table::DEPRECATED_USES`). The refusal
release must migrate the fixtures that still write it inline:
`contract_call_supervised`, `contract_takeover_chain_budget`,
`contract_takeover_gate_jitter`, `contract_takeover_jitter_short`,
`contract_takeover_link` (the inline twin of `contract_takeover_named_link`,
whose `control_mode` is inline too), `contract_transport_notice`,
`endpoint_transport` and the two `contract_spelling_deprecated*`.
S2 decided with phase 103 D4/D5. S3 and S4 planned. Design
of record: rlm `docs/design/grammar-revision/` (an overview and one file per
topic, T1-T10, each with the verified issues, counts over 192 contracts,
every method compared, and a recommendation). This phase orders that work.
The defects the same review found in shipped behaviour are phase 103, which
comes first.

## Who does what

rlm is the runtime-agnostic specification and owns every grammar, model and
meaning change below. play_launch (the Linux realizer) and nano-ros (the RTOS
realizer) follow each rlm release: a migrated corpus, any reader that
changes, and their own run-time half where a topic has one.

What makes a release cheap for nano-ros: it writes none of the keys being
renamed, and the model keeps its own field names (`max_latency_ms`,
`scope_paths`). **A contract-key rename costs nano-ros a tag bump and a
`migrate` pass over its 33 contracts; a model-field rename costs about 8 of
its files and a C ABI string.** So every rename here is of contract keys
only, unless a unit says otherwise.

## Stages

| Stage | Units | rlm | play_launch | nano-ros |
|---|---|---|---|---|
| **S1 removals** (one release) | T1: refuse the five aliases deprecated at v0.1.59/60 (`external_topics:`, `topics.<t>.max_transport`, `paths.<p>.input`, `exit.on`, a mode named in `hazards.<h>.reaction`); put `sub.<ep>.max_transport` inline (`transport-inline`) on the schedule; delete `links.monitor` (grammar and model); a deprecation registry with the release of each deprecated key, value and rule id, and a test that fails once one is overdue. T2: delete `paths.<p>.tolerance`, with a `migrate` that folds it only into an existing `sync:` (folding into a new one would turn a fan-in's derived rate from a sum into a minimum). | All of it. | Migrate its fixtures (two still rely on the aliases); `field_census.py` stops counting unrelated identifiers named like a field. | Tag bump; nothing it writes changes. |
| **S2 meanings** | Phase 103 D4 (entering a mode) and D5 (the omitted `on:`) decided and implemented. | - | - | - |
| **S3 renames and relocations** (one R release, then R+1 and R+2 per rlm `standard-names.md` §6) | T5: `reaction` split by target (a mode is `enters:` everywhere, as hazards and `supervise` already write; a chain or path takes one other key); `functions` renamed (it is a monitored capability, not IEC 61508's safety function). T6: rates as values with a unit (`min_rate: 100Hz`) through rlx's quantity parser, which already refuses a bare number and a duration. T4: `supervise.early` to `vehicle.braking` for the static check, with a domain-free run-time form (`observes: { input, field, reached }`, `early: { within, decrease }`) so a realizer that does not run rlx can still enforce it; `supervision: { tick, switch }` to the platform file; `policy` replaced by a generic `check --deny <rule>`; `settle.response` reworded. T3: `services.<s>.max_transport` replaced by `links` (0 uses); `integrity_levels` migrated to `use:` of the matching profile (22 of 23 equal the default). | Grammar, `migrate`, the extension contexts beside `hazard.`/`settle.`, the field table, `format-reference.md`. | Migrate fixtures, examples and guides; `supervision` read from the platform plan (the overrun rows are built in `load_manifests` before `check` resolves the platform file, so it must be carried in, as `ContractSources.lock` is); the run-time early check. | Tag bump and `migrate`; its `system.toml` already states the tick per board; the run-time early check if the image enforces one. |
| **S4 additions** (each when a contract needs it) | T7: end-to-end protection on a topic, `protected: { counter { field, wraps, max_delta }, id { field \| value }, crc }`; the counter replaces the stamp for repetition and resequencing; `corruption` only with `crc` or an application mechanism. T8: a quoted scalar is a string, an unquoted upper-snake-case one a constant (yaml-rust2 exposes the style). T9: action endpoints mirroring `srv`/`cli` (`act_srv`, `act_cli`), with cancel as a reaction edge; transforms as a publisher's frames plus a consumer's `transforms.<pair>.max_age`. T10: `check --system hosts.yaml`, each host's launch, arguments and platform as check input, each model stamped with its host, cross-model rules over the set. | Grammar and model for each. | Static rules; run-time halves (the counter, TF age at the consumer, transforms from `TfEdge`). | The image's half of T7 (counter check) and T10 (it reads one model per board). |

T8's quoting rule is small and may ride S3. T2's topic wiring stays a
census (`derivable-wiring`) until remap-derived wiring covers more than
Autoware.

## Decisions needed

Each topic file ends with its open questions; the ones that block a stage:

- S1: what an overdue deprecation is measured against: an rlm tag, a
  play_launch release, or nano-ros's pin (§6 says all three).
- S3: the key for a chain or path where `reaction` named one; the new name
  for `functions` (`capabilities`, `signals`); whether root `paths:` becomes
  `event_chains:` (rlm `standard-names.md` S5 decided to keep it; the model
  says `scope_paths`, a retired word, in about 147 identifiers); whether
  `supervision` goes to the platform file or a target-independent file.
- S4: per addition, the first contract that needs it.

## Acceptance

- Every stage: the corpus of all three repositories and the safety island
  migrated, with zero `deprecated-spelling` in any gate.
- S1: the registry's test fails when a deprecated row is older than its
  schedule (negative control by back-dating one).
- S3: every renamed key refused at R+1 with a message naming its
  replacement, one negative case per key.
