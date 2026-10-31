# Phase 96 - closing the nano-ros seams

**Status: 🔄 in progress (2026-10-31).** Spans rlm (one breaking tag,
v0.3.0), play_launch, and nano-ros (pull requests; we own the repository).

## Why

Four items waited on nano-ros, and one doc sweep was stale:

- phase 85 D4, route versus callback: the checker charges a rung a ROUTE
  (detection fire to the rung's first output), and the image's runtime
  monitor measures one dispatch. The two halves of the contract state
  different quantities and nothing says so.
- phase 95 decision 4: that a client's `timeout:`/`on_timeout:` derive no
  deadline or monitor on the image, and whether the image's servers see a
  request's source timestamp (`stale_after:`).
- phase 63 W6: the duration aliases (`budget_us`, `spin_period_us`,
  `lifespan_ms`, ...) and the leniency that reads a bare number in the
  legacy unit. "Deprecated" with no removal is forever.
- phase 41.6: the legacy `system.toml` scheduling bridge, kept "until
  nano-ros migrates".

What the nano-ros survey found (origin/main de7de3d0bc, 2026-10-30):

- nano-ros never calls the legacy bridge (`parse_legacy_toml`,
  `parse_system_sched`) and passes no `--sched`. Its own `system.toml` is a
  different format; its `[tiers.*]` block does reach rlm, through
  `--system` and `system_config`, as `sched::TierDef`, and every such table
  in the tree already writes units (`spin_period = "10000us"`).
- nano-ros keeps a mirror of `TierDef` (`nros-orchestration-ir`) whose
  field NAMES are the legacy `_us` spellings and which parses through rlm's
  lenient `duration::compat::opt_micros`. W6 cannot land in rlm alone.
  21 test lines in 6 files feed legacy keys with bare numbers.
- the image already has route monitors (`route-latency-runtime`,
  `route-deadline-runtime`) and `hazard-reaction` (a monitor tick that sees
  the safe state's publisher count move). It reads `HazardContract.
  reaction` as one path: the model carries no ladder, and
  `ReactionEvent` carries no timestamp, though every call site has
  `now_us` in scope.
- nothing in nano-ros reads `timeout_ms`, `on_timeout`, `on_rejected`,
  `max_transport`, `idempotent` or `stale_after`. Servers see
  `ServiceInfo.source_timestamp` under zenoh and Cyclone; XRCE-DDS fills
  none.

## W1 - the stale docs (play_launch)

- [x] Roadmap README rows for phases 85 and 95, phase 85's "still open".

## W2 - rlm v0.3.0 (breaking)

- [x] W2.1 (41.6) Delete the `.toml` bridge: `bridge.rs`,
      `PlatformFile.legacy`, `MapperInput.legacy`,
      `MapError::MissingLegacySpec`. A `.toml` platform file is refused with
      a message naming the v2 form. `mapper: manual` becomes reachable from
      YAML: it derives nothing, and `overrides:` place the nodes, which is
      what "manual" means without the tiers+assign tables.
- [x] W2.2 (63 W6) Remove every `_us`/`_ms` serde alias and make a bare
      number under a duration key an error that asks for a unit, on every
      authored input (platform file, `system.toml` tiers, contract). The
      lenient `compat` readers go; a strict serde reader replaces them for
      consumers that mirror rlm's types. A model written before the
      Duration migration still loads: `SystemModel::from_yaml_str` rewrites
      the legacy keys of `execution.tiers` (a bare number there is µs) the
      way the schema-1 upgrade rewrites lists.
- [x] W2.3 (D4) `HazardContract.rungs`: per rung the checker's route
      (`rung`, `role` rung/window/floor, `route_ms`, `output` (the topics
      the route ends on), `starts_at` (the node whose detection fires it)),
      a consequence written by the resolver, never authored. Additive.
      *(2026-10-31, rlm v0.3.0, cc547e5. GRAMMAR_VERSION 0.3.0: the contract
      grammar did not change, but rlm's test pins it to the newest release.
      `resolve()`, `AssignRule` and `TierDef` stay as library code; the
      FILE format went. The `_us`/`_ms` aliases were also on rlm's contract
      types (`QosDecl`, `PathDecl`, `Sync`), whose hand-rolled parser already
      refused them; their serde paths now do too.)*

## W3 - play_launch on v0.3.0

- [x] W3.1 `just bump-manifest v0.3.0`; the resolver writes `rungs` from
      the fault-reaction rows (`--explain`'s table, the same numbers).
- [x] W3.2 `rt_workspace`'s `system.toml` becomes a `manual` v2 file; the
      tests that pass `--sched system.toml` move to it, and one asserts the
      `.toml` refusal.
- [x] W3.3 Guides and CLAUDE.md teach `budget:`, never `budget_us:`.
      *(2026-10-31. What moved: `rt_workspace/system.toml` became
      `manual.system.posix.yaml` (its `scope = "/perception"` rule is one
      override per node there); `contract_w2`'s platform file still wrote
      `budget_us:` inline, which the first grep missed; `sched_apply.rs`
      and `sched_plan.rs` wrote legacy files in test strings. An override
      places a node in a tier named after the node, so the tests that named
      the tier `rt` now name the node. The model's `rungs` for
      `contract_takeover_link` match `--explain`'s ROUTE column
      (`the_model_carries_each_rungs_route`). `starts_at` is in the census
      baseline until nano-ros reads it.)*

## W4 - nano-ros pull requests

- [ ] W4.1 Bump rlm to v0.3.0 and the play_launch submodule; rename the
      `TierDef` mirror's fields to the canonical keys with a strict,
      unit-required parser; rewrite the two-spelling test as a refusal test;
      migrate the 21 test lines.
- [ ] W4.2 (D4) A route monitor per rung: stamp the fire where
      `hazard-detected` fires (`now_us`), stamp the first publish on the
      rung's output, judge against `rungs[].route_ms`, report
      `hazard-route-runtime`; `ReactionEvent` gains the fire timestamp.
- [ ] W4.3 (95 decision 4) Record the agreement where nano-ros documents
      what it derives from the model (the client facts derive nothing), and
      refuse or warn on `stale_after:` for a server on a backend that fills
      no source timestamp (XRCE-DDS).

## Decisions

- One breaking rlm tag for W2.1 and W2.2, so nano-ros migrates once.
- A model is read backward-compatibly (an old saved model loads); an
  authored file is not (it is refused with the canonical spelling).
