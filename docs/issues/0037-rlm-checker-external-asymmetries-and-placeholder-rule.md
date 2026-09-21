---
id: 37
title: "rlm: `dangling-entity` ignores topic `external:`, `service-wiring` ignores service `external:`, and `consistency` is a registered no-op"
status: open
type: correctness
severity: medium
---

# 0037 - three in-crate rules in ros-launch-manifest do not honour the escape hatches the grammar offers

**Repo:** `ros-launch-manifest` (rlm), `origin/main` at `ea5cbea`; filed here
because rlm has no tracker (README convention). Consumer in this repo:
`src/ros-launch-resolve/resolve/src/ros/manifest_loader.rs`.
**Affects:** `check/src/rules/dangling_entity.rs:29-44`, `:47-56`, `:72-80`;
`check/src/rules/service_wiring.rs:19-37`; `check/src/rules/consistency.rs:14-16`;
`check/src/rules/mod.rs:68-93`; `types/src/types.rs:453-458`

## Symptom

Three asymmetries, each checkable by reading the rule beside the type it
consumes.

1. `TopicDecl` has `external: Option<ExternalSide>` (`types.rs:453-458`,
   "the matching side (`pub`/`sub`/`both`) is treated as expected-external").
   `dangling_entity.rs:29-44` warns on `publishers.is_empty()` and on
   `subscribers.is_empty()` without reading `topic.external`. The same rule
   DOES read `svc.external` and `act.external` for services and actions
   (`:48`, `:60`, via `server_is_external` at `:78`). So

   ```yaml
   topics:
     /vehicle/status:
       type: autoware_vehicle_msgs/msg/VehicleStatus
       external: pub
       sub: [watchdog/status]
   ```

   still gets `warning[dangling-entity]: topic '/vehicle/status' has no
   publishers (no data source)` from the in-crate pass. The consumer's
   cross-scope re-run of `dangling-entity` honours the mark
   (`manifest_loader.rs:351-357`, "`dangling-entity` skips that side"), so
   the same file gets the warning from one pass and not the other.

2. `service_wiring.rs:19-37` builds `served` from services whose `server`
   list is non-empty and warns for every `cli:` endpoint not covered. A
   service declared

   ```yaml
   services:
     /localization/initialize:
       type: ...
       external: server
       client: [supervisor/init]
   ```

   is the exact case `dangling_entity.rs:8-14` describes as normal ("a
   client and its server are often two images") and exempts; `service-wiring`
   warns on it anyway: `service client 'init' has no matching server in
   services:`. Two rules in the same registry disagree about whether the
   file is fine.

3. `consistency.rs:14-16`:

   ```rust
   fn check(&self, _manifest: &Manifest, _graph: &DataflowGraph, _ctx: &mut CheckContext) {
       // Placeholder - cross-entity consistency checks will be added in phase 34.5.
   }
   ```

   is registered in `default_rules()` (`mod.rs:82`) and counted in the
   documented "20 rules". The real `consistency` rule is the consumer's
   cross-scope one. A reader of the registry, or of `--rule consistency`
   output, is told a rule ran that does nothing; phase 34.5 is long past.

## Impact

(1) and (2) push authors toward the two workarounds the `dangling-entity`
header explicitly calls out as bad: declaring a server or publisher the
image does not run, or leaving the entity out of the contract. On the
island contract the four Zephyr nodes' inputs are all external to the
launch tree, which is precisely the case. (3) is a false count in every
doc that says "20 rules" and in the `--rule` filter surface.

## Fix direction

- `dangling_entity.rs`: skip the publisher warning when
  `topic.external` is `Pub | Both`, the subscriber warning when `Sub | Both`;
  mirror the `server_is_external` predicate. Also consult the manifest's
  `external_topics:` block for the FQN, since that is the other spelling of
  the same fact.
- `service_wiring.rs`: treat `svc.external` in `{Server, Both}` as served.
  Consider merging the rule into `dangling-entity`'s service branch; they
  answer the same question from two ends.
- `consistency.rs`: remove from `default_rules()` (and from the "20 rules"
  count, which becomes 19), or give it a body. If the id must stay
  reserved so the consumer's `rule_id` is documented, keep a doc entry, not
  a registered rule.
- Fixtures: one per asymmetry, asserting no diagnostic where the escape
  hatch is used.

## Provenance

Brief A (`brief-A-rlm-grammar.md`, section 3.1, rule rows 9, 11 and 13),
2026-09-18; re-verified against rlm `origin/main` at `ea5cbea` (worktree
`rlm-gaps`) on 2026-09-21.
