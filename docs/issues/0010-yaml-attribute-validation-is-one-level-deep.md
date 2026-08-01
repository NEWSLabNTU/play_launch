---
id: 10
title: "YAML attribute validation stops at the top-level action, so nested param/remap/env/composable_node keys are unchecked"
status: resolved
type: bug
severity: low
---

# 0010 — YAML attribute validation is one level deep; XML is two

**Repo:** `play_launch_parser` (`src/ros-launch-resolve/parser`)
**Affects:** `src/traverser/yaml.rs`, `src/xml/attr_spec.rs`
**Found:** whole-branch review of the `machine=` removal (2026-08-01)

## Summary

The per-element attribute allowlists are enforced at two levels on the XML
frontend and one level on the YAML frontend.

XML validates the dispatched action (in `traverser/entity.rs`) **and** each
action module's own `for child in entity.children()` loop, so `<param>`,
`<remap>`, `<env>`, and `<include>`'s `<arg>` are all checked.

YAML calls `validate_yaml_keys` once, on the top-level action mapping
(`traverser/yaml.rs`, in `process_yaml_actions`). Nested mappings — `param:`,
`remap:`, `env:`, `composable_node:` — are never validated. In particular the
`composable_node` spec has **no enforcement path at all** on the YAML
frontend.

So this is caught:

```yaml
launch:
  - node: { pkg: demo_nodes_cpp, exec: talker, zzz_bogus: x }   # rejected
```

and this is not:

```yaml
launch:
  - node:
      pkg: demo_nodes_cpp
      exec: talker
      param:
        - { name: p, value: 1, zzz_bogus: x }                    # accepted
```

## Related: non-mapping action bodies skip validation entirely

`process_yaml_actions` guards with `if let Some(map) = action_map`, so an
action whose value is not a mapping (`- node: null`, `- node: "bogus"`)
produces neither a validation error nor a parse error at this layer — it is a
silent no-op. Every other arm in that function already treats a non-mapping
value the same way, so tightening only the validation call would be
inconsistent; the whole malformed-body case wants one decision.

## Why it is low severity

Neither gap makes the parser accept something it then mis-executes; they are
missed *rejections* of input that ROS 2 would refuse. The XML frontend, which
is the primary path and what Autoware uses, is fully covered.

## Fix direction

Thread `validate_yaml_keys` through the nested-mapping handlers
(`process_yaml_node`'s `param:`/`remap:`/`env:` loops, and the
`composable_node:` handler), using the same specs — `AttrSpec.children`
already distinguishes "legal child element" from "legal attribute" precisely
so both frontends can share one table.

Decide the malformed-body case separately: either reject a non-mapping action
value outright, or document the no-op as intended.

## Resolution (2026-08-01)

`validate_yaml_child_seq()` added and called at all seven nesting sites:
`node`'s `param`/`remap`/`env`, `include`'s `arg` (spec `include-arg`),
`executable`'s `env`/`arg` (spec `executable-arg`), the shared
`composable_node` sequence, and `composable_node`'s own `param`/`remap`.

Note the spec name is not always the YAML key — ROS 2 treats `<arg>` under
`<include>` and under `<executable>` as disjoint entities, and the existing
specs already modelled that, so the helper takes the spec name separately.

The malformed-body case was decided rather than documented: a non-mapping
action body now errors, naming the kind found (`expected a mapping of
attributes, found a string`). It applies only to action types that HAVE a
spec, so an undispatched type still warns rather than failing.

Six tests added, verified to fail without the fix — exactly the five
rejection tests fail; the "accepts valid nested children" test passes either
way. Parser commit `d4035c8`.

**Not closed by this:** the differential oracle covers XML only, so these
YAML tables have no ROS-2-measured safety net. That is the same structural
gap as the `executable-arg` exclusion and belongs with #0012.
