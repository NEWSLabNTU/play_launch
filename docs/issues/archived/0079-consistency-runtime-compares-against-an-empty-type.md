---
id: 79
title: "consistency-runtime reports every remap-derived topic as a type mismatch: it compares the runtime type against an empty declared type"
status: resolved
type: defect
severity: medium
---

# 0079 - `consistency-runtime` compares against an empty declared type

**Repo:** `play_launch` at `8d2c78d4` (0.29.0; seen on a build from `11bf33af`)
**Affects:** `src/play_launch/src/runtime_enforcement/mod.rs`
(`RuleEngine::check_consistency`)

## Symptom

Autoware 1.5.0's planning simulator under `play_launch launch` with the
fixture's overlay contracts (`tests/fixtures/autoware/contracts`), driving
to a goal, wrote 100 `consistency-runtime` errors to
`runtime_violations.jsonl` in about three minutes:

```
{"rule_id":"consistency-runtime","severity":"error","fqn":"/control/command/hazard_lights_cmd",
 "message":"topic '/control/command/hazard_lights_cmd' runtime msg type (hash 0x8a145102d41d45dc)
 disagrees with declared '' (hash 0xcbf29ce484222325)", ...}
```

`0xcbf29ce484222325` is the FNV-1a offset basis, the hash of the empty
string. The runtime types were all correct.

## Cause

Since phase 76 the model's `structure.topics` also holds topics derived
from a launch file's remaps. A remap carries no message type, so those
topics have `type: ''`. On this Autoware model, 106 of the 110 topics are
untyped. `ContractView::from_model` copies the empty string into
`TopicView.msg_type`, and `check_consistency` skipped a topic only when it
was absent from the view or typed `TODO/...`. So it compared every
runtime type hash against `fnv1a("")` and reported a mismatch, at
`error` severity, for each endpoint of each such topic.

The unit test declared a type, so it could only see the case where a type
exists.

## Fix

`check_consistency` treats an empty declared type as undeclared and skips
it, as it already did for an absent or `TODO/` one. The test is now a
helper that runs the same publisher init against a typed topic and an
untyped (remap-derived) one:

- `consistency_runtime_fires_on_type_mismatch` still fires.
- `consistency_runtime_ignores_a_topic_with_no_declared_type` is new. It
  fails without the fix (negative control run) and passes with it.

## Not changed

The same run wrote 481 `graph-deviation-runtime` warnings ("no manifest
declares '/tf'"). These are expected for a system where only 4 of 83
scopes have a contract. They are noise worth a separate look, but not
wrong.
