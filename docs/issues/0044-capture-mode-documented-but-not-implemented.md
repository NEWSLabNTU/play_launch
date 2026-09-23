---
id: 44
title: "the manifest spec documents a capture mode (`--save-manifest-dir`) that no binary accepts"
status: open
type: correctness
severity: low
---

# 0044 - a documented workflow with no flag behind it

**Repo:** `play_launch` at `af0aeab1`, and the `ros-launch-manifest` spec.
**Affects:** `docs/launch-manifest.md` (capture-mode section);
`src/play_launch/src/cli/options.rs`; `src/ros-launch-resolve/cli/src/options.rs`

## Symptom

The specification describes a capture workflow in which a run writes the
manifest it observed, so an author can start from a real system rather than
from a blank file. The flag it names is `--save-manifest-dir`.

```
$ grep -rn "save-manifest-dir\|save_manifest_dir" --include="*.rs" src/
$
```

No binary accepts it, and no code writes a manifest from a run.

## Impact

Small but specific: the capture workflow is the answer to "how do I get a
first contract for a system that has none", which is the question a new user
asks first. The documented answer does not exist, and the nearest real thing
(`play_launch check --emit diagnostics-params`, which prints
`diagnostic_updater` parameters from declared bounds) solves a different
problem.

`play_launch measure` is the closest shipped relative — it turns a recorded
run into a platform-file `overrides:` fragment, and phase 70 W5 extended it to
print `min_latency` floors as contract-shaped comments. So the machinery for
"observe a run, emit contract text" exists; it just does not emit a manifest.

## Fix direction

One of:

- **Implement it** on the back of `measure`: the interceptor already records
  every publisher and subscription created (`interception/endpoints.tsv`,
  phase 77) plus per-message events, which is enough to emit `nodes:`,
  `topics:` with wiring, and observed rates. The honest output marks every
  number as observed rather than required, since a measurement is a fact and
  a contract states requirements — the same distinction `measure` already
  draws by printing costs as comments under a header saying where they belong.
- **Delete the section**, and point instead at `measure` plus the worked
  example in the spec.

Either way the spec must stop naming a flag that does not exist. Phase 77's
`scripts/verify_graph.py` is prior art for the first option: it already grades
a model against `endpoints.tsv`, so the inverse direction is a short step.

## Provenance

Found 2026-09-23 during the v0.1.39 documentation pass, by two independent
agents reading the spec against the CLI.
