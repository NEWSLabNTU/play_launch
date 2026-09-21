---
id: 32
title: "`--enforce-rules strict` stops the run on the first violation of ANY severity, so `/rosout` ends it 57 ms in"
status: open
type: correctness
severity: medium
---

# 0032 - strict mode cannot tell a warning from an error

**Repo:** `play_launch` (0.11.0, `5eaa3191`)
**Affects:** `src/play_launch/src/runtime_enforcement/mod.rs:852-868` (`emit`),
`:903-912` (`emit_repeatable`), `:522-523` (the `graph-deviation-runtime`
emission at `Severity::Warning`); `src/play_launch/src/commands/up.rs:1581-1607`

## Symptom

Same setup as #0031 with interception on, `--enforce-rules strict`
(2026-09-18, 0.10.0 binary at `155ed78b`; non-ASCII transcribed):

```
$ play_launch launch --disable-web-ui --disable-monitoring --disable-diagnostics --container-mode stock --config cfg.yaml --contracts contracts --enforce-rules strict pure_nodes.launch.xml
2026-09-18T12:56:25.493707Z  WARN play_launch::runtime_enforcement: [runtime] unknown topic at publisher creation -- no manifest declares '/rosout' (hash 0xd46add98f57fdcea); add to `topics:` or `external_topics:` (graph-deviation-runtime)
2026-09-18T12:56:25.493770Z  WARN play_launch::runtime_enforcement: [runtime] unknown topic at publisher creation -- no manifest declares '/parameter_events' (hash 0x47c9785a1f548d47); add to `topics:` or `external_topics:` (graph-deviation-runtime)
2026-09-18T12:56:25.550878Z  WARN play_launch::commands::up: [runtime] Strict enforcement violated -- initiating shutdown
```

The run is over 57 ms after the talker's first publisher is created. The
`rate-hierarchy-runtime` ERROR that the contract was written to catch
(`min_rate_hz: 1000` against a 1 Hz talker) needs a 0.5 s window and never
gets one. `runtime_violations.jsonl` for the same warn-mode run shows the
severities the engine itself assigns:

```
{"rule_id":"graph-deviation-runtime","severity":"warning","fqn":"/rosout", ...}
{"rule_id":"graph-deviation-runtime","severity":"warning","fqn":"/parameter_events", ...}
{"rule_id":"rate-hierarchy-runtime","severity":"error","fqn":"/pure_test/chatter", ...}
```

## Cause

`emit()` at `mod.rs:866-867`:

```rust
if matches!(self.mode, EnforceMode::Strict) {
    self.strict_violated.store(true, Ordering::Release);
}
```

runs before `severity` is looked at; `emit_repeatable()` at `:910-911` is the
same. The `severity` argument only reaches the log line and the JSONL
record. The strict watcher in `up.rs:1581-1607` polls that one flag.

`graph-deviation-runtime` is emitted at `Severity::Warning` (`:522-523`) by
design (phase 36.5 made it "warn-only" because a launch tree is a subset of
the running graph). Every ROS 2 node creates `/rosout` and
`/parameter_events` publishers before user code runs, so any contract that
does not list both under `external_topics:` trips strict at node start.

## Impact

Strict mode is unusable as a CI gate on a real contract without first
listing two rcl-internal topics the author never wrote a requirement about,
and once one warning-class rule fires the error-class rules are never
evaluated. The engine's own severity vocabulary (`Info < Warning < Error`,
mirrored from the static checker) is not honoured by the one consumer that
acts on it.

## Fix direction

- Flip `strict_violated` only for `Severity::Error` (one-line change in both
  `emit` and `emit_repeatable`), keep warnings in the log and the JSONL.
- Optionally add `--enforce-rules strict --strict-on warning` (or a config
  key) for users who do want warnings to be fatal.
- Consider seeding the runtime view with `/rosout`, `/parameter_events`
  (and `/rosout_agg`) as implicitly external, the way the static checker
  already treats rcl-internal topics; then `graph-deviation-runtime` only
  reports topics the user could plausibly have declared.
- Test: in strict mode, a warning-severity emission must NOT set the flag;
  an error-severity one must.

## Provenance

Brief C (`brief-C-playlaunch-usage.md`, section 2.5, observation 1), runs of
2026-09-18 with the 0.10.0 binary at `155ed78b`; captures under
`scratchpad/playlaunch/runtime/{warn,strict,strict2}/`. Re-verified against
the 0.11.0 source at `5eaa3191` on 2026-09-21.
