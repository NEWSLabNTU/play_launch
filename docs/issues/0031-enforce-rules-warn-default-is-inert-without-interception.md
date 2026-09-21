---
id: 31
title: "`--enforce-rules warn` is the default and does nothing unless `--config` turns interception on, silently"
status: open
type: correctness
severity: high
---

# 0031 - the default runtime enforcement mode enforces nothing, and says so nowhere

**Repo:** `play_launch` (0.11.0, `5eaa3191`)
**Affects:** `src/play_launch/src/commands/up.rs:607`, `:1400`, `:1404-1415`;
`src/play_launch/src/cli/options.rs:744-750`; `src/play_launch/src/cli/config.rs:461-464`

## Symptom

A user who ships a contract beside the launch file and runs

```
$ play_launch launch --contracts contracts pure_nodes.launch.xml
```

gets `--enforce-rules warn` (the documented default) and no runtime
violation is ever reported, no `runtime_violations.jsonl` is written, and
nothing on the terminal says the contract is not being watched. The same
command with a config file

```
interception:
  enabled: true
```

passed through `--config cfg.yaml` produces (2026-09-18, 0.10.0 binary at
`155ed78b`, `demo_nodes_cpp` talker at 1 Hz against an overlay contract
demanding `min_rate_hz: 1000`; non-ASCII in the tool's own output
transcribed to ASCII):

```
$ play_launch launch --disable-web-ui --disable-monitoring --disable-diagnostics --container-mode stock --config cfg.yaml --contracts contracts --enforce-rules warn pure_nodes.launch.xml
2026-09-18T12:55:15.717840Z  WARN play_launch::runtime_enforcement: [runtime] unknown topic at publisher creation -- no manifest declares '/rosout' (hash 0xd46add98f57fdcea); add to `topics:` or `external_topics:` (graph-deviation-runtime)
2026-09-18T12:55:15.717882Z  WARN play_launch::runtime_enforcement: [runtime] unknown topic at publisher creation -- no manifest declares '/parameter_events' (hash 0x47c9785a1f548d47); add to `topics:` or `external_topics:` (graph-deviation-runtime)
2026-09-18T12:55:21.720018Z  WARN play_launch::runtime_enforcement: [runtime] topic '/pure_test/chatter' publisher '/pure_test/talker/chatter' measured rate 1.20 Hz < declared min_rate_hz (1000) (rate-hierarchy-runtime)
```

Without the config file: none of those three lines, and exit 0.

## Cause

Three facts that are each fine alone and wrong together:

1. `options.rs:749-750`: `enforce_rules` defaults to `EnforceMode::Warn`, and
   the help text (`:744-748`) promises "Warn: log violations".
2. `config.rs:461-464`: `InterceptionSettings::default()` has
   `enabled: false`. Interception is the ONLY event source the rule engine
   has (the LD_PRELOAD hooks on `rcl_publisher_init` / `rcl_publish` /
   `rcl_subscription_init` / `rcl_take`), and it can only be switched on
   from the `--config` YAML; there is no CLI flag.
3. `up.rs:607` creates `interception_consumers` only when
   `runtime_config.interception.enabled`, and `up.rs:1400` builds the
   `RuleEngine` only `if !interception_consumers.is_empty()` (comment: "Spawn
   interception consumer task if we have consumers"). With interception off
   the branch at `:1404-1415` that constructs the engine for any mode other
   than `Off` is never reached.

No code path checks "enforce mode is not Off but interception is off" and
tells the user. The integration test that exercises runtime rules carries
the trap as a comment ("Interception isn't enabled by default; the runtime
rules feed off interception SPSC events so we need it on") and works around
it; a user has no such comment.

## Impact

Every claim of the form "play_launch enforces the contract at runtime" is
false for the default invocation. `--enforce-rules strict` as a CI gate
passes trivially on a run where nothing is observed, which is the worst
shape for a gate: green with no measurement behind it. The safety-island
deck (brief C section 3) had to add a footnote to every runtime slide for
this reason.

## Fix direction

One of, in order of preference:

- Make a non-Off `--enforce-rules` (explicit OR default) imply
  `interception.enabled = true` unless the config file says
  `enabled: false` explicitly; log at `info` which of the two decided.
- Or keep the coupling but refuse to be quiet about it: when
  `enforce_rules != Off` and `interception.enabled == false`, `warn!` once at
  startup ("--enforce-rules warn has no event source: interception is
  disabled; set `interception.enabled: true` in --config") and, in `Strict`,
  make that an error rather than a silently green run.
- In both cases add an `--interception on|off` CLI switch so the mode can be
  turned on without authoring a YAML file, and have `--enforce-rules`'s
  help text name the precondition.

A regression test: `up` with the default options and a contract that any
run violates must either emit the violation or emit the "no event source"
warning; exiting 0 with neither is the bug.

## Provenance

Brief C (`brief-C-playlaunch-usage.md`, sections 2.5 and 3.1), runs of
2026-09-18 with the 0.10.0 binary at `155ed78b`; re-verified against the
0.11.0 source at `5eaa3191` on 2026-09-21 (line numbers above are 0.11.0).
