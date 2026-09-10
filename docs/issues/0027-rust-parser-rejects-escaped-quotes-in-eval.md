---
id: 27
title: "Rust parser rejects `$(eval '\\'$(var x)\\' == \\'y\\'')`, the escaped-quote template every Autoware `pose_source` dispatch uses"
status: resolved
type: correctness
severity: high
---

# 0027 — escaped quotes inside `$(eval '...')`

**Repo:** `play_launch` (`src/ros-launch-resolve/parser`, `substitution/eval.rs`)
**Affects:** the Rust parser (the default), on any XML launch file with this form
**Resolved by:** `e1fdc731` (2026-08-17), before this was written up; see "Why it was
reported anyway"

## What happens

```xml
<arg name="pose_source" default="ndt"/>
<group if="$(eval '\'$(var pose_source)\' == \'aruco\'')">
  <node pkg="demo_nodes_cpp" exec="talker" name="only_for_aruco"/>
</group>
<group unless="$(eval '\'$(var pose_source)\' == \'aruco\'')">
  <node pkg="demo_nodes_cpp" exec="listener" name="not_aruco"/>
</group>
```

Stock `ros2 launch` starts `listener` only. The Python parser resolves to `/not_aruco`
only. The Rust parser, on play_launch 0.8.2 and on any `ros-launch-resolve` binary
built before 2026-08-17:

```
Error: Rust parser error while parsing eval_quote.launch.xml: Invalid substitution
syntax: Invalid substitution: Failed to evaluate expression '''ndt' == 'aruco''':
SyntaxError: invalid syntax (<string>, line 1)
```

The expression handed to Python is `''ndt' == 'aruco''`: the outer quotes of the
single-quoted template survived and the `\'` escapes were reduced to bare quotes, so the
literal was never unwrapped.

## Why

In the launch frontend grammar (`launch/frontend/grammar.lark`, rule
`single_quoted_template`) the outer quotes of the `$(eval ...)` argument are delimiters,
`\'` inside is an escaped literal quote, and `replace_escaped_characters`
(`re.sub(r'\\(.)', r'\1')`) turns it back into `'` afterwards. The delimiter decision
therefore has to be escape-aware, and unescaping has to happen after it. The old
heuristic did them the other way round: it saw the escaped inner quotes as real ones,
declined to strip the outer pair, and unescaping then produced the SyntaxError above.

## Fix

`e1fdc731` — `outer_quotes_are_delimiters()` walks the inner text tracking escapes and
accepts the outer pair only when the closing quote is unescaped and no unescaped quote
of the same kind sits between; `unescape()` then mirrors the frontend's regex. Measured
against `ros2 launch` on both branches at the time.

What this issue adds: a fixture-level regression test on the XML path,
`tests/fixtures/launch/test_eval_escaped_quotes.launch.xml` with
`test_eval_with_escaped_quotes_selects_the_branch_ros2_launch_selects` in
`tests/xml_tests.rs`, asserting both branches. The unit-level helpers had no test that
went through the XML frontend, which is where the escapes come from.

## Why it was reported anyway

On 2026-09-10 the golf-cart stack (`NEWSLabNTU/2026-golf-cart`,
`golfcart_autoware.launch.xml` and `tier4_localization_component.launch.xml`, both
dispatching on `pose_source == aruco`) failed with exactly this error under
`play_launch launch --parser rust`. The machine ran pip's 0.8.2, released before the fix.
The reproduction above was then run through `src/ros-launch-resolve/target/release/
ros-launch-resolve`, a binary dated 2026-08-13, also before the fix, and failed the same
way — which read as "still broken at HEAD" until the installed 0.10.0 was tried and
passed. Two stale binaries, one wrong conclusion, and briefly one wrong GitHub issue
(#10, closed). Lesson recorded here so the next person checks `play_launch --version`
and the resolver binary's date before reading the source.

The golf-cart launch still cannot use the Rust parser, for a different reason: see
#0028.
