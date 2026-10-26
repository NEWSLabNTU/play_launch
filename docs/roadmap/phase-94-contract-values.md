# Phase 94 - contract values read from the launch tree

**Status: planned (2026-10-27).** Phase 85's D9, and DX 1.1's provenance for
numbers a parameter holds. Design of record: rlm
`docs/design/contract-values.md`.

## Why

A contract's configured numbers are typed twice, once in the launch tree and
once in the contract. Three keys can name a parameter to check it, and rlx
extension keys can read one by bare name; nothing else can. On `rt_av_demo`,
`check bringup.launch.xml lidar_hz:=100` reads a contract still saying 50 Hz,
and the safety island types each node's `update_rate` beside the `param:` that
checks it.

## What

`$(param [<node>] <name>) <unit>`, `$(var <name> [@<node>]) <unit>` and
`$(let <name>)` inside any quantity of a contract, evaluated as a
dimension-checked expression; a root `let:`; `params: … default:`. The
check form (`{ duration, param }`) stays for requirements. See the design for
the resolution table and the error messages.

## Wave 1 - references without parser changes

rlm:

- [ ] W1.1 Grammar: a reference-bearing value parses to an unevaluated
      expression (`types`), wherever a quantity is accepted; a literal parses
      exactly as before. Root `let:` (ordered, no forward reference, no
      cycle). `params.<p>.default`. `source:` beside a reference is a parse
      error (`source-on-reference`). Field table rows, generated format
      reference, `GRAMMAR_VERSION`.
- [ ] W1.2 Evaluation: the expression subset (`+ - * /`, parentheses,
      numbers with units, references with a REQUIRED unit) on rlx's evaluator
      and unit table, against a caller-supplied lookup of parameter and
      configuration values. Dimension check against the key
      (`param-dimension`, with the inverse suggestion).
- [ ] W1.3 Extension keys typed `param` accept `$(param <name>) <unit>`;
      `migrate` rewrites the bare form.
- [ ] W1.4 Model: a number read through a reference carries its provenance
      (`<field>_source: { param | var | let }`); readers of the number are
      unchanged. Release.

play_launch (resolver):

- [ ] W1.5 Evaluate every reference per scope instance, from
      `launch_param_values` (`$(param)`) and the scope's include arguments and
      `<arg>` defaults (`$(var)`), BEFORE any rule reads the index. Errors
      `param-unset`, `param-node-unknown`, `param-node-missing`,
      `param-not-number`. No launch tree: the value is unknown and rules
      report incomplete evidence.
- [ ] W1.6 Instance tag on diagnostics of a contract bound to more than one
      scope (`<contract>:<line> [<ns>]`).
- [ ] W1.7 `--explain` prints each read value's provenance (the parameter's
      source and line).
- [ ] W1.8 Fixtures: the island excerpt (read form, check form, extension
      keys, `let`), `rt_av_demo` (a `ms` parameter, a command-line override,
      a shared argument), one negative fixture per error, one contract
      included twice. `rt_av_demo`'s contract migrated to the read form, and
      its derived schedule unchanged.
- [ ] W1.9 Guide: a section in the contract guide; CHANGELOG.

## Wave 2 - `<let>` and the configuration log

- [ ] W2.1 Parser: a per-scope log of configuration events (set, group push,
      group pop) with file, line and sequence number, and each node's
      sequence number, in both frontends; crosses the pyexec C ABI (bump).
- [ ] W2.2 `$(var x)` reads the file's final state and refuses an ambiguous
      name (`var-ambiguous`); `$(var x @<node>)` reads the state where the
      node was declared.
- [ ] W2.3 `--explain` follows a parameter whose text came from a `$(var)` to
      its assignment or the command line.
- [ ] W2.4 Fixtures: a reassigned `<let>`, a `<let>` inside a scoped group,
      a child file's `<let>` reaching its parent; the Python frontend through
      the same tests.

## Out of scope

A rung switched by a boolean parameter, indexing a list parameter,
`$(env)`, run-time parameter changes. See the design.

## Acceptance

- Every existing contract fixture resolves byte-identically (no reference,
  no change), and the ranked-plan snapshots do not move.
- `rt_av_demo` with `lidar_hz:=100` re-derives its rates from the override.
- Each error of the design's resolution table is reachable from a fixture,
  with the message the design shows.
