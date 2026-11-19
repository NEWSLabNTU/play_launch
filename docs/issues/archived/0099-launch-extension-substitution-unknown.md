---
id: 99
title: "The Rust parser refuses a substitution a launch extension defines (`$(robot-value …)`), so the launch does not resolve"
status: resolved
type: defect
severity: high
---

# 0099 - A launch extension's substitution is unknown to the Rust parser

**Repo:** `play_launch` 0.40.0 (PyPI)
**Affects:** the Rust parser's substitution grammar
(`substitution/parser.rs::build`), pyexec's stand-in for `launch.frontend`,
and the C ABI between pyload and pyexec
**Found:** ceilbot (cenming-tech/amr-example-project) on a Jetson Orin Nano,
JetPack 7.2.1, Jazzy, 2026-11-19

## What happens

A package can add substitutions to the XML and YAML frontends: a module
named by the `launch.frontend.launch_extension` entry point group registers
a class with `@expose_substitution(name)`, and `launch` loads every such
module before it parses a frontend file (`Parser.load_launch_extensions`).
`ros_gz_sim` does this, and so does ceilbot's `ceilbot_profiles`
(`ceilbot_profiles/launch_ext.py`), whose four substitutions every ceilbot
launch goes through: `$(robot-value ROBOT KEY)`, `$(robot-params ROBOT
CONSUMER)`, `$(robot-file ROBOT)` and `$(robot-merged ROBOT CONSUMER BASE)`.

The default parser refuses the first one it meets:

```
$ play_launch launch ceilbot_operations robot.launch.py robot:=ceilbot site:=office sim:=true gui:=true rviz:=true
Error: Rust parser error while parsing ceilbot_operations: Invalid substitution syntax: Unknown substitution type: robot-value
```

`--parser python` resolves the same launch (29 nodes), because stock
`launch` loads the extension.

## Why

Three things were missing, any one of which was enough:

- the grammar refused a name outside `launch`'s own when it parsed the
  attribute, so nothing after it could even try;
- pyexec's `expose_substitution` returned the class and recorded nothing.
  #0095 made it a decorator so that a module defining an extension can be
  imported; it was never a registry;
- `from launch.substitution import Substitution`, the import every such
  module starts with, was refused by the import guard: the stand-ins had
  `launch.substitutions` but not the module `launch` defines the base class
  in.

## Resolution (2026-11-19)

**Grammar.** A name none of `launch`'s own parses as
`Substitution::Call { name, args }`, as the other named substitutions do.
Only the extension's Python knows whether it exists and what it takes, so
both are settled when it is performed: `resolve_call`'s fallback hands it to
`PythonBackend::perform_substitution` with each argument performed and the
context exported (`ContextState`, the state `exec_file` already crosses
with), so the extension performs in the caller's context.

**pyexec.** `_play_launch_launch_extensions` is the stand-in for
`launch.frontend.expose`'s registries and `Parser.load_launch_extensions`,
written after Jazzy's:

- a class is exposed through its `parse` class method, a function as the
  parse method itself; a substitution class without one is refused, as
  stock refuses it. An action is recorded and never refused, since one may
  inherit `parse` from `launch.Action`, which the stand-in does not define,
  and pyexec runs an extension's action through its own `execute` (#0095);
- the entry points load once, and one that fails to import is skipped, as
  stock skips it with a warning;
- `$(name a b)` is `parse([[a], [b]])` (stock's grammar hands one list of
  substitutions per argument), `type(**kwargs)` of what it returns, and its
  `perform` with the walk's `LaunchContextView` over the caller's state.

It is a module of its own, outside `launch.*`, because `register_modules`
rebuilds the stand-ins for every file while an extension module is imported
once, so its decorators run once. `launch.substitution` is a stand-in now.
An extension is imported against the stand-ins: `executor::ensure_mocks`
installs them when `sys.modules` does not hold them, and leaves them alone
when it does, so a `.launch.py` walk in progress keeps the ones it imported.

**C ABI 15.** `play_launch_py_call` has a `perform_substitution` op: `arg`
is the name, `args` the performed arguments, `state` the context. pyload
sends it. A v14 object would answer `unknown op`, hence the bump.

**Unknown names.** Refused when performed, with stock's message,
`Unknown substitution type: <name>`, followed by the extensions that could
not be imported, which stock only warns about and one of which may be the
one that would have exposed it. A build with no Python half reports
`PythonUnavailable` (`ExtensionSubstitution`), naming the substitution.

## Tests

- `tests/extension_substitution_tests.rs` (parser crate, 4 tests) with the
  extension under `tests/fixtures/extension_substitution/python`
  (`fixture_ext`: a class performing its arguments, a class reading
  `context.launch_configurations`, a parse function exposed by itself;
  `broken_ext`: fails to import): XML, YAML, an argument from the caller's
  context, and an unknown name refused with the failed extension named;
- unit: the grammar keeps the name and nested arguments
  (`test_parse_extension_substitution`), the `PythonUnavailable` message
  (`extension_need_names_the_substitution`), the op through pyexec's real
  export (`a_substitution_no_extension_exposes_is_unknown`), ABI 15, and the
  op through a dlopen'ed interpreter (`loads_a_real_interpreter`).
