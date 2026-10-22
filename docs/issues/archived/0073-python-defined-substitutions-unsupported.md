---
id: 73
title: "The Rust parser cannot load launch files that define their own `launch.Substitution`"
status: resolved
type: limitation
severity: medium
---

# 0073 - Python-defined substitutions are not supported by the Rust parser

**Repo:** `play_launch` at `ca5abc2d`
**Affects:** `pyexec` (the `launch` mock module)

## Symptom

    AttributeError: module 'launch' has no attribute 'Substitution'

for any launch file that imports `nav2_common.launch` (`RewrittenYaml`,
`ReplaceString`, `HasNodeParams` subclass `launch.Substitution`), which is
every `nav2_bringup` launch file, on Humble and Jazzy.

## Cause

The mock `launch` module has no `Substitution` base class, and
`pyobject_to_string` has no way to evaluate a Python object that implements
`perform(context)`: a user substitution needs a context with
`launch_configurations`, `perform_substitution` and
`launch.utilities.perform_substitutions`. `RewrittenYaml.perform` also writes
a temporary YAML file and returns its path.

## Workaround

`--parser python` (runs the real launch code).

## Direction

A `launch.Substitution` base class in the mock, and evaluation of subclasses
by calling their `perform` with a context object backed by the traverser's
launch configurations.

## Resolution

The mock `launch` module now carries what `nav2_common.launch` uses:

- `launch.Substitution`, `launch.Condition` and `launch.Action` as pure
  Python base classes, so `super().__init__()` works in a subclass, plus
  `launch.LaunchContext`. Their annotations run when the class body does,
  so a missing name failed the import before any `perform` was reached.
- `launch.utilities.normalize_to_list_of_substitutions` and
  `launch.utilities.perform_substitutions`. The second performs each
  element through `pyobject_to_string`, so a `LaunchConfiguration` reads
  the configuration the walk holds, as everywhere else.

`pyobject_to_string` already called `perform(context)` on anything with a
`perform` method. What it lacked was the error path: a failing `perform`
fell through to the object's `repr`, which for a launch file's own class
would have put `<RewrittenYaml object at 0x...>` where a parameter file
path belongs. A `launch.Substitution` subclass now propagates its error.

`RewrittenYaml.perform` writes its temporary YAML file and returns the
path, exactly as under `launch`. The file outlives the parse, as it does
under `ros2 launch`.

## Tests

- `execution_semantics::a_substitution_the_launch_file_defines_is_performed`
  over `tests/fixtures/execution_semantics/user_substitution/`: inline
  nav2_common-shaped `RewrittenYaml` (a launch configuration rewritten into
  a parameter file) and `HasNodeParams` (deciding an `IfCondition` and an
  `UnlessCondition`). Negative control: with the `pyexec` changes stashed
  it fails with the reported `AttributeError` verbatim.
- `play_launch resolve --parser rust` and `--parser python` on the fixture:
  `scripts/compare_models.py` PASS, `use_sim_time: true` in both.

Not run: a real `nav2_bringup` tree, which is not installed on the
machine the fix was made on.
