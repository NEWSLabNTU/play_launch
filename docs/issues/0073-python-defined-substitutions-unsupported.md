---
id: 73
title: "The Rust parser cannot load launch files that define their own `launch.Substitution`"
status: open
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
