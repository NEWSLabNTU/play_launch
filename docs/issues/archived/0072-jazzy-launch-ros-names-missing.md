---
id: 72
title: "Jazzy launch files import names the `launch_ros` mocks lack"
status: resolved
type: defect
severity: high
---

# 0072 - Jazzy `launch_ros` names missing from the mocks

**Repo:** `play_launch` at `ca5abc2d`
**Affects:** `pyexec/src/api/mod.rs`

## Symptom

`nav2_bringup`'s launch files on Jazzy stopped the Rust parser:

    ImportError: cannot import name 'PushROSNamespace' from 'launch_ros.actions'
    ImportError: cannot import name 'ParameterFile' from 'launch_ros.descriptions'

## Cause

Jazzy renamed `PushRosNamespace` / `PopRosNamespace` to `PushROSNamespace` /
`PopROSNamespace` (the old names stay as deprecated aliases), and
`launch_ros.descriptions` re-exports `ParameterFile` and `ParameterValue`.
The mocks had only the Humble names, and those two classes only under
`launch_ros.parameter_descriptions`.

## Fix

The Jazzy names are aliases of the same mock classes, and
`launch_ros.descriptions` carries `ParameterFile` and `ParameterValue`.
