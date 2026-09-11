---
id: 30
title: "A `.launch.py` include never checks required arguments: the replay passes the parent's scope as if given, and `DeclareLaunchArgument` never demands a value"
status: resolved
type: parity
severity: low
---

# 0030 — required arguments through the Python frontend

**Repo:** `play_launch` (`src/ros-launch-resolve/parser`: `pyexec` mocks, `traverser/python_exec.rs`, `traverser/include.rs`)
**Affects:** the Rust parser (the default) whenever a `.launch.py` is on either side of an include
**Follows:** #0029, which fixed the same rule for XML and YAML included files
**Resolved:** 2026-09-11, ABI 4 → 5

## What happens

`launch` applies two rules the Rust parser's Python path applies neither of:

1. `IncludeLaunchDescription.execute`: every `DeclareLaunchArgument` of the included
   description with no default, reachable without passing through a condition or an
   `OpaqueFunction`, must be in the include's OWN `launch_arguments`. The parent's
   configurations do not count.
2. `DeclareLaunchArgument.execute`: a declaration with no default whose name is unset
   raises `Required launch argument 'x' (description: 'y') was not provided`.

Under play_launch, a `.launch.py` runs against Rust stand-ins for the launch API. Two
gaps:

- **The replay hands the parent's whole scope over as the include's arguments.** In
  `python_exec.rs`, a captured `IncludeLaunchDescription`'s `launch_arguments` are
  merged into a copy of the caller's argument map, and that map is what the included
  XML or YAML file receives as "given". Every name in scope satisfies #0029's check,
  which is exactly what launch says must not satisfy it. A `.launch.py` including
  another `.launch.py` goes down the Python branch, where there is no check at all.
- **The `DeclareLaunchArgument` stand-in never raises.** With no default and nothing
  set it registers nothing and returns; launch raises. So a Python-included file's own
  required arguments are never demanded, and a root `.launch.py` with an unset required
  argument resolves to whatever `$(var)` later makes of the missing name.

Consequence: a `.launch.py` include that `ros2 launch` refuses can resolve under
play_launch. Lenient direction, same as #0029.

## Fix

Declarations cross the boundary. The `DeclareLaunchArgument` stand-in records every
declaration into the object's context as a `DeclaredArgumentCapture` — name,
description, whether it had a default, and whether it was constructed inside an
`OpaqueFunction` (the executor sets a flag around `execute`; launch's include-time
check cannot see inside an opaque function, and neither must this one).
`ExecCaptures` carries them back (**ABI 4 → 5**, for the usual reason: a v4 object
would report none and the check would be silently satisfied).

Then, in the traverser:

- After any `.launch.py` executes, a declaration with no default whose name is still
  unset is refused with launch's `DeclareLaunchArgument` message. This covers the root
  file and every include.
- A `.launch.py` included from XML (`include.rs`) or from Python (`python_exec.rs`) is
  additionally held to #0029's rule against the include's OWN arguments: non-opaque
  declarations without defaults must be passed, whatever the parent scope holds.
- A captured `IncludeLaunchDescription` of an XML or YAML file passes only its own
  `launch_arguments` on to `process_xml_include_with_namespace` / the YAML path; the
  child context already inherits the parent's configurations for substitution, so
  nothing resolves differently, and #0029's check now sees what was actually given.

One ordering difference from launch, stated rather than hidden: when an included
`.launch.py` declares a required argument that is neither passed nor in scope, launch
reports the include-level message (it checks before executing the child); here the
child executes first and the `DeclareLaunchArgument` message wins. Both name the
argument.

## Tests

- `c_abi.rs`: declarations come back over the wire with their flags; an unset required
  argument in a root file is an error naming it; the ABI is 5.
- `xml_tests.rs`, in-process backend: XML including a `.launch.py` that declares
  `required` — refused when the parent merely declares the name, accepted when passed,
  and an `OpaqueFunction`-declared argument supplied by scope alone is not demanded;
  `.launch.py` including `.launch.py` without `launch_arguments` while the value is in
  scope — refused.

## Verification

Parser, object and loader suites 519 of 519, including the six tests above. The
golf-cart entry points (indoor, NTU, aruco, logging and planning sims) resolve with
the same node counts as the 0029 build.
