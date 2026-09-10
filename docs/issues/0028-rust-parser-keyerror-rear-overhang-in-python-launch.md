---
id: 28
title: "Rust parser dies with `KeyError: 'rear_overhang'` on the golf-cart stack; the Python parser resolves it"
status: resolved
type: correctness
severity: medium
---

# 0028 — `KeyError: 'rear_overhang'` from a Python launch file under the Rust parser

**Repo:** `play_launch` (`src/ros-launch-resolve/parser`, the embedded-Python execution
of `.launch.py` includes)
**Affects:** the Rust parser (the default) on any launch tree where a `.launch.py`
reads `context.launch_configurations['global_params']` — every Autoware sensor pipeline
**Resolved:** 2026-09-11, ABI 3 → 4 (`global_parameters` in the `exec_file` request)

## What happens

play_launch 0.10.0, ROS 2 Humble, Autoware 1.5.0 apt, 2026-09-11:

```
$ play_launch resolve --parser rust golfcart_launch indoor_logging_sim.launch.xml -o out.yaml
Error: Rust parser error while parsing golfcart_launch: Python error: Python error: KeyError: 'rear_overhang'
Hint: if this is a parser limitation rather than a bad launch file, re-run the same command with `--parser python`
```

`--parser python` resolves the same launch, and stock `ros2 launch` runs it. The
error names neither the launch file nor the line; the candidates are the Autoware
`.launch.py` files that read the vehicle-info dictionary, e.g.
`single_lidar_common_launch/launch/nebula_node_container.launch.py`,
`tier4_perception_launch/.../ground_segmentation.launch.py` (Autoware 1.5.0,
`/opt/autoware/1.5.0/share`).

## Root cause

Not the vehicle-info file and not the `.launch.py` reading it. The KeyError names
`rear_overhang` only because the golf-cart `pointcloud_preprocessor.launch.py` indexes it
first; the dictionary was **empty**. Reduced to ten lines: an XML `<set_parameter
name="rear_overhang" value="0.821"/>` followed by a `.launch.py` whose `OpaqueFunction`
reads `global_params` fails the same way, and so does a `SetParameter` returned by an
earlier `.launch.py` — directly or through a nested `IncludeLaunchDescription`. Stock
`ros2 launch` and `--parser python` see both keys in every variant.

Since issue 0897 the Python half is a `dlopen`ed object that statically links its own
copy of `play_launch_parser`, so it has its own thread-local `LaunchContext`. Issue 0935
made the loader send `configs` in the `exec_file` request and take `captures` back;
ABI 3 added `namespace_stack` in and `includes` out. **Nobody sent the global
parameters in.** `c_abi.rs` seeds a fresh context from `configs` and `namespace_stack`
only, so `SetParameter` values set by the host (XML) or by an earlier call (the
vehicle-info loader) were never in the context the next file's `OpaqueFunction` read,
and `create_launch_context` / `OpaqueFunction::execute` faithfully reported an empty
`global_params`. The values did come back OUT (`ExecCaptures.global_parameters`), which
is why the model's nodes carried them and nothing upstream noticed.

The in-process backend (`play_launch_parser_pyexec::register()`, which every parser
test uses) shares the host's thread-local, so no fixture test could reproduce this; it
lived only in the shipped, dlopen'ed pairing.

## Fix

- `pyexec/src/c_abi.rs`: the request gains `global_parameters: Vec<(String, String)>`
  (serde-defaulted) and `exec_file` seeds the object's context with them before the
  file runs.
- `pyload/src/lib.rs`: `exec_file` reads `ctx.global_parameters()` from the host context
  and sends them; `eval_expr` sends none.
- **ABI 3 → 4** on both sides, for the reason 2 and 3 were bumped: a v3 object accepts
  a v4 request and answers it wrong, silently, and the version is what makes that
  pairing a sentence.

Tests: `exec_file_sees_the_global_parameters_it_was_sent` in `c_abi.rs` (object side,
over the wire) and
`a_global_parameter_set_by_the_host_reaches_the_next_file_across_the_boundary` in
`pyload/tests/loads_a_real_interpreter.rs` (the real object, the real request, from a
host context). The second is the one that can catch a regression; see above for why the
parser-level fixtures cannot.

## Also fixed on the way

**`Loaded` unloaded the object on drop, and that segfaults the next thread to exit.**
Found writing the loader test: any `exec_file` from a test thread died with SIGSEGV in
`__nptl_deallocate_tsd`, no Rust frame in the backtrace. The object registers
thread-local destructors of its own (its copy of `CURRENT_LAUNCH_CONTEXT`, PyO3 state);
a thread-local destructor is an address inside the object; `Loaded` dropped both
`libloading::Library` handles, `dlclose` unmapped it, and the thread's exit ran the
destructor into unmapped memory. The driver never saw it because it returns from `main`
without running any thread's destructors. Both handles are now `ManuallyDrop` and stay
resident for the life of the process, which is what a plugin with thread-locals (and a
CPython interpreter) has to be.

**Parity on the golf-cart stack after the fix:** `indoor_logging_sim.launch.xml` resolves
under the Rust parser to 84 nodes, the same 84 keys the Python parser produces.

The Python-error path names no file. `RUST_LOG=debug` shows the last "Executing Python
file:" line before the error, which is how this was located; making the error carry it
is still worth doing (first step below, still open).

## Where to look (original notes)

Those files build the dictionary with `autoware_vehicle_info_utils`'s
`get_vehicle_info_param()` / `add_vehicle_info` and then index it by key. Under
`ros2 launch` the keys come from `vehicle_info.param.yaml` of the vehicle description
found through the launch context. The KeyError says the embedded execution handed the
file a dictionary without that key: either the vehicle-info YAML was not located (the
`vehicle_model` argument or `find-pkg-share` not reaching the Python side), or the
dictionary was built from a different source. Not diagnosed; recorded so the golf-cart
side stops treating it as #0027.

## Consequence (before the fix)

The golf-cart replays (`just indoor-test up`, `just ntu-test up`) and `just launch` stayed
on `--parser python`; #0027 was fixed and this was what kept them there.

## First steps

1. Make the Python-error path name the launch file (and, if the traceback allows, the
   line) — the message is the expensive part, as in #0024.
2. Reproduce with one Autoware `.launch.py` alone under `ros-launch-resolve resolve`,
   with `vehicle_model` set, and compare the dictionary the two execution paths build.
