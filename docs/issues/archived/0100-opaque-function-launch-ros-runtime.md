---
id: 100
title: "A launch file's own OpaqueFunction cannot use `launch_ros.ros_adapters` or `ParameterFile.evaluate` under the Rust parser"
status: resolved
type: defect
severity: medium
---

# 0100 - `launch_ros.ros_adapters` and `ParameterFile.evaluate` are missing from pyexec

**Repo:** `play_launch` 0.40.0 (PyPI)
**Affects:** pyexec's stand-ins for `launch_ros` and `launch`'s context
**Found:** ceilbot (cenming-tech/amr-example-project) on a Jetson Orin Nano,
JetPack 7.2.1, Jazzy, 2026-11-19, once #0099 let the walk get past the
first `$(robot-value …)`

## What happens

ceilbot's `ceilbot_navigation/launch/nav2_composed.launch.py` runs Nav2 in a
container of its own and, at each start of the container, waits for the
container to answer `list_nodes` before it loads Nav2 into it. The wait is a
plain `OpaqueFunction`:

```python
from launch_ros.ros_adapters import get_ros_node
...
node = get_ros_node(context)
client = node.create_client(ListNodes, f'{name}/_container/list_nodes')
while time.monotonic() < end and not context.is_shutdown:
    if not client.wait_for_service(timeout_sec=1.0): ...
```

and the file writes Nav2's parameters out with
`ParameterFile(rewritten, allow_substs=True).evaluate(context)`. The Rust
parser failed on each in turn:

```
Python error: Python error: ModuleNotFoundError: No module named 'launch_ros.ros_adapters'; 'launch_ros' is not a package
Python error: Python error: AttributeError: 'launch_ros.parameter_descriptions.ParameterFile' object has no attribute 'evaluate'
```

and `context.is_shutdown` was the next: `LaunchContextView` had no such
attribute. `--parser python` resolves the file.

## Resolution (2026-11-19)

- `launch_ros.ros_adapters` is a stand-in: `get_ros_node(context)` returns a
  node with no graph, whose clients find their service at once and whose
  requests are answered at once (`create_client`, `wait_for_service`,
  `service_is_ready`, `call_async` with a done future, `call`,
  `destroy_client`, `get_logger`, `get_name`). A walk models the launch
  coming up, so a wait returns where the running launch would have gone on;
  nothing is sent anywhere.
- `context.is_shutdown` is `False`: a walk has not been asked to shut down.
- `ParameterFile.evaluate(context)` does what `launch_ros`'s does: the
  path, performed; with `allow_substs`, a `launch_params_*` temporary copy
  with the `$(...)` in its text performed in the walk's context, refused
  unless it is still YAML (stock's message); the result kept, so a second
  call is the same file.

## Tests

`tests/opaque_ros_tests.rs` (parser crate) with
`tests/fixtures/opaque_ros/opaque_ros.launch.py`, written after ceilbot's
file: the wait returns (a node the function returns only after it), and the
copy has the file's `$(var robot)` performed, for the default and for
`robot:=r2`.

ceilbot's `robot.launch.py` (`robot:=ceilbot site:=office sim:=true
gui:=true rviz:=true`) then resolves with the Rust parser: 28 nodes, the
same as `--parser python`, with equal parameters but for the cosmetic
differences listed under Parser Parity Status (boolean case, list and empty
string quoting), and RViz's `launch-prefix="vglrun"` kept apart from its
command rather than folded into `exec_name`.
