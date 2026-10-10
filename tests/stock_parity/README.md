# Stock parity

Checks that `play_launch resolve` produces what stock `ros2 launch` would
start. The node set is compared, and for every node so is its package,
executable or plugin, container, parameters (values and types), parameter
files, remaps, arguments, ROS arguments, environment and conditions.

`just test-parity` compares our Rust parser with our own Python parser. This
compares against ROS itself, so it catches what both of ours get wrong
together.

```bash
source /opt/ros/humble/setup.bash
just test-stock-parity                 # every case under cases/
just test-stock-parity subs ns         # named cases
just test-stock-parity --file /opt/autoware/1.5.0/share/autoware_launch/launch/planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

It uses this repository's `install/` ahead of any installed `play_launch`, so
run `just build` first. Output goes to `tmp/stock_parity/`.

## How stock is resolved, without starting anything

`stock_oracle.py` runs a real `launch.LaunchService` over the file, with
exactly two patches:

- `ExecuteLocal.execute` runs `prepare()`, so every substitution in the
  command, cwd and environment is performed exactly as stock does it. It then
  records the result instead of starting the process.
- `LoadComposableNodes.execute` builds the LoadNode requests with stock's own
  `get_composable_node_load_request` and records them instead of calling the
  container.

Everything else is unpatched stock code, including conditions, scoping,
includes, `OpaqueFunction` and event handlers. No process is spawned, so
there is nothing to orphan.

`compare.py` normalises both sides to one entity per fully-qualified name and
prints `DIFF`, `EXTRA` and `MISSING` lines. It exits 1 if there are any.

`stock_subst.py` evaluates a substitution string with stock `launch`. It's
useful when writing a case:

```bash
python3 tests/stock_parity/stock_subst.py -c num=3 -- '$(eval "$(var num) * 2")'
```

## Cases

Each case is a directory under `cases/` holding `parent.launch.{xml,yaml,py}`
plus whatever it includes. It can also contain:

| File | Meaning |
|---|---|
| `ARGS` | Launch arguments, one line: `name:=value ...` |
| `EXPECTED_DIFFS` | Known differences, one per line, prefix-matched against the `compare.py` output. Lines starting with `#` say why. |
| `EXPECT_ERROR` | Both sides must refuse the file |

These differences are known, and are listed in their cases:

- `launch-prefix` and `cwd` are not carried in the SystemModel (`exec_comp`).
- `<unset_env>` cannot be represented (`setscope`).
- `type="str"` on a list-shaped value becomes a list, because the model has
  no string-vs-array distinction (`params`, `params2`).

`known_divergent/` holds files that Humble rejects and play_launch
deliberately accepts, for compatibility with existing launch files:
`$(optenv)` and `<group ns>`. They are not run.

A new case should be the smallest file that shows one behaviour, with its
expected result taken from stock. Add a regression test in the parser crate
too (`tests/include_semantics.rs`, `tests/execution_semantics.rs`,
`tests/resolution_semantics.rs`). This harness finds a divergence; the parser
test pins its fix.
