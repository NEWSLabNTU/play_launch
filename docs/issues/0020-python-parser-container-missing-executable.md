---
id: 20
title: "The Python parser emits containers without `executable`, so its record fails to load"
status: open
type: correctness
severity: high
---

# 0020 — `--parser python` cannot round-trip a real Autoware stack

**Repo:** `play_launch`
**Affects:** `python_dump/visitor/*` (the dump side),
`ros/launch_dump.rs` `NodeContainerRecord` (the load side)

## Symptom

    $ play_launch resolve --parser python golfcart_launch golfcart.launch.yaml …
    Error: loading Python-parsed record /tmp/play_launch-parse-…record.json
    Caused by:
        0: missing field `executable` at line 6 column 9

The record the Python parser just wrote cannot be read back by the Rust side.

## Cause

Container entries in the emitted record carry only a name and a namespace:

```json
{
    "container": [
        {
            "name": "pointcloud_container",
            "namespace": "/"
        },
```

`NodeContainerRecord` requires `executable`. So the dump and the loader
disagree about the shape of a container, and every launch containing one fails
— which on an Autoware stack is every launch.

## Why it matters more than a broken flag

`--parser python` is not a side path:

- It is the **compatibility fallback** the Rust parser's own error messages
  recommend: *"re-run the same command with `--parser python` (slower, maximum
  compatibility)"*. That advice currently leads nowhere on any stack with a
  composable container.
- It is the **reference for parser parity**. `just compare-dumps` and
  `scripts/compare_models.py` compare the two parsers; parity cannot be checked
  on a real corpus while one side cannot complete. Issue #0018's cross-parser
  key divergence had to be demonstrated on a synthetic five-node fixture for
  exactly this reason.
- CLAUDE.md's standing rule is *"when Rust and Python parser behaviours differ,
  Python's behaviour is always correct"*. That rule is unenforceable where
  Python cannot run.

## Not the same as the failure it was mistaken for

Earlier in the same investigation `--parser python` failed on this stack with
`KeyError: ''`, and that was reported here as a play_launch bug more than once.
It was not: the cause was `gnss_receiver` defaulting to `""` in the launch file
and reaching a dict lookup in the sensor kit, which broke **stock `ros2 launch`
too** (27 nodes up, then `Caught exception in launch: ''`). With that fixed in
the launch file, the Python parser gets further and lands on THIS defect, which
is genuinely ours.

Worth keeping the distinction: one error message hid two unrelated faults, and
the first was assumed to be the second.

## Reproducing

Any launch with a `<node_container>`; the golf cart stack is one.

    play_launch resolve --parser python golfcart_launch golfcart.launch.yaml \
        host:=master rviz:=false imu_source:=xsens camera_model:=gscam -o /tmp/m.yaml

The temporary record is left on disk when loading fails, so the offending JSON
can be inspected directly.

## Open question

Which side is wrong. Either the visitor should emit the container's executable
(it knows it — `component_container`, `component_container_mt`, or an override),
or `executable` should be optional on `NodeContainerRecord` and defaulted at
use. The first looks right: a container with no executable is not something the
spawn path can act on, so accepting it would move the failure later.
