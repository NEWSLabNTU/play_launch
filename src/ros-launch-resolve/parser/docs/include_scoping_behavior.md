# Include and Group Scoping

What `<include>` and `<group>` do to launch configurations, as `launch`
implements it and as this parser now follows it.

## The rule

**An include does not scope launch configurations. A scoped group does.**

`IncludeLaunchDescription.execute` (launch 1.0.14, Humble; unchanged in
Jazzy) ends with:

```python
set_launch_configuration_actions = [SetLaunchConfiguration(n, v) for n, v in self.launch_arguments]
return [*set_launch_configuration_actions, launch_description]
```

The arguments are set in the includer's own context and the included
description's entities run there. Nothing is pushed or popped. `GroupAction`
with `scoped=True` (the default) is what wraps its body in
`PushLaunchConfigurations` / `PushEnvironment` ... `Pop*`.

This holds for every frontend. `<include>` in XML, `include:` in YAML and
`IncludeLaunchDescription` in Python are the same action.

## Consequences, measured

All of these come from stock `ros2 launch` on the fixtures in
`crates/play_launch_parser/tests/fixtures/include_semantics/`, checked by
`tests/include_semantics.rs`:

- An include's `<arg>`s reach the included file and beat its `<arg default>`s.
- **An argument persists into later sibling includes.** If the first include
  passes `flag=false` and the second passes nothing, the second sees `false`.
  Its own `<arg name="flag" default="true"/>` does not apply, because a
  declaration's default is used only when the configuration is unset.
- What the included file declares or `<let>`s is visible to the includer
  afterwards.
- A scoped `<group>` around the include undoes all three. This is why
  Autoware wraps its component includes in `<group>`.
- A `<let>` inside a scoped group does not survive the group. With
  `scoped="false"` (XML) or `scoped: false` (YAML) it does.

## What the parser did before, and why it was wrong

Until this change the parser split the behaviour by frontend:

- **XML target:** ran in an isolated child context (`LaunchContext::child()`),
  with nothing coming back. That rested on a sentence in the ROS 2 XML design
  article ("The included launch file description has its own scope for launch
  configurations"), which the implementation does not do.
- **YAML target:** ran in the includer's context, which is right, but the
  include's arguments were never applied. Every YAML file included with
  arguments ran on its own defaults. The YAML path existed to make Autoware's
  preset files (`config/*/preset/*_preset.yaml`, included for their
  side effects) work, and those take no arguments, so nothing noticed.
- **Groups:** restored only the namespace and remaps on exit, so a `<let>`
  inside a group leaked out. Isolating XML includes had hidden most of this.

The parallel include processing that the old version of this document
justified the isolation with no longer exists.

## Implementation

- `traverser/include.rs`, `process_include`: one path for XML and YAML
  targets. It checks required arguments, sets the include's arguments in
  `self.context` in order, then traverses the file in the same traverser and
  context. It also pushes the file onto `include_chain` (the YAML path never
  did, so a YAML cycle overflowed the stack) and stamps the include's scope
  on what the file produced.
- `traverser/xml_include.rs` (a `.launch.py` including XML): keeps its child
  traverser for namespace re-prefixing, and adopts the child's configurations
  afterwards (`LaunchContext::adopt_configurations_from`).
- `traverser/entity.rs` and `traverser/yaml.rs`, groups: a scoped group calls
  `push_launch_configurations` / `pop_launch_configurations` alongside
  `save_scope` / `restore_scope`.

## Known remaining divergence

Python launch files. A configuration that a `.launch.py` sets
(`SetLaunchConfiguration`, its `DeclareLaunchArgument` defaults) is written to
the pyexec half's own context and does not cross back to the host. So:

- an XML/YAML file that includes a `.launch.py` does not see what the Python
  file set;
- an XML/YAML file included from a `.launch.py` does not see the Python
  file's own `SetLaunchConfiguration`s.

Arguments do cross in both directions, and so does sibling persistence.
Closing the gap means carrying configurations back over the pyexec ABI.

The `forwarding="false"` attribute of `<group>` is accepted and not
implemented.
