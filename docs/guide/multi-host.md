# Multi-host launches

ROS 2 has no multi-machine launch. The ROS 1 `<machine>` tag and the
`<node machine="…">` attribute do not exist in ROS 2 — `launch_ros`'s
`Node.parse()` has no such attribute, and the XML frontend rejects it:

```
ValueError: Unexpected attribute(s) found in `node`: {'machine'}
```

A multi-machine design was proposed (`ros2/design` PR #255) and closed
without merging. Under ROS 2's peer-to-peer DDS discovery there is no
central launcher to distribute processes from, so the launch file describes
one host's processes and each host runs its own.

## The pattern

Select the host with an ordinary launch argument and gate nodes on it:

```xml
<launch>
  <arg name="host" default="all"/>

  <node pkg="talker_pkg" exec="talker" name="talker"
        if="$(eval '&quot;$(var host)&quot; in (&quot;robot1&quot;, &quot;all&quot;)')"/>

  <node pkg="listener_pkg" exec="listener" name="listener"
        if="$(eval '&quot;$(var host)&quot; in (&quot;robot2&quot;, &quot;all&quot;)')"/>

  <!-- No if= — runs on every host. -->
  <node pkg="hub_pkg" exec="hub" name="hub"/>
</launch>
```

Then on each machine:

```sh
# On robot1
play_launch launch my_bringup multihost.launch.xml host:=robot1

# On robot2
play_launch launch my_bringup multihost.launch.xml host:=robot2
```

Or resolve a model per host ahead of time:

```sh
play_launch resolve multihost.launch.xml host:=robot1 -o robot1.yaml
play_launch resolve multihost.launch.xml host:=robot2 -o robot2.yaml
```

Each model holds exactly that host's nodes plus the unconditioned ones. The
partition happens at resolve time, so anything consuming the model takes it
as-is.

`host` is not special to play_launch. It is a launch argument like any
other, and the name is yours to choose — `robot`, `role`, and `machine_id`
work identically.

## Notes

- The nodes must still reach each other. That is a DDS concern, not a
  launch one: put them on the same `ROS_DOMAIN_ID` and make sure discovery
  traffic can cross the network.
- `$(eval …)` is a Python expression. The `&quot;` entities are XML
  escaping for the double quotes inside it.
- For two or three hosts the `in (…)` form above is clearest. For many
  hosts, prefer one `<group>` per host with a single condition on the group.
- This works identically under `--parser rust` and `--parser python`, and
  the launch file is loadable by stock `ros2 launch`.

## Migrating from `machine=`

`<node machine="robot1">` used to be accepted by play_launch's Rust parser
and mapped to `execution.deploy[fqn].host` in the SystemModel. It was
removed on 2026-07-31: it was never ROS 2, the Python parser always
rejected it, and the resulting launch files could not be run by `ros2
launch`. Rewrite

```xml
<node pkg="talker_pkg" exec="talker" name="talker" machine="robot1"/>
```

as

```xml
<node pkg="talker_pkg" exec="talker" name="talker"
      if="$(eval '&quot;$(var host)&quot; == &quot;robot1&quot;')"/>
```

and add `<arg name="host"/>` at the top of the file.
