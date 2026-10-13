# play_launch

Resolve, run and analyze ROS 2 launch executions, with resource monitoring
and a web UI. See the [repository](https://github.com/NEWSLabNTU/play_launch).

play_launch is built against one ROS 2 distribution, whose libraries it
uses at run time. Install it with yours as an extra:

```bash
pip install "play_launch[${ROS_DISTRO}]"     # humble, jazzy, kilted, lyrical, rolling
```

This package has no code: the extra pulls `play_launch_<distro>`, which
provides the `play_launch` command. A plain `pip install play_launch`
installs nothing usable. The `rolling` build matches Rolling as of the
release; Rolling changes, so rebuild from source if it stops loading.
