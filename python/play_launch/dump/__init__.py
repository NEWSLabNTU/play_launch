"""dump_launch - Records ROS2 launch execution to JSON."""

import argparse
import json
import os
from collections import OrderedDict
from typing import List, Text, Tuple

from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ros2launch.api import MultipleLaunchFilesError, get_share_file_path_from_package

from .inspector import LaunchInspector


def main() -> int:
    """Entry point for dump_launch command."""
    parser = argparse.ArgumentParser()
    parser.add_argument("package_name")
    parser.add_argument("launch_file_name", nargs="?")
    parser.add_argument("launch_arguments", nargs="*")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("-o", "--output", default="record.json")
    args = parser.parse_args()

    launch_arguments = []
    if os.path.isfile(args.package_name):
        launch_path = args.package_name
        if args.launch_file_name is not None:
            # Since in single file mode, the "launch file" argument is
            # actually part of the launch arguments, if set.
            launch_arguments.append(args.launch_file_name)
    else:
        try:
            launch_path = get_share_file_path_from_package(
                package_name=args.package_name, file_name=args.launch_file_name
            )
        except PackageNotFoundError as exc:
            raise RuntimeError(f"Package '{args.package_name}' not found: {exc}") from exc
        except (FileNotFoundError, MultipleLaunchFilesError) as exc:
            raise RuntimeError(str(exc)) from exc

    launch_arguments.extend(args.launch_arguments)

    output_file = args.output

    # argv should be empty - launch_arguments are passed separately to IncludeLaunchDescription
    inspector = LaunchInspector(argv=[], noninteractive=True, debug=args.debug)

    parsed_args = parse_launch_arguments(launch_arguments)
    launch_description = LaunchDescription(
        [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(launch_path),
                launch_arguments=parsed_args,
            )
        ]
    )
    inspector.include_launch_description(launch_description)

    inspector.run(shutdown_when_idle=True)
    dump = inspector.dump()

    with open(output_file, "w") as fp:
        json.dump(dump, fp, sort_keys=True, indent=4)

    return 0


def parse_launch_arguments(launch_arguments: list[str]) -> list[tuple[str, str]]:
    """Parse the given launch arguments from the command line, into list of tuples for launch."""
    parsed_launch_arguments = OrderedDict()  # type: ignore
    for argument in launch_arguments:
        count = argument.count(":=")
        if count == 0 or argument.startswith(":=") or (count == 1 and argument.endswith(":=")):
            raise RuntimeError(
                f"malformed launch argument '{argument}', expected format '<name>:=<value>'"
            )
        name, value = argument.split(":=", maxsplit=1)
        parsed_launch_arguments[name] = value  # last one wins is intentional
    return parsed_launch_arguments.items()


__all__ = ["LaunchInspector", "main", "parse_launch_arguments"]
