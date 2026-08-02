#!/usr/bin/env python3
"""
Run dump_launch on a given launch file and print detailed variable scoping info.
"""

import sys
import json
import logging
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

from inspector import LaunchInspector
from launch.launch_description_sources import AnyLaunchDescriptionSource


def main():
    if len(sys.argv) < 2:
        print("Usage: run_dump.py <launch_file>")
        sys.exit(1)

    launch_file = sys.argv[1]

    # Setup logging to see variable scoping
    logging.basicConfig(
        level=logging.DEBUG,
        format='[PYTHON] %(levelname)s - %(name)s: %(message)s'
    )

    # Create inspector
    inspector = LaunchInspector(debug=True)

    # Load the launch description
    source = AnyLaunchDescriptionSource(launch_file)
    launch_description = source.get_launch_description(inspector.context)

    print(f"\n[PYTHON] === Initial Context Variables ===")
    print(f"[PYTHON] Launch configurations: {inspector.context.launch_configurations}")
    print(f"[PYTHON] Locals: {list(inspector.context._local_vars)}")

    # Include the launch description
    inspector.include_launch_description(launch_description)

    # Run the inspector
    print(f"\n[PYTHON] === Running Inspector ===")
    inspector.run()

    print(f"\n[PYTHON] === Final Context Variables ===")
    print(f"[PYTHON] Launch configurations: {inspector.context.launch_configurations}")

    # Dump results
    dump = inspector.dump()
    print(f"\n[PYTHON] === Results ===")
    print(f"[PYTHON] Nodes: {len(dump['node'])}")
    print(f"[PYTHON] Containers: {len(dump['container'])}")
    print(f"[PYTHON] Load nodes: {len(dump['load_node'])}")

    # Save to file for comparison
    output_file = Path("tmp/python_dump.json")
    output_file.parent.mkdir(exist_ok=True)
    with open(output_file, 'w') as f:
        json.dump(dump, f, indent=2, default=str)
    print(f"\n[PYTHON] Output saved to: {output_file}")


if __name__ == "__main__":
    main()
