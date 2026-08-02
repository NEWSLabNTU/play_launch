#!/usr/bin/env python3
"""Compare record.json outputs from Rust parser and Python dump_launch.

This comprehensive comparison tool:
- Runs both Rust and Python parsers on the same launch file
- Compares all record types (nodes, containers, composable nodes)
- Provides detailed diff reports with color-coded output
- Saves JSON outputs for manual inspection

Usage:
    python3 compare_rust_python.py [launch_file] [--profile PROFILE]

    If no launch file is specified, the script will automatically test against
    Autoware's planning_simulator.launch.xml (requires autoware symlink).

Examples:
    python3 compare_rust_python.py
    python3 compare_rust_python.py /path/to/test.launch.xml
    python3 compare_rust_python.py --profile release
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Set, Tuple

# Colors for terminal output
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    END = '\033[0m'

def get_autoware_launch_file(activate_script: Path) -> Path | None:
    """Find autoware_launch package and return path to planning_simulator.launch.xml.

    Uses ROS 2 to locate the package after sourcing the activation script.
    """
    if not activate_script.exists():
        return None

    # Use ros2 pkg prefix to find the autoware_launch package
    # Redirect activation messages to /dev/null and only capture ros2 command output
    cmd = [
        "bash", "-c",
        f"source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
        f"source {activate_script} >/dev/null 2>&1; "
        f"ros2 pkg prefix autoware_launch 2>/dev/null"
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode == 0 and result.stdout.strip():
            # Take the last line in case there are multiple lines
            pkg_prefix = Path(result.stdout.strip().split('\n')[-1])
            # Try common launch file locations
            candidates = [
                pkg_prefix / "share" / "autoware_launch" / "launch" / "planning_simulator.launch.xml",
                pkg_prefix / "launch" / "planning_simulator.launch.xml",
            ]
            for candidate in candidates:
                if candidate.exists():
                    return candidate
    except (subprocess.TimeoutExpired, Exception):
        pass

    return None

def get_paths(profile: str = "dev-release"):
    """Get important paths relative to script location."""
    script_dir = Path(__file__).parent

    # Detect if we're in autoware_test or comparison_tests
    if 'autoware_test' in str(script_dir):
        test_dir = script_dir.parent
        project_root = test_dir.parent.parent
    else:
        # In comparison_tests or tmp
        project_root = script_dir.parent.parent
        script_dir = project_root / "tests" / "autoware_test" / "scripts"

    # Rust binary location based on profile
    rust_bin = project_root / "target" / profile / "play_launch_parser"

    # Check for activation script
    activate_script = script_dir / "activate_autoware.sh"

    return {
        'script_dir': script_dir,
        'project_root': project_root,
        'rust_bin': rust_bin,
        'activate_script': activate_script if activate_script.exists() else None,
    }

def get_default_args_for_launch_file(launch_file: Path) -> Dict[str, str]:
    """Get default arguments for common Autoware launch files."""
    filename = launch_file.name

    if filename == "planning_simulator.launch.xml":
        # Minimal required arguments per Autoware documentation:
        # https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/
        #
        # Note: Our parser should load the YAML preset file automatically to get
        # velocity_smoother_type and other planning parameters, but there's currently
        # a context inheritance bug preventing YAML variables from being accessible
        # in child contexts. This will be fixed in a future update.
        return {
            "map_path": "/tmp/dummy_map",
            "vehicle_model": "sample_vehicle",
            "sensor_model": "sample_sensor_kit",
        }

    return {}

def run_rust_parser(launch_file: Path, paths: Dict, extra_args: Dict[str, str] = None) -> Dict[str, Any]:
    """Run Rust parser and return the record.json output."""
    print(f"{Colors.BLUE}Running Rust parser...{Colors.END}")

    rust_bin = paths['rust_bin']
    if not rust_bin.exists():
        print(f"{Colors.RED}Error: Rust binary not found at {rust_bin}{Colors.END}")
        print(f"Run: cargo build --profile {rust_bin.parent.name}")
        sys.exit(1)

    activate_script = paths.get('activate_script')

    # Build source command using activate script if available
    if activate_script and activate_script.exists():
        source_cmd = f"source /opt/ros/humble/setup.bash 2>/dev/null; source {activate_script} 2>/dev/null; "
    else:
        source_cmd = "source /opt/ros/humble/setup.bash 2>/dev/null; "

    # Build command with arguments
    rust_cmd = f"{rust_bin} file {launch_file}"

    # Add extra arguments if provided
    if extra_args:
        for key, value in extra_args.items():
            rust_cmd += f" {key}:={value}"

    cmd = [
        "bash", "-c",
        f"{source_cmd}{rust_cmd}"
    ]

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=60,
            cwd="/tmp"
        )

        if result.returncode != 0:
            print(f"{Colors.RED}Rust parser failed:{Colors.END}")
            print(result.stderr[-1000:])  # Last 1000 chars
            return None

        # record.json is created in the cwd (/tmp)
        record_file = Path("/tmp/record.json")
        if not record_file.exists():
            print(f"{Colors.RED}Error: record.json not found at {record_file}{Colors.END}")
            print(f"Rust stderr: {result.stderr[-500:]}")
            return None

        with open(record_file) as f:
            return json.load(f)

    except subprocess.TimeoutExpired:
        print(f"{Colors.RED}Rust parser timed out{Colors.END}")
        return None
    except Exception as e:
        print(f"{Colors.RED}Error running Rust parser: {e}{Colors.END}")
        return None

def run_python_dump(launch_file: Path, paths: Dict, extra_args: Dict[str, str] = None) -> Dict[str, Any]:
    """Run Python dump_launch and return the record.json output."""
    print(f"{Colors.BLUE}Running Python dump_launch...{Colors.END}")

    # Write Python output to tmp file
    python_output = Path("/tmp/python_dump_output.json")

    activate_script = paths.get('activate_script')

    # Try to find play_launch module - check common locations relative to project
    project_root = paths['project_root']
    play_launch_dirs = [
        project_root.parent / "play_launch" / "python",
        Path.home() / "repos" / "play_launch" / "python",
    ]

    play_launch_dir = None
    for dir_path in play_launch_dirs:
        if (dir_path / "play_launch" / "dump" / "__init__.py").exists():
            play_launch_dir = dir_path
            break

    if not play_launch_dir:
        print(f"{Colors.RED}Error: Could not find play_launch module{Colors.END}")
        print("Searched in:")
        for dir_path in play_launch_dirs:
            print(f"  - {dir_path}")
        return None

    # Build source command using activate script if available
    if activate_script and activate_script.exists():
        source_cmd = f"source /opt/ros/humble/setup.bash 2>/dev/null; source {activate_script} 2>/dev/null; "
    else:
        source_cmd = "source /opt/ros/humble/setup.bash 2>/dev/null; "

    # Build dump command with arguments
    dump_cmd = f"python3 -m play_launch.dump -o {python_output} {launch_file}"

    # Add extra arguments if provided
    if extra_args:
        for key, value in extra_args.items():
            dump_cmd += f" {key}:={value}"

    cmd = [
        "bash", "-c",
        f"{source_cmd}cd {play_launch_dir} && {dump_cmd}"
    ]

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=60,
            cwd="/tmp"
        )

        if result.returncode != 0:
            print(f"{Colors.RED}Python dump_launch failed:{Colors.END}")
            print(result.stderr[-1000:])
            return None

        # Read JSON from output file
        if not python_output.exists():
            print(f"{Colors.RED}Python output file not found: {python_output}{Colors.END}")
            return None

        with open(python_output) as f:
            return json.load(f)

    except subprocess.TimeoutExpired:
        print(f"{Colors.RED}Python dump_launch timed out{Colors.END}")
        return None
    except json.JSONDecodeError as e:
        print(f"{Colors.RED}Failed to parse Python output as JSON: {e}{Colors.END}")
        return None
    except Exception as e:
        print(f"{Colors.RED}Error running Python dump_launch: {e}{Colors.END}")
        return None

def compare_counts(rust_data: Dict, python_data: Dict) -> bool:
    """Compare counts of different entity types between Rust and Python parsers."""
    print(f"\n{Colors.BOLD}{'=' * 80}{Colors.END}")
    print(f"{Colors.BOLD}COUNT COMPARISON{Colors.END}")
    print(f"{Colors.BOLD}{'=' * 80}{Colors.END}\n")

    metrics = ['node', 'container', 'load_node', 'lifecycle_node']

    print(f"{'Metric':<20} {'Rust':<15} {'Python':<15} {'Diff':<10} {'Status'}")
    print("-" * 80)

    all_match = True
    for metric in metrics:
        rust_count = len(rust_data.get(metric, []))
        python_count = len(python_data.get(metric, []))

        # Both parsers now count things the same way (containers are separate from nodes)
        diff = rust_count - python_count

        if diff == 0:
            status = f"{Colors.GREEN}✓{Colors.END}"
        else:
            status = f"{Colors.YELLOW}✗{Colors.END}"
            all_match = False

        print(f"{metric:<20} {rust_count:<15} {python_count:<15} {diff:+10} {status}")

    return all_match

def normalize_namespace(ns: str) -> str:
    """Normalize namespace to have leading slash."""
    if not ns:
        return "/"
    if ns.startswith('/'):
        return ns
    return f"/{ns}"

def compare_containers(rust_data: Dict, python_data: Dict) -> bool:
    """Compare container entries."""
    print(f"\n{Colors.BOLD}{'=' * 80}{Colors.END}")
    print(f"{Colors.BOLD}CONTAINER COMPARISON{Colors.END}")
    print(f"{Colors.BOLD}{'=' * 80}{Colors.END}\n")

    rust_containers = {c['name']: c for c in rust_data.get('container', [])}
    python_containers = {c['name']: c for c in python_data.get('container', [])}

    all_names = sorted(set(rust_containers.keys()) | set(python_containers.keys()))

    matches = 0
    mismatches = 0

    for name in all_names:
        rust_c = rust_containers.get(name)
        python_c = python_containers.get(name)

        if rust_c and python_c:
            # Both have it, compare namespaces
            rust_ns = normalize_namespace(rust_c.get('namespace', '/'))
            python_ns = normalize_namespace(python_c.get('namespace', '/'))

            if rust_ns == python_ns:
                print(f"{Colors.GREEN}✓{Colors.END} {name:<40} (namespace: {rust_ns})")
                matches += 1
            else:
                print(f"{Colors.YELLOW}✗{Colors.END} {name:<40}")
                print(f"  Rust:   {rust_ns}")
                print(f"  Python: {python_ns}")
                mismatches += 1
        elif rust_c:
            print(f"{Colors.RED}✗{Colors.END} {name:<40} (only in Rust)")
            mismatches += 1
        else:
            print(f"{Colors.RED}✗{Colors.END} {name:<40} (only in Python)")
            mismatches += 1

    print(f"\nMatches: {matches}, Mismatches: {mismatches}")
    return mismatches == 0

def compare_load_nodes(rust_data: Dict, python_data: Dict) -> bool:
    """Compare composable node entries."""
    print(f"\n{Colors.BOLD}{'=' * 80}{Colors.END}")
    print(f"{Colors.BOLD}COMPOSABLE NODE COMPARISON{Colors.END}")
    print(f"{Colors.BOLD}{'=' * 80}{Colors.END}\n")

    rust_nodes = rust_data.get('load_node', [])
    python_nodes = python_data.get('load_node', [])

    # Group by target container
    def group_by_container(nodes):
        groups = {}
        for node in nodes:
            container = node.get('target_container_name', 'unknown')
            if container not in groups:
                groups[container] = []
            groups[container].append(node)
        return groups

    rust_by_container = group_by_container(rust_nodes)
    python_by_container = group_by_container(python_nodes)

    all_containers = sorted(set(rust_by_container.keys()) | set(python_by_container.keys()))

    total_matches = 0
    total_mismatches = 0

    for container in all_containers:
        rust_nodes_in_container = rust_by_container.get(container, [])
        python_nodes_in_container = python_by_container.get(container, [])

        rust_count = len(rust_nodes_in_container)
        python_count = len(python_nodes_in_container)

        if rust_count == python_count:
            print(f"{Colors.GREEN}✓{Colors.END} {container:<50} ({rust_count} nodes)")
            total_matches += rust_count
        else:
            print(f"{Colors.YELLOW}✗{Colors.END} {container:<50} (Rust: {rust_count}, Python: {python_count})")
            total_mismatches += abs(rust_count - python_count)

            # Show which plugins differ
            rust_plugins = {n['plugin'] for n in rust_nodes_in_container}
            python_plugins = {n['plugin'] for n in python_nodes_in_container}

            only_rust = rust_plugins - python_plugins
            only_python = python_plugins - rust_plugins

            if only_rust:
                print(f"  Only in Rust: {', '.join(sorted(only_rust))}")
            if only_python:
                print(f"  Only in Python: {', '.join(sorted(only_python))}")

    print(f"\nTotal composable nodes - Rust: {len(rust_nodes)}, Python: {len(python_nodes)}")
    print(f"Matching nodes: {total_matches}, Mismatches: {total_mismatches}")

    return total_mismatches == 0

def compare_nodes(rust_data: Dict, python_data: Dict) -> bool:
    """Compare regular node entries."""
    print(f"\n{Colors.BOLD}{'=' * 80}{Colors.END}")
    print(f"{Colors.BOLD}NODE COMPARISON{Colors.END}")
    print(f"{Colors.BOLD}{'=' * 80}{Colors.END}\n")

    rust_nodes = {n['name']: n for n in rust_data.get('node', []) if n.get('name')}
    python_nodes = {n['name']: n for n in python_data.get('node', []) if n.get('name')}

    # Python duplicates containers as nodes - filter them out
    python_containers = {c['name'] for c in python_data.get('container', [])}
    python_nodes_no_containers = {k: v for k, v in python_nodes.items() if k not in python_containers}

    all_names = sorted(set(rust_nodes.keys()) | set(python_nodes_no_containers.keys()))

    matches = 0
    only_rust = []
    only_python = []

    for name in all_names:
        if name in rust_nodes and name in python_nodes_no_containers:
            matches += 1
        elif name in rust_nodes:
            only_rust.append(name)
        else:
            only_python.append(name)

    print(f"Matching nodes: {Colors.GREEN}{matches}{Colors.END}")

    if only_rust:
        print(f"\n{Colors.YELLOW}Nodes only in Rust ({len(only_rust)}):{Colors.END}")
        for name in only_rust[:10]:
            print(f"  - {name}")
        if len(only_rust) > 10:
            print(f"  ... and {len(only_rust) - 10} more")

    if only_python:
        print(f"\n{Colors.YELLOW}Nodes only in Python ({len(only_python)}):{Colors.END}")
        for name in only_python[:10]:
            print(f"  - {name}")
        if len(only_python) > 10:
            print(f"  ... and {len(only_python) - 10} more")

    return len(only_rust) == 0 and len(only_python) == 0

def main():
    """Main test function."""
    parser = argparse.ArgumentParser(
        description="Compare Rust and Python launch file parsers on Autoware",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                           # Use default planning_simulator.launch.xml
  %(prog)s /path/to/test.launch.xml  # Test specific file
  %(prog)s --profile release         # Use release build
        """
    )
    parser.add_argument('launch_file', nargs='?', help='Path to the launch file to parse (optional)')
    parser.add_argument('--profile', default='dev-release',
                        help='Cargo build profile (default: dev-release)')

    args = parser.parse_args()

    paths = get_paths(args.profile)

    # Determine launch file to test
    if args.launch_file:
        # User provided a launch file
        if args.launch_file.startswith('autoware/'):
            launch_file = paths['autoware_ws'] / args.launch_file[9:]  # Strip 'autoware/'
        else:
            launch_file = Path(args.launch_file)
            # If relative path, resolve relative to script directory
            if not launch_file.is_absolute():
                launch_file = (paths['script_dir'] / launch_file).resolve()
    else:
        # No argument provided - use default Autoware launch file
        if paths['activate_script'] is None:
            print(f"{Colors.RED}Error: activate_autoware.sh not found{Colors.END}")
            print(f"\nCreate activation script:")
            print(f"  cd tests/autoware_test/scripts")
            print(f"  cp activate_autoware.sh.template activate_autoware.sh")
            print(f"  # Edit activate_autoware.sh to source your Autoware workspace")
            print(f"\nOr provide a launch file:")
            print(f"  {sys.argv[0]} <launch_file>")
            sys.exit(1)

        # Use ROS 2 to find autoware_launch package
        launch_file = get_autoware_launch_file(paths['activate_script'])

        if launch_file is None:
            print(f"{Colors.RED}Error: Could not find autoware_launch package{Colors.END}")
            print(f"\nMake sure:")
            print(f"  1. activate_autoware.sh sources a valid Autoware workspace")
            print(f"  2. autoware_launch package is installed")
            print(f"\nOr provide a launch file:")
            print(f"  {sys.argv[0]} <launch_file>")
            sys.exit(1)

        print(f"{Colors.BLUE}No launch file specified, using default:{Colors.END} {launch_file.name}\n")

    if not launch_file.exists():
        print(f"{Colors.RED}Error: Launch file not found: {launch_file}{Colors.END}")
        sys.exit(1)

    print(f"{Colors.BOLD}{'=' * 80}{Colors.END}")
    print(f"{Colors.BOLD}RUST vs PYTHON PARSER COMPARISON{Colors.END}")
    print(f"{Colors.BOLD}{'=' * 80}{Colors.END}")
    print(f"\nLaunch file: {launch_file}")

    # Get default arguments for this launch file
    default_args = get_default_args_for_launch_file(launch_file)
    if default_args:
        print(f"{Colors.BLUE}Using default arguments:{Colors.END}")
        for key, value in default_args.items():
            print(f"  {key}:={value}")
    print()

    # Run both parsers
    rust_data = run_rust_parser(launch_file, paths, default_args)
    if rust_data is None:
        print(f"\n{Colors.RED}Failed to run Rust parser{Colors.END}")
        sys.exit(1)

    python_data = run_python_dump(launch_file, paths, default_args)
    if python_data is None:
        print(f"\n{Colors.RED}Failed to run Python dump_launch{Colors.END}")
        sys.exit(1)

    # Save outputs for manual inspection
    output_dir = paths['script_dir']
    with open(output_dir / "rust_output.json", "w") as f:
        json.dump(rust_data, f, indent=2)
    with open(output_dir / "python_output.json", "w") as f:
        json.dump(python_data, f, indent=2)

    print(f"{Colors.GREEN}✓{Colors.END} Saved outputs to:")
    print(f"  - {output_dir / 'rust_output.json'}")
    print(f"  - {output_dir / 'python_output.json'}")

    # Compare
    counts_match = compare_counts(rust_data, python_data)
    containers_match = compare_containers(rust_data, python_data)
    load_nodes_match = compare_load_nodes(rust_data, python_data)
    nodes_match = compare_nodes(rust_data, python_data)

    # Summary
    print(f"\n{Colors.BOLD}{'=' * 80}{Colors.END}")
    print(f"{Colors.BOLD}SUMMARY{Colors.END}")
    print(f"{Colors.BOLD}{'=' * 80}{Colors.END}\n")

    all_match = counts_match and containers_match and load_nodes_match and nodes_match

    if all_match:
        print(f"{Colors.GREEN}{Colors.BOLD}✓ ALL CHECKS PASSED{Colors.END}")
        print(f"{Colors.GREEN}Rust and Python implementations produce identical outputs!{Colors.END}")
        sys.exit(0)
    else:
        print(f"{Colors.YELLOW}{Colors.BOLD}⚠ SOME DIFFERENCES FOUND{Colors.END}")
        print(f"\nResults:")
        print(f"  Counts: {Colors.GREEN if counts_match else Colors.YELLOW}{'✓' if counts_match else '✗'}{Colors.END}")
        print(f"  Containers: {Colors.GREEN if containers_match else Colors.YELLOW}{'✓' if containers_match else '✗'}{Colors.END}")
        print(f"  Composable nodes: {Colors.GREEN if load_nodes_match else Colors.YELLOW}{'✓' if load_nodes_match else '✗'}{Colors.END}")
        print(f"  Regular nodes: {Colors.GREEN if nodes_match else Colors.YELLOW}{'✓' if nodes_match else '✗'}{Colors.END}")
        print(f"\nCheck the detailed output above and the JSON files in {output_dir}")
        sys.exit(1)

if __name__ == "__main__":
    main()
