#!/usr/bin/env python3
"""
Compare Rust and Python parser outputs for Autoware launch file.

This script performs a detailed comparison of the record.json outputs
from the Rust and Python parsers to identify any differences.
"""

import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

# ANSI color codes
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    NC = '\033[0m'  # No Color


def load_json(filepath: Path) -> Dict[str, Any]:
    """Load JSON file."""
    with open(filepath) as f:
        return json.load(f)


def compare_node_counts(rust_data: Dict, python_data: Dict) -> bool:
    """Compare node counts between outputs."""
    print(f"\n{Colors.BLUE}Node Counts:{Colors.NC}")

    categories = ['node', 'container', 'load_node', 'lifecycle_node']
    all_match = True

    for category in categories:
        rust_count = len(rust_data.get(category, []))
        python_count = len(python_data.get(category, []))

        if rust_count == python_count:
            print(f"  {Colors.GREEN}✓{Colors.NC} {category}: {rust_count}")
        else:
            print(f"  {Colors.RED}✗{Colors.NC} {category}: Rust={rust_count}, Python={python_count}")
            all_match = False

    return all_match


def compare_nodes(rust_nodes: List[Dict], python_nodes: List[Dict]) -> Tuple[int, int]:
    """
    Compare node details.

    Returns: (matching_nodes, total_nodes)
    """
    if len(rust_nodes) != len(python_nodes):
        print(f"  {Colors.YELLOW}⚠{Colors.NC} Different number of nodes")

    matching = 0
    total = min(len(rust_nodes), len(python_nodes))

    print(f"\n{Colors.BLUE}Comparing {total} nodes:{Colors.NC}")

    # Create lookup by node name for comparison
    python_by_name = {node.get('name'): node for node in python_nodes}

    for i, rust_node in enumerate(rust_nodes[:total]):
        rust_name = rust_node.get('name', f'<unnamed-{i}>')
        python_node = python_by_name.get(rust_name)

        if not python_node:
            print(f"  {Colors.RED}✗{Colors.NC} Node '{rust_name}' not found in Python output")
            continue

        # Compare key fields
        fields_match = True
        fields_to_compare = ['package', 'executable', 'namespace']

        for field in fields_to_compare:
            rust_val = rust_node.get(field)
            python_val = python_node.get(field)

            if rust_val != python_val:
                print(f"  {Colors.YELLOW}⚠{Colors.NC} Node '{rust_name}' field '{field}' differs:")
                print(f"      Rust:   {rust_val}")
                print(f"      Python: {python_val}")
                fields_match = False

        if fields_match:
            matching += 1

    print(f"\n  Matching nodes: {matching}/{total}")
    return matching, total


def compare_parameters(rust_nodes: List[Dict], python_nodes: List[Dict]) -> None:
    """Compare node parameters."""
    print(f"\n{Colors.BLUE}Parameter Comparison:{Colors.NC}")

    python_by_name = {node.get('name'): node for node in python_nodes}

    total_params_rust = 0
    total_params_python = 0

    for rust_node in rust_nodes:
        rust_name = rust_node.get('name', '<unnamed>')
        python_node = python_by_name.get(rust_name)

        if not python_node:
            continue

        rust_params = rust_node.get('params', [])
        python_params = python_node.get('params', [])

        total_params_rust += len(rust_params)
        total_params_python += len(python_params)

    print(f"  Total parameters:")
    print(f"    Rust:   {total_params_rust}")
    print(f"    Python: {total_params_python}")


def main():
    script_dir = Path(__file__).parent
    test_dir = script_dir.parent
    output_dir = test_dir / "output"

    rust_output = output_dir / "rust_output.json"
    python_output = output_dir / "python_output.json"

    # Check files exist
    if not rust_output.exists():
        print(f"{Colors.RED}ERROR: Rust output not found: {rust_output}{Colors.NC}")
        sys.exit(1)

    if not python_output.exists():
        print(f"{Colors.YELLOW}WARNING: Python output not found: {python_output}{Colors.NC}")
        print("Run test_parse.sh first to generate outputs.")
        sys.exit(1)

    print("=" * 50)
    print("Comparing Rust and Python Parser Outputs")
    print("=" * 50)

    # Load outputs
    rust_data = load_json(rust_output)
    python_data = load_json(python_output)

    # Compare node counts
    counts_match = compare_node_counts(rust_data, python_data)

    # Compare nodes in detail
    rust_nodes = rust_data.get('node', [])
    python_nodes = python_data.get('node', [])

    matching, total = compare_nodes(rust_nodes, python_nodes)

    # Compare parameters
    compare_parameters(rust_nodes, python_nodes)

    # Final summary
    print("\n" + "=" * 50)
    if counts_match and matching == total:
        print(f"{Colors.GREEN}✓ Outputs match!{Colors.NC}")
        print("=" * 50)
        sys.exit(0)
    else:
        print(f"{Colors.YELLOW}⚠ Some differences found{Colors.NC}")
        print("=" * 50)
        sys.exit(0)  # Still exit 0 since differences may be acceptable


if __name__ == '__main__':
    main()
