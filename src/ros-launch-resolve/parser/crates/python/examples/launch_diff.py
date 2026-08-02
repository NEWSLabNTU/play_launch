#!/usr/bin/env python3
"""Compare two launch configurations with different arguments."""

import sys

from play_launch_parser import parse_file


def node_set(result):
    """Extract a set of (namespace, name, package) tuples."""
    nodes = set()
    for n in result["node"]:
        nodes.add((n["namespace"], n["name"], n["package"]))
    for c in result["container"]:
        nodes.add((c["namespace"], c["name"], c["package"]))
    for ln in result["load_node"]:
        nodes.add((ln["namespace"], ln["node_name"], ln["package"]))
    return nodes


def main():
    if len(sys.argv) < 2:
        print("Usage: launch_diff.py <launch_file> [key:=val_a,val_b ...]")
        print()
        print("Example:")
        print("  launch_diff.py my.launch.xml mode:=online,offline")
        sys.exit(1)

    path = sys.argv[1]

    # Parse key:=val_a,val_b pairs
    args_a, args_b = {}, {}
    for arg in sys.argv[2:]:
        key, vals = arg.split(":=", 1)
        a, b = vals.split(",", 1)
        args_a[key] = a
        args_b[key] = b

    result_a = parse_file(path, args=args_a)
    result_b = parse_file(path, args=args_b)

    set_a = node_set(result_a)
    set_b = node_set(result_b)

    only_a = set_a - set_b
    only_b = set_b - set_a

    if only_a:
        print(f"Only in config A {args_a}:")
        for ns, name, pkg in sorted(only_a):
            print(f"  {ns}/{name}  ({pkg})")

    if only_b:
        print(f"\nOnly in config B {args_b}:")
        for ns, name, pkg in sorted(only_b):
            print(f"  {ns}/{name}  ({pkg})")

    if not only_a and not only_b:
        print("Both configurations produce the same nodes.")

    print(f"\nA: {len(set_a)} nodes, B: {len(set_b)} nodes")


if __name__ == "__main__":
    main()
