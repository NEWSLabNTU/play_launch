#!/usr/bin/env python3
"""Find which launch file defines a specific node."""

import sys

from play_launch_parser import parse_file


def main():
    if len(sys.argv) < 3:
        print("Usage: find_node.py <launch_file> <node_name_substring>")
        print()
        print("Example:")
        print("  find_node.py planning_simulator.launch.xml pointcloud_preprocessor")
        sys.exit(1)

    path = sys.argv[1]
    query = sys.argv[2].lower()

    result = parse_file(path)
    scopes = result.get("scopes", [])

    def scope_label(scope_id):
        if scope_id is None or scope_id >= len(scopes):
            return "?"
        s = scopes[scope_id]
        origin = s.get("origin")
        if origin:
            pkg = origin.get("pkg", "?")
            return f"{pkg}/{origin['file']}"
        return f"scope:{scope_id}"

    found = False
    for node in result["node"]:
        name = node.get("name") or node.get("executable") or ""
        ns = node.get("namespace") or "/"
        fqn = f"{ns}/{name}"
        if query in fqn.lower() or query in name.lower():
            pkg = node.get("package") or "-"
            src = scope_label(node.get("scope"))
            print(f"  node: {fqn}  pkg={pkg}  from={src}")
            found = True

    for ln in result["load_node"]:
        fqn = f"{ln['namespace']}/{ln['node_name']}"
        if query in fqn.lower() or query in ln["node_name"].lower():
            src = scope_label(ln.get("scope"))
            print(f"  composable: {fqn}  plugin={ln['plugin']}  -> {ln['target_container_name']}  from={src}")
            found = True

    if not found:
        print(f"No nodes matching '{query}'")


if __name__ == "__main__":
    main()
