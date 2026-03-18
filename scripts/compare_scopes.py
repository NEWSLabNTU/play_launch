#!/usr/bin/env python3
"""Compare scope tables between Rust and Python parser outputs.

Usage:
    python3 scripts/compare_scopes.py rust_record.json python_record.json

Or via just recipe:
    just compare-scopes <launch_pkg> <launch_file> [args...]
"""

import json
import sys
from collections import Counter
from pathlib import Path


def load_record(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def scope_identity(scopes: list[dict], s: dict) -> tuple:
    """Return (pkg, file, parent_pkg, parent_file) for a scope."""
    parent = scopes[s["parent"]] if s["parent"] is not None else None
    return (
        s.get("pkg"),
        s["file"],
        parent.get("pkg") if parent else None,
        parent["file"] if parent else None,
    )


def node_fqn(node: dict) -> str:
    """Build a comparable node identifier."""
    ns = node.get("namespace") or "/"
    name = node.get("name") or node.get("node_name") or node.get("executable", "?")
    if name and name.startswith("/"):
        return name
    if ns.endswith("/"):
        return f"{ns}{name}"
    return f"{ns}/{name}"


def compare_scopes(rust_path: str, python_path: str) -> int:
    rust = load_record(rust_path)
    python = load_record(python_path)

    rust_scopes = rust.get("scopes", [])
    python_scopes = python.get("scopes", [])

    errors = 0

    # --- Scope count ---
    print(f"Rust:   {len(rust_scopes)} scopes")
    print(f"Python: {len(python_scopes)} scopes")

    if len(rust_scopes) != len(python_scopes):
        print(f"  MISMATCH: scope count differs")
        errors += 1
    else:
        print(f"  OK: scope counts match")
    print()

    # --- (pkg, file) counts ---
    rc = Counter((s.get("pkg"), s["file"]) for s in rust_scopes)
    pc = Counter((s.get("pkg"), s["file"]) for s in python_scopes)

    all_keys = sorted(set(rc.keys()) | set(pc.keys()))
    pkg_file_diffs = []
    for key in all_keys:
        r = rc.get(key, 0)
        p = pc.get(key, 0)
        if r != p:
            pkg_file_diffs.append((key, r, p))

    if pkg_file_diffs:
        print(f"(pkg, file) count mismatches ({len(pkg_file_diffs)}):")
        for (pkg, file), r, p in pkg_file_diffs:
            print(f"  {pkg}/{file}: Rust={r} Python={p}")
        errors += len(pkg_file_diffs)
    else:
        print("(pkg, file) counts: all match")
    print()

    # --- Scope identity with parents ---
    rust_ids = Counter(scope_identity(rust_scopes, s) for s in rust_scopes)
    python_ids = Counter(scope_identity(python_scopes, s) for s in python_scopes)

    only_rust = set(rust_ids.keys()) - set(python_ids.keys())
    only_python = set(python_ids.keys()) - set(rust_ids.keys())

    if only_rust:
        print(f"Scope identities only in Rust ({len(only_rust)}):")
        for pkg, file, ppkg, pfile in sorted(only_rust):
            print(f"  {pkg}/{file}  (parent: {ppkg}/{pfile})")
        errors += len(only_rust)

    if only_python:
        print(f"Scope identities only in Python ({len(only_python)}):")
        for pkg, file, ppkg, pfile in sorted(only_python):
            print(f"  {pkg}/{file}  (parent: {ppkg}/{pfile})")
        errors += len(only_python)

    if not only_rust and not only_python:
        print("Scope identities (pkg, file, parent): all match")
    print()

    # --- Entity counts ---
    for kind in ["node", "container", "load_node"]:
        r_count = len(rust.get(kind, []))
        p_count = len(python.get(kind, []))
        status = "OK" if r_count == p_count else "MISMATCH"
        print(f"{kind}: Rust={r_count} Python={p_count}  {status}")
        if r_count != p_count:
            errors += 1
    print()

    # --- Per-node scope comparison ---
    # Build scope lookup: scope_id → (pkg, file)
    def scope_pkg_file(scopes, scope_id):
        if scope_id is None or scope_id >= len(scopes):
            return (None, None)
        s = scopes[scope_id]
        return (s.get("pkg"), s["file"])

    # Compare nodes
    mismatches = 0
    for kind in ["node", "container", "load_node"]:
        rust_entities = rust.get(kind, [])
        python_entities = python.get(kind, [])

        # Build FQN → scope (pkg, file) maps
        rust_map = {}
        for e in rust_entities:
            fqn = node_fqn(e)
            rust_map[fqn] = scope_pkg_file(rust_scopes, e.get("scope"))

        python_map = {}
        for e in python_entities:
            fqn = node_fqn(e)
            python_map[fqn] = scope_pkg_file(python_scopes, e.get("scope"))

        common = set(rust_map.keys()) & set(python_map.keys())
        for fqn in sorted(common):
            r_scope = rust_map[fqn]
            p_scope = python_map[fqn]
            if r_scope != p_scope:
                print(f"  {kind} '{fqn}': Rust scope={r_scope}  Python scope={p_scope}")
                mismatches += 1

    if mismatches:
        print(f"\nPer-entity scope mismatches: {mismatches}")
        errors += mismatches
    else:
        print("Per-entity scope assignments: all match")

    # --- Summary ---
    print()
    if errors == 0:
        print("PASS: all scope comparisons match")
        return 0
    else:
        print(f"FAIL: {errors} difference(s) found")
        return 1


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <rust_record.json> <python_record.json>")
        sys.exit(2)

    sys.exit(compare_scopes(sys.argv[1], sys.argv[2]))


if __name__ == "__main__":
    main()
