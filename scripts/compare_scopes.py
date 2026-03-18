#!/usr/bin/env python3
"""Compare scope tables and per-node context between Rust and Python parser outputs.

Usage:
    # Compare two records (cross-parser)
    python3 scripts/compare_scopes.py rust_record.json python_record.json

    # Validate a single record (self-consistency)
    python3 scripts/compare_scopes.py --validate record.json

Or via just recipe:
    just compare-scopes <launch_pkg> <launch_file> [args...]
"""

import json
import sys
from collections import Counter


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


def validate_record(record: dict) -> int:
    """Validate self-consistency of a single record's scope data."""
    scopes = record.get("scopes", [])
    errors = 0

    print(f"Scopes: {len(scopes)}")

    if not scopes:
        print("  WARN: no scopes found")
        return 0

    # 1. Scope tree structure
    print("\n--- Scope tree structure ---")

    # Check root exists
    roots = [s for s in scopes if s.get("parent") is None]
    if not roots:
        print("  ERROR: no root scope (parent=null)")
        errors += 1
    elif len(roots) > 1:
        print(f"  WARN: {len(roots)} root scopes (expected 1)")

    # Check all parent references are valid
    for s in scopes:
        if s.get("parent") is not None:
            if s["parent"] >= len(scopes):
                print(f"  ERROR: scope {s['id']} has invalid parent {s['parent']}")
                errors += 1
            elif s["parent"] == s["id"]:
                print(f"  ERROR: scope {s['id']} is its own parent")
                errors += 1

    # Check sequential IDs
    for i, s in enumerate(scopes):
        if s["id"] != i:
            print(f"  ERROR: scope at index {i} has id {s['id']}")
            errors += 1

    # Check no cycles (walk from each scope to root)
    for s in scopes:
        visited = set()
        current = s["id"]
        while current is not None:
            if current in visited:
                print(f"  ERROR: cycle detected involving scope {s['id']}")
                errors += 1
                break
            visited.add(current)
            if current < len(scopes):
                current = scopes[current].get("parent")
            else:
                break

    if errors == 0:
        print("  OK: tree structure valid")

    # 2. Scope coverage — every entity has a scope
    print("\n--- Scope coverage ---")
    for kind in ["node", "container", "load_node"]:
        entities = record.get(kind, [])
        no_scope = [e for e in entities if e.get("scope") is None]
        invalid_scope = [
            e for e in entities
            if e.get("scope") is not None and e["scope"] >= len(scopes)
        ]
        if no_scope:
            print(f"  ERROR: {len(no_scope)} {kind}(s) without scope")
            errors += len(no_scope)
        if invalid_scope:
            print(f"  ERROR: {len(invalid_scope)} {kind}(s) with invalid scope id")
            errors += len(invalid_scope)

    total = sum(len(record.get(k, [])) for k in ["node", "container", "load_node"])
    scoped = sum(
        1
        for k in ["node", "container", "load_node"]
        for e in record.get(k, [])
        if e.get("scope") is not None
    )
    print(f"  Entities: {scoped}/{total} scoped")
    if scoped == total:
        print("  OK: all entities have valid scopes")

    # 3. Namespace consistency — scope ns should be prefix of node namespace
    print("\n--- Namespace consistency ---")
    ns_mismatches = 0
    for kind in ["node", "container"]:
        for e in record.get(kind, []):
            scope_id = e.get("scope")
            if scope_id is None or scope_id >= len(scopes):
                continue
            scope_ns = scopes[scope_id]["ns"]
            entity_ns = e.get("namespace") or "/"

            # Scope ns should be a prefix of entity ns, or they should share
            # a common path. The scope ns represents the accumulated namespace
            # at the include boundary. The entity may add its own namespace.
            # Normalize both to compare.
            scope_ns_norm = scope_ns.rstrip("/") or "/"
            entity_ns_norm = entity_ns.rstrip("/") or "/"

            # The entity namespace should start with the scope namespace
            # (or be equal), UNLESS the node uses an absolute namespace override.
            if not entity_ns_norm.startswith(scope_ns_norm) and scope_ns_norm != "/":
                # This can happen legitimately when a node overrides namespace
                # with an absolute path. Just log as info, not error.
                ns_mismatches += 1

    if ns_mismatches > 0:
        print(f"  INFO: {ns_mismatches} entities with namespace outside scope "
              f"(may be absolute namespace overrides)")
    else:
        print("  OK: all entity namespaces consistent with scope")

    # Summary
    print()
    if errors == 0:
        print(f"VALIDATE PASS: record is self-consistent")
        return 0
    else:
        print(f"VALIDATE FAIL: {errors} error(s)")
        return 1


def compare_node_context(rust: dict, python: dict) -> int:
    """Compare per-node resolved values between Rust and Python parsers."""
    errors = 0

    print("--- Per-node context comparison ---")

    for kind in ["node", "container", "load_node"]:
        rust_entities = {node_fqn(e): e for e in rust.get(kind, [])}
        python_entities = {node_fqn(e): e for e in python.get(kind, [])}

        common = sorted(set(rust_entities.keys()) & set(python_entities.keys()))
        only_rust = sorted(set(rust_entities.keys()) - set(python_entities.keys()))
        only_python = sorted(set(python_entities.keys()) - set(rust_entities.keys()))

        if only_rust:
            print(f"  {kind} only in Rust: {only_rust}")
            errors += len(only_rust)
        if only_python:
            print(f"  {kind} only in Python: {only_python}")
            errors += len(only_python)

        # Compare fields for common entities
        fields_to_compare = ["namespace"]
        if kind in ("node", "container"):
            fields_to_compare.extend(["executable", "package", "name"])
        if kind == "load_node":
            fields_to_compare.extend(["package", "plugin", "node_name",
                                      "target_container_name"])

        field_mismatches = 0
        for fqn in common:
            re = rust_entities[fqn]
            pe = python_entities[fqn]
            for field in fields_to_compare:
                rv = re.get(field)
                pv = pe.get(field)
                if rv != pv:
                    print(f"  {kind} '{fqn}' field '{field}': "
                          f"Rust={rv!r} Python={pv!r}")
                    field_mismatches += 1

        if field_mismatches:
            errors += field_mismatches

    if errors == 0:
        print("  OK: all per-node context matches")

    return errors


def compare_scopes(rust_path: str, python_path: str) -> int:
    rust = load_record(rust_path)
    python = load_record(python_path)

    rust_scopes = rust.get("scopes", [])
    python_scopes = python.get("scopes", [])

    errors = 0

    # --- Validate each record ---
    print("=== Validating Rust record ===")
    errors += validate_record(rust)
    print()
    print("=== Validating Python record ===")
    errors += validate_record(python)
    print()

    # --- Scope count ---
    print("=== Cross-parser scope comparison ===")
    print(f"Rust:   {len(rust_scopes)} scopes")
    print(f"Python: {len(python_scopes)} scopes")

    if len(rust_scopes) != len(python_scopes):
        print("  MISMATCH: scope count differs")
        errors += 1
    else:
        print("  OK: scope counts match")
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
    def scope_pkg_file(scopes, scope_id):
        if scope_id is None or scope_id >= len(scopes):
            return (None, None)
        s = scopes[scope_id]
        return (s.get("pkg"), s["file"])

    scope_mismatches = 0
    for kind in ["node", "container", "load_node"]:
        rust_map = {node_fqn(e): scope_pkg_file(rust_scopes, e.get("scope"))
                    for e in rust.get(kind, [])}
        python_map = {node_fqn(e): scope_pkg_file(python_scopes, e.get("scope"))
                      for e in python.get(kind, [])}

        for fqn in sorted(set(rust_map.keys()) & set(python_map.keys())):
            if rust_map[fqn] != python_map[fqn]:
                print(f"  {kind} '{fqn}': Rust scope={rust_map[fqn]}  "
                      f"Python scope={python_map[fqn]}")
                scope_mismatches += 1

    if scope_mismatches:
        print(f"\nPer-entity scope mismatches: {scope_mismatches}")
        errors += scope_mismatches
    else:
        print("Per-entity scope assignments: all match")
    print()

    # --- Per-node context comparison ---
    errors += compare_node_context(rust, python)
    print()

    # --- Summary ---
    if errors == 0:
        print("PASS: all comparisons match")
        return 0
    else:
        print(f"FAIL: {errors} difference(s) found")
        return 1


def main():
    if len(sys.argv) == 3 and sys.argv[1] != "--validate":
        sys.exit(compare_scopes(sys.argv[1], sys.argv[2]))
    elif len(sys.argv) == 3 and sys.argv[1] == "--validate":
        record = load_record(sys.argv[2])
        sys.exit(validate_record(record))
    else:
        print(f"Usage:")
        print(f"  {sys.argv[0]} <rust_record.json> <python_record.json>")
        print(f"  {sys.argv[0]} --validate <record.json>")
        sys.exit(2)


if __name__ == "__main__":
    main()
