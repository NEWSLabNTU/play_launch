#!/usr/bin/env python3
"""
Compare two record.json files for functional equivalence.
Part of Phase 13: Rust Parser Migration

Performs two levels of comparison:
  1. Exact match: raw field-by-field equality (after sorting for stable ordering)
  2. Functional equivalence: normalizes known acceptable differences:
     - params_files expansion (Rust keeps YAML, Python expands inline)
     - XML whitespace (xacro newlines vs spaces)
     - Array quoting: [a, b] vs ['a', 'b']
     - cmd param deduplication (Python duplicates from multiple includes)
     - cmd param/remap ordering
     - Container ordering (cosmetic, all load_node targets valid)
     - None vs empty list/args=['']
"""

import json
import re
import sys
from typing import Any, Dict, List, Optional, Tuple
from pathlib import Path

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False


# =============================================================================
# Value normalization helpers
# =============================================================================

def normalize_whitespace(value: str) -> str:
    """Collapse all whitespace (newlines, tabs, spaces) to single space."""
    return re.sub(r'\s+', ' ', value).strip()


def normalize_array_quoting(value: str) -> str:
    """Normalize array quoting: ['a', 'b'] -> [a, b]."""
    m = re.match(r'^\[(.+)\]$', value.strip())
    if not m:
        return value
    elements = [elem.strip().strip("'\"") for elem in m.group(1).split(',')]
    return '[' + ', '.join(elements) + ']'


def normalize_param_value(value: str) -> str:
    """Normalize a single param value for equivalence comparison."""
    return normalize_array_quoting(normalize_whitespace(value))


def python_repr_value(obj: Any) -> str:
    """Convert a YAML value to Python-style string representation.

    Matches how Python's parser serializes param values.
    """
    if isinstance(obj, bool):
        return str(obj)  # True/False
    if isinstance(obj, (dict, list)):
        return repr(obj)
    if obj is None:
        return ''
    return str(obj)


def extract_params_from_yaml_content(yaml_content: str) -> List[Tuple[str, str]]:
    """Extract (key, value) params from YAML content string.

    Handles ROS 2 YAML with /** and ros__parameters wrappers.
    Nested dicts/lists are kept as single values (matching Python behavior).
    """
    if not HAS_YAML:
        return []
    try:
        data = yaml.safe_load(yaml_content)
    except yaml.YAMLError:
        return []
    if not isinstance(data, dict):
        return []

    params: List[Tuple[str, str]] = []

    def extract(prefix: str, obj: Any) -> None:
        if isinstance(obj, dict):
            for k, v in obj.items():
                if k in ('/**', 'ros__parameters'):
                    extract(prefix, v)
                else:
                    new_key = f"{prefix}.{k}" if prefix else k
                    params.append((new_key, python_repr_value(v)))
        elif prefix:
            params.append((prefix, python_repr_value(obj)))

    extract('', data)
    return params


# =============================================================================
# cmd parsing
# =============================================================================

def parse_cmd(cmd: List[str]) -> Dict[str, Any]:
    """Parse a cmd array into structured components."""
    result: Dict[str, Any] = {
        'executable': cmd[0] if cmd else '',
        'args_before_ros': [],
        'node_name': None,
        'namespace': None,
        'params': {},
        'params_files': [],
        'remaps': [],
    }
    if not cmd:
        return result

    ros_idx = None
    for i, c in enumerate(cmd):
        if c == '--ros-args':
            ros_idx = i
            break
    if ros_idx is None:
        return result

    result['args_before_ros'] = cmd[1:ros_idx]

    i = ros_idx + 1
    while i < len(cmd):
        if cmd[i] == '-p' and i + 1 < len(cmd):
            pv = cmd[i + 1]
            if ':=' in pv:
                key, value = pv.split(':=', 1)
                result['params'][key] = value
            i += 2
        elif cmd[i] == '--params-file' and i + 1 < len(cmd):
            result['params_files'].append(cmd[i + 1])
            i += 2
        elif cmd[i] == '-r' and i + 1 < len(cmd):
            remap = cmd[i + 1]
            if ':=' in remap:
                key, value = remap.split(':=', 1)
                if key == '__node':
                    result['node_name'] = value
                elif key == '__ns':
                    result['namespace'] = value
                else:
                    result['remaps'].append((key, value))
            i += 2
        else:
            i += 1

    return result


def cmds_equivalent(rust_cmd: List[str], python_cmd: List[str]) -> List[str]:
    """Check if two cmd arrays are functionally equivalent.

    Returns list of difference descriptions (empty = equivalent).
    Tolerates: param ordering, param duplicates, params_file vs inline.
    """
    r = parse_cmd(rust_cmd)
    p = parse_cmd(python_cmd)
    diffs = []

    if r['executable'] != p['executable']:
        diffs.append(f"executable: {r['executable']} vs {p['executable']}")

    if sorted(r['args_before_ros']) != sorted(p['args_before_ros']):
        diffs.append(f"args_before_ros differ")

    if r['node_name'] != p['node_name']:
        diffs.append(f"node_name: {r['node_name']} vs {p['node_name']}")

    if r['namespace'] != p['namespace']:
        diffs.append(f"namespace: {r['namespace']} vs {p['namespace']}")

    if sorted(r['remaps']) != sorted(p['remaps']):
        diffs.append("remaps differ")

    # Compare params: normalize values and allow Python to have extras
    # from params_file expansion or duplication
    common = set(r['params']) & set(p['params'])
    for k in common:
        rv = normalize_param_value(r['params'][k])
        pv = normalize_param_value(p['params'][k])
        if rv != pv:
            diffs.append(f"param '{k}': {rv!r} vs {pv!r}")

    # Python-only keys are acceptable if Rust has --params-file to cover them
    py_only = set(p['params']) - set(r['params'])
    if py_only and not r['params_files']:
        diffs.append(f"python-only cmd params (no params_files): {sorted(py_only)}")

    # Rust-only keys that aren't artifacts
    rust_only = set(r['params']) - set(p['params'])
    real_rust_only = {k for k in rust_only if k != 'substitution'}
    if real_rust_only:
        diffs.append(f"rust-only cmd params: {sorted(real_rust_only)}")

    return diffs


# =============================================================================
# Node-level equivalence
# =============================================================================

def node_key(n: Dict) -> Tuple[str, str, str]:
    return (n.get('name') or '', n.get('namespace') or '', n.get('package') or '')


def nodes_equivalent(rust: Dict, python: Dict) -> List[str]:
    """Check if two node records are functionally equivalent.

    Returns list of difference descriptions (empty = equivalent).
    """
    diffs = []

    # --- params ---
    r_params = {k: normalize_param_value(v)
                for k, v in rust.get('params', [])}
    p_params = {k: normalize_param_value(v)
                for k, v in python.get('params', [])}

    # Expand Rust params_files into param set for comparison
    r_expanded = dict(r_params)
    for pf_content in rust.get('params_files', []):
        for ek, ev in extract_params_from_yaml_content(pf_content):
            if ek not in r_expanded:
                r_expanded[ek] = normalize_param_value(ev)

    # Value mismatches on common keys
    for k in set(r_expanded) & set(p_params):
        if r_expanded[k] != p_params[k]:
            diffs.append(f"param '{k}' value differs")

    # Python-only params not covered by Rust expanded
    py_only = set(p_params) - set(r_expanded)
    if py_only:
        diffs.append(f"params missing from Rust ({len(py_only)}): "
                     f"{sorted(py_only)[:5]}{'...' if len(py_only) > 5 else ''}")

    # Rust-only params (excluding known artifacts)
    rust_only = {k for k in r_expanded if k not in p_params and k != 'substitution'}
    # Don't flag Rust-only if they came from params_files (extra coverage is fine)
    rust_inline_only = rust_only & set(r_params)
    if rust_inline_only:
        diffs.append(f"rust-only params ({len(rust_inline_only)}): "
                     f"{sorted(rust_inline_only)[:5]}")

    # --- cmd ---
    cmd_diffs = cmds_equivalent(rust.get('cmd', []), python.get('cmd', []))
    diffs.extend(cmd_diffs)

    # --- simple fields ---
    for field in ('name', 'namespace', 'executable', 'exec_name',
                  'global_params', 'remaps', 'env', 'respawn', 'ros_args'):
        rv = rust.get(field)
        pv = python.get(field)
        if rv is None and pv == []:
            continue
        if rv == [] and pv is None:
            continue
        if rv != pv:
            diffs.append(f"field '{field}' differs")

    # --- args ---
    r_args = rust.get('args')
    p_args = python.get('args')
    if r_args != p_args:
        # None vs [''] both mean "no args"
        if not ((r_args is None and p_args == [''])
                or (r_args == [''] and p_args is None)):
            diffs.append(f"args differ: {r_args} vs {p_args}")

    return diffs


# =============================================================================
# Container / load_node equivalence
# =============================================================================

def containers_equivalent(rust_list: List[Dict], python_list: List[Dict]) -> List[str]:
    """Check container equivalence (order-insensitive)."""
    diffs = []

    r_sorted = sorted(rust_list, key=lambda c: (c.get('name', ''), c.get('namespace', '')))
    p_sorted = sorted(python_list, key=lambda c: (c.get('name', ''), c.get('namespace', '')))

    if len(r_sorted) != len(p_sorted):
        diffs.append(f"container count: Rust={len(r_sorted)}, Python={len(p_sorted)}")
        return diffs

    for rc, pc in zip(r_sorted, p_sorted):
        if rc.get('name') != pc.get('name') or rc.get('namespace') != pc.get('namespace'):
            diffs.append(f"container mismatch: {rc.get('name')} vs {pc.get('name')}")

    return diffs


def load_nodes_equivalent(rust_list: List[Dict], python_list: List[Dict]) -> List[str]:
    """Check load_node equivalence (order-insensitive)."""
    diffs = []

    def ln_key(ln: Dict) -> Tuple[str, str, str]:
        return (ln.get('node_name', ''), ln.get('target_container_name', ''),
                ln.get('plugin', ''))

    r_sorted = sorted(rust_list, key=ln_key)
    p_sorted = sorted(python_list, key=ln_key)

    if len(r_sorted) != len(p_sorted):
        diffs.append(f"load_node count: Rust={len(r_sorted)}, Python={len(p_sorted)}")
        return diffs

    for rl, pl in zip(r_sorted, p_sorted):
        if ln_key(rl) != ln_key(pl):
            diffs.append(f"load_node mismatch: {rl.get('node_name')} vs {pl.get('node_name')}")

    return diffs


# =============================================================================
# Top-level comparison
# =============================================================================

def compare_records(rust_path: str, python_path: str, verbose: bool = True) -> bool:
    """Compare two record.json files.

    Performs exact match first, then functional equivalence with normalization.
    Returns True if records are functionally equivalent.
    """
    try:
        with open(rust_path, 'r') as f:
            rust_record = json.load(f)
    except Exception as e:
        print(f"Error loading Rust record from {rust_path}: {e}")
        return False

    try:
        with open(python_path, 'r') as f:
            python_record = json.load(f)
    except Exception as e:
        print(f"Error loading Python record from {python_path}: {e}")
        return False

    # --- Entity counts ---
    if verbose:
        print("  Entity counts:")
    all_counts_match = True
    for key in ('node', 'container', 'load_node'):
        rc = len(rust_record.get(key, []))
        pc = len(python_record.get(key, []))
        marker = "ok" if rc == pc else "MISMATCH"
        if rc != pc:
            all_counts_match = False
        if verbose:
            print(f"    {key:12s}: Rust={rc:3d}  Python={pc:3d}  [{marker}]")

    # --- Match nodes by key ---
    rust_nodes = {}
    for n in rust_record.get('node', []):
        rust_nodes.setdefault(node_key(n), []).append(n)
    python_nodes = {}
    for n in python_record.get('node', []):
        python_nodes.setdefault(node_key(n), []).append(n)

    all_keys = sorted(set(rust_nodes) | set(python_nodes))
    common_keys = sorted(set(rust_nodes) & set(python_nodes))
    rust_only_keys = sorted(set(rust_nodes) - set(python_nodes))
    python_only_keys = sorted(set(python_nodes) - set(rust_nodes))

    # --- Exact match check ---
    exact_match = 0
    for key in common_keys:
        r = rust_nodes[key][0]
        p = python_nodes[key][0]
        if r == p:
            exact_match += 1

    # --- Functional equivalence check ---
    equiv_match = 0
    non_equiv: List[Tuple[Tuple[str, str, str], List[str]]] = []
    for key in common_keys:
        r = rust_nodes[key][0]
        p = python_nodes[key][0]
        diffs = nodes_equivalent(r, p)
        if not diffs:
            equiv_match += 1
        else:
            non_equiv.append((key, diffs))

    # --- Container / load_node equivalence ---
    container_diffs = containers_equivalent(
        rust_record.get('container', []), python_record.get('container', []))
    load_node_diffs = load_nodes_equivalent(
        rust_record.get('load_node', []), python_record.get('load_node', []))

    # --- Report ---
    if verbose:
        print()
        print(f"  Nodes ({len(common_keys)} matched):")
        print(f"    Exact match:          {exact_match}/{len(common_keys)}")
        print(f"    Functionally equiv:   {equiv_match}/{len(common_keys)}")
        non_equiv_count = len(common_keys) - equiv_match
        if non_equiv_count:
            print(f"    Non-equivalent:       {non_equiv_count}/{len(common_keys)}")

        if rust_only_keys:
            print(f"    Rust-only nodes:      {rust_only_keys}")
        if python_only_keys:
            print(f"    Python-only nodes:    {python_only_keys}")

        if not container_diffs:
            rc = len(rust_record.get('container', []))
            print(f"  Containers:             {rc}/{rc} equivalent")
        else:
            for d in container_diffs:
                print(f"  Containers:             {d}")

        if not load_node_diffs:
            rl = len(rust_record.get('load_node', []))
            print(f"  Load nodes:             {rl}/{rl} equivalent")
        else:
            for d in load_node_diffs:
                print(f"  Load nodes:             {d}")

        if non_equiv:
            print()
            print(f"  Non-equivalent nodes ({len(non_equiv)}):")
            for key, diffs in non_equiv:
                name = key[0] or 'None'
                ns = key[1] or 'None'
                pkg = key[2]
                print(f"    {name} ({ns}) [{pkg}]:")
                for d in diffs:
                    print(f"      - {d}")

    # Overall result
    success = (equiv_match == len(common_keys)
               and not rust_only_keys
               and not python_only_keys
               and not container_diffs
               and not load_node_diffs)

    if verbose:
        print()
        if success:
            print("  Result: PASS (all records functionally equivalent)")
        else:
            print(f"  Result: FAIL ({equiv_match}/{len(common_keys)} nodes equivalent)")

    return success


def main():
    if len(sys.argv) < 3:
        print("Usage: compare_records.py <rust_record.json> <python_record.json>")
        print()
        print("Compares two record.json files for functional equivalence.")
        print("Normalizes: whitespace, array quoting, params_files expansion,")
        print("cmd ordering/duplication, container ordering.")
        sys.exit(1)

    rust_path = sys.argv[1]
    python_path = sys.argv[2]

    if not Path(rust_path).exists():
        print(f"Error: {rust_path} not found")
        sys.exit(1)
    if not Path(python_path).exists():
        print(f"Error: {python_path} not found")
        sys.exit(1)

    success = compare_records(rust_path, python_path, verbose=True)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
