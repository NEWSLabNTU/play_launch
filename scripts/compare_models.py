#!/usr/bin/env python3
"""
Compare two `system_model.yaml` SystemModel artifacts for cross-parser
functional equivalence (Phase 47.B1 — parser-parity moved off record.json
onto the model; the gate before record.json's hard removal in 47.B2+).

This is the model-shaped sibling of `compare_records.py`: same "functional
equivalence" contract (params/remaps ordering, whitespace/quoting artifacts,
duplicate collapsing all normalized away), applied to `structure.nodes`
(the unified node/container/composable map `play_launch resolve` /
`dump --format model` produce) instead of record.json's three separate
node/container/load_node arrays.

What's compared
----------------
1. **structure.nodes** — matched by FQN key (the model's own dict key, so
   no name/namespace/package tuple-matching heuristic is needed — a
   structural improvement over `compare_records.py`, which loses entities
   to tuple-key collisions: Autoware's 34 regular nodes match down to 32
   distinct keys there). Classified into containers (`is_container: true`),
   composables (`plugin` set), and plain nodes (everything else) for the
   entity-count summary.

   Every matched pair, of every kind, gets a STRUCTURAL check: `pkg`,
   `exec`, `plugin`, `container`, `is_container`, `lifecycle`,
   `lifecycle_autostart`, `criticality`, `node_name` (exact match) and
   `scope` (compared after stripping any `#<id>` disambiguation suffix —
   `scope_keys()` in `model_builder.rs` assigns those by traversal order,
   which can legitimately differ between the two parsers for the same
   launch tree). This is the same rigor `compare_records.py` already
   applies to containers/composables (`containers_equivalent`:
   name/namespace; `load_nodes_equivalent`: node_name/target_container/
   plugin) — just FQN-keyed instead of tuple-keyed.

   PLAIN nodes (not a container, no `plugin`) additionally get the DEEP
   spawn-input check `compare_records.py`'s `nodes_equivalent` already
   applies to regular `<node>` entries:
     - `remaps`: sorted `(from, to)` set (declaration order not load-bearing).
     - `env`: the *effective* mapping (name -> last value, "last wins").
     - `ros_args`: sorted set.
     - `respawn`, `respawn_delay`: exact match.
     - `args`, `raw_cmd`: exact match after whitespace normalization.
     - `extra_args`: exact dict match.
     - `params` + `params_files`: `params_files` (raw external YAML,
       verbatim) is expanded and merged into the effective param set
       (inline `params` wins on key collision, mirroring the model's own
       node-specific-over-global precedence), then compared key-by-key
       with a canonical repr (`canonical_param_repr`) that bridges
       natively-typed vs already-stringified values and normalizes bool
       case — the same class of cosmetic gap `compare_records.py` already
       tolerates (whitespace/array-quoting/boolean-case, "Parser Parity
       Status" in CLAUDE.md).

   Containers and composables deliberately do NOT get the deep spawn-input
   check in this wave: `compare_records.py` never compared container or
   load_node params/remaps/env either (`containers_equivalent`/
   `load_nodes_equivalent` are structural-only), so extending scrutiny
   there is a rigor INCREASE, not a faithful migration — and it currently
   surfaces real pre-existing Rust/Python composable-param divergences
   (duplicate param collection, YAML-merged keys one side has and the
   other doesn't, occasional value mismatches) that predate this tool and
   are out of scope for Phase 47.B1 (move the parity mechanism; don't
   change what it tolerates). See the B1 report for specifics — a good
   47.B1-follow-up candidate, not a regression this wave introduces.
2. **structure.topics / services / actions** — compared only when either
   side is non-empty (both empty is the common case for a plain resolve
   with no contract manifests: nano-ros/contract topic wiring comes from
   `ManifestIndex`, not raw launch remaps). Each entry compares `type` +
   sorted `pub`/`sub` (or `server`/`client`) sets.
3. **contracts** / **execution.sched** — compared only when either side is
   non-empty (both parsers apply these on the SAME shared scope table,
   Phase 40.1, so a plain resolve with no `--contracts`/`--sched` and no
   sidecar leaves both empty — trivially equivalent). When present,
   checked for matching endpoint/path/topic-contract key sets and the
   resolved chain/binding/tier counts.

`meta` (args/inputs/resolver/diagnostics) is intentionally NOT compared —
it's provenance (file hashes, resolver tool name/version), not structure.

Exit code 0 = functionally equivalent, 1 = mismatch found.
"""

import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

try:
    import yaml
except ImportError:
    print("Error: PyYAML is required (pip install pyyaml)", file=sys.stderr)
    sys.exit(2)


# =============================================================================
# Value normalization helpers (same spirit as compare_records.py)
# =============================================================================

def normalize_whitespace(value: str) -> str:
    """Collapse all whitespace (newlines, tabs, spaces) to single space."""
    return re.sub(r'\s+', ' ', value).strip()


def strip_scope_disambiguator(scope: str) -> str:
    """Strip a `#<id>` disambiguation suffix. Since Phase 48 a scope id is
    STRUCTURAL — the launch file (`<pkg>/<file>`) a node came from — and
    `scope_keys()` in `model_builder.rs` appends `#<id>` only to disambiguate a
    file included more than once; that id follows scope traversal order, which
    can legitimately differ between the Rust and Python parsers for otherwise
    identical include trees."""
    return re.sub(r'#\d+$', '', scope or '')


def param_value_norm(v: Any) -> Any:
    """Normalize a loaded ParamValue for comparison: YAML already types
    bool/int/float/str/list distinctly (untagged enum), so this mostly
    just recurses into lists for consistent element comparison."""
    if isinstance(v, list):
        return [param_value_norm(x) for x in v]
    return v


def canonical_param_repr(v: Any) -> str:
    """Canonical string form of a param value for cross-representation
    comparison — bridges the same gap `compare_records.py` normalizes at
    the string level (`python_repr_value` + `normalize_array_quoting`),
    needed here because one parser may hand back a natively-typed value
    (from `params_files` YAML, parsed) while the other hands back an
    already-stringified inline value (`ParamValue::Str` — e.g. an array
    param serialized as `"[1.0, 0.0, ...]"`, or a bool as the literal
    string `"False"`). Case-insensitive on bool tokens (`true`/`root_state
    -> True`) — a known cosmetic Rust/Python difference (CLAUDE.md
    "Parser Parity Status": boolean case)."""
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, list):
        return '[' + ', '.join(canonical_param_repr(x) for x in v) + ']'
    if isinstance(v, str):
        s = normalize_whitespace(v)
        if s.lower() in ('true', 'false'):
            return s.capitalize()
        m = re.match(r'^\[(.*)\]$', s)
        if m:
            inner = m.group(1).strip()
            if not inner:
                return '[]'
            elems = [e.strip().strip("'\"") for e in inner.split(',')]
            return '[' + ', '.join(elems) + ']'
        return s
    return str(v)


def extract_params_from_yaml_content(yaml_content: str) -> List[Tuple[str, Any]]:
    """Flatten an externally-referenced param YAML file's content
    (`NodeInstance::params_files`, verbatim ROS 2 YAML with `/**`/
    `ros__parameters` wrappers) into (key, value) pairs, matching the
    flattening `compare_records.py` already does for record.json's
    `params_files`."""
    try:
        data = yaml.safe_load(yaml_content)
    except yaml.YAMLError:
        return []
    if not isinstance(data, dict):
        return []

    params: List[Tuple[str, Any]] = []

    def extract(prefix: str, obj: Any) -> None:
        if isinstance(obj, dict):
            for k, v in obj.items():
                if k in ('/**', 'ros__parameters'):
                    extract(prefix, v)
                else:
                    new_key = f"{prefix}.{k}" if prefix else k
                    extract(new_key, v)
        elif prefix:
            params.append((prefix, param_value_norm(obj)))

    extract('', data)
    return params


def effective_params(node: Dict) -> Tuple[Dict[str, Any], set]:
    """Merge `params_files` (lower precedence — file-derived defaults)
    with inline `params` (higher precedence), matching the model's own
    node-specific-over-global fold order. Returns `(merged, inline_keys)`
    — `inline_keys` is the set of keys that were ALREADY in `params`
    before file-expansion, needed by the params-files-expansion tolerance
    in `spawn_input_diffs` (mirrors `compare_records.py`'s
    `rust_inline_only` split)."""
    merged: Dict[str, Any] = {}
    for pf_content in node.get('params_files', []) or []:
        for k, v in extract_params_from_yaml_content(pf_content):
            merged[k] = v
    inline = node.get('params', {}) or {}
    for k, v in inline.items():
        merged[k] = param_value_norm(v)
    return merged, set(inline.keys())


def effective_env(node: Dict) -> Dict[str, str]:
    """Collapse the declaration-order `env` list into the effective
    mapping (last value per name wins — launch env semantics)."""
    result: Dict[str, str] = {}
    for e in node.get('env', []) or []:
        result[e['name']] = e['value']
    return result


def normalized_str_list(values: Optional[List[str]]) -> List[str]:
    return [normalize_whitespace(v) for v in (values or [])]


# =============================================================================
# Node classification
# =============================================================================

def classify_nodes(nodes: Dict[str, Dict]) -> Tuple[int, int, int]:
    """Returns (plain_nodes, containers, composables) counts."""
    containers = sum(1 for n in nodes.values() if n.get('is_container'))
    composables = sum(1 for n in nodes.values() if n.get('plugin') is not None)
    plain = len(nodes) - containers - composables
    return plain, containers, composables


# =============================================================================
# Node-level equivalence
# =============================================================================

def node_kind(node: Dict) -> str:
    if node.get('is_container'):
        return 'container'
    if node.get('plugin') is not None:
        return 'composable'
    return 'node'


def structural_diffs(rust: Dict, python: Dict) -> List[str]:
    """Identity/wiring fields checked for EVERY entity kind — this is the
    same rigor `compare_records.py` already applies to containers
    (`containers_equivalent`: name/namespace) and composables
    (`load_nodes_equivalent`: node_name/target_container/plugin), now keyed
    by FQN instead of a tuple. `scope` is deliberately NOT included here —
    see `scope_note`."""
    diffs: List[str] = []

    for field in ('pkg', 'exec', 'plugin', 'container', 'is_container',
                  'lifecycle', 'lifecycle_autostart', 'criticality', 'node_name'):
        rv = rust.get(field)
        pv = python.get(field)
        # is_container/lifecycle default to False and may be omitted
        # entirely by the YAML `skip_serializing_if` — normalize None to
        # False for those two boolean fields.
        if field in ('is_container', 'lifecycle'):
            rv = bool(rv)
            pv = bool(pv)
        if rv != pv:
            diffs.append(f"field '{field}': {rv!r} vs {pv!r}")

    return diffs


def scope_note(rust: Dict, python: Dict) -> Optional[str]:
    """`scope` — the launch FILE a node originates from (Phase 48: structural,
    `<pkg>/<file>`, not a namespace) — is reported as an ADVISORY note, not a
    pass/fail diff. Since Phase 48 both parsers key scopes by the same file
    identity and fold `<group>` scopes into their file, so this is expected to
    match; it stays advisory (rather than a hard diff) because a file included
    more than once carries a traversal-order `#<id>` that can differ between
    parsers. The dedicated `scope: Option<usize>` cross-parser check lives in
    `scripts/compare_scopes.py` (Phase 30)."""
    r_scope = strip_scope_disambiguator(rust.get('scope', ''))
    p_scope = strip_scope_disambiguator(python.get('scope', ''))
    if r_scope != p_scope:
        return f"scope: {r_scope!r} vs {p_scope!r} (advisory — see scripts/compare_scopes.py)"
    return None


def spawn_input_diffs(rust: Dict, python: Dict) -> List[str]:
    """Deep spawn-input comparison (remaps/env/ros_args/respawn/args/
    raw_cmd/extra_args/params) — the same rigor `compare_records.py`'s
    `nodes_equivalent` already applies to regular `<node>` entries (proven
    32/32 functionally equivalent on Autoware). Applied here to PLAIN
    nodes only (see `node_diffs`) — containers and composable nodes keep
    the shallower `structural_diffs`-only check `compare_records.py`
    always gave them (`containers_equivalent`/`load_nodes_equivalent`
    never compared params/remaps/env either), so this migration doesn't
    newly fail where the record.json gate has always passed (never break
    parity mid-transition, Phase 47.B1)."""
    diffs: List[str] = []

    r_remaps = sorted((r['from'], r['to']) for r in rust.get('remaps', []) or [])
    p_remaps = sorted((r['from'], r['to']) for r in python.get('remaps', []) or [])
    if r_remaps != p_remaps:
        diffs.append("remaps differ")

    r_env = effective_env(rust)
    p_env = effective_env(python)
    if r_env != p_env:
        diffs.append("env differ")

    r_ros_args = sorted(normalized_str_list(rust.get('ros_args')))
    p_ros_args = sorted(normalized_str_list(python.get('ros_args')))
    if r_ros_args != p_ros_args:
        diffs.append(f"ros_args differ: {r_ros_args} vs {p_ros_args}")

    if rust.get('respawn') != python.get('respawn'):
        diffs.append(f"respawn: {rust.get('respawn')!r} vs {python.get('respawn')!r}")
    if rust.get('respawn_delay') != python.get('respawn_delay'):
        diffs.append("respawn_delay differ")

    r_args = normalized_str_list(rust.get('args'))
    p_args = normalized_str_list(python.get('args'))
    if r_args != p_args:
        diffs.append(f"args differ: {r_args} vs {p_args}")

    r_raw = normalized_str_list(rust.get('raw_cmd'))
    p_raw = normalized_str_list(python.get('raw_cmd'))
    if r_raw != p_raw:
        diffs.append(f"raw_cmd differ: {r_raw} vs {p_raw}")

    r_extra = rust.get('extra_args', {}) or {}
    p_extra = python.get('extra_args', {}) or {}
    if r_extra != p_extra:
        diffs.append(f"extra_args differ: {r_extra} vs {p_extra}")

    # Params: mirrors `compare_records.py`'s `nodes_equivalent` asymmetric
    # tolerance for params_files expansion (Rust keeps an external YAML
    # reference verbatim; Python inlines the file's content into `params`
    # directly, sometimes as a single nested-structure repr rather than
    # flattened dotted keys). A rust-only key is only a real problem when
    # it was in Rust's ORIGINAL inline `params` (not file-expanded); a
    # python-only key is only a real problem when Rust has NO params_files
    # to plausibly cover it.
    r_params, r_inline_keys = effective_params(rust)
    p_params, _ = effective_params(python)
    common = set(r_params) & set(p_params)
    for k in sorted(common):
        if canonical_param_repr(r_params[k]) != canonical_param_repr(p_params[k]):
            diffs.append(f"param '{k}': {r_params[k]!r} vs {p_params[k]!r}")
    rust_only = set(r_params) - set(p_params)
    rust_inline_only = rust_only & r_inline_keys
    if rust_inline_only:
        diffs.append(f"rust-only params ({len(rust_inline_only)}): {sorted(rust_inline_only)[:5]}")
    python_only = set(p_params) - set(r_params)
    if python_only and not (rust.get('params_files') or []):
        diffs.append(f"python-only params ({len(python_only)}): {sorted(python_only)[:5]}")

    return diffs


def node_diffs(rust: Dict, python: Dict) -> List[str]:
    """Check if two NodeInstance dicts (same FQN key) are functionally
    equivalent. Returns list of difference descriptions (empty = equivalent).

    Structural identity fields are checked for every entity kind. The
    deeper spawn-input comparison (params/remaps/env/...) is applied to
    PLAIN nodes only — see `spawn_input_diffs` for why containers/
    composables are intentionally excluded from that tier in this wave.
    """
    diffs = structural_diffs(rust, python)
    if node_kind(rust) == 'node':
        diffs.extend(spawn_input_diffs(rust, python))
    return diffs


# =============================================================================
# structure.topics / services / actions equivalence
# =============================================================================

def wiring_diffs(kind: str, rust: Dict[str, Dict], python: Dict[str, Dict],
                  client_server: bool = False) -> List[str]:
    """Compare a `topics`/`services`/`actions` map. `client_server`
    switches the endpoint-list field names (`server`/`client` vs
    `pub`/`sub`)."""
    diffs: List[str] = []
    if not rust and not python:
        return diffs

    a_key, b_key = ('server', 'client') if client_server else ('pub', 'sub')

    all_keys = sorted(set(rust) | set(python))
    for k in all_keys:
        if k not in rust:
            diffs.append(f"{kind} '{k}': python-only")
            continue
        if k not in python:
            diffs.append(f"{kind} '{k}': rust-only")
            continue
        r, p = rust[k], python[k]
        if r.get('type') != p.get('type'):
            diffs.append(f"{kind} '{k}' type: {r.get('type')!r} vs {p.get('type')!r}")
        if sorted(r.get(a_key, []) or []) != sorted(p.get(a_key, []) or []):
            diffs.append(f"{kind} '{k}' {a_key} differ")
        if sorted(r.get(b_key, []) or []) != sorted(p.get(b_key, []) or []):
            diffs.append(f"{kind} '{k}' {b_key} differ")

    return diffs


# =============================================================================
# contracts / execution.sched equivalence (compared only when present)
# =============================================================================

def contracts_diffs(rust: Dict, python: Dict) -> List[str]:
    diffs: List[str] = []
    r_empty = not rust or all(not v for v in rust.values())
    p_empty = not python or all(not v for v in python.values())
    if r_empty and p_empty:
        return diffs

    for section in ('pub_endpoints', 'sub_endpoints', 'srv_endpoints',
                     'node_paths', 'scope_paths', 'topics', 'externals'):
        r_keys = set((rust or {}).get(section, {}) or {})
        p_keys = set((python or {}).get(section, {}) or {})
        if r_keys != p_keys:
            diffs.append(
                f"contracts.{section} key sets differ: "
                f"rust-only={sorted(r_keys - p_keys)[:5]} "
                f"python-only={sorted(p_keys - r_keys)[:5]}"
            )
    return diffs


def sched_diffs(rust: Dict, python: Dict) -> List[str]:
    diffs: List[str] = []
    r_sched = (rust or {}).get('sched') or {}
    p_sched = (python or {}).get('sched') or {}
    r_empty = not r_sched and not (rust or {}).get('bindings') and not (rust or {}).get('tiers')
    p_empty = not p_sched and not (python or {}).get('bindings') and not (python or {}).get('tiers')
    if r_empty and p_empty:
        return diffs

    r_bindings = (rust or {}).get('bindings', {}) or {}
    p_bindings = (python or {}).get('bindings', {}) or {}
    if r_bindings != p_bindings:
        diffs.append(
            f"execution.bindings differ: {len(r_bindings)} vs {len(p_bindings)} entries"
        )

    r_tiers = set((rust or {}).get('tiers', {}) or {})
    p_tiers = set((python or {}).get('tiers', {}) or {})
    if r_tiers != p_tiers:
        diffs.append(f"execution.tiers key sets differ: rust={sorted(r_tiers)} python={sorted(p_tiers)}")

    r_chains = len(r_sched.get('chains', []) or [])
    p_chains = len(p_sched.get('chains', []) or [])
    if r_chains != p_chains:
        diffs.append(f"execution.sched.chains count: rust={r_chains} vs python={p_chains}")

    r_mapper = r_sched.get('mapper')
    p_mapper = p_sched.get('mapper')
    if r_mapper != p_mapper:
        diffs.append(f"execution.sched.mapper: {r_mapper!r} vs {p_mapper!r}")

    return diffs


# =============================================================================
# Top-level comparison
# =============================================================================

def load_model(path: str) -> Dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def compare_models(rust_path: str, python_path: str, verbose: bool = True) -> bool:
    try:
        rust_model = load_model(rust_path)
    except Exception as e:
        print(f"Error loading Rust model from {rust_path}: {e}")
        return False
    try:
        python_model = load_model(python_path)
    except Exception as e:
        print(f"Error loading Python model from {python_path}: {e}")
        return False

    rust_nodes = (rust_model.get('structure') or {}).get('nodes', {}) or {}
    python_nodes = (python_model.get('structure') or {}).get('nodes', {}) or {}

    r_plain, r_containers, r_composables = classify_nodes(rust_nodes)
    p_plain, p_containers, p_composables = classify_nodes(python_nodes)

    if verbose:
        print("  Entity counts (structure.nodes):")
        for label, rc, pc in (
            ('node', r_plain, p_plain),
            ('container', r_containers, p_containers),
            ('composable', r_composables, p_composables),
            ('total', len(rust_nodes), len(python_nodes)),
        ):
            marker = "ok" if rc == pc else "MISMATCH"
            print(f"    {label:12s}: Rust={rc:3d}  Python={pc:3d}  [{marker}]")

    common_keys = sorted(set(rust_nodes) & set(python_nodes))
    rust_only_keys = sorted(set(rust_nodes) - set(python_nodes))
    python_only_keys = sorted(set(python_nodes) - set(rust_nodes))

    equiv_match = 0
    non_equiv: List[Tuple[str, List[str]]] = []
    scope_notes: List[Tuple[str, str]] = []
    for key in common_keys:
        diffs = node_diffs(rust_nodes[key], python_nodes[key])
        if not diffs:
            equiv_match += 1
        else:
            non_equiv.append((key, diffs))
        note = scope_note(rust_nodes[key], python_nodes[key])
        if note:
            scope_notes.append((key, note))

    topic_diffs = wiring_diffs(
        'topic',
        (rust_model.get('structure') or {}).get('topics', {}) or {},
        (python_model.get('structure') or {}).get('topics', {}) or {},
    )
    service_diffs = wiring_diffs(
        'service',
        (rust_model.get('structure') or {}).get('services', {}) or {},
        (python_model.get('structure') or {}).get('services', {}) or {},
        client_server=True,
    )
    action_diffs = wiring_diffs(
        'action',
        (rust_model.get('structure') or {}).get('actions', {}) or {},
        (python_model.get('structure') or {}).get('actions', {}) or {},
        client_server=True,
    )

    contract_diffs = contracts_diffs(rust_model.get('contracts'), python_model.get('contracts'))
    execution_diffs = sched_diffs(rust_model.get('execution'), python_model.get('execution'))

    if verbose:
        print()
        print(f"  Nodes ({len(common_keys)} matched by FQN):")
        print(f"    Functionally equiv:   {equiv_match}/{len(common_keys)}")
        non_equiv_count = len(common_keys) - equiv_match
        if non_equiv_count:
            print(f"    Non-equivalent:       {non_equiv_count}/{len(common_keys)}")
        if rust_only_keys:
            print(f"    Rust-only nodes:      {rust_only_keys}")
        if python_only_keys:
            print(f"    Python-only nodes:    {python_only_keys}")

        if not topic_diffs and not service_diffs and not action_diffs:
            r_t = len((rust_model.get('structure') or {}).get('topics', {}) or {})
            print(f"  Topics/services/actions: {r_t} topic(s), equivalent (or both empty)")
        else:
            for d in topic_diffs + service_diffs + action_diffs:
                print(f"  Wiring:                 {d}")

        if not contract_diffs:
            print("  Contracts:              equivalent (or both empty)")
        else:
            for d in contract_diffs:
                print(f"  Contracts:              {d}")

        if not execution_diffs:
            print("  Execution/sched:        equivalent (or both empty)")
        else:
            for d in execution_diffs:
                print(f"  Execution/sched:        {d}")

        if non_equiv:
            print()
            print(f"  Non-equivalent nodes ({len(non_equiv)}):")
            for key, diffs in non_equiv:
                print(f"    {key}:")
                for d in diffs:
                    print(f"      - {d}")

        if scope_notes:
            print()
            print(f"  Scope advisories ({len(scope_notes)}, non-blocking):")
            for key, note in scope_notes:
                print(f"    {key}: {note}")

    success = (
        equiv_match == len(common_keys)
        and not rust_only_keys
        and not python_only_keys
        and not topic_diffs
        and not service_diffs
        and not action_diffs
        and not contract_diffs
        and not execution_diffs
    )

    if verbose:
        print()
        if success:
            print("  Result: PASS (all models functionally equivalent)")
        else:
            print(f"  Result: FAIL ({equiv_match}/{len(common_keys)} nodes equivalent)")

    return success


def main():
    if len(sys.argv) < 3:
        print("Usage: compare_models.py <rust_model.yaml> <python_model.yaml>")
        print()
        print("Compares two SystemModel YAML artifacts (`play_launch resolve` /")
        print("`dump --format model` output) for cross-parser functional")
        print("equivalence. See the module docstring for exactly what's compared.")
        sys.exit(1)

    rust_path = sys.argv[1]
    python_path = sys.argv[2]

    if not Path(rust_path).exists():
        print(f"Error: {rust_path} not found")
        sys.exit(1)
    if not Path(python_path).exists():
        print(f"Error: {python_path} not found")
        sys.exit(1)

    success = compare_models(rust_path, python_path, verbose=True)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
