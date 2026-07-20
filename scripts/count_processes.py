#!/usr/bin/env python3
"""
Count expected vs actual processes for a play_launch session.

Reads expected process count from a `play_launch resolve`/`dump` SystemModel
YAML (plain nodes + containers in `structure.nodes` — composable nodes are
virtual members of a container process, not counted separately), and actual
running process count from play_log pid files.

Phase 47.B5 — record.json is retired as a dump artifact; a `.json` path is
still accepted read-only for old record.json files that may still be lying
around (same node+container count, different shape), but nothing in this
repo produces one anymore.

Usage:
    count_processes.py <system_model.yaml|record.json> [--play-log-dir <dir>]

Exit codes:
    0: counts match
    1: counts mismatch or error
"""

import argparse
import json
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print("Error: PyYAML is required (pip install pyyaml)", file=sys.stderr)
    sys.exit(2)


def count_expected(input_path: Path) -> int:
    """Count expected processes (plain nodes + containers) from a SystemModel
    YAML, or — for old files — a record.json."""
    if input_path.suffix in (".yaml", ".yml"):
        with open(input_path) as f:
            model = yaml.safe_load(f)
        nodes = (model or {}).get("structure", {}).get("nodes", {}) or {}
        containers = sum(1 for inst in nodes.values() if inst.get("is_container"))
        composables = sum(
            1
            for inst in nodes.values()
            if not inst.get("is_container") and inst.get("plugin")
        )
        plain = len(nodes) - containers - composables
        return plain + containers

    # Legacy record.json shape.
    with open(input_path) as f:
        record = json.load(f)

    nodes = len(record.get("node", []))
    containers = len(record.get("container", []))
    return nodes + containers


def count_actual(play_log_dir: Path) -> int:
    """Count actual spawned processes from play_log_dir/node/*/cmdline files.

    Uses cmdline (not pid) because containers don't write pid files
    but do get a cmdline file when spawned.
    """
    node_dir = play_log_dir / "node"
    if not node_dir.is_dir():
        return 0

    count = 0
    for cmdline_file in node_dir.glob("*/cmdline"):
        if cmdline_file.stat().st_size > 0:
            count += 1
    return count


def main():
    parser = argparse.ArgumentParser(description="Count expected vs actual processes")
    parser.add_argument(
        "model", type=Path, help="Path to system_model.yaml (or a legacy record.json)"
    )
    parser.add_argument(
        "--play-log-dir",
        type=Path,
        default=None,
        help="Path to play_log directory (default: model's parent / play_log/latest)",
    )
    args = parser.parse_args()

    if not args.model.is_file():
        print(f"ERROR: input file not found: {args.model}", file=sys.stderr)
        return 1

    expected = count_expected(args.model)
    print(f"Expected processes: {expected}")

    play_log_dir = args.play_log_dir
    if play_log_dir is None:
        play_log_dir = args.model.parent / "play_log" / "latest"

    if not play_log_dir.is_dir():
        print(f"ERROR: play_log directory not found: {play_log_dir}", file=sys.stderr)
        return 1

    actual = count_actual(play_log_dir)
    print(f"Actual processes:   {actual}")

    if actual == expected:
        print(f"PASS: {actual}/{expected} processes running")
        return 0
    else:
        print(f"FAIL: {actual}/{expected} processes running")
        return 1


if __name__ == "__main__":
    sys.exit(main())
