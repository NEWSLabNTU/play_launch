#!/usr/bin/env python3
"""
Count expected vs actual processes for a play_launch session.

Reads expected process count from record.json (nodes + containers),
and actual running process count from play_log pid files.

Usage:
    count_processes.py <record.json> [--play-log-dir <dir>]

Exit codes:
    0: counts match
    1: counts mismatch or error
"""

import argparse
import json
import sys
from pathlib import Path


def count_expected(record_path: Path) -> int:
    """Count expected processes from record.json (nodes + containers)."""
    with open(record_path) as f:
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
    parser.add_argument("record", type=Path, help="Path to record.json")
    parser.add_argument(
        "--play-log-dir",
        type=Path,
        default=None,
        help="Path to play_log directory (default: record.json's parent / play_log/latest)",
    )
    args = parser.parse_args()

    if not args.record.is_file():
        print(f"ERROR: record.json not found: {args.record}", file=sys.stderr)
        return 1

    expected = count_expected(args.record)
    print(f"Expected processes: {expected}")

    play_log_dir = args.play_log_dir
    if play_log_dir is None:
        play_log_dir = args.record.parent / "play_log" / "latest"

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
