#!/usr/bin/env python3
"""Batch oracle for attr_differential.rs.

Reads a JSON manifest `{tag: fixture_path}` and feeds every fixture to real
ROS 2's XML frontend inside ONE Python process, instead of `attr_differential
.rs` spawning `bash -c 'source setup.bash; python3 -c "..."'` once per
(element, attribute) probe. The per-subprocess cost (bash + sourcing
setup.bash + starting a fresh interpreter + importing `launch.frontend`) is
what made the differential test take ~145s for ~450 probes; importing
`launch.frontend` once and reusing the process for every probe removes that
overhead almost entirely.

Writes a JSON results map `{tag: {"kind": "SUCCESS"|"REJECT"|"OTHER_ERROR",
"message": str}}` to the given output path. `message` is the exception text
(empty on SUCCESS) — kept so the Rust side can pin an EXACT rejection reason
for fixtures that are known-unprobeable at the attribute level (see
`STRUCTURAL_BASELINE_EXCEPTIONS` in attr_differential.rs), not just a
loose OK/REJECT verdict.
"""

import json
import sys

from launch.frontend import Parser


def probe(path):
    try:
        root, parser = Parser.load(path)
        parser.parse_description(root)
        return "SUCCESS", ""
    except Exception as exc:
        message = str(exc)
        kind = "REJECT" if "Unexpected attribute" in message else "OTHER_ERROR"
        return kind, message


def main():
    manifest_path, results_path = sys.argv[1], sys.argv[2]
    with open(manifest_path) as fh:
        manifest = json.load(fh)

    results = {}
    for tag, path in manifest.items():
        kind, message = probe(path)
        results[tag] = {"kind": kind, "message": message}

    with open(results_path, "w") as fh:
        json.dump(results, fh)


if __name__ == "__main__":
    main()
