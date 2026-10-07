#!/usr/bin/env python3
"""Phase 78 / issue #0056: `min_rate_hz` is read in exactly ONE scheduling path.

Phase 78's ruling is "one derivation, two consumers": a `MapperNode.rate_hz` is
the fastest TIMER trigger the node declares, and nothing else.  A publisher's
`min_rate_hz` is a PROMISE -- a floor the node undertakes to meet -- and a
promise is not a period, so ranking a node by one schedules it as if it had a
timer it does not have.  The phase deleted every such read and recorded the fact
as an acceptance gate:

    - No `min_rate_hz` read remains in any scheduling path of this repository
      (`git grep min_rate_hz src/ros-launch-resolve/resolve/src/ros/sched_*`
      returns nothing).

Nothing ran that grep.  It was a sentence in a roadmap document, and the
roadmap is deliberately outside `just check-rt-docs`'s scope -- that gate reads
what a USER reads, and a phase document is a record of what was decided then.

The claim is now FALSE, and not by drift: issue #0056 put one read back on
purpose.  Phase 78 had widened the same narrowing over `rate_priority_
contradictions`, the rule whose whole job is to notice an author who writes
`min_rate_hz: 100` on one node and `10` on another and then expects the first to
outrank the second.  With the declared rates gone the rule had nothing to
compare and went silent -- a check disabled by a fix to a different problem,
which is this repository's most frequently relearned shape.  So
`input_with_declared_rates` builds a WIDENED COPY, handed only to that rule,
and `MapperNode.rate_hz` stays narrow.

This script is what the roadmap promised, with the exception named rather than
the claim quietly weakened.  It fails when a SECOND read appears, which is the
regression that matters: one read with a reason is a decision, two is the phase
being undone one call site at a time.

Test-module occurrences are not reads.  `sched_derive.rs` builds manifests
carrying `min_rate_hz: Some(30.0)` as the INPUT that pins phase 78's own
guarantee -- the test asserting a promise is never ranked needs a promise to
offer.  Counting those would make the gate fail on the evidence for the rule it
enforces.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

FIELD = "min_rate_hz"

SCHED_DIR = Path("src/ros-launch-resolve/resolve/src/ros")

# file -> {function name: why this read is legitimate}
#
# Keyed by FUNCTION, not by line: a line number rots on the next edit above it,
# and a gate that fails for being out of date gets deleted.
ALLOWED: dict[str, dict[str, str]] = {
    "sched_loader.rs": {
        "declared_rate_facts": (
            "issue #0056 -- collects the AUTHORED rates (`topics.<t>.rate_hz` and "
            "`pub.<ep>.min_rate_hz`) for `input_with_declared_rates`, its only "
            "caller, which builds a widened COPY of the mapper input and hands it "
            "to `rate_priority_contradictions` alone. `MapperNode.rate_hz` stays "
            "the fastest timer trigger, so the promise never reaches the ranking; "
            "this copy exists so the contradiction scan can compare the authored "
            "promises it is entirely about."
        ),
    },
    "sched_derive.rs": {},
}


def strip_comment(line: str) -> str:
    """Drop a `//`/`///` comment, and a line that is entirely one."""
    stripped = line.lstrip()
    if stripped.startswith("//") or stripped.startswith("*"):
        return ""
    # An inline trailing comment. Crude on purpose: `min_rate_hz` never appears
    # inside a string literal in this tree, so there is no `//`-in-a-string case
    # to get wrong, and erring toward KEEPING code is the safe direction.
    return line.split("//", 1)[0]


def enclosing_fn(lines: list[str], idx: int) -> str:
    """The nearest `fn name` at or above `idx`."""
    pattern = re.compile(r"\bfn\s+([A-Za-z_][A-Za-z0-9_]*)")
    for i in range(idx, -1, -1):
        m = pattern.search(lines[i])
        if m:
            return m.group(1)
    return "<file scope>"


def test_module_start(lines: list[str]) -> int:
    """Line index of `#[cfg(test)]`, or len(lines) when there is none."""
    for i, line in enumerate(lines):
        if line.strip().startswith("#[cfg(test)]"):
            return i
    return len(lines)


def main() -> int:
    if not SCHED_DIR.is_dir():
        print(f"FAIL: {SCHED_DIR} not found -- run from the repository root.")
        return 1

    failures: list[str] = []
    checked = 0
    reads: list[tuple[str, int, str]] = []

    for path in sorted(SCHED_DIR.glob("sched_*.rs")):
        name = path.name
        if name not in ALLOWED:
            failures.append(
                f"{name}: a new `sched_*` file this gate has never seen. Add it to "
                f"ALLOWED (with an empty dict if it reads no rates), so the phase 78 "
                f"invariant is stated for it rather than skipped."
            )
            continue

        lines = path.read_text().splitlines()
        cutoff = test_module_start(lines)
        allowed = ALLOWED[name]
        checked += 1

        for i, line in enumerate(lines[:cutoff]):
            if FIELD not in strip_comment(line):
                continue
            fn = enclosing_fn(lines, i)
            reads.append((name, i + 1, fn))
            if fn not in allowed:
                failures.append(
                    f"{name}:{i + 1}: `{FIELD}` is read in `{fn}`, which phase 78 "
                    f"deleted and nothing has licensed since.\n"
                    f"      {line.strip()}\n"
                    f"    A promise is not a period: `MapperNode.rate_hz` is the "
                    f"fastest TIMER trigger a node declares. Ranking on a declared "
                    f"floor schedules a node as if it had a timer it does not have.\n"
                    f"    If this read IS right, add `{fn}` to ALLOWED in this script "
                    f"with the reason -- the point is that the exception is written "
                    f"down, not that it is forbidden."
                )

        for fn, why in allowed.items():
            if not any(r[0] == name and r[2] == fn for r in reads):
                failures.append(
                    f"{name}: ALLOWED lists `{fn}` as a legitimate `{FIELD}` read and "
                    f"there is none. Either the read was removed (delete the entry) or "
                    f"it was renamed (update it). Reason on record: {why}"
                )

    if failures:
        print(f"FAIL: the phase 78 `{FIELD}` invariant (issue #0056)")
        for f in failures:
            print(f"  - {f}")
        print()
        print("Phase 78: one derivation, two consumers. A node's rate is its fastest")
        print("timer trigger; a declared floor is a promise and a promise is not a")
        print("period. Roadmap: docs/roadmap/phase-78-one-derivation-two-consumers.md")
        return 1

    sites = ", ".join(f"{n}:{ln} ({fn})" for n, ln, fn in reads) or "none"
    print(f"  ok: {checked} sched file(s), {len(reads)} licensed `{FIELD}` read(s): {sites}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
