#!/usr/bin/env python3
"""How well can diagnostics be joined to the nodes play_launch spawned?

Phase 62 W2 proposes a diagnostics badge per node card. That needs a join from
`DiagnosticStatus` to a spawned member, and the join is a heuristic: neither
`hardware_id` nor `name` is required by ROS to identify a node. The roadmap
says to answer this against the corpus before writing code. This is that.

Usage: diag_join_analysis.py <play_log_run_dir> [...]

Reads each run's `diagnostics.csv` plus the `node/` and `load_node/`
directories play_launch created, and reports what fraction of diagnostic names
can be attributed, by which rule, and what is left over.
"""
import csv
import collections
import pathlib
import re
import sys


def members(run: pathlib.Path):
    """Spawned member names, as play_launch recorded them.

    Node directories carry the issue-0018 ordinal (`<exec>-1`) when the launch
    file gave no name; that suffix is play_launch's, not the node's, so it is
    stripped before matching.
    """
    nodes, comps = set(), set()
    for d in (run / "node").glob("*"):
        if d.is_dir():
            nodes.add(re.sub(r"-\d+$", "", d.name))
    for d in (run / "load_node").glob("*"):
        if d.is_dir():
            comps.add(re.sub(r"-\d+$", "", d.name))
    return nodes, comps


def classify(diag_name: str, nodes: set, comps: set) -> str:
    # Convention is "<node>: <check>", but nothing enforces it.
    prefix = diag_name.split(":", 1)[0].strip() if ":" in diag_name else diag_name.strip()
    leaf = prefix.rsplit("/", 1)[-1]

    if prefix in nodes:
        return "node exact"
    if prefix in comps:
        return "composable exact"
    if prefix.startswith("/"):
        # A ROS path, e.g. "/adapi/node/localization". The last segment is the
        # only part that can match a member name.
        if leaf in nodes:
            return "node via path leaf"
        if leaf in comps:
            return "composable via path leaf"
        return "path, unmatched"
    if leaf in nodes:
        return "node via leaf"
    if leaf in comps:
        return "composable via leaf"
    return "unmatched"


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    for arg in sys.argv[1:]:
        run = pathlib.Path(arg)
        csv_path = run / "diagnostics.csv"
        if not csv_path.is_file():
            print(f"{run}: no diagnostics.csv")
            continue
        nodes, comps = members(run)

        names = set()
        hw = collections.Counter()
        hw_by_name = collections.defaultdict(set)
        with csv_path.open(newline="", errors="ignore") as f:
            for r in csv.DictReader(f):
                n = r.get("diagnostic_name", "")
                names.add(n)
                h = r.get("hardware_id", "")
                hw[h] += 1
                hw_by_name[n].add(h)

        buckets = collections.Counter(classify(n, nodes, comps) for n in names)
        print(f"\n=== {run}")
        print(f"  spawned: {len(nodes)} nodes, {len(comps)} composables")
        print(f"  distinct diagnostic names: {len(names)}")
        matched = sum(v for k, v in buckets.items() if not k.endswith("unmatched"))
        print(f"  attributable: {matched}/{len(names)} ({100*matched/max(len(names),1):.0f}%)")
        for k, v in buckets.most_common():
            print(f"    {k:24s} {v:4d}")

        # Is hardware_id a node identifier, or something else?
        host_like = sum(v for k, v in hw.items() if k in ("", "ubuntu") or "." in k)
        print(f"  hardware_id: {len(hw)} distinct; "
              f"{100*host_like/max(sum(hw.values()),1):.0f}% of rows are empty/hostname")
        useful = [k for k in hw if k and k not in ("ubuntu",) and (k in nodes or k in comps)]
        print(f"    values that name a spawned member: {len(useful)}/{len(hw)}")

        # Would hardware_id rescue anything the name could not attribute?
        rescued = 0
        for n in names:
            if classify(n, nodes, comps).endswith("unmatched"):
                if any(h in nodes or h in comps for h in hw_by_name[n]):
                    rescued += 1
        print(f"    unmatched names hardware_id could rescue: {rescued}")

        un = sorted(n for n in names if classify(n, nodes, comps).endswith("unmatched"))
        if un:
            print(f"  unmatched examples ({len(un)}):")
            for n in un[:8]:
                print(f"    {n[:70]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
