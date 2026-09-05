#!/usr/bin/env python3
"""Who READS each contract field?

Phase 69 made the contract grammar enumerable: every key a contract may carry
is a row in `ros-launch-manifest`'s `types/src/field_table.rs`. That says what
is LEGAL. It does not say what is READ, and the difference is where this
campaign's recurring defect lives.

Four fields were retired in phases 67/68 — `chains`'s `semantics`, an
endpoint's `jitter`, and the pair `lifespan`/`max_response` before they were
given rules — and every one was found the same way, by hand: grep the field,
look at each hit, notice that all of them are *transport*. `jitter` had three
read sites for its whole life, and its own documentation row said "Not
checked".

So the test is not "is this field mentioned?" but "is it mentioned anywhere
that CONSUMES it?":

  TRANSPORT   parsing it, serializing it, copying it into the model, comparing
              it for merge equality, or linting it as deprecated. Necessary,
              and none of it is a reason for the field to exist.
  CONSUMING   a check rule, mapper arithmetic, a runtime monitor. The field
              earns its place here or nowhere.

A field with zero consuming reads is write-only: delete it, or write the rule
it is waiting for. Both are decisions; the point is that neither gets made by
accident.

Usage:
    scripts/field_census.py [--json] [--all]

  --all   list every field, not just the write-only ones.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from dataclasses import dataclass
from dataclasses import field as dc_field
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
MANIFEST_REPO = REPO.parent / "ros-launch-manifest"

# ── Which files are transport, and which consume ───────────────────────────
#
# Ordered: the first pattern that matches a path decides. TRANSPORT is listed
# first because several transport files live inside directories that are
# otherwise consumers (`model_builder.rs` sits beside `sched_derive.rs`).
TRANSPORT_PATTERNS = [
    (r"types/src/parse\.rs$", "parse"),
    (r"types/src/types\.rs$", "struct definition"),
    (r"types/src/field_table\.rs$", "grammar table"),
    (r"model/src/lib\.rs$", "model struct"),
    (r"resolve/src/ros/model_builder\.rs$", "model lowering"),
    (r"check/src/rules/deprecated_unit_suffix\.rs$", "deprecation lint"),
    (r"check/src/rules/consistency\.rs$", "merge equality"),
    (r"resolve/src/ros/causal_graph\.rs$", "graph export"),
    (r"resolve/src/ros/causal_dag_global\.rs$", "graph export"),
    # The CLI verbs render and count; printing a field's value is not acting
    # on it. `check --explain` is the one that reports a DERIVED verdict, and
    # what derived it is already classified as consuming.
    (r"resolve/src/verbs/", "CLI reporting"),
    (r"resolve/src/model\.rs$", "model assembly"),
]

CONSUMING_PATTERNS = [
    (r"check/src/rules/", "check rule"),
    (r"resolve/src/ros/manifest_loader\.rs$", "cross-scope check"),
    (r"resolve/src/ros/manifest_graph\.rs$", "dataflow arithmetic"),
    (r"resolve/src/ros/sched_derive\.rs$", "mapper input"),
    (r"resolve/src/ros/sched_loader\.rs$", "scheduling derivation"),
    (r"sched/src/", "scheduling"),
    (r"check/src/graph\.rs$", "dataflow graph"),
    # The executor. A field the runtime acts on — spawning, scheduling,
    # enforcing, monitoring — is consumed in the strongest sense available:
    # it changes what the running system does.
    (r"src/play_launch/src/execution/", "executor"),
    (r"src/play_launch/src/runtime_enforcement/", "runtime enforcement"),
    (r"src/play_launch/src/member_actor/", "member lifecycle"),
    (r"src/play_launch/src/commands/", "verb implementation"),
    (r"src/play_launch/src/", "runtime"),
    # nano-ros. It builds its `MapperPath` from the MODEL, never from the
    # manifest, so a model field it reads is consumed even though nothing in
    # THIS repository touches it. Phase 68 W5 found both halves of that seam
    # unobservable from either side alone; a census that stopped at the repo
    # boundary would report `node_concurrency` — added in that very wave, FOR
    # them — as write-only.
    (r"/nano-ros/", "nano-ros"),
]

# Hits here prove nothing either way: a test can exercise a field the product
# never reads, which is exactly how a vacuous test looks from the outside.
IGNORED_PATTERNS = [
    # nano-ros VENDORS a copy of this repository. Its files are ours echoed
    # back, so counting them would let a field look consumed downstream on the
    # strength of the very lowering code that is transport here.
    r"/third-party/play_launch/",
    r"/tests?/",
    r"_test\.rs$",
    r"/target/",
    r"/docs/",
]


def classify(path: str) -> tuple[str, str]:
    for pat in IGNORED_PATTERNS:
        if re.search(pat, path):
            return ("ignored", "test or generated")
    for pat, label in TRANSPORT_PATTERNS:
        if re.search(pat, path):
            return ("transport", label)
    for pat, label in CONSUMING_PATTERNS:
        if re.search(pat, path):
            return ("consuming", label)
    return ("unknown", "unclassified file")


@dataclass
class Field:
    struct: str
    name: str
    transport: list[str] = dc_field(default_factory=list)
    consuming: list[str] = dc_field(default_factory=list)
    unknown: list[str] = dc_field(default_factory=list)

    @property
    def verdict(self) -> str:
        if self.consuming:
            return "consumed"
        if self.unknown:
            return "unclear"
        return "WRITE-ONLY"


def parse_struct_fields(src: Path) -> list[Field]:
    """Rust field names per struct, from `types.rs` and the model's `lib.rs`.

    Keyed on the RUST name rather than the YAML one on purpose: a read site
    names the field, not the key, and several differ (`if` is `if_condition`,
    `type` is `msg_type`). Serde attributes are ignored here for the same
    reason — they describe the wire form, and this is a question about code.
    """
    fields: list[Field] = []
    struct = None
    for line in src.read_text().splitlines():
        m = re.match(r"pub (?:struct|enum) (\w+)", line)
        if m:
            struct = m.group(1)
            continue
        if struct is None:
            continue
        m = re.match(r"\s+pub (\w+):", line)
        if m:
            fields.append(Field(struct=struct, name=m.group(1)))
    return fields


def read_sites(name: str, roots: list[Path]) -> list[str]:
    """Every `.<field>` mention across the given trees.

    A leading dot is what makes this a READ rather than a definition or a
    struct-literal key: `decl.max_jitter` is someone using the value, while
    `max_jitter: ...` inside an initializer is someone filling it in. The
    distinction is imperfect (a field read through a pattern match is missed)
    and deliberately errs toward reporting MORE reads, so a write-only verdict
    is conservative.
    """
    pattern = rf"\.{re.escape(name)}\b"
    out: list[str] = []
    for root in roots:
        if not root.exists():
            continue
        try:
            res = subprocess.run(
                ["grep", "-rnI", "--include=*.rs", "-E", pattern, str(root)],
                capture_output=True,
                text=True,
                check=False,
            )
        except FileNotFoundError:
            print("grep not found", file=sys.stderr)
            raise
        out.extend(line for line in res.stdout.splitlines() if line)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--all", action="store_true", help="report every field")
    ap.add_argument(
        "--check",
        action="store_true",
        help="fail if a field became write-only that was not already known to be",
    )
    args = ap.parse_args()

    sources = [
        MANIFEST_REPO / "types/src/types.rs",
        MANIFEST_REPO / "model/src/lib.rs",
    ]
    missing = [s for s in sources if not s.exists()]
    if missing:
        print(
            "Cannot run the census: these are not where this script expects "
            "them:\n  " + "\n  ".join(str(m) for m in missing),
            file=sys.stderr,
        )
        return 2

    nano_ros = REPO.parent / "nano-ros"
    roots = [
        MANIFEST_REPO / "types",
        MANIFEST_REPO / "check",
        MANIFEST_REPO / "sched",
        MANIFEST_REPO / "model",
        REPO / "src/ros-launch-resolve/resolve/src",
        REPO / "src/play_launch/src",
    ]
    if nano_ros.exists():
        roots.append(nano_ros / "packages")
    else:
        print(
            f"note: {nano_ros} is absent, so model fields only IT reads will "
            "be reported as write-only. Clone it for a complete census.",
            file=sys.stderr,
        )

    fields: list[Field] = []
    for src in sources:
        fields.extend(parse_struct_fields(src))

    # One entry per NAME, not per (struct, name): a read site says `.max_age`
    # and cannot say which struct it came from without type inference. Merging
    # them keeps the verdict honest — it can only make a field look MORE read.
    by_name: dict[str, Field] = {}
    for f in fields:
        by_name.setdefault(f.name, Field(struct=f.struct, name=f.name))

    for f in by_name.values():
        for hit in read_sites(f.name, roots):
            path = hit.split(":", 1)[0]
            kind, label = classify(path)
            entry = f"{label}: {path.replace(str(REPO) + '/', '').replace(str(MANIFEST_REPO) + '/', 'rlm/').replace(str(nano_ros) + '/', 'nano-ros/')}"
            if kind == "transport":
                f.transport.append(entry)
            elif kind == "consuming":
                f.consuming.append(entry)
            elif kind == "unknown":
                f.unknown.append(entry)

    ordered = sorted(by_name.values(), key=lambda f: (f.verdict != "WRITE-ONLY", f.name))
    shown = [f for f in ordered if args.all or f.verdict != "consumed"]

    if args.check:
        if not nano_ros.exists():
            # A gate that cannot see every consumer must not fail. nano-ros
            # reads the model directly, so without it several model fields
            # look unread that are not — and a red gate for a missing sibling
            # checkout teaches people to ignore this one.
            print(
                f"SKIP: the census is incomplete without {nano_ros} — model "
                "fields\nread only by nano-ros would be reported as unread. "
                "Clone it to run this gate."
            )
            return 0
        baseline_path = REPO / "scripts/field_census_baseline.txt"
        known = {
            line.split("#", 1)[0].strip()
            for line in baseline_path.read_text().splitlines()
            if line.split("#", 1)[0].strip()
        }
        found = {f.name for f in ordered if f.verdict != "consumed"}
        new = sorted(found - known)
        gone = sorted(known - found)
        for name in new:
            print(f"NEW write-only field: {name}")
        for name in gone:
            print(f"no longer write-only (drop from the baseline): {name}")
        if new:
            print(
                "\nA field nothing reads is a comment with a schema. Either write "
                "the rule\nit is waiting for, or delete it — and if it is "
                "deliberately unread for now,\nsay so in "
                "scripts/field_census_baseline.txt with the reason.",
                file=sys.stderr,
            )
            return 1
        if gone:
            print(
                "\nThe baseline is stale. Remove those lines: a baseline that "
                "lists fields\nsomeone has since wired up hides the next one "
                "that goes unread.",
                file=sys.stderr,
            )
            return 1
        print(f"{len(found)} unread field(s), all known.")
        return 0

    if args.json:
        print(
            json.dumps(
                [
                    {
                        "field": f.name,
                        "struct": f.struct,
                        "verdict": f.verdict,
                        "consuming": sorted(set(f.consuming)),
                        "transport": sorted(set(f.transport)),
                        "unknown": sorted(set(f.unknown)),
                    }
                    for f in shown
                ],
                indent=2,
            )
        )
        return 0

    write_only = [f for f in ordered if f.verdict == "WRITE-ONLY"]
    unclear = [f for f in ordered if f.verdict == "unclear"]

    for f in shown:
        print(f"{f.verdict:<11} {f.name}  ({f.struct})")
        for site in sorted(set(f.consuming)):
            print(f"              READ  {site}")
        for site in sorted(set(f.unknown)):
            print(f"              ?     {site}")
        if not f.consuming and not f.unknown:
            for site in sorted(set(f.transport)):
                print(f"              moved {site}")
        print()

    print(
        f"{len(by_name)} fields: {len(write_only)} write-only, "
        f"{len(unclear)} unclear, "
        f"{len(by_name) - len(write_only) - len(unclear)} consumed"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
