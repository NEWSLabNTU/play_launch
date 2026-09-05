#!/usr/bin/env python3
"""How often does an authored number agree with the derived one?

Phase 68 retired `chains:` on a provenance argument made by hand: the
derivation reproduced every authored route in the corpus, so the authored
copy was redundant. That argument was right, and it was also unrepeatable —
nothing counted, so nothing could say when the NEXT field had earned the
same retirement.

This counts. For every field the resolver can derive, it runs `check` over
the fixture corpus and tallies the resolver's own verdicts:

  agree       the authored value equals the derived one  (`derivable-*`)
  disagree    the authored value differs                  (`*-mismatch`)
  underivable authored, and the graph cannot derive it (no verdict — a
              genuine absence of information: `once`, `spontaneous`, an
              external publisher, a cycle)

The evidence for retiring an authored copy is `disagree == 0` with `agree`
large: every number the corpus wrote by hand, the graph already knew. A
non-zero `disagree` is not an argument against derivation — it is a list of
contracts one of whose numbers is wrong, and nothing else would have said so.

Usage:
    scripts/derivation_census.py [--json] [--bin PATH]
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent

# field → (agree rule, disagree rule, how to count authored occurrences)
FIELDS = {
    "topics.<t>.rate_hz": ("derivable-rate", "rate-mismatch", "topic_rate"),
    "nodes.<n>.pub.<e>.min_rate_hz": ("derivable-min-rate", "min-rate-mismatch", "pub_min_rate"),
}


def default_bin() -> Path:
    # The BUILD, never whatever `play_launch` is on PATH: a pip-installed copy
    # shadows it and fails a new rule with "unrecognized" (issue #0020).
    return REPO / "install/play_launch/lib/play_launch/play_launch"


def fixtures() -> list[Path]:
    return sorted(REPO.glob("tests/fixtures/*/launch/bringup.launch.xml"))


def contracts_beside(launch: Path) -> list[Path]:
    return sorted(launch.parent.glob("*.contract.yaml"))


def count_authored(doc: dict, how: str) -> int:
    if how == "topic_rate":
        return sum(
            1 for t in (doc.get("topics") or {}).values() if isinstance(t, dict) and "rate_hz" in t
        )
    if how == "pub_min_rate":
        n = 0
        for node in (doc.get("nodes") or {}).values():
            pubs = (node or {}).get("pub") or {}
            if isinstance(pubs, dict):
                n += sum(1 for p in pubs.values() if isinstance(p, dict) and "min_rate_hz" in p)
        return n
    raise ValueError(how)


def run_check(bin_path: Path, launch: Path) -> list[dict]:
    """`check --format json` prints one JSON array per section; read them all."""
    res = subprocess.run(
        [str(bin_path), "check", "--format", "json", str(launch)],
        capture_output=True,
        text=True,
        check=False,
    )
    out: list[dict] = []
    dec = json.JSONDecoder()
    text = res.stdout
    i = 0
    while True:
        m = re.compile(r"\S").search(text, i)
        if not m:
            break
        try:
            val, j = dec.raw_decode(text, m.start())
        except json.JSONDecodeError:
            break
        if isinstance(val, list):
            out.extend(d for d in val if isinstance(d, dict))
        i = j
    return out


@dataclass
class Tally:
    authored: int = 0
    agree: int = 0
    disagree: int = 0
    disagreements: list[str] = field(default_factory=list)

    @property
    def underivable(self) -> int:
        return max(self.authored - self.agree - self.disagree, 0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--bin", type=Path, default=default_bin())
    args = ap.parse_args()

    if not args.bin.exists():
        print(f"no binary at {args.bin} — run `just build-rust` first", file=sys.stderr)
        return 2

    tallies = {f: Tally() for f in FIELDS}
    launches = fixtures()
    for launch in launches:
        for c in contracts_beside(launch):
            try:
                doc = yaml.safe_load(c.read_text()) or {}
            except yaml.YAMLError:
                continue
            for f, (_, _, how) in FIELDS.items():
                tallies[f].authored += count_authored(doc, how)
        # A fixture that SAYS it is wrong is not a finding; it is the test
        # that keeps the rule honest. Reported, but marked.
        deliberate = any(
            "DELIBERATELY" in "\n".join(c.read_text().splitlines()[:15])
            for c in contracts_beside(launch)
        )
        for d in run_check(args.bin, launch):
            for f, (agree, disagree, _) in FIELDS.items():
                if d.get("rule") == agree:
                    tallies[f].agree += 1
                elif d.get("rule") == disagree:
                    tallies[f].disagree += 1
                    tag = " (fixture built to disagree)" if deliberate else ""
                    tallies[f].disagreements.append(
                        f"{launch.parent.parent.name}: {d.get('path')}{tag}"
                    )

    if args.json:
        print(
            json.dumps(
                {f: {**t.__dict__, "underivable": t.underivable} for f, t in tallies.items()},
                indent=2,
            )
        )
        return 0

    print(
        f"{len(launches)} launch fixture(s), {sum(len(contracts_beside(x)) for x in launches)} contract file(s)\n"
    )
    w = max(len(f) for f in FIELDS)
    print(f"{'field':<{w}}  authored  agree  disagree  underivable")
    for f, t in tallies.items():
        print(f"{f:<{w}}  {t.authored:>8}  {t.agree:>5}  {t.disagree:>8}  {t.underivable:>11}")
    print()
    for f, t in tallies.items():
        derivable = t.agree + t.disagree
        if t.authored == 0:
            print(f"{f}: nothing authored in the corpus — nothing to retire.")
        elif t.disagree == 0 and derivable:
            print(
                f"{f}: every derivable authored value agrees ({t.agree}/{derivable}); "
                f"{t.underivable} authored where the graph has no answer. "
                "Retirement evidence: the derivable copies are redundant."
            )
        elif t.disagree:
            print(
                f"{f}: {t.disagree} disagreement(s) — each is a wrong number, not a case against deriving:"
            )
            for line in t.disagreements:
                print(f"    {line}")
        else:
            print(f"{f}: authored {t.authored} time(s), none derivable — the graph cannot say.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
