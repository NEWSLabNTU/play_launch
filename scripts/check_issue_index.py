#!/usr/bin/env python3
"""Fail if `docs/issues/README.md`'s Open list disagrees with the per-file statuses.

The list is hand-written prose and the statuses are frontmatter, so they drift
-- and they have, twice in one day. Once an issue was fixed and the README kept
advertising it, which cost an agent dispatch against solved work; once four
shipped issues were still listed open. Neither is visible by reading either
file alone, which is what makes it a job for a check rather than for care.
"""
import pathlib
import re
import sys

ISSUES = pathlib.Path(__file__).resolve().parent.parent / "docs" / "issues"


def statuses() -> dict[str, str]:
    """id -> status, from the frontmatter of every non-archived issue file."""
    out = {}
    for path in sorted(ISSUES.glob("[0-9][0-9][0-9][0-9]-*.md")):
        head = path.read_text().split("---")[1] if path.read_text().startswith("---") else ""
        m = re.search(r"^status:\s*(\S+)", head, re.M)
        ident = path.name[:4]
        out[ident] = m.group(1) if m else "MISSING"
    return out


def listed_open() -> set[str]:
    """ids the README advertises under `## Open`."""
    text = (ISSUES / "README.md").read_text()
    start = text.index("## Open")
    end = text.index("## Resolved", start)
    return set(re.findall(r"^\*\*#(\d{4})\*\*", text[start:end], re.M))


def main() -> int:
    st = statuses()
    listed = listed_open()
    actually_open = {i for i, s in st.items() if s == "open"}

    stale = sorted(listed - actually_open)
    missing = sorted(actually_open - listed)
    nostatus = sorted(i for i, s in st.items() if s == "MISSING")

    for ident in stale:
        print(f"FAIL: README lists #{ident} under Open, but its file says "
              f"`status: {st.get(ident, '?')}`")
        print("  why: an issue advertised as open after it is fixed sends the next "
              "reader -- or agent -- at solved work")
    for ident in missing:
        print(f"FAIL: #{ident} is `status: open` and the README does not list it")
        print("  why: an open issue nobody can find from the index is not tracked")
    for ident in nostatus:
        print(f"FAIL: #{ident} has no `status:` in its frontmatter")

    if stale or missing or nostatus:
        print()
        print("The Open section is derived data. Rebuild it from the statuses "
              "rather than editing both by hand.")
        return 1

    print(f"  ok: {len(actually_open)} open issue(s), README agrees "
          f"({len(st)} tracked)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
