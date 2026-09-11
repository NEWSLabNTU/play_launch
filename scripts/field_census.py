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

Where the sources come from:

  ros-launch-manifest   The tag the three Cargo manifests pin, read from
                        cargo's own checkout of it (`$CARGO_HOME/git/
                        checkouts/`) — the tree the build actually compiled.
                        It used to be a sibling checkout of the repository,
                        on whatever branch that clone happened to be on, and
                        nothing tied that to the pin: the census could grade
                        a manifest the build never saw, and on a machine
                        without the sibling (CI) it could not run at all.
                        `--manifest-repo` overrides, for grading an unreleased
                        manifest before it is tagged.
  nano-ros              A consumer, not a dependency, so nothing pins it: the
                        census measures the consumer as it IS. Sibling
                        checkout by default, `--nano-ros` to name another.
                        A location that was NAMED and is absent is an error;
                        an unnamed sibling that is absent is a SKIP locally
                        and an error under CI, where a skip is a green tick
                        nobody reads.

Usage:
    scripts/field_census.py [--json] [--all] [--check]
                            [--manifest-repo DIR] [--nano-ros DIR]

  --all   list every field, not just the write-only ones.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass
from dataclasses import field as dc_field
from pathlib import Path
from typing import NoReturn

REPO = Path(__file__).resolve().parent.parent

# The manifest crate is a git dependency pinned by TAG in three Cargo manifests
# that must move together (CLAUDE.md, "Repository Layout"). The lockfiles
# carry the tag AND the commit it resolved to, which is what lets the census
# find cargo's checkout of exactly that tree without a second copy of the pin.
MANIFEST_LOCKFILES = [
    REPO / "Cargo.lock",
    REPO / "src/ros-launch-resolve/Cargo.lock",
    REPO / "tests/Cargo.lock",
]
MANIFEST_SOURCE_RE = re.compile(
    r'source = "git\+https://github\.com/NEWSLabNTU/ros-launch-manifest'
    r'(?:\.git)?\?tag=([^#"]+)#([0-9a-f]{40})"'
)
# The layer-2 workspace: it resolves with no ROS installed, so `cargo fetch`
# against it can populate the checkout on a machine that has not built yet.
LAYER2_MANIFEST = REPO / "src/ros-launch-resolve/Cargo.toml"

ENV_MANIFEST_REPO = "PLAY_LAUNCH_FIELD_CENSUS_MANIFEST"
ENV_NANO_ROS = "PLAY_LAUNCH_FIELD_CENSUS_NANO_ROS"

# ── Which files are transport, and which consume ───────────────────────────
#
# Matched against DISPLAY paths (`rlm/types/src/parse.rs`, `src/play_launch/
# src/...`, `nano-ros/packages/...`), never against absolute ones: cargo's
# checkout lives wherever `$CARGO_HOME` is, and a `$CARGO_HOME` under a
# directory called `target/` or `docs/` would otherwise have every manifest
# read silently ignored — and every field reported write-only.
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
    (r"resolve/src/ros/param_check\.rs$", "parameter check"),
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
    (r"^nano-ros/", "nano-ros"),
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


@dataclass(frozen=True)
class Root:
    """A tree to grep, and how its hits are named in the report.

    `base` is the checkout; `sub` the part of it to search; `display` the
    prefix a hit is reported under in place of `base` (`rlm/` for the manifest
    crate, `nano-ros/`, or nothing for this repository), so a report reads the
    same wherever the checkout happens to live.
    """

    base: Path
    sub: str
    display: str

    @property
    def path(self) -> Path:
        return self.base / self.sub


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


def read_sites(name: str, roots: list[Root]) -> list[str]:
    """Every `.<field>` mention across the given trees, as display paths.

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
        if not root.path.exists():
            continue
        try:
            res = subprocess.run(
                [
                    "grep",
                    "-rlI",
                    "--include=*.rs",
                    # Pruned rather than filtered afterwards: `/target/` hits
                    # are IGNORED below anyway, and a sibling checkout with
                    # build artifacts turns a 3 s census into minutes.
                    "--exclude-dir=target",
                    "--exclude-dir=.git",
                    "-E",
                    pattern,
                    str(root.path),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
        except FileNotFoundError:
            print("grep not found", file=sys.stderr)
            raise
        base = str(root.base) + "/"
        for line in res.stdout.splitlines():
            if not line:
                continue
            if line.startswith(base):
                line = root.display + line[len(base) :]
            out.append(line)
    return out


# ── Locating the sources ───────────────────────────────────────────────────


def die(message: str) -> NoReturn:
    """Exit 2: the census could not RUN. Exit 1 is reserved for a finding."""
    print(message, file=sys.stderr)
    raise SystemExit(2)


def git(path: Path, *args: str) -> str | None:
    try:
        res = subprocess.run(
            ["git", "-C", str(path), *args],
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError:
        return None
    return res.stdout.strip() if res.returncode == 0 else None


def describe_checkout(path: Path) -> str:
    head = git(path, "rev-parse", "--short", "HEAD")
    if head is None:
        return "not a git checkout"
    branch = git(path, "branch", "--show-current") or "detached"
    dirty = " (uncommitted changes)" if git(path, "status", "--porcelain") else ""
    return f"{branch} @ {head}{dirty}"


def pinned_manifest() -> tuple[str, str]:
    """The `(tag, rev)` the lockfiles pin `ros-launch-manifest` at.

    All three lockfiles are read, and they must agree: the three manifests are
    required to name one tag (CLAUDE.md), and `tests/` is the one that drifts
    unnoticed because nothing there fails to compile when it does.
    """
    pins: dict[tuple[str, str], list[str]] = {}
    for lock in MANIFEST_LOCKFILES:
        if not lock.exists():
            continue
        for m in MANIFEST_SOURCE_RE.finditer(lock.read_text()):
            pins.setdefault((m.group(1), m.group(2)), []).append(str(lock.relative_to(REPO)))
    if not pins:
        die(
            "Cannot run the census: no lockfile names the ros-launch-manifest "
            "tag.\nLooked in:\n  " + "\n  ".join(str(lock) for lock in MANIFEST_LOCKFILES)
        )
    if len(pins) > 1:
        die(
            "Cannot run the census: the lockfiles pin ros-launch-manifest at "
            "different revisions,\nso there is no ONE tree to grade. They must "
            "move together (`just bump-manifest <tag>`):\n  "
            + "\n  ".join(
                f"{tag} ({rev[:7]}): {', '.join(sorted(set(locks)))}"
                for (tag, rev), locks in sorted(pins.items())
            )
        )
    return next(iter(pins))


def cargo_checkout(rev: str) -> Path | None:
    """Cargo's checkout of `rev`, if it has one.

    Cargo keeps one directory per git source under `git/checkouts/<name>-<hash>/`
    and one per revision inside it, named by the SHORT rev; `.cargo-ok` marks a
    checkout that finished. The rev is confirmed from the checkout's own git
    HEAD rather than trusted from the directory name.
    """
    cargo_home = Path(os.environ.get("CARGO_HOME") or Path.home() / ".cargo")
    for candidate in sorted(
        (cargo_home / "git/checkouts").glob(f"ros-launch-manifest-*/{rev[:7]}")
    ):
        if not (candidate / ".cargo-ok").exists():
            continue
        head = git(candidate, "rev-parse", "HEAD")
        if head not in (None, rev):
            continue
        return candidate
    return None


def manifest_sources(repo: Path) -> list[Path]:
    return [repo / "types/src/types.rs", repo / "model/src/lib.rs"]


def locate_manifest_repo(explicit: Path | None) -> Path:
    tag, rev = pinned_manifest()

    if explicit is not None:
        missing = [s for s in manifest_sources(explicit) if not s.exists()]
        if missing:
            die(
                f"--manifest-repo / ${ENV_MANIFEST_REPO} names {explicit}, but "
                "these are not in it:\n  " + "\n  ".join(str(m) for m in missing)
            )
        head = git(explicit, "rev-parse", "HEAD")
        where = describe_checkout(explicit)
        if head != rev or "uncommitted" in where:
            print(
                f"note: grading {explicit} ({where}), not the {tag} the build "
                f"pins ({rev[:7]}); the verdicts describe that tree.",
                file=sys.stderr,
            )
        else:
            print(f"sources: ros-launch-manifest {tag} from {explicit}", file=sys.stderr)
        return explicit

    # A sibling checkout is where the pin USED to be read from, so someone
    # editing the manifest there may still expect their edits to be graded.
    # They are not: the census grades the tag the build compiled. Say so when
    # the sibling would have given a different answer, and only then.
    sibling = REPO.parent / "ros-launch-manifest"
    if all(s.exists() for s in manifest_sources(sibling)):
        where = describe_checkout(sibling)
        if git(sibling, "rev-parse", "HEAD") != rev or "uncommitted" in where:
            print(
                f"note: {sibling} ({where}) is not the {tag} the build pins "
                f"({rev[:7]}) and is NOT what this census grades; pass "
                f"--manifest-repo {sibling} to grade it instead.",
                file=sys.stderr,
            )

    checkout = cargo_checkout(rev)
    if checkout is None:
        print(
            f"note: cargo has no checkout of ros-launch-manifest {tag} "
            f"({rev[:7]}) yet; fetching it via {LAYER2_MANIFEST.relative_to(REPO)}",
            file=sys.stderr,
        )
        subprocess.run(
            ["cargo", "fetch", "--locked", "--manifest-path", str(LAYER2_MANIFEST)],
            check=False,
        )
        checkout = cargo_checkout(rev)
    if checkout is None:
        die(
            f"Cannot run the census: no cargo checkout of ros-launch-manifest "
            f"{tag} ({rev[:7]}) under\n  "
            f"{Path(os.environ.get('CARGO_HOME') or Path.home() / '.cargo')}"
            "/git/checkouts/\nBuild once (`just build-rust`) or name a checkout "
            f"with --manifest-repo / ${ENV_MANIFEST_REPO}."
        )
    print(f"sources: ros-launch-manifest {tag} from {checkout}", file=sys.stderr)
    return checkout


def locate_nano_ros(explicit: Path | None) -> Path | None:
    if explicit is not None:
        if not (explicit / "packages").is_dir():
            die(
                f"--nano-ros / ${ENV_NANO_ROS} names {explicit}, but there is no "
                "nano-ros checkout there (no `packages/` directory)."
            )
        path = explicit
    else:
        path = REPO.parent / "nano-ros"
        if not (path / "packages").is_dir():
            return None
    print(f"sources: nano-ros from {path} ({describe_checkout(path)})", file=sys.stderr)
    return path


def path_arg(value: str) -> Path:
    return (REPO / value).resolve()


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--all", action="store_true", help="report every field")
    ap.add_argument(
        "--check",
        action="store_true",
        help="fail if a field became write-only that was not already known to be",
    )
    ap.add_argument(
        "--manifest-repo",
        type=path_arg,
        default=os.environ.get(ENV_MANIFEST_REPO) or None,
        metavar="DIR",
        help=f"a ros-launch-manifest checkout to grade instead of the pinned tag "
        f"(env ${ENV_MANIFEST_REPO}); relative to the repository root",
    )
    ap.add_argument(
        "--nano-ros",
        type=path_arg,
        default=os.environ.get(ENV_NANO_ROS) or None,
        metavar="DIR",
        help=f"the nano-ros checkout whose reads count (env ${ENV_NANO_ROS}); "
        f"default: the sibling checkout `{REPO.parent / 'nano-ros'}`",
    )
    # argparse applies `type` to a STRING default too, so the env-var defaults
    # above arrive as resolved paths, and an unset one stays `None`.
    args = ap.parse_args()

    manifest_repo = locate_manifest_repo(args.manifest_repo)
    sources = manifest_sources(manifest_repo)
    missing = [s for s in sources if not s.exists()]
    if missing:
        print(
            "Cannot run the census: these are not where this script expects "
            "them:\n  " + "\n  ".join(str(m) for m in missing),
            file=sys.stderr,
        )
        return 2

    nano_ros = locate_nano_ros(args.nano_ros)
    roots = [
        Root(manifest_repo, "types", "rlm/"),
        Root(manifest_repo, "check", "rlm/"),
        Root(manifest_repo, "sched", "rlm/"),
        Root(manifest_repo, "model", "rlm/"),
        Root(REPO, "src/ros-launch-resolve/resolve/src", ""),
        Root(REPO, "src/play_launch/src", ""),
    ]
    if nano_ros is not None:
        roots.append(Root(nano_ros, "packages", "nano-ros/"))
    elif args.check:
        # A gate that cannot see every consumer must not fail. nano-ros reads
        # the model directly, so without it several model fields look unread
        # that are not — and a red gate for a missing sibling checkout teaches
        # people to ignore this one.
        #
        # Except under CI, where nobody reads a skip: there the checkout is
        # provisioned by the workflow and named in the environment, so its
        # absence is a broken workflow, not a developer's machine.
        if os.environ.get("CI"):
            print(
                "CI has no nano-ros checkout, so the census cannot see every "
                f"consumer. The workflow must provide one and name it in "
                f"${ENV_NANO_ROS}; a gate that skips here is a green tick "
                "nobody reads.",
                file=sys.stderr,
            )
            return 2
        print(
            f"SKIP: the census is incomplete without {REPO.parent / 'nano-ros'}"
            " — model fields\nread only by nano-ros would be reported as "
            "unread. Clone it to run this gate."
        )
        return 0
    else:
        print(
            f"note: {REPO.parent / 'nano-ros'} is absent, so model fields only "
            "IT reads will be reported as write-only. Clone it, or name a "
            f"checkout with --nano-ros / ${ENV_NANO_ROS}, for a complete census.",
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
        for path in read_sites(f.name, roots):
            kind, label = classify(path)
            entry = f"{label}: {path}"
            if kind == "transport":
                f.transport.append(entry)
            elif kind == "consuming":
                f.consuming.append(entry)
            elif kind == "unknown":
                f.unknown.append(entry)

    ordered = sorted(by_name.values(), key=lambda f: (f.verdict != "WRITE-ONLY", f.name))
    shown = [f for f in ordered if args.all or f.verdict != "consumed"]

    if args.check:
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
