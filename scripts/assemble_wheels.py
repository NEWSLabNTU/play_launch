#!/usr/bin/env python3
"""Merge one-payload wheels into one wheel per wheel tag (phase 92 P2).

Each release-matrix row builds a wheel for one ROS distribution: shared
files (Python code, web assets) plus that distribution's payload under
``play_launch/dist/<distro>/``. Rows whose wheels carry the same tag
(cp312 + manylinux_2_39 for jazzy and kilted, for example) are merged
into one wheel holding every payload, which ``payload.py`` chooses from
at run time.

Refuses rather than guesses:
- a row wheel carrying no payload or more than one;
- the same distribution twice under one tag;
- a shared file that differs between rows of one tag (they are built from
  one commit, so a difference means a row built something else);
- a distribution in ``--expect`` that landed in no wheel, or one that
  landed in a wheel not expected.

Rows of one tag build wheels of the same file name, so each row's wheel
sits in its own subdirectory of IN_DIR (as actions/download-artifact lays
them out without merge-multiple); IN_DIR is searched recursively.

Usage:
    assemble_wheels.py IN_DIR OUT_DIR [--expect humble,jazzy,...]
"""

import argparse
import base64
import csv
import hashlib
import io
import re
import sys
import zipfile
from collections import defaultdict
from pathlib import Path

PAYLOAD = re.compile(r"^play_launch/dist/([^/]+)/")


def tag_of(name: str) -> str:
    # {dist}-{version}(-{build})?-{python}-{abi}-{platform}.whl
    parts = name[: -len(".whl")].split("-")
    return "-".join(parts[-3:])


def arch_of(tag: str) -> str:
    for arch in ("x86_64", "aarch64"):
        if tag.endswith("_" + arch):
            return arch
    return tag


def record_line(path: str, data: bytes) -> list[str]:
    digest = base64.urlsafe_b64encode(hashlib.sha256(data).digest()).rstrip(b"=")
    return [path, f"sha256={digest.decode()}", str(len(data))]


def read_rows(in_dir: Path):
    rows = defaultdict(list)  # tag -> [(wheel path, distro, {name: (ZipInfo, bytes)})]
    for whl in sorted(in_dir.rglob("*.whl")):
        with zipfile.ZipFile(whl) as z:
            files = {i.filename: (i, z.read(i)) for i in z.infolist() if not i.is_dir()}
        distros = sorted({m.group(1) for n in files if (m := PAYLOAD.match(n))})
        if len(distros) != 1:
            sys.exit(f"error: {whl.name} carries {len(distros)} payloads {distros}, expected one")
        rows[tag_of(whl.name)].append((whl, distros[0], files))
    return rows


def merge(tag: str, members, out_dir: Path) -> tuple[Path, list[str]]:
    first_whl, _, _ = members[0]
    distros = [d for _, d, _ in members]
    if len(set(distros)) != len(distros):
        sys.exit(f"error: {tag}: a distribution appears twice: {distros}")

    merged: dict[str, tuple[zipfile.ZipInfo, bytes]] = {}
    origin: dict[str, str] = {}
    for whl, _, files in members:
        for name, (info, data) in files.items():
            if name.endswith(".dist-info/RECORD"):
                continue
            if name in merged:
                if merged[name][1] != data:
                    sys.exit(
                        f"error: {tag}: {name} differs between {origin[name]} and {whl.parent.name}/{whl.name}; "
                        "rows of one tag must share every file outside play_launch/dist/"
                    )
                continue
            merged[name] = (info, data)
            origin[name] = f"{whl.parent.name}/{whl.name}"

    records = [n for n in (f for _, _, fs in members for f in fs) if n.endswith(".dist-info/RECORD")]
    record_name = records[0]
    rows = [record_line(n, d) for n, (_, d) in sorted(merged.items())]
    rows.append([record_name, "", ""])
    buf = io.StringIO()
    csv.writer(buf, lineterminator="\n").writerows(rows)

    out = out_dir / first_whl.name
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        # dist-info last, RECORD at the very end, as the wheel spec recommends.
        names = sorted(merged, key=lambda n: (".dist-info/" in n, n))
        for name in names:
            info, data = merged[name]
            zi = zipfile.ZipInfo(name, date_time=info.date_time)
            zi.external_attr = info.external_attr
            zi.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(zi, data)
        zi = zipfile.ZipInfo(record_name, date_time=merged[names[0]][0].date_time)
        zi.external_attr = 0o644 << 16
        zi.compress_type = zipfile.ZIP_DEFLATED
        z.writestr(zi, buf.getvalue())
    return out, sorted(distros)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("in_dir", type=Path)
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--expect", default="", help="comma-separated distributions every arch must carry")
    args = ap.parse_args()

    rows = read_rows(args.in_dir)
    if not rows:
        sys.exit(f"error: no wheels in {args.in_dir}")
    args.out_dir.mkdir(parents=True, exist_ok=True)

    by_arch = defaultdict(list)
    for tag, members in sorted(rows.items()):
        out, distros = merge(tag, members, args.out_dir)
        by_arch[arch_of(tag)].extend(distros)
        print(f"{out.name}: {', '.join(distros)}")
        with zipfile.ZipFile(out) as z:
            for i in z.infolist():
                print(f"  {i.file_size:>10}  {i.filename}")

    expect = sorted(d for d in args.expect.split(",") if d)
    if expect:
        bad = False
        for arch, got in sorted(by_arch.items()):
            missing = sorted(set(expect) - set(got))
            extra = sorted(set(got) - set(expect))
            if missing or extra:
                print(f"error: {arch}: missing {missing}, unexpected {extra}", file=sys.stderr)
                bad = True
        if bad:
            sys.exit(1)
        print(f"complete: every arch carries {', '.join(expect)} exactly once")


if __name__ == "__main__":
    main()
