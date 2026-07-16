#!/usr/bin/env python3
"""Phase 42 study tooling: join the DECLARED causal graph (from
`play_launch check --export-graph`) against the MEASURED interception
summaries (`stats_summary.json` / `frontier_summary.json` from
`play_log/<ts>/interception/`).

Key measurement fact this script compensates for: the interception layer
registers topics under the name each endpoint was CREATED with
(namespace-expanded but possibly pre-remap), while the declared graph uses
wire (post-remap) FQNs. Passing `--record record.json` (same launch
config) lets the script build an alias map from every node's `remaps` so
pre-remap endpoint names are folded into their wire topic. A side effect:
where different endpoints of the same wire topic used different pre-remap
names, we get PARTIAL per-endpoint visibility (reported in the fan-in
section).

Double-count correction (Phase 42 W2 finding): when both the frontier and
stats plugins are enabled, messages whose type carries `header.stamp` emit
TWO `Publish`/`Take` events (one from each plugin: FrontierPlugin fires
only for stamped types, StatsPlugin for all), and the consumer counts both
— so stamped topics show exactly 2x pub/take counts and rates. This script
infers the set of stamped message TYPES from frontier records with a
non-zero stamp and halves the counts of every measured name whose
`msg_type` is in that set (best effort; a stamped type whose stamp was
always zero escapes detection). Disable with --no-halving. The proper fix
is in the interception consumer (dedupe) or the .so (distinct event kind).

Outputs a Markdown report to stdout covering:
  - per-topic declared rate vs measured avg pub rate (delta table, |delta|>20% flagged)
  - Q5: measured truth of the declared-rate monoculture (rate clusters)
  - Q2: fan-in junctions (topics with multiple declared subscribers) with
    measured pub/take counts + per-alias endpoint breakdown where visible
  - frontier stamp status (data-age observability)
  - coverage: declared topics with measured activity; ring-drop counters

Usage:
  p42_join.py --declared declared.json --stats stats_summary.json \
              --frontier frontier_summary.json [--record record.json] \
              [--csv out.csv]
"""

import argparse
import csv
import json
import sys
from collections import defaultdict

FNV_OFFSET = 0xCBF29CE484222325
FNV_PRIME = 0x100000001B3


def fnv1a(s: str) -> int:
    h = FNV_OFFSET
    for b in s.encode():
        h ^= b
        h = (h * FNV_PRIME) & 0xFFFFFFFFFFFFFFFF
    return h


def load(path):
    with open(path) as f:
        return json.load(f)


def expand_topic(name: str, ns: str, node_name: str | None) -> str:
    """ROS name expansion: absolute stays, ~ is node-private, else ns-relative."""
    if name.startswith("/"):
        return name
    ns = ns or ""
    ns = ns.rstrip("/")
    if name.startswith("~"):
        rest = name[1:].lstrip("/")
        base = f"{ns}/{node_name}" if node_name else ns
        return f"{base}/{rest}" if rest else base
    return f"{ns}/{name}"


def build_alias_map(record: dict) -> dict:
    """expanded pre-remap endpoint name -> wire (post-remap) name."""
    aliases = {}

    def add_entity(ns, node_name, remaps):
        for pair in remaps or []:
            if isinstance(pair, (list, tuple)) and len(pair) == 2:
                frm, to = pair
            elif isinstance(pair, str) and ":=" in pair:
                frm, to = pair.split(":=", 1)
            else:
                continue
            if frm.startswith("__"):
                continue
            src = expand_topic(frm, ns, node_name)
            dst = expand_topic(to, ns, node_name)
            if src != dst:
                aliases[src] = dst

    for key in ("node", "lifecycle_node", "container"):
        for n in record.get(key, []) or []:
            add_entity(n.get("namespace") or "", n.get("name") or n.get("exec_name"),
                       n.get("remaps"))
    for n in record.get("load_node", []) or []:
        add_entity(n.get("namespace") or "", n.get("node_name"), n.get("remaps"))
    return aliases


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--declared", required=True)
    ap.add_argument("--stats", required=True)
    ap.add_argument("--frontier", required=True)
    ap.add_argument("--record", help="record.json for remap alias resolution")
    ap.add_argument("--csv", help="optional joined-table CSV output path")
    ap.add_argument(
        "--delta-threshold", type=float, default=20.0,
        help="flag |declared-vs-measured| rate delta above this percent",
    )
    ap.add_argument(
        "--no-halving", action="store_true",
        help="disable the stamped-type double-count correction",
    )
    args = ap.parse_args()

    declared = load(args.declared)
    stats = load(args.stats)
    frontier = load(args.frontier)

    dropped = stats.pop("_events_dropped_total_best_effort", None)
    frontier_dropped = frontier.pop("_events_dropped_total_best_effort", None)

    aliases = {}
    if args.record:
        aliases = build_alias_map(load(args.record))

    # wire topic -> list of measured names contributing to it
    contributors = defaultdict(set)
    for src, dst in aliases.items():
        contributors[dst].add(src)

    # name -> stats record (prefer embedded names; fall back to fnv1a of name)
    name_to_stats = {}
    for h, rec in stats.items():
        if isinstance(rec, dict):
            if rec.get("name"):
                name_to_stats[rec["name"]] = rec
    name_to_frontier = {}
    for h, rec in frontier.items():
        if isinstance(rec, dict) and rec.get("name"):
            name_to_frontier[rec["name"]] = rec

    # ---- stamped-type inference for double-count correction ----
    # A frontier record with a non-zero stamp proves that topic's type has
    # header.stamp, hence its Publish/Take events were emitted by BOTH the
    # frontier and stats plugins (2x counts). Collect those types.
    stamped_types = set()
    if not args.no_halving:
        for rec in frontier.values():
            if (isinstance(rec, dict) and rec.get("msg_type")
                    and (rec["stamp_sec"] > 0 or rec["stamp_nanosec"] > 0)):
                stamped_types.add(rec["msg_type"])

    def correct(rec):
        """Halve counts/rate for names whose msg type is a stamped type."""
        if rec is None or args.no_halving:
            return rec
        if rec.get("msg_type") in stamped_types:
            rec = dict(rec)
            rec["pub_count"] //= 2
            rec["take_count"] //= 2
            rec["avg_pub_rate_hz"] /= 2.0
            rec["_halved"] = True
        return rec

    def raw_stats(name):
        rec = name_to_stats.get(name)
        if rec is None:
            rec = stats.get(str(fnv1a(name)))
        return correct(rec)

    def measured_for(topic_fqn):
        """Aggregate wire-name record + all alias records. Returns
        (aggregate dict or None, per-name breakdown list)."""
        names = [topic_fqn] + sorted(contributors.get(topic_fqn, ()))
        parts = []
        for n in names:
            rec = raw_stats(n)
            if rec:
                parts.append((n, rec))
        if not parts:
            return None, []
        agg = {
            "pub_count": sum(r["pub_count"] for _, r in parts),
            "take_count": sum(r["take_count"] for _, r in parts),
            "duration_ms": max(r["duration_ms"] for _, r in parts),
            # pub rate: take the max avg_pub_rate_hz over contributing names
            # (a wire topic's publisher lives under exactly one name; other
            # names are subscriber-side aliases with pub_count 0)
            "avg_pub_rate_hz": max(r["avg_pub_rate_hz"] for _, r in parts),
        }
        return agg, parts

    def frontier_for(topic_fqn):
        names = [topic_fqn] + sorted(contributors.get(topic_fqn, ()))
        best = None
        for n in names:
            rec = name_to_frontier.get(n) or frontier.get(str(fnv1a(n)))
            if rec and (best is None
                        or (rec["stamp_sec"], rec["stamp_nanosec"])
                        > (best["stamp_sec"], best["stamp_nanosec"])):
                best = rec
        return best

    topics = declared["topics"]
    pub_edges = declared["pub_edges"]
    sub_edges = declared["sub_edges"]

    topic_rate = {}
    for t in topics:
        if t.get("rate_hz") is not None:
            topic_rate[t["fqn"]] = t["rate_hz"]
    for e in pub_edges:
        if e["topic"] not in topic_rate and e.get("rate_hz") is not None:
            topic_rate[e["topic"]] = e["rate_hz"]

    subs_by_topic = defaultdict(list)
    for e in sub_edges:
        subs_by_topic[e["topic"]].append(e)

    # ---------------- joined rate table ----------------
    rows = []
    for fqn, decl_hz in sorted(topic_rate.items()):
        m, _parts = measured_for(fqn)
        meas_hz = m["avg_pub_rate_hz"] if m else None
        delta_pct = (
            (meas_hz - decl_hz) / decl_hz * 100.0
            if (meas_hz is not None and decl_hz) else None
        )
        rows.append({
            "topic": fqn,
            "declared_hz": decl_hz,
            "measured_hz": meas_hz,
            "delta_pct": delta_pct,
            "pub_count": m["pub_count"] if m else 0,
            "take_count": m["take_count"] if m else 0,
            "active_window_ms": m["duration_ms"] if m else 0.0,
            "n_declared_subs": len(subs_by_topic.get(fqn, [])),
        })

    if args.csv and rows:
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

    thr = args.delta_threshold

    print("# Declared vs Measured join")
    print()
    print(f"- declared topics: {len(topics)}; with a declared rate: {len(topic_rate)}")
    print(f"- measured topics (stats): {len(stats)}")
    print(f"- remap aliases resolved: {len(aliases)}"
          + ("" if args.record else " (no --record: alias folding disabled)"))
    print(f"- stamped msg types (double-count halving applied): "
          f"{'DISABLED' if args.no_halving else len(stamped_types)}")
    print(f"- ring-drop counter (stats file): {dropped}")
    print(f"- ring-drop counter (frontier file): {frontier_dropped}")
    print()

    # coverage
    active, inactive = [], []
    for t in topics:
        m, _ = measured_for(t["fqn"])
        if m and (m["pub_count"] > 0 or m["take_count"] > 0):
            active.append(t["fqn"])
        else:
            inactive.append(t["fqn"])
    print(f"## Coverage: {len(active)}/{len(topics)} declared topics had measured activity")
    for fqn in inactive:
        print(f"  - inactive: {fqn}")
    print()

    # rate delta table
    print(f"## Rate deltas (declared rate topics, |delta| > {thr:.0f}% flagged)")
    print()
    print("| topic | declared Hz | measured Hz | delta % | pubs | takes | window s | flag |")
    print("|---|---|---|---|---|---|---|---|")
    flagged = 0
    for r in sorted(rows, key=lambda r: -(abs(r["delta_pct"]) if r["delta_pct"] is not None else 1e9)):
        mh = f"{r['measured_hz']:.2f}" if r["measured_hz"] is not None else "-"
        dp = f"{r['delta_pct']:+.1f}" if r["delta_pct"] is not None else "no data"
        flag = ""
        if r["delta_pct"] is None:
            flag = "NO-DATA"
        elif abs(r["delta_pct"]) > thr:
            flag = "DELTA"
            flagged += 1
        win = f"{r['active_window_ms']/1000.0:.1f}"
        print(f"| {r['topic']} | {r['declared_hz']} | {mh} | {dp} | "
              f"{r['pub_count']} | {r['take_count']} | {win} | {flag} |")
    print()
    print(f"flagged (|delta| > {thr:.0f}%): {flagged}/{len(rows)}")
    print()

    # Q5: cluster measured rates by declared rate
    decl_values = defaultdict(list)
    for r in rows:
        decl_values[r["declared_hz"]].append(r)
    print("## Q5: declared-rate clusters vs measured")
    print()
    for hz in sorted(decl_values):
        grp = decl_values[hz]
        meas = sorted(g["measured_hz"] for g in grp if g["measured_hz"] is not None)
        if meas:
            print(f"- declared {hz} Hz: {len(grp)} topics; measured "
                  f"[{', '.join(f'{m:.1f}' for m in meas)}] Hz "
                  f"({len(meas)} with data)")
        else:
            print(f"- declared {hz} Hz: {len(grp)} topics; no measured data")
    print()

    # Q2: fan-in junctions
    print("## Q2: fan-in junctions (topics with >1 declared subscriber)")
    print()
    print("Measured stats are aggregated per topic NAME (pub_count/take_count),")
    print("not per subscription endpoint. take_count under one name is the SUM")
    print("over all subscribers that used that name. However, remap aliasing")
    print("splits some endpoints across distinct pre-remap names — those rows")
    print("show a per-name breakdown, giving partial per-endpoint sampling")
    print("evidence. Endpoints sharing one name remain unattributable.")
    print()
    print("| topic | n subs | subs (causal/state) | pubs | takes | takes/pubs | expected if all-take |")
    print("|---|---|---|---|---|---|---|")
    breakdowns = []
    for fqn, subs in sorted(subs_by_topic.items(), key=lambda kv: -len(kv[1])):
        if len(subs) < 2:
            continue
        m, parts = measured_for(fqn)
        pubs = m["pub_count"] if m else 0
        takes = m["take_count"] if m else 0
        n_causal = sum(1 for s in subs if s["causal"])
        n_state = sum(1 for s in subs if s["state"])
        ratio = f"{takes/pubs:.2f}" if pubs else "-"
        print(f"| {fqn} | {len(subs)} | {n_causal}c/{n_state}s | {pubs} | {takes} | {ratio} | {len(subs)} |")
        if len(parts) > 1:
            breakdowns.append((fqn, parts))
    print()
    if breakdowns:
        print("### Per-name endpoint breakdowns (alias-split fan-in topics)")
        print()
        for fqn, parts in breakdowns:
            print(f"- {fqn}:")
            for n, rec in parts:
                print(f"    - {n}: pubs={rec['pub_count']} takes={rec['take_count']} "
                      f"rate={rec['avg_pub_rate_hz']:.2f} Hz")
        print()

    # frontier
    print("## Frontier stamps (data-age observability)")
    print()
    nonzero, zero = [], []
    for t in topics:
        fr = frontier_for(t["fqn"])
        if fr is None:
            continue
        if fr["stamp_sec"] > 0 or fr["stamp_nanosec"] > 0:
            nonzero.append((t["fqn"], fr))
        else:
            zero.append((t["fqn"], fr))
    print(f"- declared topics with a frontier record: {len(nonzero) + len(zero)}")
    print(f"- non-zero stamps: {len(nonzero)}; zero stamps: {len(zero)}")
    for fqn, fr in sorted(nonzero):
        print(f"  - {fqn}: stamp={fr['stamp_sec']}.{fr['stamp_nanosec']:09d} "
              f"(events={fr['event_count']})")
    print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
