"""Stamp propagation on a recorded run: how many topics carry a stamp first
published on another topic, the edges that implies, and data age.

Usage: python3 stamp_analysis.py <events.jsonl>
"""
import collections
import json
import statistics
import sys

names = {}
pubs = []  # (t, topic_hash, stamp, node)
with open(sys.argv[1]) as f:
    for line in f:
        r = json.loads(line)
        if r.get("r") == "t":
            names[r["h"]] = r["n"]
            continue
        if r.get("r") != "e" or r.get("d") != "pub":
            continue
        s, ns = r.get("s", 0), r.get("ns", 0)
        stamp = None if (s == 0 and ns == 0) else (s, ns)
        pubs.append((r["t"], r["h"], stamp, r["n"]))

pubs.sort()
skip = {h for h, n in names.items() if n in ('/parameter_events', '/rosout')}
pubs = [p for p in pubs if p[1] not in skip]
name = lambda h: names.get(h, hex(h))
first = {}  # stamp -> (t, topic)
last_by_stamp = {}  # stamp -> {topic: t}
per_topic = collections.defaultdict(lambda: {"n": 0, "stamped": 0, "inherited": 0, "ages": []})
edges = collections.defaultdict(list)  # (parent, child) -> latencies ms
origins = collections.Counter()

for t, h, stamp, node in pubs:
    pt = per_topic[h]
    pt["n"] += 1
    if stamp is None:
        continue
    pt["stamped"] += 1
    if stamp not in first:
        first[stamp] = (t, h)
        last_by_stamp[stamp] = {h: t}
        origins[h] += 1
        continue
    t0, h0 = first[stamp]
    seen = last_by_stamp[stamp]
    if h0 != h:
        pt["inherited"] += 1
        pt["ages"].append((t - t0) / 1e6)
        # Immediate parent: the latest earlier publish of this stamp on another topic.
        cands = [(tt, hh) for hh, tt in seen.items() if hh != h]
        if cands:
            tp, hp = max(cands)
            edges[(hp, h)].append((t - tp) / 1e6)
    seen[h] = t

total_topics = len(per_topic)
stamped_topics = [h for h, v in per_topic.items() if v["stamped"] > 0]
inherit_topics = [h for h, v in per_topic.items() if v["stamped"] and v["inherited"] / v["stamped"] >= 0.5]
print(f"publish records: {len(pubs)}; topics published: {total_topics}")
print(f"topics with stamped messages: {len(stamped_topics)}")
print(f"topics where >=50% of stamps were first published on ANOTHER topic: {len(inherit_topics)}")
print(f"origin topics (stamp first seen there): {len([h for h in origins])}")
print()
print("Inherited topics, by median data age (ms) — top 30:")
rows = []
for h in inherit_topics:
    a = per_topic[h]["ages"]
    rows.append((statistics.median(a), max(a), len(a), name(h)))
for med, mx, n, nm in sorted(rows, reverse=True)[:30]:
    print(f"  {med:9.1f} med {mx:9.1f} max  n={n:6d}  {nm}")
print()
print(f"discovered edges: {len(edges)}")
strong = sorted(((len(v), k) for k, v in edges.items()), reverse=True)
for n, (a, b) in strong[:40]:
    v = edges[(a, b)]
    print(f"  {n:6d}  {statistics.median(v):8.2f} ms  {name(a)} -> {name(b)}")
print()
print("Top origin topics (where stamps are born):")
for h, n in origins.most_common(20):
    print(f"  {n:7d}  {name(h)}")
