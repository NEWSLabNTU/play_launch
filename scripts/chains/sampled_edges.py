"""Same-thread causality: a publish by node N on thread T whose most recent
take on T (by N, within 200 ms) was topic A gives a triggered edge A -> B,
stamp or no stamp. Usage: <events.jsonl>"""
import collections, json, statistics, sys
names = {}; recs = []
with open(sys.argv[1]) as f:
    for line in f:
        r = json.loads(line)
        if r.get("r") == "t": names[r["h"]] = r["n"]; continue
        if r.get("r") == "e": recs.append((r["t"], r["d"], r["n"], r["tid"], r["h"], (r["s"], r["ns"])))
recs.sort()
skip = {h for h, n in names.items() if n in ("/parameter_events", "/rosout", "/diagnostics")}
nm = lambda h: names.get(h, hex(h)).split("/")[-1] if False else names.get(h, hex(h))
last_take = {}   # (node, tid) -> (topic, t, stamp)
pubs = collections.Counter(); covered = collections.Counter(); stamp_kept = collections.Counter()
edges = collections.defaultdict(list)
for t, d, n, tid, h, s in recs:
    if h in skip: continue
    if d == "take":
        last_take[(n, tid)] = (h, t, s); continue
    pubs[h] += 1
    lt = last_take.get((n, tid))
    if lt and lt[0] != h and t - lt[1] <= 200e6:
        covered[h] += 1
        if s != (0, 0) and s == lt[2]: stamp_kept[h] += 1
        edges[(lt[0], h)].append((t - lt[1]) / 1e6)
tot = sum(pubs.values()); cov = sum(covered.values())
print(f"publishes: {tot}; preceded on the same thread by a take of another topic (<=200 ms): {cov} ({100*cov/tot:.0f}%)")
topics_cov = [h for h in pubs if covered[h] / pubs[h] >= 0.5]
print(f"topics: {len(pubs)}; >=50% of publishes triggered by a take: {len(topics_cov)}; of those, stamp carried over: {sum(1 for h in topics_cov if stamp_kept[h] / max(covered[h],1) >= 0.5)}")
strong = [(k, v) for k, v in edges.items() if len(v) >= 20]
print(f"triggered edges seen >=20 times: {len(strong)}")
for k, v in sorted(strong, key=lambda kv: -len(kv[1]))[:25]:
    print(f"  {len(v):5d} med {statistics.median(v):7.2f} ms  {nm(k[0])} -> {nm(k[1])}")
for want in ["/control/trajectory_follower/control_cmd", "/control/command/control_cmd", "/perception/object_recognition/tracking/objects", "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id"]:
    ins = [(len(v), statistics.median(v), nm(a)) for (a, b), v in edges.items() if names.get(b) == want]
    print(f"\ninputs triggering {want}: {sorted(ins, reverse=True)[:4]}  (publishes {sum(pubs[h] for h in pubs if names.get(h)==want)})")
