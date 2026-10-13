"""Node-grounded stamp propagation: an edge A -> N -> B when node N took a
message stamped s on A and later published s on B. Usage: <events.jsonl>"""
import collections, json, statistics, sys
names = {}
recs = []
with open(sys.argv[1]) as f:
    for line in f:
        r = json.loads(line)
        if r.get("r") == "t":
            names[r["h"]] = r["n"]; continue
        if r.get("r") != "e" or (r["s"] == 0 and r["ns"] == 0):
            continue
        recs.append((r["t"], r["d"], r["n"], r["h"], (r["s"], r["ns"])))
recs.sort()
skip = {h for h, n in names.items() if n in ("/parameter_events", "/rosout")}
nm = lambda h: names.get(h, hex(h))
taken = collections.defaultdict(dict)   # node -> {stamp: (topic, t)} latest take
edges = collections.defaultdict(list)    # (A, node, B) -> ms
pub_total = collections.Counter(); pub_inherited = collections.Counter()
first_pub = {}
age = collections.defaultdict(list)
for t, d, node, h, s in recs:
    if h in skip: continue
    if d == "take":
        taken[node][s] = (h, t)
        continue
    pub_total[h] += 1
    if s not in first_pub: first_pub[s] = t
    else: age[h].append((t - first_pub[s]) / 1e6)
    src = taken[node].get(s)
    if src and src[0] != h:
        pub_inherited[h] += 1
        edges[(src[0], node, h)].append((t - src[1]) / 1e6)
topics = [h for h in pub_total]
inh = [h for h in topics if pub_inherited[h] / pub_total[h] >= 0.5]
print(f"stamped topics published: {len(topics)}; >=50% of publishes carry a stamp the node TOOK on another topic: {len(inh)}")
print(f"node-grounded edges: {len(edges)}")
rows = sorted(((len(v), statistics.median(v), max(v), a, n, b) for (a, n, b), v in edges.items()), reverse=True)
for c, med, mx, a, n, b in rows[:45]:
    print(f"  {c:5d}  med {med:7.2f}  max {mx:8.2f} ms  {nm(a)}  --[{n}]-->  {nm(b)}")
# chains: walk from topics with no inherited parents
succ = collections.defaultdict(set); has_parent = set()
for (a, n, b), v in edges.items():
    if len(v) >= 20: succ[a].add(b); has_parent.add(b)
def paths(t, seen):
    if t in seen or not succ.get(t): return [[t]]
    out = []
    for nx in succ[t]:
        for p in paths(nx, seen | {t}): out.append([t] + p)
    return out
print("\nlongest chains (edges seen >= 20 times):")
allp = [p for r in list(succ) if r not in has_parent for p in paths(r, set())]
for p in sorted(allp, key=len, reverse=True)[:6]:
    print("  " + "  ->  ".join(nm(h).split("/")[-1] for h in p), f"({len(p)} topics)")
