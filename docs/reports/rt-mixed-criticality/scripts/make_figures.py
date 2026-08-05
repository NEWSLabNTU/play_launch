#!/usr/bin/env python3
"""Turn a `just ab` run pair into the report's figures and numbers.

Every number in the report comes from here. Nothing is typed by hand into the
prose, so a re-run cannot leave a stale figure next to a fresh sentence.

Usage: make_figures.py <ab-output-dir> <report-dir>
  <ab-output-dir> is examples/rt_av_demo/out, holding trace-off/ and trace-on/.
"""
import collections
import json
import pathlib
import sys

WARMUP = 200
DEADLINE = 60.0
WINDOW_MS = 400

SAFETY = ["/safety/lidar_driver", "/safety/obstacle_detector", "/safety/brake_controller"]
OTHER = ["/planning/path_planner", "/telemetry/telemetry_logger", "/tools/rosbag_recorder"]

C_OFF, C_ON, C_DEAD, C_GRID, C_TEXT, C_MUTE = (
    "#c2410c", "#0f766e", "#b91c1c", "#e0dcd6", "#1a1917", "#a8a29e",
)


def run_dir(base, mode):
    return base / f"trace-{mode}/play_log/latest"


def chain(base, mode):
    """Steady-state chain samples measured by brake_controller itself."""
    p = run_dir(base, mode) / "node/brake_controller/out"
    out = []
    for line in p.open():
        if not line.startswith("CHAIN "):
            continue
        kv = dict(x.split("=", 1) for x in line.split()[1:])
        if int(kv["seq"]) <= WARMUP:
            continue
        out.append((int(kv["seq"]), float(kv["latency_ms"])))
    if not out:
        sys.exit(f"no steady-state samples in {p}")
    return out


def trace(base, mode):
    return json.loads((run_dir(base, mode) / "interception/trace.json").read_text())


def publishes(ev):
    rows = {x["pid"]: x["args"]["name"] for x in ev if x["ph"] == "M"}
    return [(rows[x["pid"]], x["ts"] / 1000.0) for x in ev if x.get("cat") == "publish"]


def stats(samples):
    lat = sorted(v for _, v in samples)
    n = len(lat)
    q = lambda f: lat[min(int(n * f), n - 1)]
    missed = sum(1 for v in lat if v > DEADLINE)
    return {
        "p50": q(0.50), "p99": q(0.99), "max": lat[-1],
        "missed": missed, "total": n, "miss_pct": 100.0 * missed / n,
    }


# --------------------------------------------------------------------------
# figure 1 — per-frame latency
# --------------------------------------------------------------------------
def fig_latency(data, out):
    W, H, L, R, T, B = 900, 300, 56, 96, 16, 40
    iw, ih = W - L - R, H - T - B
    ymax = 120.0
    seqs = [s for m in ("off", "on") for s, _ in data["chain"][m]]
    x0, x1 = min(seqs), max(seqs)
    X = lambda s: L + (s - x0) / (x1 - x0) * iw
    Y = lambda v: T + ih - min(v, ymax) / ymax * ih

    p = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" font-family="sans-serif">']
    p.append(f'<rect x="{L}" y="{T}" width="{iw}" height="{Y(DEADLINE)-T:.1f}" '
             f'fill="{C_DEAD}" opacity="0.06"/>')
    for v in range(0, 121, 30):
        p.append(f'<line x1="{L}" x2="{L+iw}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" '
                 f'stroke="{C_GRID}"/>')
        p.append(f'<text x="{L-9}" y="{Y(v)+4:.1f}" font-size="11" fill="{C_MUTE}" '
                 f'text-anchor="end">{v}</text>')
    p.append(f'<line x1="{L}" x2="{L+iw}" y1="{Y(DEADLINE):.1f}" y2="{Y(DEADLINE):.1f}" '
             f'stroke="{C_DEAD}" stroke-width="2" stroke-dasharray="5 4"/>')
    p.append(f'<text x="{L+iw+9}" y="{Y(DEADLINE)+4:.1f}" font-size="11" fill="{C_DEAD}" '
             f'font-weight="bold">{DEADLINE:.0f} ms deadline</text>')

    for mode, color, w in (("off", C_OFF, 1.0), ("on", C_ON, 1.6)):
        pts = " ".join(f"{X(s):.1f},{Y(v):.1f}" for s, v in data["chain"][mode])
        p.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="{w}"/>')
        p.append(f'<text x="{L+iw+9}" y="{Y(data["stats"][mode]["p50"])+4:.1f}" font-size="11" '
                 f'fill="{color}" font-weight="bold">RT {mode}</text>')

    p.append(f'<text x="{L+iw/2}" y="{H-8}" font-size="11" fill="{C_MUTE}" '
             f'text-anchor="middle">frame (50 Hz)</text>')
    p.append(f'<text x="16" y="{T+ih/2}" font-size="11" fill="{C_MUTE}" text-anchor="middle" '
             f'transform="rotate(-90 16 {T+ih/2})">end-to-end latency (ms)</text>')
    p.append("</svg>")
    out.write_text("\n".join(p))


# --------------------------------------------------------------------------
# figure 2 — publish timeline
# --------------------------------------------------------------------------
def fig_timeline(data, out):
    W, L, R = 900, 170, 20
    iw = W - L - R
    rows = [r for r in SAFETY + OTHER if data["events"]["off"].get(r) or data["events"]["on"].get(r)]
    rh, gap, blockgap = 13, 5, 30
    p, y = [], 26

    body = []
    for mode, label, color in (("off", "RT OFF", C_OFF), ("on", "RT ON", C_ON)):
        body.append(f'<text x="{L}" y="{y-9}" font-size="11" fill="{color}" '
                    f'font-weight="bold" letter-spacing="1">{label}</text>')
        for r in rows:
            safety = r.startswith("/safety/")
            name = r.split("/")[-1]
            weight = "bold" if safety else "normal"
            fill = C_OFF if safety else C_MUTE
            body.append(f'<text x="{L-10}" y="{y+rh-3}" font-size="11" fill="{fill}" '
                        f'font-weight="{weight}" text-anchor="end">{name}</text>')
            body.append(f'<rect x="{L}" y="{y}" width="{iw}" height="{rh}" rx="2" '
                        f'fill="{C_GRID}" opacity="0.5"/>')
            for t in data["events"][mode].get(r, []):
                body.append(f'<rect x="{L + t/WINDOW_MS*iw - 1:.1f}" y="{y}" width="2.4" '
                            f'height="{rh}" rx="1" fill="{color if safety else C_MUTE}"/>')
            y += rh + gap
        y += blockgap

    H = y - blockgap + 16
    p.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" font-family="sans-serif">')
    for ms in range(0, WINDOW_MS + 1, 100):
        x = L + ms / WINDOW_MS * iw
        p.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="18" y2="{y-blockgap-gap}" '
                 f'stroke="{C_GRID}"/>')
        p.append(f'<text x="{x:.1f}" y="{H-2}" font-size="11" fill="{C_MUTE}" '
                 f'text-anchor="middle">{ms} ms</text>')
    p.extend(body)
    p.append("</svg>")
    out.write_text("\n".join(p))


# --------------------------------------------------------------------------
# figure 3 — throughput by band
# --------------------------------------------------------------------------
def fig_throughput(data, out):
    W, L, R = 900, 120, 96
    iw = W - L - R
    bands = sorted(data["throughput"]["off"], key=lambda b: -data["throughput"]["off"][b])
    mx = max(max(data["throughput"][m].get(b, 0) for m in ("off", "on")) for b in bands)
    bh, gap, grp = 12, 4, 32
    p, y = [], 14
    body = []
    for b in bands:
        weight = "bold" if b == "safety" else "normal"
        body.append(f'<text x="{L-10}" y="{y+bh+4}" font-size="11" fill="{C_TEXT}" '
                    f'font-weight="{weight}" text-anchor="end">{b}</text>')
        for mode, color, op in (("off", C_OFF, "0.55"), ("on", C_ON, "1")):
            v = data["throughput"][mode].get(b, 0)
            w = max(v / mx * iw, 1)
            body.append(f'<rect x="{L}" y="{y}" width="{w:.1f}" height="{bh}" rx="2" '
                        f'fill="{color}" opacity="{op}"/>')
            body.append(f'<text x="{L+w+7:.1f}" y="{y+bh-1}" font-size="10" '
                        f'fill="{C_MUTE}">{v}</text>')
            y += bh + gap
        a, c = data["throughput"]["off"][b], data["throughput"]["on"].get(b, 0)
        d = 100.0 * (c - a) / a
        body.append(f'<text x="{W-4}" y="{y-bh-4}" font-size="11" '
                    f'fill="{C_ON if d >= 0 else C_OFF}" font-weight="bold" '
                    f'text-anchor="end">{d:+.0f}%</text>')
        y += grp - bh - gap
    p.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {y}" font-family="sans-serif">')
    p.extend(body)
    p.append("</svg>")
    out.write_text("\n".join(p))


# --------------------------------------------------------------------------
def main():
    base = pathlib.Path(sys.argv[1])
    report = pathlib.Path(sys.argv[2])
    figs = report / "figures"
    figs.mkdir(parents=True, exist_ok=True)

    data = {"chain": {}, "events": {}, "throughput": {}, "stats": {}}
    for mode in ("off", "on"):
        data["chain"][mode] = chain(base, mode)
        data["stats"][mode] = stats(data["chain"][mode])

        pubs = publishes(trace(base, mode))
        t0 = min(t for _, t in pubs)
        start = t0 + (max(t for _, t in pubs) - t0) * 0.5
        win = collections.defaultdict(list)
        for name, t in pubs:
            if start <= t < start + WINDOW_MS:
                win[name].append(round(t - start, 3))
        data["events"][mode] = dict(win)

        bands = collections.Counter()
        for name, _ in pubs:
            bands[name.split("/")[1]] += 1
        data["throughput"][mode] = dict(bands)

    fig_latency(data, figs / "latency.svg")
    fig_timeline(data, figs / "timeline.svg")
    fig_throughput(data, figs / "throughput.svg")

    # Numbers for the prose. Typst reads this, so no figure and no sentence
    # can disagree with the run they came from.
    (report / "data.json").write_text(json.dumps(
        {"stats": data["stats"], "throughput": data["throughput"],
         "deadline_ms": DEADLINE, "warmup": WARMUP, "window_ms": WINDOW_MS},
        indent=2))

    for m in ("off", "on"):
        s = data["stats"][m]
        print(f"RT {m:3} p50={s['p50']:6.1f} p99={s['p99']:7.1f} max={s['max']:7.1f} "
              f"missed={s['missed']}/{s['total']} ({s['miss_pct']:.0f}%)")


if __name__ == "__main__":
    main()
