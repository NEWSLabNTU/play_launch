// HealthView component — the landing page: what is wrong with this run now,
// what was wrong recently, and what play_launch itself costs.
//
// One snapshot per change (at most once a second) arrives over SSE
// (`/api/health/stream`); a plain fetch of `/api/health` paints first and is
// the only place a 404/409 status (and its error text) is visible.

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useMemo } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import {
    currentView, selectedNode, nodePanelOpen, activeTab, graphSelectedElement,
    nodes, getNodeByKey, memberFqn, healthIssueCounts, applyHealthResponse,
    entitiesFilterRequest,
} from '../store.js';

const html = htm.bind(h);

const UNAVAILABLE = 'Health is not available for this run.';

const SEVERITY_RANK = { error: 0, warning: 1, info: 2 };

/** Collapse "Recently resolved" by default when it holds more than this. */
const RESOLVED_COLLAPSE_AT = 5;

// --- formatting -----------------------------------------------------------

/** crash_loop -> "crash loop". */
function humanKind(kind) {
    return (kind || '').replace(/_/g, ' ');
}

/** A duration in ms as "2m 10s", "45s", "1h 5m", "3d 2h". */
function fmtDuration(ms) {
    if (ms == null || !isFinite(ms)) return '—';
    let s = Math.max(0, Math.floor(ms / 1000));
    if (s < 60) return s + 's';
    const d = Math.floor(s / 86400); s -= d * 86400;
    const hr = Math.floor(s / 3600); s -= hr * 3600;
    const m = Math.floor(s / 60); s -= m * 60;
    if (d > 0) return `${d}d ${hr}h`;
    if (hr > 0) return `${hr}h ${m}m`;
    return `${m}m ${s}s`;
}

const UNITS = ['B', 'KiB', 'MiB', 'GiB', 'TiB'];

function fmtBytes(b) {
    if (b == null || !isFinite(b)) return '—';
    let i = 0;
    let v = b;
    while (v >= 1024 && i < UNITS.length - 1) { v /= 1024; i++; }
    return (i === 0 ? Math.round(v) : v.toFixed(v < 10 ? 2 : 1)) + ' ' + UNITS[i];
}

function fmtPct(p) {
    if (p == null || !isFinite(p)) return '—';
    return p < 10 ? p.toFixed(1) : Math.round(p).toString();
}

// --- links ----------------------------------------------------------------

/** Resolve a link target to a member key in the node store, or null.
 * The target may be a canonical member id, a member's display name, or a
 * ROS node FQN (a composable inside a container). */
function resolveMember(target) {
    if (!target) return null;
    const direct = getNodeByKey(target);
    if (direct) return direct.id || direct.name;
    for (const n of nodes.value.values()) {
        const fqn = memberFqn(n.id || n.name);
        if (fqn === target || fqn.replace(/#\d+$/, '') === target) return n.id || n.name;
    }
    return null;
}

function followLink(link) {
    if (!link || !link.view) return;
    if (link.view === 'nodes' && link.target) {
        const key = resolveMember(link.target);
        if (key) {
            selectedNode.value = key;
            nodePanelOpen.value = true;
            activeTab.value = activeTab.peek() || 'stderr';
        }
    } else if (link.view === 'entities' && link.target) {
        entitiesFilterRequest.value = link.target;
    }
    if (link.view !== 'graph') graphSelectedElement.value = null;
    currentView.value = link.view;
}

// --- startup timeline -----------------------------------------------------

/** Poll `/api/startup` every 2 s until startup completes, then every 30 s. */
function useStartupTimeline() {
    const [t, setT] = useState(null);
    useEffect(() => {
        let live = true;
        let timer = null;
        const poll = async () => {
            let done = false;
            try {
                const r = await fetch('/api/startup');
                if (r.ok) {
                    const body = await r.json();
                    if (!live) return;
                    setT(body);
                    done = body.complete_ms != null;
                }
            } catch (_) { /* retried below */ }
            if (live) timer = setTimeout(poll, done ? 30000 : 2000);
        };
        poll();
        return () => { live = false; if (timer) clearTimeout(timer); };
    }, []);
    return t;
}

const KIND_TAG = { node: 'node', executable: 'exec', container: 'cont', composable: 'comp' };

function fmtMs(ms) {
    if (ms == null) return '—';
    return ms < 10000 ? (ms / 1000).toFixed(2) + 's' : (ms / 1000).toFixed(1) + 's';
}

/** Segments of one member's bar, as [from, to, class, title]. */
function startupSegments(r, end) {
    const seg = [];
    if (r.kind === 'composable') {
        if (r.load_started_ms != null) {
            const to = r.loaded_ms ?? r.failed_ms ?? end;
            seg.push([r.load_started_ms, to, r.loaded_ms != null ? 'st-load' : (r.failed_ms != null ? 'st-failed' : 'st-wait'),
                'loading ' + fmtMs(r.load_started_ms) + ' → ' + (r.loaded_ms != null ? fmtMs(r.loaded_ms) : 'not loaded')]);
        }
    } else if (r.spawned_ms != null && (r.inactive_ms != null || r.activates)) {
        // A lifecycle node: up, then configured, then active.
        const up = Math.min(r.spawned_ms, r.first_endpoint_ms ?? r.spawned_ms);
        const upTo = r.first_endpoint_ms ?? r.inactive_ms ?? up;
        seg.push([up, upTo, 'st-start', 'process start to first endpoint']);
        if (r.inactive_ms != null) {
            seg.push([upTo, r.inactive_ms, 'st-config', 'configured at ' + fmtMs(r.inactive_ms)]);
        }
        if (r.active_ms != null) {
            seg.push([r.inactive_ms ?? upTo, r.active_ms, 'st-activate', 'active at ' + fmtMs(r.active_ms)]);
        } else if (r.activates && r.failed_ms == null) {
            seg.push([r.inactive_ms ?? upTo, end, 'st-wait', r.inactive_ms != null ? 'inactive, not activated' : 'not configured']);
        }
    } else if (r.spawned_ms != null) {
        if (r.kind === 'node' && r.first_endpoint_ms != null) {
            // The actor reports a spawn after its post-fork setup, and a fast
            // node can create its first endpoint inside that window.
            const from = Math.min(r.spawned_ms, r.first_endpoint_ms);
            seg.push([from, r.first_endpoint_ms, 'st-start',
                'process start to first endpoint: ' + fmtMs(r.first_endpoint_ms - from)]);
            if (r.first_publish_ms != null && r.first_publish_ms > r.first_endpoint_ms) {
                seg.push([r.first_endpoint_ms, r.first_publish_ms, 'st-wired',
                    'first endpoint to first publish: ' + fmtMs(r.first_publish_ms - r.first_endpoint_ms)]);
            }
        } else if (r.ready_ms == null && r.failed_ms == null) {
            seg.push([r.spawned_ms, end, 'st-wait', 'spawned at ' + fmtMs(r.spawned_ms) + ', not ready']);
        } else {
            seg.push([r.spawned_ms, r.spawned_ms, 'st-start', 'spawned at ' + fmtMs(r.spawned_ms)]);
        }
    }
    return seg;
}

function StartupTimeline() {
    const t = useStartupTimeline();
    const [order, setOrder] = useState('spawn');
    const [open, setOpen] = useState(true);
    if (!t || !t.rows || t.rows.length === 0) return null;

    const end = Math.max(t.complete_ms ?? t.now_ms, 1);
    const span = end * 1.02;
    const pct = (ms) => Math.min(100, Math.max(0, (ms / span) * 100));
    const rows = t.rows.slice();
    if (order === 'ready') {
        const key = (r) => r.ready_ms ?? (r.failed_ms != null ? -1 : Infinity);
        rows.sort((a, b) => key(b) - key(a));
    }
    const last = t.rows.find(r => r.id === t.last_ready);
    const pending = t.rows.filter(r => r.ready_ms == null && r.failed_ms == null).length;
    const ticks = [];
    const step = [100, 250, 500, 1000, 2000, 5000, 10000, 20000, 30000, 60000, 120000]
        .find(s => end / s <= 8) || 300000;
    for (let x = 0; x <= end; x += step) ticks.push(x);

    return html`
        <section class="hl-section">
            <h3 class="hl-heading hl-toggle" onClick=${() => setOpen(!open)}>
                <span class="hl-caret">${open ? '▾' : '▸'}</span>
                Startup${' '}
                <span class="hl-muted hl-heading-count">
                    ${t.complete_ms != null
                        ? html`complete at <b>${fmtMs(t.complete_ms)}</b>${last ? html`; last ready: <b>${last.name}</b> at ${fmtMs(last.ready_ms)}` : ''}`
                        : html`in progress, ${fmtMs(t.now_ms)}; ${pending} not ready`}
                </span>
            </h3>
            ${open && html`
                <div class="st-controls">
                    <label>order <select value=${order} onChange=${(e) => setOrder(e.target.value)}>
                        <option value="spawn">by spawn</option>
                        <option value="ready">last ready first</option>
                    </select></label>
                    <span class="st-legend"><i class="st-start"></i>${t.intercepting ? 'start → node up' : 'spawned'}</span>
                    ${t.intercepting && html`<span class="st-legend"><i class="st-wired"></i>up → first publish</span>`}
                    <span class="st-legend"><i class="st-load"></i>composable load</span>
                    ${t.rows.some(r => r.inactive_ms != null || r.activates) && html`<span class="st-legend"><i class="st-config"></i>configure</span><span class="st-legend"><i class="st-activate"></i>activate</span>`}
                    <span class="st-legend"><i class="st-wait"></i>not ready</span>
                    <span class="st-legend"><i class="st-failed"></i>failed</span>
                    ${!t.intercepting && html`<span class="hl-muted">nodes count as ready at spawn; --interception on shows when each node came up</span>`}
                </div>
                <div class="st-chart">
                    <div class="st-row st-axis">
                        <div class="st-label"></div>
                        <div class="st-track">
                            ${ticks.map(x => html`<span class="st-tick" style=${{ left: pct(x) + '%' }}>${fmtMs(x)}</span>`)}
                        </div>
                    </div>
                    ${rows.map(r => html`
                        <div key=${r.id} class=${'st-row' + (r.id === t.last_ready ? ' st-last' : '')}>
                            <div class="st-label" title=${r.id + ' (' + r.kind + (r.container ? ' in ' + r.container : '') + ')'}>
                                <span class=${'st-kind st-kind-' + r.kind}>${KIND_TAG[r.kind] || '?'}</span>${r.name}
                            </div>
                            <div class="st-track">
                                ${t.complete_ms != null && html`<span class="st-complete" style=${{ left: pct(t.complete_ms) + '%' }}></span>`}
                                ${startupSegments(r, end).map(([a, b, cls, title], i) => html`
                                    <span key=${i} class=${'st-seg ' + cls} title=${r.name + ': ' + title}
                                        style=${{ left: pct(a) + '%', width: Math.max(0.3, pct(b) - pct(a)) + '%' }}></span>`)}
                                ${r.first_publish_ms != null && html`<span class="st-pub" title=${'first publish at ' + fmtMs(r.first_publish_ms)}
                                    style=${{ left: pct(r.first_publish_ms) + '%' }}></span>`}
                                ${r.failed_ms != null && html`<span class="st-fail" title=${'failed at ' + fmtMs(r.failed_ms) + (r.error ? ': ' + r.error : '')}
                                    style=${{ left: pct(r.failed_ms) + '%' }}>✕</span>`}
                            </div>
                            <div class="st-when">${r.ready_ms != null ? fmtMs(r.ready_ms) : (r.failed_ms != null ? 'failed' : 'waiting')}</div>
                        </div>
                    `)}
                </div>
            `}
        </section>
    `;
}

// --- pieces ---------------------------------------------------------------

function SeverityBadge({ severity }) {
    return html`<span class=${'hl-sev hl-sev-' + severity}>${severity}</span>`;
}

function Subject({ subject }) {
    if (!subject) return html`<span class="hl-muted">—</span>`;
    return html`
        <span class="hl-subject" title=${subject.kind + ': ' + subject.name}>
            <span class="hl-subject-kind">${subject.kind}</span>${subject.name}
        </span>
    `;
}

function IssueRow({ issue, now }) {
    const link = issue.link;
    return html`
        <tr class=${'hl-row hl-row-' + issue.severity}>
            <td><${SeverityBadge} severity=${issue.severity} /></td>
            <td class="hl-kind">${humanKind(issue.kind)}</td>
            <td class="hl-subject-cell"><${Subject} subject=${issue.subject} /></td>
            <td class="hl-title">${issue.title}</td>
            <td class="hl-detail" title=${issue.detail || ''}>${issue.detail || ''}</td>
            <td class="hl-since" title=${new Date(issue.opened_ms).toLocaleString()}>
                ${fmtDuration(now - issue.opened_ms)}
            </td>
            <td class="hl-link">
                ${link && link.view && html`
                    <button class="hl-show" title=${'Open ' + link.view + (link.target ? ': ' + link.target : '')}
                        onClick=${() => followLink(link)}>Show</button>`}
            </td>
        </tr>
    `;
}

function ResolvedRow({ issue, now }) {
    const closed = issue.closed_ms ?? issue.last_seen_ms;
    const lasted = closed != null ? closed - issue.opened_ms : null;
    return html`
        <li class="hl-resolved-row">
            <span class="hl-kind">${humanKind(issue.kind)}</span>
            <${Subject} subject=${issue.subject} />
            <span class="hl-resolved-title">${issue.title}</span>
            <span class="hl-muted hl-resolved-when">
                ${closed != null ? `resolved ${fmtDuration(now - closed)} ago` : 'resolved'}
                ${lasted != null ? ` (lasted ${fmtDuration(lasted)})` : ''}
            </span>
        </li>
    `;
}

/** Log activity per member: rate, ERROR/WARN per minute, and the most
 *  frequent message signatures (repeats collapsed, counted). */
function Logs({ logs }) {
    const [open, setOpen] = useState(null);
    if (!logs || logs.length === 0) return null;
    return html`
        <section class="hl-section">
            <h3 class="hl-heading">Logs <span class="hl-muted hl-heading-count">(busiest first)</span></h3>
            <table class="hl-table hl-logs">
                <thead><tr>
                    <th>member</th><th class="hl-num">lines/s</th><th class="hl-num">bytes/s</th>
                    <th class="hl-num">errors/min</th><th class="hl-num">warnings/min</th><th>most frequent</th>
                </tr></thead>
                <tbody>
                    ${logs.map(l => {
                        const top = (l.signatures || [])[0];
                        const expanded = open === l.member;
                        return html`
                            <tr class="hl-row" key=${l.member} onClick=${() => setOpen(expanded ? null : l.member)}>
                                <td class="hl-subject">${l.name}</td>
                                <td class="hl-num">${(l.lines_per_s || 0).toFixed(1)}</td>
                                <td class="hl-num">${fmtBytes(l.bytes_per_s || 0)}/s${l.unread_bytes > 0 ? html` <span class="hl-muted" title="bytes counted but not read: arriving faster than the watcher reads">(${fmtBytes(l.unread_bytes)} unread)</span>` : ''}</td>
                                <td class=${'hl-num' + (l.errors_per_min > 0 ? ' hl-count-err' : '')}>${l.errors_per_min}</td>
                                <td class=${'hl-num' + (l.warnings_per_min > 0 ? ' hl-count-warn' : '')}>${l.warnings_per_min}</td>
                                <td class="hl-detail" title=${top ? top.example : ''}>${top ? `${top.example} (×${top.count})` : ''}</td>
                            </tr>
                            ${expanded && (l.signatures || []).map((s, i) => html`
                                <tr class="hl-sig-row" key=${l.member + ':' + i}>
                                    <td></td>
                                    <td colspan="4" class="hl-muted">${s.level} ×${s.count}</td>
                                    <td class="hl-sig-example">${s.example}</td>
                                </tr>`)}
                        `;
                    })}
                </tbody>
            </table>
        </section>
    `;
}

/** Members' memory and descriptors, with their trend over the window. */
function Resources({ res }) {
    const rows = res.members || [];
    if (rows.length === 0) return null;
    const win = Math.round(res.window_s / 60);
    const trend = (f, fmt) => {
        if (!f) return html`<span class="hl-muted">—</span>`;
        const steady = f.r2 >= 0.8;
        return html`<span class=${steady && f.per_min > 0 ? 'hl-trend-up' : 'hl-muted'}
            title=${'r² ' + f.r2.toFixed(2) + ' over ' + Math.round(f.span_s) + ' s'}>${fmt(f.per_min)}/min</span>`;
    };
    const mb = (b) => (b / 1048576).toFixed(b >= 10485760 ? 0 : 1) + ' MiB';
    const signed = (v, text) => (v > 0 ? '+' : '') + text;
    return html`
        <section class="hl-section">
            <h3 class="hl-heading">Resources <span class="hl-muted hl-heading-count">(fastest-growing first; trend over ${win > 0 ? win + ' min' : res.window_s + ' s'})</span></h3>
            <table class="hl-table">
                <thead><tr>
                    <th>member</th><th class="hl-num">RSS</th><th class="hl-num">RSS trend</th>
                    <th class="hl-num">fds</th><th class="hl-num">fd trend</th><th class="hl-num">threads</th>
                    <th class="hl-num">cpu throttled</th><th class="hl-num">memory.high</th>
                </tr></thead>
                <tbody>
                    ${rows.map(m => html`
                        <tr class="hl-row" key=${m.name}>
                            <td>${m.name}</td>
                            <td class="hl-num">${fmtBytes(m.rss_bytes)}</td>
                            <td class="hl-num">${trend(m.rss_trend, v => signed(v, mb(v)))}</td>
                            <td class="hl-num">${m.fds}</td>
                            <td class="hl-num">${trend(m.fd_trend, v => signed(v, v.toFixed(1)))}</td>
                            <td class="hl-num">${m.threads}</td>
                            <td class="hl-num">${m.cpu_throttled_percent != null ? m.cpu_throttled_percent.toFixed(0) + '%' : '—'}</td>
                            <td class="hl-num">${m.memory_high_events ?? '—'}</td>
                        </tr>`)}
                </tbody>
            </table>
        </section>
    `;
}

/** The parameter probe (launched values against running ones) and the
 * time base the nodes run on. */
function Params({ p }) {
    const nodes = p.nodes || [];
    const answered = nodes.filter(n => !n.error);
    const drift = answered.reduce((a, n) => a + n.drift.length, 0);
    const ignored = answered.reduce((a, n) => a + n.ignored.length, 0);
    const checked = answered.reduce((a, n) => a + n.checked, 0);
    const sim = answered.filter(n => n.use_sim_time === true).length;
    const wall = answered.filter(n => n.use_sim_time === false).length;
    const c = p.clock;
    const rows = [
        ['probed', html`${answered.length} of ${nodes.length} nodes answered, ${checked} launched values checked${' '}
            <span class="hl-muted">(${new Date(p.probed_ms).toLocaleTimeString()}; again every 10 min)</span>`],
        ['differ', html`<b class=${drift ? 'hl-count-warning' : ''}>${drift}</b> running values differ from the launch;${' '}
            <b>${ignored}</b> launched values the nodes never declared`],
        ...((p.broadcast || []).length ? [['broadcast', html`<span class="hl-muted" title="Set for many nodes, declared by few: not counted above">
            ${p.broadcast.slice(0, 6).map(([n, c]) => `${n} (${c})`).join(', ')}${p.broadcast.length > 6 ? `, and ${p.broadcast.length - 6} more` : ''}</span>`]] : []),
        ['time base', sim && wall ? html`<b class="hl-count-warning">mixed</b>: ${sim} on sim time, ${wall} on wall clock`
            : sim ? html`${sim} node(s) on sim time` : html`wall clock (${wall} nodes)`],
        ['/clock', c ? html`${c.value_s.toFixed(3)} s, ${Math.round(c.rate_hz)} Hz, last ${c.since_last_s.toFixed(1)} s ago${c.backward_jumps ? html`, <b class="hl-count-warning">went back ${c.backward_jumps}×</b>` : ''}`
            : p.clock_published === false ? html`<span class="hl-muted">not published</span>`
            : p.clock_published ? html`published <span class="hl-muted">(value needs --interception on)</span>` : html`<span class="hl-muted">—</span>`],
    ];
    const silent = nodes.filter(n => n.error);
    return html`
        <section class="hl-section">
            <h3 class="hl-heading">Parameters and time</h3>
            <table class="hl-kv"><tbody>
                ${rows.map(([k, v]) => html`<tr key=${k}><th>${k}</th><td>${v}</td></tr>`)}
                ${silent.length > 0 && html`<tr><th>no answer</th><td class="hl-muted">${silent.slice(0, 8).map(n => n.fqn).join(', ')}${silent.length > 8 ? ', …' : ''}</td></tr>`}
            </tbody></table>
        </section>
    `;
}

/** The TF tree as published: each frame under its parent, with who sends
 * it, how often, and whether it moves. */
function TfTree({ tf }) {
    const edges = tf.edges || [];
    if (edges.length === 0) return null;
    const byParent = new Map();
    const children = new Set();
    for (const e of edges) {
        if (!byParent.has(e.parent)) byParent.set(e.parent, []);
        byParent.get(e.parent).push(e);
        children.add(e.child);
    }
    const roots = [...byParent.keys()].filter(p => !children.has(p)).sort();
    /** @type {Array<{root?: string, e?: any, depth?: number}>} */
    const rows = [];
    const seen = new Set();
    const walk = (frame, depth) => {
        for (const e of (byParent.get(frame) || []).slice().sort((a, b) => a.child.localeCompare(b.child))) {
            const key = e.parent + '>' + e.child + '>' + (e.publisher || '');
            if (seen.has(key)) continue;
            seen.add(key);
            rows.push({ e, depth });
            if (depth < 32) walk(e.child, depth + 1);
        }
    };
    roots.forEach(r => { rows.push({ root: r }); walk(r, 1); });
    // Cycles leave edges no root reaches; list them after.
    for (const e of edges) {
        const key = e.parent + '>' + e.child + '>' + (e.publisher || '');
        if (!seen.has(key)) { seen.add(key); rows.push({ e, depth: 1 }); }
    }
    const fmtHz = (h) => h >= 10 ? Math.round(h) : h.toFixed(1);
    return html`
        <section class="hl-section">
            <h3 class="hl-heading">TF <span class="hl-muted hl-heading-count">(${edges.length} edges, ${roots.length} root${roots.length === 1 ? '' : 's'}; as published, read by the interceptor)</span></h3>
            <table class="hl-table">
                <thead><tr><th>frame</th><th>publisher</th><th></th><th class="hl-num">rate</th><th class="hl-num">last</th><th class="hl-num">changes</th></tr></thead>
                <tbody>
                    ${rows.map((r, i) => r.root ? html`
                        <tr class="hl-row" key=${'r' + i}><td><b>${r.root}</b></td><td></td><td></td><td></td><td></td><td></td></tr>` : html`
                        <tr class="hl-row" key=${'e' + i}>
                            <td style=${{ paddingLeft: (r.depth * 18 + 8) + 'px' }}><span class="hl-muted">└ </span>${r.e.child}</td>
                            <td class="hl-muted">${r.e.publisher || '?'}</td>
                            <td>${r.e.is_static ? html`<span class="hl-muted">static</span>` : ''}</td>
                            <td class="hl-num">${r.e.is_static && r.e.rate_hz < 0.5 ? '—' : fmtHz(r.e.rate_hz) + ' Hz'}</td>
                            <td class="hl-num">${r.e.since_last_s < 10 ? r.e.since_last_s.toFixed(1) + ' s' : Math.round(r.e.since_last_s) + ' s'}</td>
                            <td class="hl-num">${r.e.is_static ? '' : r.e.changes}</td>
                        </tr>`)}
                </tbody>
            </table>
        </section>
    `;
}

/** DDS participants this run holds against the middleware's limit, and
 * datagrams the domain's sockets dropped. */
function Discovery({ d }) {
    const rate = (r) => r == null ? '—' : (r < 10 ? r.toFixed(1) : Math.round(r)) + '/min';
    const rows = [
        ['participants', html`<b>${d.participants}</b>${d.executables ? html` <span class="hl-muted">(+${d.executables} if the executables are ROS processes)</span>` : ''}${d.participants_observable < d.participants ? html`; <span class="hl-muted">${d.participants_observable} under --container-mode observable</span>` : ''}`],
        ['limit', d.limit != null ? html`${d.limit} <span class="hl-muted">(${d.limit_reason})</span>` : html`<span class="hl-muted">none known for ${d.rmw}</span>`],
        ['domain', html`ROS_DOMAIN_ID ${d.domain_id}${d.graph_nodes != null ? `, ${d.graph_nodes} nodes in the graph` : ''}`],
        ['UDP drops', html`${rate(d.domain_udp_drops_per_min)} on the domain's ${d.domain_udp_sockets} DDS sockets; <span class="hl-muted">${rate(d.udp_rcvbuf_errors_per_min)} host-wide</span>`],
    ];
    return html`
        <section class="hl-section">
            <h3 class="hl-heading">Discovery</h3>
            <table class="hl-kv"><tbody>
                ${rows.map(([k, v]) => html`<tr key=${k}><th>${k}</th><td>${v}</td></tr>`)}
            </tbody></table>
        </section>
    `;
}

function Host({ host }) {
    const used = (u, t) => (u != null && t) ? `${fmtBytes(u)} of ${fmtBytes(t)} (${Math.round(u * 100 / t)}%)` : '—';
    const rows = [
        ['middleware', html`<b>${host.rmw}</b>${host.rmw_is_default ? html` <span class="hl-muted">(RMW_IMPLEMENTATION unset: the ${host.ros_distro || 'distro'} default)</span>` : ''}`],
        ['DDS config', host.dds_config.length ? host.dds_config.map(c => html`<div class="hl-mono">${c}</div>`) : html`<span class="hl-muted">none set</span>`],
        ['socket buffers', html`rmem_max ${fmtBytes(host.rmem_max)}, wmem_max ${fmtBytes(host.wmem_max)}${host.cyclone_rcvbuf_min != null ? html`; Cyclone asks for ≥ ${fmtBytes(host.cyclone_rcvbuf_min)}` : ''}`],
        ['/dev/shm', html`${used(host.shm_used, host.shm_total)}; Fast DDS files: ${host.fastdds_live} in use, ${host.fastdds_zombie} orphaned${host.fastdds_zombie ? ' (' + fmtBytes(host.fastdds_zombie_bytes) + ')' : ''}`],
        ['log disk', html`${host.disk_free != null ? fmtBytes(host.disk_free) + ' free' : '—'} <span class="hl-muted hl-mono">${host.log_dir}</span>`],
        ['swap', html`${host.swap_total ? used(host.swap_used, host.swap_total) : 'none'}${host.swap_pages_per_s != null ? `, ${Math.round(host.swap_pages_per_s)} pages/s` : ''}`],
    ];
    return html`
        <section class="hl-section">
            <h3 class="hl-heading">Host</h3>
            <table class="hl-kv">
                <tbody>
                    ${rows.map(([k, v]) => html`<tr key=${k}><th>${k}</th><td>${v}</td></tr>`)}
                </tbody>
            </table>
        </section>
    `;
}

function SelfCost({ cost }) {
    const subs = Array.isArray(cost.subsystems) ? cost.subsystems : [];
    const cpu = cost.cpu_percent || 0;
    const rclrs = cost.rclrs_percent || 0;
    let sum = 0;
    for (const [, p] of subs) sum += p || 0;
    const other = Math.max(0, cpu - sum - rclrs);
    const scale = Math.max(cpu, 1);
    const bars = [
        ...subs.map(([name, p]) => ({ name, pct: p || 0, cls: '' })),
        { name: 'rclrs executor', pct: rclrs, cls: 'hl-bar-rclrs' },
        { name: 'other', pct: other, cls: 'hl-bar-other' },
    ];
    return html`
        <section class="hl-section">
            <h3 class="hl-heading">play_launch itself</h3>
            <div class="hl-self-stats">
                <span><b>${fmtPct(cpu)}%</b> of a core</span>
                <span class="hl-sep">·</span>
                <span><b>${fmtBytes(cost.rss_bytes)}</b> RSS</span>
                <span class="hl-sep">·</span>
                <span><b>${cost.threads ?? '—'}</b> threads</span>
            </div>
            <div class="hl-bars">
                ${bars.map(b => html`
                    <div class="hl-bar-row" key=${b.name}>
                        <span class="hl-bar-name">${b.name}</span>
                        <span class="hl-bar-track">
                            <span class=${'hl-bar-fill ' + b.cls}
                                style=${{ width: Math.min(100, (b.pct / scale) * 100) + '%' }}></span>
                        </span>
                        <span class="hl-bar-val">${fmtPct(b.pct)}%</span>
                    </div>
                `)}
            </div>
        </section>
    `;
}

// --- view -----------------------------------------------------------------

export function HealthView() {
    const [snap, setSnap] = useState(null);
    const [offMsg, setOffMsg] = useState(null);
    const [disconnected, setDisconnected] = useState(false);
    // Server clock minus client clock, so "since" is measured on the
    // server's timeline and a skewed browser clock does not distort it.
    const [skew, setSkew] = useState(0);
    const [tick, setTick] = useState(Date.now());
    const [resolvedOpen, setResolvedOpen] = useState(null); // null = auto

    useEffect(() => {
        const id = setInterval(() => setTick(Date.now()), 1000);
        return () => clearInterval(id);
    }, []);

    useEffect(() => {
        let live = true;
        let es = null;
        let retry = null;
        let gotData = false;

        const accept = (data) => {
            gotData = true;
            setSnap(data);
            if (typeof data.t_ms === 'number') setSkew(data.t_ms - Date.now());
            setOffMsg(null);
            setDisconnected(false);
            applyHealthResponse(data); // keep the tab badge in step
        };

        const open = () => {
            if (!live) return;
            es = new EventSource('/api/health/issues/stream');
            es.onmessage = (e) => {
                if (e.data === 'keep-alive') return;
                try { accept(JSON.parse(e.data)); } catch (err) {
                    console.error('[health] bad snapshot:', err);
                }
            };
            es.onerror = () => {
                setDisconnected(true);
                // A non-200 answer closes the source for good; re-probe.
                if (es && es.readyState === EventSource.CLOSED) {
                    es.close();
                    es = null;
                    retry = setTimeout(probe, 3000);
                }
            };
        };

        const probe = async () => {
            if (!live) return;
            try {
                const r = await fetch('/api/health/issues');
                if (!live) return;
                if (r.status === 404 || r.status === 409) {
                    let msg = null;
                    try { const j = await r.json(); if (j && j.error) msg = j.error; } catch (_) { /* no body */ }
                    if (!gotData) setOffMsg({ text: msg });
                    healthIssueCounts.value = null;
                    return;
                }
                if (r.ok) {
                    const data = await r.json();
                    // An older server answers /api/health with the member
                    // summary, which has no issue list.
                    if (!Array.isArray(data.open)) {
                        if (!gotData) setOffMsg({ text: null });
                        return;
                    }
                    accept(data);
                }
            } catch (_) {
                setDisconnected(true);
            }
            open();
        };

        probe();
        return () => {
            live = false;
            if (retry) clearTimeout(retry);
            if (es) es.close();
        };
    }, []);

    const openIssues = useMemo(() => {
        const list = (snap && snap.open) ? snap.open.slice() : [];
        list.sort((a, b) =>
            ((SEVERITY_RANK[a.severity] ?? 3) - (SEVERITY_RANK[b.severity] ?? 3))
            || ((b.opened_ms || 0) - (a.opened_ms || 0))
            || (a.key < b.key ? -1 : a.key > b.key ? 1 : 0));
        return list;
    }, [snap]);

    if (offMsg && !snap) {
        return html`
            <div class="health-view">
                <div class="hl-empty">
                    <div>${UNAVAILABLE}</div>
                    ${offMsg.text && html`<div class="hl-muted hl-empty-detail">${offMsg.text}</div>`}
                </div>
            </div>
        `;
    }

    const now = tick + skew;
    const c = (snap && snap.counts) || { error: 0, warning: 0, info: 0 };
    const closed = (snap && snap.closed) || [];
    const showResolved = resolvedOpen ?? (closed.length <= RESOLVED_COLLAPSE_AT);

    return html`
        <div class="health-view">
            <div class="hl-counts">
                ${snap ? html`
                    <span class=${c.error > 0 ? 'hl-count-error' : ''}><b>${c.error}</b> ${c.error === 1 ? 'error' : 'errors'}</span>
                    <span class="hl-sep">·</span>
                    <span class=${c.warning > 0 ? 'hl-count-warning' : ''}><b>${c.warning}</b> ${c.warning === 1 ? 'warning' : 'warnings'}</span>
                    <span class="hl-sep">·</span>
                    <span><b>${c.info}</b> info</span>
                ` : html`<span class="hl-muted">waiting for first snapshot…</span>`}
                ${disconnected && html`<span class="hl-muted hl-disconnected">disconnected — retrying</span>`}
            </div>

            <section class="hl-section">
                <h3 class="hl-heading">Issues</h3>
                ${snap && openIssues.length === 0 ? html`
                    <div class="hl-calm"><span class="hl-check">✓</span> No open issues</div>
                ` : openIssues.length > 0 && html`
                    <table class="hl-table">
                        <thead>
                            <tr>
                                <th>severity</th>
                                <th>kind</th>
                                <th>subject</th>
                                <th>title</th>
                                <th>detail</th>
                                <th class="hl-since">since</th>
                                <th></th>
                            </tr>
                        </thead>
                        <tbody>
                            ${openIssues.map(i => html`<${IssueRow} key=${i.key} issue=${i} now=${now} />`)}
                        </tbody>
                    </table>
                `}
            </section>

            ${closed.length > 0 && html`
                <section class="hl-section">
                    <h3 class="hl-heading hl-toggle" onClick=${() => setResolvedOpen(!showResolved)}>
                        <span class="hl-caret">${showResolved ? '▾' : '▸'}</span>
                        Recently resolved <span class="hl-muted hl-heading-count">(${closed.length})</span>
                    </h3>
                    ${showResolved && html`
                        <ul class="hl-resolved">
                            ${closed.map((i, idx) => html`
                                <${ResolvedRow} key=${i.key + ':' + (i.closed_ms ?? idx)} issue=${i} now=${now} />`)}
                        </ul>
                    `}
                </section>
            `}

            <${StartupTimeline} />

            ${snap && snap.logs && html`<${Logs} logs=${snap.logs} />`}

            ${snap && snap.tf && html`<${TfTree} tf=${snap.tf} />`}

            ${snap && snap.params && html`<${Params} p=${snap.params} />`}

            ${snap && snap.resources && html`<${Resources} res=${snap.resources} />`}

            ${snap && snap.discovery && html`<${Discovery} d=${snap.discovery} />`}

            ${snap && snap.host && html`<${Host} host=${snap.host} />`}

            ${snap && snap.self_cost && html`<${SelfCost} cost=${snap.self_cost} />`}
        </div>
    `;
}
