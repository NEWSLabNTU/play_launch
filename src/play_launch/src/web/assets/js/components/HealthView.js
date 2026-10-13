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

            ${snap && snap.self_cost && html`<${SelfCost} cost=${snap.self_cost} />`}
        </div>
    `;
}
