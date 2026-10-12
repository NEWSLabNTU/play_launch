// EntitiesView component — a live, htop-like table of every ROS publisher
// and subscription the interceptor has seen.
//
// One snapshot per second arrives over SSE (`/api/entities/stream`). The
// server already trims and pre-sorts; everything here (sort, filter, group)
// is client-side over the rows received, so a header click never waits for
// the next tick.

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useCallback, useMemo } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import { selectedNode, nodePanelOpen, activeTab, nodes, getNodeByKey, memberFqn } from '../store.js';

const html = htm.bind(h);

const INTERCEPTION_OFF =
    'No live traffic: interception is off for this run (start with --interception on)';

// Filter/sort choices survive a tab switch (the component unmounts so the
// stream closes, but the user's view of the table should not reset).
const prefs = {
    sortCol: 'bytes',
    sortDir: 'desc',
    filterTerm: '',
    direction: 'all',
    hideSilent: false,
    onlyFlagged: false,
    topOnly: true,
    groupByTopic: false,
};

function usePref(key) {
    const [v, setV] = useState(prefs[key]);
    const set = useCallback((nv) => { prefs[key] = nv; setV(nv); }, [key]);
    return [v, set];
}

// --- formatting -----------------------------------------------------------

const UNITS = ['B', 'KiB', 'MiB', 'GiB', 'TiB'];

/** Human-readable byte count; "—" when unknown. */
function fmtBytes(b) {
    if (b == null || !isFinite(b)) return '—';
    let i = 0;
    let v = b;
    while (v >= 1024 && i < UNITS.length - 1) { v /= 1024; i++; }
    return (i === 0 ? Math.round(v) : v.toFixed(v < 10 ? 2 : 1)) + ' ' + UNITS[i];
}

function fmtRate(r) {
    if (r == null || !isFinite(r)) return '—';
    if (r === 0) return '0';
    if (r < 10) return r.toFixed(2);
    if (r < 100) return r.toFixed(1);
    return Math.round(r).toString();
}

/** Time since the last message. A null with zero messages means "never",
 * which is a different statement from "unknown". */
function fmtLast(row) {
    const s = row.since_last_message_s;
    if (s == null) return row.messages === 0 ? 'never' : '—';
    if (s < 10) return s.toFixed(1) + 's';
    if (s < 60) return Math.round(s) + 's';
    if (s < 3600) return Math.floor(s / 60) + 'm' + Math.round(s % 60) + 's';
    return (s / 3600).toFixed(1) + 'h';
}

const REL = { reliable: 'R', best_effort: 'B', system_default: 'D', best_available: 'A' };
const DUR = { volatile: 'V', transient_local: 'T', system_default: 'D', best_available: 'A' };

function qosCompact(q) {
    if (!q) return '?';
    const r = REL[q.reliability] || '?';
    const d = DUR[q.durability] || '?';
    let hst = '?';
    if (q.history === 'keep_last') hst = 'K' + q.depth;
    else if (q.history === 'keep_all') hst = 'A';
    else if (q.history === 'system_default') hst = 'D';
    return `${r} ${d} ${hst}`;
}

function qosTitle(q) {
    if (!q) return 'QoS unknown';
    const ms = (v) => (v == null ? 'none' : v + ' ms');
    return [
        `reliability: ${q.reliability}`,
        `durability: ${q.durability}`,
        `history: ${q.history}${q.history === 'keep_last' ? ' (depth ' + q.depth + ')' : ''}`,
        `deadline: ${ms(q.deadline_ms)}`,
        `liveliness: ${q.liveliness}`,
        `lease: ${ms(q.lease_ms)}`,
    ].join('\n');
}

// --- flags ----------------------------------------------------------------

const FLAG_WEIGHT = { incompatible: 100, advisory: 10, stale: 5, silent: 1, confirmed: 0 };

function flagScore(row) {
    let s = 0;
    for (const f of row.flags || []) s += FLAG_WEIGHT[f.kind] || 0;
    return s;
}

function hasFlag(row, kind) {
    return (row.flags || []).some(f => f.kind === kind);
}

/** "Flagged" means something worth a look; a DDS confirmation is not. */
function isFlagged(row) {
    return (row.flags || []).some(f => f.kind !== 'confirmed');
}

function FlagBadge({ flag }) {
    let cls = 'ent-flag';
    let label = flag.code;
    switch (flag.kind) {
        case 'incompatible': cls += ' ent-flag-bad'; label = '✖ ' + flag.code; break;
        case 'advisory': cls += ' ent-flag-warn'; label = '⚠ ' + flag.code; break;
        case 'stale': cls += ' ent-flag-warn'; label = 'stale'; break;
        case 'silent': cls += ' ent-flag-muted'; label = 'silent'; break;
        case 'confirmed': cls += ' ent-flag-ok'; label = 'dds'; break;
    }
    return html`<span class=${cls} title=${flag.message}>${label}</span>`;
}

// --- sorting --------------------------------------------------------------

/** Numeric/string value of a row for a column. Unknown numbers sort as the
 * smallest value so they sink under desc, the default direction. */
const SORT_VALUE = {
    dir: r => r.direction,
    topic: r => r.topic || '',
    node: r => r.node || r.member || '',
    rate: r => r.rate_1s ?? -1,
    bytes: r => r.bytes_per_s ?? -1,
    size: r => r.mean_message_bytes ?? -1,
    last: r => (r.since_last_message_s ?? (r.messages === 0 ? Infinity : -1)),
    qos: r => qosCompact(r.qos),
    flags: r => flagScore(r),
};

const STRING_COLS = new Set(['dir', 'topic', 'node', 'qos']);

function cmpVal(a, b) {
    if (typeof a === 'string') return a.localeCompare(b);
    return a < b ? -1 : a > b ? 1 : 0;
}

function rowComparator(col, dir) {
    const get = SORT_VALUE[col] || SORT_VALUE.bytes;
    const sign = dir === 'asc' ? 1 : -1;
    return (a, b) => {
        let c = cmpVal(get(a), get(b)) * sign;
        // Fixed tiebreak, independent of direction, so equal rows do not
        // shuffle between ticks.
        if (c === 0) c = (b.rate_1s ?? 0) - (a.rate_1s ?? 0);
        if (c === 0) c = a.id < b.id ? -1 : a.id > b.id ? 1 : 0;
        return c;
    };
}

/** A group's sort value: the max over members (string columns use the
 * topic name, which is the group's identity). */
function groupValue(g, col) {
    if (STRING_COLS.has(col)) return g.topic;
    const get = SORT_VALUE[col] || SORT_VALUE.bytes;
    let m = -Infinity;
    for (const r of g.rows) { const v = get(r); if (v > m) m = v; }
    return m;
}

// --- selection ------------------------------------------------------------

/** Resolve a row to a member key in the node store, or null. The ROS node
 * FQN wins (it singles out a composable inside a container); the member key
 * is the fallback. */
function resolveMember(row) {
    if (row.node) {
        for (const n of nodes.value.values()) {
            const fqn = memberFqn(n.id || n.name);
            if (fqn === row.node || fqn.replace(/#\d+$/, '') === row.node) return n.id || n.name;
        }
    }
    if (row.member) {
        const n = getNodeByKey(row.member);
        if (n) return n.id || n.name;
    }
    return null;
}

// --- rows -----------------------------------------------------------------

function EntityRow({ row, selected, onSelect }) {
    const flags = row.flags || [];
    const silent = hasFlag(row, 'silent');
    return html`
        <tr class=${'ent-row' + (selected ? ' ent-row-selected' : '') + (silent ? ' ent-row-silent' : '')}
            onClick=${() => onSelect(row)}>
            <td><span class=${'ent-dir ent-dir-' + row.direction}>${row.direction}</span></td>
            <td class="ent-topic" title=${row.msg_type || ''}>${row.topic || '?'}</td>
            <td class="ent-node" title=${row.member || ''}>${row.node || row.member}</td>
            <td class="ent-num" title=${'10 s mean: ' + fmtRate(row.rate_10s) + ' msg/s; ' + row.messages + ' total'}>
                ${fmtRate(row.rate_1s)}<span class="ent-sub">${fmtRate(row.rate_10s)}</span>
            </td>
            <td class="ent-num">${row.bytes_per_s == null ? '—' : fmtBytes(row.bytes_per_s) + '/s'}</td>
            <td class="ent-num">${fmtBytes(row.mean_message_bytes)}</td>
            <td class="ent-num">${fmtLast(row)}</td>
            <td class="ent-qos" title=${qosTitle(row.qos)}>${qosCompact(row.qos)}</td>
            <td class="ent-flags">${flags.map((f, i) => html`<${FlagBadge} key=${i} flag=${f} />`)}</td>
        </tr>
    `;
}

function GroupRow({ group }) {
    let pubRate = 0, pubs = 0, subs = 0;
    for (const r of group.rows) {
        if (r.direction === 'pub') { pubs++; pubRate += r.rate_10s || 0; } else { subs++; }
    }
    return html`
        <tr class="ent-group">
            <td colspan="9">
                <span class="ent-group-topic">${group.topic}</span>
                ${group.msgType && html`<span class="ent-group-type">${group.msgType}</span>`}
                <span class="ent-group-stat">${fmtRate(pubRate)} msg/s published</span>
                <span class="ent-group-stat">${pubs} pub · ${subs} sub</span>
            </td>
        </tr>
    `;
}

// --- view -----------------------------------------------------------------

export function EntitiesView() {
    const [snap, setSnap] = useState(null);
    const [offMsg, setOffMsg] = useState(null);      // interception off / 409 text
    const [disconnected, setDisconnected] = useState(false);
    const [selectedRow, setSelectedRow] = useState(null);

    const [sortCol, setSortCol] = usePref('sortCol');
    const [sortDir, setSortDir] = usePref('sortDir');
    const [filterTerm, setFilterTerm] = usePref('filterTerm');
    const [direction, setDirection] = usePref('direction');
    const [hideSilent, setHideSilent] = usePref('hideSilent');
    const [onlyFlagged, setOnlyFlagged] = usePref('onlyFlagged');
    const [topOnly, setTopOnly] = usePref('topOnly');
    const [groupByTopic, setGroupByTopic] = usePref('groupByTopic');

    // Stream while mounted; reconnect when "Top rows only" changes.
    useEffect(() => {
        const q = topOnly ? '' : '?all=1';
        let live = true;
        let es = null;
        let retry = null;
        let gotData = false;

        const accept = (data) => {
            gotData = true;
            setSnap(data);
            setOffMsg(null);
            setDisconnected(false);
        };

        const open = () => {
            if (!live) return;
            es = new EventSource('/api/entities/stream' + q);
            es.onmessage = (e) => {
                if (e.data === 'keep-alive') return;
                try { accept(JSON.parse(e.data)); } catch (err) {
                    console.error('[entities] bad snapshot:', err);
                }
            };
            es.onerror = () => {
                setDisconnected(true);
                // A non-200 answer (404/409) closes the source for good
                // instead of retrying; re-probe so a recovered server, or
                // one that turns out to have interception off, is reported.
                if (es && es.readyState === EventSource.CLOSED) {
                    es.close();
                    es = null;
                    retry = setTimeout(probe, 3000);
                }
            };
        };

        // One plain fetch first: it paints before the first tick, and it is
        // the only place a status code (and a 409's error text) is visible.
        const probe = async () => {
            if (!live) return;
            try {
                const r = await fetch('/api/entities' + q);
                if (!live) return;
                if (r.status === 404) {
                    if (!gotData) setOffMsg(INTERCEPTION_OFF);
                    return;
                }
                if (r.status === 409) {
                    let msg = INTERCEPTION_OFF;
                    try { const j = await r.json(); if (j && j.error) msg = j.error; } catch (_) { /* keep default */ }
                    if (!gotData) setOffMsg(msg);
                    return;
                }
                if (r.ok) accept(await r.json());
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
    }, [topOnly]);

    const toggleSort = useCallback((col) => {
        if (prefs.sortCol === col) {
            setSortDir(prefs.sortDir === 'asc' ? 'desc' : 'asc');
        } else {
            setSortCol(col);
            setSortDir(STRING_COLS.has(col) ? 'asc' : 'desc');
        }
    }, []);

    const onSelect = useCallback((row) => {
        setSelectedRow(row.id);
        const key = resolveMember(row);
        if (!key) return;
        selectedNode.value = key;
        nodePanelOpen.value = true;
        activeTab.value = activeTab.peek() || 'stderr';
    }, []);

    // Filter + sort (+ group). Plain loops; this runs once per tick.
    const body = useMemo(() => {
        const rows = (snap && snap.rows) || [];
        const term = filterTerm.trim().toLowerCase();
        const kept = [];
        for (const r of rows) {
            if (direction !== 'all' && r.direction !== direction) continue;
            if (hideSilent && hasFlag(r, 'silent')) continue;
            if (onlyFlagged && !isFlagged(r)) continue;
            if (term) {
                const hay = ((r.topic || '') + '\u0000' + (r.node || '') + '\u0000' + (r.member || '')).toLowerCase();
                if (!hay.includes(term)) continue;
            }
            kept.push(r);
        }
        const cmp = rowComparator(sortCol, sortDir);
        if (!groupByTopic) {
            kept.sort(cmp);
            return { rows: kept, groups: null, count: kept.length };
        }
        const byTopic = new Map();
        for (const r of kept) {
            const t = r.topic || '(unknown topic)';
            let g = byTopic.get(t);
            if (!g) { g = { topic: t, msgType: r.msg_type, rows: [] }; byTopic.set(t, g); }
            if (!g.msgType && r.msg_type) g.msgType = r.msg_type;
            g.rows.push(r);
        }
        const groups = Array.from(byTopic.values());
        for (const g of groups) {
            g.rows.sort((a, b) => (a.direction === b.direction ? cmp(a, b) : a.direction === 'pub' ? -1 : 1));
        }
        const sign = sortDir === 'asc' ? 1 : -1;
        groups.sort((a, b) => cmpVal(groupValue(a, sortCol), groupValue(b, sortCol)) * sign
            || a.topic.localeCompare(b.topic));
        return { rows: null, groups, count: kept.length };
    }, [snap, filterTerm, direction, hideSilent, onlyFlagged, sortCol, sortDir, groupByTopic]);

    if (offMsg && !snap) {
        return html`
            <div class="entities-view">
                <div class="ent-empty">${offMsg}</div>
            </div>
        `;
    }

    const t = snap && snap.totals;
    const sortClass = (col) => sortCol !== col ? 'sortable' : (sortDir === 'asc' ? 'sorted-asc' : 'sorted-desc');
    const th = (col, label, cls = '', title = '') => html`
        <th class=${sortClass(col) + (cls ? ' ' + cls : '')} title=${title} onClick=${() => toggleSort(col)}>${label}</th>
    `;

    const renderRow = (r) => html`<${EntityRow} key=${r.id} row=${r} selected=${selectedRow === r.id} onSelect=${onSelect} />`;

    return html`
        <div class="entities-view">
            <div class="ent-summary">
                ${t ? html`
                    <span>pubs <b>${t.pubs}</b> (${t.silent_pubs} silent)</span>
                    <span class="ent-sep">·</span>
                    <span>subs <b>${t.subs}</b> (${t.silent_subs} silent)</span>
                    <span class="ent-sep">·</span>
                    <span>QoS: <b class=${t.incompatible > 0 ? 'ent-bad' : ''}>${t.incompatible}</b> incompatible,
                        <b class=${t.advisories > 0 ? 'ent-warn' : ''}>${t.advisories}</b> advisories</span>
                    <span class="ent-sep">·</span>
                    <span>ring drops <b class=${t.ring_drops > 0 ? 'ent-warn' : ''}>${t.ring_drops}</b></span>
                    ${snap.truncated > 0 && html`
                        <span class="ent-sep">·</span>
                        <span title="The server trimmed quiet rows; uncheck 'Top rows only' to see all">
                            showing ${snap.rows.length} of ${snap.rows.length + snap.truncated}</span>`}
                ` : html`<span class="ent-muted">waiting for first snapshot…</span>`}
                ${disconnected && html`<span class="ent-muted ent-disconnected">disconnected — retrying</span>`}
            </div>
            <div class="ent-controls">
                <input type="text" class="ent-search" placeholder="Filter topic / node..."
                    value=${filterTerm} onInput=${(e) => setFilterTerm(e.target.value)} />
                <select class="ent-select" value=${direction} onChange=${(e) => setDirection(e.target.value)}>
                    <option value="all">pub + sub</option>
                    <option value="pub">pub</option>
                    <option value="sub">sub</option>
                </select>
                <label><input type="checkbox" checked=${hideSilent}
                    onChange=${(e) => setHideSilent(e.target.checked)} /> hide silent</label>
                <label><input type="checkbox" checked=${onlyFlagged}
                    onChange=${(e) => setOnlyFlagged(e.target.checked)} /> only flagged</label>
                <label><input type="checkbox" checked=${groupByTopic}
                    onChange=${(e) => setGroupByTopic(e.target.checked)} /> group by topic</label>
                <label title="The server sends only the busiest rows; uncheck to receive every entity">
                    <input type="checkbox" checked=${topOnly}
                        onChange=${(e) => setTopOnly(e.target.checked)} /> top rows only</label>
                <span class="ent-muted ent-count">${body.count} shown</span>
            </div>
            <table class="ent-table">
                <thead>
                    <tr>
                        ${th('dir', 'dir')}
                        ${th('topic', 'topic')}
                        ${th('node', 'node')}
                        ${th('rate', 'msg/s', 'ent-num', '1 s rate (10 s mean below)')}
                        ${th('bytes', 'bytes/s', 'ent-num')}
                        ${th('size', 'size', 'ent-num', 'Mean message size')}
                        ${th('last', 'last', 'ent-num', 'Time since the last message')}
                        ${th('qos', 'QoS', '', 'Reliability (R/B) · durability (V/T) · history (K<depth>/A)')}
                        ${th('flags', 'flags')}
                    </tr>
                </thead>
                <tbody>
                    ${body.count === 0 && html`
                        <tr><td colspan="9" class="ent-none">${snap ? 'No entities match' : ''}</td></tr>`}
                    ${body.rows
                        ? body.rows.map(renderRow)
                        : body.groups.flatMap(g => [
                            html`<${GroupRow} key=${'g:' + g.topic} group=${g} />`,
                            ...g.rows.map(renderRow),
                        ])}
                </tbody>
            </table>
        </div>
    `;
}
