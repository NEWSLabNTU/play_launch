// DiagnosticsView component — sortable, filterable diagnostics table.

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useCallback, useMemo } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import { diagnostics, currentView } from '../store.js';

const html = htm.bind(h);

const LEVEL_PRIORITY = {
    'OK': 0, 'Ok': 0,
    'WARNING': 1, 'Warning': 1,
    'ERROR': 2, 'Error': 2,
    'STALE': 3, 'Stale': 3,
};

const LEVEL_FILTER_OPTIONS = ['ALL', 'OK', 'WARNING', 'ERROR', 'STALE'];

function formatRelativeTime(timestamp) {
    const now = new Date();
    const date = new Date(timestamp);
    const seconds = Math.floor((now.getTime() - date.getTime()) / 1000);
    if (seconds < 10) return 'just now';
    if (seconds < 60) return seconds + 's ago';
    const minutes = Math.floor(seconds / 60);
    if (minutes < 60) return minutes + 'm ago';
    const hours = Math.floor(minutes / 60);
    if (hours < 24) return hours + 'h ago';
    const days = Math.floor(hours / 24);
    if (days < 7) return days + 'd ago';
    return Math.floor(days / 7) + 'w ago';
}

function DiagValues({ values }) {
    if (!values || Object.keys(values).length === 0) return html`<span>-</span>`;
    const entries = Object.entries(values);

    if (entries.length === 1) {
        const [key, value] = entries[0];
        let formatted;
        try {
            const parsed = JSON.parse(value);
            formatted = html`<pre class="diagnostic-json-value">${JSON.stringify(parsed, null, 2)}</pre>`;
        } catch {
            formatted = value;
        }
        return html`<div class="value-inline"><strong>${key}:</strong> ${formatted}</div>`;
    }

    return html`
        <div class="value-structured">
            ${entries.map(([key, value]) => {
                let formatted;
                try {
                    const parsed = JSON.parse(value);
                    formatted = html`<pre class="diagnostic-json-value">${JSON.stringify(parsed, null, 2)}</pre>`;
                } catch {
                    formatted = value;
                }
                return html`
                    <div class="value-entry">
                        <span class="value-key">${key}:</span>
                        <span class="value-content">${formatted}</span>
                    </div>
                `;
            })}
        </div>
    `;
}

/** Level-over-time strip for one diagnostic (phase 62 W3).
 *
 * Answers "was it flapping, or did it fail once" — which neither the table nor
 * the CSV answers without a spreadsheet.
 *
 * Segments are proportional to time spent at each level, so a fault lasting a
 * single publish interval is a sliver rather than an equal share. It is still
 * drawn: a minimum width keeps a 100 ms ERROR visible beside an hour of OK,
 * because "briefly failed" and "never failed" must not look alike. That
 * momentary fault surviving at all is the W1 debounce fix's payoff.
 *
 * When `dropped` is non-zero the buffer truncated and the strip says so — a
 * silently truncated strip implies a quiet period that never happened.
 */
function LevelStrip({ history }) {
    if (!history || !history.transitions || history.transitions.length === 0) {
        return html`<span class="strip-empty" title="No level change recorded">-</span>`;
    }
    const t = history.transitions;
    const start = new Date(t[0].at).getTime();
    const end = Date.now();
    const span = Math.max(end - start, 1);
    const segs = t.map((tr, i) => {
        const from = new Date(tr.at).getTime();
        const to = i + 1 < t.length ? new Date(t[i + 1].at).getTime() : end;
        return {
            level: String(tr.level).toLowerCase(),
            pct: Math.max(((to - from) / span) * 100, 0.8),
            at: tr.at,
        };
    });
    const note = `${t.length} transition(s)` + (history.dropped ? `, ${history.dropped} dropped` : '');
    return html`
        <div class="level-strip" title=${note}>
            ${segs.map((sg, i) => html`
                <span key=${i} class=${'level-seg level-seg-' + sg.level}
                      style=${{ width: sg.pct + '%' }}
                      title=${sg.level.toUpperCase() + ' from ' + sg.at}></span>
            `)}
            ${history.dropped > 0 && html`
                <span class="strip-truncated"
                      title=${history.dropped + ' earlier transition(s) fell outside the buffer - this is not the whole run'}>+${history.dropped}</span>
            `}
        </div>
    `;
}

export function DiagnosticsView() {
    const [sortCol, setSortCol] = useState('level');
    const [sortDir, setSortDir] = useState('desc');
    const [levelFilter, setLevelFilter] = useState('ALL');
    const [filterTerm, setFilterTerm] = useState('');

    const view = currentView.value;
    const diags = diagnostics.value;

    // Stream diagnostics while the view is active.
    //
    // This was a 5 s poll, which put an ERROR on screen up to five seconds
    // after the system raised it. The server pushes on every level change and
    // once a second otherwise — the tick is what surfaces a publisher that has
    // gone silent, since silence raises no change event.
    useEffect(() => {
        if (view !== 'diagnostics') return;

        const es = new EventSource('/api/diagnostics/stream');

        es.onmessage = (e) => {
            if (e.data === 'keep-alive') return;
            try {
                const payload = JSON.parse(e.data);
                diagnostics.value = payload.diagnostics || [];
            } catch (err) {
                console.error('Error parsing diagnostics event:', err);
            }
        };

        // EventSource reconnects on its own; log once rather than per retry.
        es.onerror = () => console.debug('[diagnostics] stream interrupted, reconnecting');

        return () => es.close();
    }, [view]);

    // Level history for the strips. Polled at 2 s rather than streamed: it
    // changes only when a level changes, the stream already fires then, and a
    // second SSE channel for a few hundred transitions would be more moving
    // parts than the thing is worth.
    const [history, setHistory] = useState({});
    useEffect(() => {
        if (view !== 'diagnostics') return;
        let live = true;
        const pull = async () => {
            try {
                const r = await fetch('/api/diagnostics/history');
                if (live && r.ok) setHistory(await r.json());
            } catch (err) { /* transient; the next tick retries */ }
        };
        pull();
        const id = setInterval(pull, 2000);
        return () => { live = false; clearInterval(id); };
    }, [view]);

    const toggleSort = useCallback((col) => {
        setSortCol(prev => {
            if (prev === col) {
                setSortDir(d => d === 'asc' ? 'desc' : 'asc');
                return col;
            }
            setSortDir(col === 'level' ? 'desc' : 'asc');
            return col;
        });
    }, []);

    const sorted = useMemo(() => {
        let data = [...diags];

        // Level filter
        if (levelFilter !== 'ALL') {
            const minPriority = LEVEL_PRIORITY[levelFilter] ?? 0;
            data = data.filter(d => (LEVEL_PRIORITY[d.level] ?? 0) >= minPriority);
        }

        // Text filter
        if (filterTerm) {
            const lower = filterTerm.toLowerCase();
            data = data.filter(d => {
                const str = (d.hardware_id + ' ' + d.name + ' ' + (d.message || '')).toLowerCase();
                return str.includes(lower);
            });
        }

        // Sort
        data.sort((a, b) => {
            let cmp = 0;
            if (sortCol === 'level') {
                cmp = (LEVEL_PRIORITY[a.level] ?? 0) - (LEVEL_PRIORITY[b.level] ?? 0);
                if (cmp === 0) cmp = a.name.localeCompare(b.name);
            } else if (sortCol === 'timestamp') {
                cmp = new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime();
            } else {
                cmp = (a[sortCol] || '').localeCompare(b[sortCol] || '');
            }
            return sortDir === 'asc' ? cmp : -cmp;
        });

        return data;
    }, [diags, sortCol, sortDir, levelFilter, filterTerm]);

    const sortClass = (col) => {
        if (sortCol !== col) return 'sortable';
        return sortDir === 'asc' ? 'sorted-asc' : 'sorted-desc';
    };

    return html`
        <div class="diagnostics-view" style=${{ display: view === 'diagnostics' ? 'block' : 'none' }}>
            <div class="diagnostics-header">
                <h2 style="margin:0;">System Diagnostics</h2>
                <select class="diag-level-select" value=${levelFilter}
                    onChange=${(e) => setLevelFilter(e.target.value)}>
                    ${LEVEL_FILTER_OPTIONS.map(l => html`<option key=${l} value=${l}>${l}</option>`)}
                </select>
                <input type="text" class="diagnostics-search" placeholder="Filter diagnostics..."
                    value=${filterTerm} onInput=${(e) => setFilterTerm(e.target.value)} />
            </div>
            <table class="diagnostics-table">
                <thead>
                    <tr>
                        <th class=${sortClass('hardware_id')} onClick=${() => toggleSort('hardware_id')}>Hardware ID</th>
                        <th class=${sortClass('name')} onClick=${() => toggleSort('name')}>Name</th>
                        <th class=${sortClass('level')} onClick=${() => toggleSort('level')}>Level</th>
                        <th title="Level over time — was it flapping, or did it fail once">History</th>
                        <th>Message</th>
                        <th>Values</th>
                        <th class=${sortClass('timestamp')} onClick=${() => toggleSort('timestamp')}>Last Seen</th>
                    </tr>
                </thead>
                <tbody>
                    ${sorted.length === 0 && html`
                        <tr><td colspan="7" class="no-diagnostics">No diagnostics available</td></tr>
                    `}
                    ${sorted.map(diag => html`
                        <tr key=${diag.hardware_id + '/' + diag.name}>
                            <td>${diag.hardware_id}</td>
                            <td>${diag.name}</td>
                            <td><span class="diag-level diag-level-${diag.level.toLowerCase()}">${diag.level}</span></td>
                            <td><${LevelStrip} history=${history[`${diag.hardware_id}/${diag.name}`]} /></td>
                            <td>${diag.message || ''}</td>
                            <td class="diag-values"><${DiagValues} values=${diag.values} /></td>
                            <td class="diag-timestamp">${formatRelativeTime(diag.timestamp)}</td>
                        </tr>
                    `)}
                </tbody>
            </table>
        </div>
    `;
}
