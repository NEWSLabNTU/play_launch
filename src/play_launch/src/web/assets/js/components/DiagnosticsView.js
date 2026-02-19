// DiagnosticsView component — sortable, filterable diagnostics table.

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useCallback, useMemo } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import { diagnostics, currentView } from '../store.js';

const html = htm.bind(h);

const LEVEL_PRIORITY = { 'ERROR': 3, 'WARNING': 2, 'STALE': 1, 'OK': 0 };

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

export function DiagnosticsView() {
    const [sortCol, setSortCol] = useState('level');
    const [sortDir, setSortDir] = useState('desc');
    const [filterTerm, setFilterTerm] = useState('');

    const view = currentView.value;
    const diags = diagnostics.value;

    // Fetch diagnostics when view is active
    useEffect(() => {
        if (view !== 'diagnostics') return;

        function fetchDiags() {
            fetch('/api/diagnostics/list')
                .then(r => r.json())
                .then(data => { diagnostics.value = data; })
                .catch(err => console.error('Error fetching diagnostics:', err));
        }

        fetchDiags();
        const interval = setInterval(fetchDiags, 5000);
        return () => clearInterval(interval);
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

        // Filter
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
                cmp = (LEVEL_PRIORITY[a.level] || 0) - (LEVEL_PRIORITY[b.level] || 0);
                if (cmp === 0) cmp = a.name.localeCompare(b.name);
            } else if (sortCol === 'timestamp') {
                cmp = new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime();
            } else {
                cmp = (a[sortCol] || '').localeCompare(b[sortCol] || '');
            }
            return sortDir === 'asc' ? cmp : -cmp;
        });

        return data;
    }, [diags, sortCol, sortDir, filterTerm]);

    const sortClass = (col) => {
        if (sortCol !== col) return 'sortable';
        return 'sortable ' + (sortDir === 'asc' ? 'sorted-asc' : 'sorted-desc');
    };

    return html`
        <div class="diagnostics-view" style=${{ display: view === 'diagnostics' ? 'block' : 'none' }}>
            <div class="diagnostics-header">
                <h2 style="margin:0;">System Diagnostics</h2>
                <input type="text" class="diagnostics-search" placeholder="Filter diagnostics..."
                    value=${filterTerm} onInput=${(e) => setFilterTerm(e.target.value)} />
            </div>
            <table class="diagnostics-table">
                <thead>
                    <tr>
                        <th class=${sortClass('hardware_id')} onClick=${() => toggleSort('hardware_id')}>Hardware ID</th>
                        <th class=${sortClass('name')} onClick=${() => toggleSort('name')}>Name</th>
                        <th class=${sortClass('level')} onClick=${() => toggleSort('level')}>Level</th>
                        <th>Message</th>
                        <th>Values</th>
                        <th class=${sortClass('timestamp')} onClick=${() => toggleSort('timestamp')}>Last Seen</th>
                    </tr>
                </thead>
                <tbody>
                    ${sorted.length === 0 && html`
                        <tr><td colspan="6" class="no-diagnostics">No diagnostics available</td></tr>
                    `}
                    ${sorted.map(diag => html`
                        <tr key=${diag.hardware_id + '/' + diag.name}>
                            <td>${diag.hardware_id}</td>
                            <td>${diag.name}</td>
                            <td><span class="diag-level diag-level-${diag.level.toLowerCase()}">${diag.level}</span></td>
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
