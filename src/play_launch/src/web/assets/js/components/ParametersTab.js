// ParametersTab component — displays and edits ROS 2 node parameters (Phase 24).

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useCallback, useRef, useMemo } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import { nodes, parameterVersion, getStatusString } from '../store.js';

const html = htm.bind(h);

/** Short abbreviations for type badges. */
const TYPE_ABBREV = {
    bool: 'bool',
    integer: 'i64',
    double: 'f64',
    string: 'str',
    bool_array: 'bool[]',
    integer_array: 'i64[]',
    double_array: 'f64[]',
    string_array: 'str[]',
    byte_array: 'bytes',
};

/** Sort modes for the parameter list. */
const SORT_MODES = [
    { key: 'name-asc', label: 'Name A\u2192Z' },
    { key: 'name-desc', label: 'Name Z\u2192A' },
    { key: 'type', label: 'Type' },
];

function sortParams(params, mode) {
    const sorted = [...params];
    switch (mode) {
        case 'name-desc':
            sorted.sort((a, b) => b.name.localeCompare(a.name));
            break;
        case 'type':
            sorted.sort((a, b) => {
                const tc = a.type_name.localeCompare(b.type_name);
                return tc !== 0 ? tc : a.name.localeCompare(b.name);
            });
            break;
        case 'name-asc':
        default:
            sorted.sort((a, b) => a.name.localeCompare(b.name));
            break;
    }
    return sorted;
}

/**
 * Check if a node is in a state where parameters can be queried.
 * @param {string} statusStr
 * @returns {boolean}
 */
function canQueryParams(statusStr) {
    return statusStr === 'running' || statusStr === 'loaded';
}

/**
 * Render the appropriate input for a parameter based on its type.
 */
function ParamInput({ entry, onSubmit }) {
    const [value, setValue] = useState('');
    const [editing, setEditing] = useState(false);
    const inputRef = useRef(null);

    // Sync displayed value when entry changes externally
    useEffect(() => {
        if (!editing) {
            setValue(formatValue(entry.value));
        }
    }, [entry.value, editing]);

    const submit = useCallback(() => {
        const parsed = parseInput(entry, value);
        if (parsed !== null) {
            onSubmit(entry.name, parsed);
        }
        setEditing(false);
    }, [entry, value, onSubmit]);

    const onKeyDown = useCallback((e) => {
        if (e.key === 'Enter') submit();
        if (e.key === 'Escape') { setEditing(false); setValue(formatValue(entry.value)); }
    }, [submit, entry.value]);

    if (entry.read_only) {
        return html`<span class="param-value-text param-readonly">${formatValue(entry.value)}</span>`;
    }

    // Bool: toggle switch
    if (entry.type_name === 'bool') {
        const checked = entry.value?.value === true;
        return html`<label class="param-toggle">
            <input type="checkbox" checked=${checked}
                onChange=${(e) => onSubmit(entry.name, { type: 'Bool', value: e.target.checked })} />
            <span class="param-toggle-slider"></span>
        </label>`;
    }

    // Click to edit for other types
    if (!editing) {
        return html`<span class="param-value-text param-editable"
            onClick=${() => { setEditing(true); setTimeout(() => inputRef.current?.focus(), 0); }}
        >${formatValue(entry.value)}</span>`;
    }

    const inputType = (entry.type_name === 'integer' || entry.type_name === 'double') ? 'number' : 'text';
    const step = entry.type_name === 'integer' ? '1' : 'any';
    const min = entry.integer_range?.from ?? entry.floating_point_range?.from;
    const max = entry.integer_range?.to ?? entry.floating_point_range?.to;

    return html`<input ref=${inputRef} class="param-input" type=${inputType}
        step=${step} min=${min} max=${max}
        value=${value}
        onInput=${(e) => setValue(e.target.value)}
        onKeyDown=${onKeyDown}
        onBlur=${submit} />`;
}

/**
 * Format a ParamValue for display.
 */
function formatValue(pv) {
    if (!pv) return '(not set)';
    switch (pv.type) {
        case 'Bool': return pv.value ? 'true' : 'false';
        case 'Integer': return String(pv.value);
        case 'Double': return String(pv.value);
        case 'String': return pv.value;
        case 'BoolArray': return (pv.value || []).join(', ');
        case 'IntegerArray': return (pv.value || []).join(', ');
        case 'DoubleArray': return (pv.value || []).join(', ');
        case 'StringArray': return (pv.value || []).join(', ');
        case 'ByteArray': return (pv.value || []).map(b => b.toString(16).padStart(2, '0')).join(' ');
        case 'NotSet': return '(not set)';
        default: return JSON.stringify(pv);
    }
}

/**
 * Parse user input back to a ParamValue wire format.
 */
function parseInput(entry, rawValue) {
    const type = entry.type_name;
    switch (type) {
        case 'bool':
            return { type: 'Bool', value: rawValue === 'true' || rawValue === '1' };
        case 'integer': {
            const n = parseInt(rawValue, 10);
            return isNaN(n) ? null : { type: 'Integer', value: n };
        }
        case 'double': {
            const n = parseFloat(rawValue);
            return isNaN(n) ? null : { type: 'Double', value: n };
        }
        case 'string':
            return { type: 'String', value: rawValue };
        case 'bool_array':
            return { type: 'BoolArray', value: rawValue.split(',').map(s => s.trim() === 'true') };
        case 'integer_array':
            return { type: 'IntegerArray', value: rawValue.split(',').map(s => parseInt(s.trim(), 10)) };
        case 'double_array':
            return { type: 'DoubleArray', value: rawValue.split(',').map(s => parseFloat(s.trim())) };
        case 'string_array':
            return { type: 'StringArray', value: rawValue.split(',').map(s => s.trim()) };
        default:
            return { type: 'String', value: rawValue };
    }
}

/**
 * Build a tooltip string for a parameter (description + range info).
 */
function buildTooltip(entry) {
    const parts = [];
    if (entry.description) parts.push(entry.description);
    if (entry.integer_range) {
        parts.push(`Range: ${entry.integer_range.from}..${entry.integer_range.to}` +
            (entry.integer_range.step > 0 ? ` (step ${entry.integer_range.step})` : ''));
    }
    if (entry.floating_point_range) {
        parts.push(`Range: ${entry.floating_point_range.from}..${entry.floating_point_range.to}` +
            (entry.floating_point_range.step > 0 ? ` (step ${entry.floating_point_range.step})` : ''));
    }
    return parts.join('\n') || entry.name;
}

export function ParametersTab({ nodeName }) {
    const [params, setParams] = useState([]);
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState(null);
    const [filter, setFilter] = useState('');
    const [sortMode, setSortMode] = useState('name-asc');
    const [feedback, setFeedback] = useState({}); // { paramName: { ok: bool, msg: string } }
    const [nameColWidth, setNameColWidth] = useState(50); // percentage
    const dragging = useRef(false);
    const tableRef = useRef(null);

    const storeNode = nodes.value.get(nodeName);
    const statusStr = getStatusString(storeNode?.status);
    const isContainer = storeNode?.node_type === 'container';
    const version = parameterVersion.value.get(nodeName) || 0;

    // Fetch parameters when node is selected or version bumps
    useEffect(() => {
        if (!nodeName || !canQueryParams(statusStr) || isContainer) {
            setParams([]);
            return;
        }
        setLoading(true);
        setError(null);
        fetch('/api/nodes/' + encodeURIComponent(nodeName) + '/parameters')
            .then(res => {
                if (!res.ok) return res.json().then(e => Promise.reject(e.error || res.statusText));
                return res.json();
            })
            .then(data => { setParams(data); setLoading(false); })
            .catch(err => { setError(String(err)); setLoading(false); });
    }, [nodeName, statusStr, isContainer, version]);

    const handleSet = useCallback((paramName, newValue) => {
        fetch('/api/nodes/' + encodeURIComponent(nodeName) + '/parameters', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name: paramName, value: newValue }),
        })
        .then(res => res.json())
        .then(result => {
            if (result.successful) {
                setFeedback(f => ({ ...f, [paramName]: { ok: true, msg: '' } }));
                // Re-fetch to get updated values
                return fetch('/api/nodes/' + encodeURIComponent(nodeName) + '/parameters')
                    .then(r => r.json())
                    .then(data => setParams(data));
            } else {
                setFeedback(f => ({ ...f, [paramName]: { ok: false, msg: result.reason } }));
            }
        })
        .catch(err => {
            setFeedback(f => ({ ...f, [paramName]: { ok: false, msg: String(err) } }));
        });

        // Clear feedback after 3 seconds
        setTimeout(() => setFeedback(f => { const n = { ...f }; delete n[paramName]; return n; }), 3000);
    }, [nodeName]);

    // Column resize handlers
    const onResizeStart = useCallback((e) => {
        e.preventDefault();
        dragging.current = true;
        document.body.style.cursor = 'col-resize';
        document.body.style.userSelect = 'none';

        const onMove = (ev) => {
            if (!dragging.current || !tableRef.current) return;
            const rect = tableRef.current.getBoundingClientRect();
            const x = (ev.clientX || ev.touches?.[0]?.clientX) - rect.left;
            const pct = Math.max(20, Math.min(80, (x / rect.width) * 100));
            setNameColWidth(pct);
        };
        const onUp = () => {
            dragging.current = false;
            document.body.style.cursor = '';
            document.body.style.userSelect = '';
            document.removeEventListener('mousemove', onMove);
            document.removeEventListener('mouseup', onUp);
            document.removeEventListener('touchmove', onMove);
            document.removeEventListener('touchend', onUp);
        };
        document.addEventListener('mousemove', onMove);
        document.addEventListener('mouseup', onUp);
        document.addEventListener('touchmove', onMove);
        document.addEventListener('touchend', onUp);
    }, []);

    // Filter and sort
    const displayed = useMemo(() => {
        const filtered = filter
            ? params.filter(p => p.name.toLowerCase().includes(filter.toLowerCase()))
            : params;
        return sortParams(filtered, sortMode);
    }, [params, filter, sortMode]);

    // State-aware messages
    if (!nodeName) return null;

    if (isContainer) {
        return html`<div class="param-message">Containers do not expose parameter services. Select a composable node to view its parameters.</div>`;
    }
    if (statusStr === 'pending' || statusStr === 'loading') {
        return html`<div class="param-message">Waiting for node to start...</div>`;
    }
    if (statusStr === 'blocked') {
        return html`<div class="param-message">Node blocked — parameters unavailable</div>`;
    }
    if (!canQueryParams(statusStr)) {
        return html`<div class="param-message">Node not running — parameters unavailable</div>`;
    }

    if (loading) {
        return html`<div class="param-message">Loading parameters...</div>`;
    }
    if (error) {
        return html`<div class="param-message param-error-msg">${error}</div>`;
    }

    return html`
        <div class="param-container">
            <div class="param-toolbar">
                <input class="param-search" type="text" placeholder="Filter parameters..."
                    value=${filter} onInput=${(e) => setFilter(e.target.value)} />
                <select class="param-sort-select" value=${sortMode}
                    onChange=${(e) => setSortMode(e.target.value)}>
                    ${SORT_MODES.map(m => html`<option key=${m.key} value=${m.key}>${m.label}</option>`)}
                </select>
                <button class="param-refresh-btn" onClick=${() => {
                    const pmap = new Map(parameterVersion.value);
                    pmap.set(nodeName, (pmap.get(nodeName) || 0) + 1);
                    parameterVersion.value = pmap;
                }}>Refresh</button>
            </div>
            ${displayed.length === 0
                ? html`<div class="param-message">${filter ? 'No matching parameters' : 'No parameters'}</div>`
                : html`<div class="param-table-wrap">
                    <table class="param-table" ref=${tableRef}>
                        <colgroup>
                            <col style=${{ width: nameColWidth + '%' }} />
                            <col style=${{ width: (100 - nameColWidth) + '%' }} />
                        </colgroup>
                        <thead><tr>
                            <th class="param-th-name">
                                Name
                                <span class="param-col-resizer" onMouseDown=${onResizeStart}
                                    onTouchStart=${onResizeStart}></span>
                            </th>
                            <th class="param-th-value">Value</th>
                        </tr></thead>
                        <tbody>
                            ${displayed.map(entry => html`
                                <tr key=${entry.name}
                                    class=${feedback[entry.name]?.ok === true ? 'param-row-success' : ''}>
                                    <td class="param-name" title=${buildTooltip(entry)}>
                                        <span class="param-type-badge">${TYPE_ABBREV[entry.type_name] || entry.type_name}</span>
                                        ${entry.read_only ? html`<span class="param-lock" title="Read-only">${'\uD83D\uDD12'}</span>` : null}
                                        ${' '}${entry.name}
                                    </td>
                                    <td class="param-value-cell">
                                        <${ParamInput} entry=${entry} onSubmit=${handleSet} />
                                        ${feedback[entry.name] && !feedback[entry.name].ok
                                            ? html`<div class="param-error">${feedback[entry.name].msg}</div>`
                                            : null}
                                    </td>
                                </tr>
                            `)}
                        </tbody>
                    </table>
                </div>`
            }
        </div>
    `;
}
