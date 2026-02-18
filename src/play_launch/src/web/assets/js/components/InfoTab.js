// InfoTab component — JSON viewer for node details.

import { h } from '../vendor/preact.module.js';
import htm from '../vendor/htm.module.js';

const html = htm.bind(h);

function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = String(text);
    return div.innerHTML;
}

function shouldInclude(key, value) {
    return value !== null && value !== undefined;
}

function JsonValue({ val, depth }) {
    if (val === null || val === undefined) {
        return html`<span class="json-value null">null</span>`;
    }
    if (typeof val === 'string') {
        return html`<span class="json-value string">"${val}"</span>`;
    }
    if (typeof val === 'number') {
        return html`<span class="json-value number">${val}</span>`;
    }
    if (typeof val === 'boolean') {
        return html`<span class="json-value boolean">${String(val)}</span>`;
    }
    if (Array.isArray(val)) {
        if (val.length === 0) return html`<span class="json-value">[]</span>`;
        const items = val.map(v => {
            if (typeof v === 'string') return '"' + v + '"';
            if (typeof v === 'number' || typeof v === 'boolean') return String(v);
            return JSON.stringify(v);
        }).join(', ');
        return html`<span class="json-value">[${items}]</span>`;
    }
    if (typeof val === 'object') {
        const keys = Object.keys(val).filter(k => shouldInclude(k, val[k]));
        if (keys.length === 0) return html`<span class="json-value">{}</span>`;
        if (keys.length === 1 && (depth || 0) < 2) {
            const k = keys[0];
            const v = val[k];
            if (typeof v !== 'object') {
                return html`<span class="json-value">${k}: <${JsonValue} val=${v} depth=${(depth || 0) + 1} /></span>`;
            }
        }
        return html`<span class="json-value">{...}</span>`;
    }
    return html`<span class="json-value">${String(val)}</span>`;
}

export function InfoTab({ nodeData }) {
    if (!nodeData) {
        return html`<div class="content-details">
            <div class="no-nodes">Click "View" on a node to see information</div>
        </div>`;
    }

    const entries = Object.entries(nodeData).filter(([k, v]) => shouldInclude(k, v));

    return html`
        <div class="content-details">
            <div class="json-viewer">
                ${entries.map(([key, value]) => {
                    if (value !== null && typeof value === 'object' && !Array.isArray(value)) {
                        const childEntries = Object.entries(value).filter(([k, v]) => shouldInclude(k, v));
                        if (childEntries.length === 0) return null;
                        return html`
                            <div class="json-row">
                                <div class="json-key">${key}</div>
                                <div class="json-value">{...}</div>
                            </div>
                            <div class="json-children" style="margin-left: 20px;">
                                ${childEntries.map(([k, v]) => html`
                                    <div class="json-row">
                                        <div class="json-key">${k}</div>
                                        <div><${JsonValue} val=${v} depth=${1} /></div>
                                    </div>
                                `)}
                            </div>
                        `;
                    }
                    return html`
                        <div class="json-row">
                            <div class="json-key">${key}</div>
                            <div><${JsonValue} val=${value} depth=${0} /></div>
                        </div>
                    `;
                })}
            </div>
        </div>
    `;
}
