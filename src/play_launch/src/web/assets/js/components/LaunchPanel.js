// LaunchPanel — right panel for the Launch page.
// Shows scope info (with tabs) when a scope is selected,
// or node details (with tabs) when a node is selected.

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useCallback } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import {
    launchTree, launchTreeSelection, nodes, getStatusString,
} from '../store.js';
import { InfoTab } from './InfoTab.js';
import { LogTab } from './LogTab.js';
import { ParametersTab } from './ParametersTab.js';
import { TopicsTab } from './TopicsTab.js';
import { MetricsTab } from './MetricsTab.js';

const html = htm.bind(h);

// ── Scope tabs ──

function ScopeInfoTab({ scope, scopes, nodeScopes }) {
    // Walk parent chain (with cycle protection)
    const chain = [];
    const visited = new Set();
    let current = scope.id;
    while (current !== null && current !== undefined && current < scopes.length && !visited.has(current)) {
        visited.add(current);
        chain.unshift(scopes[current]);
        current = scopes[current].parent;
    }

    // Entities in this scope
    const entities = [];
    for (const [name, sid] of Object.entries(nodeScopes)) {
        if (sid === scope.id) entities.push(name);
    }
    entities.sort();

    // Child scopes
    const children = scopes.filter(s => s.parent === scope.id);

    return html`
        <div class="lp-scroll">
            <div class="lp-section">
                <div class="lp-label">Package</div>
                <div class="lp-value lp-pkg">${scope.pkg || '(none)'}</div>
            </div>
            <div class="lp-section">
                <div class="lp-label">File</div>
                <div class="lp-value">${scope.file}</div>
            </div>
            <div class="lp-section">
                <div class="lp-label">Namespace</div>
                <div class="lp-value lp-ns-val">${scope.ns}</div>
            </div>

            ${chain.length > 1 && html`
                <div class="lp-section">
                    <div class="lp-label">Include Chain</div>
                    <div class="lp-chain">
                        ${chain.map((s, i) => html`
                            <div class="lp-chain-item" key=${s.id}>
                                <span class="lp-chain-depth">${'\u00A0'.repeat(i * 2)}${i > 0 ? '\u2514\u2500 ' : ''}</span>
                                <span class="lp-pkg">${s.pkg || '?'}</span>
                                <span> ${s.file}</span>
                            </div>
                        `)}
                    </div>
                </div>
            `}

            ${entities.length > 0 && html`
                <div class="lp-section">
                    <div class="lp-label">Entities (${entities.length})</div>
                    <div class="lp-entities">
                        ${entities.map(name => {
                            const node = nodes.value.get(name);
                            const status = node ? getStatusString(node.status) : 'unknown';
                            return html`
                                <div class="lp-entity" key=${name}
                                     onClick=${() => { launchTreeSelection.value = { type: 'node', name }; }}>
                                    <span class="lt-status-dot lt-status-${status}"></span>
                                    <span>${name}</span>
                                </div>
                            `;
                        })}
                    </div>
                </div>
            `}

            ${children.length > 0 && html`
                <div class="lp-section">
                    <div class="lp-label">Includes (${children.length})</div>
                    <div class="lp-includes">
                        ${children.map(cs => html`
                            <div class="lp-include" key=${cs.id}
                                 onClick=${() => { launchTreeSelection.value = { type: 'scope', id: cs.id }; }}>
                                <span class="lp-pkg">${cs.pkg || '?'}</span>
                                <span> ${cs.file}</span>
                                ${cs.ns !== '/' && html`<span class="lp-ns"> ${cs.ns}</span>`}
                            </div>
                        `)}
                    </div>
                </div>
            `}
        </div>
    `;
}

function ScopeArgsTab({ scope }) {
    const args = scope.args
        ? Object.entries(scope.args).sort((a, b) => a[0].localeCompare(b[0]))
        : [];

    if (args.length === 0) {
        return html`<div class="lp-scroll lp-empty-tab">No arguments</div>`;
    }

    return html`
        <div class="lp-scroll">
            <div class="lp-args-table">
                ${args.map(([k, v]) => html`
                    <div class="lp-arg" key=${k}>
                        <span class="lp-arg-key">${k}</span>
                        <span class="lp-arg-val">${v}</span>
                    </div>
                `)}
            </div>
        </div>
    `;
}

/** Scope detail view with tabs. */
function ScopeDetail({ scopeId }) {
    const [tab, setTab] = useState('info');
    const tree = launchTree.value;
    if (!tree || !tree.scopes) return null;

    const scopes = tree.scopes;
    const scope = scopes[scopeId];
    if (!scope) return null;

    const nodeScopes = tree.node_scopes || {};
    const argCount = scope.args ? Object.keys(scope.args).length : 0;

    return html`
        <div class="lp-scope-detail">
            <div class="right-panel-tabs">
                <button class="tab-btn ${tab === 'info' ? 'active' : ''}"
                    onClick=${() => setTab('info')}>Info</button>
                <button class="tab-btn ${tab === 'args' ? 'active' : ''}"
                    onClick=${() => setTab('args')}>Args${argCount > 0 ? ` (${argCount})` : ''}</button>
            </div>
            <div class="right-panel-content">
                <div class="tab-content" style=${{ display: tab === 'info' ? 'flex' : 'none' }}>
                    <${ScopeInfoTab} scope=${scope} scopes=${scopes} nodeScopes=${nodeScopes} />
                </div>
                <div class="tab-content" style=${{ display: tab === 'args' ? 'flex' : 'none' }}>
                    <${ScopeArgsTab} scope=${scope} />
                </div>
            </div>
        </div>
    `;
}

// ── Node detail ──

function NodeDetail({ name }) {
    const [tab, setTab] = useState('stderr');
    const [nodeData, setNodeData] = useState(null);

    useEffect(() => {
        if (!name) { setNodeData(null); return; }
        const controller = new AbortController();
        const timeout = setTimeout(() => controller.abort(), 5000);
        fetch('/api/nodes/' + encodeURIComponent(name), { signal: controller.signal })
            .then(res => {
                if (!res.ok) throw new Error(`HTTP ${res.status}`);
                return res.json();
            })
            .then(data => setNodeData(data))
            .catch(err => {
                if (err.name !== 'AbortError') {
                    console.warn('[launch-panel] fetch failed for', name, err);
                }
                setNodeData(null);
            })
            .finally(() => clearTimeout(timeout));
        return () => { controller.abort(); clearTimeout(timeout); };
    }, [name]);

    // Use store node as fallback for live data
    const storeNode = nodes.value.get(name);
    const displayData = nodeData || storeNode;
    const isComposable = storeNode?.node_type === 'composable_node';
    const hasOwnLogs = storeNode?.has_own_logs;
    const containerName = (isComposable && !hasOwnLogs) ? storeNode?.container_name : null;

    return html`
        <div class="lp-node-detail">
            <div class="right-panel-tabs">
                <button class="tab-btn ${tab === 'info' ? 'active' : ''}"
                    onClick=${() => setTab('info')}>Info</button>
                <button class="tab-btn ${tab === 'params' ? 'active' : ''}"
                    onClick=${() => setTab('params')}>Params</button>
                <button class="tab-btn ${tab === 'topics' ? 'active' : ''}"
                    onClick=${() => setTab('topics')}>Topics</button>
                <button class="tab-btn ${tab === 'metrics' ? 'active' : ''}"
                    onClick=${() => setTab('metrics')}>Metrics</button>
                <button class="tab-btn ${tab === 'stdout' ? 'active' : ''}"
                    onClick=${() => setTab('stdout')}>stdout</button>
                <button class="tab-btn ${tab === 'stderr' ? 'active' : ''}"
                    onClick=${() => setTab('stderr')}>stderr</button>
            </div>
            <div class="right-panel-content">
                <div class="tab-content" style=${{ display: tab === 'info' ? 'flex' : 'none' }}>
                    <${InfoTab} nodeData=${displayData} />
                </div>
                <div class="tab-content" style=${{ display: tab === 'params' ? 'flex' : 'none' }}>
                    ${tab === 'params' && html`<${ParametersTab} nodeName=${name} key=${'params-' + name} />`}
                </div>
                <div class="tab-content" style=${{ display: tab === 'topics' ? 'flex' : 'none' }}>
                    ${tab === 'topics' && html`<${TopicsTab} nodeName=${name} key=${'topics-' + name} />`}
                </div>
                <div class="tab-content" style=${{ display: tab === 'metrics' ? 'flex' : 'none' }}>
                    ${tab === 'metrics' && html`<${MetricsTab} nodeName=${name} key=${'metrics-' + name} />`}
                </div>
                <div class="tab-content" style=${{ display: tab === 'stdout' ? 'flex' : 'none' }}>
                    ${tab === 'stdout' && html`<${LogTab} nodeName=${name} logType="stdout" containerName=${containerName} key=${'stdout-' + name} />`}
                </div>
                <div class="tab-content" style=${{ display: tab === 'stderr' ? 'flex' : 'none' }}>
                    ${tab === 'stderr' && html`<${LogTab} nodeName=${name} logType="stderr" containerName=${containerName} key=${'stderr-' + name} />`}
                </div>
            </div>
        </div>
    `;
}

// ── Main panel ──

export function LaunchPanel() {
    const sel = launchTreeSelection.value;

    const close = useCallback(() => {
        launchTreeSelection.value = null;
    }, []);

    useEffect(() => {
        const onKey = (e) => {
            if (e.key === 'Escape' && launchTreeSelection.value) {
                launchTreeSelection.value = null;
            }
        };
        document.addEventListener('keydown', onKey);
        return () => document.removeEventListener('keydown', onKey);
    }, []);

    if (!sel) return null;

    const title = sel.type === 'scope'
        ? (() => {
            const tree = launchTree.value;
            const scope = tree?.scopes?.[sel.id];
            return scope ? `${scope.pkg || '?'} ${scope.file}` : `Scope ${sel.id}`;
        })()
        : sel.name;

    const statusStr = sel.type === 'node'
        ? getStatusString(nodes.value.get(sel.name)?.status)
        : null;

    return html`
        <div class="right-panel open">
            <div class="right-panel-header">
                <span class="right-panel-title">
                    <span>${title}</span>
                    ${statusStr && html`<span class="state-badge ${statusStr}">${statusStr}</span>`}
                </span>
                <button class="close-btn" onClick=${close}>${'\u00D7'}</button>
            </div>
            ${sel.type === 'scope'
                ? html`<${ScopeDetail} scopeId=${sel.id} key=${'scope-' + sel.id} />`
                : html`<${NodeDetail} name=${sel.name} key=${'node-' + sel.name} />`
            }
        </div>
    `;
}
