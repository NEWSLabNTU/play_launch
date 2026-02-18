// NodeCard component — renders a single node in the node list.

import { h } from '../vendor/preact.module.js';
import { useState, useCallback, useEffect } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import { selectedNode, fetchNodes, fetchHealth, getStatusString } from '../store.js';

const html = htm.bind(h);

/** Get CSS class for the node card based on status. */
function getStatusClass(status) {
    return 'status-' + getStatusString(status);
}

/** Get the display label for node type. */
function getTypeLabel(nodeType) {
    switch (nodeType) {
        case 'node': return 'Node';
        case 'container': return 'Container';
        case 'composable_node': return 'Composable';
        default: return nodeType;
    }
}

/** Get CSS class for node type badge. */
function getTypeClass(nodeType) {
    switch (nodeType) {
        case 'node': return 'type-node';
        case 'container': return 'type-container';
        case 'composable_node': return 'type-composable';
        default: return '';
    }
}

/** Build clickable namespace segments from a ROS name. */
function RosName({ namespace, nodeName, onFilterNamespace }) {
    if (!namespace && !nodeName) return null;

    const fullName = namespace
        ? (namespace === '/'
            ? '/' + (nodeName || '')
            : namespace + '/' + (nodeName || ''))
        : '/' + (nodeName || '');

    const parts = fullName.split('/').filter(Boolean);
    if (parts.length === 0) return null;

    const segments = [];
    for (let i = 0; i < parts.length; i++) {
        if (i > 0 || fullName.startsWith('/')) {
            segments.push(html`<span class="ns-sep">/</span>`);
        }
        const isLast = i === parts.length - 1;
        const nsPath = '/' + parts.slice(0, i + 1).join('/');
        if (isLast) {
            segments.push(html`<span class="ns-leaf">${parts[i]}</span>`);
        } else {
            segments.push(html`<span class="ns-segment"
                onClick=${(e) => { e.stopPropagation(); onFilterNamespace(nsPath); }}
            >${parts[i]}</span>`);
        }
    }

    return html`<span class="node-ros-name">${segments}</span>`;
}

/** Stderr activity icon with tooltip. */
function StderrIcon({ node }) {
    const [showTooltip, setShowTooltip] = useState(false);
    const [tooltipPos, setTooltipPos] = useState({ top: 0, left: 0 });

    const mtime = node.stderr_last_modified;
    const size = node.stderr_size || 0;
    if (!mtime || size === 0) return null;

    const now = Math.floor(Date.now() / 1000);
    const elapsed = now - mtime;
    if (elapsed >= 60) return null;

    const statusStr = getStatusString(node.status);
    const isActive = statusStr === 'running' || statusStr === 'loaded';

    let iconClass = 'stderr-icon';
    if (elapsed < 10 && isActive) {
        iconClass += ' hot jumping';
    } else if (elapsed < 10) {
        iconClass += ' hot';
    } else {
        iconClass += ' warm';
    }

    const onMouseEnter = useCallback((e) => {
        const rect = e.target.getBoundingClientRect();
        let top = rect.bottom + 8;
        let left = rect.left;
        if (left + 400 > window.innerWidth) left = window.innerWidth - 410;
        if (top + 200 > window.innerHeight) top = rect.top - 208;
        setTooltipPos({ top, left });
        setShowTooltip(true);
    }, []);

    const onMouseLeave = useCallback(() => setShowTooltip(false), []);

    const preview = node.stderr_preview;

    return html`
        <span class=${iconClass} title="Hover to see recent stderr"
            onMouseEnter=${onMouseEnter} onMouseLeave=${onMouseLeave}>
            ${'📋'}
        </span>
        ${showTooltip && preview && preview.length > 0 && html`
            <div class="stderr-tooltip" style=${{ position: 'fixed', top: tooltipPos.top + 'px', left: tooltipPos.left + 'px', zIndex: 10000 }}>
                <div class="stderr-tooltip-header">Recent stderr:</div>
                <div class="stderr-tooltip-content">
                    ${preview.map(line => html`<div class="stderr-tooltip-line">${line}</div>`)}
                </div>
            </div>
        `}
    `;
}

/** Convert a boolean to a BoolParam path segment ("true" | "false").
 * @param {boolean} value
 * @returns {import('../types').BoolParam}
 */
function toBoolParam(value) {
    return value ? 'true' : 'false';
}

/** POST action helper — fires action and refreshes store. */
async function postAction(url) {
    try {
        const res = await fetch(url, { method: 'POST' });
        if (!res.ok) {
            const text = await res.text();
            throw new Error(text);
        }
    } catch (err) {
        alert('Action failed: ' + err.message);
    }
    // SSE will update store, but also do a full fetch for completeness
    setTimeout(() => { fetchNodes(); fetchHealth(); }, 500);
}

/** Process node action button (Start or Stop, mutually exclusive). */
function ProcessButton({ name, statusStr, pendingAction, setPendingAction }) {
    if (pendingAction === 'starting' || statusStr === 'pending') {
        return html`<button class="btn-pending" disabled>Starting...</button>`;
    }
    if (pendingAction === 'stopping') {
        return html`<button class="btn-pending" disabled>Stopping...</button>`;
    }
    if (statusStr === 'running') {
        return html`<button class="btn-stop" onClick=${() => {
            setPendingAction('stopping');
            postAction('/api/nodes/' + encodeURIComponent(name) + '/stop');
        }}>Stop</button>`;
    }
    // stopped, failed, or unknown → Start
    return html`<button class="btn-start" onClick=${() => {
        setPendingAction('starting');
        postAction('/api/nodes/' + encodeURIComponent(name) + '/start');
    }}>Start</button>`;
}

/** Composable node action button (Load or Unload, mutually exclusive). */
function ComposableButton({ name, statusStr, pendingAction, setPendingAction }) {
    if (pendingAction === 'loading' || statusStr === 'loading') {
        return html`<button class="btn-pending" disabled>Loading...</button>`;
    }
    if (pendingAction === 'unloading' || statusStr === 'unloading') {
        return html`<button class="btn-pending" disabled>Unloading...</button>`;
    }
    if (statusStr === 'loaded') {
        return html`<button class="btn-unload" onClick=${() => {
            setPendingAction('unloading');
            postAction('/api/nodes/' + encodeURIComponent(name) + '/unload');
        }}>Unload</button>`;
    }
    if (statusStr === 'blocked') {
        return null; // container not started — no button
    }
    // unloaded, failed, pending, or unknown → Load
    return html`<button class="btn-load" onClick=${() => {
        setPendingAction('loading');
        postAction('/api/nodes/' + encodeURIComponent(name) + '/load');
    }}>Load</button>`;
}

/** NodeCard component. */
export function NodeCard({ node, isChild, onFilterNamespace, onViewNode }) {
    const name = node.name;
    const statusStr = getStatusString(node.status);
    const isProcess = node.node_type === 'node' || node.node_type === 'container';
    const isComposable = node.node_type === 'composable_node';
    const isContainer = node.node_type === 'container';
    const isSelected = selectedNode.value === name;

    const [pendingAction, setPendingAction] = useState(null);

    // Clear pending action when status changes (SSE delivered the update)
    useEffect(() => {
        setPendingAction(null);
    }, [statusStr]);

    let cardClass = 'node-card ' + getStatusClass(node.status);
    if (isChild) cardClass += ' child-node';
    if (isContainer) cardClass += ' container-node';
    if (isSelected) cardClass += ' selected';

    const handleView = useCallback(() => {
        onViewNode(name);
    }, [name, onViewNode]);

    return html`
        <div class=${cardClass} data-node=${name}>
            <div class="node-info">
                <div class="node-header">
                    <span class="node-type ${getTypeClass(node.node_type)}">${getTypeLabel(node.node_type)}</span>
                    <span class="node-name">${name}</span>
                    ${node.pid != null && html`<span class="node-pid">PID ${node.pid}</span>`}
                    <${StderrIcon} node=${node} />
                </div>
                <div class="node-meta">
                    <${RosName} namespace=${node.namespace} nodeName=${node.node_name} onFilterNamespace=${onFilterNamespace} />
                </div>
            </div>
            <div class="node-controls">
                ${isProcess && node.respawn_enabled !== undefined && html`
                    <label class="toggle-checkbox" onClick=${(e) => e.stopPropagation()}>
                        <input type="checkbox" checked=${node.respawn_enabled}
                            onChange=${(e) => postAction('/api/nodes/' + encodeURIComponent(name) + '/respawn/' + toBoolParam(e.target.checked))} />
                        <span class="toggle-label">Auto-restart</span>
                    </label>
                `}
                ${isComposable && node.auto_load !== undefined && html`
                    <label class="toggle-checkbox" onClick=${(e) => e.stopPropagation()}>
                        <input type="checkbox" checked=${node.auto_load}
                            onChange=${(e) => postAction('/api/nodes/' + encodeURIComponent(name) + '/auto-load/' + toBoolParam(e.target.checked))} />
                        <span class="toggle-label">Auto-load</span>
                    </label>
                `}
                ${isProcess && html`
                    <${ProcessButton} name=${name} statusStr=${statusStr}
                        pendingAction=${pendingAction} setPendingAction=${setPendingAction} />
                `}
                ${isComposable && html`
                    <${ComposableButton} name=${name} statusStr=${statusStr}
                        pendingAction=${pendingAction} setPendingAction=${setPendingAction} />
                `}
                ${isContainer && statusStr === 'running' && html`
                    <button class="btn-load-all" onClick=${() => postAction('/api/nodes/' + encodeURIComponent(name) + '/load-all')}>
                        Load All
                    </button>
                    <button class="btn-unload-all" onClick=${() => postAction('/api/nodes/' + encodeURIComponent(name) + '/unload-all')}>
                        Unload All
                    </button>
                `}
                <button class="btn-view" onClick=${handleView}>View</button>
            </div>
        </div>
    `;
}
