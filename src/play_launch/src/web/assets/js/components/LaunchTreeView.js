// Launch Tree View — displays the launch file include tree with nodes.

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useCallback } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import {
    launchTree, launchTreeExpanded, launchTreeSelection,
    nodes, getStatusString,
} from '../store.js';

const html = htm.bind(h);

/** Toggle expand/collapse of a scope. */
function toggleExpand(scopeId, e) {
    if (e) e.stopPropagation();
    const expanded = new Set(launchTreeExpanded.value);
    if (expanded.has(scopeId)) {
        expanded.delete(scopeId);
    } else {
        expanded.add(scopeId);
    }
    launchTreeExpanded.value = expanded;
}

/** Expand all scopes. */
function expandAll(scopes) {
    launchTreeExpanded.value = new Set(scopes.map(s => s.id));
}

/** Collapse all scopes (keep root expanded). */
function collapseAll() {
    launchTreeExpanded.value = new Set([0]);
}

/** Check if a node has recent stderr activity. Returns 'hot'|'warm'|null. */
function getNodeActivity(nodeName) {
    const node = nodes.value.get(nodeName);
    if (!node || !node.stderr_last_modified || !node.stderr_size) return null;
    const elapsed = Math.floor(Date.now() / 1000) - node.stderr_last_modified;
    if (elapsed < 10) return 'hot';
    if (elapsed < 60) return 'warm';
    return null;
}

/** Check if any node in a subtree has recent activity. */
function getSubtreeActivity(scopeId, scopeChildren, scopeNodes) {
    // Check direct nodes
    const nodeNames = scopeNodes.get(scopeId) || [];
    for (const name of nodeNames) {
        const act = getNodeActivity(name);
        if (act === 'hot') return 'hot';
        if (act === 'warm') return 'warm';
    }
    // Check child scopes recursively
    const children = scopeChildren.get(scopeId) || [];
    let hasWarm = false;
    for (const childId of children) {
        const act = getSubtreeActivity(childId, scopeChildren, scopeNodes);
        if (act === 'hot') return 'hot';
        if (act === 'warm') hasWarm = true;
    }
    return hasWarm ? 'warm' : null;
}

/** Activity icon for a node — same pattern as NodeCard's StderrIcon. */
function ActivityIcon({ nodeName }) {
    const [, setTick] = useState(0);
    const node = nodes.value.get(nodeName);
    const mtime = node?.stderr_last_modified;
    const size = node?.stderr_size || 0;

    useEffect(() => {
        if (!mtime || size === 0) return;
        const elapsed = Math.floor(Date.now() / 1000) - mtime;
        if (elapsed >= 60) return;
        const nextBoundary = elapsed < 10 ? 10 : 60;
        const delay = (nextBoundary - elapsed) * 1000 + 200;
        const timer = setTimeout(() => setTick(t => t + 1), delay);
        return () => clearTimeout(timer);
    }, [mtime, size]);

    if (!mtime || size === 0) return null;
    const elapsed = Math.floor(Date.now() / 1000) - mtime;
    if (elapsed >= 60) return null;

    const statusStr = node ? getStatusString(node.status) : 'unknown';
    const isActive = statusStr === 'running' || statusStr === 'loaded';

    let cls = 'stderr-icon';
    if (elapsed < 10 && isActive) cls += ' hot jumping';
    else if (elapsed < 10) cls += ' hot';
    else cls += ' warm';

    return html`<span class=${cls}>${'\uD83D\uDCCB'}</span>`;
}

/** Activity icon for a collapsed scope — propagates from subtree. */
function ScopeActivityIcon({ scopeId, scopeChildren, scopeNodes }) {
    const [, setTick] = useState(0);

    // Re-render periodically to update activity state
    useEffect(() => {
        const timer = setInterval(() => setTick(t => t + 1), 5000);
        return () => clearInterval(timer);
    }, []);

    const activity = getSubtreeActivity(scopeId, scopeChildren, scopeNodes);
    if (!activity) return null;

    let cls = 'stderr-icon';
    if (activity === 'hot') cls += ' hot jumping';
    else cls += ' warm';

    return html`<span class=${cls}>${'\uD83D\uDCCB'}</span>`;
}

/** Get status CSS class for a node. */
function statusClass(name) {
    const node = nodes.value.get(name);
    if (!node) return 'stopped';
    return getStatusString(node.status);
}

/** Render a single scope row. */
function ScopeRow({ scope, depth, entityCount, isExpanded, hasChildren, scopeId, scopeChildren, scopeNodes }) {
    const sel = launchTreeSelection.value;
    const isSelected = sel?.type === 'scope' && sel?.id === scope.id;
    const pkg = scope.pkg || '(none)';

    const handleSelect = useCallback((e) => {
        e.stopPropagation();
        launchTreeSelection.value = { type: 'scope', id: scope.id };
    }, [scope.id]);

    const handleToggle = useCallback((e) => {
        toggleExpand(scope.id, e);
    }, [scope.id]);

    return html`
        <div class="lt-row lt-scope ${isSelected ? 'lt-selected' : ''}"
             style=${{ paddingLeft: `${depth * 20 + 8}px` }}
             onClick=${handleSelect}>
            <span class="lt-toggle" onClick=${handleToggle}>
                ${hasChildren ? (isExpanded ? '\u25BC' : '\u25B6') : '\u00A0'}
            </span>
            <span class="lt-pkg">${pkg}</span>
            <span class="lt-file">${scope.file}</span>
            ${scope.ns !== '/' && html`<span class="lt-ns">${scope.ns}</span>`}
            ${entityCount > 0 && html`<span class="lt-count">${entityCount}</span>`}
            ${!isExpanded && hasChildren && html`
                <${ScopeActivityIcon} scopeId=${scopeId}
                    scopeChildren=${scopeChildren} scopeNodes=${scopeNodes} />
            `}
        </div>
    `;
}

/** Render a node row under a scope. */
function NodeRow({ name, depth }) {
    const sel = launchTreeSelection.value;
    const isSelected = sel?.type === 'node' && sel?.name === name;
    const status = statusClass(name);
    const shortName = name.split('/').filter(Boolean).pop() || name;

    const handleClick = useCallback((e) => {
        e.stopPropagation();
        launchTreeSelection.value = { type: 'node', name };
    }, [name]);

    return html`
        <div class="lt-row lt-node ${isSelected ? 'lt-selected' : ''}"
             style=${{ paddingLeft: `${depth * 20 + 28}px` }}
             onClick=${handleClick}>
            <span class="lt-status-dot lt-status-${status}"></span>
            <span class="lt-node-name">${shortName}</span>
            <span class="lt-node-fqn">${name}</span>
            <${ActivityIcon} nodeName=${name} />
        </div>
    `;
}

/** Recursive tree renderer. */
function ScopeTree({ scopes, scopeChildren, scopeNodes, scopeId, depth }) {
    const expanded = launchTreeExpanded.value;
    const scope = scopes[scopeId];
    if (!scope) return null;

    const children = scopeChildren.get(scopeId) || [];
    const nodeNames = scopeNodes.get(scopeId) || [];
    const entityCount = nodeNames.length;
    const hasChildren = children.length > 0 || nodeNames.length > 0;
    const isExpanded = expanded.has(scopeId);

    return html`
        <${ScopeRow} scope=${scope} depth=${depth} entityCount=${entityCount}
            isExpanded=${isExpanded} hasChildren=${hasChildren}
            scopeId=${scopeId} scopeChildren=${scopeChildren} scopeNodes=${scopeNodes} />
        ${isExpanded && nodeNames.map(name => html`
            <${NodeRow} name=${name} depth=${depth + 1} key=${name} />
        `)}
        ${isExpanded && children.map(childId => html`
            <${ScopeTree}
                scopes=${scopes}
                scopeChildren=${scopeChildren}
                scopeNodes=${scopeNodes}
                scopeId=${childId}
                depth=${depth + 1}
                key=${childId} />
        `)}
    `;
}

export function LaunchTreeView() {
    const tree = launchTree.value;
    // Subscribe to nodes for activity icons
    nodes.value;

    if (!tree || !tree.scopes || tree.scopes.length === 0) {
        return html`
            <div class="lt-empty">
                <p>No launch tree data available.</p>
                <p class="lt-hint">Scope tracking requires a scoped record.json.</p>
            </div>
        `;
    }

    const scopes = tree.scopes;
    const nodeScopes = tree.node_scopes || {};

    // Build children map
    const scopeChildren = new Map();
    for (const s of scopes) {
        if (s.parent !== null && s.parent !== undefined) {
            if (!scopeChildren.has(s.parent)) scopeChildren.set(s.parent, []);
            scopeChildren.get(s.parent).push(s.id);
        }
    }

    // Build scope → node names map
    const scopeNodes = new Map();
    for (const [name, scopeId] of Object.entries(nodeScopes)) {
        if (!scopeNodes.has(scopeId)) scopeNodes.set(scopeId, []);
        scopeNodes.get(scopeId).push(name);
    }
    for (const names of scopeNodes.values()) names.sort();

    const roots = scopes.filter(s => s.parent === null || s.parent === undefined);

    // Expand all on first load
    useEffect(() => {
        if (scopes.length > 0 && launchTreeExpanded.value.size <= 1) {
            expandAll(scopes);
        }
    }, [scopes.length]);

    return html`
        <div class="lt-container">
            <div class="lt-header">
                <h2>Launch</h2>
                <span class="lt-summary">${scopes.length} scopes</span>
                <div class="lt-actions">
                    <button class="lt-btn" onClick=${() => expandAll(scopes)}>Expand All</button>
                    <button class="lt-btn" onClick=${collapseAll}>Collapse All</button>
                </div>
            </div>
            <div class="lt-tree">
                ${roots.map(root => html`
                    <${ScopeTree}
                        scopes=${scopes}
                        scopeChildren=${scopeChildren}
                        scopeNodes=${scopeNodes}
                        scopeId=${root.id}
                        depth=${0}
                        key=${root.id} />
                `)}
            </div>
        </div>
    `;
}
