// Launch Tree View — displays the launch file include tree with nodes.

import { useState, useEffect, useCallback, useRef } from '../vendor/hooks.module.js';
import { html } from '../html.js';
import {
    launchTree, launchTreeExpanded, launchTreeSelection,
    nodes, getStatusString, getNodeByKey, memberFqn } from '../store.js';
import { clickable } from '../a11y.js';
import { StderrIcon } from './StderrIcon.js';
import {
    pending, pendingReport, pendingCount, selectionInfo, effectiveOn, setSwitches,
    applyPending, discardPending, fetchSelection, reloadTree, copyCommand,
    exportOpen, exportResult, exportSelection,
} from '../selection.js';

/**
 * Export what is on as a launch file (phase 106 W5). The server writes it
 * into this run's log directory and says how to run it; the zip is for
 * another machine.
 */
function ExportPanel() {
    const formRef = useRef(null);
    const nameRef = useRef(null);
    if (!exportOpen.value) return null;
    const r = exportResult.value;
    const run = () => exportSelection(formRef.current.value, nameRef.current.value.trim());
    return html`
        <div class="lt-pending lt-export" role="region" aria-label="Export">
            <div class="lt-pending-head">
                <strong>Export what is on:</strong>
                <select ref=${formRef} class="lt-export-form" aria-label="Form">
                    <option value="include">include (follows edits to the included files)</option>
                    <option value="self-contained">self-contained (a snapshot)</option>
                    <option value="command">command (one node)</option>
                </select>
                <input ref=${nameRef} class="lt-export-name" placeholder="name (letters, digits, _ -)"
                    aria-label="Name" />
                <span class="lt-pending-btns">
                    <button class="lt-btn lt-btn-primary" disabled=${!!(r && r.busy)} onClick=${run}>Export</button>
                    <button class="lt-btn" onClick=${() => { exportOpen.value = false; exportResult.value = null; }}>Close</button>
                </span>
            </div>
            ${r && r.busy && html`<div class="lt-pending-list">Resolving and checking the export…</div>`}
            ${r && r.error && html`<div class="lt-pending-error">${r.error}</div>`}
            ${r && r.file && html`
                <div class="lt-pending-list"><span class="lt-pending-label">Written on the host:</span>
                    <span class="lt-pending-item">${r.file}</span>
                    <a class="lt-btn" href=${r.zip} download>Download zip</a></div>
                <div class="lt-pending-list"><span class="lt-pending-label">Run it with:</span></div>
                ${r.run.map((c) => html`<pre class="lt-export-cmd">${c}</pre>`)}
                ${r.opened.length > 0 && html`<ul class="lt-pending-warnings">${r.opened.map((o) =>
                    html`<li>written one level down (the include alone did not match): ${o}</li>`)}</ul>`}
                ${r.lost.length > 0 && html`<ul class="lt-pending-warnings">${r.lost.map((l) =>
                    html`<li>not expressible in XML, noted in a comment: ${l}</li>`)}</ul>`}
                ${r.differences.length > 0 && html`<ul class="lt-pending-warnings">${r.differences.map((d) =>
                    html`<li>does not reproduce the selection exactly: ${d}</li>`)}</ul>`}
            `}
        </div>
    `;
}

/**
 * For a file included more than once, the argument that tells this
 * inclusion apart (`part=a`), as `resolve --explain` labels it; else null.
 */
function repeatLabel(scope, scopes) {
    if (!scope.origin) return null;
    const same = scopes.filter((s) => s.origin && s.origin.file === scope.origin.file
        && s.origin.pkg === scope.origin.pkg);
    if (same.length < 2) return null;
    const parent = scope.parent != null ? scopes[scope.parent] : null;
    const keys = Object.keys(scope.args || {}).filter((k) => same.every(
        (o) => o === scope || (o.args || {})[k] !== scope.args[k]));
    // Prefer an argument the include passed (it differs from the parent's).
    keys.sort((a, b) => {
        const pa = parent && (parent.args || {})[a] === scope.args[a] ? 1 : 0;
        const pb = parent && (parent.args || {})[b] === scope.args[b] ? 1 : 0;
        return pa - pb || a.length - b.length || a.localeCompare(b);
    });
    return keys.length ? `${keys[0]}=${scope.args[keys[0]]}` : null;
}

/** Every member id in a scope's subtree. */
function subtreeMembers(scopeId, scopeChildren, scopeNodes) {
    const out = [...(scopeNodes.get(scopeId) || [])];
    for (const c of scopeChildren.get(scopeId) || []) {
        out.push(...subtreeMembers(c, scopeChildren, scopeNodes));
    }
    return out;
}

/**
 * The on/off switch of a node or a whole include (phase 106). An include's
 * switch shows `on`, `off` or `partly` over its subtree; a click turns the
 * whole subtree on unless all of it already is.
 */
function Switch({ ids, label }) {
    pending.value; // re-render on edits
    if (ids.length === 0) return null;
    const on = ids.filter((id) => effectiveOn(id)).length;
    const state = on === 0 ? 'off' : on === ids.length ? 'on' : 'partly';
    const edited = ids.some((id) => pending.value.has(id));
    const toggle = (e) => {
        e.stopPropagation();
        setSwitches(ids, state !== 'on');
    };
    return html`
        <span class="lt-switch lt-switch-${state} ${edited ? 'lt-switch-pending' : ''}"
              role="switch" aria-checked=${state === 'on' ? 'true' : state === 'off' ? 'false' : 'mixed'}
              aria-label=${`${label}: ${state}${edited ? ' (pending)' : ''}`}
              title=${state === 'on' ? 'On: click to switch off' : 'Off or partly on: click to switch on'}
              ...${clickable(toggle)}></span>
    `;
}

/** The pending change, its evaluation, and Apply/Discard. */
function PendingBar() {
    const n = pendingCount.value;
    if (n === 0) return null;
    const r = pendingReport.value;
    return html`
        <div class="lt-pending" role="region" aria-label="Pending change">
            <div class="lt-pending-head">
                <strong>Pending:</strong>
                ${r && !r.error
                    ? html`<span>start ${r.start.length}, stop ${r.stop.length}${
                        r.restart.length ? `, restart ${r.restart.length}` : ''}</span>`
                    : html`<span>${n} switch(es) edited</span>`}
                <span class="lt-pending-btns">
                    <button class="lt-btn lt-btn-primary" disabled=${!!(r && r.error)}
                        title="Start and stop only what changed"
                        onClick=${() => applyPending('live')}>Apply live</button>
                    <button class="lt-btn" disabled=${!!(r && r.error)}
                        title="Stop everything that runs, then start the new set from nothing"
                        onClick=${() => applyPending('fresh')}>Apply fresh</button>
                    <button class="lt-btn" onClick=${discardPending}>Discard</button>
                </span>
            </div>
            ${r && r.error && html`<div class="lt-pending-error">${r.error}</div>`}
            ${r && !r.error && r.added.length > 0 && html`
                <div class="lt-pending-list"><span class="lt-pending-label">Kept for their composables:</span>
                    ${r.added.map((a) => html`<span class="lt-pending-item">${memberFqn(a.container)} (${a.needed_by})</span>`)}
                </div>`}
            ${r && !r.error && r.cut_inputs.length > 0 && html`
                <div class="lt-pending-list"><span class="lt-pending-label">Inputs cut:</span>
                    ${r.cut_inputs.map((c) => html`<span class="lt-pending-item"
                        title=${'published only by ' + c.publishers.join(', ')}>${c.topic} (${c.subscribers})</span>`)}
                </div>`}
            ${r && !r.error && r.warnings.length > 0 && html`
                <ul class="lt-pending-warnings">${r.warnings.map((w) => html`<li>${w}</li>`)}</ul>`}
        </div>
    `;
}

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

/** Expand this scope and all its descendants. */
function expandSubtree(scopeId, scopeChildren) {
    const expanded = new Set(launchTreeExpanded.value);
    const stack = [scopeId];
    while (stack.length > 0) {
        const id = stack.pop();
        expanded.add(id);
        const children = scopeChildren.get(id) || [];
        for (const child of children) stack.push(child);
    }
    launchTreeExpanded.value = expanded;
}

/** Expand this scope but collapse all descendant scopes. */
function collapseSubtree(scopeId, scopeChildren) {
    const expanded = new Set(launchTreeExpanded.value);
    // Ensure this scope is open (so user sees immediate children)
    expanded.add(scopeId);
    // Collapse all descendants
    const stack = [...(scopeChildren.get(scopeId) || [])];
    while (stack.length > 0) {
        const id = stack.pop();
        expanded.delete(id);
        const children = scopeChildren.get(id) || [];
        for (const child of children) stack.push(child);
    }
    launchTreeExpanded.value = expanded;
}

/**
 * Derive a display label for a group scope.
 * The group scope's own `ns` is captured at entry (before push-ros-namespace),
 * so it typically equals its parent's ns.  We look at direct child scopes
 * to find the namespace segment(s) introduced by the group.
 */
function getGroupLabel(scope, scopes, scopeChildren) {
    const groupNs = scope.ns;

    // Check direct children for namespace differences
    const childIds = scopeChildren.get(scope.id) || [];
    const childNamespaces = [];
    for (const cid of childIds) {
        const cs = scopes[cid];
        if (cs && cs.ns !== groupNs) childNamespaces.push(cs.ns);
    }

    if (childNamespaces.length > 0) {
        // Pick the shortest differing namespace (closest to the group's ns push)
        childNamespaces.sort((a, b) => a.length - b.length);
        const childNs = childNamespaces[0];
        // Compute relative part
        const prefix = groupNs === '/' ? '/' : groupNs + '/';
        const rel = childNs.startsWith(prefix)
            ? childNs.slice(prefix.length - 1)  // keep leading /
            : childNs;
        return 'group ' + rel;
    }

    return 'group';
}

/** Check if a node has recent stderr activity. Returns 'hot'|'warm'|null. */
function getNodeActivity(nodeName) {
    const node = getNodeByKey(nodeName);
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

/** Activity icon for a node: the node card's. */
function ActivityIcon({ nodeName }) {
    return html`<${StderrIcon} node=${getNodeByKey(nodeName)} />`;
}

/** Activity icon for a scope — propagates from subtree nodes. */
function ScopeActivityIcon({ scopeId, scopeChildren, scopeNodes }) {
    const [, setTick] = useState(0);

    const activity = getSubtreeActivity(scopeId, scopeChildren, scopeNodes);

    // Re-render at phase boundaries (hot→warm at 10s, warm→gone at 60s)
    useEffect(() => {
        if (!activity) return;
        const interval = activity === 'hot' ? 2000 : 5000;
        const timer = setInterval(() => setTick(t => t + 1), interval);
        return () => clearInterval(timer);
    }, [activity]);

    if (!activity) return null;

    let cls = 'stderr-icon';
    if (activity === 'hot') cls += ' hot jumping';
    else cls += ' warm';

    return html`<span class=${cls}>${'\uD83D\uDCCB'}</span>`;
}

/** Get status string for a node. */
function statusClass(name) {
    const node = getNodeByKey(name);
    if (!node) return 'stopped';
    return getStatusString(node.status);
}

/** Compute aggregate status for a subtree: { running, loaded, failed, total }. */
function getSubtreeStatus(scopeId, scopeChildren, scopeNodes) {
    const counts = { running: 0, loaded: 0, failed: 0, stopped: 0, total: 0 };

    // Direct nodes
    const nodeNames = scopeNodes.get(scopeId) || [];
    for (const name of nodeNames) {
        counts.total++;
        const s = statusClass(name);
        if (s === 'running') counts.running++;
        else if (s === 'loaded') counts.loaded++;
        else if (s === 'failed') counts.failed++;
        else counts.stopped++;
    }

    // Recurse into children
    const children = scopeChildren.get(scopeId) || [];
    for (const childId of children) {
        const child = getSubtreeStatus(childId, scopeChildren, scopeNodes);
        counts.running += child.running;
        counts.loaded += child.loaded;
        counts.failed += child.failed;
        counts.stopped += child.stopped;
        counts.total += child.total;
    }

    return counts;
}

/** Render a single scope row with aggregate status. */
function ScopeRow({ scope, scopes, depth, isExpanded, hasChildren, scopeId, scopeChildren, scopeNodes }) {
    const sel = launchTreeSelection.value;
    const isSelected = sel?.type === 'scope' && sel?.id === scope.id;
    const isGroup = !scope.origin;
    const pkg = scope.origin?.pkg || '(none)';
    const file = scope.origin?.file || null;

    const agg = getSubtreeStatus(scopeId, scopeChildren, scopeNodes);

    // Has child scopes (not just nodes) — determines if subtree button is shown
    const hasChildScopes = (scopeChildren.get(scopeId) || []).length > 0;

    const handleSelect = useCallback((e) => {
        e.stopPropagation();
        launchTreeSelection.value = { type: 'scope', id: scope.id };
    }, [scope.id]);

    const handleToggle = useCallback((e) => {
        toggleExpand(scope.id, e);
    }, [scope.id]);

    const handleDblClick = useCallback((e) => {
        e.stopPropagation();
        e.preventDefault();
        toggleExpand(scope.id);
    }, [scope.id]);

    const handleExpandSubtree = useCallback((e) => {
        e.stopPropagation();
        expandSubtree(scopeId, scopeChildren);
    }, [scopeId, scopeChildren]);

    const handleCollapseSubtree = useCallback((e) => {
        e.stopPropagation();
        collapseSubtree(scopeId, scopeChildren);
    }, [scopeId, scopeChildren]);

    let healthClass = '';
    if (agg.total > 0) {
        if (agg.failed > 0) healthClass = 'lt-scope-has-failed';
        else if (agg.running + agg.loaded === agg.total) healthClass = 'lt-scope-all-ok';
    }

    return html`
        <div class="lt-row lt-scope ${isGroup ? 'lt-scope-group' : ''} ${isSelected ? 'lt-selected' : ''} ${healthClass}"
             style=${{ paddingLeft: `${depth * 20 + 8}px` }}
             ...${clickable(handleSelect)}
             onDblClick=${handleDblClick}>
            <${Switch} ids=${subtreeMembers(scopeId, scopeChildren, scopeNodes)}
                label=${file || 'group'} />
            <span class="lt-toggle" ...${clickable(handleToggle)}>
                ${hasChildren ? (isExpanded ? '\u25BC' : '\u25B6') : '\u00A0'}
            </span>
            ${isGroup ? html`
                <span class="lt-group-label">${getGroupLabel(scope, scopes, scopeChildren)}</span>
            ` : html`
                <span class="lt-pkg">${pkg}</span>
                <span class="lt-file">${file}</span>
                ${repeatLabel(scope, scopes) && html`<span class="lt-badge">${repeatLabel(scope, scopes)}</span>`}
            `}
            ${!isGroup && scope.ns !== '/' && html`<span class="lt-ns">${scope.ns}</span>`}
            <${ScopeActivityIcon} scopeId=${scopeId}
                scopeChildren=${scopeChildren} scopeNodes=${scopeNodes} />
            ${hasChildScopes && html`
                <span class="lt-subtree-btns">
                    <span class="lt-subtree-btn" ...${clickable(handleExpandSubtree)}
                          title="Expand all subtrees">${'\u229E'}</span>
                    <span class="lt-subtree-btn" ...${clickable(handleCollapseSubtree)}
                          title="Collapse all subtrees">${'\u229F'}</span>
                </span>
            `}
            ${agg.total > 0 && html`
                <span class="lt-agg">
                    <span class="lt-agg-ok">${agg.running + agg.loaded}</span>
                    <span class="lt-agg-sep">/</span>
                    <span class="lt-agg-total">${agg.total}</span>
                    ${agg.failed > 0 && html`
                        <span class="lt-agg-failed">${agg.failed} failed</span>
                    `}
                </span>
            `}
        </div>
    `;
}

/** Render a node row with type-aware styling. */
function NodeRow({ name, depth }) {
    const sel = launchTreeSelection.value;
    const isSelected = sel?.type === 'node' && sel?.name === name;
    const storeNode = getNodeByKey(name);
    const status = storeNode ? getStatusString(storeNode.status) : 'stopped';
    const nodeType = storeNode?.node_type || 'node';
    const fqn = memberFqn(name);
    const shortName = fqn.split('/').filter(Boolean).pop() || fqn;

    const handleClick = useCallback((e) => {
        e.stopPropagation();
        launchTreeSelection.value = { type: 'node', name };
    }, [name]);

    // Type-specific CSS class
    const typeClass = nodeType === 'container' ? 'lt-type-container'
        : nodeType === 'composable_node' ? 'lt-type-composable'
        : 'lt-type-node';

    // Container name for composable nodes (cross-reference)
    const containerRef = nodeType === 'composable_node'
        ? storeNode?.target_container || storeNode?.container_name
        : null;

    const excluded = !!storeNode?.excluded;
    const changed = (selectionInfo.value?.changed || []).includes(name);
    return html`
        <div class="lt-row lt-node lt-node-${status} ${typeClass} ${isSelected ? 'lt-selected' : ''} ${excluded ? 'lt-excluded' : ''}"
             style=${{ paddingLeft: `${depth * 20 + 8}px` }}
             ...${clickable(handleClick)}>
            <${Switch} ids=${[name]} label=${fqn} />
            <span class="lt-toggle">${'\u00A0'}</span>
            <span class="lt-dot ${typeClass} lt-status-${status}"></span>
            <span class="lt-node-name">${shortName}</span>
            ${containerRef && html`
                <span class="lt-container-ref">${'\u2192'} ${containerRef}</span>
            `}
            <span class="lt-node-fqn">${fqn}</span>
            ${excluded && html`<span class="lt-badge lt-badge-excluded" title="Not started in this run; switch it on to start it">Excluded</span>`}
            ${changed && html`<span class="lt-badge lt-badge-changed" title="Its resolution changed at the last Reload; Apply restarts it">changed</span>`}
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
    const hasChildren = children.length > 0 || nodeNames.length > 0;
    const isExpanded = expanded.has(scopeId);

    return html`
        <${ScopeRow} scope=${scope} scopes=${scopes} depth=${depth}
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

    useEffect(() => { fetchSelection(); }, []);

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
                    <button class="lt-btn" onClick=${copyCommand}
                        title="Copy the launch command, with --only/--exclude for what is on now">Copy command</button>
                    ${selectionInfo.value?.export && html`
                        <button class="lt-btn" onClick=${() => { exportOpen.value = !exportOpen.value; }}
                            title="Write what is on as a launch file, to run later or elsewhere">Export…</button>`}
                    ${selectionInfo.value?.reload && html`
                        <button class="lt-btn" onClick=${reloadTree}
                            title="Re-read the launch files with the same arguments; changed nodes are restarted by the next Apply">Reload</button>`}
                </div>
            </div>
            <${PendingBar} />
            <${ExportPanel} />
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
