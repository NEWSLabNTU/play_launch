// NodeList component — renders the grouped, sorted, filtered node list.

import { h } from '../vendor/preact.module.js';
import { useState, useMemo, useCallback, useEffect } from '../vendor/hooks.module.js';
import { useSignal } from '../vendor/signals.module.js';
import htm from '../vendor/htm.module.js';
import { nodeList, selectedNode, nodePanelOpen, activeTab, getStatusString, fetchNodes, statusFacet } from '../store.js';
import { NodeCard } from './NodeCard.js';

const html = htm.bind(h);

/** Status priority for sorting. */
function statusPriority(node) {
    const val = getStatusString(node.status);
    if (val === 'running' || val === 'loaded') return 0;
    if (val === 'loading' || val === 'unloading') return 1;
    if (val === 'failed') return 2;
    if (val === 'stopped' || val === 'unloaded') return 3;
    return 4;
}

/** Sort comparator factory. */
function makeSorter(sortBy) {
    return (a, b) => {
        if (sortBy === 'name') return a.name.localeCompare(b.name);
        if (sortBy === 'name-desc') return b.name.localeCompare(a.name);
        if (sortBy === 'type') {
            const tc = (a.node_type || '').localeCompare(b.node_type || '');
            return tc !== 0 ? tc : a.name.localeCompare(b.name);
        }
        if (sortBy === 'status') {
            const sp = statusPriority(a) - statusPriority(b);
            return sp !== 0 ? sp : a.name.localeCompare(b.name);
        }
        if (sortBy === 'activity') {
            const ta = a.stderr_last_modified || 0;
            const tb = b.stderr_last_modified || 0;
            if (ta !== tb) return tb - ta; // most recent first
            return a.name.localeCompare(b.name);
        }
        return 0;
    };
}

/** Map a node's status string onto a facet bucket (phase-50). */
function facetOf(node) {
    const val = getStatusString(node.status);
    if (val === 'running' || val === 'loaded') return 'active';
    if (val === 'loading' || val === 'unloading' || val === 'pending') return 'transitioning';
    if (val === 'failed') return 'failed';
    if (val === 'blocked') return 'blocked';
    return 'inactive'; // stopped, unloaded, unknown
}

/** Check if a node matches the search term + facet + kind. */
function matchesFilter(node, term, facet, kind) {
    if (facet && facet !== 'all' && facetOf(node) !== facet) return false;
    if (kind && kind !== 'all' && node.node_type !== kind) return false;
    if (!term) return true;
    const lower = term.toLowerCase();
    const name = (node.name || '').toLowerCase();
    const rosName = buildRosName(node).toLowerCase();
    const pkg = (node.package || '').toLowerCase();
    const exec = (node.executable || '').toLowerCase();
    return name.includes(lower) || rosName.includes(lower)
        || pkg.includes(lower) || exec.includes(lower);
}

function buildRosName(node) {
    const ns = node.namespace;
    const nn = node.node_name;
    if (!ns && !nn) return '';
    if (!ns) return '/' + (nn || '');
    if (ns === '/') return '/' + (nn || '');
    return ns + '/' + (nn || '');
}

/** Facet chip strip with live counts (phase-50). */
function FacetBar({ nodes, facet, setFacet }) {
    const counts = { all: nodes.length, active: 0, transitioning: 0, failed: 0, inactive: 0, blocked: 0 };
    for (const n of nodes) counts[facetOf(n)]++;
    const chips = [
        ['all', 'All'], ['active', 'Active'], ['transitioning', 'Transitioning'],
        ['failed', 'Failed'], ['inactive', 'Inactive'], ['blocked', 'Blocked'],
    ];
    return html`
        <div class="facet-bar">
            ${chips.map(([key, label]) => (key === 'all' || counts[key] > 0) && html`
                <button key=${key}
                    class="facet-chip facet-${key} ${facet === key ? 'active' : ''}"
                    onClick=${() => setFacet(facet === key ? 'all' : key)}>
                    ${label} <span class="facet-count">${counts[key]}</span>
                </button>
            `)}
        </div>
    `;
}

export function NodeList() {
    const [sortBy, setSortBy] = useState('name');
    const [filterTerm, setFilterTerm] = useState('');
    const [treeView, setTreeView] = useState(true);
    const [collapsedNs, setCollapsedNs] = useState(new Set());
    const facet = statusFacet.value;
    const setFacet = useCallback((f) => { statusFacet.value = f; }, []);

    // Poll node data periodically when sorting by activity so timestamps stay fresh.
    useEffect(() => {
        if (sortBy !== 'activity') return;
        const id = setInterval(fetchNodes, 3000);
        return () => clearInterval(id);
    }, [sortBy]);

    const allNodes = nodeList.value;

    const grouped = useMemo(() => {
        const sorter = makeSorter(sortBy);
        const matches = (n) => matchesFilter(n, filterTerm, facet, 'all');

        if (!treeView) {
            // Flat mode: all nodes sorted together, no parent-child grouping
            const flat = allNodes.filter(matches);
            flat.sort(sorter);
            return flat.map(node => ({ node, isChild: false }));
        }

        // Tree mode: namespace groups; containers with their composable
        // children grouped together. All keying by canonical id (phase-50).
        const containerChildren = new Map();
        const containers = [];
        const regularNodes = [];

        for (const node of allNodes) {
            if (node.node_type === 'container') {
                containers.push(node);
                if (!containerChildren.has(node.id)) {
                    containerChildren.set(node.id, []);
                }
            } else if (node.node_type === 'composable_node' && node.container_id) {
                if (!containerChildren.has(node.container_id)) {
                    containerChildren.set(node.container_id, []);
                }
                containerChildren.get(node.container_id).push(node);
            } else {
                regularNodes.push(node);
            }
        }

        containerChildren.forEach(children => children.sort(sorter));

        // Build top-level entries: regular nodes + container groups.
        // For activity sort, compute effective activity per container group
        // (max of container + children) so groups interleave with regular nodes.
        const topLevel = []; // { node, type: 'regular'|'container' }

        for (const node of regularNodes) {
            if (matches(node)) {
                topLevel.push({ node, type: 'regular' });
            }
        }
        for (const container of containers) {
            const children = containerChildren.get(container.id) || [];
            const containerMatches = matches(container);
            const matchingChildren = children.filter(matches);
            if (containerMatches || matchingChildren.length > 0) {
                topLevel.push({ node: container, type: 'container', children, containerMatches, matchingChildren });
            }
        }

        if (sortBy === 'activity') {
            // Effective activity: for containers, max across group members
            const effectiveActivity = (entry) => {
                let t = entry.node.stderr_last_modified || 0;
                if (entry.type === 'container') {
                    for (const child of (containerChildren.get(entry.node.id) || [])) {
                        t = Math.max(t, child.stderr_last_modified || 0);
                    }
                }
                return t;
            };
            topLevel.sort((a, b) => {
                const ta = effectiveActivity(a);
                const tb = effectiveActivity(b);
                if (ta !== tb) return tb - ta;
                return a.node.name.localeCompare(b.node.name);
            });
        } else {
            topLevel.sort((a, b) => sorter(a.node, b.node));
        }

        // Group top-level entries by namespace (collapsible headers),
        // preserving the chosen sort inside each group. Skipped under
        // activity sort — interleaving matters more there.
        const useNsGroups = sortBy !== 'activity';
        const emit = (result, entry) => {
            result.push({ node: entry.node, isChild: false });
            if (entry.type === 'container') {
                const childrenToShow = entry.containerMatches ? entry.children : entry.matchingChildren;
                for (const child of childrenToShow) {
                    result.push({ node: child, isChild: true });
                }
            }
        };

        const result = [];
        if (!useNsGroups) {
            for (const entry of topLevel) emit(result, entry);
            return result;
        }

        const nsGroups = new Map(); // ns -> entries, insertion keeps sort inside group
        for (const entry of topLevel) {
            const ns = entry.node.namespace || '/';
            if (!nsGroups.has(ns)) nsGroups.set(ns, []);
            nsGroups.get(ns).push(entry);
        }
        for (const ns of Array.from(nsGroups.keys()).sort()) {
            const entries = nsGroups.get(ns);
            result.push({ nsHeader: ns, count: entries.length });
            if (collapsedNs.has(ns)) continue;
            for (const entry of entries) emit(result, entry);
        }
        return result;
    }, [allNodes, sortBy, filterTerm, facet, treeView, collapsedNs]);

    const onFilterNamespace = useCallback((ns) => {
        setFilterTerm(ns);
    }, []);

    const onViewNode = useCallback((name) => {
        selectedNode.value = name;
        nodePanelOpen.value = true;
        activeTab.value = activeTab.peek() || 'stderr';
    }, []);

    return html`
        <div class="nodes-view" style="display:block;">
            <div class="view-header">
                <div class="sort-controls">
                    <label for="sort-by">Sort by:</label>
                    <select id="sort-by" value=${sortBy} onChange=${(e) => setSortBy(e.target.value)}>
                        <option value="name">Name (A-Z)</option>
                        <option value="name-desc">Name (Z-A)</option>
                        <option value="type">Type</option>
                        <option value="status">Status</option>
                        <option value="activity">Last Activity</option>
                    </select>
                    <label class="toggle-checkbox tree-toggle">
                        <input type="checkbox" checked=${treeView}
                            onChange=${(e) => setTreeView(e.target.checked)} />
                        <span class="toggle-label">Tree</span>
                    </label>
                </div>
                <input type="text" class="search-box" id="search" placeholder="Filter nodes (name, ns, package, exec)..."
                    value=${filterTerm} onInput=${(e) => setFilterTerm(e.target.value)} />
                <${BulkOperations} />
            </div>
            <${FacetBar} nodes=${allNodes} facet=${facet} setFacet=${setFacet} />
            <div class="node-list">
                ${grouped.length === 0 && html`<div class="no-nodes">No nodes found</div>`}
                ${grouped.map((entry) => entry.nsHeader !== undefined ? html`
                    <div key=${'ns:' + entry.nsHeader} class="ns-group-header"
                        onClick=${() => {
                            const next = new Set(collapsedNs);
                            if (next.has(entry.nsHeader)) next.delete(entry.nsHeader);
                            else next.add(entry.nsHeader);
                            setCollapsedNs(next);
                        }}>
                        <span class="ns-group-arrow">${collapsedNs.has(entry.nsHeader) ? '▸' : '▾'}</span>
                        <span class="ns-group-name">${entry.nsHeader}</span>
                        <span class="ns-group-count">${entry.count}</span>
                    </div>
                ` : html`
                    <${NodeCard} key=${entry.node.id} node=${entry.node} isChild=${entry.isChild}
                        onFilterNamespace=${onFilterNamespace} onViewNode=${onViewNode} />
                `)}
            </div>
        </div>
    `;
}

function BulkOperations() {
    const startAll = useCallback(async () => {
        if (!confirm('Start all nodes?')) return;
        try {
            await fetch('/api/nodes/start-all', { method: 'POST' });
        } catch (err) {
            alert('Failed to start all nodes: ' + err.message);
        }
    }, []);

    const stopAll = useCallback(async () => {
        if (!confirm('Stop all nodes?')) return;
        try {
            await fetch('/api/nodes/stop-all', { method: 'POST' });
        } catch (err) {
            alert('Failed to stop all nodes: ' + err.message);
        }
    }, []);

    return html`
        <div class="bulk-operations">
            <button class="btn-bulk btn-success" onClick=${startAll}>${'▶'} Start All</button>
            <button class="btn-bulk btn-danger" onClick=${stopAll}>${'■'} Stop All</button>
        </div>
    `;
}
