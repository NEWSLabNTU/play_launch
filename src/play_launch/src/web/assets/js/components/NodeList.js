// NodeList component — renders the grouped, sorted, filtered node list.

import { h } from '../vendor/preact.module.js';
import { useState, useMemo, useCallback } from '../vendor/hooks.module.js';
import { useSignal } from '../vendor/signals.module.js';
import htm from '../vendor/htm.module.js';
import { nodeList, selectedNode, panelOpen, activeTab, getStatusString } from '../store.js';
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
        return 0;
    };
}

/** Check if a node matches the search term. */
function matchesFilter(node, term) {
    if (!term) return true;
    const lower = term.toLowerCase();
    const name = (node.name || '').toLowerCase();
    const rosName = buildRosName(node).toLowerCase();
    return name.includes(lower) || rosName.includes(lower);
}

function buildRosName(node) {
    const ns = node.namespace;
    const nn = node.node_name;
    if (!ns && !nn) return '';
    if (!ns) return '/' + (nn || '');
    if (ns === '/') return '/' + (nn || '');
    return ns + '/' + (nn || '');
}

export function NodeList() {
    const [sortBy, setSortBy] = useState('name');
    const [filterTerm, setFilterTerm] = useState('');

    const allNodes = nodeList.value;

    // Group nodes: containers with their composable children, regular nodes standalone
    const grouped = useMemo(() => {
        // Build container member-name -> composable nodes map
        const containerChildren = new Map();
        const containers = [];
        const regularNodes = [];

        for (const node of allNodes) {
            if (node.node_type === 'container') {
                containers.push(node);
                if (!containerChildren.has(node.name)) {
                    containerChildren.set(node.name, []);
                }
            } else if (node.node_type === 'composable_node' && node.container_name) {
                if (!containerChildren.has(node.container_name)) {
                    containerChildren.set(node.container_name, []);
                }
                containerChildren.get(node.container_name).push(node);
            } else {
                regularNodes.push(node);
            }
        }

        const sorter = makeSorter(sortBy);
        regularNodes.sort(sorter);
        containers.sort(sorter);
        containerChildren.forEach(children => children.sort(sorter));

        // Build flat output: regular nodes, then container groups
        const result = [];
        for (const node of regularNodes) {
            if (matchesFilter(node, filterTerm)) {
                result.push({ node, isChild: false });
            }
        }
        for (const container of containers) {
            const children = containerChildren.get(container.name) || [];
            const containerMatches = matchesFilter(container, filterTerm);
            const matchingChildren = children.filter(c => matchesFilter(c, filterTerm));

            // Show container if it matches or any child matches
            if (containerMatches || matchingChildren.length > 0) {
                result.push({ node: container, isChild: false });
                // Show all children if container matches, otherwise only matching children
                const childrenToShow = containerMatches ? children : matchingChildren;
                for (const child of childrenToShow) {
                    result.push({ node: child, isChild: true });
                }
            }
        }

        return result;
    }, [allNodes, sortBy, filterTerm]);

    const onFilterNamespace = useCallback((ns) => {
        setFilterTerm(ns);
    }, []);

    const onViewNode = useCallback((name) => {
        selectedNode.value = name;
        panelOpen.value = true;
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
                    </select>
                </div>
                <input type="text" class="search-box" id="search" placeholder="Filter nodes..."
                    value=${filterTerm} onInput=${(e) => setFilterTerm(e.target.value)} />
                <${BulkOperations} />
            </div>
            <div class="node-list">
                ${grouped.length === 0 && html`<div class="no-nodes">No nodes found</div>`}
                ${grouped.map(({ node, isChild }) => html`
                    <${NodeCard} key=${node.name} node=${node} isChild=${isChild}
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
