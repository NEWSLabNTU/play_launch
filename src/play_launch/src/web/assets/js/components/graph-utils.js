// graph-utils.js — Pure helper functions for graph visualization.
// No Cytoscape or Preact dependency.

// Infrastructure prefixes to filter
export const INFRA_PREFIXES = [
    '/parameter_events', '/rosout', '/tf', '/tf_static',
    '/clock', '/diagnostics', '/_action',
];

export function isInfra(name) {
    return INFRA_PREFIXES.some(p => name === p || name.startsWith(p + '/'));
}

/** Extract namespace from FQN: "/a/b/node" → "/a/b" */
export function getNamespace(fqn) {
    const idx = fqn.lastIndexOf('/');
    if (idx <= 0) return '/';
    return fqn.substring(0, idx);
}

/** Get short name from FQN: "/a/b/node" → "node" */
export function getShortName(fqn) {
    const idx = fqn.lastIndexOf('/');
    return fqn.substring(idx + 1);
}

/** Build namespace tree levels: "/a/b/c" → ["/", "/a", "/a/b", "/a/b/c"] */
export function namespaceLevels(ns) {
    if (ns === '/') return ['/'];
    const parts = ns.split('/').filter(Boolean);
    const levels = ['/'];
    let acc = '';
    for (const p of parts) {
        acc += '/' + p;
        levels.push(acc);
    }
    return levels;
}

/**
 * Resolve which visible element represents a node FQN given collapsed namespaces.
 * Walks namespace levels root→leaf; first collapsed level captures the node.
 */
export function resolveVisible(fqn, collapsedSet) {
    const ns = getNamespace(fqn);
    const levels = namespaceLevels(ns);
    for (const level of levels) {
        if (collapsedSet.has(level)) {
            return 'ns:' + level;
        }
    }
    return 'node:' + fqn;
}

/** Read collapsed namespace set from Cytoscape state. */
export function getCollapsedSet(cy) {
    const set = new Set();
    cy.nodes('.namespace.cy-expand-collapse-collapsed-node').forEach(n => {
        set.add(n.data('fullNs'));
    });
    return set;
}

/** Get node status color for Cytoscape styling */
export function statusToColor(statusStr, isDark) {
    switch (statusStr) {
        case 'running': case 'loaded':
            return isDark ? '#22c55e' : '#16a34a';
        case 'stopped': case 'unloaded':
            return isDark ? '#9ca3af' : '#6b7280';
        case 'failed':
            return isDark ? '#ef4444' : '#dc2626';
        case 'loading': case 'pending':
            return isDark ? '#fbbf24' : '#d97706';
        case 'blocked':
            return isDark ? '#9ca3af' : '#94a3b8';
        default:
            return isDark ? '#6b7280' : '#9ca3af';
    }
}
