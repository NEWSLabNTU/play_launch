// GraphView component — namespace-grouped interactive graph visualization.
//
// Uses Cytoscape.js with ELK layered layout (Phase 25.9) and expand-collapse
// extension to render the ROS communication graph. Nodes are grouped by
// namespace. Edges are direct aggregated connections between visible endpoints.
// Supports drill-down, focus mode, edge visibility toggle, semantic zoom,
// jump-to-node, tooltips.

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useRef, useCallback, useMemo } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import cytoscape from '../vendor/cytoscape.esm.min.js';
import ELK from '../vendor/elk.bundled.esm.js';
import expandCollapse from '../vendor/cytoscape-expand-collapse.js';
import {
    graphSnapshot, nodes, selectedNode, panelOpen, activeTab,
    getStatusString, currentView,
} from '../store.js';

const html = htm.bind(h);
const elk = new ELK();

// Register extensions once
let _registered = false;
function ensureRegistered() {
    if (_registered) return;
    _registered = true;
    cytoscape.use(expandCollapse);
}

// Infrastructure prefixes to filter
const INFRA_PREFIXES = [
    '/parameter_events', '/rosout', '/tf', '/tf_static',
    '/clock', '/diagnostics', '/_action',
];

function isInfra(name) {
    return INFRA_PREFIXES.some(p => name === p || name.startsWith(p + '/'));
}

// ─── Helpers ──────────────────────────────────────────────────────────────

/** Extract namespace from FQN: "/a/b/node" → "/a/b" */
function getNamespace(fqn) {
    const idx = fqn.lastIndexOf('/');
    if (idx <= 0) return '/';
    return fqn.substring(0, idx);
}

/** Get short name from FQN: "/a/b/node" → "node" */
function getShortName(fqn) {
    const idx = fqn.lastIndexOf('/');
    return fqn.substring(idx + 1);
}

/** Build namespace tree levels: "/a/b/c" → ["/", "/a", "/a/b", "/a/b/c"] */
function namespaceLevels(ns) {
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
function resolveVisible(fqn, collapsedSet) {
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
function getCollapsedSet(cy) {
    const set = new Set();
    cy.nodes('.namespace.cy-expand-collapse-collapsed-node').forEach(n => {
        set.add(n.data('fullNs'));
    });
    return set;
}

/** Get node status color for Cytoscape styling */
function statusToColor(statusStr, isDark) {
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

// ─── Graph data builders ─────────────────────────────────────────────────

/**
 * Build Cytoscape node elements from a GraphSnapshot (no edges).
 * Returns { nodeElements, nodeIndex, allFqns }.
 */
function buildCyNodes(snapshot, nodesMap, showInfra) {
    if (!snapshot || !snapshot.topics) {
        return { nodeElements: [], nodeIndex: new Map(), allFqns: new Set() };
    }

    // Collect all unique node FQNs from the graph
    const allFqns = new Set();
    for (const topic of snapshot.topics) {
        if (!showInfra && isInfra(topic.name)) continue;
        for (const ep of topic.publishers) allFqns.add(ep.fqn);
        for (const ep of topic.subscribers) allFqns.add(ep.fqn);
    }

    // Build namespace set
    const namespaces = new Set();
    const nodeIndex = new Map(); // FQN → { member_name, status, namespace, ... }
    for (const fqn of allFqns) {
        const ns = getNamespace(fqn);
        // Add all ancestor namespaces
        for (const level of namespaceLevels(ns)) {
            namespaces.add(level);
        }

        // Scan all endpoints to find member_name and count pub/sub
        let memberName = null;
        let statusStr = 'unknown';
        let pubCount = 0, subCount = 0, srvCount = 0;

        for (const topic of snapshot.topics) {
            if (!showInfra && isInfra(topic.name)) continue;
            for (const ep of topic.publishers) {
                if (ep.fqn === fqn) {
                    pubCount++;
                    if (ep.member_name && !memberName) memberName = ep.member_name;
                }
            }
            for (const ep of topic.subscribers) {
                if (ep.fqn === fqn) {
                    subCount++;
                    if (ep.member_name && !memberName) memberName = ep.member_name;
                }
            }
        }

        if (snapshot.services) {
            for (const svc of snapshot.services) {
                if (svc.servers.includes(fqn)) srvCount++;
            }
        }

        let pid = null;
        if (memberName) {
            const nodeData = nodesMap.get(memberName);
            if (nodeData) {
                statusStr = getStatusString(nodeData.status);
                pid = nodeData.pid || null;
            }
        }

        nodeIndex.set(fqn, {
            member_name: memberName, status: statusStr, namespace: ns,
            pub: pubCount, sub: subCount, srv: srvCount, pid,
        });
    }

    const nodeElements = [];

    // Create namespace compound nodes
    const sortedNs = Array.from(namespaces).sort();
    for (const ns of sortedNs) {
        const parentNs = ns === '/' ? null : getNamespace(ns) || '/';
        const label = ns === '/' ? '/' : ns.split('/').filter(Boolean).pop();
        // Count direct children (leaf nodes in this namespace)
        let childCount = 0;
        for (const [, info] of nodeIndex) {
            if (info.namespace === ns) childCount++;
        }

        nodeElements.push({
            group: 'nodes',
            data: {
                id: 'ns:' + ns,
                label: label,
                fullNs: ns,
                childCount,
                isNamespace: true,
                ...(parentNs != null && ns !== '/' ? { parent: 'ns:' + parentNs } : {}),
            },
            classes: 'namespace',
        });
    }

    // Create leaf nodes (individual ROS nodes)
    for (const [fqn, info] of nodeIndex) {
        nodeElements.push({
            group: 'nodes',
            data: {
                id: 'node:' + fqn,
                label: getShortName(fqn),
                fqn,
                memberName: info.member_name,
                statusStr: info.status,
                isNamespace: false,
                parent: 'ns:' + info.namespace,
            },
            classes: 'leaf-node status-' + info.status,
        });
    }

    return { nodeElements, nodeIndex, allFqns };
}

/**
 * Build raw edge map from a GraphSnapshot (pub→sub pairs with topic info).
 * Returns Map<"src\0tgt", { topics: Set<string>, dangling: bool }>.
 */
function buildRawEdges(snapshot, allFqns, showInfra) {
    const edgeMap = new Map();
    if (!snapshot || !snapshot.topics) return edgeMap;

    for (const topic of snapshot.topics) {
        if (!showInfra && isInfra(topic.name)) continue;
        for (const pub of topic.publishers) {
            if (!allFqns.has(pub.fqn)) continue;
            for (const sub of topic.subscribers) {
                if (!allFqns.has(sub.fqn)) continue;
                if (pub.fqn === sub.fqn) continue; // skip self-loops
                const key = pub.fqn + '\0' + sub.fqn;
                if (!edgeMap.has(key)) {
                    edgeMap.set(key, { topics: new Set(), dangling: false });
                }
                const e = edgeMap.get(key);
                e.topics.add(topic.name + ' (' + topic.msg_type + ')');
                if (topic.dangling) e.dangling = true;
            }
        }
    }
    return edgeMap;
}

// ─── Edge aggregation (direct, no hubs) ──────────────────────────────────

/**
 * Recompute edges as direct aggregated connections between visible endpoints.
 * No hubs, no trunk/branch distinction — just one edge per visible pair.
 */
function recomputeEdges(cy, rawEdges) {
    const collapsedSet = getCollapsedSet(cy);
    const aggEdges = new Map(); // "srcVis\0tgtVis" → { topics, dangling }

    for (const [key, data] of rawEdges) {
        const [srcFqn, tgtFqn] = key.split('\0');
        const srcVisId = resolveVisible(srcFqn, collapsedSet);
        const tgtVisId = resolveVisible(tgtFqn, collapsedSet);
        if (srcVisId === tgtVisId) continue;

        const eKey = srcVisId + '\0' + tgtVisId;
        if (!aggEdges.has(eKey))
            aggEdges.set(eKey, { topics: new Set(), dangling: false });
        const agg = aggEdges.get(eKey);
        for (const t of data.topics) agg.topics.add(t);
        if (data.dangling) agg.dangling = true;
    }

    cy.remove(cy.edges());
    const els = [];
    for (const [key, data] of aggEdges) {
        const [src, tgt] = key.split('\0');
        const count = data.topics.size;
        els.push({
            group: 'edges',
            data: {
                id: 'edge:' + src + '->' + tgt,
                source: src, target: tgt,
                topicCount: count,
                label: count === 1
                    ? Array.from(data.topics)[0].split(' (')[0]
                    : count + ' topics',
                topicList: Array.from(data.topics).join('\n'),
                dangling: data.dangling,
            },
            classes: data.dangling ? 'dangling-edge' : '',
        });
    }
    cy.add(els);
}

// ─── ELK layout engine ───────────────────────────────────────────────────

/**
 * Convert visible Cytoscape nodes/edges to ELK hierarchical format.
 */
function buildElkGraph(cy) {
    const visibleNodes = cy.nodes(':visible');
    const visibleEdges = cy.edges(':visible');

    // Build a map of ELK nodes by id, with children arrays for compounds
    const elkNodesById = new Map();

    // Root node
    const root = {
        id: 'root',
        layoutOptions: {
            'algorithm': 'layered',
            'elk.direction': 'RIGHT',
            'hierarchyHandling': 'INCLUDE_CHILDREN',
            'crossingMinimization.strategy': 'LAYER_SWEEP',
            'nodePlacement.strategy': 'NETWORK_SIMPLEX',
            'spacing.nodeNode': '30',
            'spacing.nodeNodeBetweenLayers': '50',
            'spacing.edgeNode': '20',
            'spacing.edgeEdge': '15',
            'spacing.componentComponent': '40',
            'padding': '[top=30,left=20,bottom=20,right=20]',
        },
        children: [],
        edges: [],
    };
    elkNodesById.set('root', root);

    // Create ELK nodes for each visible Cytoscape node
    visibleNodes.forEach(cyNode => {
        const id = cyNode.id();
        const isNs = cyNode.hasClass('namespace');
        const isCollapsed = cyNode.hasClass('cy-expand-collapse-collapsed-node');
        const dims = cyNode.layoutDimensions({ nodeDimensionsIncludeLabels: true });

        const elkNode = {
            id: id,
            width: Math.max(dims.w || 80, 40),
            height: Math.max(dims.h || 30, 20),
        };

        // Compound (expanded) namespace nodes have children
        if (isNs && !isCollapsed) {
            elkNode.children = [];
            elkNode.layoutOptions = {
                'padding': '[top=30,left=15,bottom=15,right=15]',
            };
        }

        elkNodesById.set(id, elkNode);
    });

    // Place each node under its parent (or root)
    visibleNodes.forEach(cyNode => {
        const id = cyNode.id();
        const elkNode = elkNodesById.get(id);
        const parent = cyNode.parent();
        let parentId = 'root';
        if (parent && parent.length > 0) {
            const pid = parent.id();
            if (elkNodesById.has(pid) && elkNodesById.get(pid).children) {
                parentId = pid;
            }
        }
        const parentElk = elkNodesById.get(parentId);
        if (parentElk && parentElk.children) {
            parentElk.children.push(elkNode);
        }
    });

    // Add all visible edges at root level (ELK handles cross-hierarchy routing)
    visibleEdges.forEach(cyEdge => {
        root.edges.push({
            id: cyEdge.id(),
            sources: [cyEdge.source().id()],
            targets: [cyEdge.target().id()],
        });
    });

    return root;
}

/**
 * Apply ELK layout result positions to Cytoscape.
 * ELK uses top-left coordinates with cumulative offsets for nested children.
 */
function applyElkPositions(cy, elkResult) {
    const posMap = new Map();

    function walk(node, offX, offY) {
        const x = (node.x || 0) + offX;
        const y = (node.y || 0) + offY;
        posMap.set(node.id, {
            x: x + (node.width || 0) / 2,
            y: y + (node.height || 0) / 2,
        });
        if (node.children) node.children.forEach(c => walk(c, x, y));
    }

    if (elkResult.children) {
        elkResult.children.forEach(c => walk(c, 0, 0));
    }

    cy.startBatch();
    posMap.forEach((pos, id) => {
        const ele = cy.getElementById(id);
        if (ele && ele.length > 0) {
            ele.position(pos);
        }
    });
    cy.endBatch();
}

/**
 * Run ELK layout with concurrency guard (prevents overlapping layout calls).
 * @param {boolean} fit — if true, fit viewport to content after layout (initial load only)
 */
const layoutRunning = { current: false, pending: false, pendingFit: false };

async function runElkLayout(cy, fit) {
    if (layoutRunning.current) {
        layoutRunning.pending = true;
        if (fit) layoutRunning.pendingFit = true;
        return;
    }
    layoutRunning.current = true;
    try {
        const elkGraph = buildElkGraph(cy);
        // Only run layout if there are nodes to lay out
        if (elkGraph.children.length === 0) return;
        const result = await elk.layout(elkGraph);
        applyElkPositions(cy, result);
        if (fit) cy.fit(undefined, 40);
    } catch (e) {
        // ELK layout can fail on degenerate graphs — fall back to preset
        console.warn('ELK layout failed, using current positions:', e.message);
    } finally {
        layoutRunning.current = false;
        if (layoutRunning.pending) {
            layoutRunning.pending = false;
            const pendingFit = layoutRunning.pendingFit;
            layoutRunning.pendingFit = false;
            await runElkLayout(cy, pendingFit);
        }
    }
}

// ─── Focus mode ──────────────────────────────────────────────────────────

function enterFocusMode(cy, node) {
    const hood = node.neighborhood().add(node).union(node.neighborhood().parents());
    cy.elements().addClass('faded');
    hood.removeClass('faded');
    node.connectedEdges().removeClass('faded');
}

function exitFocusMode(cy) {
    cy.elements().removeClass('faded');
}

// ─── Edge visibility ─────────────────────────────────────────────────────

function applyEdgeVisibility(cy, mode) {
    switch (mode) {
        case 'all':
            cy.edges().style('display', 'element');
            break;
        case 'none':
            cy.edges().style('display', 'none');
            break;
        case 'selected':
            cy.edges().style('display', 'none');
            cy.nodes(':selected').connectedEdges().style('display', 'element');
            break;
    }
}

// ─── Tooltip component ────────────────────────────────────────────────────

function Tooltip({ text, x, y }) {
    if (!text) return null;
    return html`
        <div class="graph-tooltip" style=${{
            left: x + 'px',
            top: y + 'px',
        }}>
            ${text.split('\n').map((line, i) => html`<div key=${i}>${line}</div>`)}
        </div>
    `;
}

// ─── Breadcrumb ───────────────────────────────────────────────────────────

function Breadcrumb({ path, onNavigate }) {
    if (!path || path.length === 0) return null;
    return html`
        <div class="graph-breadcrumb">
            ${path.map((seg, i) => html`
                <span key=${i}>
                    ${i > 0 && html`<span class="breadcrumb-sep">/</span>`}
                    <button class="breadcrumb-btn ${i === path.length - 1 ? 'current' : ''}"
                        onClick=${() => onNavigate(seg.ns)}>
                        ${seg.label}
                    </button>
                </span>
            `)}
        </div>
    `;
}

// ─── Main GraphView component ─────────────────────────────────────────────

export function GraphView() {
    const containerRef = useRef(null);
    const cyRef = useRef(null);
    const ecApiRef = useRef(null);
    const nodeIndexRef = useRef(new Map());
    const rawEdgesRef = useRef(new Map());
    const focusedNodeRef = useRef(null);
    const edgeModeRef = useRef('all');
    const [showInfra, setShowInfra] = useState(false);
    const [edgeMode, setEdgeMode] = useState('all');
    const [tooltip, setTooltip] = useState(null);
    const [breadcrumb, setBreadcrumb] = useState([{ label: '/', ns: '/' }]);
    const [statsText, setStatsText] = useState('');
    const snap = graphSnapshot.value;
    const nodesMap = nodes.value;
    const view = currentView.value;

    // Keep edgeModeRef in sync for event handler closures
    edgeModeRef.current = edgeMode;

    // Memoize node elements and raw edges separately
    const { nodeElements, nodeIndex, rawEdges } = useMemo(() => {
        const { nodeElements, nodeIndex, allFqns } = buildCyNodes(snap, nodesMap, showInfra);
        const rawEdges = buildRawEdges(snap, allFqns, showInfra);
        return { nodeElements, nodeIndex, rawEdges };
    }, [snap, nodesMap, showInfra]);

    // Keep refs in sync for event handler closures
    nodeIndexRef.current = nodeIndex;
    rawEdgesRef.current = rawEdges;

    // Compute stats
    useEffect(() => {
        if (!snap) { setStatsText(''); return; }
        const nodeCount = nodeIndex.size;
        const topicCount = snap.topics ? snap.topics.length : 0;
        const danglingCount = snap.topics ? snap.topics.filter(t => t.dangling).length : 0;
        setStatsText(`${nodeCount} nodes, ${topicCount} topics, ${danglingCount} dangling`);
    }, [snap, nodeIndex]);

    // Initialize / update Cytoscape
    useEffect(() => {
        if (view !== 'graph' || !containerRef.current) return;
        if (nodeElements.length === 0) {
            // Destroy existing instance
            if (cyRef.current) {
                cyRef.current.destroy();
                cyRef.current = null;
                ecApiRef.current = null;
            }
            return;
        }

        ensureRegistered();

        const isDark = document.documentElement.getAttribute('data-theme') === 'dark';

        // Create or re-use instance
        if (!cyRef.current) {
            const cy = cytoscape({
                container: containerRef.current,
                elements: [],
                style: getCyStyle(isDark),
                layout: { name: 'preset' },
                wheelSensitivity: 0.3,
                minZoom: 0.1,
                maxZoom: 4,
            });

            // Register expand-collapse (no edges loaded, so extension has
            // nothing to reroute — we handle edges ourselves)
            const ecApi = cy.expandCollapse({
                layoutBy: null,        // we handle layout ourselves
                fisheye: false,
                animate: false,
                undoable: false,
                cueEnabled: false,
            });

            // Interactions — click any node to highlight it + connected edges
            cy.on('tap', 'node', (evt) => {
                const node = evt.target;
                cy.elements().removeClass('highlighted');
                node.addClass('highlighted');
                node.connectedEdges().addClass('highlighted');
                node.connectedEdges().connectedNodes().addClass('highlighted');
            });

            // Click leaf node — also open right panel
            cy.on('tap', 'node.leaf-node', (evt) => {
                const data = evt.target.data();
                if (data.memberName) {
                    selectedNode.value = data.memberName;
                    panelOpen.value = true;
                    activeTab.value = 'topics';
                }
            });

            // Namespace double-click — toggle expand/collapse
            // (single-click kept free for drag; avoids accidental collapse)
            cy.on('dbltap', 'node.namespace', async (evt) => {
                const node = evt.target;
                const data = node.data();
                const ns = data.fullNs;

                // Exit focus mode on expand/collapse
                if (focusedNodeRef.current) {
                    exitFocusMode(cy);
                    focusedNodeRef.current = null;
                }

                // Toggle expand/collapse (root namespace always stays expanded)
                if (ecApiRef.current && ns !== '/') {
                    try {
                        if (node.hasClass('cy-expand-collapse-collapsed-node')) {
                            ecApiRef.current.expand(node);
                        } else {
                            ecApiRef.current.collapse(node);
                        }
                    } catch (_e) { /* ignore */ }
                }

                // Recompute edges after expand/collapse state change
                recomputeEdges(cy, rawEdgesRef.current);
                applyEdgeVisibility(cy, edgeModeRef.current);
                await runElkLayout(cy);

                // Update breadcrumb
                const levels = namespaceLevels(ns);
                const crumbs = levels.map(l => ({
                    label: l === '/' ? '/' : l.split('/').filter(Boolean).pop(),
                    ns: l,
                }));
                setBreadcrumb(crumbs);
            });

            // Focus mode — double-click leaf node
            cy.on('dbltap', 'node.leaf-node', (evt) => {
                const nodeId = evt.target.id();
                if (focusedNodeRef.current === nodeId) {
                    exitFocusMode(cy);
                    focusedNodeRef.current = null;
                } else {
                    enterFocusMode(cy, evt.target);
                    focusedNodeRef.current = nodeId;
                }
            });

            // Click canvas background — clear highlight and exit focus
            cy.on('tap', (evt) => {
                if (evt.target === cy) {
                    cy.elements().removeClass('highlighted');
                    if (focusedNodeRef.current) {
                        exitFocusMode(cy);
                        focusedNodeRef.current = null;
                    }
                }
            });

            // Edge visibility update on node selection change
            cy.on('select unselect', 'node', () => {
                if (edgeModeRef.current === 'selected') {
                    applyEdgeVisibility(cy, 'selected');
                }
            });

            // Semantic zoom
            let lastZoomBand = 'full';
            cy.on('zoom', () => {
                const z = cy.zoom();
                const band = z < 0.4 ? 'minimal' : z < 0.8 ? 'reduced' : 'full';
                if (band !== lastZoomBand) {
                    lastZoomBand = band;
                    cy.startBatch();
                    cy.elements().removeClass('zoom-minimal zoom-reduced');
                    if (band !== 'full') cy.elements().addClass('zoom-' + band);
                    cy.endBatch();
                }
            });

            // Hover tooltips — leaf nodes
            cy.on('mouseover', 'node.leaf-node', (evt) => {
                const data = evt.target.data();
                const pos = evt.renderedPosition;
                const info = nodeIndexRef.current.get(data.fqn);
                const lines = [data.fqn];
                if (data.memberName) lines.push('Member: ' + data.memberName);
                lines.push('Status: ' + data.statusStr);
                if (info) {
                    lines.push(info.pub + ' pub, ' + info.sub + ' sub, ' + info.srv + ' srv');
                    if (info.pid) lines.push('PID: ' + info.pid);
                }
                setTooltip({ text: lines.join('\n'), x: pos.x + 15, y: pos.y + 15 });
            });

            // Hover tooltips — edges
            cy.on('mouseover', 'edge', (evt) => {
                const data = evt.target.data();
                const mp = evt.target.renderedMidpoint();
                const pos = mp || evt.renderedPosition || { x: 0, y: 0 };
                const lines = data.topicList ? data.topicList.split('\n').slice(0, 10) : [];
                if (data.topicCount > 10) lines.push('... and ' + (data.topicCount - 10) + ' more');
                setTooltip({ text: lines.join('\n'), x: pos.x + 15, y: pos.y + 15 });
            });

            cy.on('mouseout', 'node, edge', () => {
                setTooltip(null);
            });

            // Pan/zoom clears tooltip
            cy.on('viewport', () => setTooltip(null));

            cyRef.current = cy;
            ecApiRef.current = ecApi;
        }

        const cy = cyRef.current;

        // Update theme
        cy.style(getCyStyle(isDark));

        // Update node elements only (no edges — recomputeEdges handles those)
        cy.json({ elements: nodeElements });
        cy.elements().forEach(ele => {
            // Update leaf node colors based on current status
            if (ele.hasClass('leaf-node')) {
                const info = nodeIndex.get(ele.data('fqn'));
                if (info) {
                    ele.data('statusStr', info.status);
                    ele.style('background-color', statusToColor(info.status, isDark));
                }
            }
        });

        // Collapse top-level namespaces by default (root always expanded)
        setTimeout(async () => {
            if (ecApiRef.current) {
                try {
                    const rootChildren = cy.nodes('.namespace').filter(n => {
                        return n.data('parent') === 'ns:/';
                    });
                    if (rootChildren.length > 3) {
                        rootChildren.forEach(n => {
                            try { ecApiRef.current.collapse(n); } catch (_e) { /* */ }
                        });
                    }
                } catch (_e) { /* ignore */ }
            }
            recomputeEdges(cy, rawEdgesRef.current);
            applyEdgeVisibility(cy, edgeModeRef.current);
            await runElkLayout(cy, true); // fit on initial load
        }, 100);

    }, [nodeElements, rawEdges, view]);

    // Handle resize
    useEffect(() => {
        if (view !== 'graph' || !cyRef.current) return;
        const onResize = () => {
            if (cyRef.current) cyRef.current.resize();
        };
        window.addEventListener('resize', onResize);
        // Also resize when view becomes active
        const t = setTimeout(onResize, 50);
        return () => {
            window.removeEventListener('resize', onResize);
            clearTimeout(t);
        };
    }, [view]);

    // Cleanup on unmount
    useEffect(() => {
        return () => {
            if (cyRef.current) {
                cyRef.current.destroy();
                cyRef.current = null;
                ecApiRef.current = null;
            }
        };
    }, []);

    // ─── Control handlers ─────────────────────────────────────────────────

    const handleFit = useCallback(() => {
        if (cyRef.current) cyRef.current.fit(undefined, 40);
    }, []);

    const handleExpandAll = useCallback(async () => {
        if (ecApiRef.current) {
            try {
                // Exit focus mode
                if (focusedNodeRef.current && cyRef.current) {
                    exitFocusMode(cyRef.current);
                    focusedNodeRef.current = null;
                }
                ecApiRef.current.expandAll();
                if (cyRef.current) {
                    recomputeEdges(cyRef.current, rawEdgesRef.current);
                    applyEdgeVisibility(cyRef.current, edgeModeRef.current);
                    await runElkLayout(cyRef.current);
                }
            } catch (_e) { /* ignore */ }
        }
    }, []);

    const handleCollapseAll = useCallback(async () => {
        if (ecApiRef.current) {
            try {
                // Exit focus mode
                if (focusedNodeRef.current && cyRef.current) {
                    exitFocusMode(cyRef.current);
                    focusedNodeRef.current = null;
                }
                ecApiRef.current.collapseAll();
                // Re-expand root — root namespace always stays expanded
                const rootNode = cyRef.current && cyRef.current.getElementById('ns:/');
                if (rootNode && rootNode.hasClass('cy-expand-collapse-collapsed-node')) {
                    try { ecApiRef.current.expand(rootNode); } catch (_e) { /* */ }
                }
                if (cyRef.current) {
                    recomputeEdges(cyRef.current, rawEdgesRef.current);
                    applyEdgeVisibility(cyRef.current, edgeModeRef.current);
                    await runElkLayout(cyRef.current);
                }
            } catch (_e) { /* ignore */ }
        }
    }, []);

    const handleToggleInfra = useCallback(() => {
        setShowInfra(v => !v);
    }, []);

    const handleEdgeMode = useCallback((mode) => {
        setEdgeMode(mode);
        if (cyRef.current) {
            applyEdgeVisibility(cyRef.current, mode);
        }
    }, []);

    const handleBreadcrumbNav = useCallback(async (ns) => {
        if (!cyRef.current) return;
        const cy = cyRef.current;
        // Find the namespace node and fit to it
        const nsNode = cy.getElementById('ns:' + ns);
        if (nsNode && nsNode.length > 0) {
            // Expand this namespace if collapsed
            if (ecApiRef.current && nsNode.hasClass('cy-expand-collapse-collapsed-node')) {
                try { ecApiRef.current.expand(nsNode); } catch (_e) { /* */ }
            }
            recomputeEdges(cy, rawEdgesRef.current);
            applyEdgeVisibility(cy, edgeModeRef.current);
            // Fit to the compound node (includes all descendants)
            cy.fit(nsNode, 40);
        } else if (ns === '/') {
            cy.fit(undefined, 40);
        }
        // Update breadcrumb
        const levels = namespaceLevels(ns);
        const crumbs = levels.map(l => ({
            label: l === '/' ? '/' : l.split('/').filter(Boolean).pop(),
            ns: l,
        }));
        setBreadcrumb(crumbs);
    }, []);

    // ─── Render ───────────────────────────────────────────────────────────

    if (!snap) {
        return html`
            <div class="graph-view">
                <div class="graph-message">
                    <div class="graph-message-text">Waiting for graph data...</div>
                    <div class="graph-message-sub">Graph will appear once nodes are running.</div>
                </div>
            </div>
        `;
    }

    return html`
        <div class="graph-view">
            <div class="graph-toolbar">
                <${Breadcrumb} path=${breadcrumb} onNavigate=${handleBreadcrumbNav} />
                <div class="graph-controls">
                    <span class="graph-stats">${statsText}</span>
                    <label class="graph-infra-toggle">
                        <input type="checkbox" checked=${showInfra}
                            onChange=${handleToggleInfra} />
                        <span>Infra</span>
                    </label>
                    <div class="graph-edge-toggle">
                        <button class="graph-btn ${edgeMode === 'all' ? 'active' : ''}"
                            onClick=${() => handleEdgeMode('all')} title="Show all edges">All</button>
                        <button class="graph-btn ${edgeMode === 'selected' ? 'active' : ''}"
                            onClick=${() => handleEdgeMode('selected')} title="Show selected node edges">Sel</button>
                        <button class="graph-btn ${edgeMode === 'none' ? 'active' : ''}"
                            onClick=${() => handleEdgeMode('none')} title="Hide all edges">None</button>
                    </div>
                    <button class="graph-btn" onClick=${handleFit} title="Fit to screen">Fit</button>
                    <button class="graph-btn" onClick=${handleExpandAll} title="Expand all groups">Expand</button>
                    <button class="graph-btn" onClick=${handleCollapseAll} title="Collapse all groups">Collapse</button>
                </div>
            </div>
            <div class="graph-canvas-wrapper">
                <div class="graph-canvas" ref=${containerRef}></div>
                ${tooltip && html`<${Tooltip} ...${tooltip} />`}
            </div>
        </div>
    `;
}

// ─── Cytoscape stylesheet ─────────────────────────────────────────────────

function getCyStyle(isDark) {
    const bg = isDark ? '#1a1d23' : '#ffffff';
    const textMuted = isDark ? '#8a8d91' : '#6c757d';
    const borderColor = isDark ? '#3a3d45' : '#dee2e6';
    const nsBg = isDark ? '#1e2128' : '#f1f3f5';
    const nsBorder = isDark ? '#4a4d55' : '#c5c9d0';
    const edgeColor = isDark ? '#6b7280' : '#9ca3af';
    const danglingEdge = isDark ? '#fbbf24' : '#d97706';
    const defaultNode = isDark ? '#6b7280' : '#9ca3af';

    return [
        // Namespace compound nodes — opaque background masks cross-namespace edges.
        // Depth-based shading: deeper namespaces are slightly different so
        // multi-level nesting is visually distinguishable.
        {
            selector: 'node.namespace',
            style: {
                'shape': 'round-rectangle',
                'background-color': function(ele) {
                    const ns = ele.data('fullNs');
                    const depth = ns === '/' ? 0 : ns.split('/').filter(Boolean).length;
                    if (isDark) {
                        // Dark: base #1e2128, lighter per level
                        const base = 30; // R/G/B base
                        const step = 5;
                        const v = Math.min(base + depth * step, 55);
                        return 'rgb(' + v + ',' + (v + 2) + ',' + (v + 5) + ')';
                    } else {
                        // Light: base #f1f3f5, darker per level
                        const base = 245;
                        const step = 6;
                        const v = Math.max(base - depth * step, 215);
                        return 'rgb(' + v + ',' + (v - 2) + ',' + (v - 4) + ')';
                    }
                },
                'background-opacity': 1,
                'border-color': nsBorder,
                'border-width': 1.5,
                'border-style': 'dashed',
                'label': 'data(label)',
                'text-valign': 'top',
                'text-halign': 'center',
                'text-margin-y': -6,
                'font-size': '12px',
                'font-weight': 'bold',
                'color': textMuted,
                'padding': '16px',
                'min-width': '60px',
                'min-height': '40px',
                'text-max-width': '120px',
                'text-wrap': 'ellipsis',
            },
        },
        // Collapsed namespace nodes
        {
            selector: 'node.namespace.cy-expand-collapse-collapsed-node',
            style: {
                'border-style': 'solid',
                'border-width': 2,
                'label': function(ele) {
                    const cc = ele.data('childCount');
                    const label = ele.data('label');
                    return cc > 0 ? label + ' (' + cc + ')' : label;
                },
            },
        },
        // Leaf nodes (ROS nodes)
        {
            selector: 'node.leaf-node',
            style: {
                'shape': 'round-rectangle',
                'background-color': defaultNode,
                'border-color': borderColor,
                'border-width': 1,
                'label': 'data(label)',
                'text-valign': 'center',
                'text-halign': 'center',
                'font-size': '10px',
                'color': '#fff',
                'width': 'label',
                'height': 28,
                'padding': '6px',
                'text-max-width': '100px',
                'text-wrap': 'ellipsis',
            },
        },
        // Status-colored leaf nodes
        {
            selector: 'node.status-running, node.status-loaded',
            style: { 'background-color': isDark ? '#22c55e' : '#16a34a' },
        },
        {
            selector: 'node.status-stopped, node.status-unloaded',
            style: { 'background-color': isDark ? '#4b5563' : '#6b7280' },
        },
        {
            selector: 'node.status-failed',
            style: { 'background-color': isDark ? '#ef4444' : '#dc2626' },
        },
        {
            selector: 'node.status-loading, node.status-pending',
            style: { 'background-color': isDark ? '#d97706' : '#b45309' },
        },
        {
            selector: 'node.status-blocked',
            style: { 'background-color': isDark ? '#6b7280' : '#94a3b8' },
        },
        // Selected node highlight
        {
            selector: 'node.leaf-node:selected',
            style: {
                'border-color': isDark ? '#3b82f6' : '#0d6efd',
                'border-width': 3,
                'overlay-opacity': 0.1,
            },
        },
        // Unified edge style
        {
            selector: 'edge',
            style: {
                'curve-style': 'bezier',
                'line-color': edgeColor,
                'target-arrow-shape': 'triangle',
                'target-arrow-color': edgeColor,
                'width': function(ele) {
                    const c = ele.data('topicCount') || 1;
                    return Math.min(1.5 + Math.log2(c) * 1.2, 7);
                },
                'opacity': 0.7,
                'arrow-scale': 0.8,
                'font-size': '8px',
                'color': textMuted,
                'text-rotation': 'autorotate',
                'text-background-color': bg,
                'text-background-opacity': 0.85,
                'text-background-padding': '2px',
            },
        },
        // Edge labels on selection
        {
            selector: 'edge:selected',
            style: {
                'label': 'data(label)',
                'line-color': isDark ? '#60a5fa' : '#3b82f6',
                'target-arrow-color': isDark ? '#60a5fa' : '#3b82f6',
                'opacity': 1,
            },
        },
        // Dangling edges (amber)
        {
            selector: 'edge.dangling-edge',
            style: {
                'line-color': danglingEdge,
                'target-arrow-color': danglingEdge,
                'line-style': 'dashed',
            },
        },
        // Click-to-highlight: highlighted edges get accent color
        {
            selector: 'edge.highlighted',
            style: {
                'line-color': isDark ? '#60a5fa' : '#3b82f6',
                'target-arrow-color': isDark ? '#60a5fa' : '#3b82f6',
                'opacity': 1,
            },
        },
        // Focus mode — faded elements
        {
            selector: '.faded',
            style: { 'opacity': 0.08, 'events': 'yes' },
        },
        // Semantic zoom — minimal (very zoomed out)
        {
            selector: 'node.leaf-node.zoom-minimal',
            style: { 'label': '', 'width': 16, 'height': 16, 'shape': 'ellipse' },
        },
        {
            selector: 'node.namespace.zoom-minimal',
            style: { 'font-size': '8px' },
        },
        {
            selector: 'edge.zoom-minimal',
            style: { 'width': 1, 'opacity': 0.3 },
        },
        // Semantic zoom — reduced
        {
            selector: 'node.leaf-node.zoom-reduced',
            style: { 'font-size': '8px' },
        },
    ];
}
