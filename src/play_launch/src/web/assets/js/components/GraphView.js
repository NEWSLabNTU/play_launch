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
    graphSnapshot, nodes, panelOpen,
    getStatusString, currentView, graphSelectedElement,
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

// ─── Edge aggregation with port-based bundling ───────────────────────────

/**
 * Get the immediate Cytoscape parent id of a visible element.
 * Returns null for root-level elements.
 */
function getVisibleParent(cy, eleId) {
    const ele = cy.getElementById(eleId);
    if (!ele || ele.length === 0) return null;
    const parent = ele.parent();
    if (!parent || parent.length === 0) return null;
    return parent.id();
}

/**
 * Recompute edges with port-based bundling for cross-namespace traffic.
 *
 * Internal edges (both endpoints share the same immediate parent NS) are
 * drawn as direct lines. Cross-NS edges are routed through port nodes:
 * one outbound port per expanded NS that has external traffic.
 *
 * Branch edges: internal-node → port (thin, dotted, no arrow)
 * Trunk edges: port → external target/port (thick, arrow, aggregated count)
 */
function recomputeEdges(cy, rawEdges) {
    const collapsedSet = getCollapsedSet(cy);

    // Step 1: Aggregate raw edges to visible endpoints
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

    // Step 2: Classify edges as internal vs cross-NS
    const internalEdges = [];   // direct: same parent NS
    const crossNsEdges = [];    // need port bundling

    for (const [key, data] of aggEdges) {
        const [src, tgt] = key.split('\0');
        const srcParent = getVisibleParent(cy, src);
        const tgtParent = getVisibleParent(cy, tgt);

        if (srcParent && srcParent === tgtParent) {
            internalEdges.push({ src, tgt, data });
        } else {
            crossNsEdges.push({ src, tgt, srcParent, tgtParent, data });
        }
    }

    // Step 3: Build port nodes and trunk/branch edges for cross-NS traffic
    // Group cross-NS edges by (srcParent, tgtParent) to aggregate trunks
    const trunkKey = (sp, tp) => (sp || 'root') + '\0' + (tp || 'root');
    const trunkMap = new Map(); // trunkKey → { topics, dangling, srcs: Set, tgts: Set }
    const portNeeded = new Set(); // parent NS ids that need a port node

    for (const e of crossNsEdges) {
        const tk = trunkKey(e.srcParent, e.tgtParent);
        if (!trunkMap.has(tk)) {
            trunkMap.set(tk, {
                topics: new Set(), dangling: false,
                srcs: new Set(), tgts: new Set(),
                srcParent: e.srcParent, tgtParent: e.tgtParent,
            });
        }
        const t = trunkMap.get(tk);
        for (const topic of e.data.topics) t.topics.add(topic);
        if (e.data.dangling) t.dangling = true;
        t.srcs.add(e.src);
        t.tgts.add(e.tgt);

        // Mark which expanded NS need port nodes (only leaf nodes inside
        // expanded NS get ports — collapsed NS nodes ARE their own port)
        if (e.srcParent && !e.src.startsWith('ns:')) portNeeded.add(e.srcParent);
        if (e.tgtParent && !e.tgt.startsWith('ns:')) portNeeded.add(e.tgtParent);
    }

    // Remove old edges and port nodes
    cy.remove(cy.edges());
    cy.remove(cy.nodes('.port-node'));

    const els = [];

    // Create port nodes for expanded NS with cross-NS traffic
    const portIds = new Map(); // parent NS id → port node id
    for (const parentId of portNeeded) {
        const portId = 'port:' + parentId;
        portIds.set(parentId, portId);
        const parentNode = cy.getElementById(parentId);
        const parentNs = parentNode.length ? parentNode.data('fullNs') : parentId;

        // Collect all topics going through this port for tooltip
        const portTopics = new Set();
        for (const [, t] of trunkMap) {
            if (t.srcParent === parentId || t.tgtParent === parentId) {
                for (const topic of t.topics) portTopics.add(topic);
            }
        }

        els.push({
            group: 'nodes',
            data: {
                id: portId,
                label: '',
                parent: parentId,
                parentNs: parentNs,
                isPort: true,
                topicList: Array.from(portTopics).join('\n'),
            },
            classes: 'port-node',
        });
    }

    // Create internal edges (direct, no port)
    for (const e of internalEdges) {
        const count = e.data.topics.size;
        els.push({
            group: 'edges',
            data: {
                id: 'edge:' + e.src + '->' + e.tgt,
                source: e.src, target: e.tgt,
                topicCount: count,
                label: count === 1
                    ? Array.from(e.data.topics)[0].split(' (')[0]
                    : count + ' topics',
                topicList: Array.from(e.data.topics).join('\n'),
                dangling: e.data.dangling,
            },
            classes: e.data.dangling ? 'dangling-edge' : '',
        });
    }

    // Create trunk and branch edges for cross-NS traffic
    for (const [, t] of trunkMap) {
        const srcPort = portIds.get(t.srcParent);
        const tgtPort = portIds.get(t.tgtParent);
        // Trunk source: port node if src side has leaf nodes, else collapsed NS
        const trunkSrc = srcPort || [...t.srcs][0];
        const trunkTgt = tgtPort || [...t.tgts][0];
        const count = t.topics.size;

        // Trunk edge (port-to-port or port-to-collapsed-NS)
        els.push({
            group: 'edges',
            data: {
                id: 'trunk:' + trunkSrc + '->' + trunkTgt,
                source: trunkSrc, target: trunkTgt,
                topicCount: count,
                label: count === 1
                    ? Array.from(t.topics)[0].split(' (')[0]
                    : count + ' topics',
                topicList: Array.from(t.topics).join('\n'),
                dangling: t.dangling,
            },
            classes: 'trunk-edge' + (t.dangling ? ' dangling-edge' : ''),
        });

        // Branch edges: leaf nodes → their port (within the NS)
        if (srcPort) {
            for (const src of t.srcs) {
                if (src === srcPort) continue; // skip self
                const branchId = 'branch:' + src + '->' + srcPort;
                // Deduplicate: only add if not already present
                if (!els.some(e => e.data && e.data.id === branchId)) {
                    els.push({
                        group: 'edges',
                        data: {
                            id: branchId,
                            source: src, target: srcPort,
                        },
                        classes: 'branch-edge',
                    });
                }
            }
        }
        if (tgtPort) {
            for (const tgt of t.tgts) {
                if (tgt === tgtPort) continue;
                const branchId = 'branch:' + tgtPort + '->' + tgt;
                if (!els.some(e => e.data && e.data.id === branchId)) {
                    els.push({
                        group: 'edges',
                        data: {
                            id: branchId,
                            source: tgtPort, target: tgt,
                        },
                        classes: 'branch-edge',
                    });
                }
            }
        }
    }

    cy.add(els);
    console.log('[GraphView] edges: %d internal, %d cross-NS (%d ports, %d trunk, %d branch)',
        internalEdges.length, crossNsEdges.length,
        portNeeded.size, trunkMap.size,
        els.filter(e => e.classes && e.classes.includes('branch-edge')).length);
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
            'spacing.nodeNode': '40',
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

        let w, h;
        if (cyNode.hasClass('leaf-node')) {
            // Match the style function exactly to avoid stale layoutDimensions
            const label = cyNode.data('label') || '';
            w = Math.max(label.length * 7 + 12, 40);
            h = 28;
        } else if (cyNode.hasClass('port-node')) {
            w = 8;
            h = 8;
        } else {
            // Namespace / collapsed — use layoutDimensions
            const dims = cyNode.layoutDimensions({ nodeDimensionsIncludeLabels: true });
            w = Math.max(dims.w || 80, 40);
            h = Math.max(dims.h || 30, 20);
        }

        const elkNode = { id, width: w, height: h };

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
function applyElkPositions(cy, elkResult, animate = true) {
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

    if (!animate) {
        // Instant mode: used for initial load
        cy.startBatch();
        posMap.forEach((pos, id) => {
            const ele = cy.getElementById(id);
            if (ele && ele.length > 0) ele.position(pos);
        });
        cy.endBatch();
        return Promise.resolve();
    }

    // Animated mode: each element slides to new position
    const duration = 300;
    const promises = [];
    posMap.forEach((pos, id) => {
        const ele = cy.getElementById(id);
        if (ele && ele.length > 0) {
            promises.push(ele.animation({
                position: pos,
                duration: duration,
                easing: 'ease-in-out-cubic',
            }).play().promise());
        }
    });
    return Promise.all(promises);
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
        console.log('[GraphView] layout: queued (fit=%s)', fit);
        return;
    }
    layoutRunning.current = true;
    const animate = !fit;
    console.log('[GraphView] layout: starting (fit=%s, animate=%s)', fit, animate);
    try {
        const elkGraph = buildElkGraph(cy);
        // Only run layout if there are nodes to lay out
        if (elkGraph.children.length === 0) return;
        const result = await elk.layout(elkGraph);
        await applyElkPositions(cy, result, animate);
        if (fit) {
            cy.fit(undefined, 40);           // instant fit on initial load
        } else {
            cy.animate({ fit: { eles: cy.elements(), padding: 40 }, duration: 300 });
        }
        console.log('[GraphView] layout: complete (%d nodes positioned)', cy.nodes(':visible').length);
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
    const initializedRef = useRef(false);
    const loadedLeafIdsRef = useRef(new Set()); // tracks loaded leaf IDs (collapse removes nodes from cy)
    const batchRef = useRef(false); // true during collapseAll/expandAll to suppress per-node events
    const [showInfra, setShowInfra] = useState(false);
    const [edgeMode, setEdgeMode] = useState('all');
    const [tooltip, setTooltip] = useState(null);
    const [breadcrumb, setBreadcrumb] = useState([{ label: '/', ns: '/' }]);
    const [statsText, setStatsText] = useState('');
    const [initDoneCounter, setInitDoneCounter] = useState(0);
    const snap = graphSnapshot.value;
    const nodesMap = nodes.value;
    const view = currentView.value;

    // Keep edgeModeRef in sync for event handler closures
    edgeModeRef.current = edgeMode;

    // Memoize structural graph data — depends only on snapshot, NOT nodesMap.
    // Status colors are handled by a separate lightweight effect below.
    // This prevents every SSE state event from triggering a structural rebuild.
    const { nodeElements, nodeIndex, rawEdges } = useMemo(() => {
        const { nodeElements, nodeIndex, allFqns } = buildCyNodes(snap, new Map(), showInfra);
        const rawEdges = buildRawEdges(snap, allFqns, showInfra);
        return { nodeElements, nodeIndex, rawEdges };
    }, [snap, showInfra]);

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
                initializedRef.current = false;
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
                minZoom: 0.1,
                maxZoom: 4,
            });

            // Register expand-collapse with cue buttons for [+]/[−] toggle
            const ecApi = cy.expandCollapse({
                layoutBy: null,        // we handle layout ourselves
                fisheye: false,
                animate: false,
                undoable: false,
                cueEnabled: true,
                expandCollapseCuePosition: 'top-left',
                expandCollapseCueSize: 14,
                expandCollapseCueSensitivity: 1,
            });

            // Interactions — click any node or edge to highlight and select
            // it in the graph panel. Single click drives the panel.
            cy.on('tap', 'node, edge', (evt) => {
                cy.elements().removeClass('highlighted');
                const target = evt.target;
                if (target.isNode()) {
                    target.addClass('highlighted');
                    target.connectedEdges().addClass('highlighted');
                    target.connectedEdges().connectedNodes().addClass('highlighted');

                    if (target.hasClass('leaf-node')) {
                        graphSelectedElement.value = {
                            type: 'node', id: target.id(), data: target.data(),
                        };
                    } else if (target.hasClass('port-node')) {
                        // Port nodes → treat as parent namespace
                        const parentNs = target.data('parentNs') || '/';
                        const parentEle = cy.getElementById('ns:' + parentNs);
                        const pData = parentEle.length ? parentEle.data() : { fullNs: parentNs, label: parentNs };
                        graphSelectedElement.value = {
                            type: 'namespace', id: 'ns:' + parentNs, data: pData,
                        };
                    } else if (target.hasClass('namespace')) {
                        graphSelectedElement.value = {
                            type: 'namespace', id: target.id(), data: target.data(),
                        };
                    }
                    panelOpen.value = true;
                } else {
                    // Edge: highlight edge + its two endpoints
                    target.addClass('highlighted');
                    target.connectedNodes().addClass('highlighted');
                    graphSelectedElement.value = {
                        type: 'edge', id: target.id(),
                        data: {
                            ...target.data(),
                            source: target.source().id(),
                            target: target.target().id(),
                        },
                    };
                    panelOpen.value = true;
                }
            });

            // Post-expand/collapse via cue button — recompute edges + layout.
            // Skipped during batch operations (collapseAll/expandAll) which
            // handle edges and layout themselves after the batch completes.
            cy.on('expandcollapse.afterExpand expandcollapse.afterCollapse', async (evt) => {
                if (batchRef.current) return;
                const evtType = evt.type.includes('Expand') ? 'expand' : 'collapse';
                const ns = evt.target.data('fullNs') || evt.target.id();
                console.log('[GraphView] %s: %s', evtType, ns);
                // Protect root namespace from being collapsed
                const rootNode = cy.getElementById('ns:/');
                if (rootNode.hasClass('cy-expand-collapse-collapsed-node') && ecApiRef.current) {
                    try { ecApiRef.current.expand(rootNode); } catch (_) { /* */ }
                }
                recomputeEdges(cy, rawEdgesRef.current);
                applyEdgeVisibility(cy, edgeModeRef.current);
                await runElkLayout(cy);
                // Update breadcrumb to the expanded/collapsed node
                if (ns) {
                    const levels = namespaceLevels(ns);
                    setBreadcrumb(levels.map(l => ({
                        label: l === '/' ? '/' : l.split('/').filter(Boolean).pop(),
                        ns: l,
                    })));
                }
            });

            // Click canvas background — clear highlight, exit focus, close panel
            cy.on('tap', (evt) => {
                if (evt.target === cy) {
                    cy.elements().removeClass('highlighted');
                    graphSelectedElement.value = null;
                    panelOpen.value = false;
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
                }
                if (data.memberName) {
                    const storeNode = nodes.value.get(data.memberName);
                    if (storeNode?.pid) lines.push('PID: ' + storeNode.pid);
                }
                setTooltip({ text: lines.join('\n'), x: pos.x + 15, y: pos.y + 15 });
            });

            // Hover tooltips — port nodes (aggregated topic list)
            cy.on('mouseover', 'node.port-node', (evt) => {
                const data = evt.target.data();
                const pos = evt.renderedPosition;
                const lines = ['Port: ' + (data.parentNs || '')];
                if (data.topicList) {
                    const topics = data.topicList.split('\n').slice(0, 10);
                    lines.push(...topics);
                    const total = data.topicList.split('\n').length;
                    if (total > 10) lines.push('... and ' + (total - 10) + ' more');
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

        // State machine: false → 'pending' → true
        //   false:   first run — load elements, schedule deferred collapse
        //   pending: collapse/layout in progress — skip SSE-triggered re-runs
        //   true:    ready for incremental updates
        if (initializedRef.current === 'pending') {
            // Setup in progress — ignore SSE updates until collapse/layout completes
            console.log('[GraphView] skipping SSE update (init pending)');
            return;
        }

        if (!initializedRef.current) {
            // ── First run: full element load → collapse → edges → layout ──
            // Mark 'pending' immediately so SSE-triggered re-runs are skipped
            // while the deferred collapse/layout is in progress.
            initializedRef.current = 'pending';
            cy.json({ elements: nodeElements });
            loadedLeafIdsRef.current = new Set(
                nodeElements.filter(e => e.classes && e.classes.includes('leaf-node')).map(e => e.data.id)
            );
            const leafCount = loadedLeafIdsRef.current.size;
            const nsCount = cy.nodes('.namespace').length;
            console.log('[GraphView] init: loaded %d leaf nodes, %d namespaces', leafCount, nsCount);
            cy.nodes('.leaf-node').forEach(ele => {
                const info = nodeIndex.get(ele.data('fqn'));
                if (info) {
                    ele.data('statusStr', info.status);
                    ele.style('background-color', statusToColor(info.status, isDark));
                }
            });

            // Collapse top-level namespaces by default (root always expanded).
            // Deferred so the expand-collapse extension can process the elements.
            setTimeout(async () => {
                let collapsed = 0;
                if (ecApiRef.current) {
                    batchRef.current = true;
                    try {
                        const rootChildren = cy.nodes('.namespace').filter(n => {
                            return n.data('parent') === 'ns:/';
                        });
                        if (rootChildren.length > 3) {
                            rootChildren.forEach(n => {
                                try { ecApiRef.current.collapse(n); collapsed++; } catch (_e) { /* */ }
                            });
                        }
                    } catch (_e) { /* ignore */ }
                    batchRef.current = false;
                }
                console.log('[GraphView] init: collapsed %d top-level namespaces', collapsed);
                recomputeEdges(cy, rawEdgesRef.current);
                applyEdgeVisibility(cy, edgeModeRef.current);
                await runElkLayout(cy, true); // fit on initial load
                initializedRef.current = true;
                console.log('[GraphView] init: complete');
                // Force re-render so any updates that arrived during 'pending'
                // (and were skipped by the early-return) get processed.
                setInitDoneCounter(c => c + 1);
            }, 100);
        } else {
            // ── Subsequent SSE updates: in-place update or rebuild ──
            // Compare against loadedLeafIdsRef (not cy.nodes) because the
            // expand-collapse extension removes child elements from cy on
            // collapse, making cy.nodes('.leaf-node') return fewer than loaded.
            const prevIds = loadedLeafIdsRef.current;
            const newIds = new Set(
                nodeElements
                    .filter(e => e.classes && e.classes.includes('leaf-node'))
                    .map(e => e.data.id)
            );
            const changed = prevIds.size !== newIds.size
                || [...prevIds].some(id => !newIds.has(id));

            if (changed) {
                console.log('[GraphView] update: node set changed (%d → %d leaves), rebuilding',
                    prevIds.size, newIds.size);
                // Node set changed — save EXPANDED set, rebuild, collapse rest.
                // New namespaces (not previously known) default to collapsed.
                const expandedSet = new Set();
                cy.nodes('.namespace').forEach(n => {
                    if (!n.hasClass('cy-expand-collapse-collapsed-node')) {
                        expandedSet.add(n.data('fullNs'));
                    }
                });
                expandedSet.add('/'); // root always expanded
                console.log('[GraphView] rebuild: saving expanded set:', [...expandedSet]);

                cy.json({ elements: nodeElements });
                loadedLeafIdsRef.current = newIds;

                // Deferred collapse — the expand-collapse extension needs a
                // tick after cy.json() to register new elements, matching the
                // initial-load pattern (setTimeout).
                setTimeout(async () => {
                    batchRef.current = true;
                    // Collapse all namespaces not in expandedSet (deepest first
                    // so children are collapsed before parents)
                    let collapsed = 0;
                    if (ecApiRef.current) {
                        const nsNodes = [];
                        cy.nodes('.namespace').forEach(n => nsNodes.push(n));
                        nsNodes.sort((a, b) => {
                            const aDepth = (a.data('fullNs') || '').split('/').length;
                            const bDepth = (b.data('fullNs') || '').split('/').length;
                            return bDepth - aDepth;
                        });
                        for (const n of nsNodes) {
                            if (!expandedSet.has(n.data('fullNs'))) {
                                try {
                                    ecApiRef.current.collapse(n);
                                    collapsed++;
                                } catch (e) {
                                    console.warn('[GraphView] collapse failed for', n.data('fullNs'), e.message);
                                }
                            }
                        }
                    }
                    batchRef.current = false;
                    console.log('[GraphView] rebuild: collapsed', collapsed, 'namespaces');
                    recomputeEdges(cy, rawEdgesRef.current);
                    applyEdgeVisibility(cy, edgeModeRef.current);
                    await runElkLayout(cy);
                }, 0);
            } else {
                // Same node set — recompute edges in case topic connections changed
                console.log('[GraphView] update: node set unchanged, updating edges only');
                recomputeEdges(cy, rawEdgesRef.current);
                applyEdgeVisibility(cy, edgeModeRef.current);
            }
        }

    }, [nodeElements, rawEdges, view, initDoneCounter]);

    // Status-only updates — lightweight color sync on every nodesMap change.
    // Decoupled from the structural effect so SSE state events don't trigger
    // expensive graph rebuilds. Also fires after structural changes and init.
    useEffect(() => {
        if (view !== 'graph' || !cyRef.current) return;
        const isDark = document.documentElement.getAttribute('data-theme') === 'dark';
        const cy = cyRef.current;
        cy.startBatch();
        cy.nodes('.leaf-node').forEach(ele => {
            const memberName = ele.data('memberName');
            if (memberName) {
                const storeNode = nodesMap.get(memberName);
                const statusStr = storeNode ? getStatusString(storeNode.status) : 'unknown';
                ele.data('statusStr', statusStr);
                ele.style('background-color', statusToColor(statusStr, isDark));
            }
        });
        cy.endBatch();
    }, [nodesMap, nodeElements, view, initDoneCounter]);

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
                initializedRef.current = false;
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
                batchRef.current = true;
                ecApiRef.current.expandAll();
                batchRef.current = false;
                if (cyRef.current) {
                    recomputeEdges(cyRef.current, rawEdgesRef.current);
                    applyEdgeVisibility(cyRef.current, edgeModeRef.current);
                    await runElkLayout(cyRef.current);
                }
            } catch (_e) {
                batchRef.current = false;
            }
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
                batchRef.current = true;
                ecApiRef.current.collapseAll();
                // Re-expand root — root namespace always stays expanded
                const rootNode = cyRef.current && cyRef.current.getElementById('ns:/');
                if (rootNode && rootNode.hasClass('cy-expand-collapse-collapsed-node')) {
                    try { ecApiRef.current.expand(rootNode); } catch (_e) { /* */ }
                }
                batchRef.current = false;
                if (cyRef.current) {
                    recomputeEdges(cyRef.current, rawEdgesRef.current);
                    applyEdgeVisibility(cyRef.current, edgeModeRef.current);
                    await runElkLayout(cyRef.current);
                }
            } catch (_e) {
                batchRef.current = false;
            }
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
            cy.animate({ fit: { eles: nsNode, padding: 40 }, duration: 300 });
        } else if (ns === '/') {
            cy.animate({ fit: { eles: cy.elements(), padding: 40 }, duration: 300 });
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
                    <button class="graph-btn" onClick=${handleExpandAll} title="Expand all groups">Expand All</button>
                    <button class="graph-btn" onClick=${handleCollapseAll} title="Collapse all groups">Collapse All</button>
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
                'width': function(ele) {
                    const label = ele.data('label') || '';
                    return Math.max(label.length * 7 + 12, 40);
                },
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
        // Port nodes (small circles on NS border for edge bundling)
        {
            selector: 'node.port-node',
            style: {
                'width': 8,
                'height': 8,
                'shape': 'ellipse',
                'background-color': isDark ? '#6b7280' : '#9ca3af',
                'border-width': 0,
                'label': '',
                'opacity': 0.6,
                'min-width': '0px',
                'min-height': '0px',
                'padding': '0px',
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
        // Branch edges: internal node → port (thin, dotted, no arrow).
        // Haystack avoids "invalid endpoints" warnings when port overlaps leaf.
        {
            selector: 'edge.branch-edge',
            style: {
                'curve-style': 'haystack',
                'haystack-radius': 0.5,
                'line-style': 'dotted',
                'width': 1,
                'opacity': 0.3,
                'target-arrow-shape': 'none',
            },
        },
        // Trunk edges: port → external (thicker, aggregated)
        {
            selector: 'edge.trunk-edge',
            style: {
                'width': function(ele) {
                    const c = ele.data('topicCount') || 1;
                    return Math.min(2 + Math.log2(c) * 1.5, 8);
                },
                'opacity': 0.8,
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
