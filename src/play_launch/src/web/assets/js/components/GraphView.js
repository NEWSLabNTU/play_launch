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
import expandCollapse from '../vendor/cytoscape-expand-collapse.js';
import {
    graphSnapshot, nodes, panelOpen,
    getStatusString, currentView, graphSelectedElement,
} from '../store.js';

import { namespaceLevels, statusToColor } from './graph-utils.js';
import { buildCyNodes, buildRawEdges } from './graph-builders.js';
import { traceEdgePaths, NEIGHBOR_CLASSES, recomputeEdges } from './graph-edges.js';
import { runElkLayout, updateScrollbars } from './graph-layout.js';
import { getCyStyle } from './graph-styles.js';

const html = htm.bind(h);

// Register extensions once
let _registered = false;
function ensureRegistered() {
    if (_registered) return;
    _registered = true;
    cytoscape.use(expandCollapse);
}

// Expand-collapse extension event names (lowercase — verified against vendored source).
// Centralised here so a case mismatch is caught once, not scattered across handlers.
const EC_EVENTS = {
    AFTER_EXPAND:  'expandcollapse.afterexpand',
    AFTER_COLLAPSE: 'expandcollapse.aftercollapse',
};

// Note: event names verified by grepping vendored source. A runtime toString()
// smoke-check is not feasible because the exported register function doesn't
// contain the internal event-emitting code.

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
    const hThumbRef = useRef(null);
    const vThumbRef = useRef(null);
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
                userZoomingEnabled: false,
            });

            // Register expand-collapse (cue disabled — we show ▾/▸ in labels)
            const ecApi = cy.expandCollapse({
                layoutBy: null,        // we handle layout ourselves
                fisheye: false,
                animate: false,
                undoable: false,
                cueEnabled: false,
            });

            // Interactions — click any node or edge to highlight and select
            // it in the graph panel. Single click drives the panel.
            cy.on('tap', 'node, edge', (evt) => {
                cy.elements().removeClass('highlighted ' + NEIGHBOR_CLASSES);
                const target = evt.target;
                if (target.isNode()) {
                    if (target.hasClass('leaf-node') ||
                        (target.hasClass('namespace') && target.hasClass('cy-expand-collapse-collapsed-node'))) {
                        // Leaf or collapsed NS: trace edges + color neighbors
                        const { path, inputNeighbors, outputNeighbors } = traceEdgePaths(target);
                        path.addClass('highlighted');
                        // Classify far-end neighbors by direction
                        inputNeighbors.forEach(n => {
                            n.addClass(outputNeighbors.contains(n) ? 'neighbor-bidir' : 'neighbor-input');
                        });
                        outputNeighbors.forEach(n => {
                            if (!inputNeighbors.contains(n)) n.addClass('neighbor-output');
                        });
                        // Build endpoint data for panel
                        const buildEndpoints = (col) => col.map(n => ({
                            id: n.id(), label: n.data('label') || n.id(),
                            fqn: n.data('fqn'), memberName: n.data('memberName'),
                            isNs: n.hasClass('namespace'), fullNs: n.data('fullNs'),
                        }));
                        const epData = {
                            inputEndpoints: buildEndpoints(inputNeighbors),
                            outputEndpoints: buildEndpoints(outputNeighbors),
                        };
                        graphSelectedElement.value = target.hasClass('leaf-node')
                            ? { type: 'node', id: target.id(), data: { ...target.data(), ...epData } }
                            : { type: 'namespace', id: target.id(), data: { ...target.data(), ...epData } };
                    } else if (target.hasClass('namespace')) {
                        // Expanded NS: only border highlight
                        target.addClass('highlighted');
                        graphSelectedElement.value = {
                            type: 'namespace', id: target.id(), data: target.data(),
                        };
                    }
                    panelOpen.value = true;
                } else {
                    // Edge: trace full path through ports + color terminals
                    target.addClass('highlighted');
                    const srcNode = target.source();
                    const tgtNode = target.target();

                    // Trace backward from source through ports → inputTerminals
                    const inputTerminals = cy.collection();
                    const outputTerminals = cy.collection();
                    const visited = new Set([target.id()]);

                    function traceBack(node) {
                        if (visited.has(node.id())) return;
                        visited.add(node.id());
                        if (node.hasClass('port-node')) {
                            node.addClass('highlighted');
                            node.connectedEdges().forEach(e => {
                                if (e.target().id() !== node.id()) return;
                                if (visited.has(e.id())) return;
                                visited.add(e.id());
                                e.addClass('highlighted');
                                traceBack(e.source());
                            });
                        } else {
                            inputTerminals.merge(node);
                        }
                    }

                    function traceForward(node) {
                        if (visited.has(node.id())) return;
                        visited.add(node.id());
                        if (node.hasClass('port-node')) {
                            node.addClass('highlighted');
                            node.connectedEdges().forEach(e => {
                                if (e.source().id() !== node.id()) return;
                                if (visited.has(e.id())) return;
                                visited.add(e.id());
                                e.addClass('highlighted');
                                traceForward(e.target());
                            });
                        } else {
                            outputTerminals.merge(node);
                        }
                    }

                    traceBack(srcNode);
                    traceForward(tgtNode);

                    // Classify terminals by direction
                    inputTerminals.forEach(n => {
                        n.addClass(outputTerminals.contains(n) ? 'neighbor-bidir' : 'neighbor-input');
                    });
                    outputTerminals.forEach(n => {
                        if (!inputTerminals.contains(n)) n.addClass('neighbor-output');
                    });

                    // Build endpoint data for the panel
                    const inputEndpoints = inputTerminals.map(n => ({
                        id: n.id(), label: n.data('label') || n.id(),
                        fqn: n.data('fqn'), memberName: n.data('memberName'),
                        isNs: n.hasClass('namespace'), fullNs: n.data('fullNs'),
                    }));
                    const outputEndpoints = outputTerminals.map(n => ({
                        id: n.id(), label: n.data('label') || n.id(),
                        fqn: n.data('fqn'), memberName: n.data('memberName'),
                        isNs: n.hasClass('namespace'), fullNs: n.data('fullNs'),
                    }));

                    graphSelectedElement.value = {
                        type: 'edge', id: target.id(),
                        data: {
                            ...target.data(),
                            source: srcNode.id(),
                            target: tgtNode.id(),
                            inputEndpoints,
                            outputEndpoints,
                        },
                    };
                    panelOpen.value = true;
                }
            });

            // Double-tap on namespace → toggle expand/collapse
            cy.on('dbltap', 'node.namespace', (evt) => {
                const target = evt.target;
                if (!ecApiRef.current) return;
                // Protect root from collapse
                if (target.data('fullNs') === '/') {
                    if (target.hasClass('cy-expand-collapse-collapsed-node')) {
                        try { ecApiRef.current.expand(target); } catch (_) { /* */ }
                    }
                    return;
                }
                if (target.hasClass('cy-expand-collapse-collapsed-node')) {
                    try { ecApiRef.current.expand(target); } catch (_) { /* */ }
                } else if (target.isParent()) {
                    try { ecApiRef.current.collapse(target); } catch (_) { /* */ }
                }
            });

            // Post-expand/collapse — recompute edges + layout.
            // Skipped during batch operations (collapseAll/expandAll) which
            // handle edges and layout themselves after the batch completes.
            cy.on(EC_EVENTS.AFTER_EXPAND + ' ' + EC_EVENTS.AFTER_COLLAPSE, async (evt) => {
                if (batchRef.current) return;
                const evtType = evt.type.includes('expand') ? 'expand' : 'collapse';
                const target = evt.target;
                const ns = target.data('fullNs') || target.id();
                console.log('[GraphView] %s: %s (id=%s)', evtType, ns, target.id());

                // Protect root namespace from being collapsed
                const rootNode = cy.getElementById('ns:/');
                if (rootNode.hasClass('cy-expand-collapse-collapsed-node') && ecApiRef.current) {
                    try { ecApiRef.current.expand(rootNode); } catch (_) { /* */ }
                }
                recomputeEdges(cy, rawEdgesRef.current);
                applyEdgeVisibility(cy, edgeModeRef.current);
                await runElkLayout(cy);

                // Auto-focus expanded namespace
                if (evtType === 'expand') {
                    cy.elements().removeClass('highlighted ' + NEIGHBOR_CLASSES);
                    target.addClass('highlighted');
                    graphSelectedElement.value = {
                        type: 'namespace', id: target.id(), data: target.data(),
                    };
                    panelOpen.value = true;
                }

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
                    cy.elements().removeClass('highlighted ' + NEIGHBOR_CLASSES);
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

            // Expanded namespaces are auto-positioned — don't let users drag them
            cy.on('grab', 'node.namespace', (evt) => {
                const n = evt.target;
                if (!n.hasClass('cy-expand-collapse-collapsed-node')) {
                    n.ungrabify();
                    setTimeout(() => n.grabify(), 0);
                }
            });

            // ── NS border resize ────────────────────────────────────
            // Allow users to drag expanded NS borders to resize them.
            // Uses invisible spacer nodes at NS extents to control
            // compound node auto-sizing.
            {
                const RESIZE_THRESHOLD = 12; // px from border to trigger resize
                let resizeState = null; // { nsId, edge, startModel, startBB }
                const container = containerRef.current;

                /** Find if cursor (model coords) is near an expanded NS border. */
                function findNsBorder(mx, my) {
                    let best = null;
                    let bestDist = RESIZE_THRESHOLD / cy.zoom();
                    cy.nodes('.namespace').forEach(n => {
                        if (n.hasClass('cy-expand-collapse-collapsed-node')) return;
                        if (n.data('fullNs') === '/') return; // don't resize root
                        const bb = n.boundingBox();
                        // Check if cursor is inside or very near the NS box
                        const inY = my >= bb.y1 - bestDist && my <= bb.y2 + bestDist;
                        const inX = mx >= bb.x1 - bestDist && mx <= bb.x2 + bestDist;
                        if (!inX || !inY) return;

                        const dL = Math.abs(mx - bb.x1);
                        const dR = Math.abs(mx - bb.x2);
                        const dT = Math.abs(my - bb.y1);
                        const dB = Math.abs(my - bb.y2);
                        const minD = Math.min(dL, dR, dT, dB);
                        if (minD < bestDist) {
                            bestDist = minD;
                            let edge;
                            if (minD === dL) edge = 'left';
                            else if (minD === dR) edge = 'right';
                            else if (minD === dT) edge = 'top';
                            else edge = 'bottom';
                            best = { nsNode: n, edge, bb };
                        }
                    });
                    return best;
                }

                function getCursorForEdge(edge) {
                    if (edge === 'left' || edge === 'right') return 'ew-resize';
                    return 'ns-resize';
                }

                /** Ensure spacer nodes exist at current NS extents. */
                function ensureSpacers(nsNode) {
                    const nsId = nsNode.id();
                    const bb = nsNode.boundingBox();
                    const spacerIds = [
                        nsId + ':spacer:tl', nsId + ':spacer:tr',
                        nsId + ':spacer:bl', nsId + ':spacer:br',
                    ];
                    const positions = [
                        { x: bb.x1 + 2, y: bb.y1 + 2 },
                        { x: bb.x2 - 2, y: bb.y1 + 2 },
                        { x: bb.x1 + 2, y: bb.y2 - 2 },
                        { x: bb.x2 - 2, y: bb.y2 - 2 },
                    ];
                    for (let i = 0; i < 4; i++) {
                        let spacer = cy.getElementById(spacerIds[i]);
                        if (!spacer || spacer.length === 0) {
                            cy.add({
                                group: 'nodes',
                                data: { id: spacerIds[i], parent: nsId, label: '', isSpacer: true },
                                classes: 'spacer-node',
                                position: positions[i],
                            });
                        } else {
                            spacer.position(positions[i]);
                        }
                    }
                    return spacerIds;
                }

                container.addEventListener('mousemove', (e) => {
                    if (resizeState) {
                        // Active resize — move spacer nodes
                        e.preventDefault();
                        e.stopPropagation();
                        const rect = container.getBoundingClientRect();
                        const rx = e.clientX - rect.left;
                        const ry = e.clientY - rect.top;
                        const zoom = cy.zoom();
                        const pan = cy.pan();
                        const mx = (rx - pan.x) / zoom;
                        const my = (ry - pan.y) / zoom;

                        const { nsId, edge, spacerIds, startBB } = resizeState;
                        const tl = cy.getElementById(spacerIds[0]);
                        const tr = cy.getElementById(spacerIds[1]);
                        const bl = cy.getElementById(spacerIds[2]);
                        const br = cy.getElementById(spacerIds[3]);

                        if (edge === 'right') {
                            const newX = Math.max(mx, startBB.x1 + 40);
                            tr.position({ x: newX, y: tr.position().y });
                            br.position({ x: newX, y: br.position().y });
                        } else if (edge === 'left') {
                            const newX = Math.min(mx, startBB.x2 - 40);
                            tl.position({ x: newX, y: tl.position().y });
                            bl.position({ x: newX, y: bl.position().y });
                        } else if (edge === 'bottom') {
                            const newY = Math.max(my, startBB.y1 + 30);
                            bl.position({ x: bl.position().x, y: newY });
                            br.position({ x: br.position().x, y: newY });
                        } else if (edge === 'top') {
                            const newY = Math.min(my, startBB.y2 - 30);
                            tl.position({ x: tl.position().x, y: newY });
                            tr.position({ x: tr.position().x, y: newY });
                        }
                        return;
                    }

                    // Not resizing — update cursor based on proximity to NS borders
                    const rect = container.getBoundingClientRect();
                    const rx = e.clientX - rect.left;
                    const ry = e.clientY - rect.top;
                    const zoom = cy.zoom();
                    const pan = cy.pan();
                    const mx = (rx - pan.x) / zoom;
                    const my = (ry - pan.y) / zoom;

                    const hit = findNsBorder(mx, my);
                    if (hit) {
                        container.style.cursor = getCursorForEdge(hit.edge);
                    } else {
                        container.style.cursor = '';
                    }
                });

                container.addEventListener('mousedown', (e) => {
                    if (e.button !== 0) return;
                    const rect = container.getBoundingClientRect();
                    const rx = e.clientX - rect.left;
                    const ry = e.clientY - rect.top;
                    const zoom = cy.zoom();
                    const pan = cy.pan();
                    const mx = (rx - pan.x) / zoom;
                    const my = (ry - pan.y) / zoom;

                    const hit = findNsBorder(mx, my);
                    if (!hit) return;

                    e.preventDefault();
                    e.stopPropagation();

                    const spacerIds = ensureSpacers(hit.nsNode);
                    resizeState = {
                        nsId: hit.nsNode.id(),
                        edge: hit.edge,
                        spacerIds,
                        startBB: { ...hit.bb },
                    };
                    container.style.cursor = getCursorForEdge(hit.edge);
                    // Disable cy panning during resize
                    cy.userPanningEnabled(false);
                });

                const endResize = () => {
                    if (!resizeState) return;
                    resizeState = null;
                    container.style.cursor = '';
                    cy.userPanningEnabled(true);
                };
                container.addEventListener('mouseup', endResize);
                container.addEventListener('mouseleave', endResize);
            }

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

            // Pan/zoom clears tooltip and updates scrollbars
            cy.on('viewport', () => {
                setTooltip(null);
                updateScrollbars(cy, hThumbRef.current, vThumbRef.current);
            });

            // Two-finger trackpad: pan vs zoom.
            // Browsers set ctrlKey=true for pinch-to-zoom gestures on trackpads.
            // Two-finger scroll (same direction) → pan. Pinch (ctrlKey) → zoom.
            if (containerRef.current._wheelHandler) {
                containerRef.current.removeEventListener('wheel', containerRef.current._wheelHandler);
            }
            const wheelHandler = (e) => {
                e.preventDefault();
                if (e.ctrlKey) {
                    // Pinch-to-zoom (trackpad) or Ctrl+scroll (mouse) → zoom at cursor
                    const zoomFactor = Math.pow(1.01, -e.deltaY);
                    const rect = containerRef.current.getBoundingClientRect();
                    cy.zoom({
                        level: cy.zoom() * zoomFactor,
                        renderedPosition: { x: e.clientX - rect.left, y: e.clientY - rect.top },
                    });
                } else {
                    // Two-finger scroll (same direction) or mouse wheel → pan
                    cy.panBy({ x: -e.deltaX, y: -e.deltaY });
                }
            };
            containerRef.current._wheelHandler = wheelHandler;
            containerRef.current.addEventListener('wheel', wheelHandler, { passive: false });

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

    // Jump-to-node: listen for 'graph-jump' custom events from the panel.
    // Handles collapsed namespaces by expanding ancestors first.
    useEffect(() => {
        if (view !== 'graph') return;

        function onGraphJump(evt) {
            const targetId = evt.detail;
            if (!targetId || !cyRef.current) return;

            const cy = cyRef.current;
            let ele = cy.getElementById(targetId);

            // If not found, expand ancestor namespaces to reveal it.
            if (!ele || ele.length === 0) {
                let nsPath = null;
                if (targetId.startsWith('node:')) {
                    const fqn = targetId.substring(5);
                    const lastSlash = fqn.lastIndexOf('/');
                    nsPath = lastSlash <= 0 ? '/' : fqn.substring(0, lastSlash);
                } else if (targetId.startsWith('ns:')) {
                    nsPath = targetId.substring(3);
                }

                if (nsPath && ecApiRef.current) {
                    const levels = namespaceLevels(nsPath);
                    let expanded = false;
                    for (const ns of levels) {
                        if (ns === '/') continue;
                        const nsNode = cy.getElementById('ns:' + ns);
                        if (nsNode && nsNode.length > 0 && nsNode.hasClass('cy-expand-collapse-collapsed-node')) {
                            try { ecApiRef.current.expand(nsNode); expanded = true; } catch (_) { /* */ }
                        }
                    }

                    if (expanded) {
                        recomputeEdges(cy, rawEdgesRef.current);
                        applyEdgeVisibility(cy, edgeModeRef.current);
                        runElkLayout(cy).then(() => {
                            const retryEle = cy.getElementById(targetId);
                            if (retryEle && retryEle.length > 0) {
                                performJump(cy, retryEle);
                            }
                        });
                        return;
                    }
                }
                return;
            }

            performJump(cy, ele);
        }

        function performJump(cy, target) {
            cy.animate({ center: { eles: target }, duration: 300 });
            cy.elements().removeClass('highlighted ' + NEIGHBOR_CLASSES);

            if (target.hasClass('leaf-node') ||
                (target.hasClass('namespace') && target.hasClass('cy-expand-collapse-collapsed-node'))) {
                const { path, inputNeighbors, outputNeighbors } = traceEdgePaths(target);
                path.addClass('highlighted');
                inputNeighbors.forEach(n => {
                    n.addClass(outputNeighbors.contains(n) ? 'neighbor-bidir' : 'neighbor-input');
                });
                outputNeighbors.forEach(n => {
                    if (!inputNeighbors.contains(n)) n.addClass('neighbor-output');
                });
                const buildEndpoints = (col) => col.map(n => ({
                    id: n.id(), label: n.data('label') || n.id(),
                    fqn: n.data('fqn'), memberName: n.data('memberName'),
                    isNs: n.hasClass('namespace'), fullNs: n.data('fullNs'),
                }));
                const epData = {
                    inputEndpoints: buildEndpoints(inputNeighbors),
                    outputEndpoints: buildEndpoints(outputNeighbors),
                };
                graphSelectedElement.value = target.hasClass('leaf-node')
                    ? { type: 'node', id: target.id(), data: { ...target.data(), ...epData } }
                    : { type: 'namespace', id: target.id(), data: { ...target.data(), ...epData } };
            } else if (target.hasClass('namespace')) {
                target.addClass('highlighted');
                graphSelectedElement.value = {
                    type: 'namespace', id: target.id(), data: target.data(),
                };
            }
            panelOpen.value = true;
        }

        document.addEventListener('graph-jump', onGraphJump);
        return () => document.removeEventListener('graph-jump', onGraphJump);
    }, [view]);

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

    // Scrollbar drag-to-pan interaction
    useEffect(() => {
        function setupDrag(thumb, axis) {
            if (!thumb) return () => {};
            let dragging = false;
            let startPos = 0;
            let startScroll = 0;

            const onDown = (e) => {
                if (!cyRef.current) return;
                dragging = true;
                startPos = axis === 'h' ? e.clientX : e.clientY;
                const bb = cyRef.current.elements().boundingBox();
                const ext = cyRef.current.extent();
                startScroll = axis === 'h' ? (ext.x1 - bb.x1) / bb.w : (ext.y1 - bb.y1) / bb.h;
                thumb.setPointerCapture(e.pointerId);
                thumb.parentNode.classList.add('dragging');
                e.preventDefault();
            };
            const onMove = (e) => {
                if (!dragging || !cyRef.current) return;
                const cy = cyRef.current;
                const bb = cy.elements().boundingBox();
                const track = thumb.parentNode;
                const trackSize = axis === 'h' ? track.clientWidth : track.clientHeight;
                const delta = (axis === 'h' ? e.clientX : e.clientY) - startPos;
                const scrollDelta = delta / trackSize;
                const newFrac = startScroll + scrollDelta;
                const zoom = cy.zoom();
                if (axis === 'h') {
                    const targetX = bb.x1 + newFrac * bb.w;
                    cy.pan({ x: -targetX * zoom + cy.width() / 2, y: cy.pan().y });
                } else {
                    const targetY = bb.y1 + newFrac * bb.h;
                    cy.pan({ x: cy.pan().x, y: -targetY * zoom + cy.height() / 2 });
                }
            };
            const onUp = (e) => {
                if (!dragging) return;
                dragging = false;
                thumb.releasePointerCapture(e.pointerId);
                thumb.parentNode.classList.remove('dragging');
            };

            thumb.addEventListener('pointerdown', onDown);
            thumb.addEventListener('pointermove', onMove);
            thumb.addEventListener('pointerup', onUp);
            return () => {
                thumb.removeEventListener('pointerdown', onDown);
                thumb.removeEventListener('pointermove', onMove);
                thumb.removeEventListener('pointerup', onUp);
            };
        }

        const cleanH = setupDrag(hThumbRef.current, 'h');
        const cleanV = setupDrag(vThumbRef.current, 'v');
        return () => { cleanH(); cleanV(); };
    }, [snap]); // re-bind when snap changes (refs might be new)

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
                <div class="graph-scrollbar graph-scrollbar-h">
                    <div class="graph-scrollbar-thumb" ref=${hThumbRef}></div>
                </div>
                <div class="graph-scrollbar graph-scrollbar-v">
                    <div class="graph-scrollbar-thumb" ref=${vThumbRef}></div>
                </div>
            </div>
        </div>
    `;
}
