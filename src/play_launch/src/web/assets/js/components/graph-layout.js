// graph-layout.js — ELK layout engine integration + scrollbar helper.

import ELK from '../vendor/elk.bundled.esm.js';

const elk = new ELK();

/**
 * Convert visible Cytoscape nodes/edges to ELK hierarchical format.
 */
export function buildElkGraph(cy) {
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

    // Create ELK nodes for each visible Cytoscape node (skip spacer nodes)
    visibleNodes.forEach(cyNode => {
        if (cyNode.hasClass('spacer-node')) return;
        const id = cyNode.id();
        const isNs = cyNode.hasClass('namespace');
        const isCollapsed = cyNode.hasClass('cy-expand-collapse-collapsed-node');

        let w, h;
        if (cyNode.hasClass('leaf-node')) {
            // Match the style function exactly to avoid stale layoutDimensions
            const label = cyNode.data('label') || '';
            w = Math.max(label.length * 8 + 16, 50);
            h = 32;
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
        if (cyNode.hasClass('spacer-node')) return;
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
export function applyElkPositions(cy, elkResult, animate = true) {
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

    // Skip expanded compound nodes — let Cytoscape auto-size them from
    // children positions so parent NS always contains its children.
    const expandedCompounds = new Set();
    cy.nodes('.namespace').forEach(n => {
        if (!n.hasClass('cy-expand-collapse-collapsed-node')) {
            expandedCompounds.add(n.id());
        }
    });
    for (const id of expandedCompounds) {
        posMap.delete(id);
    }

    console.log('[GraphView] applyElkPositions: %d nodes, animate=%s', posMap.size, animate);

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
 * Snap port nodes to the nearest border of their parent NS bounding box.
 * Called after layout so ports sit on the NS region boundary, not inside it.
 *
 * Uses the bounding box of non-port children to avoid circular dependency
 * (ports influencing the box they're being snapped to). Falls back to
 * parent.boundingBox() if there are no non-port children.
 */
export function snapPortsToBorder(cy) {
    // Pre-compute non-port bounding boxes per parent NS
    const nsBBoxCache = new Map();
    function getNonPortBBox(parentNode) {
        const pid = parentNode.id();
        if (nsBBoxCache.has(pid)) return nsBBoxCache.get(pid);

        // Get children that are NOT port nodes or spacer nodes
        const nonPortChildren = parentNode.children().filter(c =>
            !c.hasClass('port-node') && !c.hasClass('spacer-node'));
        let bb;
        if (nonPortChildren.length > 0) {
            bb = nonPortChildren.boundingBox();
        } else {
            // No non-port children — use parent's own position + padding
            bb = parentNode.boundingBox();
        }
        nsBBoxCache.set(pid, bb);
        return bb;
    }

    cy.nodes('.port-node').forEach(port => {
        const parent = port.parent();
        if (!parent || parent.length === 0) return;

        const bb = getNonPortBBox(parent);
        const pos = port.position();

        // Add padding around the content box for the NS border
        const pad = 15;
        const x1 = bb.x1 - pad;
        const x2 = bb.x2 + pad;
        const y1 = bb.y1 - pad;
        const y2 = bb.y2 + pad;

        // Determine which port side based on edge direction:
        // outports should be on the right/bottom, inports on the left/top
        const isOutport = port.id().startsWith('outport:');

        // Find the connected edge to determine direction preference
        const edges = port.connectedEdges();
        let prefersRight = isOutport;  // outports generally face right (toward target)

        // Check if there's a connected non-port node to determine which side
        let connectedLeafX = null;
        edges.forEach(e => {
            const other = e.source().id() === port.id() ? e.target() : e.source();
            if (!other.hasClass('port-node')) {
                connectedLeafX = other.position().x;
            }
        });

        const snapped = { x: pos.x, y: pos.y };

        if (connectedLeafX !== null) {
            // Snap to the side opposite to the connected leaf (port faces outward)
            const midX = (x1 + x2) / 2;
            if (connectedLeafX < midX) {
                // Leaf is to the left → port on left border
                snapped.x = x1;
            } else {
                // Leaf is to the right → port on right border
                snapped.x = x2;
            }
            // Clamp Y to border range
            snapped.y = Math.max(y1, Math.min(pos.y, y2));
        } else {
            // Port-to-port segment: snap to nearest border
            const dLeft = Math.abs(pos.x - x1);
            const dRight = Math.abs(pos.x - x2);
            const dTop = Math.abs(pos.y - y1);
            const dBottom = Math.abs(pos.y - y2);
            const minD = Math.min(dLeft, dRight, dTop, dBottom);

            if (minD === dLeft) snapped.x = x1;
            else if (minD === dRight) snapped.x = x2;
            else if (minD === dTop) snapped.y = y1;
            else snapped.y = y2;
        }

        port.position(snapped);
    });
}

/**
 * Run ELK layout with concurrency guard (prevents overlapping layout calls).
 * @param {boolean} fit — if true, fit viewport to content after layout (initial load only)
 */
export const layoutRunning = { current: false, pending: false, pendingFit: false };

export async function runElkLayout(cy, fit) {
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
        if (elkGraph.children.length === 0) {
            console.warn('[GraphView] layout: ELK graph has 0 children, skipping');
            return;
        }
        console.log('[GraphView] layout: ELK graph has %d top-level children, %d edges',
            elkGraph.children.length, elkGraph.edges.length);
        const result = await elk.layout(elkGraph);
        await applyElkPositions(cy, result, animate);
        snapPortsToBorder(cy);
        if (fit) {
            cy.fit(undefined, 40);           // instant fit on initial load
        }
        // Non-initial layouts: don't change viewport — let user control zoom/pan
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

/** Update scrollbar thumb positions to reflect viewport within the full graph. */
export function updateScrollbars(cy, hThumb, vThumb) {
    if (!hThumb || !vThumb) return;
    const eles = cy.elements();
    if (eles.length === 0) {
        hThumb.parentNode.style.display = 'none';
        vThumb.parentNode.style.display = 'none';
        return;
    }
    const bb = eles.boundingBox();
    const ext = cy.extent();

    // Horizontal
    const hFrac = ext.w / bb.w;
    if (hFrac >= 1) {
        hThumb.parentNode.style.display = 'none';
    } else {
        hThumb.parentNode.style.display = '';
        const left = Math.max(0, Math.min((ext.x1 - bb.x1) / bb.w, 1 - hFrac));
        hThumb.style.width = (hFrac * 100) + '%';
        hThumb.style.left = (left * 100) + '%';
    }

    // Vertical
    const vFrac = ext.h / bb.h;
    if (vFrac >= 1) {
        vThumb.parentNode.style.display = 'none';
    } else {
        vThumb.parentNode.style.display = '';
        const top = Math.max(0, Math.min((ext.y1 - bb.y1) / bb.h, 1 - vFrac));
        vThumb.style.height = (vFrac * 100) + '%';
        vThumb.style.top = (top * 100) + '%';
    }
}
