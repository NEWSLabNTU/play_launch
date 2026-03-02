// graph-edges.js — Edge path tracing, hierarchical routing, and port-based bundling.

import { namespaceLevels, resolveVisible, getCollapsedSet } from './graph-utils.js';

// ─── Edge path tracing (for highlighting) ─────────────────────────────────

/**
 * Trace all edge segments reachable from a leaf or collapsed-NS node,
 * following the direction of edges through port nodes.
 *
 * Returns { path, inputNeighbors, outputNeighbors } where:
 * - path: ego node + all edges + intermediate port nodes (for 'highlighted')
 * - inputNeighbors: far-end nodes that publish TO ego (reached via backward)
 * - outputNeighbors: far-end nodes that subscribe FROM ego (reached via forward)
 */
export function traceEdgePaths(startNode) {
    const cy = startNode.cy();
    const path = cy.collection();
    const inputNeighbors = cy.collection();
    const outputNeighbors = cy.collection();
    path.merge(startNode);

    // Separate visited sets for forward/backward traversal of port chains.
    // Terminal (non-port) nodes are always collected — a node can appear in
    // both inputNeighbors and outputNeighbors (bidirectional).
    const visitedFwd = new Set([startNode.id()]);
    const visitedBwd = new Set([startNode.id()]);

    // Forward: follow edges where current node is the source
    function forward(node) {
        node.connectedEdges().forEach(edge => {
            if (edge.source().id() !== node.id()) return;
            if (path.contains(edge)) return;
            path.merge(edge);
            const tgt = edge.target();
            if (tgt.hasClass('port-node')) {
                if (visitedFwd.has(tgt.id())) return;
                visitedFwd.add(tgt.id());
                path.merge(tgt);
                forward(tgt);
            } else {
                outputNeighbors.merge(tgt);
            }
        });
    }

    // Backward: follow edges where current node is the target
    function backward(node) {
        node.connectedEdges().forEach(edge => {
            if (edge.target().id() !== node.id()) return;
            if (path.contains(edge)) return;
            path.merge(edge);
            const src = edge.source();
            if (src.hasClass('port-node')) {
                if (visitedBwd.has(src.id())) return;
                visitedBwd.add(src.id());
                path.merge(src);
                backward(src);
            } else {
                inputNeighbors.merge(src);
            }
        });
    }

    forward(startNode);
    backward(startNode);

    return { path, inputNeighbors, outputNeighbors };
}

/** CSS class names used for neighbor direction highlighting. */
export const NEIGHBOR_CLASSES = 'neighbor-input neighbor-output neighbor-bidir';

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
 * Extract the namespace string of a visible element's Cytoscape parent.
 * E.g., if element's parent is 'ns:/a/b', returns '/a/b'.
 * Returns null for root-level elements (no parent).
 */
function getContainerNs(cy, eleId) {
    const parentId = getVisibleParent(cy, eleId);
    if (!parentId) return null;
    return parentId.startsWith('ns:') ? parentId.substring(3) : null;
}

/**
 * Find the Lowest Common Ancestor of two namespace strings.
 * E.g., computeLCA('/a/b/c', '/a/d/e') → '/a'
 */
function computeLCA(nsA, nsB) {
    const levelsA = namespaceLevels(nsA);
    const levelsB = namespaceLevels(nsB);
    let lca = '/';
    for (let i = 0; i < Math.min(levelsA.length, levelsB.length); i++) {
        if (levelsA[i] === levelsB[i]) lca = levelsA[i];
        else break;
    }
    return lca;
}

/**
 * Compute the hierarchical route between two visible endpoints.
 * Returns { outports: string[], inports: string[] } listing namespace strings
 * for each border the edge must cross, or null for internal edges.
 *
 * Chain: srcVis → outport(srcNs) → ... → outport(lca+1) → inport(lca+1) → ... → inport(tgtNs) → tgtVis
 */
function computeRoute(cy, srcVis, tgtVis) {
    const srcNs = getContainerNs(cy, srcVis);
    const tgtNs = getContainerNs(cy, tgtVis);

    const effSrcNs = srcNs || '/';
    const effTgtNs = tgtNs || '/';

    // Same container namespace → internal edge
    if (effSrcNs === effTgtNs) return null;

    const lca = computeLCA(effSrcNs, effTgtNs);

    // Ascending outports: from srcNs up to (but not including) LCA, deepest first
    const srcLevels = namespaceLevels(effSrcNs);
    const lcaSrcIdx = srcLevels.indexOf(lca);
    const outports = srcLevels.slice(lcaSrcIdx + 1).reverse();

    // Descending inports: from just below LCA down to tgtNs
    const tgtLevels = namespaceLevels(effTgtNs);
    const lcaTgtIdx = tgtLevels.indexOf(lca);
    const inports = tgtLevels.slice(lcaTgtIdx + 1);

    if (outports.length === 0 && inports.length === 0) return null;

    return { outports, inports };
}

/**
 * Recompute edges with hierarchical port-based bundling for cross-namespace traffic.
 *
 * Edges merge/unmerge at every namespace boundary they cross. A port node on
 * each NS border acts as a merge point — edges converge entering a namespace,
 * then scatter to inner elements or nested namespace ports.
 *
 * For each cross-NS edge, the route is decomposed into segments through every
 * namespace border between source and target (via LCA computation).
 * Shared segments across multiple edges are merged (bundled).
 *
 * Branch edges: leaf/collapsed-NS ↔ port (thin, dotted, no arrow)
 * Trunk edges:  port ↔ port (thick, arrow, aggregated count)
 */
export function recomputeEdges(cy, rawEdges) {
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

    // Step 2: Compute hierarchical routes and accumulate segments.
    // For each aggregated edge, computeRoute() finds the LCA and builds the
    // chain of outports/inports at every namespace boundary. Shared segments
    // across multiple edges are merged via segmentMap.
    const internalEdges = [];
    const segmentMap = new Map(); // "hopSrc\0hopTgt" → { topics, dangling }

    for (const [key, data] of aggEdges) {
        const [src, tgt] = key.split('\0');
        const route = computeRoute(cy, src, tgt);

        if (!route) {
            internalEdges.push({ src, tgt, data });
            continue;
        }

        // Build the full chain: srcVis → outport(deepest) → ... → inport(...) → tgtVis
        const chain = [src];
        for (const ns of route.outports) chain.push('outport:ns:' + ns);
        for (const ns of route.inports) chain.push('inport:ns:' + ns);
        chain.push(tgt);

        // Decompose chain into segments and merge shared ones
        for (let i = 0; i < chain.length - 1; i++) {
            const segKey = chain[i] + '\0' + chain[i + 1];
            if (!segmentMap.has(segKey)) {
                segmentMap.set(segKey, { topics: new Set(), dangling: false });
            }
            const seg = segmentMap.get(segKey);
            for (const t of data.topics) seg.topics.add(t);
            if (data.dangling) seg.dangling = true;
        }
    }

    // Step 3: Create port nodes and segment edges.
    // Collect all port node IDs referenced by segments.
    const portSet = new Set();
    for (const segKey of segmentMap.keys()) {
        const [hopSrc, hopTgt] = segKey.split('\0');
        if (hopSrc.startsWith('outport:') || hopSrc.startsWith('inport:')) portSet.add(hopSrc);
        if (hopTgt.startsWith('outport:') || hopTgt.startsWith('inport:')) portSet.add(hopTgt);
    }

    // Remove old edges and port nodes
    cy.remove(cy.edges());
    cy.remove(cy.nodes('.port-node'));

    const els = [];

    // Create port nodes (each lives inside its namespace container)
    let outportCount = 0, inportCount = 0;
    for (const portId of portSet) {
        // portId is like 'outport:ns:/a/b' or 'inport:ns:/a/b'
        const parentId = portId.replace(/^(?:out|in)port:/, ''); // 'ns:/a/b'
        els.push({
            group: 'nodes',
            data: { id: portId, label: '', parent: parentId, isPort: true },
            classes: 'port-node',
        });
        if (portId.startsWith('outport:')) outportCount++;
        else inportCount++;
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

    // Create segment edges: classify as trunk (port↔port) or branch (leaf/NS endpoint)
    let trunkCount = 0, branchCount = 0;
    for (const [segKey, seg] of segmentMap) {
        const [hopSrc, hopTgt] = segKey.split('\0');
        const srcIsPort = hopSrc.startsWith('outport:') || hopSrc.startsWith('inport:');
        const tgtIsPort = hopTgt.startsWith('outport:') || hopTgt.startsWith('inport:');
        const isTrunk = srcIsPort && tgtIsPort;
        const count = seg.topics.size;

        if (isTrunk) {
            trunkCount++;
            els.push({
                group: 'edges',
                data: {
                    id: 'trunk:' + hopSrc + '->' + hopTgt,
                    source: hopSrc, target: hopTgt,
                    topicCount: count,
                    label: count === 1
                        ? Array.from(seg.topics)[0].split(' (')[0]
                        : count + ' topics',
                    topicList: Array.from(seg.topics).join('\n'),
                    dangling: seg.dangling,
                },
                classes: 'trunk-edge' + (seg.dangling ? ' dangling-edge' : ''),
            });
        } else {
            branchCount++;
            els.push({
                group: 'edges',
                data: {
                    id: 'branch:' + hopSrc + '->' + hopTgt,
                    source: hopSrc, target: hopTgt,
                },
                classes: 'branch-edge',
            });
        }
    }

    cy.add(els);
    console.log('[GraphView] edges: %d internal, %d segments (%d trunk, %d branch), %d ports [%d out, %d in]',
        internalEdges.length, segmentMap.size,
        trunkCount, branchCount,
        portSet.size, outportCount, inportCount);
}
