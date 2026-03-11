// graph-containment.js — Node containment enforcement and push mechanics.
//
// Prevents leaf nodes from visually overlapping sibling expanded namespace
// compounds during drag, and pushes sibling NS regions apart when they overlap
// after resize or layout.

const PAD = 4; // gap between node edge and NS border

/**
 * Get expanded NS nodes that are siblings of the given node
 * (share the same Cytoscape parent compound).
 */
export function getSiblingExpandedNs(cy, node) {
    const parentId = node.data('parent');
    if (!parentId) return [];
    const parent = cy.getElementById(parentId);
    if (!parent || parent.length === 0) return [];
    return parent.children('.namespace')
        .filter(n => !n.hasClass('cy-expand-collapse-collapsed-node') && n.id() !== node.id())
        .map(n => ({ node: n, bb: n.boundingBox() }));
}

/**
 * Get leaf-node dimensions matching the style function in graph-styles.js.
 */
export function getNodeDimensions(node) {
    if (node.hasClass('leaf-node')) {
        const label = node.data('label') || '';
        return { w: Math.max(label.length * 8 + 16, 50), h: 32 };
    }
    // Collapsed NS or other — use actual bounding box
    const bb = node.boundingBox();
    return { w: bb.w, h: bb.h };
}

/**
 * Get bounding box for any element. Leaf / collapsed-NS use computed
 * dimensions (matching the style formula); compounds use boundingBox().
 */
function getElementBB(node) {
    if (node.hasClass('leaf-node') ||
        node.hasClass('cy-expand-collapse-collapsed-node')) {
        const p = node.position();
        const d = getNodeDimensions(node);
        return { x1: p.x - d.w / 2, x2: p.x + d.w / 2,
                 y1: p.y - d.h / 2, y2: p.y + d.h / 2 };
    }
    return node.boundingBox();
}

/**
 * Constrain a node position so it doesn't overlap sibling expanded NS.
 * Returns { pos, pushes[] } where pushes are { node, dx, dy } for NS to shift.
 */
export function constrainPosition(pos, dims, siblingBBs, prevPos) {
    let { x, y } = pos;
    const hw = dims.w / 2, hh = dims.h / 2;
    const pushes = [];

    for (const sib of siblingBBs) {
        const { bb } = sib;
        // Node bounding rect
        const nx1 = x - hw, nx2 = x + hw;
        const ny1 = y - hh, ny2 = y + hh;

        // Check overlap
        if (nx2 + PAD <= bb.x1 || nx1 - PAD >= bb.x2 ||
            ny2 + PAD <= bb.y1 || ny1 - PAD >= bb.y2) continue;

        // Overlap detected — determine approach direction from prevPos
        const dx = x - prevPos.x;
        const dy = y - prevPos.y;

        // Compute penetration depths from each side
        const fromLeft = nx2 + PAD - bb.x1;   // node coming from left
        const fromRight = bb.x2 + PAD - nx1;  // node coming from right
        const fromTop = ny2 + PAD - bb.y1;    // node coming from top
        const fromBottom = bb.y2 + PAD - ny1; // node coming from bottom

        // Choose separation axis based on smallest penetration
        const minPen = Math.min(fromLeft, fromRight, fromTop, fromBottom);

        if (minPen === fromLeft && dx > 0) {
            x = bb.x1 - hw - PAD;
            pushes.push({ node: sib.node, dx: fromLeft, dy: 0 });
        } else if (minPen === fromRight && dx < 0) {
            x = bb.x2 + hw + PAD;
            pushes.push({ node: sib.node, dx: -fromRight, dy: 0 });
        } else if (minPen === fromTop && dy > 0) {
            y = bb.y1 - hh - PAD;
            pushes.push({ node: sib.node, dx: 0, dy: fromTop });
        } else if (minPen === fromBottom && dy < 0) {
            y = bb.y2 + hh + PAD;
            pushes.push({ node: sib.node, dx: 0, dy: -fromBottom });
        } else {
            // Fallback: push in min-penetration direction
            if (minPen === fromLeft) {
                x = bb.x1 - hw - PAD;
                pushes.push({ node: sib.node, dx: fromLeft, dy: 0 });
            } else if (minPen === fromRight) {
                x = bb.x2 + hw + PAD;
                pushes.push({ node: sib.node, dx: -fromRight, dy: 0 });
            } else if (minPen === fromTop) {
                y = bb.y1 - hh - PAD;
                pushes.push({ node: sib.node, dx: 0, dy: fromTop });
            } else {
                y = bb.y2 + hh + PAD;
                pushes.push({ node: sib.node, dx: 0, dy: -fromBottom });
            }
        }
    }

    return { pos: { x, y }, pushes };
}

/**
 * Shift a node by (dx, dy).
 *
 * Compound (parent) nodes: only shift non-compound descendants.
 * Cytoscape auto-computes a compound's position from its children, so
 * intermediate compounds must NOT be shifted directly — otherwise their
 * children get shifted twice (once explicitly, once by the compound's
 * position setter), causing the "double-speed" bug.
 */
function shiftNode(node, dx, dy) {
    if (node.isParent()) {
        node.descendants().forEach(n => {
            if (n.isParent()) return;  // skip intermediate compounds
            const p = n.position();
            n.position({ x: p.x + dx, y: p.y + dy });
        });
    } else {
        const p = node.position();
        node.position({ x: p.x + dx, y: p.y + dy });
    }
}

/**
 * Push all siblings of `node` that overlap it, with cascading.
 * The node itself is NOT shifted — only its overlapping siblings are.
 *
 * @param {object} cy     - Cytoscape instance
 * @param {object} node   - the source node (already in its final position)
 * @param {number} biasX  - preferred push direction (hint for cascading)
 * @param {number} biasY  - preferred push direction (hint for cascading)
 * @param {Set}  [visited] - IDs already processed (cycle guard)
 */
export function pushOverlappingSiblings(cy, node, biasX, biasY, visited) {
    if (!visited) visited = new Set();
    visited.add(node.id());

    const parentId = node.data('parent');
    if (!parentId) return;
    const parent = cy.getElementById(parentId);
    if (!parent || parent.length === 0) return;

    const nodeBB = getElementBB(node);

    parent.children().forEach(sib => {
        if (sib.id() === node.id()) return;
        if (visited.has(sib.id())) return;
        if (sib.hasClass('spacer-node') || sib.hasClass('port-node')) return;

        const sibBB = getElementBB(sib);

        // No overlap → skip
        if (nodeBB.x2 + PAD <= sibBB.x1 || nodeBB.x1 - PAD >= sibBB.x2 ||
            nodeBB.y2 + PAD <= sibBB.y1 || nodeBB.y1 - PAD >= sibBB.y2) return;

        const pushRight = nodeBB.x2 + PAD - sibBB.x1;
        const pushLeft  = sibBB.x2 + PAD - nodeBB.x1;
        const pushDown  = nodeBB.y2 + PAD - sibBB.y1;
        const pushUp    = sibBB.y2 + PAD - nodeBB.y1;

        let dx = 0, dy = 0;
        if (biasX > 0 && pushRight > 0)       dx = pushRight;
        else if (biasX < 0 && pushLeft > 0)   dx = -pushLeft;
        else if (biasY > 0 && pushDown > 0)   dy = pushDown;
        else if (biasY < 0 && pushUp > 0)     dy = -pushUp;
        else {
            const min = Math.min(pushRight, pushLeft, pushDown, pushUp);
            if (min === pushRight)      dx = pushRight;
            else if (min === pushLeft)  dx = -pushLeft;
            else if (min === pushDown)  dy = pushDown;
            else                        dy = -pushUp;
        }

        cascadePush(cy, sib, dx, dy, visited);
    });
}

/**
 * Shift a node by (dx, dy), then recursively push any siblings (same
 * Cytoscape parent) that now overlap. Propagates the push direction so
 * A->B->C chains resolve in one pass.
 */
export function cascadePush(cy, node, dx, dy, visited) {
    if (!visited) visited = new Set();
    if (visited.has(node.id())) return;

    shiftNode(node, dx, dy);
    pushOverlappingSiblings(cy, node, dx, dy, visited);
}

/**
 * Post-layout overlap resolution. Checks all leaf and collapsed-NS nodes
 * against sibling expanded NS and shifts to resolve overlaps.
 */
export function resolvePostLayoutOverlaps(cy) {
    const movableNodes = cy.nodes('.leaf-node, .namespace.cy-expand-collapse-collapsed-node');
    movableNodes.forEach(node => {
        const siblings = getSiblingExpandedNs(cy, node);
        if (siblings.length === 0) return;
        const pos = node.position();
        const dims = getNodeDimensions(node);
        const { pos: newPos } = constrainPosition(pos, dims, siblings, pos);
        if (newPos.x !== pos.x || newPos.y !== pos.y) {
            node.position(newPos);
        }
    });
}
