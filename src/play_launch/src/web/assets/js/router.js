// URL routing (phase 105 W5): the page and the selected member live in the
// hash, `#/<view>` or `#/<view>/<member id>`, so a reload keeps them and a
// link opens them. Side-effect module — import to activate.
//
// A change of page pushes a history entry, so Back returns to the previous
// page; a change of selection within a page replaces it.

import { effect } from './vendor/signals.module.js';
import { currentView, selectedNode, nodePanelOpen } from './store.js';

const VIEWS = new Set([
    'health', 'nodes', 'graph', 'launch-tree', 'entities', 'chains', 'diagnostics',
]);

/** Pages whose node panel shows `selectedNode`. */
const MEMBER_VIEWS = new Set(['nodes', 'entities']);

/** `{view, member}` from a hash; `view` null when it names no page. */
export function parseHash(hash) {
    const parts = (hash || '').replace(/^#\/?/, '').split('/');
    const view = VIEWS.has(parts[0]) ? parts[0] : null;
    let member = null;
    if (view && parts.length > 1 && parts[1]) {
        try {
            member = decodeURIComponent(parts.slice(1).join('/'));
        } catch (_) {
            member = null;
        }
    }
    return { view, member };
}

/** The hash for a page and an optional member. */
export function formatHash(view, member) {
    return '#/' + view + (member ? '/' + encodeURIComponent(member) : '');
}

function applyHash() {
    const { view, member } = parseHash(window.location.hash);
    if (!view) return;
    currentView.value = view;
    if (member && MEMBER_VIEWS.has(view)) {
        selectedNode.value = member;
        nodePanelOpen.value = true;
    } else if (MEMBER_VIEWS.has(view)) {
        nodePanelOpen.value = false;
    }
}

applyHash();
window.addEventListener('hashchange', applyHash);

let lastView = currentView.peek();
effect(() => {
    const view = currentView.value;
    const member = MEMBER_VIEWS.has(view) && nodePanelOpen.value ? selectedNode.value : null;
    const hash = formatHash(view, member);
    if (window.location.hash === hash) {
        lastView = view;
        return;
    }
    if (view !== lastView) {
        window.history.pushState(null, '', hash);
    } else {
        window.history.replaceState(null, '', hash);
    }
    lastView = view;
});
