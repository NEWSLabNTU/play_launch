// State store for play_launch web UI.
// Single source of truth using Preact signals.

import { signal, computed } from './vendor/signals.module.js';

// --- Node state ---

/** @type {import('./vendor/signals.module.js').Signal<Map<string, object>>} */
export const nodes = signal(new Map());

// --- UI state ---

/** @type {import('./vendor/signals.module.js').Signal<string|null>} */
export const selectedNode = signal(null);

/** @type {import('./vendor/signals.module.js').Signal<boolean>} */
export const panelOpen = signal(false);

/** @type {import('./vendor/signals.module.js').Signal<'info'|'stdout'|'stderr'>} */
export const activeTab = signal('stderr');

/** @type {import('./vendor/signals.module.js').Signal<'light'|'dark'>} */
export const theme = signal(localStorage.getItem('theme') || 'light');

/** @type {import('./vendor/signals.module.js').Signal<'nodes'|'diagnostics'>} */
export const currentView = signal('nodes');

// --- Diagnostics ---

/** @type {import('./vendor/signals.module.js').Signal<object[]>} */
export const diagnostics = signal([]);

// --- Health summary ---

/** @type {import('./vendor/signals.module.js').Signal<object>} */
export const healthSummary = signal({
    nodes_running: 0,
    nodes_total: 0,
    containers_running: 0,
    containers_total: 0,
    composable_loaded: 0,
    composable_total: 0,
});

// --- Computed values ---

export const nodeList = computed(() => Array.from(nodes.value.values()));

export const nodeCount = computed(() => nodes.value.size);

// --- Actions ---

/** Replace all node state from GET /api/nodes response. */
export function loadNodes(nodeArray) {
    const map = new Map();
    for (const node of nodeArray) {
        map.set(node.name, node);
    }
    nodes.value = map;
}

/** Apply a single SSE StateEvent to the store.
 *
 * Events use serde tag="type" rename_all="snake_case", so the JSON looks like:
 *   {"type":"started","name":"foo","pid":123}
 *   {"type":"exited","name":"foo","exit_code":0}
 *   {"type":"load_succeeded","name":"foo","full_node_name":"/ns/foo","unique_id":1}
 */
export function applyStateEvent(event) {
    const name = event.name;
    if (!name) return;

    const map = new Map(nodes.value);
    const node = map.get(name);
    if (!node) return; // unknown node — wait for full resync

    // Clone to trigger signal update
    const updated = { ...node };

    switch (event.type) {
        case 'started':
            updated.status = { type: 'Process', value: { status: 'running' } };
            updated.pid = event.pid;
            break;

        case 'exited':
            if (event.exit_code != null && event.exit_code !== 0) {
                updated.status = { type: 'Process', value: { status: 'failed' } };
            } else {
                updated.status = { type: 'Process', value: { status: 'stopped' } };
            }
            updated.pid = null;
            break;

        case 'respawning':
            updated.status = { type: 'Process', value: { status: 'running' } };
            break;

        case 'terminated':
            updated.status = { type: 'Process', value: { status: 'stopped' } };
            updated.pid = null;
            break;

        case 'failed':
            updated.status = { type: 'Process', value: { status: 'failed' } };
            updated.pid = null;
            break;

        case 'load_started':
            updated.status = { type: 'Composable', value: { status: 'loading' } };
            break;

        case 'load_succeeded':
            updated.status = { type: 'Composable', value: { status: 'loaded' } };
            break;

        case 'load_failed':
            updated.status = { type: 'Composable', value: { status: 'failed' } };
            break;

        case 'unloaded':
            updated.status = { type: 'Composable', value: { status: 'unloaded' } };
            break;

        case 'blocked':
            updated.status = {
                type: 'Composable',
                value: { status: 'blocked', reason: event.reason },
            };
            break;

        default:
            return; // unknown event type — ignore
    }

    map.set(name, updated);
    nodes.value = map;
}

/** Fetch full node list from the API and update the store. */
export async function fetchNodes() {
    try {
        const resp = await fetch('/api/nodes');
        if (!resp.ok) return;
        const data = await resp.json();
        loadNodes(data);
    } catch (e) {
        console.warn('[store] Failed to fetch nodes:', e);
    }
}

/** Fetch health summary from the API and update the store. */
export async function fetchHealth() {
    try {
        const resp = await fetch('/api/health');
        if (!resp.ok) return;
        healthSummary.value = await resp.json();
    } catch (e) {
        console.warn('[store] Failed to fetch health:', e);
    }
}
