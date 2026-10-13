// State store for play_launch web UI.
// Single source of truth using Preact signals.

import { signal, computed, effect } from './vendor/signals.module.js';

// --- Node state ---

/** @type {{ value: Map<string, import('./types').NodeSummary> }} */
export const nodes = signal(new Map());

// --- Parameter cache (Phase 24) ---

/** Incremented when parameters change for a node, so the Params tab can re-fetch. */
export const parameterVersion = signal(new Map());

// --- UI state ---

export const selectedNode = signal(null);
export const nodePanelOpen = signal(false);
export const graphPanelOpen = signal(false);
export const activeTab = signal('stderr');
export const theme = signal(localStorage.getItem('theme') || 'light');
export const currentView = signal('health');

/** Status facet for the node list (phase-50): 'all' | 'active' | 'transitioning'
 *  | 'failed' | 'inactive' | 'blocked'. Set by the header health strip too. */
export const statusFacet = signal('all');

// --- Connection state ---

/** SSE connection state: null = initial (never connected), true = connected, false = lost. */
export const connected = signal(null);

// --- System metrics (Phase 26) ---

/** @type {{ value: import('./types').SystemStatsSnapshot|null }} */
export const systemMetrics = signal(null);

// --- Diagnostics ---

export const diagnostics = signal([]);

// --- Health summary ---

/** @type {{ value: import('./types').HealthSummary }} */
export const healthSummary = signal(/** @type {any} */ ({
    nodes_running: 0,
    nodes_total: 0,
    containers_running: 0,
    containers_total: 0,
    composable_loaded: 0,
    composable_total: 0,
}));

// --- Health issues (the Health page) ---

/** Open-issue counts for the Health tab badge: {error, warning, info}, or
 *  null when the server has no issue tracker for this run. Kept current by a
 *  5 s poll in the Header and by the Health page's own stream while open. */
export const healthIssueCounts = signal(null);

/** Apply a health response: the issue snapshot from /api/health/issues
 *  (sets the badge counts) or the member summary from /api/health (sets the
 *  header strip). Either shape is accepted, so one helper serves both. */
export function applyHealthResponse(data) {
    if (!data || typeof data !== 'object') return;
    if (data.counts && typeof data.counts === 'object') {
        healthIssueCounts.value = {
            error: data.counts.error || 0,
            warning: data.counts.warning || 0,
            info: data.counts.info || 0,
        };
    }
    if (typeof data.nodes_running === 'number') {
        healthSummary.value = data;
    } else if (data.summary && typeof data.summary.nodes_running === 'number') {
        healthSummary.value = data.summary;
    }
}

/** One-shot request the Health page's "Show" button leaves for the Entities
 *  page: a filter term to start with. Consumed on mount. */
export const entitiesFilterRequest = signal(null);

// --- Launch tree state (Phase 30) ---

/** @type {{ value: object|null }} Launch tree from GET /api/launch-tree */
export const launchTree = signal(null);

/** Currently selected scope or node in the launch tree view. */
export const launchTreeSelection = signal(null);

/** Expanded scope IDs in the launch tree view. */
export const launchTreeExpanded = signal(new Set([0]));

// --- Graph state (Phase 25) ---

/** @type {{ value: object|null }} Full graph snapshot from GET /api/graph */
export const graphSnapshot = signal(null);

/** Currently focused graph element, or null.
 *  Shape: { type: 'node'|'namespace'|'edge', id: string, data: object }
 */
export const graphSelectedElement = signal(null);


// --- Computed values ---

export const nodeList = computed(() => Array.from(nodes.value.values()));

export const nodeCount = computed(() => nodes.value.size);

/** Derive per-node topic counts from the graph snapshot.
 * @returns {Map<string, {pub: number, sub: number, srv: number, dangling: number}>}
 */
export const nodeTopicCounts = computed(() => {
    const snap = graphSnapshot.value;
    if (!snap || !snap.topics) return new Map();

    const counts = new Map();

    const ensure = (name) => {
        if (!counts.has(name)) counts.set(name, { pub: 0, sub: 0, srv: 0, dangling: 0 });
        return counts.get(name);
    };

    for (const topic of snap.topics) {
        for (const ep of topic.publishers) {
            if (ep.member_name) {
                const c = ensure(ep.member_name);
                c.pub++;
                if (topic.dangling) c.dangling++;
            }
        }
        for (const ep of topic.subscribers) {
            if (ep.member_name) {
                const c = ensure(ep.member_name);
                c.sub++;
                if (topic.dangling) c.dangling++;
            }
        }
    }


    return counts;
});

// --- Helpers ---

/** Extract the simple status string from a UnifiedStatus object.
 *
 * Serde formats:
 *   Process:    {"type": "Process",    "value": "running"}           — value is a string
 *   Composable: {"type": "Composable", "value": {"status": "loaded"}} — value is an object
 *
 * @param {import('./types').UnifiedStatus | null | undefined} status
 * @returns {string}
 */
export function getStatusString(status) {
    if (!status) return 'unknown';
    if (status.type === 'Process') {
        return (typeof status.value === 'string') ? status.value : 'unknown';
    }
    if (status.type === 'Composable') {
        return status.value?.status || 'unknown';
    }
    return 'unknown';
}

/** Look up a member by canonical id, falling back to display name.
 *
 * Launch-tree scope maps still key by display name (phase-50 residue);
 * duplicates resolve to the first match there — same as pre-phase-50.
 * @param {string} key
 */
/** Strip the `kind:` prefix off a canonical member id for display.
 *
 * Ids are `kind:/ns/name[#N]` (phase-50). Users think in FQNs, so the UI
 * shows `/alpha/worker`, not `node:/alpha/worker` — the prefix exists to keep
 * a node, a container and a composable with the same FQN distinct as KEYS,
 * which is not something to put in front of a reader.
 * @param {string} id
 */
export function memberFqn(id) {
    if (typeof id !== 'string') return id;
    const colon = id.indexOf(':/');
    return colon === -1 ? id : id.slice(colon + 1);
}

export function getNodeByKey(key) {
    const map = nodes.value;
    if (map.has(key)) return map.get(key);
    for (const n of map.values()) {
        if (n.name === key) return n;
    }
    return undefined;
}

// --- Actions ---

/** Replace all node state from GET /api/nodes response.
 * @param {import('./types').NodeSummary[]} nodeArray
 */
export function loadNodes(nodeArray) {
    const map = new Map();
    for (const node of nodeArray) {
        // Phase-50: canonical id (`kind:/ns/name[#N]`) is THE key — bare
        // names collide (issue 0001). Fallback keeps old servers working.
        map.set(node.id || node.name, node);
    }
    nodes.value = map;
}

/** Apply a single SSE StateEvent to the store.
 *
 * Events use serde tag="type" rename_all="snake_case", so the JSON looks like:
 *   {"type":"started","name":"foo","pid":123}
 *   {"type":"exited","name":"foo","exit_code":0}
 *   {"type":"load_succeeded","name":"foo","full_node_name":"/ns/foo","unique_id":1}
 *
 * @param {import('./types').StateEvent} event
 */
export function applyStateEvent(event) {
    // Actors are named by canonical member id (phase-50), so event.name IS
    // the store key.
    const name = event.name;
    if (!name) return;

    const map = new Map(nodes.value);
    const node = map.get(name);
    if (!node) {
        // Unknown member — refetch instead of silently dropping the event
        // (store.js:174 antipattern from the 2026-07 audit).
        debounceFetchNodes();
        return;
    }

    // Clone to trigger signal update
    const updated = { ...node };

    switch (event.type) {
        case 'started':
            updated.status = { type: 'Process', value: 'running' };
            updated.pid = event.pid;
            break;

        case 'exited':
            if (event.exit_code != null && event.exit_code !== 0) {
                updated.status = { type: 'Process', value: 'failed' };
            } else {
                updated.status = { type: 'Process', value: 'stopped' };
            }
            updated.pid = null;
            break;

        case 'respawning':
            updated.status = { type: 'Process', value: 'running' };
            break;

        case 'terminated':
            updated.status = { type: 'Process', value: 'stopped' };
            updated.pid = null;
            break;

        case 'failed':
            updated.status = { type: 'Process', value: 'failed' };
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

        case 'composable_respawning':
            // Status stays failed until the reload starts (load_started).
            updated.restart_count = event.restart_count;
            break;

        case 'blocked':
            // One BlockReason since phase-51: snake_case ("container_not_started")
            // on both the SSE and REST wires. The UI only checks the status
            // string, not the reason value.
            updated.status = {
                type: 'Composable',
                value: { status: 'blocked', reason: /** @type {any} */ (event.reason) },
            };
            break;

        case 'parameter_changed': {
            // Phase 24: bump parameter version so ParametersTab re-fetches
            const pmap = new Map(parameterVersion.value);
            pmap.set(name, (pmap.get(name) || 0) + 1);
            parameterVersion.value = pmap;
            return; // no node status update needed
        }

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
        applyHealthResponse(await resp.json());
    } catch (e) {
        console.warn('[store] Failed to fetch health:', e);
    }
}

/** Fetch the full communication graph snapshot (Phase 25). */
export async function fetchGraph() {
    try {
        const resp = await fetch('/api/graph');
        if (!resp.ok) return;
        graphSnapshot.value = await resp.json();
    } catch (e) {
        console.warn('[store] Failed to fetch graph:', e);
    }
}

/** Fetch the launch tree (Phase 30). One-time fetch — scopes are static. */
export async function fetchLaunchTree() {
    try {
        const resp = await fetch('/api/launch-tree');
        if (!resp.ok) return;
        launchTree.value = await resp.json();
    } catch (e) {
        console.warn('[store] Failed to fetch launch tree:', e);
    }
}

/** Debounced node-list refetch — called when SSE mentions an unknown member. */
let _nodesDebounceTimer = null;
export function debounceFetchNodes() {
    if (_nodesDebounceTimer) return;
    _nodesDebounceTimer = setTimeout(() => {
        _nodesDebounceTimer = null;
        fetchNodes();
    }, 300);
}

/** The graph is read only while something on screen shows it: the Graph
 * page, the Nodes page (its cards carry topic counts), or a Topics tab
 * (counted in `graphViewers`). Then once on opening and every 10 s. Building it
 * costs the server a walk of every topic and service, so it used to be
 * refetched after every state event — hundreds of times during a big
 * launch's startup. */
const GRAPH_REFRESH_MS = 10000;
let _graphTimer = null;
/** Mounted components that read the graph snapshot outside those pages. */
export const graphViewers = signal(0);
export const graphWanted = computed(() =>
    currentView.value === 'graph' || currentView.value === 'nodes' || graphViewers.value > 0);
effect(() => {
    if (graphWanted.value) {
        if (_graphTimer === null) {
            fetchGraph();
            _graphTimer = setInterval(fetchGraph, GRAPH_REFRESH_MS);
        }
    } else if (_graphTimer !== null) {
        clearInterval(_graphTimer);
        _graphTimer = null;
    }
});
