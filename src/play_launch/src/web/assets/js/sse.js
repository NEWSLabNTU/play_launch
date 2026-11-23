// SSE connection manager for play_launch web UI.
// Connects to /api/state/updates and applies events to the store.
// Side-effect module — import to activate.
//
// One connection carries everything global (phase 105 W3): unnamed events
// are StateEvents, and named events carry the header's figures (`summary`,
// `issue_counts`, `diagnostic_counts`, `recording`), each member's stderr
// `activity`, and the system `metrics`, each sent on connect and when it changes. The metrics used to
// need a second EventSource, and the rest were polled or fetched once.

import {
    applyStateEvent, applyActivity, fetchNodes, fetchGraph, graphWanted, fetchLaunchTree, connected,
    systemMetrics, healthSummary, healthIssueCounts, diagnosticCounts, eventRecording,
} from './store.js';

/** Named events on the global stream, and the signal each one sets. */
const NAMED = {
    summary: (v) => { healthSummary.value = v; },
    issue_counts: (v) => { healthIssueCounts.value = v; },
    diagnostic_counts: (v) => { diagnosticCounts.value = v; },
    recording: (v) => { eventRecording.value = v; },
    metrics: (v) => { systemMetrics.value = v; },
    activity: (v) => applyActivity(v),
};

/** How long to wait without any SSE message before declaring disconnect (ms). */
const KEEPALIVE_TIMEOUT_MS = 8000;

/** How often to probe the server when disconnected (ms). */
const RECONNECT_INTERVAL_MS = 2000;

let eventSource = null;
let keepAliveTimer = null;
let reconnectTimer = null;

/** Reset the keep-alive watchdog. Called on every SSE message/comment. */
function resetKeepAlive() {
    clearTimeout(keepAliveTimer);
    keepAliveTimer = setTimeout(() => {
        console.debug('[sse] Keep-alive timeout — server unresponsive');
        disconnect();
        startReconnectLoop();
    }, KEEPALIVE_TIMEOUT_MS);
}

/** Tear down the current EventSource. */
function disconnect() {
    clearTimeout(keepAliveTimer);
    if (eventSource) {
        eventSource.onopen = null;
        eventSource.onmessage = null;
        eventSource.onerror = null;
        eventSource.close();
        eventSource = null;
    }
    if (connected.value !== null) {
        connected.value = false;
    }
}

/** Create a fresh EventSource and wire up handlers. */
function connect() {
    disconnect();
    clearInterval(reconnectTimer);
    reconnectTimer = null;

    eventSource = new EventSource('/api/state/updates');

    eventSource.onopen = () => {
        console.debug('[sse] Connected to /api/state/updates');
        connected.value = true;
        resetKeepAlive();
        // Full resync on every (re)connect to cover missed events. The
        // header's figures arrive on the stream itself.
        fetchNodes();
        if (graphWanted.value) fetchGraph();
    };

    eventSource.onmessage = (e) => {
        resetKeepAlive();
        if (e.data === 'keep-alive') { return; }
        try {
            const event = JSON.parse(e.data);
            applyStateEvent(event);
        } catch (err) {
            console.warn('[sse] Failed to parse event:', err, e.data);
        }
    };

    eventSource.addEventListener('refresh', () => {
        console.warn('[sse] Server sent refresh (lagged) — re-syncing state');
        fetchNodes();
        if (graphWanted.value) fetchGraph();
    });

    for (const [name, apply] of Object.entries(NAMED)) {
        eventSource.addEventListener(name, (e) => {
            resetKeepAlive();
            try {
                apply(JSON.parse(e.data));
            } catch (err) {
                console.warn(`[sse] Failed to parse ${name}:`, err, e.data);
            }
        });
    }

    eventSource.onerror = () => {
        console.debug('[sse] Connection error');
        disconnect();
        startReconnectLoop();
    };
}

/** Periodically probe the server and reconnect when it's back. */
function startReconnectLoop() {
    if (reconnectTimer) { return; } // already running
    reconnectTimer = setInterval(async () => {
        try {
            const resp = await fetch('/api/health', { signal: AbortSignal.timeout(3000) });
            // Any HTTP answer means the server is up.
            if (resp.status > 0) {
                console.debug('[sse] Server is back — reconnecting');
                connect(); // clears the interval
            }
        } catch {
            // still down
        }
    }, RECONNECT_INTERVAL_MS);
}

// Initial load
fetchNodes();
fetchLaunchTree();
connect();
