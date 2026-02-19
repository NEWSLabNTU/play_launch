// SSE connection manager for play_launch web UI.
// Connects to /api/state/updates and applies events to the store.
// Side-effect module — import to activate.

import { applyStateEvent, fetchNodes, fetchHealth, connected } from './store.js';

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
        // Full resync on every (re)connect to cover missed events
        fetchNodes();
        fetchHealth();
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
            if (resp.ok) {
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
fetchHealth();
connect();
