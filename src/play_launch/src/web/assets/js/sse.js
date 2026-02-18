// SSE connection manager for play_launch web UI.
// Connects to /api/state/updates and applies events to the store.
// Side-effect module — import to activate.

import { applyStateEvent, fetchNodes, fetchHealth } from './store.js';

let eventSource = null;

function connect() {
    if (eventSource) {
        eventSource.close();
    }

    eventSource = new EventSource('/api/state/updates');

    eventSource.onopen = () => {
        console.debug('[sse] Connected to /api/state/updates');
    };

    eventSource.onmessage = (e) => {
        if (e.data === 'keep-alive') return;
        try {
            const event = JSON.parse(e.data);
            applyStateEvent(event);
        } catch (err) {
            console.warn('[sse] Failed to parse event:', err, e.data);
        }
    };

    eventSource.onerror = () => {
        // EventSource auto-reconnects. On reconnect (re-open), do a full resync.
        console.debug('[sse] Connection error, will auto-reconnect');
        // Fetch full state on reconnect to cover any missed events.
        // EventSource fires onerror then re-opens, so the next onopen triggers resync.
        const prevOnOpen = eventSource.onopen;
        eventSource.onopen = () => {
            console.debug('[sse] Reconnected — resyncing full state');
            fetchNodes();
            fetchHealth();
            // Restore normal onopen
            eventSource.onopen = prevOnOpen;
        };
    };
}

// Initial load: fetch full state, then connect SSE for incremental updates.
fetchNodes();
fetchHealth();
connect();
