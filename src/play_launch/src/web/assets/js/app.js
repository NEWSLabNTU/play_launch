// play_launch web UI — Preact app entry point.
// Phase 20.1: initializes the state store and SSE connection.
// Phase 20.2+: will mount Preact components here.

import * as store from './store.js';
import './sse.js'; // side-effect: connects SSE, populates store

// Expose store on window for debugging (browser console: store.nodes.value)
window.store = store;

console.debug('[app] play_launch Preact app initialized');
