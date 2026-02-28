// play_launch web UI — Preact app entry point.

import { h, render } from './vendor/preact.module.js';
import htm from './vendor/htm.module.js';
import * as store from './store.js';
import './sse.js'; // side-effect: connects SSE, populates store
import { Header } from './components/Header.js';
import { NodeList } from './components/NodeList.js';
import { RightPanel } from './components/RightPanel.js';
import { PanelResizer } from './components/PanelResizer.js';
import { DiagnosticsView } from './components/DiagnosticsView.js';
import { GraphView } from './components/GraphView.js';

const html = htm.bind(h);

// Initialize theme
const savedTheme = localStorage.getItem('theme') ||
    (window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');
store.theme.value = savedTheme;
document.documentElement.setAttribute('data-theme', savedTheme);

// Watch for system theme changes
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', e => {
    if (!localStorage.getItem('theme')) {
        const t = e.matches ? 'dark' : 'light';
        store.theme.value = t;
        document.documentElement.setAttribute('data-theme', t);
    }
});

/** Blocking overlay shown when SSE connection is lost (not on initial load). */
function ConnectionOverlay() {
    const connState = store.connected.value;
    if (connState === null || connState === true) { return null; }

    return html`
        <div class="connection-overlay">
            <div class="connection-overlay-content">
                <div class="connection-spinner"></div>
                <div class="connection-text">Connection lost</div>
                <div class="connection-subtext">Waiting for server...</div>
            </div>
        </div>
    `;
}

function App() {
    const view = store.currentView.value;
    const isOpen = store.panelOpen.value;

    // Nodes and Diagnostics live in separate containers toggled by display.
    // This suppresses the left-panel width transition on view switches
    // (display:none → visible doesn't trigger CSS transitions) while
    // preserving the slide animation when the user opens/closes the panel.
    return html`
        <${Header} />
        <div class="main-container">
            <div class="left-panel ${isOpen ? 'with-sidebar' : ''}"
                 style=${{ display: view === 'nodes' ? '' : 'none' }}>
                <${NodeList} />
            </div>
            <div class="left-panel"
                 style=${{ display: view === 'diagnostics' ? '' : 'none' }}>
                <${DiagnosticsView} />
            </div>
            ${view === 'graph' && html`
                <div class="left-panel graph-container ${isOpen ? 'with-sidebar' : ''}">
                    <${GraphView} />
                </div>
            `}
            ${(view === 'nodes' || view === 'graph') && html`
                <${PanelResizer} />
                <${RightPanel} />
            `}
        </div>
        <${ConnectionOverlay} />
    `;
}

// Mount the app
render(html`<${App} />`, document.getElementById('app'));

// Expose store on window for debugging (browser console: store.nodes.value)
/** @type {any} */ (window).store = store;

console.debug('[app] play_launch Preact app initialized');
