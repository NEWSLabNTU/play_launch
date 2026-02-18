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

function App() {
    const view = store.currentView.value;
    const isOpen = store.panelOpen.value;

    return html`
        <${Header} />
        <div class="main-container">
            <div class="left-panel ${isOpen ? 'with-sidebar' : ''}">
                ${view === 'nodes' && html`<${NodeList} />`}
                <${DiagnosticsView} />
            </div>
            <${PanelResizer} />
            <${RightPanel} />
        </div>
    `;
}

// Mount the app
render(html`<${App} />`, document.getElementById('app'));

// Expose store on window for debugging (browser console: store.nodes.value)
window.store = store;

console.debug('[app] play_launch Preact app initialized');
