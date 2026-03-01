// Header component — nav tabs, health badges, diagnostic badges, theme toggle.

import { h } from '../vendor/preact.module.js';
import { useCallback } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import { currentView, theme, healthSummary, diagnostics, systemMetrics, panelOpen, graphSelectedElement } from '../store.js';

const html = htm.bind(h);

function HealthBar() {
    const hs = healthSummary.value;
    return html`
        <div class="badge-group">
            <span class="badge-label">Nodes:</span>
            <div class="health-summary">
                <span class="badge badge-nodes">${hs.nodes_running}/${hs.nodes_total} nodes</span>
                <span class="badge badge-containers">${hs.containers_running}/${hs.containers_total} containers</span>
                <span class="badge badge-composable">${hs.composable_loaded}/${hs.composable_total} composable</span>
            </div>
        </div>
    `;
}

function DiagnosticBadges() {
    const diags = diagnostics.value;
    if (!diags || diags.length === 0) return null;

    let ok = 0, warning = 0, error = 0, stale = 0;
    for (const d of diags) {
        switch (d.level) {
            case 'OK': ok++; break;
            case 'WARNING': warning++; break;
            case 'ERROR': error++; break;
            case 'STALE': stale++; break;
        }
    }

    const total = ok + warning + error + stale;
    if (total === 0) return null;

    return html`
        <div class="badge-group">
            <span class="badge-label">Diagnostics:</span>
            <div class="health-summary">
                ${ok > 0 && html`<span class="badge" style="background: var(--running-bg); color: var(--running-text);">${ok} OK</span>`}
                ${warning > 0 && html`<span class="badge" style="background: var(--pending-bg); color: var(--pending-text);">${warning} WARN</span>`}
                ${error > 0 && html`<span class="badge" style="background: var(--failed-bg); color: var(--failed-text);">${error} ERR</span>`}
                ${stale > 0 && html`<span class="badge" style="background: var(--blocked-bg); color: var(--blocked-text);">${stale} STALE</span>`}
            </div>
        </div>
    `;
}

/** Color class for a percentage value: green < 60%, yellow 60-85%, red >= 85%. */
function metricColor(pct) {
    if (pct >= 85) return 'metrics-badge-critical';
    if (pct >= 60) return 'metrics-badge-warning';
    return 'metrics-badge-ok';
}

function MetricsBadges() {
    const m = systemMetrics.value;
    if (!m) return null;

    const cpuPct = m.cpu_percent.toFixed(0);
    const memPct = m.memory_total_bytes > 0
        ? ((m.memory_used_bytes / m.memory_total_bytes) * 100).toFixed(0)
        : 0;

    return html`
        <div class="badge-group">
            <span class="badge-label">System:</span>
            <div class="health-summary">
                <span class="badge metrics-badge ${metricColor(m.cpu_percent)}">CPU ${cpuPct}%</span>
                <span class="badge metrics-badge ${metricColor(Number(memPct))}">Mem ${memPct}%</span>
                ${m.gpu_utilization_percent != null && html`
                    <span class="badge metrics-badge ${metricColor(m.gpu_utilization_percent)}">GPU ${m.gpu_utilization_percent.toFixed(0)}%</span>
                `}
            </div>
        </div>
    `;
}

function ThemeToggle() {
    const toggleTheme = useCallback(() => {
        const next = theme.value === 'dark' ? 'light' : 'dark';
        theme.value = next;
        document.documentElement.setAttribute('data-theme', next);
        localStorage.setItem('theme', next);
    }, []);

    const icon = theme.value === 'dark' ? '\u2600' : '\uD83C\uDF19';

    return html`
        <button class="theme-toggle" onClick=${toggleTheme}>
            <span>${icon}</span>
        </button>
    `;
}

export function Header() {
    const switchView = useCallback((view) => {
        // Clear graph selection when leaving graph view
        if (view !== 'graph') {
            graphSelectedElement.value = null;
        }
        currentView.value = view;
    }, []);

    const view = currentView.value;

    return html`
        <header>
            <h1>play_launch</h1>
            <div class="nav-tabs">
                <button class="nav-tab ${view === 'nodes' ? 'active' : ''}"
                    onClick=${() => switchView('nodes')}>Nodes</button>
                <button class="nav-tab ${view === 'graph' ? 'active' : ''}"
                    onClick=${() => switchView('graph')}>Graph</button>
                <button class="nav-tab ${view === 'diagnostics' ? 'active' : ''}"
                    onClick=${() => switchView('diagnostics')}>Diagnostics</button>
            </div>
            <div class="header-controls">
                <${HealthBar} />
                <${DiagnosticBadges} />
                <${MetricsBadges} />
                <${ThemeToggle} />
            </div>
        </header>
    `;
}
