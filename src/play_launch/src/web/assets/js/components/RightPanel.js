// RightPanel component — node detail panel with info/stdout/stderr tabs.

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useCallback } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import { selectedNode, panelOpen, activeTab, nodes, getStatusString } from '../store.js';
import { InfoTab } from './InfoTab.js';
import { LogTab } from './LogTab.js';
import { ParametersTab } from './ParametersTab.js';
import { TopicsTab } from './TopicsTab.js';
import { MetricsTab } from './MetricsTab.js';

const html = htm.bind(h);

export function RightPanel() {
    const name = selectedNode.value;
    const isOpen = panelOpen.value;
    const tab = activeTab.value;
    const [nodeData, setNodeData] = useState(null);

    // Fetch full node details when selected node changes
    useEffect(() => {
        if (!name) {
            setNodeData(null);
            return;
        }
        fetch('/api/nodes/' + encodeURIComponent(name))
            .then(res => res.json())
            .then(data => setNodeData(data))
            .catch(err => {
                console.warn('[panel] Failed to fetch node details:', err);
                setNodeData(null);
            });
    }, [name]);

    // Close on Escape
    useEffect(() => {
        const onKey = (e) => {
            if (e.key === 'Escape' && panelOpen.value) {
                panelOpen.value = false;
            }
        };
        document.addEventListener('keydown', onKey);
        return () => document.removeEventListener('keydown', onKey);
    }, []);

    const close = useCallback(() => {
        panelOpen.value = false;
        selectedNode.value = null;
    }, []);

    const switchTab = useCallback((t) => {
        activeTab.value = t;
    }, []);

    if (!isOpen || !name) return null;

    // Get reactive status from store
    const storeNode = nodes.value.get(name);
    const status = storeNode?.status;
    const statusStr = getStatusString(status);
    const isComposable = storeNode?.node_type === 'composable_node';
    // Composable nodes with their own logs (isolated mode) stream directly;
    // others fall back to the container's log stream.
    const hasOwnLogs = storeNode?.has_own_logs;
    const containerName = (isComposable && !hasOwnLogs) ? storeNode?.container_name : null;

    return html`
        <div class="right-panel open">
            <div class="right-panel-header">
                <span class="right-panel-title">
                    <span>${name}</span>
                    ${status && html`<span class="state-badge ${statusStr}">${statusStr}</span>`}
                </span>
                <button class="close-btn" onClick=${close}>${'\u00D7'}</button>
            </div>
            <div class="right-panel-tabs">
                <button class="tab-btn ${tab === 'info' ? 'active' : ''}"
                    onClick=${() => switchTab('info')}>Info</button>
                <button class="tab-btn ${tab === 'params' ? 'active' : ''}"
                    onClick=${() => switchTab('params')}>Params</button>
                <button class="tab-btn ${tab === 'topics' ? 'active' : ''}"
                    onClick=${() => switchTab('topics')}>Topics</button>
                <button class="tab-btn ${tab === 'metrics' ? 'active' : ''}"
                    onClick=${() => switchTab('metrics')}>Metrics</button>
                <button class="tab-btn ${tab === 'stdout' ? 'active' : ''}"
                    onClick=${() => switchTab('stdout')}>stdout</button>
                <button class="tab-btn ${tab === 'stderr' ? 'active' : ''}"
                    onClick=${() => switchTab('stderr')}>stderr</button>
            </div>
            <div class="right-panel-content">
                <div class="tab-content" style=${{ display: tab === 'info' ? 'flex' : 'none' }}>
                    <${InfoTab} nodeData=${nodeData} />
                </div>
                <div class="tab-content" style=${{ display: tab === 'params' ? 'flex' : 'none' }}>
                    ${tab === 'params' && html`<${ParametersTab} nodeName=${name} />`}
                </div>
                <div class="tab-content" style=${{ display: tab === 'topics' ? 'flex' : 'none' }}>
                    ${tab === 'topics' && html`<${TopicsTab} nodeName=${name} />`}
                </div>
                <div class="tab-content" style=${{ display: tab === 'metrics' ? 'flex' : 'none' }}>
                    ${tab === 'metrics' && html`<${MetricsTab} nodeName=${name} />`}
                </div>
                <div class="tab-content" style=${{ display: tab === 'stdout' ? 'flex' : 'none' }}>
                    ${tab === 'stdout' && html`<${LogTab} nodeName=${name} logType="stdout" containerName=${containerName} />`}
                </div>
                <div class="tab-content" style=${{ display: tab === 'stderr' ? 'flex' : 'none' }}>
                    ${(tab === 'stderr' || true) && html`<${LogTab} nodeName=${name} logType="stderr" containerName=${containerName} />`}
                </div>
            </div>
        </div>
    `;
}
