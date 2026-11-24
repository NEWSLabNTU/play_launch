// RightPanel component — node detail panel with info/stdout/stderr tabs.

import { useCallback } from '../vendor/hooks.module.js';
import { html } from '../html.js';
import { selectedNode, nodePanelOpen, activeTab, nodes, getStatusString } from '../store.js';
import { InfoTab } from './InfoTab.js';
import { LogTab } from './LogTab.js';
import { ParametersTab } from './ParametersTab.js';
import { TopicsTab } from './TopicsTab.js';
import { MetricsTab } from './MetricsTab.js';
import { useEscape, useNodeDetails } from '../hooks.js';

export function RightPanel() {
    const name = selectedNode.value;
    const isOpen = nodePanelOpen.value;
    const tab = activeTab.value;
    // Full node details when the selected node changes
    const nodeData = useNodeDetails(name);

    // Close on Escape
    useEscape(() => nodePanelOpen.value, () => { nodePanelOpen.value = false; });

    const close = useCallback(() => {
        nodePanelOpen.value = false;
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
    // Log fallback routes through the container's API id (phase-50)
    const containerName = (isComposable && !hasOwnLogs)
        ? (storeNode?.container_id || storeNode?.container_name)
        : null;
    const displayName = storeNode?.name || name;

    return html`
        <div class="right-panel open">
            <div class="right-panel-header">
                <span class="right-panel-title">
                    <span title=${name}>${displayName}</span>
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
                    <${InfoTab} id=${name} nodeData=${nodeData || storeNode} />
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
                    ${tab === 'stderr' && html`<${LogTab} nodeName=${name} logType="stderr" containerName=${containerName} />`}
                </div>
            </div>
        </div>
    `;
}
