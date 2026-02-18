// RightPanel component — node detail panel with info/stdout/stderr tabs.

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useCallback } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import { selectedNode, panelOpen, activeTab, nodes } from '../store.js';
import { InfoTab } from './InfoTab.js';
import { LogTab } from './LogTab.js';

const html = htm.bind(h);

function getStatusString(status) {
    if (!status) return 'unknown';
    if (status.type === 'Process') return status.value?.status || 'unknown';
    if (status.type === 'Composable') return status.value?.status || 'unknown';
    return 'unknown';
}

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
    const containerName = isComposable ? storeNode?.container_name : null;

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
                    onClick=${() => switchTab('info')}>Member Info</button>
                <button class="tab-btn ${tab === 'stdout' ? 'active' : ''}"
                    onClick=${() => switchTab('stdout')}>stdout</button>
                <button class="tab-btn ${tab === 'stderr' ? 'active' : ''}"
                    onClick=${() => switchTab('stderr')}>stderr</button>
            </div>
            <div class="right-panel-content">
                <div class="tab-content" style=${{ display: tab === 'info' ? 'flex' : 'none' }}>
                    <${InfoTab} nodeData=${nodeData} />
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
