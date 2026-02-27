// TopicsTab component — shows topics/services for the selected node (Phase 25).

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useRef, useCallback } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import { nodes, getStatusString, selectedNode, panelOpen, graphSnapshot } from '../store.js';

const html = htm.bind(h);

// Infrastructure topic/service prefixes — hidden by default
const INFRA_PREFIXES = [
    '/parameter_events', '/rosout', '/tf', '/tf_static',
    '/clock', '/diagnostics', '/_action',
];

function isInfra(name) {
    return INFRA_PREFIXES.some(p => name === p || name.startsWith(p + '/'));
}

/** QoS badge — short label with tooltip. */
function QosBadge({ label, title, variant }) {
    return html`<span class="qos-badge qos-${variant}" title=${title}>${label}</span>`;
}

/** Render QoS badges from a QosSummary object. */
function QosBadges({ qos }) {
    if (!qos) return null;
    const badges = [];

    // Reliability
    if (qos.reliability === 'reliable') {
        badges.push(html`<${QosBadge} label="REL" title="Reliable" variant="reliable" />`);
    } else if (qos.reliability === 'best_effort') {
        badges.push(html`<${QosBadge} label="BE" title="Best Effort" variant="besteffort" />`);
    }

    // Durability
    if (qos.durability === 'transient_local') {
        badges.push(html`<${QosBadge} label="TL" title="Transient Local" variant="transient" />`);
    } else if (qos.durability === 'volatile') {
        badges.push(html`<${QosBadge} label="VOL" title="Volatile" variant="volatile" />`);
    }

    // History
    if (qos.history) {
        const m = qos.history.match(/keep_last\((\d+)\)/);
        if (m) {
            badges.push(html`<${QosBadge} label="KL(${m[1]})" title="Keep Last ${m[1]}" variant="history" />`);
        } else if (qos.history === 'keep_all') {
            badges.push(html`<${QosBadge} label="KA" title="Keep All" variant="history" />`);
        }
    }

    return html`<span class="qos-badges">${badges}</span>`;
}

/** Jump to a managed node in the left panel. */
function jumpToNode(targetName) {
    selectedNode.value = targetName;
    panelOpen.value = true;
    // Scroll the node card into view
    requestAnimationFrame(() => {
        const el = document.querySelector(`[data-node="${CSS.escape(targetName)}"]`);
        if (el) el.scrollIntoView({ behavior: 'smooth', block: 'center' });
    });
}

/** A single topic row with click-to-expand. */
function TopicRow({ entry, direction }) {
    const [expanded, setExpanded] = useState(false);
    const snap = graphSnapshot.value;

    // Find connected endpoints from graph snapshot when expanded
    let connectedEndpoints = null;
    if (expanded && snap && snap.topics) {
        const topicGraph = snap.topics.find(t => t.name === entry.name);
        if (topicGraph) {
            // Show the "other side" — if this node publishes, show subscribers and vice versa
            connectedEndpoints = direction === 'pub' ? topicGraph.subscribers : topicGraph.publishers;
        }
    }

    return html`
        <div class="topic-row">
            <div class="topic-row-header" onClick=${() => setExpanded(!expanded)}>
                <span class="topic-expand">${expanded ? '\u25BC' : '\u25B6'}</span>
                <span class="topic-name" title=${entry.name}>${entry.name}</span>
                <span class="topic-type">${entry.msg_type}</span>
                <${QosBadges} qos=${entry.qos} />
                <span class="topic-counts">
                    ${entry.publisher_count}P / ${entry.subscriber_count}S
                </span>
                ${entry.dangling && html`<span class="topic-dangling" title="No ${direction === 'pub' ? 'subscribers' : 'publishers'}">!</span>`}
            </div>
            ${expanded && html`
                <div class="topic-row-detail">
                    <div class="topic-detail-label">
                        ${direction === 'pub' ? 'Subscribers' : 'Publishers'}:
                    </div>
                    ${connectedEndpoints && connectedEndpoints.length > 0
                        ? connectedEndpoints.map(ep => html`
                            <div class="topic-endpoint">
                                <span class="endpoint-fqn">${ep.fqn}</span>
                                ${ep.member_name && html`
                                    <button class="btn-jump" onClick=${(e) => { e.stopPropagation(); jumpToNode(ep.member_name); }}>
                                        Jump
                                    </button>
                                `}
                                <${QosBadges} qos=${ep.qos} />
                            </div>
                        `)
                        : html`<div class="topic-endpoint-none">None</div>`
                    }
                </div>
            `}
        </div>
    `;
}

/** A single service row (no expand). */
function ServiceRow({ entry }) {
    return html`
        <div class="topic-row">
            <div class="topic-row-header">
                <span class="topic-expand">${'\u2022'}</span>
                <span class="topic-name" title=${entry.name}>${entry.name}</span>
                <span class="topic-type">${entry.srv_type}</span>
            </div>
        </div>
    `;
}

/** Collapsible section header. */
function SectionHeader({ title, count, collapsed, onToggle }) {
    return html`
        <div class="topics-section-header" onClick=${onToggle}>
            <span class="topics-section-arrow">${collapsed ? '\u25B6' : '\u25BC'}</span>
            <span class="topics-section-title">${title}</span>
            <span class="topics-section-count">(${count})</span>
        </div>
    `;
}

/** Main TopicsTab component. */
export function TopicsTab({ nodeName }) {
    const [data, setData] = useState(null);
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState(null);
    const [showInfra, setShowInfra] = useState(false);
    const [collapsedSections, setCollapsedSections] = useState(new Set());

    const storeNode = nodes.value.get(nodeName);
    const statusStr = getStatusString(storeNode?.status);
    const isRunning = statusStr === 'running' || statusStr === 'loaded';

    // Fetch node topics when node changes or becomes running
    useEffect(() => {
        if (!nodeName || !isRunning) {
            setData(null);
            return;
        }
        setLoading(true);
        setError(null);

        fetch('/api/nodes/' + encodeURIComponent(nodeName) + '/topics')
            .then(res => {
                if (!res.ok) throw new Error(`HTTP ${res.status}`);
                return res.json();
            })
            .then(d => { setData(d); setLoading(false); })
            .catch(e => { setError(e.message); setLoading(false); });
    }, [nodeName, isRunning]);

    const toggleSection = useCallback((section) => {
        setCollapsedSections(prev => {
            const next = new Set(prev);
            if (next.has(section)) next.delete(section);
            else next.add(section);
            return next;
        });
    }, []);

    // Filter infrastructure topics
    const filterEntries = (entries) => {
        if (showInfra) return entries || [];
        return (entries || []).filter(e => !isInfra(e.name));
    };

    if (!isRunning) {
        const msg = statusStr === 'pending' || statusStr === 'loading'
            ? 'Waiting for node to start...'
            : 'Node not running';
        return html`<div class="topics-tab-message">${msg}</div>`;
    }

    if (loading) {
        return html`<div class="topics-tab-message">Loading topics...</div>`;
    }

    if (error) {
        return html`<div class="topics-tab-message topics-tab-error">Error: ${error}</div>`;
    }

    if (!data) {
        return html`<div class="topics-tab-message">No data</div>`;
    }

    const pubs = filterEntries(data.publishers);
    const subs = filterEntries(data.subscribers);
    const srvs = filterEntries(data.servers);
    const clis = filterEntries(data.clients);

    return html`
        <div class="topics-tab">
            <div class="topics-toolbar">
                <label class="topics-infra-toggle">
                    <input type="checkbox" checked=${showInfra}
                        onChange=${(e) => setShowInfra(e.target.checked)} />
                    Show infrastructure
                </label>
            </div>
            <div class="topics-sections">
                <${SectionHeader} title="PUBLISHERS" count=${pubs.length}
                    collapsed=${collapsedSections.has('pub')}
                    onToggle=${() => toggleSection('pub')} />
                ${!collapsedSections.has('pub') && pubs.map(e =>
                    html`<${TopicRow} key=${e.name} entry=${e} direction="pub" />`
                )}

                <${SectionHeader} title="SUBSCRIBERS" count=${subs.length}
                    collapsed=${collapsedSections.has('sub')}
                    onToggle=${() => toggleSection('sub')} />
                ${!collapsedSections.has('sub') && subs.map(e =>
                    html`<${TopicRow} key=${e.name} entry=${e} direction="sub" />`
                )}

                <${SectionHeader} title="SERVICE SERVERS" count=${srvs.length}
                    collapsed=${collapsedSections.has('srv')}
                    onToggle=${() => toggleSection('srv')} />
                ${!collapsedSections.has('srv') && srvs.map(e =>
                    html`<${ServiceRow} key=${e.name} entry=${e} />`
                )}

                <${SectionHeader} title="SERVICE CLIENTS" count=${clis.length}
                    collapsed=${collapsedSections.has('cli')}
                    onToggle=${() => toggleSection('cli')} />
                ${!collapsedSections.has('cli') && clis.map(e =>
                    html`<${ServiceRow} key=${e.name} entry=${e} />`
                )}
            </div>
        </div>
    `;
}
