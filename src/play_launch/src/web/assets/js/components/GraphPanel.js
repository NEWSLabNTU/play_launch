// GraphPanel component — context-sensitive right panel for the graph view.
//
// Dispatches to element-type-specific views based on graphSelectedElement:
// - No selection → hint message
// - type 'node' → node detail with Topics/Params/stdout/stderr tabs
// - type 'namespace' → namespace members list with statuses
// - type 'edge' → topic list with types and endpoints

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useCallback, useMemo } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import {
    graphSelectedElement, panelOpen, nodes, graphSnapshot,
    getStatusString,
} from '../store.js';

/** Dispatch a jump event to navigate the graph to a specific element. */
function graphJumpTo(elementId) {
    document.dispatchEvent(new CustomEvent('graph-jump', { detail: elementId }));
}
import { TopicsTab } from './TopicsTab.js';
import { ParametersTab } from './ParametersTab.js';
import { LogTab } from './LogTab.js';

const html = htm.bind(h);

// ─── Node detail view ─────────────────────────────────────────────────────

function GraphNodeDetail({ data }) {
    const [tab, setTab] = useState('topics');
    const memberName = data.memberName;

    const storeNode = nodes.value.get(memberName);
    const status = storeNode?.status;
    const statusStr = getStatusString(status);
    const isComposable = storeNode?.node_type === 'composable_node';
    const hasOwnLogs = storeNode?.has_own_logs;
    const containerName = (isComposable && !hasOwnLogs) ? storeNode?.container_name : null;

    if (!memberName) {
        return html`<div class="graph-panel-message">
            This node is not managed by play_launch.
        </div>`;
    }

    return html`
        <div class="right-panel-header">
            <span class="right-panel-title">
                <span title=${data.fqn}>${memberName}</span>
                ${status && html`<span class="state-badge ${statusStr}">${statusStr}</span>`}
            </span>
            <button class="panel-close-btn" onClick=${() => {
                graphSelectedElement.value = null;
                panelOpen.value = false;
            }}>\u00d7</button>
        </div>
        <div class="right-panel-tabs">
            <button class="tab-btn ${tab === 'topics' ? 'active' : ''}"
                onClick=${() => setTab('topics')}>Topics</button>
            <button class="tab-btn ${tab === 'params' ? 'active' : ''}"
                onClick=${() => setTab('params')}>Params</button>
            <button class="tab-btn ${tab === 'stdout' ? 'active' : ''}"
                onClick=${() => setTab('stdout')}>stdout</button>
            <button class="tab-btn ${tab === 'stderr' ? 'active' : ''}"
                onClick=${() => setTab('stderr')}>stderr</button>
        </div>
        <div class="right-panel-content">
            <div class="tab-content" style=${{ display: tab === 'topics' ? 'flex' : 'none' }}>
                ${tab === 'topics' && html`<${TopicsTab} nodeName=${memberName} />`}
            </div>
            <div class="tab-content" style=${{ display: tab === 'params' ? 'flex' : 'none' }}>
                ${tab === 'params' && html`<${ParametersTab} nodeName=${memberName} />`}
            </div>
            <div class="tab-content" style=${{ display: tab === 'stdout' ? 'flex' : 'none' }}>
                ${tab === 'stdout' && html`<${LogTab} nodeName=${memberName} logType="stdout" containerName=${containerName} />`}
            </div>
            <div class="tab-content" style=${{ display: tab === 'stderr' ? 'flex' : 'none' }}>
                ${tab === 'stderr' && html`<${LogTab} nodeName=${memberName} logType="stderr" containerName=${containerName} />`}
            </div>
        </div>
    `;
}

// ─── Namespace detail view ────────────────────────────────────────────────

/** Collect child nodes and sub-namespaces for a given namespace. */
function getNamespaceChildren(snap, nodesMap, ns) {
    if (!snap || !snap.topics) return { childNodes: [], childNamespaces: [] };

    // Collect all unique FQNs from topics
    const fqnInfo = new Map(); // fqn → { memberName }
    for (const topic of snap.topics) {
        for (const ep of topic.publishers) {
            if (!fqnInfo.has(ep.fqn)) fqnInfo.set(ep.fqn, { memberName: ep.member_name });
            else if (ep.member_name && !fqnInfo.get(ep.fqn).memberName) {
                fqnInfo.get(ep.fqn).memberName = ep.member_name;
            }
        }
        for (const ep of topic.subscribers) {
            if (!fqnInfo.has(ep.fqn)) fqnInfo.set(ep.fqn, { memberName: ep.member_name });
            else if (ep.member_name && !fqnInfo.get(ep.fqn).memberName) {
                fqnInfo.get(ep.fqn).memberName = ep.member_name;
            }
        }
    }

    const childNodes = [];
    const subNs = new Set();

    for (const [fqn, info] of fqnInfo) {
        const lastSlash = fqn.lastIndexOf('/');
        const nodeNs = lastSlash <= 0 ? '/' : fqn.substring(0, lastSlash);

        if (nodeNs === ns) {
            // Direct child node
            const shortName = fqn.substring(lastSlash + 1);
            const storeNode = info.memberName ? nodesMap.get(info.memberName) : null;
            childNodes.push({
                fqn,
                shortName,
                memberName: info.memberName,
                status: storeNode ? getStatusString(storeNode.status) : 'unknown',
            });
        } else if (ns === '/' ? nodeNs.startsWith('/') : nodeNs.startsWith(ns + '/')) {
            // Descendant — find direct child namespace
            const rest = ns === '/' ? nodeNs : nodeNs.substring(ns.length);
            const parts = rest.split('/').filter(Boolean);
            if (parts.length > 0) {
                subNs.add(ns === '/' ? '/' + parts[0] : ns + '/' + parts[0]);
            }
        }
    }

    childNodes.sort((a, b) => a.shortName.localeCompare(b.shortName));

    const childNamespaces = Array.from(subNs).sort().map(sub => ({
        fullNs: sub,
        label: sub.split('/').filter(Boolean).pop(),
    }));

    return { childNodes, childNamespaces };
}

function GraphNsDetail({ data }) {
    const fullNs = data.fullNs;
    const snap = graphSnapshot.value;
    const nodesMap = nodes.value;

    const { childNodes, childNamespaces } = useMemo(
        () => getNamespaceChildren(snap, nodesMap, fullNs),
        [snap, nodesMap, fullNs],
    );

    const handleNodeClick = useCallback((fqn) => {
        if (!fqn) return;
        graphJumpTo('node:' + fqn);
    }, []);

    const handleNsClick = useCallback((ns) => {
        graphJumpTo('ns:' + ns);
    }, []);

    // Input/output endpoints (present when collapsed NS is focused)
    const inputEndpoints = data.inputEndpoints || [];
    const outputEndpoints = data.outputEndpoints || [];
    const hasEndpoints = inputEndpoints.length > 0 || outputEndpoints.length > 0;

    // For namespace endpoints, resolve inner nodes from topic data
    const topicNameSet = useMemo(() => {
        if (!snap || !snap.topics) return new Set();
        // Collect all topic names involving this NS's descendant nodes
        const names = new Set();
        const prefix = fullNs === '/' ? '/' : fullNs + '/';
        for (const topic of snap.topics) {
            for (const ep of [...topic.publishers, ...topic.subscribers]) {
                if (ep.fqn && (ep.fqn.startsWith(prefix) || (fullNs === '/' && ep.fqn.startsWith('/')))) {
                    names.add(topic.name);
                    break;
                }
            }
        }
        return names;
    }, [snap, fullNs]);

    const inputDetails = useMemo(() => {
        return inputEndpoints.map(ep => ({
            ...ep,
            innerNodes: ep.isNs ? resolveNsInnerNodes(snap, ep.fullNs, topicNameSet, 'pub') : [],
        }));
    }, [inputEndpoints, snap, topicNameSet]);

    const outputDetails = useMemo(() => {
        return outputEndpoints.map(ep => ({
            ...ep,
            innerNodes: ep.isNs ? resolveNsInnerNodes(snap, ep.fullNs, topicNameSet, 'sub') : [],
        }));
    }, [outputEndpoints, snap, topicNameSet]);

    return html`
        <div class="right-panel-header">
            <span class="right-panel-title">
                <span class="graph-panel-ns-path">${fullNs}</span>
                <span class="graph-panel-count">${childNodes.length} nodes</span>
            </span>
            <button class="panel-close-btn" onClick=${() => {
                graphSelectedElement.value = null;
                panelOpen.value = false;
            }}>\u00d7</button>
        </div>
        <div class="right-panel-content" style=${{ overflow: 'auto' }}>
            ${hasEndpoints && html`
                ${inputDetails.length > 0 && html`
                    <div class="graph-panel-section">
                        <div class="graph-panel-section-title">
                            <span class="endpoint-dot endpoint-dot-input"></span> Sources
                        </div>
                        ${inputDetails.map(ep => html`
                            <${EndpointRow} key=${ep.id} endpoint=${ep} />
                        `)}
                    </div>
                `}
                ${outputDetails.length > 0 && html`
                    <div class="graph-panel-section">
                        <div class="graph-panel-section-title">
                            <span class="endpoint-dot endpoint-dot-output"></span> Targets
                        </div>
                        ${outputDetails.map(ep => html`
                            <${EndpointRow} key=${ep.id} endpoint=${ep} />
                        `)}
                    </div>
                `}
            `}
            ${childNamespaces.length > 0 && html`
                <div class="graph-panel-section">
                    <div class="graph-panel-section-title">Sub-namespaces</div>
                    ${childNamespaces.map(sub => html`
                        <div key=${sub.fullNs} class="graph-panel-ns-row"
                            onClick=${() => handleNsClick(sub.fullNs)}>
                            <span class="graph-panel-ns-label">${sub.label}/</span>
                        </div>
                    `)}
                </div>
            `}
            <div class="graph-panel-section">
                <div class="graph-panel-section-title">Nodes</div>
                ${childNodes.length === 0
                    ? html`<div class="graph-panel-empty">No direct child nodes</div>`
                    : childNodes.map(n => html`
                        <div key=${n.fqn} class="graph-panel-node-row"
                            onClick=${() => handleNodeClick(n.fqn)}>
                            <span class="graph-panel-node-name">${n.shortName}</span>
                            <span class="state-badge ${n.status}">${n.status}</span>
                        </div>
                    `)
                }
            </div>
        </div>
    `;
}

// ─── Edge detail view ─────────────────────────────────────────────────────

/** Resolve inner FQNs that live inside a namespace, based on topic snapshot data. */
function resolveNsInnerNodes(snap, fullNs, topicNames, role) {
    if (!snap || !snap.topics || !fullNs) return [];
    const prefix = fullNs === '/' ? '/' : fullNs + '/';
    const fqns = new Set();
    for (const topic of snap.topics) {
        if (topicNames && !topicNames.has(topic.name)) continue;
        const endpoints = role === 'pub' ? topic.publishers : topic.subscribers;
        for (const ep of endpoints) {
            if (ep.fqn && (ep.fqn.startsWith(prefix) || (fullNs === '/' && ep.fqn.startsWith('/')))) {
                fqns.add(ep.fqn);
            }
        }
    }
    return Array.from(fqns).sort();
}

function GraphEdgeDetail({ data }) {
    const snap = graphSnapshot.value;

    // Parse topic list from edge data
    const topics = useMemo(() => {
        if (!data.topicList) return [];
        return data.topicList.split('\n').filter(Boolean).map(entry => {
            // Format: "topic_name (msg_type)"
            const match = entry.match(/^(.+?) \((.+)\)$/);
            if (match) return { name: match[1], msg_type: match[2] };
            return { name: entry, msg_type: '' };
        });
    }, [data.topicList]);

    const topicNameSet = useMemo(() => new Set(topics.map(t => t.name)), [topics]);

    // Resolve source/target labels from cy IDs
    const sourceLabel = data.source?.replace(/^(node:|ns:|(?:out|in)port:ns:)/, '') || '?';
    const targetLabel = data.target?.replace(/^(node:|ns:|(?:out|in)port:ns:)/, '') || '?';

    // Input/output endpoints from the tap handler
    const inputEndpoints = data.inputEndpoints || [];
    const outputEndpoints = data.outputEndpoints || [];

    // For namespace endpoints, resolve inner nodes
    const inputDetails = useMemo(() => {
        return inputEndpoints.map(ep => ({
            ...ep,
            innerNodes: ep.isNs ? resolveNsInnerNodes(snap, ep.fullNs, topicNameSet, 'pub') : [],
        }));
    }, [inputEndpoints, snap, topicNameSet]);

    const outputDetails = useMemo(() => {
        return outputEndpoints.map(ep => ({
            ...ep,
            innerNodes: ep.isNs ? resolveNsInnerNodes(snap, ep.fullNs, topicNameSet, 'sub') : [],
        }));
    }, [outputEndpoints, snap, topicNameSet]);

    // For each topic, look up publisher/subscriber details from snapshot
    const topicDetails = useMemo(() => {
        if (!snap || !snap.topics) return topics.map(t => ({ ...t, publishers: [], subscribers: [] }));
        return topics.map(t => {
            const snapTopic = snap.topics.find(st => st.name === t.name);
            return {
                ...t,
                publishers: snapTopic?.publishers || [],
                subscribers: snapTopic?.subscribers || [],
            };
        });
    }, [topics, snap]);

    return html`
        <div class="right-panel-header">
            <span class="right-panel-title">
                <span class="graph-panel-edge-title">
                    ${sourceLabel} <span class="graph-panel-arrow">${'\u2192'}</span> ${targetLabel}
                </span>
                <span class="graph-panel-count">${topics.length} topics</span>
            </span>
            <button class="panel-close-btn" onClick=${() => {
                graphSelectedElement.value = null;
                panelOpen.value = false;
            }}>\u00d7</button>
        </div>
        <div class="right-panel-content" style=${{ overflow: 'auto' }}>
            ${inputDetails.length > 0 && html`
                <div class="graph-panel-section">
                    <div class="graph-panel-section-title">
                        <span class="endpoint-dot endpoint-dot-input"></span> Sources
                    </div>
                    ${inputDetails.map(ep => html`
                        <${EndpointRow} key=${ep.id} endpoint=${ep} />
                    `)}
                </div>
            `}
            ${outputDetails.length > 0 && html`
                <div class="graph-panel-section">
                    <div class="graph-panel-section-title">
                        <span class="endpoint-dot endpoint-dot-output"></span> Targets
                    </div>
                    ${outputDetails.map(ep => html`
                        <${EndpointRow} key=${ep.id} endpoint=${ep} />
                    `)}
                </div>
            `}
            <div class="graph-panel-section">
                <div class="graph-panel-section-title">Topics</div>
                ${topicDetails.map(t => html`
                    <${EdgeTopicRow} key=${t.name} topic=${t} />
                `)}
            </div>
        </div>
    `;
}

function EndpointRow({ endpoint }) {
    const [expanded, setExpanded] = useState(false);
    const hasInner = endpoint.innerNodes && endpoint.innerNodes.length > 0;

    const handleJump = useCallback((e) => {
        e.stopPropagation();
        graphJumpTo(endpoint.id);
    }, [endpoint.id]);

    return html`
        <div class="graph-panel-node-row" style=${{ flexDirection: 'column', alignItems: 'stretch' }}>
            <div style=${{ display: 'flex', alignItems: 'center', gap: '6px', cursor: hasInner ? 'pointer' : 'default' }}
                onClick=${() => hasInner && setExpanded(!expanded)}>
                ${hasInner && html`<span class="topic-expand">${expanded ? '\u25BC' : '\u25B6'}</span>`}
                <span class="graph-panel-node-name" style=${{ flex: 1 }}>${endpoint.label}</span>
                ${endpoint.isNs && html`<span class="graph-panel-count">namespace</span>`}
                ${hasInner && html`<span class="graph-panel-count">${endpoint.innerNodes.length} nodes</span>`}
                <button class="btn-jump" onClick=${handleJump}>Jump</button>
            </div>
            ${expanded && hasInner && html`
                <div style=${{ paddingLeft: '20px', paddingTop: '4px' }}>
                    ${endpoint.innerNodes.map(fqn => html`
                        <div key=${fqn} class="topic-endpoint">
                            <span class="endpoint-fqn">${fqn}</span>
                        </div>
                    `)}
                </div>
            `}
        </div>
    `;
}

function EdgeTopicRow({ topic }) {
    const [expanded, setExpanded] = useState(false);

    return html`
        <div class="topic-row">
            <div class="topic-row-header" onClick=${() => setExpanded(!expanded)}>
                <span class="topic-expand">${expanded ? '\u25BC' : '\u25B6'}</span>
                <span class="topic-name" title=${topic.name}>${topic.name}</span>
                <span class="topic-type">${topic.msg_type}</span>
                <span class="topic-counts">
                    ${topic.publishers.length}P / ${topic.subscribers.length}S
                </span>
            </div>
            ${expanded && html`
                <div class="topic-row-detail">
                    <div class="topic-detail-label">Publishers:</div>
                    ${topic.publishers.length > 0
                        ? topic.publishers.map(ep => html`
                            <div class="topic-endpoint">
                                <span class="endpoint-fqn">${ep.fqn}</span>
                                ${ep.member_name && html`
                                    <span class="graph-panel-member">${ep.member_name}</span>
                                `}
                            </div>
                        `)
                        : html`<div class="topic-endpoint-none">None</div>`
                    }
                    <div class="topic-detail-label" style=${{ marginTop: '6px' }}>Subscribers:</div>
                    ${topic.subscribers.length > 0
                        ? topic.subscribers.map(ep => html`
                            <div class="topic-endpoint">
                                <span class="endpoint-fqn">${ep.fqn}</span>
                                ${ep.member_name && html`
                                    <span class="graph-panel-member">${ep.member_name}</span>
                                `}
                            </div>
                        `)
                        : html`<div class="topic-endpoint-none">None</div>`
                    }
                </div>
            `}
        </div>
    `;
}

// ─── Main GraphPanel component ────────────────────────────────────────────

export function GraphPanel() {
    const sel = graphSelectedElement.value;
    const isOpen = panelOpen.value;

    // Escape closes the panel
    useEffect(() => {
        const onKey = (e) => {
            if (e.key === 'Escape' && panelOpen.value) {
                graphSelectedElement.value = null;
                panelOpen.value = false;
            }
        };
        document.addEventListener('keydown', onKey);
        return () => document.removeEventListener('keydown', onKey);
    }, []);

    if (!isOpen) return null;

    // Determine content based on selection
    let content;
    if (!sel) {
        content = html`
            <div class="graph-panel-hint">
                <div>Click a node, namespace, or edge to inspect it.</div>
            </div>
        `;
    } else if (sel.type === 'node') {
        content = html`<${GraphNodeDetail} key=${sel.id} data=${sel.data} />`;
    } else if (sel.type === 'namespace') {
        content = html`<${GraphNsDetail} key=${sel.id} data=${sel.data} />`;
    } else if (sel.type === 'edge') {
        content = html`<${GraphEdgeDetail} key=${sel.id} data=${sel.data} />`;
    }

    return html`
        <div class="right-panel open">
            ${content}
        </div>
    `;
}
