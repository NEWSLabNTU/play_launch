// graph-builders.js — Convert snapshot data into Cytoscape elements.

import { isInfra, getNamespace, getShortName, namespaceLevels } from './graph-utils.js';
import { getStatusString } from '../store.js';

/**
 * Build Cytoscape node elements from a GraphSnapshot (no edges).
 * Returns { nodeElements, nodeIndex, allFqns }.
 */
export function buildCyNodes(snapshot, nodesMap, showInfra) {
    if (!snapshot || !snapshot.topics) {
        return { nodeElements: [], nodeIndex: new Map(), allFqns: new Set() };
    }

    // Collect all unique node FQNs from the graph
    const allFqns = new Set();
    for (const topic of snapshot.topics) {
        if (!showInfra && isInfra(topic.name)) continue;
        for (const ep of topic.publishers) allFqns.add(ep.fqn);
        for (const ep of topic.subscribers) allFqns.add(ep.fqn);
    }

    // Build namespace set
    const namespaces = new Set();
    const nodeIndex = new Map(); // FQN → { member_name, status, namespace, ... }
    for (const fqn of allFqns) {
        const ns = getNamespace(fqn);
        // Add all ancestor namespaces
        for (const level of namespaceLevels(ns)) {
            namespaces.add(level);
        }

        // Scan all endpoints to find member_name and count pub/sub
        let memberName = null;
        let statusStr = 'unknown';
        let pubCount = 0, subCount = 0, srvCount = 0;

        for (const topic of snapshot.topics) {
            if (!showInfra && isInfra(topic.name)) continue;
            for (const ep of topic.publishers) {
                if (ep.fqn === fqn) {
                    pubCount++;
                    if (ep.member_name && !memberName) memberName = ep.member_name;
                }
            }
            for (const ep of topic.subscribers) {
                if (ep.fqn === fqn) {
                    subCount++;
                    if (ep.member_name && !memberName) memberName = ep.member_name;
                }
            }
        }

        if (snapshot.services) {
            for (const svc of snapshot.services) {
                if (svc.servers.includes(fqn)) srvCount++;
            }
        }

        let pid = null;
        if (memberName) {
            const nodeData = nodesMap.get(memberName);
            if (nodeData) {
                statusStr = getStatusString(nodeData.status);
                pid = nodeData.pid || null;
            }
        }

        nodeIndex.set(fqn, {
            member_name: memberName, status: statusStr, namespace: ns,
            pub: pubCount, sub: subCount, srv: srvCount, pid,
        });
    }

    const nodeElements = [];

    // Create namespace compound nodes
    const sortedNs = Array.from(namespaces).sort();
    for (const ns of sortedNs) {
        const parentNs = ns === '/' ? null : getNamespace(ns) || '/';
        const label = ns === '/' ? '/' : ns.split('/').filter(Boolean).pop();
        // Count direct children (leaf nodes in this namespace)
        let childCount = 0;
        for (const [, info] of nodeIndex) {
            if (info.namespace === ns) childCount++;
        }

        nodeElements.push({
            group: 'nodes',
            data: {
                id: 'ns:' + ns,
                label: label,
                fullNs: ns,
                childCount,
                isNamespace: true,
                ...(parentNs != null && ns !== '/' ? { parent: 'ns:' + parentNs } : {}),
            },
            classes: 'namespace',
        });
    }

    // Create leaf nodes (individual ROS nodes)
    for (const [fqn, info] of nodeIndex) {
        nodeElements.push({
            group: 'nodes',
            data: {
                id: 'node:' + fqn,
                label: getShortName(fqn),
                fqn,
                memberName: info.member_name,
                statusStr: info.status,
                isNamespace: false,
                parent: 'ns:' + info.namespace,
            },
            classes: 'leaf-node status-' + info.status,
        });
    }

    return { nodeElements, nodeIndex, allFqns };
}

/**
 * Build raw edge map from a GraphSnapshot (pub→sub pairs with topic info).
 * Returns Map<"src\0tgt", { topics: Set<string>, dangling: bool }>.
 */
export function buildRawEdges(snapshot, allFqns, showInfra) {
    const edgeMap = new Map();
    if (!snapshot || !snapshot.topics) return edgeMap;

    for (const topic of snapshot.topics) {
        if (!showInfra && isInfra(topic.name)) continue;
        for (const pub of topic.publishers) {
            if (!allFqns.has(pub.fqn)) continue;
            for (const sub of topic.subscribers) {
                if (!allFqns.has(sub.fqn)) continue;
                if (pub.fqn === sub.fqn) continue; // skip self-loops
                const key = pub.fqn + '\0' + sub.fqn;
                if (!edgeMap.has(key)) {
                    edgeMap.set(key, { topics: new Set(), dangling: false });
                }
                const e = edgeMap.get(key);
                e.topics.add(topic.name + ' (' + topic.msg_type + ')');
                if (topic.dangling) e.dangling = true;
            }
        }
    }
    return edgeMap;
}
