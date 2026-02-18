// LogTab component — SSE log stream viewer for stdout/stderr.

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useRef, useCallback } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import { nodes } from '../store.js';

const html = htm.bind(h);

const MAX_LINES = 5000;

export function LogTab({ nodeName, logType, containerName }) {
    const [lines, setLines] = useState([]);
    const [status, setStatus] = useState('Not connected');
    const [connected, setConnected] = useState(false);
    const viewerRef = useRef(null);
    const autoScrollRef = useRef(true);
    const esRef = useRef(null);
    const pidRef = useRef(null);

    // Determine which node's log to stream (composable nodes use container)
    const targetNode = containerName || nodeName;

    // Track PID changes from the store to detect restarts
    const currentNode = nodes.value.get(nodeName);
    const currentPid = currentNode?.pid;

    // Connect/reconnect SSE
    useEffect(() => {
        if (!targetNode) return;

        // Close old connection
        if (esRef.current) {
            esRef.current.close();
            esRef.current = null;
        }

        setLines([]);
        setStatus('Connecting...');
        setConnected(false);

        const url = '/api/nodes/' + encodeURIComponent(targetNode) + '/logs/' + logType;
        const es = new EventSource(url);
        esRef.current = es;

        es.onopen = () => {
            if (esRef.current === es) {
                setStatus('Connected - streaming');
                setConnected(true);
            }
        };

        es.onmessage = (e) => {
            if (esRef.current !== es) return;
            setLines(prev => {
                const next = [...prev, e.data];
                if (next.length > MAX_LINES) next.splice(0, next.length - MAX_LINES);
                return next;
            });
        };

        es.onerror = () => {
            if (esRef.current === es) {
                setStatus('Connection lost - retrying...');
                setConnected(false);
            }
        };

        // Track PID at connection time
        pidRef.current = currentPid;

        return () => {
            es.close();
            if (esRef.current === es) esRef.current = null;
        };
    }, [targetNode, logType]);

    // Detect PID changes (restart) — reconnect log stream
    useEffect(() => {
        if (currentPid && pidRef.current && currentPid !== pidRef.current) {
            pidRef.current = currentPid;
            // Force reconnect by closing and re-creating
            if (esRef.current) {
                esRef.current.close();
                esRef.current = null;
            }
            setLines([]);
            setStatus('Reconnecting (PID changed)...');
            setConnected(false);

            const url = '/api/nodes/' + encodeURIComponent(targetNode) + '/logs/' + logType;
            const es = new EventSource(url);
            esRef.current = es;

            es.onopen = () => {
                if (esRef.current === es) {
                    setStatus('Connected - streaming');
                    setConnected(true);
                }
            };
            es.onmessage = (e) => {
                if (esRef.current !== es) return;
                setLines(prev => {
                    const next = [...prev, e.data];
                    if (next.length > MAX_LINES) next.splice(0, next.length - MAX_LINES);
                    return next;
                });
            };
            es.onerror = () => {
                if (esRef.current === es) {
                    setStatus('Connection lost - retrying...');
                    setConnected(false);
                }
            };
        }
    }, [currentPid, targetNode, logType]);

    // Auto-scroll
    useEffect(() => {
        if (autoScrollRef.current && viewerRef.current) {
            viewerRef.current.scrollTop = viewerRef.current.scrollHeight;
        }
    }, [lines]);

    const onScroll = useCallback(() => {
        if (viewerRef.current) {
            const el = viewerRef.current;
            autoScrollRef.current = Math.abs(el.scrollHeight - el.scrollTop - el.clientHeight) < 50;
        }
    }, []);

    const clearLog = useCallback(() => setLines([]), []);

    const scrollToBottom = useCallback(() => {
        if (viewerRef.current) {
            viewerRef.current.scrollTop = viewerRef.current.scrollHeight;
            autoScrollRef.current = true;
        }
    }, []);

    return html`
        ${containerName && html`
            <div class="log-reminder">
                Note: Showing ${logType} from container "${containerName}" (composable nodes share container logs)
            </div>
        `}
        <div class="log-viewer" ref=${viewerRef} onScroll=${onScroll}>
            ${lines.map((line, i) => html`<div class="log-line" key=${i}>${line}</div>`)}
        </div>
        <div class="log-controls">
            <span class="log-status ${connected ? 'connected' : ''}">${status}</span>
            <div>
                <button onClick=${clearLog}>Clear</button>
                <button onClick=${scrollToBottom}>${'\u2193'} Bottom</button>
            </div>
        </div>
    `;
}
