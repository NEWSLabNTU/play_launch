// LogTab component — SSE log stream viewer for stdout/stderr.

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useRef, useCallback, useMemo } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import { nodes } from '../store.js';

const html = htm.bind(h);

const MAX_LINES = 5000;

/** Matches ROS log lines: [LEVEL] [seconds.nanos] [logger]: message */
const ROS_LOG_RE = /^\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)/;

const LEVEL_PRIORITY = { 'DEBUG': 0, 'INFO': 1, 'WARN': 2, 'WARNING': 2, 'ERROR': 3, 'FATAL': 4 };
const LEVEL_OPTIONS = ['DEBUG', 'INFO', 'WARN', 'ERROR'];

/** Convert epoch seconds string to HH:MM:SS.mmm */
function formatTimestamp(epochStr) {
    const secs = parseFloat(epochStr);
    const d = new Date(secs * 1000);
    const hh = String(d.getHours()).padStart(2, '0');
    const mm = String(d.getMinutes()).padStart(2, '0');
    const ss = String(d.getSeconds()).padStart(2, '0');
    const ms = String(d.getMilliseconds()).padStart(3, '0');
    return `${hh}:${mm}:${ss}.${ms}`;
}

/** Normalize WARN/WARNING to a single CSS class name. */
function levelClass(level) {
    if (level === 'WARNING') return 'warn';
    return level.toLowerCase();
}

/** Render a formatted (colored) ROS log line. */
function FormattedLogLine({ match }) {
    const [, level, ts, logger, msg] = match;
    const cls = levelClass(level);
    const label = level === 'WARNING' ? 'WARN' : level;

    return html`
        <div class="log-line log-line-${cls}">
            <span class="log-level log-level-${cls}">${label}</span>
            <span class="log-ts">${formatTimestamp(ts)}</span>
            <span class="log-logger">[${logger}]</span>
            <span class="log-msg">${msg}</span>
        </div>
    `;
}

/** Filter lines by level and render. Non-ROS lines inherit the preceding
 *  ROS line's level for filtering purposes (default DEBUG before any ROS line). */
function filterAndRender(lines, rawMode, levelFilter) {
    // Raw mode: no parsing, no filtering
    if (rawMode) {
        return lines.map((line, i) => html`<div class="log-line" key=${i}>${line}</div>`);
    }

    const minPriority = LEVEL_PRIORITY[levelFilter] || 0;
    const result = [];
    let lastLevel = 'DEBUG';

    for (let i = 0; i < lines.length; i++) {
        const line = lines[i];
        const m = ROS_LOG_RE.exec(line);

        if (m) {
            lastLevel = m[1];
            if ((LEVEL_PRIORITY[lastLevel] || 0) >= minPriority) {
                result.push(html`<${FormattedLogLine} key=${i} match=${m} />`);
            }
        } else {
            // Non-ROS line — inherit lastLevel for filtering
            if ((LEVEL_PRIORITY[lastLevel] || 0) >= minPriority) {
                result.push(html`<div class="log-line" key=${i}>${line}</div>`);
            }
        }
    }
    return result;
}

export function LogTab({ nodeName, logType, containerName }) {
    const [lines, setLines] = useState([]);
    const [status, setStatus] = useState('Not connected');
    const [connected, setConnected] = useState(false);
    const [rawMode, setRawMode] = useState(false);
    const [levelFilter, setLevelFilter] = useState('DEBUG');
    const [following, setFollowing] = useState(true);
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
            const atBottom = Math.abs(el.scrollHeight - el.scrollTop - el.clientHeight) < 50;
            if (autoScrollRef.current !== atBottom) {
                autoScrollRef.current = atBottom;
                setFollowing(atBottom);
            }
        }
    }, []);

    const clearLog = useCallback(() => setLines([]), []);

    const scrollToBottom = useCallback(() => {
        if (viewerRef.current) {
            viewerRef.current.scrollTop = viewerRef.current.scrollHeight;
            autoScrollRef.current = true;
            setFollowing(true);
        }
    }, []);

    return html`
        ${containerName && html`
            <div class="log-reminder">
                Note: Showing ${logType} from container "${containerName}" (composable nodes share container logs)
            </div>
        `}
        <div class="log-viewer" ref=${viewerRef} onScroll=${onScroll}>
            ${useMemo(() => filterAndRender(lines, rawMode, levelFilter), [lines, rawMode, levelFilter])}
        </div>
        <div class="log-controls">
            <span class="log-status ${connected ? 'connected' : ''}">${status}</span>
            <div class="log-controls-right">
                <label class="log-checkbox">
                    <input type="checkbox" checked=${rawMode}
                        onChange=${(e) => setRawMode(e.target.checked)} /> Raw
                </label>
                ${!rawMode && html`
                    <select class="log-level-select" value=${levelFilter}
                        onChange=${(e) => setLevelFilter(e.target.value)}>
                        ${LEVEL_OPTIONS.map(l => html`<option key=${l} value=${l}>${l}</option>`)}
                    </select>
                `}
                <button onClick=${clearLog}>Clear</button>
                <button class=${following ? 'active' : ''} onClick=${scrollToBottom}>${'\u2193'} Bottom</button>
            </div>
        </div>
    `;
}
