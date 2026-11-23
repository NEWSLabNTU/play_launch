// LogTab component — SSE log stream viewer for stdout/stderr.
//
// Built for a flood (phase 105 W6): a node logging hundreds of lines a
// second is what the Health page's `log_flood` reports, and exactly when
// someone opens this tab. Lines arriving between two frames are applied in
// one update; each line is parsed once, when it arrives, and keeps its key
// (a sequence number), so dropping the oldest lines does not re-render the
// rest; and `content-visibility: auto` on each line lets the browser skip
// laying out the lines off screen. It used to copy the whole buffer (up to
// 5000 lines) for every message, key rows by index, and re-parse every
// line on every render.

import { useState, useEffect, useRef, useCallback, useMemo } from '../vendor/hooks.module.js';
import { html } from '../html.js';
import { nodes } from '../store.js';

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

/**
 * One received line, parsed once.
 * @typedef {{key: number, text: string, match: RegExpExecArray|null}} Line
 */

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
 *  ROS line's level for filtering purposes (default DEBUG before any ROS line).
 *  @param {Line[]} lines */
function filterAndRender(lines, rawMode, levelFilter) {
    // Raw mode: no parsing, no filtering
    if (rawMode) {
        return lines.map((l) => html`<div class="log-line" key=${l.key}>${l.text}</div>`);
    }

    const minPriority = LEVEL_PRIORITY[levelFilter] || 0;
    const result = [];
    let lastLevel = 'DEBUG';

    for (const l of lines) {
        if (l.match) {
            lastLevel = l.match[1];
            if ((LEVEL_PRIORITY[lastLevel] || 0) >= minPriority) {
                result.push(html`<${FormattedLogLine} key=${l.key} match=${l.match} />`);
            }
        } else if ((LEVEL_PRIORITY[lastLevel] || 0) >= minPriority) {
            // Non-ROS line — inherit lastLevel for filtering
            result.push(html`<div class="log-line" key=${l.key}>${l.text}</div>`);
        }
    }
    return result;
}

/** Run `fn` at the next frame, or soon when the tab is hidden (no frames). */
function nextFrame(fn) {
    if (typeof document !== 'undefined' && document.hidden) {
        return { cancel: clearTimeout.bind(null, setTimeout(fn, 250)) };
    }
    const id = requestAnimationFrame(fn);
    return { cancel: () => cancelAnimationFrame(id) };
}

export function LogTab({ nodeName, logType, containerName }) {
    /** @type {[Line[], (v: any) => void]} */
    const [lines, setLines] = useState([]);
    const [status, setStatus] = useState('Not connected');
    const [connected, setConnected] = useState(false);
    const [rawMode, setRawMode] = useState(false);
    const [levelFilter, setLevelFilter] = useState('DEBUG');
    const [following, setFollowing] = useState(true);
    const viewerRef = useRef(null);
    const autoScrollRef = useRef(true);

    // Determine which node's log to stream (composable nodes use container)
    const targetNode = containerName || nodeName;

    // A restart is a new process with a new file: reconnect when a new PID
    // appears. Not when the PID goes away: a stopped or crashed process's
    // last lines are the ones to read.
    const currentPid = nodes.value.get(nodeName)?.pid;
    const lastPid = useRef(currentPid);
    const [epoch, setEpoch] = useState(0);
    useEffect(() => {
        if (currentPid != null && lastPid.current != null && currentPid !== lastPid.current) {
            setEpoch((n) => n + 1);
        }
        if (currentPid != null) lastPid.current = currentPid;
    }, [currentPid]);

    useEffect(() => {
        if (!targetNode) return undefined;

        setLines([]);
        setStatus(epoch > 0 ? 'Reconnecting (PID changed)...' : 'Connecting...');
        setConnected(false);

        let seq = 0;
        /** @type {Line[]} */
        let pending = [];
        let scheduled = null;

        const flush = () => {
            scheduled = null;
            const batch = pending;
            pending = [];
            setLines((prev) => {
                const next = prev.concat(batch);
                return next.length > MAX_LINES ? next.slice(next.length - MAX_LINES) : next;
            });
        };

        const url = '/api/nodes/' + encodeURIComponent(targetNode) + '/logs/' + logType;
        const es = new EventSource(url);
        es.onopen = () => {
            setStatus('Connected - streaming');
            setConnected(true);
        };
        es.onmessage = (e) => {
            pending.push({ key: seq++, text: e.data, match: ROS_LOG_RE.exec(e.data) });
            if (pending.length > MAX_LINES) pending.splice(0, pending.length - MAX_LINES);
            if (!scheduled) scheduled = nextFrame(flush);
        };
        es.onerror = () => {
            setStatus('Connection lost - retrying...');
            setConnected(false);
        };

        return () => {
            es.close();
            if (scheduled) scheduled.cancel();
        };
    }, [targetNode, logType, epoch]);

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

    const rendered = useMemo(() => filterAndRender(lines, rawMode, levelFilter), [lines, rawMode, levelFilter]);

    return html`
        ${containerName && html`
            <div class="log-reminder">
                Note: Showing ${logType} from container "${containerName}" (composable nodes share container logs)
            </div>
        `}
        <div class="log-viewer" ref=${viewerRef} onScroll=${onScroll}>
            ${rendered}
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
                <button class=${following ? 'active' : ''} onClick=${scrollToBottom}>${'↓'} Bottom</button>
            </div>
        </div>
    `;
}
