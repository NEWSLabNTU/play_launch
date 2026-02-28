// MetricsTab component — live per-node sparkline charts (Phase 26).

import { h } from '../vendor/preact.module.js';
import { useState, useEffect, useRef, useCallback } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import { nodes, getStatusString } from '../store.js';

const html = htm.bind(h);

// --- Ring buffer ---

class RingBuffer {
    constructor(capacity) {
        this.capacity = capacity;
        this.data = [];
        this.timestamps = []; // parallel array of epoch-ms
    }
    push(val, tsMs) {
        this.data.push(val);
        this.timestamps.push(tsMs);
        if (this.data.length > this.capacity) {
            this.data.shift();
            this.timestamps.shift();
        }
    }
    toArray() { return this.data; }
    tsArray() { return this.timestamps; }
    get length() { return this.data.length; }
    last() { return this.data.length > 0 ? this.data[this.data.length - 1] : null; }
    lastTs() { return this.timestamps.length > 0 ? this.timestamps[this.timestamps.length - 1] : null; }
    clear() { this.data = []; this.timestamps = []; }

    /** Return a view trimmed to the last `windowMs` milliseconds. */
    window(windowMs) {
        if (this.data.length === 0) return { values: [], timestamps: [] };
        const cutoff = this.timestamps[this.timestamps.length - 1] - windowMs;
        let start = 0;
        while (start < this.timestamps.length && this.timestamps[start] < cutoff) start++;
        return {
            values: this.data.slice(start),
            timestamps: this.timestamps.slice(start),
        };
    }
}

// --- Chart constants ---

const CHART_HEIGHT = 80;
const RING_CAPACITY = 150; // 5 min at 2s interval

/** Minimum panel width (px) to show charts; below this only numbers are shown. */
const MIN_CHART_WIDTH = 280;

// Margins for axis labels (in CSS px)
const MARGIN = { top: 14, right: 4, bottom: 16, left: 48 };

// Time range options (value in seconds, 0 = all data)
const TIME_RANGES = [
    { label: '1m', seconds: 60 },
    { label: '2m', seconds: 120 },
    { label: '5m', seconds: 300 },
];

// --- Resolve CSS colours for canvas ---

/** Read a CSS custom property as a resolved color string. */
function cssColor(varName, fallback) {
    try {
        const val = getComputedStyle(document.documentElement).getPropertyValue(varName).trim();
        return val || fallback;
    } catch {
        return fallback;
    }
}

// --- Chart drawing ---

/** Format a relative time offset in seconds as a short label. */
function formatRelative(offsetSec) {
    const abs = Math.abs(Math.round(offsetSec));
    if (abs === 0) return 'now';
    if (abs < 60) return '-' + abs + 's';
    const m = Math.floor(abs / 60);
    const s = abs % 60;
    return s === 0 ? '-' + m + 'm' : '-' + m + 'm' + s + 's';
}

/** Draw a chart with grid, Y-axis max label, relative-time X-axis, and filled line. */
function drawChart(canvas, values, timestamps, color, opts, formatFn, windowMs) {
    if (!canvas) return;
    const dpr = window.devicePixelRatio || 1;

    const cssWidth = canvas.clientWidth || canvas.parentElement?.clientWidth || 200;
    const cssHeight = CHART_HEIGHT;

    const bufW = Math.round(cssWidth * dpr);
    const bufH = Math.round(cssHeight * dpr);
    if (canvas.width !== bufW || canvas.height !== bufH) {
        canvas.width = bufW;
        canvas.height = bufH;
    }

    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cssWidth, cssHeight);

    // Resolve theme colours once per draw
    const textColor = cssColor('--text-secondary', '#6c757d');
    const gridColor = cssColor('--border-color', '#dee2e6');

    const plotL = MARGIN.left;
    const plotR = cssWidth - MARGIN.right;
    const plotT = MARGIN.top;
    const plotB = cssHeight - MARGIN.bottom;
    const plotW = plotR - plotL;
    const plotH = plotB - plotT;

    if (plotW < 10 || plotH < 10) return;

    // --- Y range ---
    let min = opts.min != null ? opts.min : 0;
    let max = (values.length > 0) ? Math.max(...values) : 1;
    if (opts.max != null) max = Math.max(max, opts.max);
    if (max <= min) max = min + 1;
    const actualMax = max; // label value before headroom
    max = min + (max - min) * 1.1;

    // --- Grid lines (3 horizontal) ---
    ctx.strokeStyle = gridColor;
    ctx.lineWidth = 0.5;
    const gridSteps = 4;
    for (let i = 1; i < gridSteps; i++) {
        const gy = plotT + (plotH * i) / gridSteps;
        ctx.beginPath();
        ctx.moveTo(plotL, gy);
        ctx.lineTo(plotR, gy);
        ctx.stroke();
    }

    // --- Y-axis labels ---
    ctx.fillStyle = textColor;
    ctx.font = '9px sans-serif';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'top';
    ctx.fillText(formatFn(actualMax), plotL - 4, plotT);
    ctx.textBaseline = 'bottom';
    ctx.fillText(formatFn(min), plotL - 4, plotB);

    // --- X-axis: relative time labels ---
    // The rightmost point is "now" (latest timestamp); X spans windowMs to the left.
    const nowMs = (timestamps.length > 0) ? timestamps[timestamps.length - 1] : Date.now();
    const windowSec = windowMs / 1000;

    ctx.fillStyle = textColor;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';

    // Pick label interval: ensure >= 50px between labels
    const minLabelGap = 50;
    const maxLabels = Math.max(2, Math.floor(plotW / minLabelGap));
    // Choose a round interval in seconds
    const rawInterval = windowSec / maxLabels;
    const niceIntervals = [10, 15, 20, 30, 60, 120, 300];
    let interval = niceIntervals.find(n => n >= rawInterval) || windowSec;

    for (let sec = 0; sec <= windowSec; sec += interval) {
        const x = plotR - (sec / windowSec) * plotW;
        if (x < plotL) break;
        // Grid line
        ctx.strokeStyle = gridColor;
        ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.moveTo(x, plotT);
        ctx.lineTo(x, plotB);
        ctx.stroke();
        // Label
        ctx.fillStyle = textColor;
        ctx.fillText(sec === 0 ? 'now' : formatRelative(-sec), x, plotB + 2);
    }

    // --- Plot border ---
    ctx.strokeStyle = gridColor;
    ctx.lineWidth = 0.5;
    ctx.strokeRect(plotL, plotT, plotW, plotH);

    if (values.length < 2) return;

    // Map data points into plot coordinates.
    // X position is based on each point's timestamp relative to the window.
    const tMin = nowMs - windowMs;
    const tMax = nowMs;

    function xOf(tsMs) { return plotL + ((tsMs - tMin) / (tMax - tMin)) * plotW; }
    function yOf(v) { return plotB - ((v - min) / (max - min)) * plotH; }

    // --- Filled area ---
    ctx.beginPath();
    ctx.moveTo(xOf(timestamps[0]), plotB);
    for (let i = 0; i < values.length; i++) {
        ctx.lineTo(xOf(timestamps[i]), yOf(values[i]));
    }
    ctx.lineTo(xOf(timestamps[timestamps.length - 1]), plotB);
    ctx.closePath();
    ctx.fillStyle = color + '18';
    ctx.fill();

    // --- Line ---
    ctx.beginPath();
    for (let i = 0; i < values.length; i++) {
        const x = xOf(timestamps[i]);
        const y = yOf(values[i]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.stroke();
}

// --- Formatting helpers ---

function formatBytes(bytes) {
    if (bytes == null || bytes === '') return '\u2014';
    const b = Number(bytes);
    if (b < 1024) return b + ' B';
    if (b < 1024 * 1024) return (b / 1024).toFixed(1) + ' KB';
    if (b < 1024 * 1024 * 1024) return (b / (1024 * 1024)).toFixed(1) + ' MB';
    return (b / (1024 * 1024 * 1024)).toFixed(2) + ' GB';
}

function formatRate(bps) {
    if (bps == null || bps === '') return '\u2014';
    const b = Number(bps);
    if (b < 1024) return b.toFixed(0) + ' B/s';
    if (b < 1024 * 1024) return (b / 1024).toFixed(1) + ' KB/s';
    return (b / (1024 * 1024)).toFixed(2) + ' MB/s';
}

function formatPercent(v) {
    if (v == null) return '\u2014';
    return Number(v).toFixed(1) + '%';
}

// --- Metric definitions (chart metrics only) ---

const CHART_METRICS = [
    { key: 'cpu_percent', label: 'CPU', color: '#3b82f6', format: formatPercent, sparkOpts: { min: 0 } },
    { key: 'rss_bytes', label: 'RSS', color: '#8b5cf6', format: formatBytes, sparkOpts: { min: 0 } },
    { key: 'total_read_rate_bps', label: 'I/O Read', color: '#10b981', format: formatRate, sparkOpts: { min: 0 } },
    { key: 'total_write_rate_bps', label: 'I/O Write', color: '#f59e0b', format: formatRate, sparkOpts: { min: 0 } },
];

/** Keys that we also buffer but display as text, not charts. */
const NUMBER_KEYS = ['num_threads', 'num_fds'];
const ALL_BUFFER_KEYS = [...CHART_METRICS.map(m => m.key), ...NUMBER_KEYS];

// --- ChartRow sub-component ---

function ChartRow({ metric, buffer, showChart, windowMs }) {
    const canvasRef = useRef(null);
    const { values, timestamps } = buffer.window(windowMs);
    const lastVal = buffer.last();

    useEffect(() => {
        if (showChart) {
            drawChart(canvasRef.current, values, timestamps, metric.color, metric.sparkOpts, metric.format, windowMs);
        }
    }, [values.length, lastVal, showChart, windowMs]);

    return html`
        <div class="chart-row">
            <div class="chart-header">
                <span class="metric-label">${metric.label}</span>
                <span class="metric-value">${metric.format(lastVal)}</span>
            </div>
            ${showChart && html`
                <canvas ref=${canvasRef}
                        class="chart-canvas"
                        height=${CHART_HEIGHT} />
            `}
        </div>
    `;
}

// --- TimeRangeSelector ---

function TimeRangeSelector({ value, onChange }) {
    return html`
        <div class="time-range-selector">
            ${TIME_RANGES.map(r => html`
                <button key=${r.seconds}
                        class="time-range-btn ${value === r.seconds ? 'active' : ''}"
                        onClick=${() => onChange(r.seconds)}>
                    ${r.label}
                </button>
            `)}
        </div>
    `;
}

// --- Main MetricsTab ---

export function MetricsTab({ nodeName }) {
    const [colMap, setColMap] = useState(null);
    const [buffers, setBuffers] = useState(null);
    const [textMetrics, setTextMetrics] = useState({});
    const [tick, setTick] = useState(0);
    const [showChart, setShowChart] = useState(true);
    const [rangeSec, setRangeSec] = useState(120); // default 2 min
    const esRef = useRef(null);
    const containerRef = useRef(null);

    const storeNode = nodes.value.get(nodeName);
    const isComposable = storeNode?.node_type === 'composable_node';
    const hasOwnLogs = storeNode?.has_own_logs;

    const windowMs = rangeSec * 1000;

    // Observe container width to hide charts when panel is narrow
    useEffect(() => {
        const el = containerRef.current;
        if (!el) return;
        const ro = new ResizeObserver((entries) => {
            for (const entry of entries) {
                setShowChart(entry.contentRect.width >= MIN_CHART_WIDTH);
            }
        });
        ro.observe(el);
        return () => ro.disconnect();
    }, []);

    // Initialize buffers
    useEffect(() => {
        const bufs = {};
        for (const key of ALL_BUFFER_KEYS) {
            bufs[key] = new RingBuffer(RING_CAPACITY);
        }
        setBuffers(bufs);
        setColMap(null);
        setTextMetrics({});
    }, [nodeName]);

    // SSE connection
    useEffect(() => {
        if (!buffers) return;

        const es = new EventSource('/api/metrics/node/' + encodeURIComponent(nodeName));
        esRef.current = es;

        es.onmessage = (e) => {
            if (e.data === 'keep-alive' || e.data === 'Metrics file not yet created') return;

            const line = e.data;

            // Detect header row
            if (line.startsWith('timestamp,') || line.startsWith('"timestamp"')) {
                const cols = line.split(',').map(c => c.replace(/"/g, ''));
                const map = {};
                cols.forEach((col, i) => { map[col] = i; });
                setColMap(map);
                return;
            }

            // Parse data row using current colMap
            setColMap(prev => {
                if (!prev) return prev;
                const fields = line.split(',');

                // Parse ISO 8601 timestamp to epoch-ms
                const tsIdx = prev['timestamp'];
                let tsMs = null;
                if (tsIdx != null) {
                    const raw = fields[tsIdx];
                    if (raw) {
                        const parsed = Date.parse(raw);
                        if (!isNaN(parsed)) tsMs = parsed;
                    }
                }

                for (const key of ALL_BUFFER_KEYS) {
                    const idx = prev[key];
                    if (idx != null) {
                        const raw = fields[idx];
                        const val = (raw != null && raw !== '') ? Number(raw) : null;
                        if (val != null && !isNaN(val)) {
                            buffers[key].push(val, tsMs);
                        }
                    }
                }

                // Text-only metrics
                const stateIdx = prev['state'];
                const tcpIdx = prev['tcp_connections'];
                const udpIdx = prev['udp_connections'];
                setTextMetrics({
                    state: stateIdx != null ? fields[stateIdx] : null,
                    tcp: tcpIdx != null ? fields[tcpIdx] : null,
                    udp: udpIdx != null ? fields[udpIdx] : null,
                });

                setTick(t => t + 1);
                return prev;
            });
        };

        es.onerror = () => {
            // EventSource auto-reconnects
        };

        return () => {
            es.close();
            esRef.current = null;
        };
    }, [nodeName, buffers]);

    if (isComposable && !hasOwnLogs) {
        return html`
            <div class="metrics-tab" ref=${containerRef}>
                <div class="metrics-notice">
                    Metrics are collected from the parent container process.
                    Select the container to view its metrics.
                </div>
            </div>
        `;
    }

    if (!buffers) return null;

    const threadsVal = buffers['num_threads'].last();
    const fdsVal = buffers['num_fds'].last();

    return html`
        <div class="metrics-tab" ref=${containerRef}>
            <${TimeRangeSelector} value=${rangeSec} onChange=${setRangeSec} />
            ${CHART_METRICS.map(m => html`
                <${ChartRow} key=${m.key} metric=${m} buffer=${buffers[m.key]}
                             showChart=${showChart} windowMs=${windowMs} />
            `)}
            <div class="metric-text-group">
                <div class="metric-text-row">
                    <span class="metric-label">Threads</span>
                    <span class="metric-value">${threadsVal != null ? String(threadsVal) : '\u2014'}</span>
                </div>
                <div class="metric-text-row">
                    <span class="metric-label">FDs</span>
                    <span class="metric-value">${fdsVal != null ? String(fdsVal) : '\u2014'}</span>
                </div>
                <div class="metric-text-row">
                    <span class="metric-label">State</span>
                    <span class="metric-value">${textMetrics.state || '\u2014'}</span>
                </div>
                <div class="metric-text-row">
                    <span class="metric-label">TCP/UDP</span>
                    <span class="metric-value">${textMetrics.tcp || '0'} / ${textMetrics.udp || '0'}</span>
                </div>
            </div>
        </div>
    `;
}
