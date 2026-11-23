// Shared data hooks (phase 105 W4).
//
// Every page used to fetch in its own way: Entities and Health each carried
// a copy of probe-then-stream, Chains had the only `usePoll`, and Header,
// Health and Diagnostics hand-rolled `setInterval`. None of them paused
// while the tab was hidden. These hooks are the one way, and both pause:
// a hidden tab holds no page stream and runs no poll, and becoming visible
// again fetches at once.

import { useState, useEffect } from './vendor/hooks.module.js';
import { signal } from './vendor/signals.module.js';

/** Whether the page is visible. A hidden tab polls and streams nothing. */
export const pageVisible = signal(typeof document === 'undefined' || !document.hidden);
if (typeof document !== 'undefined') {
    document.addEventListener('visibilitychange', () => {
        pageVisible.value = !document.hidden;
    });
}

/** Read an error answer's `{error}` text, if it has one. */
async function errorText(r) {
    try {
        const j = await r.json();
        return (j && j.error) || null;
    } catch (_) {
        return null;
    }
}

/**
 * Poll `url` every `ms` while mounted and visible.
 *
 * Returns `{data, off, offText, error}`. A 404 or 409 is `off` (the run has
 * no such source, e.g. interception off), a state rather than an error, with
 * the server's `{error}` text when it gives one. `parse` turns the response
 * into data (default: JSON). A null `url` polls nothing.
 *
 * @param {string|null} url
 * @param {number} ms
 * @param {{parse?: (r: Response) => Promise<any>}} [opts]
 */
export function usePoll(url, ms, opts = {}) {
    const parse = opts.parse || ((r) => r.json());
    const [state, setState] = useState({ data: null, off: false, offText: null, error: null });
    const visible = pageVisible.value;
    useEffect(() => {
        if (!url) {
            setState({ data: null, off: false, offText: null, error: null });
            return undefined;
        }
        if (!visible) return undefined;
        let alive = true;
        const load = async () => {
            try {
                const r = await fetch(url);
                if (!alive) return;
                if (r.status === 404 || r.status === 409) {
                    const offText = await errorText(r);
                    if (alive) setState({ data: null, off: true, offText, error: null });
                    return;
                }
                if (!r.ok) {
                    setState((s) => ({ ...s, error: 'HTTP ' + r.status }));
                    return;
                }
                const data = await parse(r);
                if (alive) setState({ data, off: false, offText: null, error: null });
            } catch (e) {
                if (alive) setState((s) => ({ ...s, error: String(e) }));
            }
        };
        load();
        const id = setInterval(load, ms);
        return () => { alive = false; clearInterval(id); };
    }, [url, ms, visible]);
    return state;
}

/**
 * Fetch a snapshot once, then follow its stream, while mounted and visible.
 *
 * The plain fetch paints before the first tick and is the only place a
 * status code is visible: a 404 or 409 is `off` (with the server's text),
 * and nothing is streamed. When the stream is closed by the server, the
 * snapshot is probed again after `retryMs`. Returns
 * `{data, off, offText, disconnected}`; `data` is kept across a disconnect.
 *
 * @param {string} url the snapshot endpoint
 * @param {string} streamUrl its SSE stream
 * @param {{onData?: (data: any) => void, retryMs?: number}} [opts]
 */
export function useSnapshotStream(url, streamUrl, opts = {}) {
    const retryMs = opts.retryMs || 3000;
    const [state, setState] = useState({ data: null, off: false, offText: null, disconnected: false });
    const visible = pageVisible.value;
    const onData = opts.onData;
    useEffect(() => {
        if (!visible) return undefined;
        let live = true;
        let es = null;
        let retry = null;

        const accept = (data) => {
            setState({ data, off: false, offText: null, disconnected: false });
            if (onData) onData(data);
        };

        const open = () => {
            if (!live) return;
            es = new EventSource(streamUrl);
            es.onmessage = (e) => {
                if (e.data === 'keep-alive') return;
                try {
                    accept(JSON.parse(e.data));
                } catch (err) {
                    console.error('[stream] bad snapshot from ' + streamUrl + ':', err);
                }
            };
            es.onerror = () => {
                setState((s) => ({ ...s, disconnected: true }));
                // A non-200 answer closes the source for good instead of
                // retrying; probe again so a recovered server, or one that
                // turns out to have the source off, is reported.
                if (es && es.readyState === EventSource.CLOSED) {
                    es.close();
                    es = null;
                    retry = setTimeout(probe, retryMs);
                }
            };
        };

        const probe = async () => {
            if (!live) return;
            try {
                const r = await fetch(url);
                if (!live) return;
                if (r.status === 404 || r.status === 409) {
                    const offText = await errorText(r);
                    if (live) setState((s) => ({ ...s, off: s.data == null, offText }));
                    return;
                }
                if (r.ok) accept(await r.json());
            } catch (_) {
                setState((s) => ({ ...s, disconnected: true }));
            }
            open();
        };

        probe();
        return () => {
            live = false;
            if (retry) clearTimeout(retry);
            if (es) es.close();
        };
    }, [url, streamUrl, visible]);
    return state;
}

/**
 * Call `onEscape` when Escape is pressed anywhere and `isOpen()` says the
 * caller is showing. The three side panels each carried a copy.
 * @param {() => boolean} isOpen
 * @param {() => void} onEscape
 */
export function useEscape(isOpen, onEscape) {
    useEffect(() => {
        const onKey = (e) => {
            if (e.key === 'Escape' && isOpen()) onEscape();
        };
        document.addEventListener('keydown', onKey);
        return () => document.removeEventListener('keydown', onKey);
    }, []);
}

/**
 * The full `/api/nodes/<name>` details of a member, fetched when `name`
 * changes (5 s limit), or null. The node panel and the launch panel each
 * carried a copy.
 * @param {string|null} name
 */
export function useNodeDetails(name) {
    const [data, setData] = useState(null);
    useEffect(() => {
        if (!name) { setData(null); return undefined; }
        const controller = new AbortController();
        const timeout = setTimeout(() => controller.abort(), 5000);
        fetch('/api/nodes/' + encodeURIComponent(name), { signal: controller.signal })
            .then((res) => {
                if (!res.ok) throw new Error(`HTTP ${res.status}`);
                return res.json();
            })
            .then((d) => setData(d))
            .catch((err) => {
                if (err.name !== 'AbortError') {
                    console.warn('[panel] Failed to fetch node details for', name, err);
                }
                setData(null);
            })
            .finally(() => clearTimeout(timeout));
        return () => { controller.abort(); clearTimeout(timeout); };
    }, [name]);
    return data;
}
