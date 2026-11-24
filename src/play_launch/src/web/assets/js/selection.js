// The Launch tree's on/off switches (phase 106 W4).
//
// A switch edits a pending set; nothing starts or stops until Apply. The
// server evaluates the pending set with the same code as `--only` and
// `--exclude` (`POST /api/selection/check`), so the summary beside the tree
// shows the containers the closure adds, the inputs the kept nodes lose and
// the warnings before anything happens. Design:
// docs/design/launch-selection.md section 6.

import { signal, computed } from './vendor/signals.module.js';
import { nodes, fetchNodes } from './store.js';
import { notify } from './components/Toasts.js';

/** Member id -> the state a switch asks for (true = on). */
export const pending = signal(new Map());

/** The server's evaluation of the pending set, or null. */
export const pendingReport = signal(null);

/** What the server says about the current selection (`GET /api/selection`). */
export const selectionInfo = signal(null);

/** Whether a member is on now (not Excluded). */
export function isOn(id) {
    const n = nodes.value.get(id);
    return !!n && !n.excluded;
}

/** The state a member's switch shows: the pending one, else the current one. */
export function effectiveOn(id) {
    const p = pending.value;
    return p.has(id) ? p.get(id) : isOn(id);
}

/** Set members' switches; an edit back to the current state is dropped. */
export function setSwitches(ids, on) {
    const next = new Map(pending.value);
    for (const id of ids) {
        if (isOn(id) === on) next.delete(id);
        else next.set(id, on);
    }
    pending.value = next;
    scheduleCheck();
}

export const pendingCount = computed(() => pending.value.size);

function body() {
    const on = [];
    const off = [];
    for (const [id, v] of pending.value) (v ? on : off).push(id);
    return { on, off };
}

let checkTimer = null;
function scheduleCheck() {
    clearTimeout(checkTimer);
    if (pending.value.size === 0) {
        pendingReport.value = null;
        return;
    }
    checkTimer = setTimeout(runCheck, 150);
}

async function post(url, payload) {
    const r = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
    });
    const text = await r.text();
    let data = null;
    try { data = JSON.parse(text); } catch (_) { /* plain text error */ }
    if (!r.ok) throw new Error((data && data.error) || text || `HTTP ${r.status}`);
    return data;
}

async function runCheck() {
    try {
        pendingReport.value = await post('/api/selection/check', body());
    } catch (e) {
        pendingReport.value = { error: e instanceof Error ? e.message : String(e) };
    }
}

/** Carry out the pending set. `mode` is 'live' or 'fresh'. */
export async function applyPending(mode) {
    try {
        const r = await post('/api/selection/apply', { ...body(), mode });
        pending.value = new Map();
        pendingReport.value = null;
        notify(`Applied (${mode}): started ${r.start.length}, stopped ${r.stop.length}`);
    } catch (e) {
        notify(`Apply failed: ${e instanceof Error ? e.message : String(e)}`, 'error');
    }
    await fetchNodes();
    await fetchSelection();
}

export function discardPending() {
    pending.value = new Map();
    pendingReport.value = null;
}

export async function fetchSelection() {
    try {
        const r = await fetch('/api/selection');
        if (r.ok) selectionInfo.value = await r.json();
    } catch (_) { /* the tree still works without it */ }
}

/** Re-read the launch files, keeping the switches (launch only). */
export async function reloadTree() {
    try {
        const r = await post('/api/selection/reload', {});
        notify(
            `Reloaded: ${r.changed.length} changed` +
            (r.added.length ? `, ${r.added.length} new (restart the launch to run them)` : '') +
            (r.removed.length ? `, ${r.removed.length} gone` : '') +
            (r.changed.length ? '; Apply restarts the changed ones' : ''),
        );
        // A changed member is restarted by the next Apply: mark it pending
        // on so Apply has something to do.
        if (r.changed.length) {
            const next = new Map(pending.value);
            for (const id of r.changed) if (isOn(id)) next.set(id, true);
            pending.value = next;
            scheduleCheck();
        }
    } catch (e) {
        notify(`Reload failed: ${e instanceof Error ? e.message : String(e)}`, 'error');
    }
    await fetchSelection();
}

/** Copy the launch command for what is on now. */
export async function copyCommand() {
    await fetchSelection();
    const cmd = selectionInfo.value?.command;
    if (!cmd) {
        notify('No command to copy', 'error');
        return;
    }
    try {
        await navigator.clipboard.writeText(cmd);
        notify('Command copied');
    } catch (_) {
        window.prompt('Copy the command:', cmd);
    }
}

/** Whether the Export panel is open, and the last export's answer. */
export const exportOpen = signal(false);
export const exportResult = signal(null);

/**
 * Export what is on as a launch file, into the run's log directory.
 * @param {'include'|'self-contained'|'command'} form
 * @param {string} name
 */
export async function exportSelection(form, name) {
    exportResult.value = { busy: true };
    try {
        exportResult.value = await post('/api/selection/export', { form, name: name || undefined });
    } catch (e) {
        exportResult.value = { error: e instanceof Error ? e.message : String(e) };
    }
}
