// Toasts (phase 105 W5): what an action reports, without blocking the page.
// A failed action used to open a blocking alert(), and a failed bulk start
// or stop said nothing at all, since it never looked at the answer.

import { h } from '../vendor/preact.module.js';
import htm from '../vendor/htm.module.js';
import { signal } from '../vendor/signals.module.js';
import { clickable } from '../a11y.js';

const html = htm.bind(h);

/** @type {{ value: {id: number, text: string, kind: string}[] }} */
export const toasts = signal([]);
let nextId = 1;

/**
 * Show a message for a few seconds (errors stay twice as long).
 * @param {string} text
 * @param {'error'|'info'} [kind]
 */
export function notify(text, kind = 'info') {
    const id = nextId++;
    toasts.value = [...toasts.value, { id, text, kind }].slice(-5);
    setTimeout(() => dismiss(id), kind === 'error' ? 10000 : 5000);
}

function dismiss(id) {
    toasts.value = toasts.value.filter((t) => t.id !== id);
}

/**
 * POST an action and report a failure, whether the request failed or the
 * server refused it. Returns whether it succeeded.
 * @param {string} url
 * @param {string} what  the action, for the message ("Start all")
 */
export async function postAction(url, what) {
    try {
        const r = await fetch(url, { method: 'POST' });
        if (!r.ok) {
            const body = (await r.text().catch(() => '')).trim();
            notify(`${what} failed: HTTP ${r.status}${body ? ' — ' + body : ''}`, 'error');
            return false;
        }
        return true;
    } catch (e) {
        notify(`${what} failed: ${e instanceof Error ? e.message : String(e)}`, 'error');
        return false;
    }
}

export function Toasts() {
    const list = toasts.value;
    if (list.length === 0) return null;
    return html`
        <div class="toasts" role="status" aria-live="polite">
            ${list.map((t) => html`
                <div key=${t.id} class="toast toast-${t.kind}" title="Dismiss"
                    ...${clickable(() => dismiss(t.id))}>${t.text}</div>
            `)}
        </div>
    `;
}
