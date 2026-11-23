// Keyboard access for elements that act like buttons (phase 105 W5).
//
// The UI had no `role` or `aria-` attribute anywhere, and about 35 clickable
// rows, headers and badges that a keyboard could not reach. `clickable`
// gives such an element what a <button> has: a tab stop, Enter and Space,
// and a role. Spread it into the element: <div ...${clickable(fn)}>.

/**
 * @param {(e: Event) => void} handler what a click does
 * @param {string|null} [role] 'button' by default; null keeps the element's
 *   own role (a table row or header)
 */
export function clickable(handler, role = 'button') {
    const props = {
        tabIndex: 0,
        onClick: handler,
        onKeyDown: (e) => {
            if (e.key !== 'Enter' && e.key !== ' ') return;
            // Not for keys typed in a field inside the element.
            const t = e.target;
            if (t !== e.currentTarget && t && /^(INPUT|TEXTAREA|SELECT|BUTTON)$/.test(t.tagName)) return;
            e.preventDefault();
            // One activation: a clickable inside a clickable must not
            // activate both.
            e.stopPropagation();
            handler(e);
        },
    };
    if (role) props.role = role;
    return props;
}
