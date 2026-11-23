// The stderr activity icon on a node card and a launch-tree row (phase 105
// W7: the two were copies). Hot for 10 s after the node last wrote to
// stderr (jumping while it runs), warm until 60 s, then gone; hovering
// shows the last lines.

import { useState, useEffect, useCallback } from '../vendor/hooks.module.js';
import { html } from '../html.js';
import { getStatusString } from '../store.js';

/** @param {{node: any}} props */
export function StderrIcon({ node }) {
    // Every hook before any return: the card's copy returned early above a
    // useCallback, so the hook count changed when a node first wrote to
    // stderr.
    const [showTooltip, setShowTooltip] = useState(false);
    const [tooltipPos, setTooltipPos] = useState({ top: 0, left: 0 });
    const [, setTick] = useState(0);

    const mtime = node?.stderr_last_modified;
    const size = node?.stderr_size || 0;

    // Re-render at the next boundary (10 s, then 60 s).
    useEffect(() => {
        if (!mtime || size === 0) return undefined;
        const elapsed = Math.floor(Date.now() / 1000) - mtime;
        if (elapsed >= 60) return undefined;
        const nextBoundary = elapsed < 10 ? 10 : 60;
        const delay = (nextBoundary - elapsed) * 1000 + 200;
        const timer = setTimeout(() => setTick((t) => t + 1), delay);
        return () => clearTimeout(timer);
    }, [mtime, size]);

    const onMouseEnter = useCallback((e) => {
        const rect = e.target.getBoundingClientRect();
        let top = rect.bottom + 8;
        let left = rect.left;
        if (left + 400 > window.innerWidth) { left = window.innerWidth - 410; }
        if (top + 200 > window.innerHeight) { top = rect.top - 208; }
        setTooltipPos({ top, left });
        setShowTooltip(true);
    }, []);
    const onMouseLeave = useCallback(() => setShowTooltip(false), []);

    if (!node || !mtime || size === 0) return null;
    const elapsed = Math.floor(Date.now() / 1000) - mtime;
    if (elapsed >= 60) return null;

    const statusStr = getStatusString(node.status);
    const isActive = statusStr === 'running' || statusStr === 'loaded';
    let iconClass = 'stderr-icon';
    if (elapsed < 10 && isActive) iconClass += ' hot jumping';
    else if (elapsed < 10) iconClass += ' hot';
    else iconClass += ' warm';

    const preview = node.stderr_preview;
    return html`
        <span class=${iconClass} title="Hover to see recent stderr"
            onMouseEnter=${onMouseEnter} onMouseLeave=${onMouseLeave}>
            ${'📋'}
        </span>
        ${showTooltip && preview && preview.length > 0 && html`
            <div class="stderr-tooltip" style=${{ position: 'fixed', top: tooltipPos.top + 'px', left: tooltipPos.left + 'px', zIndex: 10000 }}>
                <div class="stderr-tooltip-header">Recent stderr:</div>
                <div class="stderr-tooltip-content">
                    ${preview.map((line) => html`<div class="stderr-tooltip-line">${line}</div>`)}
                </div>
            </div>
        `}
    `;
}
