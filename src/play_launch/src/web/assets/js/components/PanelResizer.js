// PanelResizer component — drag to resize right panel.

import { useRef, useCallback, useEffect, useState } from '../vendor/hooks.module.js';
import { html } from '../html.js';
import { nodePanelOpen, graphPanelOpen, launchTreeSelection, currentView } from '../store.js';

export function PanelResizer() {
    const isResizing = useRef(false);
    const resizerRef = useRef(null);
    const [width, setWidthState] = useState(60);

    const setWidth = useCallback((pct) => {
        pct = Math.round(Math.max(30, Math.min(80, pct)));
        document.documentElement.style.setProperty('--right-panel-width', pct + '%');
        setWidthState(pct);
        try { localStorage.setItem('rightPanelWidth', String(pct)); } catch (_) { /* storage blocked */ }
    }, []);

    // Initialize width from localStorage
    useEffect(() => {
        let saved = 60;
        try { saved = parseInt(localStorage.getItem('rightPanelWidth') || '', 10) || 60; } catch (_) { /* storage blocked */ }
        setWidth(saved);
    }, [setWidth]);

    // Arrow keys move the divider by 5% (phase 105 W5: it was mouse-only).
    const onKeyDown = useCallback((e) => {
        if (e.key === 'ArrowLeft') { e.preventDefault(); setWidth(width + 5); }
        if (e.key === 'ArrowRight') { e.preventDefault(); setWidth(width - 5); }
    }, [width, setWidth]);

    const onMouseDown = useCallback((e) => {
        isResizing.current = true;
        if (resizerRef.current) resizerRef.current.classList.add('dragging');
        document.body.style.cursor = 'col-resize';
        document.body.style.userSelect = 'none';
        e.preventDefault();
    }, []);

    useEffect(() => {
        const onMouseMove = (e) => {
            if (!isResizing.current) return;
            const rightPx = window.innerWidth - e.clientX;
            const pct = (rightPx / window.innerWidth) * 100;
            setWidth(pct);
        };

        const onMouseUp = () => {
            if (isResizing.current) {
                isResizing.current = false;
                if (resizerRef.current) resizerRef.current.classList.remove('dragging');
                document.body.style.cursor = '';
                document.body.style.userSelect = '';
            }
        };

        document.addEventListener('mousemove', onMouseMove);
        document.addEventListener('mouseup', onMouseUp);
        return () => {
            document.removeEventListener('mousemove', onMouseMove);
            document.removeEventListener('mouseup', onMouseUp);
        };
    }, [setWidth]);

    const view = currentView.value;
    const isOpen = view === 'graph' ? graphPanelOpen.value
        : view === 'launch-tree' ? launchTreeSelection.value !== null
        : nodePanelOpen.value;

    return html`
        <div class="panel-resizer ${isOpen ? 'visible' : ''}"
            ref=${resizerRef} onMouseDown=${onMouseDown} onKeyDown=${onKeyDown}
            role="separator" aria-orientation="vertical" aria-label="Resize the side panel"
            aria-valuenow=${width} aria-valuemin="30" aria-valuemax="80"
            tabIndex=${isOpen ? 0 : -1} />
    `;
}
