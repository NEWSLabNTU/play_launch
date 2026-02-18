// PanelResizer component — drag to resize right panel.

import { h } from '../vendor/preact.module.js';
import { useRef, useCallback, useEffect } from '../vendor/hooks.module.js';
import htm from '../vendor/htm.module.js';
import { panelOpen } from '../store.js';

const html = htm.bind(h);

export function PanelResizer() {
    const isResizing = useRef(false);
    const resizerRef = useRef(null);

    const setWidth = useCallback((pct) => {
        pct = Math.max(30, Math.min(80, pct));
        document.documentElement.style.setProperty('--right-panel-width', pct + '%');
        localStorage.setItem('rightPanelWidth', pct);
    }, []);

    // Initialize width from localStorage
    useEffect(() => {
        const saved = parseInt(localStorage.getItem('rightPanelWidth')) || 60;
        setWidth(saved);
    }, [setWidth]);

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

    const isOpen = panelOpen.value;

    return html`
        <div class="panel-resizer ${isOpen ? 'visible' : ''}"
            ref=${resizerRef} onMouseDown=${onMouseDown} />
    `;
}
