
// HTMX event handlers
document.body.addEventListener('htmx:afterRequest', function(evt) {
    const path = evt.detail.pathInfo?.requestPath || '';

    // Extract node name from path (e.g., /api/nodes/my_node/start -> my_node)
    const nodeMatch = path.match(/\/api\/nodes\/([^/]+)\/(start|stop|restart|load|unload)/);

    if (path.includes('/start') || path.includes('/stop') || path.includes('/restart')) {
        setTimeout(() => {
            htmx.ajax('GET', '/api/nodes', {target: '#node-list'});
            htmx.ajax('GET', '/api/health', {target: '#health-badges'});
        }, 500);

        // Open right panel for the node that was controlled
        if (nodeMatch && nodeMatch[1]) {
            const nodeName = decodeURIComponent(nodeMatch[1]);
            setTimeout(() => {
                showNodePanel(nodeName, true); // preserveTab = true
            }, 600);
        }
    } else if (path.includes('/load') || path.includes('/unload')) {
        // For load/unload, only update the panel if it's already open
        if (nodeMatch && nodeMatch[1]) {
            const panelOpen = document.getElementById('right-panel').classList.contains('open');
            if (panelOpen) {
                const nodeName = decodeURIComponent(nodeMatch[1]);
                setTimeout(() => {
                    showNodePanel(nodeName, true); // preserveTab = true
                }, 100);
            }
        }
    }
});

document.body.addEventListener('htmx:afterSwap', function(evt) {
    if (evt.detail.target.id === 'node-list') {
        const searchTerm = document.getElementById('search').value;
        if (searchTerm) {
            filterNodes(searchTerm);
        }
        // Update stderr icons immediately after node list refresh
        updateStderrIcons();

        // Restore selected card highlighting if right panel is open
        if (currentNode) {
            updateSelectedCard(currentNode);
        }

        // Re-apply current sort
        const sortSelect = document.getElementById('sort-by');
        if (sortSelect && sortSelect.value !== 'default') {
            sortNodes(sortSelect.value);
        }
    }
});

// Keyboard shortcuts
document.addEventListener('keydown', function(e) {
    if (e.key === 'Escape') {
        closeRightPanel();
    }
});

// Bulk operations
function startAll() {
    if (!confirm('Start all nodes?')) return;

    fetch('/api/nodes/start-all', { method: 'POST' })
        .then(response => response.text())
        .then(() => {
            // Refresh node list
            htmx.ajax('GET', '/api/nodes', {target: '#node-list'});
            htmx.ajax('GET', '/api/health', {target: '#health-badges'});
        })
        .catch(error => {
            console.error('Error starting all:', error);
            alert('Failed to start all nodes: ' + error);
        });
}

function stopAll() {
    if (!confirm('Stop all nodes?')) return;

    fetch('/api/nodes/stop-all', { method: 'POST' })
        .then(response => response.text())
        .then(() => {
            // Refresh node list
            htmx.ajax('GET', '/api/nodes', {target: '#node-list'});
            htmx.ajax('GET', '/api/health', {target: '#health-badges'});
        })
        .catch(error => {
            console.error('Error stopping all:', error);
            alert('Failed to stop all nodes: ' + error);
        });
}

function restartAll() {
    if (!confirm('Restart all nodes?')) return;

    fetch('/api/nodes/restart-all', { method: 'POST' })
        .then(response => response.text())
        .then(() => {
            // Refresh node list
            htmx.ajax('GET', '/api/nodes', {target: '#node-list'});
            htmx.ajax('GET', '/api/health', {target: '#health-badges'});
        })
        .catch(error => {
            console.error('Error restarting all:', error);
            alert('Failed to restart all nodes: ' + error);
        });
}
