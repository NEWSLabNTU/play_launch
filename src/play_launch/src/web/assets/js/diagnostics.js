
// ===== Diagnostics View Management =====
let currentView = 'nodes';
let diagnosticsData = [];
let diagnosticsInterval = null;
let diagnosticsSortColumn = 'level';
let diagnosticsSortDirection = 'desc';

function applySortToDiagnostics() {
    // Apply current sort without toggling direction
    // Used when refreshing data
    const levelPriority = { 'ERROR': 3, 'WARNING': 2, 'STALE': 1, 'OK': 0 };

    const sorted = [...diagnosticsData].sort((a, b) => {
        let comparison = 0;

        if (diagnosticsSortColumn === 'level') {
            comparison = levelPriority[a.level] - levelPriority[b.level];
            // Secondary sort by name when level is the same
            if (comparison === 0) {
                comparison = a.name.localeCompare(b.name);
            }
        } else if (diagnosticsSortColumn === 'timestamp') {
            comparison = new Date(a.timestamp) - new Date(b.timestamp);
        } else {
            comparison = a[diagnosticsSortColumn].localeCompare(b[diagnosticsSortColumn]);
        }

        return diagnosticsSortDirection === 'asc' ? comparison : -comparison;
    });

    renderDiagnostics(sorted);
}

function sortDiagnostics(column) {
    // Toggle direction if same column, else default to desc for level
    if (diagnosticsSortColumn === column) {
        diagnosticsSortDirection = diagnosticsSortDirection === 'asc' ? 'desc' : 'asc';
    } else {
        diagnosticsSortColumn = column;
        diagnosticsSortDirection = column === 'level' ? 'desc' : 'asc';
    }

    // Update header classes
    document.querySelectorAll('.diagnostics-table th').forEach(th => {
        th.classList.remove('sorted-asc', 'sorted-desc');
    });

    const headers = {
        'hardware_id': 0,
        'name': 1,
        'level': 2,
        'timestamp': 5
    };

    const headerIndex = headers[column];
    if (headerIndex !== undefined) {
        const th = document.querySelectorAll('.diagnostics-table th')[headerIndex];
        th.classList.add(diagnosticsSortDirection === 'asc' ? 'sorted-asc' : 'sorted-desc');
    }

    // Apply the sort
    applySortToDiagnostics();
}

function formatRelativeTime(timestamp) {
    const now = new Date();
    const date = new Date(timestamp);
    const seconds = Math.floor((now - date) / 1000);

    if (seconds < 10) return 'just now';
    if (seconds < 60) return `${seconds}s ago`;

    const minutes = Math.floor(seconds / 60);
    if (minutes < 60) return `${minutes}m ago`;

    const hours = Math.floor(minutes / 60);
    if (hours < 24) return `${hours}h ago`;

    const days = Math.floor(hours / 24);
    if (days < 7) return `${days}d ago`;

    const weeks = Math.floor(days / 7);
    return `${weeks}w ago`;
}

function switchView(view, clickedTab) {
    currentView = view;

    // Update tab active states
    document.querySelectorAll('.nav-tab').forEach(tab => {
        tab.classList.remove('active');
    });
    if (clickedTab) {
        clickedTab.classList.add('active');
    } else {
        // Fallback: find and activate the correct tab
        document.querySelectorAll('.nav-tab').forEach(tab => {
            if ((view === 'nodes' && tab.textContent.trim() === 'Nodes') ||
                (view === 'diagnostics' && tab.textContent.trim() === 'Diagnostics')) {
                tab.classList.add('active');
            }
        });
    }

    // Switch view visibility
    const nodesView = document.getElementById('nodes-view');
    const diagnosticsView = document.getElementById('diagnostics-view');

    if (view === 'nodes') {
        nodesView.style.display = 'block';
        diagnosticsView.style.display = 'none';

        // Stop diagnostic polling
        if (diagnosticsInterval) {
            clearInterval(diagnosticsInterval);
            diagnosticsInterval = null;
        }
    } else {
        nodesView.style.display = 'none';
        diagnosticsView.style.display = 'block';

        // Close right panel in diagnostics view
        closeRightPanel();

        // Start diagnostic polling
        fetchDiagnostics();
        if (!diagnosticsInterval) {
            diagnosticsInterval = setInterval(fetchDiagnostics, 5000);
        }
    }
}

function fetchDiagnostics() {
    fetch('/api/diagnostics/list')
        .then(response => response.json())
        .then(data => {
            diagnosticsData = data;
            // Apply current sort when fetching new data (without toggling)
            applySortToDiagnostics();
        })
        .catch(error => {
            console.error('Error fetching diagnostics:', error);
        });
}

function formatDiagnosticValue(value) {
    // Try to parse as JSON
    try {
        const parsed = JSON.parse(value);
        return `<pre class="diagnostic-json-value">${JSON.stringify(parsed, null, 2)}</pre>`;
    } catch (e) {
        // Not JSON, return as plain text
        return escapeHtml(value);
    }
}

function formatDiagnosticValues(values) {
    if (!values || Object.keys(values).length === 0) {
        return '-';
    }

    const entries = Object.entries(values);

    // If only one entry, show it inline
    if (entries.length === 1) {
        const [key, value] = entries[0];
        return `<div class="value-inline"><strong>${escapeHtml(key)}:</strong> ${formatDiagnosticValue(value)}</div>`;
    }

    // Multiple entries - show as structured list
    return `<div class="value-structured">${
        entries.map(([key, value]) =>
            `<div class="value-entry">
                <span class="value-key">${escapeHtml(key)}:</span>
                <span class="value-content">${formatDiagnosticValue(value)}</span>
            </div>`
        ).join('')
    }</div>`;
}

function renderDiagnostics(diagnostics) {
    const tbody = document.getElementById('diagnostics-tbody');

    if (!diagnostics || diagnostics.length === 0) {
        tbody.innerHTML = '<tr><td colspan="6" class="no-diagnostics">No diagnostics available</td></tr>';
        return;
    }

    const now = new Date();

    tbody.innerHTML = diagnostics.map(diag => {
        const levelClass = `diag-level-${diag.level.toLowerCase()}`;
        const relativeTime = formatRelativeTime(diag.timestamp);

        // Check if diagnostic is fresh (< 10 seconds old)
        const diagDate = new Date(diag.timestamp);
        const ageSeconds = Math.floor((now - diagDate) / 1000);
        const freshClass = ageSeconds < 10 ? 'fresh' : '';

        // Format key-value pairs with structure
        const valuesHtml = formatDiagnosticValues(diag.values);

        return `
            <tr class="${freshClass}">
                <td>${escapeHtml(diag.hardware_id)}</td>
                <td>${escapeHtml(diag.name)}</td>
                <td><span class="diag-level ${levelClass}">${diag.level}</span></td>
                <td>${escapeHtml(diag.message || '')}</td>
                <td class="diag-values">${valuesHtml}</td>
                <td class="diag-timestamp">${relativeTime}</td>
            </tr>
        `;
    }).join('');
}

function filterDiagnostics(query) {
    if (!query) {
        renderDiagnostics(diagnosticsData);
        return;
    }

    const filtered = diagnosticsData.filter(diag => {
        const searchStr = `${diag.hardware_id} ${diag.name} ${diag.message}`.toLowerCase();
        return searchStr.includes(query.toLowerCase());
    });

    renderDiagnostics(filtered);
}

// Fetch and update diagnostic counts
function updateDiagnosticCounts() {
    fetch('/api/diagnostics/counts')
        .then(response => response.json())
        .then(counts => {
            const badgeGroup = document.getElementById('diagnostic-badge-group');
            const badges = document.getElementById('diagnostic-badges');

            if (counts.total === 0) {
                badgeGroup.style.display = 'none';
                return;
            }

            badgeGroup.style.display = 'flex';

            let html = '';
            if (counts.ok > 0) html += `<span class="badge" style="background: var(--running-bg); color: var(--running-text);">${counts.ok} OK</span>`;
            if (counts.warning > 0) html += `<span class="badge" style="background: var(--pending-bg); color: var(--pending-text);">${counts.warning} WARN</span>`;
            if (counts.error > 0) html += `<span class="badge" style="background: var(--failed-bg); color: var(--failed-text);">${counts.error} ERR</span>`;
            if (counts.stale > 0) html += `<span class="badge" style="background: var(--blocked-bg); color: var(--blocked-text);">${counts.stale} STALE</span>`;

            badges.innerHTML = html;
        })
        .catch(error => {
            console.error('Error fetching diagnostic counts:', error);
        });
}

// Poll diagnostic counts every 5 seconds
setInterval(updateDiagnosticCounts, 5000);
updateDiagnosticCounts(); // Initial fetch
