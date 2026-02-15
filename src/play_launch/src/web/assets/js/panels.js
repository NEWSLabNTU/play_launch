// Panel resize functionality
let rightPanelWidth = parseInt(localStorage.getItem('rightPanelWidth')) || 60; // Default 60%

function setRightPanelWidth(widthPercent) {
    // Clamp between 30% and 80%
    widthPercent = Math.max(30, Math.min(80, widthPercent));
    rightPanelWidth = widthPercent;
    document.documentElement.style.setProperty('--right-panel-width', widthPercent + '%');
    localStorage.setItem('rightPanelWidth', widthPercent);
}

// Initialize right panel width
setRightPanelWidth(rightPanelWidth);

// Resizer drag functionality
const resizer = document.getElementById('panel-resizer');
let isResizing = false;

resizer.addEventListener('mousedown', (e) => {
    isResizing = true;
    resizer.classList.add('dragging');
    document.body.style.cursor = 'col-resize';
    document.body.style.userSelect = 'none';
    e.preventDefault();
});

document.addEventListener('mousemove', (e) => {
    if (!isResizing) return;

    const windowWidth = window.innerWidth;
    const rightPx = windowWidth - e.clientX;
    const widthPercent = (rightPx / windowWidth) * 100;

    setRightPanelWidth(widthPercent);
});

document.addEventListener('mouseup', () => {
    if (isResizing) {
        isResizing = false;
        resizer.classList.remove('dragging');
        document.body.style.cursor = '';
        document.body.style.userSelect = '';
    }
});

// Right panel management
let currentNode = null;
let currentTab = 'stderr';
let eventSources = { stdout: null, stderr: null };
let autoScroll = { stdout: true, stderr: true };
let currentNodePid = null; // Track PID to detect restarts

function updateSelectedCard(nodeName) {
    // Remove selected class from all cards
    document.querySelectorAll('.node-card').forEach(card => {
        card.classList.remove('selected');
    });

    // Add selected class to current card
    if (nodeName) {
        const card = document.querySelector(`.node-card[data-node="${CSS.escape(nodeName)}"]`);
        if (card) {
            card.classList.add('selected');
        }
    }
}

function openRightPanel(nodeName, nodeData) {
    currentNode = nodeName;

    document.getElementById('left-panel').classList.add('with-sidebar');
    document.getElementById('right-panel').classList.add('open');
    document.getElementById('panel-resizer').classList.add('visible');

    // Set title
    document.getElementById('right-panel-title').textContent = nodeName;

    // Update state badge if nodeData provided
    const stateBadge = document.getElementById('right-panel-state');
    if (nodeData && nodeData.status) {
        let stateClass = '';
        let stateText = '';

        if (nodeData.status.Process) {
            stateClass = nodeData.status.Process.toLowerCase();
            stateText = nodeData.status.Process;
        } else if (nodeData.status.Composable) {
            const compState = nodeData.status.Composable;
            if (typeof compState === 'object') {
                stateClass = Object.keys(compState)[0].toLowerCase();
                stateText = Object.keys(compState)[0];
            } else {
                stateClass = compState.toLowerCase();
                stateText = compState;
            }
        }

        stateBadge.className = 'state-badge ' + stateClass;
        stateBadge.textContent = stateText;
        stateBadge.style.display = '';
    } else {
        stateBadge.style.display = 'none';
    }

    // Update selected card highlight
    updateSelectedCard(nodeName);
}

function closeRightPanel() {
    document.getElementById('left-panel').classList.remove('with-sidebar');
    document.getElementById('right-panel').classList.remove('open');
    document.getElementById('panel-resizer').classList.remove('visible');

    // Close all event sources
    if (eventSources.stdout) {
        eventSources.stdout.close();
        eventSources.stdout = null;
    }
    if (eventSources.stderr) {
        eventSources.stderr.close();
        eventSources.stderr = null;
    }
    currentNode = null;
    currentNodePid = null;

    // Remove selected highlighting
    updateSelectedCard(null);
}

function switchTab(tabName) {
    currentTab = tabName;

    // Update tab buttons
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.classList.remove('active');
    });
    event.target.classList.add('active');

    // Update tab content visibility
    document.querySelectorAll('.tab-content').forEach(content => {
        content.style.display = 'none';
    });

    if (tabName === 'info') {
        document.getElementById('tab-info').style.display = 'flex';
    } else if (tabName === 'stdout') {
        document.getElementById('tab-stdout').style.display = 'flex';
        connectToLog('stdout');
    } else if (tabName === 'stderr') {
        document.getElementById('tab-stderr').style.display = 'flex';
        connectToLog('stderr');
    }
}

function showNodePanel(nodeName, preserveTab, onlyIfOpen) {
    // If onlyIfOpen is set, skip when panel is closed
    if (onlyIfOpen && !document.getElementById('right-panel').classList.contains('open')) {
        return;
    }

    // Fetch node details first
    fetch(`/api/nodes/${encodeURIComponent(nodeName)}`)
        .then(res => res.json())
        .then(data => {
            // Check if right panel is already open (to preserve tab)
            const panelAlreadyOpen = document.getElementById('right-panel').classList.contains('open');

            // Open panel with state badge
            openRightPanel(nodeName, data);

            // Update details tab
            document.getElementById('content-details').innerHTML = renderJSON(data);

            // Track PID to detect restarts
            if (data.pid) {
                currentNodePid = data.pid;
            }

            // Determine which tab to show
            // If preserveTab is explicitly passed, use currentTab
            // If panel was already open, preserve the current tab
            // Otherwise, default to stderr
            let targetTab = (preserveTab || panelAlreadyOpen) ? currentTab : 'stderr';

            // Check if this is a composable node
            const isComposableNode = data.node_type === 'ComposableNode';
            const containerName = isComposableNode ? data.container_name : null;

            // Update tab visibility
            document.querySelectorAll('.tab-btn').forEach(btn => {
                btn.classList.toggle('active', btn.textContent === targetTab);
            });
            document.querySelectorAll('.tab-content').forEach(content => {
                content.style.display = 'none';
            });

            if (targetTab === 'info') {
                document.getElementById('tab-info').style.display = 'flex';
            } else if (targetTab === 'stdout') {
                document.getElementById('tab-stdout').style.display = 'flex';
                setupLogTab('stdout', isComposableNode, containerName);
            } else if (targetTab === 'stderr') {
                document.getElementById('tab-stderr').style.display = 'flex';
                setupLogTab('stderr', isComposableNode, containerName);
            }
        })
        .catch(err => {
            document.getElementById('content-details').innerHTML =
                `<div style="color: var(--failed-text);">Error loading details: ${err.message}</div>`;
        });
}

function setupLogTab(logType, isComposableNode, containerName) {
    const logContainer = document.getElementById(`tab-${logType}`);

    // Remove any existing reminders
    const existingReminder = logContainer.querySelector('.log-reminder');
    if (existingReminder) {
        existingReminder.remove();
    }

    if (isComposableNode && containerName) {
        // Add reminder for composable nodes
        const reminder = document.createElement('div');
        reminder.className = 'log-reminder';
        reminder.textContent = `Note: Showing ${logType} from container "${containerName}" (composable nodes share container logs)`;

        const logViewer = logContainer.querySelector('.log-viewer');
        if (logViewer) {
            logContainer.insertBefore(reminder, logViewer);
        } else {
            // If no log viewer found, append to container
            logContainer.appendChild(reminder);
        }

        // Connect to container's log instead
        connectToLog(logType, containerName);
    } else {
        // Connect to node's own log
        connectToLog(logType);
    }
}

