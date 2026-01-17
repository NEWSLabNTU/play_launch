// Log viewer - updated for tab-based interface
function connectToLog(logType, nodeName) {
    // Close old connection for this log type
    if (eventSources[logType]) {
        eventSources[logType].close();
        eventSources[logType] = null;
    }

    // Use provided nodeName or currentNode
    const targetNode = nodeName || currentNode;
    if (!targetNode) return;

    // Clear log content to prevent duplication
    document.getElementById(`log-${logType}`).innerHTML = '';

    const url = `/api/nodes/${encodeURIComponent(targetNode)}/logs/${logType}`;
    const statusEl = document.getElementById(`log-status-${logType}`);

    statusEl.textContent = 'Connecting...';
    statusEl.classList.remove('connected');

    const newEventSource = new EventSource(url);

    newEventSource.onopen = function() {
        // Only update if this is still the current event source
        if (newEventSource === eventSources[logType]) {
            statusEl.textContent = 'Connected - streaming';
            statusEl.classList.add('connected');
        }
    };

    newEventSource.onmessage = function(event) {
        // Only append if this is still the current event source
        if (newEventSource === eventSources[logType]) {
            appendLogLine(logType, event.data);
        }
    };

    newEventSource.onerror = function() {
        // Only update if this is still the current event source
        if (newEventSource === eventSources[logType]) {
            statusEl.textContent = 'Connection lost - retrying...';
            statusEl.classList.remove('connected');
        }
    };

    eventSources[logType] = newEventSource;
}

function appendLogLine(logType, text) {
    const container = document.getElementById(`log-${logType}`);
    const line = document.createElement('div');
    line.className = 'log-line';
    line.textContent = text;
    container.appendChild(line);

    // Limit lines
    while (container.children.length > 5000) {
        container.removeChild(container.firstChild);
    }

    if (autoScroll[logType]) {
        scrollToBottom(logType);
    }
}

function clearLog(logType) {
    document.getElementById(`log-${logType}`).innerHTML = '';
}

function scrollToBottom(logType) {
    const container = document.getElementById(`log-${logType}`);
    if (container) {
        container.scrollTop = container.scrollHeight;
    }
}

// Track scroll position for both log viewers
function setupScrollTracking(logType) {
    const container = document.getElementById(`log-${logType}`);
    if (container) {
        container.addEventListener('scroll', function() {
            autoScroll[logType] = Math.abs(this.scrollHeight - this.scrollTop - this.clientHeight) < 50;
        });
    }
}

// Setup scroll tracking when page loads
setTimeout(() => {
    setupScrollTracking('stdout');
    setupScrollTracking('stderr');
}, 100);

// Periodically check for PID changes and reconnect if needed
setInterval(() => {
    if (!currentNode || !currentNodePid) return;

    fetch(`/api/nodes/${encodeURIComponent(currentNode)}`)
        .then(res => res.json())
        .then(data => {
            if (data.pid && data.pid !== currentNodePid) {
                // PID changed, node was restarted - reconnect logs
                currentNodePid = data.pid;

                // Reconnect active log viewers
                if (eventSources.stdout) {
                    connectToLog('stdout');
                }
                if (eventSources.stderr) {
                    connectToLog('stderr');
                }

                // Update info tab if it's displaying
                document.getElementById('content-details').innerHTML = renderJSON(data);
            }
        })
        .catch(err => {
            // Ignore errors (node might be stopped)
        });
}, 2000); // Check every 2 seconds

