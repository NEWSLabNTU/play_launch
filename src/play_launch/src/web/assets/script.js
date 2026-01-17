        // Theme management
        function getSystemTheme() {
            return window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
        }

        function setTheme(theme) {
            document.documentElement.setAttribute('data-theme', theme);
            localStorage.setItem('theme', theme);
            updateThemeIcon();
        }

        function toggleTheme() {
            const current = document.documentElement.getAttribute('data-theme');
            const next = current === 'dark' ? 'light' : 'dark';
            setTheme(next);
        }

        function updateThemeIcon() {
            const theme = document.documentElement.getAttribute('data-theme');
            document.getElementById('theme-icon').textContent = theme === 'dark' ? '☀' : '🌙';
        }

        // Initialize theme
        const savedTheme = localStorage.getItem('theme') || getSystemTheme();
        setTheme(savedTheme);

        // Watch for system theme changes
        window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', e => {
            if (!localStorage.getItem('theme')) {
                setTheme(e.matches ? 'dark' : 'light');
            }
        });

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

        function showNodePanel(nodeName, preserveTab) {
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

        // Node filter - searches both node name and ROS name
        function filterNodes(term) {
            const cards = document.querySelectorAll('.node-card');
            const lowerTerm = term.toLowerCase();
            cards.forEach(card => {
                const name = card.getAttribute('data-node') || '';
                const rosNameElem = card.querySelector('.node-ros-name');
                const rosName = rosNameElem ? rosNameElem.textContent : '';

                // Match if term is in node name OR ROS name
                if (name.toLowerCase().includes(lowerTerm) || rosName.toLowerCase().includes(lowerTerm)) {
                    card.classList.remove('hidden');
                } else {
                    card.classList.add('hidden');
                }
            });
        }

        // Filter by namespace - populates search box and triggers filter
        function filterByNamespace(namespace) {
            const searchBox = document.getElementById('search');
            searchBox.value = namespace;
            searchBox.focus();
            filterNodes(namespace);
        }

        // Sorting functions
        function sortNodes(sortBy) {
            const nodeList = document.getElementById('node-list');

            if (sortBy === 'default') {
                // Restore original order (do nothing, let htmx refresh handle it)
                return;
            }

            // Separate parent-level cards (nodes/containers) from child-level cards (composable nodes)
            const allCards = Array.from(nodeList.querySelectorAll('.node-card'));
            const parentCards = allCards.filter(card => !card.classList.contains('child-node'));
            const childCards = allCards.filter(card => card.classList.contains('child-node'));

            // Build a map of container -> its children
            const containerChildren = new Map();
            childCards.forEach(child => {
                // Find the previous parent card (scan backwards until we find a non-child card)
                let parent = child.previousElementSibling;
                while (parent && parent.classList.contains('child-node')) {
                    parent = parent.previousElementSibling;
                }
                if (parent) {
                    const parentName = parent.getAttribute('data-node');
                    if (!containerChildren.has(parentName)) {
                        containerChildren.set(parentName, []);
                    }
                    containerChildren.get(parentName).push(child);
                }
            });

            // Sort function for cards
            const sortCards = (cardA, cardB) => {
                const nameA = cardA.getAttribute('data-node') || '';
                const nameB = cardB.getAttribute('data-node') || '';

                if (sortBy === 'name') {
                    return nameA.localeCompare(nameB);
                } else if (sortBy === 'name-desc') {
                    return nameB.localeCompare(nameA);
                } else if (sortBy === 'type') {
                    const typeA = cardA.querySelector('.node-type')?.textContent || '';
                    const typeB = cardB.querySelector('.node-type')?.textContent || '';
                    return typeA.localeCompare(typeB) || nameA.localeCompare(nameB);
                } else if (sortBy === 'status') {
                    const getStatusPriority = (card) => {
                        if (card.classList.contains('status-running') || card.classList.contains('status-loaded')) return 0;
                        if (card.classList.contains('status-loading') || card.classList.contains('status-unloading')) return 1;
                        if (card.classList.contains('status-failed')) return 2;
                        if (card.classList.contains('status-stopped') || card.classList.contains('status-unloaded')) return 3;
                        return 4;
                    };
                    const statusA = getStatusPriority(cardA);
                    const statusB = getStatusPriority(cardB);
                    return statusA - statusB || nameA.localeCompare(nameB);
                }
                return 0;
            };

            // Sort parent cards
            parentCards.sort(sortCards);

            // Sort children for each container
            containerChildren.forEach((children) => {
                children.sort(sortCards);
            });

            // Rebuild the list: parent followed by its sorted children
            parentCards.forEach(parent => {
                nodeList.appendChild(parent);
                const parentName = parent.getAttribute('data-node');
                const children = containerChildren.get(parentName);
                if (children) {
                    children.forEach(child => nodeList.appendChild(child));
                }
            });
        }

        // Toggle respawn for a node
        function toggleRespawn(nodeName, enabled) {
            fetch(`/api/nodes/${encodeURIComponent(nodeName)}/respawn/${enabled}`, {
                method: 'POST'
            })
            .then(res => {
                if (!res.ok) {
                    return res.text().then(text => {
                        throw new Error(text);
                    });
                }
                // Refresh node list after toggling respawn (wait for state propagation)
                setTimeout(() => {
                    htmx.ajax('GET', '/api/nodes', {target: '#node-list'});
                }, 1000);
            })
            .catch(err => {
                alert(`Failed to toggle respawn: ${err.message}`);
                // Revert checkbox state on error
                setTimeout(() => {
                    htmx.ajax('GET', '/api/nodes', {target: '#node-list'});
                }, 100);
            });
        }

        function toggleAutoLoad(nodeName, enabled) {
            fetch(`/api/nodes/${encodeURIComponent(nodeName)}/auto-load/${enabled}`, {
                method: 'POST'
            })
            .then(res => {
                if (!res.ok) {
                    return res.text().then(text => {
                        throw new Error(text);
                    });
                }
                // Refresh node list after toggling auto-load (wait for state propagation)
                setTimeout(() => {
                    htmx.ajax('GET', '/api/nodes', {target: '#node-list'});
                }, 1000);
            })
            .catch(err => {
                alert(`Failed to toggle auto-load: ${err.message}`);
                // Revert checkbox state on error
                setTimeout(() => {
                    htmx.ajax('GET', '/api/nodes', {target: '#node-list'});
                }, 100);
            });
        }

        // Firefox-style JSON renderer
        function renderJSON(obj, depth = 0) {
            const rows = [];

            // Filter out null/undefined/irrelevant fields
            function shouldIncludeField(key, value) {
                // Skip null and undefined values
                if (value === null || value === undefined) {
                    return false;
                }
                return true;
            }

            function renderValue(val, depth = 0) {
                if (val === null || val === undefined) {
                    return '<span class="json-value null">null</span>';
                } else if (typeof val === 'string') {
                    return `<span class="json-value string">"${escapeHtml(val)}"</span>`;
                } else if (typeof val === 'number') {
                    return `<span class="json-value number">${val}</span>`;
                } else if (typeof val === 'boolean') {
                    return `<span class="json-value boolean">${val}</span>`;
                } else if (Array.isArray(val)) {
                    if (val.length === 0) return '<span class="json-value">[]</span>';
                    // Render array items inline if simple values
                    const items = val.map(v => {
                        if (typeof v === 'string') return `"${escapeHtml(v)}"`;
                        if (typeof v === 'number' || typeof v === 'boolean') return String(v);
                        return JSON.stringify(v);
                    }).join(', ');
                    return `<span class="json-value">[${items}]</span>`;
                } else if (typeof val === 'object') {
                    const keys = Object.keys(val).filter(k => shouldIncludeField(k, val[k]));
                    if (keys.length === 0) return '<span class="json-value">{}</span>';

                    // For nested objects, render them inline if they have a single value
                    if (keys.length === 1 && depth < 2) {
                        const singleKey = keys[0];
                        const singleVal = val[singleKey];
                        if (typeof singleVal !== 'object') {
                            return `<span class="json-value">${escapeHtml(singleKey)}: ${renderValue(singleVal, depth + 1)}</span>`;
                        }
                    }

                    // Otherwise show expandable object notation
                    return `<span class="json-value">{...}</span>`;
                }
                return `<span class="json-value">${escapeHtml(String(val))}</span>`;
            }

            for (const [key, value] of Object.entries(obj)) {
                // Skip null/undefined fields
                if (!shouldIncludeField(key, value)) {
                    continue;
                }

                if (value !== null && typeof value === 'object' && !Array.isArray(value)) {
                    // Filter child entries
                    const childEntries = Object.entries(value).filter(([k, v]) => shouldIncludeField(k, v));

                    if (childEntries.length === 0) {
                        // Skip empty objects
                        continue;
                    }

                    const childRows = childEntries.map(([k, v]) => {
                        return `<div class="json-row">
                            <div class="json-key">${escapeHtml(k)}</div>
                            <div>${renderValue(v, depth + 1)}</div>
                        </div>`;
                    }).join('');

                    rows.push(`
                        <div class="json-row">
                            <div class="json-key">${escapeHtml(key)}</div>
                            <div class="json-value">{...}</div>
                        </div>
                        <div class="json-children" style="margin-left: 20px;">
                            ${childRows}
                        </div>
                    `);
                } else {
                    rows.push(`
                        <div class="json-row">
                            <div class="json-key">${escapeHtml(key)}</div>
                            <div>${renderValue(value, depth)}</div>
                        </div>
                    `);
                }
            }

            return '<div class="json-viewer">' + rows.join('') + '</div>';
        }

        function escapeHtml(text) {
            const div = document.createElement('div');
            div.textContent = text;
            return div.innerHTML;
        }

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

        // Stderr tooltip management
        let currentTooltip = null;

        function showStderrTooltip(icon, card) {
            // Get preview data from card
            const previewData = card.getAttribute('data-stderr-preview');
            if (!previewData) return;

            try {
                const lines = JSON.parse(previewData);
                if (!lines || lines.length === 0) return;

                // Remove existing tooltip if any
                hideStderrTooltip();

                // Create tooltip
                const tooltip = document.createElement('div');
                tooltip.className = 'stderr-tooltip';

                const header = document.createElement('div');
                header.className = 'stderr-tooltip-header';
                header.textContent = 'Recent stderr:';

                const content = document.createElement('div');
                content.className = 'stderr-tooltip-content';

                lines.forEach(line => {
                    const lineDiv = document.createElement('div');
                    lineDiv.className = 'stderr-tooltip-line';
                    lineDiv.textContent = line;
                    content.appendChild(lineDiv);
                });

                tooltip.appendChild(header);
                tooltip.appendChild(content);
                document.body.appendChild(tooltip);

                // Position tooltip near icon
                const iconRect = icon.getBoundingClientRect();
                const tooltipRect = tooltip.getBoundingClientRect();

                // Position below icon by default
                let top = iconRect.bottom + 8;
                let left = iconRect.left;

                // Adjust if tooltip goes off right edge
                if (left + tooltipRect.width > window.innerWidth) {
                    left = window.innerWidth - tooltipRect.width - 10;
                }

                // Adjust if tooltip goes off bottom edge
                if (top + tooltipRect.height > window.innerHeight) {
                    top = iconRect.top - tooltipRect.height - 8;
                }

                tooltip.style.top = top + 'px';
                tooltip.style.left = left + 'px';

                currentTooltip = tooltip;
            } catch (e) {
                console.error('Error parsing stderr preview:', e);
            }
        }

        function hideStderrTooltip() {
            if (currentTooltip) {
                currentTooltip.remove();
                currentTooltip = null;
            }
        }

        // Stderr icon updater
        let lastStderrSizes = new Map(); // Track sizes to detect frequent activity

        function updateStderrIcons() {
            const now = Math.floor(Date.now() / 1000); // Current time in seconds

            document.querySelectorAll('.node-card[data-stderr-mtime]').forEach(card => {
                const mtimeStr = card.getAttribute('data-stderr-mtime');
                const sizeStr = card.getAttribute('data-stderr-size');

                if (!mtimeStr) return;

                const mtime = parseInt(mtimeStr);
                const size = parseInt(sizeStr) || 0;
                const nodeName = card.getAttribute('data-node');
                const elapsed = now - mtime;

                // Find or create icon element
                let icon = card.querySelector('.stderr-icon');
                const header = card.querySelector('.node-header');

                if (!header) return;

                // Remove icon if no stderr content or > 60 seconds
                if (size === 0 || elapsed >= 60) {
                    if (icon) {
                        icon.remove();
                    }
                    lastStderrSizes.delete(nodeName);
                    return;
                }

                // Create icon if it doesn't exist
                if (!icon) {
                    icon = document.createElement('span');
                    icon.className = 'stderr-icon';
                    icon.textContent = '📋'; // Clipboard icon for logs/output
                    icon.title = 'Hover to see recent stderr';

                    // Add hover handlers to show/hide tooltip
                    icon.addEventListener('mouseenter', function(e) {
                        showStderrTooltip(e.target, card);
                    });

                    icon.addEventListener('mouseleave', function() {
                        hideStderrTooltip();
                    });

                    // Insert after PID element
                    const pid = header.querySelector('.node-pid');
                    if (pid) {
                        // Insert right after PID
                        if (pid.nextSibling) {
                            header.insertBefore(icon, pid.nextSibling);
                        } else {
                            header.appendChild(icon);
                        }
                    } else {
                        // No PID, append to end of header
                        header.appendChild(icon);
                    }
                }

                // Update icon based on time since last modification
                icon.classList.remove('hot', 'warm', 'jumping');

                // Check if node is running (only show jumping icon for running nodes)
                const isRunning = card.classList.contains('status-running') ||
                                  card.classList.contains('status-loaded');

                if (elapsed < 10 && isRunning) {
                    // 0-10 seconds AND node is running: bright yellow, jumping
                    icon.classList.add('hot', 'jumping');
                } else if (elapsed < 10) {
                    // 0-10 seconds BUT node is stopped: bright yellow, static
                    icon.classList.add('hot');
                } else {
                    // 10-60 seconds: mild orange, static
                    icon.classList.add('warm');
                }

                // Update size tracking
                lastStderrSizes.set(nodeName, size);
            });
        }

        // Run updater every second
        setInterval(updateStderrIcons, 1000);

        // Run once on page load
        updateStderrIcons();

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
                // For load/unload, the card swaps immediately, so open panel right away
                if (nodeMatch && nodeMatch[1]) {
                    const nodeName = decodeURIComponent(nodeMatch[1]);
                    setTimeout(() => {
                        showNodePanel(nodeName, true); // preserveTab = true
                    }, 100);
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
