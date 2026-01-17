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
