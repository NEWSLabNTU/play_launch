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

