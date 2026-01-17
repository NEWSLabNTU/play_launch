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

