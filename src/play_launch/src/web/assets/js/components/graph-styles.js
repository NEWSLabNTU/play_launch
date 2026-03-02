// graph-styles.js — Cytoscape stylesheet definition.

export function getCyStyle(isDark) {
    const bg = isDark ? '#1a1d23' : '#ffffff';
    const textMuted = isDark ? '#8a8d91' : '#6c757d';
    const borderColor = isDark ? '#3a3d45' : '#dee2e6';
    const nsBg = isDark ? '#1e2128' : '#f1f3f5';
    const nsBorder = isDark ? '#4a4d55' : '#c5c9d0';
    const edgeColor = isDark ? '#6b7280' : '#9ca3af';
    const danglingEdge = isDark ? '#fbbf24' : '#d97706';
    const defaultNode = isDark ? '#818cf8' : '#6366f1';

    return [
        // Namespace compound nodes — opaque background masks cross-namespace edges.
        // Depth-based shading: deeper namespaces are slightly different so
        // multi-level nesting is visually distinguishable.
        {
            selector: 'node.namespace',
            style: {
                'shape': 'round-rectangle',
                'background-color': function(ele) {
                    const ns = ele.data('fullNs');
                    const depth = ns === '/' ? 0 : ns.split('/').filter(Boolean).length;
                    if (isDark) {
                        // Dark: base #1e2128, lighter per level
                        const base = 30; // R/G/B base
                        const step = 5;
                        const v = Math.min(base + depth * step, 55);
                        return 'rgb(' + v + ',' + (v + 2) + ',' + (v + 5) + ')';
                    } else {
                        // Light: base #f1f3f5, darker per level
                        const base = 245;
                        const step = 6;
                        const v = Math.max(base - depth * step, 215);
                        return 'rgb(' + v + ',' + (v - 2) + ',' + (v - 4) + ')';
                    }
                },
                'background-opacity': 1,
                'border-color': nsBorder,
                'border-width': 1.5,
                'border-style': 'dashed',
                'label': function(ele) {
                    return '\u25BE ' + (ele.data('label') || '');
                },
                'text-valign': 'top',
                'text-halign': 'center',
                'text-margin-y': -6,
                'font-size': '14px',
                'font-weight': 'bold',
                'color': textMuted,
                'padding': '16px',
                'min-width': '60px',
                'min-height': '40px',
                'text-max-width': '140px',
                'text-wrap': 'ellipsis',
            },
        },
        // Collapsed namespace nodes
        {
            selector: 'node.namespace.cy-expand-collapse-collapsed-node',
            style: {
                'border-style': 'solid',
                'border-width': 2,
                'label': function(ele) {
                    const cc = ele.data('childCount');
                    const label = ele.data('label') || '';
                    return cc > 0 ? '\u25B8 ' + label + ' (' + cc + ')' : '\u25B8 ' + label;
                },
            },
        },
        // Leaf nodes (ROS nodes)
        {
            selector: 'node.leaf-node',
            style: {
                'shape': 'round-rectangle',
                'background-color': defaultNode,
                'border-color': borderColor,
                'border-width': 1,
                'label': 'data(label)',
                'text-valign': 'center',
                'text-halign': 'center',
                'font-size': '12px',
                'color': '#fff',
                'width': function(ele) {
                    const label = ele.data('label') || '';
                    return Math.max(label.length * 8 + 16, 50);
                },
                'height': 32,
                'padding': '6px',
                'text-max-width': '120px',
                'text-wrap': 'ellipsis',
            },
        },
        // Status-colored leaf nodes — darker greens for white text contrast
        {
            selector: 'node.status-running, node.status-loaded',
            style: { 'background-color': isDark ? '#15803d' : '#166534' },
        },
        {
            selector: 'node.status-stopped, node.status-unloaded',
            style: { 'background-color': isDark ? '#4b5563' : '#6b7280' },
        },
        {
            selector: 'node.status-failed',
            style: { 'background-color': isDark ? '#ef4444' : '#dc2626' },
        },
        {
            selector: 'node.status-loading, node.status-pending',
            style: { 'background-color': isDark ? '#d97706' : '#b45309' },
        },
        {
            selector: 'node.status-blocked',
            style: { 'background-color': isDark ? '#6b7280' : '#94a3b8' },
        },
        // Selected node highlight
        {
            selector: 'node.leaf-node:selected',
            style: {
                'border-color': isDark ? '#3b82f6' : '#0d6efd',
                'border-width': 3,
                'overlay-opacity': 0.1,
            },
        },
        // Namespace highlighted on click
        {
            selector: 'node.namespace.highlighted',
            style: {
                'border-color': isDark ? '#60a5fa' : '#3b82f6',
                'border-width': 3,
                'border-style': 'solid',
            },
        },
        // Port nodes (small implicit circles on NS border for edge bundling)
        {
            selector: 'node.port-node',
            style: {
                'width': 8,
                'height': 8,
                'shape': 'ellipse',
                'background-color': isDark ? '#6b7280' : '#9ca3af',
                'border-width': 0,
                'label': '',
                'opacity': 0.6,
                'min-width': '0px',
                'min-height': '0px',
                'padding': '0px',
                'events': 'no',
            },
        },
        // Spacer nodes (invisible anchors for NS border resize)
        {
            selector: 'node.spacer-node',
            style: {
                'width': 1,
                'height': 1,
                'shape': 'ellipse',
                'background-opacity': 0,
                'border-width': 0,
                'label': '',
                'opacity': 0,
                'events': 'no',
                'min-width': '0px',
                'min-height': '0px',
                'padding': '0px',
            },
        },
        // Unified edge style
        {
            selector: 'edge',
            style: {
                'curve-style': 'bezier',
                'line-color': edgeColor,
                'target-arrow-shape': 'triangle',
                'target-arrow-color': edgeColor,
                'width': function(ele) {
                    const c = ele.data('topicCount') || 1;
                    return Math.min(1.5 + Math.log2(c) * 1.2, 7);
                },
                'opacity': 0.7,
                'arrow-scale': 0.8,
                'font-size': '10px',
                'color': textMuted,
                'text-rotation': 'autorotate',
                'text-background-color': bg,
                'text-background-opacity': 0.85,
                'text-background-padding': '2px',
            },
        },
        // Branch edges: leaf/NS ↔ port (thin, no arrow, smooth bezier curves)
        {
            selector: 'edge.branch-edge',
            style: {
                'curve-style': 'bezier',
                'line-style': 'dotted',
                'width': 1,
                'opacity': 0.3,
                'target-arrow-shape': 'none',
            },
        },
        // Trunk edges: port → external (thicker, aggregated)
        {
            selector: 'edge.trunk-edge',
            style: {
                'width': function(ele) {
                    const c = ele.data('topicCount') || 1;
                    return Math.min(2 + Math.log2(c) * 1.5, 8);
                },
                'opacity': 0.8,
            },
        },
        // Edge labels on selection
        {
            selector: 'edge:selected',
            style: {
                'label': 'data(label)',
                'line-color': isDark ? '#60a5fa' : '#3b82f6',
                'target-arrow-color': isDark ? '#60a5fa' : '#3b82f6',
                'opacity': 1,
            },
        },
        // Dangling edges (amber)
        {
            selector: 'edge.dangling-edge',
            style: {
                'line-color': danglingEdge,
                'target-arrow-color': danglingEdge,
                'line-style': 'dashed',
            },
        },
        // Click-to-highlight: ego node border
        {
            selector: 'node.leaf-node.highlighted, node.namespace.highlighted',
            style: {
                'border-color': isDark ? '#60a5fa' : '#3b82f6',
                'border-width': 3,
            },
        },
        // Neighbor highlighting: input (publishers to ego) — amber
        {
            selector: 'node.neighbor-input',
            style: {
                'border-color': isDark ? '#fbbf24' : '#d97706',
                'border-width': 2,
                'border-opacity': 0.8,
            },
        },
        // Neighbor highlighting: output (subscribers from ego) — violet
        {
            selector: 'node.neighbor-output',
            style: {
                'border-color': isDark ? '#c084fc' : '#9333ea',
                'border-width': 2,
                'border-opacity': 0.8,
            },
        },
        // Neighbor highlighting: bidirectional — amber border + violet outer ring
        {
            selector: 'node.neighbor-bidir',
            style: {
                'border-color': isDark ? '#fbbf24' : '#d97706',
                'border-width': 3,
                'border-opacity': 1,
                'underlay-color': isDark ? '#c084fc' : '#9333ea',
                'underlay-padding': 6,
                'underlay-opacity': 0.55,
            },
        },
        // Click-to-highlight: highlighted edges get accent color
        {
            selector: 'edge.highlighted',
            style: {
                'line-color': isDark ? '#60a5fa' : '#3b82f6',
                'target-arrow-color': isDark ? '#60a5fa' : '#3b82f6',
                'opacity': 1,
            },
        },
        // Click-to-highlight: branch edges become visible when highlighted
        {
            selector: 'edge.branch-edge.highlighted',
            style: {
                'line-style': 'solid',
                'width': 2,
                'opacity': 0.8,
                'line-color': isDark ? '#60a5fa' : '#3b82f6',
            },
        },
        // Focus mode — faded elements
        {
            selector: '.faded',
            style: { 'opacity': 0.08, 'events': 'yes' },
        },
        // Semantic zoom — minimal (very zoomed out)
        {
            selector: 'node.leaf-node.zoom-minimal',
            style: { 'label': '', 'width': 16, 'height': 16, 'shape': 'ellipse' },
        },
        {
            selector: 'node.namespace.zoom-minimal',
            style: { 'font-size': '10px' },
        },
        {
            selector: 'edge.zoom-minimal',
            style: { 'width': 1, 'opacity': 0.3 },
        },
        // Semantic zoom — reduced
        {
            selector: 'node.leaf-node.zoom-reduced',
            style: { 'font-size': '10px' },
        },
    ];
}
