#!/usr/bin/env python3
"""
Interactive resource usage charts for play_launch logs.

Generates individual HTML files with interactive visualizations including:
- CPU and memory usage (timeline and distribution sorted by average)
- GPU usage and distribution (when available)
- I/O rates (read/write timeline)
- Network statistics
- Comprehensive statistics report

Each chart is in a separate file for full-screen viewing without legend clutter.
Process names are shown in hover tooltips.
"""

import argparse
import csv
import json
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import sys

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    import pandas as pd
except ImportError:
    print(
        "Error: plotly and pandas are required. Install with: pip install plotly pandas"
    )
    sys.exit(1)


def find_latest_log_dir(base_log_dir: Path) -> Path:
    """Find the most recent timestamped log directory."""
    log_dirs = [d for d in base_log_dir.iterdir() if d.is_dir() and d.name[0].isdigit()]
    if not log_dirs:
        raise FileNotFoundError(
            f"No timestamped log directories found in {base_log_dir}\n"
            f"Expected directories like: YYYY-MM-DD_HH-MM-SS\n"
            f"Tip: Use --log-dir to specify a specific log directory"
        )

    latest = sorted(log_dirs, key=lambda d: d.name)[-1]
    return latest


def parse_csv_file(csv_path: Path) -> Optional[Dict]:
    """
    Parse a CSV file and extract all metrics.

    Returns:
        Dict with metric arrays, or None if file is empty
    """
    timestamps = []
    cpu_percents = []
    rss_bytes = []
    gpu_mem_bytes = []
    gpu_util_percents = []
    gpu_mem_util_percents = []
    gpu_temps = []
    gpu_powers = []
    gpu_graphics_clocks = []
    gpu_memory_clocks = []
    io_read_rates = []
    io_write_rates = []
    tcp_conns = []
    udp_conns = []
    num_threads_list = []
    io_syscr_list = []
    io_syscw_list = []
    io_storage_read_list = []
    io_storage_write_list = []
    io_cancelled_write_list = []

    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            ts = datetime.fromisoformat(row["timestamp"].replace("Z", "+00:00"))
            timestamps.append(ts)
            cpu_percents.append(float(row["cpu_percent"]))
            rss_bytes.append(int(row["rss_bytes"]))

            # Parse optional GPU fields
            gpu_mem_str = row.get("gpu_memory_bytes", "").strip()
            gpu_mem_bytes.append(int(gpu_mem_str) if gpu_mem_str else None)

            gpu_util_str = row.get("gpu_utilization_percent", "").strip()
            gpu_util_percents.append(int(gpu_util_str) if gpu_util_str else None)

            gpu_mem_util_str = row.get("gpu_memory_utilization_percent", "").strip()
            gpu_mem_util_percents.append(int(gpu_mem_util_str) if gpu_mem_util_str else None)

            gpu_temp_str = row.get("gpu_temperature_celsius", "").strip()
            gpu_temps.append(int(gpu_temp_str) if gpu_temp_str else None)

            gpu_power_str = row.get("gpu_power_milliwatts", "").strip()
            gpu_powers.append(int(gpu_power_str) if gpu_power_str else None)

            gpu_graphics_clock_str = row.get("gpu_graphics_clock_mhz", "").strip()
            gpu_graphics_clocks.append(int(gpu_graphics_clock_str) if gpu_graphics_clock_str else None)

            gpu_memory_clock_str = row.get("gpu_memory_clock_mhz", "").strip()
            gpu_memory_clocks.append(int(gpu_memory_clock_str) if gpu_memory_clock_str else None)

            # Parse I/O rate fields
            read_rate_str = row.get("total_read_rate_bps", "").strip()
            io_read_rates.append(float(read_rate_str) if read_rate_str else None)

            write_rate_str = row.get("total_write_rate_bps", "").strip()
            io_write_rates.append(float(write_rate_str) if write_rate_str else None)

            # Parse network connection fields
            tcp_str = row.get("tcp_connections", "").strip()
            tcp_conns.append(int(tcp_str) if tcp_str else None)

            udp_str = row.get("udp_connections", "").strip()
            udp_conns.append(int(udp_str) if udp_str else None)

            # Parse thread count
            threads_str = row.get("num_threads", "").strip()
            num_threads_list.append(int(threads_str) if threads_str else None)

            # Parse I/O syscall counts
            syscr_str = row.get("io_syscr", "").strip()
            io_syscr_list.append(int(syscr_str) if syscr_str else None)

            syscw_str = row.get("io_syscw", "").strip()
            io_syscw_list.append(int(syscw_str) if syscw_str else None)

            # Parse storage I/O bytes (cache-excluded)
            storage_read_str = row.get("io_storage_read_bytes", "").strip()
            io_storage_read_list.append(int(storage_read_str) if storage_read_str else None)

            storage_write_str = row.get("io_storage_write_bytes", "").strip()
            io_storage_write_list.append(int(storage_write_str) if storage_write_str else None)

            # Parse cancelled write bytes
            cancelled_str = row.get("io_cancelled_write_bytes", "").strip()
            io_cancelled_write_list.append(int(cancelled_str) if cancelled_str else None)

    if not timestamps:
        return None

    # Convert timestamps to relative seconds from start
    start_time = timestamps[0]
    times = [(ts - start_time).total_seconds() for ts in timestamps]

    return {
        "timestamps": timestamps,
        "times": times,
        "cpu": cpu_percents,
        "rss_mb": [rss / (1024**2) for rss in rss_bytes],
        "gpu_mem_mb": [gm / (1024**2) if gm is not None else None for gm in gpu_mem_bytes],
        "gpu_util": gpu_util_percents,
        "gpu_mem_util": gpu_mem_util_percents,
        "gpu_temp": gpu_temps,
        "gpu_power_w": [gp / 1000 if gp is not None else None for gp in gpu_powers],
        "gpu_graphics_clock": gpu_graphics_clocks,
        "gpu_memory_clock": gpu_memory_clocks,
        "io_read_mbps": [r / (1024**2) if r is not None else None for r in io_read_rates],
        "io_write_mbps": [w / (1024**2) if w is not None else None for w in io_write_rates],
        "tcp": tcp_conns,
        "udp": udp_conns,
        "num_threads": num_threads_list,
        "io_syscr": io_syscr_list,
        "io_syscw": io_syscw_list,
        "io_storage_read_mb": [sr / (1024**2) if sr is not None else None for sr in io_storage_read_list],
        "io_storage_write_mb": [sw / (1024**2) if sw is not None else None for sw in io_storage_write_list],
        "io_cancelled_write_kb": [cw / 1024 if cw is not None else None for cw in io_cancelled_write_list],
    }


def collect_metrics(log_dir: Path) -> Dict[str, Dict]:
    """
    Collect metrics from all node CSV files in the log directory.

    Returns:
        Dictionary mapping node names to their metrics
    """
    metrics = {}

    # Scan node/ and load_node/ directories
    for subdir in ["node", "load_node"]:
        subdir_path = log_dir / subdir
        if not subdir_path.exists():
            continue

        for node_dir in subdir_path.iterdir():
            if not node_dir.is_dir():
                continue

            csv_file = node_dir / "metrics.csv"
            if not csv_file.exists():
                continue

            node_data = parse_csv_file(csv_file)
            if node_data is not None:
                metrics[node_dir.name] = node_data

    return metrics


def has_data(metrics: Dict[str, Dict], metric_key: str) -> bool:
    """Check if any node has non-None data for the given metric."""
    for node_data in metrics.values():
        if any(v is not None for v in node_data.get(metric_key, [])):
            return True
    return False


def load_all_metadata(log_dir: Path) -> Dict[str, Dict]:
    """
    Load all metadata.json files from node/ and load_node/ directories.

    Returns:
        Dictionary mapping node names to their metadata
    """
    metadata = {}

    # Load regular node and container metadata
    node_dir = log_dir / "node"
    if node_dir.exists():
        for node_subdir in node_dir.iterdir():
            if not node_subdir.is_dir():
                continue

            metadata_file = node_subdir / "metadata.json"
            if metadata_file.exists():
                with open(metadata_file, 'r') as f:
                    metadata[node_subdir.name] = json.load(f)

    # Load composable node metadata
    load_node_dir = log_dir / "load_node"
    if load_node_dir.exists():
        for node_subdir in load_node_dir.iterdir():
            if not node_subdir.is_dir():
                continue

            metadata_file = node_subdir / "metadata.json"
            if metadata_file.exists():
                with open(metadata_file, 'r') as f:
                    metadata[node_subdir.name] = json.load(f)

    return metadata


def build_container_mapping(metadata: Dict[str, Dict]) -> Dict[str, List[str]]:
    """
    Build a mapping from container names to lists of composable nodes they contain.

    Args:
        metadata: Dictionary of node metadata from load_all_metadata()

    Returns:
        Dictionary mapping container names to lists of contained composable node names
    """
    container_map = {}

    # First, identify all containers and initialize their lists
    for node_name, node_meta in metadata.items():
        if node_meta.get("is_container", False):
            container_map[node_name] = []

    # Then, map composable nodes to their containers
    for node_name, node_meta in metadata.items():
        if node_meta.get("type") == "composable_node":
            target_container = node_meta.get("target_container_node_name")
            if target_container and target_container in container_map:
                container_map[target_container].append(node_name)

    return container_map


def abbreviate_name(name: str, max_len: int = 15) -> str:
    """
    Abbreviate a node name if it's longer than max_len.

    Args:
        name: Full node name
        max_len: Maximum length before abbreviation

    Returns:
        Abbreviated name with ellipsis if needed
    """
    if len(name) <= max_len:
        return name
    return name[:max_len-3] + "..."


def inject_statistics_panel(html_path: Path, unit: str = "%", container_map: Dict[str, List[str]] = None):
    """
    Inject JavaScript into HTML file to add a statistics panel that shows box plot statistics.

    The panel appears at the top-left when hovering over a box plot and shows:
    - Node name
    - Count, Min, Max, Q1, Median, Q3, Mean, StdDev
    - List of contained nodes (if hovering over a container)

    Args:
        html_path: Path to the HTML file to modify
        unit: Unit string to append to values (e.g., "%", " MB")
        container_map: Dictionary mapping container names to lists of contained node names
    """
    # Read the HTML file
    with open(html_path, 'r', encoding='utf-8') as f:
        html_content = f.read()

    # Embed container map as JSON
    container_map_json = json.dumps(container_map or {})

    # JavaScript code to add statistics panel
    js_code = f'''
<script>
window.addEventListener('load', function() {{
    // Find the Plotly div (first div with class 'plotly-graph-div')
    var plotlyDiv = document.querySelector('.plotly-graph-div');
    if (!plotlyDiv) return;

    // Container mapping (container name -> list of contained nodes)
    var containerMap = {container_map_json};

    // Create statistics panel
    var statsPanel = document.createElement('div');
    statsPanel.id = 'stats-panel';
    statsPanel.style.cssText = `
        position: fixed;
        top: 80px;
        left: 20px;
        background-color: rgba(255, 255, 255, 0.95);
        border: 2px solid #333;
        border-radius: 8px;
        padding: 15px;
        display: none;
        font-family: 'Courier New', monospace;
        font-size: 13px;
        line-height: 1.6;
        z-index: 10000;
        box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        min-width: 200px;
    `;
    document.body.appendChild(statsPanel);

    // Helper functions for statistics calculations
    function calculateMedian(arr) {{
        var sorted = arr.slice().sort((a, b) => a - b);
        var mid = Math.floor(sorted.length / 2);
        return sorted.length % 2 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
    }}

    function calculateQ1(arr) {{
        var sorted = arr.slice().sort((a, b) => a - b);
        var mid = Math.floor(sorted.length / 2);
        var lowerHalf = sorted.length % 2 ? sorted.slice(0, mid) : sorted.slice(0, mid);
        return calculateMedian(lowerHalf);
    }}

    function calculateQ3(arr) {{
        var sorted = arr.slice().sort((a, b) => a - b);
        var mid = Math.floor(sorted.length / 2);
        var upperHalf = sorted.length % 2 ? sorted.slice(mid + 1) : sorted.slice(mid);
        return calculateMedian(upperHalf);
    }}

    function calculateMean(arr) {{
        return arr.reduce((sum, val) => sum + val, 0) / arr.length;
    }}

    function calculateStdDev(arr) {{
        var mean = calculateMean(arr);
        var squareDiffs = arr.map(val => Math.pow(val - mean, 2));
        var avgSquareDiff = calculateMean(squareDiffs);
        return Math.sqrt(avgSquareDiff);
    }}

    // Listen to hover events
    plotlyDiv.on('plotly_hover', function(data) {{
        var pt = data.points[0];
        var trace = pt.data;
        var yValues = trace.y;

        // Calculate statistics
        var count = yValues.length;
        var min = Math.min(...yValues);
        var max = Math.max(...yValues);
        var q1 = calculateQ1(yValues);
        var median = calculateMedian(yValues);
        var q3 = calculateQ3(yValues);
        var mean = calculateMean(yValues);
        var stddev = calculateStdDev(yValues);

        // Build HTML for panel
        var unit = "{unit}";
        // customdata is an array, all elements are the same (node name)
        var nodeName = trace.customdata ? trace.customdata[0] : trace.name;
        var html = '<div style="font-weight: bold; margin-bottom: 8px; font-size: 14px;">' +
                   nodeName + '</div>';
        html += '<div style="border-top: 1px solid #ccc; padding-top: 8px;">';
        html += 'Count: ' + count + '<br>';
        html += 'Min: ' + min.toFixed(2) + unit + '<br>';
        html += 'Q1: ' + q1.toFixed(2) + unit + '<br>';
        html += 'Median: ' + median.toFixed(2) + unit + '<br>';
        html += 'Q3: ' + q3.toFixed(2) + unit + '<br>';
        html += 'Max: ' + max.toFixed(2) + unit + '<br>';
        html += 'Mean: ' + mean.toFixed(2) + unit + '<br>';
        html += 'StdDev: ' + stddev.toFixed(2) + unit;
        html += '</div>';

        // Add contained nodes if this is a container
        var containedNodes = containerMap[nodeName];
        if (containedNodes && containedNodes.length > 0) {{
            html += '<div style="border-top: 1px solid #ccc; margin-top: 8px; padding-top: 8px;">';
            html += '<b>Contains:</b><br>';
            for (var i = 0; i < containedNodes.length; i++) {{
                html += '  • ' + containedNodes[i] + '<br>';
            }}
            html += '</div>';
        }}

        statsPanel.innerHTML = html;
        statsPanel.style.display = 'block';
    }});

    // Hide panel when not hovering
    plotlyDiv.on('plotly_unhover', function() {{
        statsPanel.style.display = 'none';
    }});
}});
</script>
'''

    # Insert JavaScript before </body> tag
    html_content = html_content.replace('</body>', js_code + '\n</body>')

    # Write back to file
    with open(html_path, 'w', encoding='utf-8') as f:
        f.write(html_content)


def inject_container_panel(html_path: Path, container_map: Dict[str, List[str]]):
    """
    Inject JavaScript into HTML file to add a container panel for timeline charts.

    The panel appears at the top-left when hovering over a container's curve and shows:
    - Container name
    - List of contained composable nodes

    Args:
        html_path: Path to the HTML file to modify
        container_map: Dictionary mapping container names to lists of contained node names
    """
    # Read the HTML file
    with open(html_path, 'r', encoding='utf-8') as f:
        html_content = f.read()

    # Embed container map as JSON
    container_map_json = json.dumps(container_map or {})

    # JavaScript code to add container panel
    js_code = f'''
<script>
window.addEventListener('load', function() {{
    // Find the Plotly div (first div with class 'plotly-graph-div')
    var plotlyDiv = document.querySelector('.plotly-graph-div');
    if (!plotlyDiv) return;

    // Container mapping (container name -> list of contained nodes)
    var containerMap = {container_map_json};

    // Create container panel
    var containerPanel = document.createElement('div');
    containerPanel.id = 'container-panel';
    containerPanel.style.cssText = `
        position: fixed;
        top: 80px;
        left: 20px;
        background-color: rgba(255, 255, 255, 0.95);
        border: 2px solid #333;
        border-radius: 8px;
        padding: 15px;
        display: none;
        font-family: 'Courier New', monospace;
        font-size: 13px;
        line-height: 1.6;
        z-index: 10000;
        box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        min-width: 200px;
    `;
    document.body.appendChild(containerPanel);

    // Listen to hover events
    plotlyDiv.on('plotly_hover', function(data) {{
        var pt = data.points[0];
        var trace = pt.data;
        var traceName = trace.name;

        // Check if this trace is a container
        var containedNodes = containerMap[traceName];

        if (containedNodes && containedNodes.length > 0) {{
            var html = '<div style="font-weight: bold; margin-bottom: 8px; font-size: 14px;">';
            html += 'Container: ' + traceName + '</div>';
            html += '<div style="border-top: 1px solid #ccc; padding-top: 8px;">';
            html += '<b>Contains:</b><br>';
            for (var i = 0; i < containedNodes.length; i++) {{
                html += '  • ' + containedNodes[i] + '<br>';
            }}
            html += '</div>';

            containerPanel.innerHTML = html;
            containerPanel.style.display = 'block';
        }} else {{
            containerPanel.style.display = 'none';
        }}
    }});

    // Hide panel when not hovering
    plotlyDiv.on('plotly_unhover', function() {{
        containerPanel.style.display = 'none';
    }});
}});
</script>
'''

    # Insert JavaScript before </body> tag
    html_content = html_content.replace('</body>', js_code + '\n</body>')

    # Write back to file
    with open(html_path, 'w', encoding='utf-8') as f:
        f.write(html_content)


def create_individual_charts(
    metrics: Dict[str, Dict],
    output_dir: Path,
    log_dir: Path,
    metrics_to_plot: List[str] = None
):
    """
    Create individual interactive HTML charts (one file per chart).

    Args:
        metrics: Dictionary of node metrics
        output_dir: Base directory (will create plot/ subdirectory)
        log_dir: Log directory to read metadata from
        metrics_to_plot: List of metric groups to plot (cpu, memory, io, gpu, network, all)
    """
    if not metrics:
        print("No metrics data available to plot")
        return

    # Default to all metrics
    if metrics_to_plot is None or "all" in metrics_to_plot:
        metrics_to_plot = ["cpu", "memory", "io", "gpu", "network"]

    # Create plot/ subdirectory
    output_dir = output_dir / "plot"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Load metadata and build container mapping
    metadata = load_all_metadata(log_dir)
    container_map = build_container_mapping(metadata)

    # Determine available metrics
    has_cpu = has_data(metrics, "cpu")
    has_memory = has_data(metrics, "rss_mb")
    has_io = has_data(metrics, "io_read_mbps") or has_data(metrics, "io_write_mbps")
    has_gpu_mem = has_data(metrics, "gpu_mem_mb")
    has_gpu_util = has_data(metrics, "gpu_util")
    has_gpu_temp = has_data(metrics, "gpu_temp")
    has_gpu_power = has_data(metrics, "gpu_power_w")
    has_gpu_clock = has_data(metrics, "gpu_graphics_clock")
    has_network = has_data(metrics, "tcp") or has_data(metrics, "udp")
    has_threads = has_data(metrics, "num_threads")
    has_syscalls = has_data(metrics, "io_syscr") or has_data(metrics, "io_syscw")
    has_storage_io = has_data(metrics, "io_storage_read_mb") or has_data(metrics, "io_storage_write_mb")

    chart_config = {
        'displayModeBar': True,
        'displaylogo': False,
        'modeBarButtonsToRemove': ['lasso2d', 'select2d'],
        'toImageButtonOptions': {
            'format': 'png',
            'filename': 'chart',
            'height': 1000,
            'width': 1600,
            'scale': 2
        }
    }

    charts_created = []

    # Create CPU timeline chart
    if "cpu" in metrics_to_plot and has_cpu:
        fig = go.Figure()
        for node_name, node_data in sorted(metrics.items()):
            times = node_data["times"]
            cpu_values = node_data["cpu"]
            plot_times = [t for t, v in zip(times, cpu_values) if v is not None]
            plot_values = [v for v in cpu_values if v is not None]

            if plot_values:
                # Check if this node is a container
                node_meta = metadata.get(node_name, {})
                is_container = node_meta.get("is_container", False)

                if is_container and node_name in container_map:
                    # Container: show contained nodes in hover
                    contained_nodes = container_map[node_name]
                    if contained_nodes:
                        nodes_list = "<br>Contains: " + ", ".join(contained_nodes[:5])
                        if len(contained_nodes) > 5:
                            nodes_list += f", ... ({len(contained_nodes)-5} more)"
                    else:
                        nodes_list = "<br>Contains: (empty)"

                    hovertemplate = f'<b>Container: %{{fullData.name}}</b><br>Time: %{{x:.2f}}s<br>CPU: %{{y:.2f}}%{nodes_list}<extra></extra>'
                else:
                    # Regular node: simple hover
                    hovertemplate = '<b>%{fullData.name}</b><br>Time: %{x:.2f}s<br>CPU: %{y:.2f}%<extra></extra>'

                fig.add_trace(
                    go.Scatter(
                        x=plot_times,
                        y=plot_values,
                        mode='lines',
                        name=node_name,
                        hovertemplate=hovertemplate
                    )
                )

        fig.update_layout(
            title="CPU Usage Over Time",
            xaxis_title="Time (s)",
            yaxis_title="CPU Usage (%)",
            height=800,
            hovermode='closest',
            template='plotly_white',
            showlegend=False
        )

        output_path = output_dir / "cpu_timeline.html"
        fig.write_html(str(output_path), config=chart_config)

        # Inject container panel JavaScript
        inject_container_panel(output_path, container_map)

        charts_created.append(("CPU Timeline", output_path))

    # Create Memory timeline chart
    if "memory" in metrics_to_plot and has_memory:
        fig = go.Figure()
        for node_name, node_data in sorted(metrics.items()):
            times = node_data["times"]
            mem_values = node_data["rss_mb"]
            plot_times = [t for t, v in zip(times, mem_values) if v is not None]
            plot_values = [v for v in mem_values if v is not None]

            if plot_values:
                # Check if this node is a container
                node_meta = metadata.get(node_name, {})
                is_container = node_meta.get("is_container", False)

                if is_container and node_name in container_map:
                    # Container: show contained nodes in hover
                    contained_nodes = container_map[node_name]
                    if contained_nodes:
                        nodes_list = "<br>Contains: " + ", ".join(contained_nodes[:5])
                        if len(contained_nodes) > 5:
                            nodes_list += f", ... ({len(contained_nodes)-5} more)"
                    else:
                        nodes_list = "<br>Contains: (empty)"

                    hovertemplate = f'<b>Container: %{{fullData.name}}</b><br>Time: %{{x:.2f}}s<br>Memory: %{{y:.2f}} MB{nodes_list}<extra></extra>'
                else:
                    # Regular node: simple hover
                    hovertemplate = '<b>%{fullData.name}</b><br>Time: %{x:.2f}s<br>Memory: %{y:.2f} MB<extra></extra>'

                fig.add_trace(
                    go.Scatter(
                        x=plot_times,
                        y=plot_values,
                        mode='lines',
                        name=node_name,
                        hovertemplate=hovertemplate
                    )
                )

        fig.update_layout(
            title="Memory Usage Over Time",
            xaxis_title="Time (s)",
            yaxis_title="Memory Usage (MB)",
            height=800,
            hovermode='closest',
            template='plotly_white',
            showlegend=False
        )

        output_path = output_dir / "memory_timeline.html"
        fig.write_html(str(output_path), config=chart_config)

        # Inject container panel JavaScript
        inject_container_panel(output_path, container_map)

        charts_created.append(("Memory Timeline", output_path))

    # Create CPU distribution chart (sorted by average)
    if "cpu" in metrics_to_plot and has_cpu:
        # Calculate averages and sort
        cpu_data = []
        for node_name, node_data in metrics.items():
            cpu_values = [v for v in node_data["cpu"] if v is not None]
            if cpu_values:
                avg_cpu = sum(cpu_values) / len(cpu_values)
                cpu_data.append((node_name, cpu_values, avg_cpu))

        # Sort by average (ascending: low to high)
        cpu_data.sort(key=lambda x: x[2], reverse=False)

        fig = go.Figure()
        for node_name, cpu_values, avg_cpu in cpu_data:
            # Abbreviate label, store full name for hover
            abbreviated_name = abbreviate_name(node_name, max_len=15)

            fig.add_trace(
                go.Box(
                    y=cpu_values,
                    name=abbreviated_name,
                    boxmean='sd',
                    customdata=[node_name] * len(cpu_values),  # Array with repeated value
                    hovertemplate='<b>%{customdata}</b><br>%{y:.2f}%<extra></extra>',
                    hoveron='points'  # Only show hover for actual data points, not box components
                )
            )

        fig.update_layout(
            title="CPU Usage Distribution (sorted by average, low to high)",
            yaxis_title="CPU Usage (%)",
            height=800,
            hovermode='closest',
            template='plotly_white',
            showlegend=False
        )

        output_path = output_dir / "cpu_distribution.html"
        fig.write_html(str(output_path), config=chart_config)

        # Inject statistics panel JavaScript with container mapping
        inject_statistics_panel(output_path, unit="%", container_map=container_map)

        charts_created.append(("CPU Distribution", output_path))

    # Create Memory distribution chart (sorted by average)
    if "memory" in metrics_to_plot and has_memory:
        # Calculate averages and sort
        mem_data = []
        for node_name, node_data in metrics.items():
            mem_values = [v for v in node_data["rss_mb"] if v is not None]
            if mem_values:
                avg_mem = sum(mem_values) / len(mem_values)
                mem_data.append((node_name, mem_values, avg_mem))

        # Sort by average (ascending: low to high)
        mem_data.sort(key=lambda x: x[2], reverse=False)

        fig = go.Figure()
        for node_name, mem_values, avg_mem in mem_data:
            # Abbreviate label, store full name for hover
            abbreviated_name = abbreviate_name(node_name, max_len=15)

            fig.add_trace(
                go.Box(
                    y=mem_values,
                    name=abbreviated_name,
                    boxmean='sd',
                    customdata=[node_name] * len(mem_values),  # Array with repeated value
                    hovertemplate='<b>%{customdata}</b><br>%{y:.2f} MB<extra></extra>',
                    hoveron='points'  # Only show hover for actual data points, not box components
                )
            )

        fig.update_layout(
            title="Memory Usage Distribution (sorted by average, low to high)",
            yaxis_title="Memory Usage (MB)",
            height=800,
            hovermode='closest',
            template='plotly_white',
            showlegend=False
        )

        output_path = output_dir / "memory_distribution.html"
        fig.write_html(str(output_path), config=chart_config)

        # Inject statistics panel JavaScript with container mapping
        inject_statistics_panel(output_path, unit=" MB", container_map=container_map)

        charts_created.append(("Memory Distribution", output_path))

    # Create I/O timeline chart
    if "io" in metrics_to_plot and has_io:
        fig = go.Figure()
        for node_name, node_data in sorted(metrics.items()):
            times = node_data["times"]

            # Plot read rates
            read_values = node_data["io_read_mbps"]
            plot_times_read = [t for t, v in zip(times, read_values) if v is not None]
            plot_values_read = [v for v in read_values if v is not None]

            if plot_values_read:
                fig.add_trace(
                    go.Scatter(
                        x=plot_times_read,
                        y=plot_values_read,
                        mode='lines',
                        name=f"{node_name} (read)",
                        hovertemplate='<b>%{fullData.name}</b><br>Time: %{x:.2f}s<br>Rate: %{y:.2f} MB/s<extra></extra>',
                        line=dict(dash='solid')
                    )
                )

            # Plot write rates
            write_values = node_data["io_write_mbps"]
            plot_times_write = [t for t, v in zip(times, write_values) if v is not None]
            plot_values_write = [v for v in write_values if v is not None]

            if plot_values_write:
                fig.add_trace(
                    go.Scatter(
                        x=plot_times_write,
                        y=plot_values_write,
                        mode='lines',
                        name=f"{node_name} (write)",
                        hovertemplate='<b>%{fullData.name}</b><br>Time: %{x:.2f}s<br>Rate: %{y:.2f} MB/s<extra></extra>',
                        line=dict(dash='dash')
                    )
                )

        fig.update_layout(
            title="I/O Rates Over Time",
            xaxis_title="Time (s)",
            yaxis_title="I/O Rate (MB/s)",
            height=800,
            hovermode='closest',
            template='plotly_white',
            showlegend=False
        )

        output_path = output_dir / "io_timeline.html"
        fig.write_html(str(output_path), config=chart_config)
        charts_created.append(("I/O Timeline", output_path))

    # Create GPU usage timeline chart
    if "gpu" in metrics_to_plot and (has_gpu_mem or has_gpu_util):
        fig = go.Figure()
        for node_name, node_data in sorted(metrics.items()):
            times = node_data["times"]

            # Plot GPU memory
            if has_gpu_mem:
                gpu_mem_values = node_data["gpu_mem_mb"]
                plot_times = [t for t, v in zip(times, gpu_mem_values) if v is not None]
                plot_values = [v for v in gpu_mem_values if v is not None]

                if plot_values:
                    fig.add_trace(
                        go.Scatter(
                            x=plot_times,
                            y=plot_values,
                            mode='lines',
                            name=f"{node_name}",
                            hovertemplate='<b>%{fullData.name}</b><br>Time: %{x:.2f}s<br>GPU Mem: %{y:.2f} MB<extra></extra>'
                        )
                    )

        fig.update_layout(
            title="GPU Memory Usage Over Time",
            xaxis_title="Time (s)",
            yaxis_title="GPU Memory (MB)",
            height=800,
            hovermode='closest',
            template='plotly_white',
            showlegend=False
        )

        output_path = output_dir / "gpu_timeline.html"
        fig.write_html(str(output_path), config=chart_config)
        charts_created.append(("GPU Timeline", output_path))

    # Create GPU temp/power chart
    if "gpu" in metrics_to_plot and (has_gpu_temp or has_gpu_power):
        fig = go.Figure()
        for node_name, node_data in sorted(metrics.items()):
            times = node_data["times"]

            if has_gpu_temp:
                temp_values = node_data["gpu_temp"]
                plot_times = [t for t, v in zip(times, temp_values) if v is not None]
                plot_values = [v for v in temp_values if v is not None]

                if plot_values:
                    fig.add_trace(
                        go.Scatter(
                            x=plot_times,
                            y=plot_values,
                            mode='lines',
                            name=node_name,
                            hovertemplate='<b>%{fullData.name}</b><br>Time: %{x:.2f}s<br>Temp: %{y:.0f}°C<extra></extra>'
                        )
                    )

        fig.update_layout(
            title="GPU Temperature Over Time",
            xaxis_title="Time (s)",
            yaxis_title="GPU Temperature (°C)",
            height=800,
            hovermode='closest',
            template='plotly_white',
            showlegend=False
        )

        output_path = output_dir / "gpu_temp_power.html"
        fig.write_html(str(output_path), config=chart_config)
        charts_created.append(("GPU Temp/Power", output_path))

    # Create Network connections chart
    if "network" in metrics_to_plot and has_network:
        fig = go.Figure()
        for node_name, node_data in sorted(metrics.items()):
            times = node_data["times"]

            tcp_values = node_data["tcp"]
            plot_times = [t for t, v in zip(times, tcp_values) if v is not None]
            plot_values = [v for v in tcp_values if v is not None]

            if plot_values:
                fig.add_trace(
                    go.Scatter(
                        x=plot_times,
                        y=plot_values,
                        mode='lines',
                        name=node_name,
                        hovertemplate='<b>%{fullData.name}</b><br>Time: %{x:.2f}s<br>TCP: %{y:.0f}<extra></extra>'
                    )
                )

        fig.update_layout(
            title="Network Connections Over Time",
            xaxis_title="Time (s)",
            yaxis_title="Connections",
            height=800,
            hovermode='closest',
            template='plotly_white',
            showlegend=False
        )

        output_path = output_dir / "network_timeline.html"
        fig.write_html(str(output_path), config=chart_config)
        charts_created.append(("Network Timeline", output_path))

    # Create GPU clocks chart
    if "gpu" in metrics_to_plot and has_gpu_clock:
        fig = go.Figure()
        for node_name, node_data in sorted(metrics.items()):
            times = node_data["times"]

            graphics_values = node_data["gpu_graphics_clock"]
            plot_times = [t for t, v in zip(times, graphics_values) if v is not None]
            plot_values = [v for v in graphics_values if v is not None]

            if plot_values:
                fig.add_trace(
                    go.Scatter(
                        x=plot_times,
                        y=plot_values,
                        mode='lines',
                        name=node_name,
                        hovertemplate='<b>%{fullData.name}</b><br>Time: %{x:.2f}s<br>Clock: %{y:.0f} MHz<extra></extra>'
                    )
                )

        fig.update_layout(
            title="GPU Clock Frequencies Over Time",
            xaxis_title="Time (s)",
            yaxis_title="Clock Frequency (MHz)",
            height=800,
            hovermode='closest',
            template='plotly_white',
            showlegend=False
        )

        output_path = output_dir / "gpu_clocks.html"
        fig.write_html(str(output_path), config=chart_config)
        charts_created.append(("GPU Clocks", output_path))

    # Create Thread count timeline chart
    if "cpu" in metrics_to_plot and has_threads:
        fig = go.Figure()
        for node_name, node_data in sorted(metrics.items()):
            times = node_data["times"]
            thread_values = node_data["num_threads"]
            plot_times = [t for t, v in zip(times, thread_values) if v is not None]
            plot_values = [v for v in thread_values if v is not None]

            if plot_values:
                fig.add_trace(
                    go.Scatter(
                        x=plot_times,
                        y=plot_values,
                        mode='lines',
                        name=node_name,
                        hovertemplate='<b>%{fullData.name}</b><br>Time: %{x:.2f}s<br>Threads: %{y:.0f}<extra></extra>'
                    )
                )

        fig.update_layout(
            title="Thread Count Over Time",
            xaxis_title="Time (s)",
            yaxis_title="Thread Count",
            height=800,
            hovermode='closest',
            template='plotly_white',
            showlegend=False
        )

        output_path = output_dir / "threads_timeline.html"
        fig.write_html(str(output_path), config=chart_config)
        charts_created.append(("Thread Count Timeline", output_path))

    # Create I/O Syscalls timeline chart
    if "io" in metrics_to_plot and has_syscalls:
        fig = go.Figure()
        for node_name, node_data in sorted(metrics.items()):
            times = node_data["times"]

            # Plot read syscalls
            syscr_values = node_data["io_syscr"]
            plot_times_read = [t for t, v in zip(times, syscr_values) if v is not None]
            plot_values_read = [v for v in syscr_values if v is not None]

            if plot_values_read:
                fig.add_trace(
                    go.Scatter(
                        x=plot_times_read,
                        y=plot_values_read,
                        mode='lines',
                        name=f"{node_name} (read)",
                        hovertemplate='<b>%{fullData.name}</b><br>Time: %{x:.2f}s<br>Read syscalls: %{y:.0f}<extra></extra>',
                        line=dict(dash='solid')
                    )
                )

            # Plot write syscalls
            syscw_values = node_data["io_syscw"]
            plot_times_write = [t for t, v in zip(times, syscw_values) if v is not None]
            plot_values_write = [v for v in syscw_values if v is not None]

            if plot_values_write:
                fig.add_trace(
                    go.Scatter(
                        x=plot_times_write,
                        y=plot_values_write,
                        mode='lines',
                        name=f"{node_name} (write)",
                        hovertemplate='<b>%{fullData.name}</b><br>Time: %{x:.2f}s<br>Write syscalls: %{y:.0f}<extra></extra>',
                        line=dict(dash='dash')
                    )
                )

        fig.update_layout(
            title="I/O System Calls Over Time",
            xaxis_title="Time (s)",
            yaxis_title="System Call Count",
            height=800,
            hovermode='closest',
            template='plotly_white',
            showlegend=False
        )

        output_path = output_dir / "io_syscalls_timeline.html"
        fig.write_html(str(output_path), config=chart_config)
        charts_created.append(("I/O Syscalls Timeline", output_path))

    # Create Storage I/O timeline chart (cache-excluded disk I/O)
    if "io" in metrics_to_plot and has_storage_io:
        fig = go.Figure()
        for node_name, node_data in sorted(metrics.items()):
            times = node_data["times"]

            # Plot storage read
            storage_read_values = node_data["io_storage_read_mb"]
            plot_times_read = [t for t, v in zip(times, storage_read_values) if v is not None]
            plot_values_read = [v for v in storage_read_values if v is not None]

            if plot_values_read:
                fig.add_trace(
                    go.Scatter(
                        x=plot_times_read,
                        y=plot_values_read,
                        mode='lines',
                        name=f"{node_name} (read)",
                        hovertemplate='<b>%{fullData.name}</b><br>Time: %{x:.2f}s<br>Storage Read: %{y:.2f} MB<extra></extra>',
                        line=dict(dash='solid')
                    )
                )

            # Plot storage write
            storage_write_values = node_data["io_storage_write_mb"]
            plot_times_write = [t for t, v in zip(times, storage_write_values) if v is not None]
            plot_values_write = [v for v in storage_write_values if v is not None]

            if plot_values_write:
                fig.add_trace(
                    go.Scatter(
                        x=plot_times_write,
                        y=plot_values_write,
                        mode='lines',
                        name=f"{node_name} (write)",
                        hovertemplate='<b>%{fullData.name}</b><br>Time: %{x:.2f}s<br>Storage Write: %{y:.2f} MB<extra></extra>',
                        line=dict(dash='dash')
                    )
                )

        fig.update_layout(
            title="Storage I/O Over Time (cache-excluded)",
            xaxis_title="Time (s)",
            yaxis_title="Storage I/O (MB)",
            height=800,
            hovermode='closest',
            template='plotly_white',
            showlegend=False
        )

        output_path = output_dir / "io_storage_timeline.html"
        fig.write_html(str(output_path), config=chart_config)
        charts_created.append(("Storage I/O Timeline", output_path))

    # Return created charts list for minimalist summary
    return charts_created


def calculate_statistics(metrics: Dict[str, Dict], output_path: Path):
    """Generate comprehensive statistics report."""
    if not metrics:
        return

    stats_lines = []
    stats_lines.append("=" * 80)
    stats_lines.append("RESOURCE USAGE STATISTICS")
    stats_lines.append("=" * 80)
    stats_lines.append("")

    # CPU statistics
    cpu_stats = []
    for node_name, node_data in metrics.items():
        cpu_values = [v for v in node_data["cpu"] if v is not None]
        if cpu_values:
            cpu_stats.append({
                'node': node_name,
                'max': max(cpu_values),
                'avg': sum(cpu_values) / len(cpu_values)
            })

    if cpu_stats:
        stats_lines.append("CPU USAGE (Top 10)")
        stats_lines.append("-" * 80)
        cpu_stats.sort(key=lambda x: x['max'], reverse=True)
        for i, stat in enumerate(cpu_stats[:10], 1):
            stats_lines.append(f"{i:2d}. {stat['node']:50s} Max: {stat['max']:6.2f}%  Avg: {stat['avg']:6.2f}%")
        stats_lines.append("")

    # Memory statistics
    mem_stats = []
    for node_name, node_data in metrics.items():
        mem_values = [v for v in node_data["rss_mb"] if v is not None]
        if mem_values:
            mem_stats.append({
                'node': node_name,
                'max': max(mem_values),
                'avg': sum(mem_values) / len(mem_values)
            })

    if mem_stats:
        stats_lines.append("MEMORY USAGE (Top 10)")
        stats_lines.append("-" * 80)
        mem_stats.sort(key=lambda x: x['max'], reverse=True)
        for i, stat in enumerate(mem_stats[:10], 1):
            stats_lines.append(f"{i:2d}. {stat['node']:50s} Max: {stat['max']:8.2f} MB  Avg: {stat['avg']:8.2f} MB")
        stats_lines.append("")

    # Thread count statistics
    thread_stats = []
    for node_name, node_data in metrics.items():
        thread_values = [v for v in node_data["num_threads"] if v is not None]
        if thread_values:
            thread_stats.append({
                'node': node_name,
                'max': max(thread_values),
                'avg': sum(thread_values) / len(thread_values)
            })

    if thread_stats:
        stats_lines.append("THREAD COUNT (Top 10)")
        stats_lines.append("-" * 80)
        thread_stats.sort(key=lambda x: x['max'], reverse=True)
        for i, stat in enumerate(thread_stats[:10], 1):
            stats_lines.append(f"{i:2d}. {stat['node']:50s} Max: {stat['max']:6.0f}  Avg: {stat['avg']:6.1f}")
        stats_lines.append("")

    stats_lines.append("=" * 80)

    # Write to file
    with open(output_path, 'w') as f:
        f.write('\n'.join(stats_lines))


def list_available_metrics(metrics: Dict[str, Dict]):
    """List which metrics are available in the dataset."""
    print("\nAvailable metrics in this dataset:")
    print("=" * 50)

    metric_checks = [
        ("cpu", "CPU usage"),
        ("rss_mb", "Memory usage"),
        ("io_read_mbps", "I/O read rates"),
        ("io_write_mbps", "I/O write rates"),
        ("io_syscr", "I/O syscalls (read)"),
        ("io_syscw", "I/O syscalls (write)"),
        ("io_storage_read_mb", "Storage I/O read (cache-excluded)"),
        ("io_storage_write_mb", "Storage I/O write (cache-excluded)"),
        ("num_threads", "Thread count"),
        ("gpu_mem_mb", "GPU memory"),
        ("gpu_util", "GPU utilization"),
        ("gpu_temp", "GPU temperature"),
        ("gpu_power_w", "GPU power"),
        ("gpu_graphics_clock", "GPU clocks"),
        ("tcp", "Network connections (TCP)"),
        ("udp", "Network connections (UDP)"),
    ]

    for metric_key, metric_name in metric_checks:
        if has_data(metrics, metric_key):
            print(f"  ✓ {metric_name}")
        else:
            print(f"  ✗ {metric_name} (no data)")


def main():
    parser = argparse.ArgumentParser(
        description="Generate interactive HTML charts from play_launch resource logs"
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        help="Path to specific log directory (e.g., play_log/2025-10-29_12-00-00). "
             "If not provided, uses latest log in base-log-dir."
    )
    parser.add_argument(
        "--base-log-dir",
        type=Path,
        default=Path("./play_log"),
        help="Base directory containing timestamped log directories (default: ./play_log)"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Directory to save dashboard and statistics (default: same as log-dir)"
    )
    parser.add_argument(
        "--metrics",
        nargs='+',
        choices=['cpu', 'memory', 'io', 'gpu', 'network', 'all'],
        default=['all'],
        help="Which metrics to include in the dashboard (default: all)"
    )
    parser.add_argument(
        "--list-metrics",
        action="store_true",
        help="List available metrics in the log and exit"
    )

    args = parser.parse_args()

    # Determine log directory
    if args.log_dir:
        log_dir = args.log_dir
        if not log_dir.exists():
            print(f"Error: Log directory not found: {log_dir}")
            sys.exit(1)
    else:
        try:
            log_dir = find_latest_log_dir(args.base_log_dir)
        except FileNotFoundError as e:
            print(f"Error: {e}")
            sys.exit(1)

    # Determine output directory
    output_dir = args.output_dir if args.output_dir else log_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    # Collect metrics
    metrics = collect_metrics(log_dir)

    if not metrics:
        print("Error: No metrics files found")
        print(f"Expected: {log_dir}/node/*/metrics.csv, {log_dir}/load_node/*/metrics.csv")
        sys.exit(1)

    # List metrics if requested
    if args.list_metrics:
        list_available_metrics(metrics)
        return

    # Print minimalist header
    print(f"Log dir:    {log_dir}")
    print(f"Output dir: {output_dir / 'plot'}")

    # Generate individual charts
    charts_created = create_individual_charts(metrics, output_dir, log_dir, args.metrics)

    # Generate statistics (in plot/ subdirectory)
    stats_path = output_dir / "plot" / "statistics.txt"
    calculate_statistics(metrics, stats_path)

    # Print minimalist summary of generated charts
    if charts_created:
        print("\nGenerated:")
        for chart_name, chart_path in charts_created:
            print(f"  {chart_path.name}")
        print(f"  statistics.txt")

    # List charts not generated with reasons
    requested_metrics = set(args.metrics)
    if "all" in requested_metrics:
        requested_metrics = {"cpu", "memory", "io", "gpu", "network"}

    not_generated = []

    if "cpu" in requested_metrics:
        if not has_data(metrics, "cpu"):
            not_generated.append("CPU charts (no data)")

    if "memory" in requested_metrics:
        if not has_data(metrics, "rss_mb"):
            not_generated.append("Memory charts (no data)")

    if "io" in requested_metrics:
        if not (has_data(metrics, "io_read_mbps") or has_data(metrics, "io_write_mbps")):
            not_generated.append("I/O charts (no data)")

    if "gpu" in requested_metrics:
        has_any_gpu = (has_data(metrics, "gpu_mem_mb") or has_data(metrics, "gpu_util") or
                       has_data(metrics, "gpu_temp") or has_data(metrics, "gpu_power_w") or
                       has_data(metrics, "gpu_graphics_clock"))
        if not has_any_gpu:
            not_generated.append("GPU charts (no data)")

    if "network" in requested_metrics:
        if not (has_data(metrics, "tcp") or has_data(metrics, "udp")):
            not_generated.append("Network charts (no data)")

    if not_generated:
        print("\nNot generated:")
        for reason in not_generated:
            print(f"  {reason}")


if __name__ == "__main__":
    main()
