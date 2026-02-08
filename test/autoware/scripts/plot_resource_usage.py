#!/usr/bin/env python3
"""
Enhanced resource usage plotting tool for play_launch logs.

Generates comprehensive visualizations including:
- CPU and memory usage (timeline and distribution)
- GPU usage and distribution (when available)
- I/O rates (read/write timeline and distribution)
- Network statistics
- Comprehensive statistics report
"""

import argparse
import csv
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import sys

try:
    import matplotlib.pyplot as plt
    import matplotlib.dates as mdates
    import numpy as np
except ImportError:
    print(
        "Error: matplotlib and numpy are required. Install with: pip install matplotlib numpy"
    )
    sys.exit(1)


def find_latest_log_dir(base_log_dir: Path) -> Path:
    """Find the most recent timestamped log directory."""
    log_dirs = [d for d in base_log_dir.iterdir() if d.is_dir() and d.name[0].isdigit()]
    if not log_dirs:
        raise FileNotFoundError(f"No log directories found in {base_log_dir}")

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
            tcp_str = row.get("tcp_connections", "0").strip()
            tcp_conns.append(int(tcp_str) if tcp_str else 0)

            udp_str = row.get("udp_connections", "0").strip()
            udp_conns.append(int(udp_str) if udp_str else 0)

    if not timestamps:
        return None

    # Convert to relative time and appropriate units
    start_time = timestamps[0]
    relative_times = [(ts - start_time).total_seconds() for ts in timestamps]
    rss_mb = [rss / (1024 * 1024) for rss in rss_bytes]
    gpu_mem_mb = [
        (gm / (1024 * 1024)) if gm is not None else None for gm in gpu_mem_bytes
    ]
    gpu_power_watts = [
        (power / 1000.0) if power is not None else None for power in gpu_powers
    ]
    io_read_mb_s = [
        (rate / (1024 * 1024)) if rate is not None else None for rate in io_read_rates
    ]
    io_write_mb_s = [
        (rate / (1024 * 1024)) if rate is not None else None for rate in io_write_rates
    ]

    return {
        "times": relative_times,
        "cpu": cpu_percents,
        "mem": rss_mb,
        "gpu_mem": gpu_mem_mb,
        "gpu_util": gpu_util_percents,
        "gpu_mem_util": gpu_mem_util_percents,
        "gpu_temp": gpu_temps,
        "gpu_power": gpu_power_watts,
        "gpu_graphics_clock": gpu_graphics_clocks,
        "gpu_memory_clock": gpu_memory_clocks,
        "io_read_rate": io_read_mb_s,
        "io_write_rate": io_write_mb_s,
        "tcp_conns": tcp_conns,
        "udp_conns": udp_conns,
    }


def load_all_metrics(log_dir: Path) -> Dict[str, Dict]:
    """Load all metrics.csv files from node and load_node directories."""
    metrics = {}

    # Find all metrics.csv files in node/ and load_node/ subdirectories
    csv_files = []
    for subdir in ["node", "load_node"]:
        subdir_path = log_dir / subdir
        if subdir_path.exists():
            csv_files.extend(subdir_path.glob("*/metrics.csv"))

    if not csv_files:
        raise FileNotFoundError(f"No metrics.csv files found in {log_dir}/node or {log_dir}/load_node")

    print(f"Loading {len(csv_files)} metrics.csv files...")

    for csv_path in csv_files:
        # Use parent directory name as node name
        node_name = csv_path.parent.name

        node_metrics = parse_csv_file(csv_path)
        if node_metrics:
            metrics[node_name] = node_metrics

    print(f"Loaded metrics for {len(metrics)} nodes")
    return metrics


def has_gpu_data(metrics: Dict[str, Dict]) -> bool:
    """Check if any node has GPU data (any GPU metric)."""
    for node_data in metrics.values():
        # Check all GPU metric fields
        if (any(g is not None for g in node_data["gpu_mem"]) or
            any(g is not None for g in node_data["gpu_util"]) or
            any(g is not None for g in node_data["gpu_mem_util"]) or
            any(g is not None for g in node_data["gpu_temp"]) or
            any(g is not None for g in node_data["gpu_power"]) or
            any(g is not None for g in node_data["gpu_graphics_clock"]) or
            any(g is not None for g in node_data["gpu_memory_clock"])):
            return True
    return False


def has_io_data(metrics: Dict[str, Dict]) -> bool:
    """Check if any node has I/O rate data."""
    for node_data in metrics.values():
        if any(r is not None for r in node_data["io_read_rate"]):
            return True
    return False


def plot_timeline(
    metrics: Dict[str, Dict],
    metric_key: str,
    ylabel: str,
    title: str,
    output_path: Path,
    node_colors: Dict = None,
) -> Dict:
    """Generic timeline plotting function."""
    fig, ax = plt.subplots(figsize=(14, 8))

    if node_colors is None:
        node_colors = {}

    for idx, (node_name, node_data) in enumerate(sorted(metrics.items())):
        values = node_data[metric_key]
        times = node_data["times"]

        # Filter out None values for plotting (all GPU and I/O metrics can have None)
        if metric_key in ["gpu_mem", "gpu_util", "gpu_mem_util", "gpu_temp", "gpu_power",
                          "gpu_graphics_clock", "gpu_memory_clock", "io_read_rate", "io_write_rate"]:
            plot_times = [t for t, v in zip(times, values) if v is not None]
            plot_values = [v for v in values if v is not None]
        else:
            plot_times = times
            plot_values = values

        if not plot_values:
            continue

        if node_name in node_colors:
            color = node_colors[node_name][1]
        else:
            color = None
        ax.plot(plot_times, plot_values, label=f"[{idx}]", linewidth=0.8, color=color)

        if node_name not in node_colors:
            node_colors[node_name] = (idx, ax.get_lines()[-1].get_color())

    ax.set_xlabel("Time (seconds)", fontsize=12)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.set_title(title, fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()

    print(f"{title} saved to: {output_path}")
    return node_colors


def plot_distribution(
    metrics: Dict[str, Dict],
    metric_key: str,
    xlabel: str,
    title: str,
    output_path: Path,
    node_colors: Dict,
):
    """Generic distribution box plot function."""
    fig, ax = plt.subplots(figsize=(14, 8))

    data_to_plot = []
    positions = []
    colors_to_use = []

    for idx, (node_name, node_data) in enumerate(sorted(metrics.items())):
        values = node_data[metric_key]

        # Filter None values (all GPU and I/O metrics can have None)
        if metric_key in ["gpu_mem", "gpu_util", "gpu_mem_util", "gpu_temp", "gpu_power",
                          "gpu_graphics_clock", "gpu_memory_clock", "io_read_rate", "io_write_rate"]:
            plot_values = [v for v in values if v is not None]
        else:
            plot_values = values

        if plot_values:
            data_to_plot.append(plot_values)
            positions.append(idx)
            colors_to_use.append(node_colors[node_name][1])

    if not data_to_plot:
        print(f"Warning: No data for {metric_key} distribution plot")
        return

    bp = ax.boxplot(
        data_to_plot,
        positions=positions,
        widths=0.6,
        patch_artist=True,
        showfliers=False,
    )

    for patch, color in zip(bp["boxes"], colors_to_use):
        patch.set_facecolor(color)
        patch.set_alpha(0.6)

    ax.set_xlabel("Node Index", fontsize=12)
    ax.set_ylabel(xlabel, fontsize=12)
    ax.set_title(title, fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.3, axis="y")
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()

    print(f"{title} saved to: {output_path}")


def plot_gpu_clocks(
    metrics: Dict[str, Dict],
    output_path: Path,
    node_colors: Dict,
):
    """Plot GPU graphics and memory clocks on the same chart."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 12))

    # Plot graphics clocks
    for idx, (node_name, node_data) in enumerate(sorted(metrics.items())):
        graphics_clocks = node_data["gpu_graphics_clock"]
        times = node_data["times"]

        # Filter out None values
        plot_times = [t for t, v in zip(times, graphics_clocks) if v is not None]
        plot_values = [v for v in graphics_clocks if v is not None]

        if not plot_values:
            continue

        color = node_colors.get(node_name, (idx, None))[1]
        ax1.plot(plot_times, plot_values, label=f"[{idx}]", linewidth=0.8, color=color)

    ax1.set_xlabel("Time (seconds)", fontsize=12)
    ax1.set_ylabel("Graphics Clock (MHz)", fontsize=12)
    ax1.set_title("GPU Graphics Clock Over Time", fontsize=14, fontweight="bold")
    ax1.grid(True, alpha=0.3)

    # Plot memory clocks
    for idx, (node_name, node_data) in enumerate(sorted(metrics.items())):
        memory_clocks = node_data["gpu_memory_clock"]
        times = node_data["times"]

        # Filter out None values
        plot_times = [t for t, v in zip(times, memory_clocks) if v is not None]
        plot_values = [v for v in memory_clocks if v is not None]

        if not plot_values:
            continue

        color = node_colors.get(node_name, (idx, None))[1]
        ax2.plot(plot_times, plot_values, label=f"[{idx}]", linewidth=0.8, color=color)

    ax2.set_xlabel("Time (seconds)", fontsize=12)
    ax2.set_ylabel("Memory Clock (MHz)", fontsize=12)
    ax2.set_title("GPU Memory Clock Over Time", fontsize=14, fontweight="bold")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()

    print(f"GPU Clocks saved to: {output_path}")


def create_legend_image(node_colors: Dict, output_path: Path):
    """Create a legend image mapping indices to node names."""
    items_per_column = 30
    num_nodes = len(node_colors)
    num_columns = (num_nodes + items_per_column - 1) // items_per_column

    fig_width = max(12, num_columns * 6)
    fig_height = max(8, min(items_per_column, num_nodes) * 0.3 + 1)

    fig, axes = plt.subplots(1, num_columns, figsize=(fig_width, fig_height))
    if num_columns == 1:
        axes = [axes]

    for ax in axes:
        ax.axis("off")

    sorted_nodes = sorted(node_colors.items(), key=lambda x: x[1][0])

    for col_idx in range(num_columns):
        start_idx = col_idx * items_per_column
        end_idx = min(start_idx + items_per_column, num_nodes)
        col_nodes = sorted_nodes[start_idx:end_idx]

        if not col_nodes:
            continue

        y_position = 0.95
        y_step = 0.9 / len(col_nodes)

        for node_name, (idx, color) in col_nodes:
            axes[col_idx].plot(
                [0.02, 0.08],
                [y_position, y_position],
                color=color,
                linewidth=3,
                solid_capstyle="round",
            )
            axes[col_idx].text(
                0.12,
                y_position,
                f"[{idx}] {node_name}",
                fontsize=8,
                verticalalignment="center",
                fontfamily="monospace",
            )
            y_position -= y_step

    plt.suptitle("Node Index Legend", fontsize=14, fontweight="bold", y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()

    print(f"Legend saved to: {output_path}")


def calculate_statistics(metrics: Dict[str, Dict], node_colors: Dict) -> str:
    """Calculate comprehensive statistics."""
    stats = []

    for idx, (node_name, node_data) in enumerate(sorted(metrics.items())):
        cpu_vals = node_data["cpu"]
        mem_vals = node_data["mem"]

        if not cpu_vals or not mem_vals:
            continue

        # Basic stats
        max_cpu = max(cpu_vals)
        avg_cpu = sum(cpu_vals) / len(cpu_vals)
        max_mem = max(mem_vals)
        avg_mem = sum(mem_vals) / len(mem_vals)

        # I/O stats
        io_read_vals = [r for r in node_data["io_read_rate"] if r is not None]
        io_write_vals = [w for w in node_data["io_write_rate"] if w is not None]
        avg_io_read = sum(io_read_vals) / len(io_read_vals) if io_read_vals else 0
        avg_io_write = sum(io_write_vals) / len(io_write_vals) if io_write_vals else 0

        # Network stats
        tcp_vals = node_data["tcp_conns"]
        udp_vals = node_data["udp_conns"]
        avg_tcp = sum(tcp_vals) / len(tcp_vals) if tcp_vals else 0
        avg_udp = sum(udp_vals) / len(udp_vals) if udp_vals else 0
        max_tcp = max(tcp_vals) if tcp_vals else 0
        max_udp = max(udp_vals) if udp_vals else 0

        # GPU stats
        gpu_mem_vals = [g for g in node_data["gpu_mem"] if g is not None]
        gpu_util_vals = [g for g in node_data["gpu_util"] if g is not None]
        gpu_mem_util_vals = [g for g in node_data["gpu_mem_util"] if g is not None]
        gpu_temp_vals = [g for g in node_data["gpu_temp"] if g is not None]
        gpu_power_vals = [g for g in node_data["gpu_power"] if g is not None]
        gpu_graphics_clock_vals = [g for g in node_data["gpu_graphics_clock"] if g is not None]
        gpu_memory_clock_vals = [g for g in node_data["gpu_memory_clock"] if g is not None]

        max_gpu_mem = max(gpu_mem_vals) if gpu_mem_vals else 0
        avg_gpu_mem = sum(gpu_mem_vals) / len(gpu_mem_vals) if gpu_mem_vals else 0
        max_gpu_util = max(gpu_util_vals) if gpu_util_vals else 0
        avg_gpu_util = sum(gpu_util_vals) / len(gpu_util_vals) if gpu_util_vals else 0
        max_gpu_mem_util = max(gpu_mem_util_vals) if gpu_mem_util_vals else 0
        avg_gpu_mem_util = sum(gpu_mem_util_vals) / len(gpu_mem_util_vals) if gpu_mem_util_vals else 0
        max_gpu_temp = max(gpu_temp_vals) if gpu_temp_vals else 0
        avg_gpu_temp = sum(gpu_temp_vals) / len(gpu_temp_vals) if gpu_temp_vals else 0
        max_gpu_power = max(gpu_power_vals) if gpu_power_vals else 0
        avg_gpu_power = sum(gpu_power_vals) / len(gpu_power_vals) if gpu_power_vals else 0
        avg_gpu_graphics_clock = sum(gpu_graphics_clock_vals) / len(gpu_graphics_clock_vals) if gpu_graphics_clock_vals else 0
        avg_gpu_memory_clock = sum(gpu_memory_clock_vals) / len(gpu_memory_clock_vals) if gpu_memory_clock_vals else 0

        stats.append(
            {
                "index": idx,
                "name": node_name,
                "max_cpu": max_cpu,
                "avg_cpu": avg_cpu,
                "max_mem": max_mem,
                "avg_mem": avg_mem,
                "avg_io_read": avg_io_read,
                "avg_io_write": avg_io_write,
                "avg_tcp": avg_tcp,
                "avg_udp": avg_udp,
                "max_tcp": max_tcp,
                "max_udp": max_udp,
                "max_gpu_mem": max_gpu_mem,
                "avg_gpu_mem": avg_gpu_mem,
                "max_gpu_util": max_gpu_util,
                "avg_gpu_util": avg_gpu_util,
                "max_gpu_mem_util": max_gpu_mem_util,
                "avg_gpu_mem_util": avg_gpu_mem_util,
                "max_gpu_temp": max_gpu_temp,
                "avg_gpu_temp": avg_gpu_temp,
                "max_gpu_power": max_gpu_power,
                "avg_gpu_power": avg_gpu_power,
                "avg_gpu_graphics_clock": avg_gpu_graphics_clock,
                "avg_gpu_memory_clock": avg_gpu_memory_clock,
            }
        )

    # Build report
    report = []
    report.append("=" * 80)
    report.append("RESOURCE USAGE STATISTICS")
    report.append("=" * 80)
    report.append("")

    # CPU stats
    report.append("Top 10 Nodes by Maximum CPU Usage")
    report.append("-" * 80)
    report.append(f"{'Rank':<6} {'Index':<8} {'Max CPU %':<12} {'Node Name'}")
    report.append("-" * 80)
    for rank, s in enumerate(
        sorted(stats, key=lambda x: x["max_cpu"], reverse=True)[:10], 1
    ):
        report.append(
            f"{rank:<6} [{s['index']}]{'':<6} {s['max_cpu']:>10.2f}%  {s['name']}"
        )
    report.append("")

    report.append("Top 10 Nodes by Average CPU Usage")
    report.append("-" * 80)
    report.append(f"{'Rank':<6} {'Index':<8} {'Avg CPU %':<12} {'Node Name'}")
    report.append("-" * 80)
    for rank, s in enumerate(
        sorted(stats, key=lambda x: x["avg_cpu"], reverse=True)[:10], 1
    ):
        report.append(
            f"{rank:<6} [{s['index']}]{'':<6} {s['avg_cpu']:>10.2f}%  {s['name']}"
        )
    report.append("")

    # Memory stats
    report.append("Top 10 Nodes by Maximum Memory Usage")
    report.append("-" * 80)
    report.append(f"{'Rank':<6} {'Index':<8} {'Max Mem (MB)':<14} {'Node Name'}")
    report.append("-" * 80)
    for rank, s in enumerate(
        sorted(stats, key=lambda x: x["max_mem"], reverse=True)[:10], 1
    ):
        report.append(
            f"{rank:<6} [{s['index']}]{'':<6} {s['max_mem']:>12.2f}  {s['name']}"
        )
    report.append("")

    report.append("Top 10 Nodes by Average Memory Usage")
    report.append("-" * 80)
    report.append(f"{'Rank':<6} {'Index':<8} {'Avg Mem (MB)':<14} {'Node Name'}")
    report.append("-" * 80)
    for rank, s in enumerate(
        sorted(stats, key=lambda x: x["avg_mem"], reverse=True)[:10], 1
    ):
        report.append(
            f"{rank:<6} [{s['index']}]{'':<6} {s['avg_mem']:>12.2f}  {s['name']}"
        )
    report.append("")

    # I/O stats (if available)
    if any(s["avg_io_read"] > 0 or s["avg_io_write"] > 0 for s in stats):
        report.append("Top 10 Nodes by Average I/O Read Rate")
        report.append("-" * 80)
        report.append(f"{'Rank':<6} {'Index':<8} {'Avg Read (MB/s)':<18} {'Node Name'}")
        report.append("-" * 80)
        for rank, s in enumerate(
            sorted(stats, key=lambda x: x["avg_io_read"], reverse=True)[:10], 1
        ):
            if s["avg_io_read"] > 0:
                report.append(
                    f"{rank:<6} [{s['index']}]{'':<6} {s['avg_io_read']:>16.4f}  {s['name']}"
                )
        report.append("")

        report.append("Top 10 Nodes by Average I/O Write Rate")
        report.append("-" * 80)
        report.append(
            f"{'Rank':<6} {'Index':<8} {'Avg Write (MB/s)':<18} {'Node Name'}"
        )
        report.append("-" * 80)
        for rank, s in enumerate(
            sorted(stats, key=lambda x: x["avg_io_write"], reverse=True)[:10], 1
        ):
            if s["avg_io_write"] > 0:
                report.append(
                    f"{rank:<6} [{s['index']}]{'':<6} {s['avg_io_write']:>16.4f}  {s['name']}"
                )
        report.append("")

    # Network stats
    if any(s["avg_tcp"] > 0 or s["avg_udp"] > 0 for s in stats):
        report.append("Top 10 Nodes by Average TCP Connections")
        report.append("-" * 80)
        report.append(
            f"{'Rank':<6} {'Index':<8} {'Avg TCP':<12} {'Max TCP':<12} {'Node Name'}"
        )
        report.append("-" * 80)
        for rank, s in enumerate(
            sorted(stats, key=lambda x: x["avg_tcp"], reverse=True)[:10], 1
        ):
            if s["avg_tcp"] > 0:
                report.append(
                    f"{rank:<6} [{s['index']}]{'':<6} {s['avg_tcp']:>10.1f}  {s['max_tcp']:>10}  {s['name']}"
                )
        report.append("")

        report.append("Top 10 Nodes by Average UDP Connections")
        report.append("-" * 80)
        report.append(
            f"{'Rank':<6} {'Index':<8} {'Avg UDP':<12} {'Max UDP':<12} {'Node Name'}"
        )
        report.append("-" * 80)
        for rank, s in enumerate(
            sorted(stats, key=lambda x: x["avg_udp"], reverse=True)[:10], 1
        ):
            if s["avg_udp"] > 0:
                report.append(
                    f"{rank:<6} [{s['index']}]{'':<6} {s['avg_udp']:>10.1f}  {s['max_udp']:>10}  {s['name']}"
                )
        report.append("")

    # GPU stats (if available)
    if any(s["max_gpu_mem"] > 0 for s in stats):
        report.append("Top 10 Nodes by Maximum GPU Memory Usage")
        report.append("-" * 80)
        report.append(f"{'Rank':<6} {'Index':<8} {'Max GPU (MB)':<14} {'Node Name'}")
        report.append("-" * 80)
        for rank, s in enumerate(
            sorted(stats, key=lambda x: x["max_gpu_mem"], reverse=True)[:10], 1
        ):
            if s["max_gpu_mem"] > 0:
                report.append(
                    f"{rank:<6} [{s['index']}]{'':<6} {s['max_gpu_mem']:>12.2f}  {s['name']}"
                )
        report.append("")

        report.append("Top 10 Nodes by Average GPU Utilization")
        report.append("-" * 80)
        report.append(f"{'Rank':<6} {'Index':<8} {'Avg GPU %':<12} {'Node Name'}")
        report.append("-" * 80)
        for rank, s in enumerate(
            sorted(stats, key=lambda x: x["avg_gpu_util"], reverse=True)[:10], 1
        ):
            if s["avg_gpu_util"] > 0:
                report.append(
                    f"{rank:<6} [{s['index']}]{'':<6} {s['avg_gpu_util']:>10.2f}%  {s['name']}"
                )
        report.append("")

        report.append("Top 10 Nodes by Maximum GPU Memory Utilization")
        report.append("-" * 80)
        report.append(f"{'Rank':<6} {'Index':<8} {'Max Mem Util %':<16} {'Node Name'}")
        report.append("-" * 80)
        for rank, s in enumerate(
            sorted(stats, key=lambda x: x["max_gpu_mem_util"], reverse=True)[:10], 1
        ):
            if s["max_gpu_mem_util"] > 0:
                report.append(
                    f"{rank:<6} [{s['index']}]{'':<6} {s['max_gpu_mem_util']:>14.2f}%  {s['name']}"
                )
        report.append("")

        report.append("Top 10 Nodes by Maximum GPU Temperature")
        report.append("-" * 80)
        report.append(f"{'Rank':<6} {'Index':<8} {'Max Temp (°C)':<16} {'Node Name'}")
        report.append("-" * 80)
        for rank, s in enumerate(
            sorted(stats, key=lambda x: x["max_gpu_temp"], reverse=True)[:10], 1
        ):
            if s["max_gpu_temp"] > 0:
                report.append(
                    f"{rank:<6} [{s['index']}]{'':<6} {s['max_gpu_temp']:>14.1f}  {s['name']}"
                )
        report.append("")

        report.append("Top 10 Nodes by Maximum GPU Power Consumption")
        report.append("-" * 80)
        report.append(f"{'Rank':<6} {'Index':<8} {'Max Power (W)':<16} {'Node Name'}")
        report.append("-" * 80)
        for rank, s in enumerate(
            sorted(stats, key=lambda x: x["max_gpu_power"], reverse=True)[:10], 1
        ):
            if s["max_gpu_power"] > 0:
                report.append(
                    f"{rank:<6} [{s['index']}]{'':<6} {s['max_gpu_power']:>14.2f}  {s['name']}"
                )
        report.append("")

        report.append("Top 10 Nodes by Average GPU Graphics Clock")
        report.append("-" * 80)
        report.append(f"{'Rank':<6} {'Index':<8} {'Avg Clock (MHz)':<18} {'Node Name'}")
        report.append("-" * 80)
        for rank, s in enumerate(
            sorted(stats, key=lambda x: x["avg_gpu_graphics_clock"], reverse=True)[:10], 1
        ):
            if s["avg_gpu_graphics_clock"] > 0:
                report.append(
                    f"{rank:<6} [{s['index']}]{'':<6} {s['avg_gpu_graphics_clock']:>16.1f}  {s['name']}"
                )
        report.append("")

        report.append("Top 10 Nodes by Average GPU Memory Clock")
        report.append("-" * 80)
        report.append(f"{'Rank':<6} {'Index':<8} {'Avg Clock (MHz)':<18} {'Node Name'}")
        report.append("-" * 80)
        for rank, s in enumerate(
            sorted(stats, key=lambda x: x["avg_gpu_memory_clock"], reverse=True)[:10], 1
        ):
            if s["avg_gpu_memory_clock"] > 0:
                report.append(
                    f"{rank:<6} [{s['index']}]{'':<6} {s['avg_gpu_memory_clock']:>16.1f}  {s['name']}"
                )
        report.append("")

    report.append("=" * 80)
    return "\n".join(report)


def generate_container_listing(log_dir: Path, node_colors: Dict) -> str:
    """Generate container listing (simplified - full implementation in original)."""
    # This is a placeholder - full implementation would parse load_node logs
    return "Container listing (see original implementation for details)\n"


def main():
    parser = argparse.ArgumentParser(
        description="Enhanced resource usage plotting from play_launch logs"
    )
    parser.add_argument("--log-dir", type=Path, help="Specific log directory")
    parser.add_argument("--base-log-dir", type=Path, default=Path("play_log"))
    parser.add_argument("--output-dir", type=Path, help="Output directory for plots")

    args = parser.parse_args()

    # Find log directory
    log_dir = args.log_dir if args.log_dir else find_latest_log_dir(args.base_log_dir)
    print(f"Using log directory: {log_dir}")

    # Load metrics from node/ and load_node/ subdirectories
    metrics = load_all_metrics(log_dir)
    if not metrics:
        print("Error: No metrics data found")
        sys.exit(1)

    # Setup output directory
    output_dir = (args.output_dir if args.output_dir else log_dir) / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Generate all plots
    print("\nGenerating plots...")

    # CPU and Memory (always available)
    node_colors = plot_timeline(
        metrics,
        "cpu",
        "CPU Usage (%)",
        "CPU Usage Over Time",
        output_dir / "cpu_usage.png",
    )
    plot_timeline(
        metrics,
        "mem",
        "Memory (MB)",
        "Memory Usage Over Time",
        output_dir / "memory_usage.png",
        node_colors,
    )
    plot_distribution(
        metrics,
        "cpu",
        "CPU Usage (%)",
        "CPU Usage Distribution",
        output_dir / "cpu_distribution.png",
        node_colors,
    )
    plot_distribution(
        metrics,
        "mem",
        "Memory (MB)",
        "Memory Usage Distribution",
        output_dir / "memory_distribution.png",
        node_colors,
    )

    # I/O plots (if data available)
    if has_io_data(metrics):
        plot_timeline(
            metrics,
            "io_read_rate",
            "Read Rate (MB/s)",
            "I/O Read Rate Over Time",
            output_dir / "io_read_usage.png",
            node_colors,
        )
        plot_timeline(
            metrics,
            "io_write_rate",
            "Write Rate (MB/s)",
            "I/O Write Rate Over Time",
            output_dir / "io_write_usage.png",
            node_colors,
        )
        plot_distribution(
            metrics,
            "io_read_rate",
            "Read Rate (MB/s)",
            "I/O Read Rate Distribution",
            output_dir / "io_distribution.png",
            node_colors,
        )
    else:
        print("No I/O rate data available - skipping I/O plots")

    # GPU plots (if data available)
    if has_gpu_data(metrics):
        plot_timeline(
            metrics,
            "gpu_mem",
            "GPU Memory (MB)",
            "GPU Memory Usage Over Time",
            output_dir / "gpu_memory_usage.png",
            node_colors,
        )
        plot_timeline(
            metrics,
            "gpu_util",
            "GPU Utilization (%)",
            "GPU Utilization Over Time",
            output_dir / "gpu_utilization.png",
            node_colors,
        )
        plot_timeline(
            metrics,
            "gpu_temp",
            "GPU Temperature (°C)",
            "GPU Temperature Over Time",
            output_dir / "gpu_temperature.png",
            node_colors,
        )
        plot_timeline(
            metrics,
            "gpu_power",
            "GPU Power (W)",
            "GPU Power Consumption Over Time",
            output_dir / "gpu_power.png",
            node_colors,
        )

        # GPU clocks plot - combine graphics and memory clocks
        plot_gpu_clocks(
            metrics,
            output_dir / "gpu_clocks.png",
            node_colors,
        )

        plot_distribution(
            metrics,
            "gpu_mem",
            "GPU Memory (MB)",
            "GPU Memory Distribution",
            output_dir / "gpu_distribution.png",
            node_colors,
        )
    else:
        print("No GPU data available - skipping GPU plots")

    # Legend
    create_legend_image(node_colors, output_dir / "legend.png")

    # Statistics
    print("Generating statistics...")
    stats_report = calculate_statistics(metrics, node_colors)
    with open(output_dir / "statistics.txt", "w") as f:
        f.write(stats_report)
    print(f"Statistics saved to: {output_dir / 'statistics.txt'}")

    # Container listing
    container_listing = generate_container_listing(log_dir, node_colors)
    with open(output_dir / "containers.txt", "w") as f:
        f.write(container_listing)
    print(f"Container listing saved to: {output_dir / 'containers.txt'}")

    print(f"\nAll plots saved to: {output_dir}")
    print("Done!")


if __name__ == "__main__":
    main()
