"""play_launch_analyzer - Analysis and visualization tools for play_launch execution logs."""

from .plot_resource_usage import main as plot_main

__all__ = ["plot_main", "main"]


def main():
    """Entry point for play_launch_plot command."""
    plot_main()
