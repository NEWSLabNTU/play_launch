"""
play_launch - ROS2 Launch Inspection Tool

Records and replays ROS 2 launch file executions for performance analysis.
"""

__version__ = "0.4.0"

from play_launch.cli import main

__all__ = ["main", "__version__"]
