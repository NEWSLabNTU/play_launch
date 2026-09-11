# A root file with a required argument nobody set: launch's DeclareLaunchArgument
# raises, and so must the parser (issue 0030).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("required", description="nobody set me")])
