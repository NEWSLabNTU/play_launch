# An include does not scope configurations: this one is the includer's
# afterwards.
from launch import LaunchDescription
from launch.actions import SetLaunchConfiguration


def generate_launch_description():
    return LaunchDescription([SetLaunchConfiguration('child_hz', '20')])
