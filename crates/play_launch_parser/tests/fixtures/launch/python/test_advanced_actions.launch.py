"""Test advanced actions: ExecuteLocal, Shutdown, RosTimer, SetUseSimTime."""

import launch
from launch.actions import ExecuteLocal, Shutdown, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, RosTimer, SetUseSimTime


def generate_launch_description():
    return launch.LaunchDescription([
        # Test 1: SetUseSimTime
        SetUseSimTime(True),

        # Test 2: Node after setting sim time
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='node_with_sim_time',
        ),

        # Test 3: ExecuteLocal with simple command
        ExecuteLocal(
            cmd=['ls', '-la'],
            output='log',
        ),

        # Test 4: RosTimer with period
        RosTimer(
            period=10.0,
            actions=[
                Node(
                    package='demo_nodes_cpp',
                    executable='listener',
                    name='delayed_node',
                ),
            ],
        ),

        # Test 5: Another node
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='regular_node',
        ),

        # Test 6: ExecuteLocal with shell (commented out as it might be system-dependent)
        # ExecuteLocal(
        #     cmd=['echo', 'hello'],
        #     shell=True,
        #     output='screen',
        # ),

        # Test 7: Shutdown with reason (this would terminate the launch)
        # Shutdown(reason='Test completed'),
    ])
