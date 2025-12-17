"""Test launch file for SetParameter action investigation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # Declare launch argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Set global parameters - should apply to all nodes after this
    set_use_sim_time = SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))
    set_global_rate = SetParameter(name='global_rate', value=50.0)
    set_global_string = SetParameter(name='global_string', value='hello_world')

    # Node 1: Should inherit global params
    talker1 = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='talker1',
        namespace='/test',
        parameters=[{'node_specific': 'talker1_value'}]
    )

    # Node 2: Should also inherit global params
    listener1 = Node(
        package='demo_nodes_cpp',
        executable='listener',
        name='listener1',
        namespace='/test',
        parameters=[{'node_specific': 'listener1_value'}]
    )

    # Test with a group scope
    group_action = GroupAction([
        SetParameter(name='group_param', value='inside_group'),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker2',
            namespace='/group_test',
            parameters=[{'node_specific': 'talker2_value'}]
        )
    ])

    # Node 3: Outside group - should NOT have group_param
    listener2 = Node(
        package='demo_nodes_cpp',
        executable='listener',
        name='listener2',
        namespace='/outside_group',
        parameters=[{'node_specific': 'listener2_value'}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        set_use_sim_time,
        set_global_rate,
        set_global_string,
        talker1,
        listener1,
        group_action,
        listener2,
    ])
